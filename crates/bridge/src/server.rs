//! The WebSocket server: LAN check, authentication, one controller, message handling.
//!
//! Each connection runs this sequence:
//!
//! 1. The peer address is checked against the LAN policy, and the number of open
//!    connections against [`MAX_CONNECTIONS`]; otherwise the socket is dropped.
//! 2. The WebSocket handshake runs, with the browser `Origin` checked against the
//!    allowlist when one is configured. Frames are capped at [`MAX_MESSAGE_BYTES`].
//! 3. The bridge sends `hello` with a fresh challenge. The first message must be a
//!    valid `auth` within the auth timeout; anything else closes the connection.
//! 4. The connection becomes the controller if none is connected; otherwise `busy`.
//!    Taking control never arms: the controller must send `arm`.
//! 5. Then `arm`, `disarm` and `target` update the [`SharedInbox`], and status goes out
//!    at the configured rate. A malformed message, a second `auth`, a binary frame or
//!    silence longer than the idle timeout releases control (force stops on the next
//!    servo tick) and closes the connection. A disconnect does the same.
//!
//! Release happens in a drop guard, so every exit path, including a panic in the
//! connection task, stops force.
use crate::auth::{self, AuthPolicy};
use crate::config::Config;
use crate::netpolicy;
use crate::protocol::{ClientMsg, ErrorCode, ServerMsg, Status, parse_client};
use crate::servo::{SharedInbox, Target};
use crate::view::View;
use futures_util::{SinkExt, StreamExt};
use std::io;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::watch;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::tungstenite::handshake::server::{ErrorResponse, Request, Response};
use tokio_tungstenite::tungstenite::http::StatusCode;
use tokio_tungstenite::tungstenite::protocol::CloseFrame;
use tokio_tungstenite::tungstenite::protocol::WebSocketConfig;
use tokio_tungstenite::tungstenite::protocol::frame::coding::CloseCode;

/// Largest accepted WebSocket message or frame, bytes.
pub const MAX_MESSAGE_BYTES: usize = 8 * 1024;
/// Most connections open at once (authenticated or not).
pub const MAX_CONNECTIONS: usize = 8;
/// Longest a single send may take before the connection is treated as dead.
pub const SEND_TIMEOUT: Duration = Duration::from_millis(250);

/// Shared state for every connection.
pub struct Bridge {
    config: Config,
    policy: AuthPolicy,
    inbox: SharedInbox,
    status: watch::Receiver<Status>,
    next_id: AtomicU64,
    open: AtomicUsize,
}

impl Bridge {
    /// Build from a validated configuration, the servo inbox and its status channel.
    pub fn new(
        config: Config,
        inbox: SharedInbox,
        status: watch::Receiver<Status>,
    ) -> Result<Arc<Self>, String> {
        config.validate()?;
        let policy = config.auth_policy()?;
        Ok(Arc::new(Self {
            config,
            policy,
            inbox,
            status,
            next_id: AtomicU64::new(1),
            open: AtomicUsize::new(0),
        }))
    }

    /// Accept connections on `listener` until `shutdown` turns true.
    pub async fn serve(
        self: Arc<Self>,
        listener: TcpListener,
        mut shutdown: watch::Receiver<bool>,
    ) -> io::Result<()> {
        loop {
            tokio::select! {
                accepted = listener.accept() => {
                    let (stream, peer) = match accepted {
                        Ok(a) => a,
                        Err(e) => {
                            eprintln!("visiontouch-bridge: accept failed: {e}");
                            tokio::time::sleep(Duration::from_millis(50)).await;
                            continue;
                        }
                    };
                    if !netpolicy::accept_peer(peer, self.config.allow_public) {
                        eprintln!("visiontouch-bridge: refused non-LAN peer {peer}");
                        continue;
                    }
                    if self.open.fetch_add(1, Ordering::SeqCst) >= MAX_CONNECTIONS {
                        self.open.fetch_sub(1, Ordering::SeqCst);
                        eprintln!("visiontouch-bridge: refused {peer}: too many connections");
                        continue;
                    }
                    let bridge = self.clone();
                    let shutdown = shutdown.clone();
                    tokio::spawn(async move {
                        let _slot = Slot(bridge.clone());
                        bridge.connection(stream, peer, shutdown).await;
                    });
                }
                _ = shutdown.changed() => {
                    if *shutdown.borrow() {
                        return Ok(());
                    }
                }
            }
        }
    }

    fn release_if(&self, id: u64) {
        let mut inbox = self.inbox.lock().unwrap_or_else(|e| e.into_inner());
        if inbox.controller == Some(id) {
            inbox.release();
        }
    }

    async fn connection(
        self: &Arc<Self>,
        stream: TcpStream,
        peer: SocketAddr,
        shutdown: watch::Receiver<bool>,
    ) {
        let _ = stream.set_nodelay(true);
        let id = self.next_id.fetch_add(1, Ordering::SeqCst);
        let _guard = Release {
            bridge: self.clone(),
            id,
        };
        if let Err(why) = self.session(stream, id, shutdown).await {
            eprintln!(
                "visiontouch-bridge: connection {id} from {peer} closed: {}",
                for_log(&why)
            );
        }
    }

    async fn session(
        self: &Arc<Self>,
        stream: TcpStream,
        id: u64,
        mut shutdown: watch::Receiver<bool>,
    ) -> Result<(), String> {
        let link = &self.config.link;
        let auth_timeout = Duration::from_millis(link.auth_timeout_ms);
        let origins = self.config.allowed_origins.clone();
        // The callback's error type is fixed by tungstenite's handshake API.
        #[allow(clippy::result_large_err)]
        let check_origin =
            move |req: &Request, resp: Response| -> Result<Response, ErrorResponse> {
                // No Origin header: not a browser. Otherwise the page must be listed,
                // so a web page the operator did not choose cannot drive the arm.
                let Some(origin) = req.headers().get("origin") else {
                    return Ok(resp);
                };
                match origin.to_str() {
                    Ok(o) if origins.iter().any(|a| a == o) => Ok(resp),
                    _ => {
                        let mut r = ErrorResponse::new(Some("origin not allowed".into()));
                        *r.status_mut() = StatusCode::FORBIDDEN;
                        Err(r)
                    }
                }
            };
        let ws_config = WebSocketConfig::default()
            .max_message_size(Some(MAX_MESSAGE_BYTES))
            .max_frame_size(Some(MAX_MESSAGE_BYTES));
        let ws = tokio::time::timeout(
            auth_timeout,
            tokio_tungstenite::accept_hdr_async_with_config(stream, check_origin, Some(ws_config)),
        )
        .await
        .map_err(|_| "handshake timed out".to_string())?
        .map_err(|e| format!("handshake: {e}"))?;
        let (mut tx, mut rx) = ws.split();

        macro_rules! send {
            ($msg:expr) => {
                match tokio::time::timeout(SEND_TIMEOUT, tx.send(Message::text($msg.to_text())))
                    .await
                {
                    Ok(Ok(())) => {}
                    Ok(Err(e)) => return Err(format!("send: {e}")),
                    Err(_) => return Err("send timed out".into()),
                }
            };
        }
        macro_rules! close_with {
            ($code:expr, $why:expr) => {{
                let why: String = $why;
                let _ = tokio::time::timeout(
                    SEND_TIMEOUT,
                    tx.send(Message::text(
                        ServerMsg::Error {
                            code: $code,
                            message: why.clone(),
                        }
                        .to_text(),
                    )),
                )
                .await;
                let frame = CloseFrame {
                    code: match $code {
                        ErrorCode::Malformed => CloseCode::Policy,
                        ErrorCode::Unauthorised | ErrorCode::AuthTimeout => CloseCode::Policy,
                        ErrorCode::Busy => CloseCode::Again,
                        ErrorCode::Idle => CloseCode::Away,
                        ErrorCode::Shutdown => CloseCode::Away,
                    },
                    reason: "".into(),
                };
                let _ =
                    tokio::time::timeout(SEND_TIMEOUT, tx.send(Message::Close(Some(frame)))).await;
                return Err(why);
            }};
        }

        // Hello and authentication.
        let challenge = auth::new_challenge().map_err(|e| format!("challenge: {e}"))?;
        send!(ServerMsg::Hello {
            schema: crate::protocol::SCHEMA_VERSION,
            challenge: challenge.clone(),
            limits: self.config.limits(),
        });
        let first = match tokio::time::timeout(auth_timeout, next_text(&mut rx)).await {
            Err(_) => close_with!(ErrorCode::AuthTimeout, "no auth within the deadline".into()),
            Ok(Err(why)) => close_with!(ErrorCode::Malformed, why),
            Ok(Ok(None)) => return Err("closed before auth".into()),
            Ok(Ok(Some(text))) => text,
        };
        let event = match parse_client(&first) {
            Ok(ClientMsg::Auth { event }) => event,
            Ok(other) => close_with!(
                ErrorCode::Unauthorised,
                format!("`{}` before auth", other.kind())
            ),
            Err(e) => close_with!(ErrorCode::Malformed, e.to_string()),
        };
        let now_unix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let pubkey = match auth::verify(&event, &challenge, &self.policy, now_unix) {
            Ok(k) => k,
            Err(e) => {
                eprintln!("visiontouch-bridge: connection {id}: auth refused: {e}");
                close_with!(ErrorCode::Unauthorised, "authentication refused".into())
            }
        };
        let claimed = {
            let mut inbox = self.inbox.lock().unwrap_or_else(|e| e.into_inner());
            if inbox.controller.is_some() {
                false
            } else {
                inbox.release();
                inbox.controller = Some(id);
                true
            }
        };
        if !claimed {
            close_with!(ErrorCode::Busy, "another controller is connected".into());
        }
        eprintln!(
            "visiontouch-bridge: connection {id}: controller {}",
            pubkey.to_hex()
        );
        send!(ServerMsg::AuthOk {
            pubkey: pubkey.to_hex()
        });

        // Control loop.
        let idle = Duration::from_millis(link.idle_timeout_ms);
        let mut deadline = tokio::time::Instant::now() + idle;
        let mut ticker =
            tokio::time::interval(Duration::from_secs_f64(1. / f64::from(link.status_hz)));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            tokio::select! {
                incoming = next_text(&mut rx) => {
                    let text = match incoming {
                        Ok(Some(t)) => t,
                        Ok(None) => return Err("closed by client".into()),
                        Err(why) => {
                            self.release_if(id);
                            close_with!(ErrorCode::Malformed, why)
                        }
                    };
                    deadline = tokio::time::Instant::now() + idle;
                    let msg = match parse_client(&text) {
                        Ok(m) => m,
                        Err(e) => {
                            self.release_if(id);
                            close_with!(ErrorCode::Malformed, e.to_string())
                        }
                    };
                    // Apply under the lock, decide outside it: no guard lives across an await.
                    let refused = {
                        let mut inbox = self.inbox.lock().unwrap_or_else(|e| e.into_inner());
                        if inbox.controller != Some(id) {
                            Some((ErrorCode::Busy, "control was lost"))
                        } else {
                            match msg {
                                ClientMsg::Arm => {
                                    inbox.arm_wanted = true;
                                    inbox.arm_epoch = inbox.arm_epoch.wrapping_add(1);
                                    None
                                }
                                ClientMsg::Disarm => {
                                    inbox.arm_wanted = false;
                                    None
                                }
                                ClientMsg::Target { seq, p, view } => {
                                    // Scene targets are mapped into the model frame here,
                                    // with the view sent in the same message.
                                    let device = |p| view.map_or(p, |v: View| v.to_device(p));
                                    inbox.target = p.map(|p| Target {
                                        p: device(p),
                                        at: Instant::now(),
                                    });
                                    inbox.target_seq = Some(seq);
                                    inbox.view = view;
                                    None
                                }
                                ClientMsg::Auth { .. } => {
                                    inbox.release();
                                    Some((ErrorCode::Malformed, "`auth` after authentication"))
                                }
                            }
                        }
                    };
                    if let Some((code, why)) = refused {
                        close_with!(code, why.to_string());
                    }
                }
                _ = ticker.tick() => {
                    let status = *self.status.borrow();
                    send!(ServerMsg::Status(status));
                }
                _ = tokio::time::sleep_until(deadline) => {
                    self.release_if(id);
                    close_with!(ErrorCode::Idle, "no message within the idle timeout".into());
                }
                _ = shutdown.changed() => {
                    if *shutdown.borrow() {
                        self.release_if(id);
                        close_with!(ErrorCode::Shutdown, "bridge shutting down".into());
                    }
                }
            }
        }
    }
}

/// Frees a connection slot when its task ends, however it ends.
struct Slot(Arc<Bridge>);
impl Drop for Slot {
    fn drop(&mut self) {
        self.0.open.fetch_sub(1, Ordering::SeqCst);
    }
}

/// Client-supplied text made safe for one log line: escaped and truncated.
fn for_log(s: &str) -> String {
    let mut out: String = s.chars().take(200).flat_map(char::escape_debug).collect();
    if s.chars().count() > 200 {
        out.push('…');
    }
    out
}

/// Releases control when a connection ends, however it ends.
struct Release {
    bridge: Arc<Bridge>,
    id: u64,
}
impl Drop for Release {
    fn drop(&mut self) {
        self.bridge.release_if(self.id);
    }
}

/// The next text message. `Ok(None)` on a close or end of stream; `Err` on a binary
/// frame or a protocol error. Ping and pong frames are skipped (tungstenite answers
/// pings itself).
async fn next_text<S>(rx: &mut S) -> Result<Option<String>, String>
where
    S: futures_util::Stream<Item = Result<Message, tokio_tungstenite::tungstenite::Error>> + Unpin,
{
    loop {
        match rx.next().await {
            None => return Ok(None),
            Some(Err(e)) => return Err(format!("websocket: {e}")),
            Some(Ok(Message::Text(t))) => return Ok(Some(t.as_str().to_owned())),
            Some(Ok(Message::Close(_))) => return Ok(None),
            Some(Ok(Message::Ping(_) | Message::Pong(_))) => continue,
            Some(Ok(Message::Binary(_) | Message::Frame(_))) => {
                return Err("binary frames are not part of the schema".into());
            }
        }
    }
}
