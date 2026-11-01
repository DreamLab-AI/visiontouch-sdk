//! The reconnecting monitor: one thread that connects to spacenavd, reads events and
//! publishes the latest state.
//!
//! Consumers hold a [`Latest`] and read it whenever they like; there is no queue, so a
//! slow consumer sees only the newest state, which is what a motion stream wants.
//!
//! The thread loops: connect, handshake, read until the stream ends, publish a zero
//! snapshot, wait one poll interval, repeat. Reads time out every
//! [`READ_TIMEOUT`], so the thread notices a stop request promptly and the monitor's
//! drop joins it. Even a panic on the thread leaves a zero snapshot behind (a drop
//! guard).
use crate::wire::Event;
use crate::wire::{
    FRAME_LEN, Frame, HANDSHAKE, REQ_CHANGE_PROTO, REQ_DEV_NAXES, REQ_TAG, classify, naxes_request,
};
use std::io::{self, Read, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// How long one read waits before the thread re-checks its stop flag.
pub const READ_TIMEOUT: Duration = Duration::from_millis(20);

/// How long the client waits for the handshake reply before it decides the daemon is
/// protocol 0 (libspnav waits 300 ms too).
pub const HANDSHAKE_TIMEOUT: Duration = Duration::from_millis(300);

/// Where the link to the daemon and its device stand. Every state but
/// [`Ready`](Health::Ready) and [`ReadyV0`](Health::ReadyV0) carries zero input.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Health {
    /// No daemon answered on the socket: it is not running, the socket file is stale
    /// or unreadable, or the monitor has not connected yet.
    #[default]
    DaemonAbsent,
    /// The daemon was reachable and the connection ended.
    DaemonLost,
    /// The daemon is running and reports no device (never plugged in, or unplugged).
    NoDevice,
    /// The daemon is running (protocol 1) and has a device.
    Ready,
    /// The daemon is protocol 0, which reports no device changes: input is live but
    /// whether a device exists is unknown.
    ReadyV0,
}

impl Health {
    /// Input is expected from this link: a device is attached, or (protocol 0) the
    /// daemon is up and cannot say otherwise.
    pub fn present(self) -> bool {
        matches!(self, Self::Ready | Self::ReadyV0)
    }

    /// A short machine-readable reason, `None` when [`present`](Self::present).
    pub fn reason(self) -> Option<&'static str> {
        match self {
            Self::DaemonAbsent => Some("spacenavd_absent"),
            Self::DaemonLost => Some("spacenavd_lost"),
            Self::NoDevice => Some("no_device"),
            Self::Ready | Self::ReadyV0 => None,
        }
    }

    fn describe(self) -> &'static str {
        match self {
            Self::DaemonAbsent => "spacenavd not reachable; waiting for it",
            Self::DaemonLost => "spacenavd connection lost; reconnecting",
            Self::NoDevice => "spacenavd running, no device attached",
            Self::Ready => "device attached",
            Self::ReadyV0 => "spacenavd running (protocol 0, device presence unknown)",
        }
    }
}

/// What the monitor last saw.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Snapshot {
    /// Link and device state.
    pub health: Health,
    /// Motion `x, y, z, rx, ry, rz`, in the daemon's units. All zero unless
    /// [`Health::present`].
    pub axes: [i32; 6],
    /// Button bitmask, bit 0 = button 0. Zero unless [`Health::present`].
    pub buttons: u32,
    /// Incremented on every published change, so a consumer can notice updates cheaply.
    pub version: u64,
}

/// A cheap handle to the newest [`Snapshot`].
#[derive(Clone, Debug, Default)]
pub struct Latest(Arc<Mutex<Snapshot>>);

impl Latest {
    /// The newest snapshot. A poisoned lock reads as the default: absent and still.
    pub fn get(&self) -> Snapshot {
        match self.0.lock() {
            Ok(g) => *g,
            Err(_) => Snapshot::default(),
        }
    }

    fn publish(&self, health: Health, axes: [i32; 6], buttons: u32) {
        let (axes, buttons) = if health.present() {
            (axes, buttons)
        } else {
            ([0; 6], 0)
        };
        let mut g = self.0.lock().unwrap_or_else(|e| e.into_inner());
        if g.health != health || g.axes != axes || g.buttons != buttons {
            g.health = health;
            g.axes = axes;
            g.buttons = buttons;
            g.version += 1;
        }
    }
}

/// Resets the snapshot when the thread ends, however it ends.
struct Reset(Latest);
impl Drop for Reset {
    fn drop(&mut self) {
        self.0.publish(Health::DaemonAbsent, [0; 6], 0);
    }
}

/// The running monitor. Dropping it stops the thread.
pub struct Monitor {
    latest: Latest,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Monitor {
    /// Start the monitor thread on the socket at `socket`. `poll` is the wait between
    /// connection attempts while the daemon is absent or after it is lost. The only
    /// error is failing to start the thread.
    pub fn spawn(socket: impl Into<PathBuf>, poll: Duration) -> io::Result<Self> {
        let latest = Latest::default();
        let stop = Arc::new(AtomicBool::new(false));
        let socket = socket.into();
        let thread = {
            let (latest, stop) = (latest.clone(), stop.clone());
            thread::Builder::new()
                .name("spnav".into())
                .spawn(move || run(&socket, poll, &latest, &stop))?
        };
        Ok(Self {
            latest,
            stop,
            thread: Some(thread),
        })
    }

    /// A handle to the newest snapshot.
    pub fn latest(&self) -> Latest {
        self.latest.clone()
    }
}

impl Drop for Monitor {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn sleep_unless_stopped(total: Duration, stop: &AtomicBool) {
    let step = Duration::from_millis(10);
    let mut left = total;
    while !left.is_zero() && !stop.load(Ordering::Relaxed) {
        let s = left.min(step);
        thread::sleep(s);
        left -= s;
    }
}

/// Publishes, and logs each distinct health once.
struct Out<'a> {
    latest: &'a Latest,
    logged: Option<Health>,
}

impl Out<'_> {
    fn put(&mut self, health: Health, axes: [i32; 6], buttons: u32) {
        self.latest.publish(health, axes, buttons);
        if self.logged != Some(health) {
            eprintln!("spnav-client: {}", health.describe());
            self.logged = Some(health);
        }
    }
}

fn run(socket: &Path, poll: Duration, latest: &Latest, stop: &AtomicBool) {
    let _reset = Reset(latest.clone());
    let mut out = Out {
        latest,
        logged: None,
    };
    while !stop.load(Ordering::Relaxed) {
        match UnixStream::connect(socket) {
            Ok(stream) => {
                let end = session(stream, &mut out, stop);
                if !stop.load(Ordering::Relaxed) {
                    out.put(end, [0; 6], 0);
                }
            }
            Err(_) => out.put(Health::DaemonAbsent, [0; 6], 0),
        }
        sleep_unless_stopped(poll, stop);
    }
}

/// Read exactly `buf.len()` bytes within `timeout`; `Ok(false)` on timeout.
fn read_exact_timeout(s: &mut UnixStream, buf: &mut [u8], timeout: Duration) -> io::Result<bool> {
    s.set_read_timeout(Some(timeout))?;
    let mut got = 0;
    while got < buf.len() {
        match s.read(&mut buf[got..]) {
            Ok(0) => return Err(io::ErrorKind::UnexpectedEof.into()),
            Ok(n) => got += n,
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                return Ok(false);
            }
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
    Ok(true)
}

/// Run one connection to its end and say how it ended.
fn session(mut s: UnixStream, out: &mut Out<'_>, stop: &AtomicBool) -> Health {
    let Ok(v1) = handshake(&mut s) else {
        return Health::DaemonLost;
    };
    // Protocol 0 cannot say whether a device exists; protocol 1 can be asked.
    let mut known = !v1;
    let mut present = !v1;
    let mut axes = [0i32; 6];
    let mut buttons = 0u32;
    let health = |present: bool, v1: bool| match (present, v1) {
        (true, true) => Health::Ready,
        (true, false) => Health::ReadyV0,
        (false, _) => Health::NoDevice,
    };
    if v1 {
        // Until the reply (or an event) says otherwise, the daemon is up with no
        // device: the safe state, with zero input.
        out.put(Health::NoDevice, axes, buttons);
        if s.write_all(&naxes_request()).is_err() {
            return Health::DaemonLost;
        }
    } else {
        out.put(Health::ReadyV0, axes, buttons);
    }
    if s.set_read_timeout(Some(READ_TIMEOUT)).is_err() {
        return Health::DaemonLost;
    }
    let mut pending: Vec<u8> = Vec::with_capacity(4 * FRAME_LEN);
    let mut chunk = [0u8; 8 * FRAME_LEN];
    while !stop.load(Ordering::Relaxed) {
        match s.read(&mut chunk) {
            Ok(0) => return Health::DaemonLost,
            Ok(n) => pending.extend_from_slice(&chunk[..n]),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                continue;
            }
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(_) => return Health::DaemonLost,
        }
        // A stream has no message boundaries: take whole frames only.
        let whole = pending.len() / FRAME_LEN * FRAME_LEN;
        for raw in pending[..whole].chunks_exact(FRAME_LEN) {
            let Ok(raw) = <&[u8; FRAME_LEN]>::try_from(raw) else {
                continue;
            };
            match classify(raw) {
                Frame::Event(Event::Motion { axes: a }) => {
                    // Motion proves a device exists.
                    axes = a;
                    present = true;
                    known = true;
                }
                Frame::Event(Event::Button { number, press }) => {
                    present = true;
                    known = true;
                    if let Ok(n @ 0..32) = u32::try_from(number) {
                        if press {
                            buttons |= 1 << n;
                        } else {
                            buttons &= !(1 << n);
                        }
                    }
                }
                Frame::Event(Event::Device { added }) => {
                    present = added;
                    known = true;
                    axes = [0; 6];
                    buttons = 0;
                }
                Frame::Reply { kind, status } if kind == REQ_TAG | REQ_DEV_NAXES => {
                    if !known {
                        present = status >= 0;
                        known = true;
                    }
                }
                Frame::Reply { .. } | Frame::Ignored => {}
            }
        }
        pending.drain(..whole);
        out.put(health(present, v1), axes, buttons);
    }
    Health::DaemonAbsent
}

/// Send the protocol-1 request. `Ok(true)` if the daemon answered (protocol 1),
/// `Ok(false)` if it stayed silent (protocol 0, after restoring its sensitivity).
fn handshake(s: &mut UnixStream) -> io::Result<bool> {
    s.write_all(&HANDSHAKE.to_ne_bytes())?;
    let mut reply = [0u8; 4];
    if read_exact_timeout(s, &mut reply, HANDSHAKE_TIMEOUT)? {
        let r = i32::from_ne_bytes(reply);
        let accepted = r & 0xff;
        Ok(r & !0xff == REQ_TAG | REQ_CHANGE_PROTO && accepted >= 1)
    } else {
        // An old daemon took the handshake as a sensitivity float; put it back to 1.
        s.write_all(&1.0f32.to_ne_bytes())?;
        Ok(false)
    }
}
