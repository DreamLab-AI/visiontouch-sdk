//! End to end: a real WebSocket on 127.0.0.1 against the simulated unit.
//!
//! Nothing here opens hardware: the servo thread drives `FakeUnit`.
use futures_util::{SinkExt, StreamExt};
use nostr::prelude::*;
use serde_json::{Value, json};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::{MaybeTlsStream, WebSocketStream, connect_async};
use visionclaw_bridge::config::Config;
use visionclaw_bridge::fake::FakeUnit;
use visionclaw_bridge::server::Bridge;
use visionclaw_bridge::servo::{self, Inbox, Servo};

type Ws = WebSocketStream<MaybeTlsStream<TcpStream>>;

const TARGET_TIMEOUT_MS: u64 = 100;

struct Rig {
    url: String,
    keys: Keys,
    fake: FakeUnit,
    stop: Arc<AtomicBool>,
    shutdown: tokio::sync::watch::Sender<bool>,
}

impl Drop for Rig {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.shutdown.send(true);
    }
}

async fn rig() -> Rig {
    let keys = Keys::generate();
    let config = Config::from_toml(&format!(
        "url = \"ws://bridge\"\nauthorised_keys = [\"{}\"]\n[link]\ntarget_timeout_ms = {TARGET_TIMEOUT_MS}\n\
         idle_timeout_ms = 400\nauth_timeout_ms = 1000\nstatus_hz = 100\n",
        keys.public_key().to_hex()
    ))
    .unwrap();
    let fake = FakeUnit::registered();
    let inbox = Arc::new(Mutex::new(Inbox::default()));
    let mut initial = servo::fault_status(0);
    initial.device_fault = false;
    let (tx, rx) = tokio::sync::watch::channel(initial);
    let stop = Arc::new(AtomicBool::new(false));
    servo::spawn(
        Servo::new(fake.clone(), &config, Instant::now()).unwrap(),
        inbox.clone(),
        tx,
        stop.clone(),
        Duration::from_millis(1),
    )
    .unwrap();
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("ws://{}", listener.local_addr().unwrap());
    let (shutdown, shutdown_rx) = tokio::sync::watch::channel(false);
    let bridge = Bridge::new(config, inbox, rx).unwrap();
    tokio::spawn(bridge.serve(listener, shutdown_rx));
    Rig {
        url,
        keys,
        fake,
        stop,
        shutdown,
    }
}

async fn recv(ws: &mut Ws) -> Option<Value> {
    loop {
        match tokio::time::timeout(Duration::from_secs(2), ws.next()).await {
            Ok(Some(Ok(Message::Text(t)))) => return Some(serde_json::from_str(&t).unwrap()),
            Ok(Some(Ok(Message::Close(_)))) | Ok(None) | Ok(Some(Err(_))) => return None,
            Ok(Some(Ok(_))) => continue,
            Err(_) => panic!("no message within 2 s"),
        }
    }
}

/// Receive until a non-status message, or the close (`None`).
async fn recv_event(ws: &mut Ws) -> Option<Value> {
    loop {
        let m = recv(ws).await?;
        if m["type"] != "status" {
            return Some(m);
        }
    }
}

async fn send(ws: &mut Ws, v: Value) {
    ws.send(Message::text(v.to_string())).await.unwrap();
}

fn auth_event(keys: &Keys, challenge: &str) -> Value {
    let e = EventBuilder::new(Kind::Authentication, "")
        .tags([
            Tag::custom("challenge", [challenge]),
            Tag::custom("relay", ["ws://bridge"]),
        ])
        .finalize(keys)
        .unwrap();
    serde_json::from_str(&e.as_json()).unwrap()
}

async fn connect(r: &Rig) -> (Ws, String) {
    let (mut ws, _) = connect_async(&r.url).await.unwrap();
    let hello = recv(&mut ws).await.unwrap();
    assert_eq!(hello["v"], 1);
    assert_eq!(hello["type"], "hello");
    assert_eq!(hello["limits"]["target_timeout_ms"], TARGET_TIMEOUT_MS);
    let challenge = hello["challenge"].as_str().unwrap().to_owned();
    (ws, challenge)
}

async fn authed(r: &Rig, keys: &Keys) -> Ws {
    let (mut ws, challenge) = connect(r).await;
    send(
        &mut ws,
        json!({"v":1,"type":"auth","event":auth_event(keys, &challenge)}),
    )
    .await;
    let ok = recv_event(&mut ws).await.unwrap();
    assert_eq!(ok["type"], "auth_ok", "{ok}");
    ws
}

fn target(seq: u64, p: [f64; 3]) -> Value {
    json!({"v":1,"type":"target","seq":seq,"p":p})
}

/// Authenticate, arm, release-then-press, and stream a target 10 mm from the pen
/// until force is flowing. Returns the socket and the next sequence number.
async fn engaged(r: &Rig) -> (Ws, u64) {
    let mut ws = authed(r, &r.keys).await;
    send(&mut ws, json!({"v":1,"type":"arm"})).await;
    r.fake.set_button(false);
    let mut seq = 0;
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        assert!(Instant::now() < deadline, "never engaged");
        seq += 1;
        send(&mut ws, target(seq, [0.01, 0., 0.])).await;
        let s = recv(&mut ws).await.unwrap();
        if s["type"] == "status" && s["armed"] == true && s["deadman_ready"] == true {
            r.fake.set_button(true);
        }
        if s["type"] == "status" && s["reason"] == "engaged" && r.fake.snapshot().max_force_n > 0.5
        {
            return (ws, seq + 1);
        }
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
}

fn force(r: &Rig) -> f64 {
    let f = r.fake.snapshot().force;
    f.iter().map(|x| x * x).sum::<f64>().sqrt()
}

async fn wait_until(what: &str, within: Duration, mut ok: impl FnMut() -> bool) -> Duration {
    let start = Instant::now();
    while !ok() {
        assert!(start.elapsed() < within, "{what} not within {within:?}");
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    start.elapsed()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_unauthenticated_arm_is_refused_and_arms_nothing() {
    let r = rig().await;
    let (mut ws, _) = connect(&r).await;
    r.fake.set_button(true);
    send(&mut ws, json!({"v":1,"type":"arm"})).await;
    let e = recv_event(&mut ws).await.unwrap();
    assert_eq!(
        (e["type"].as_str(), e["code"].as_str()),
        (Some("error"), Some("unauthorised"))
    );
    assert!(recv_event(&mut ws).await.is_none(), "closed");
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert_eq!(r.fake.snapshot().arm_calls, 0);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_key_outside_the_allowlist_is_refused() {
    let r = rig().await;
    let (mut ws, challenge) = connect(&r).await;
    let stranger = Keys::generate();
    send(
        &mut ws,
        json!({"v":1,"type":"auth","event":auth_event(&stranger, &challenge)}),
    )
    .await;
    let e = recv_event(&mut ws).await.unwrap();
    assert_eq!(e["code"], "unauthorised");
    assert!(recv_event(&mut ws).await.is_none());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_replayed_signature_is_refused_on_a_new_connection() {
    let r = rig().await;
    let (mut first, challenge) = connect(&r).await;
    let event = auth_event(&r.keys, &challenge);
    send(
        &mut first,
        json!({"v":1,"type":"auth","event":event.clone()}),
    )
    .await;
    assert_eq!(recv_event(&mut first).await.unwrap()["type"], "auth_ok");
    drop(first);
    tokio::time::sleep(Duration::from_millis(50)).await;
    let (mut second, _) = connect(&r).await;
    send(&mut second, json!({"v":1,"type":"auth","event":event})).await;
    assert_eq!(
        recv_event(&mut second).await.unwrap()["code"],
        "unauthorised"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_happy_path_delivers_force_towards_the_target() {
    let r = rig().await;
    let (mut ws, seq) = engaged(&r).await;
    send(&mut ws, target(seq, [0.01, 0., 0.])).await;
    let s = recv(&mut ws).await.unwrap();
    assert_eq!(s["reason"], "engaged");
    assert!(
        s["force"][0].as_f64().unwrap() > 0.,
        "pulls +x towards the target: {s}"
    );
    assert!(force(&r) > 0.5);
    assert!(force(&r) <= visiontouch_device::unit::CONTINUOUS_N + 1e-9);
    assert_eq!(r.fake.snapshot().arm_calls, 1);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_socket_drop_zeroes_force_promptly() {
    let r = rig().await;
    let (ws, _) = engaged(&r).await;
    assert!(force(&r) > 0.5);
    drop(ws);
    let took = wait_until(
        "force off and disarmed after a drop",
        Duration::from_millis(500),
        || {
            let s = r.fake.snapshot();
            s.force == [0.; 3] && !s.armed
        },
    )
    .await;
    assert!(
        took < Duration::from_millis(TARGET_TIMEOUT_MS),
        "the drop itself cuts force, not the target timeout: {took:?}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_stalled_target_zeroes_force_at_the_target_timeout() {
    let r = rig().await;
    let (mut ws, _) = engaged(&r).await;
    // Stop sending; the connection stays open (idle timeout is longer).
    let took = wait_until(
        "force off after the target stalls",
        Duration::from_millis(350),
        || r.fake.snapshot().force == [0.; 3],
    )
    .await;
    assert!(
        took <= Duration::from_millis(TARGET_TIMEOUT_MS + 50),
        "{took:?}"
    );
    let mut saw_stale = false;
    for _ in 0..20 {
        match recv(&mut ws).await {
            Some(s) if s["reason"] == "target_stale" => {
                saw_stale = true;
                break;
            }
            Some(_) => {}
            None => break,
        }
    }
    assert!(saw_stale, "status names the reason");
    assert!(
        r.fake.snapshot().armed,
        "a stall stops force but keeps the arm"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_malformed_message_zeroes_force_and_closes() {
    let r = rig().await;
    let (mut ws, _) = engaged(&r).await;
    send(
        &mut ws,
        json!({"v":1,"type":"target","seq":1,"p":[0, "x", 0]}),
    )
    .await;
    wait_until(
        "force off after malformed input",
        Duration::from_millis(50),
        || {
            let s = r.fake.snapshot();
            s.force == [0.; 3] && !s.armed
        },
    )
    .await;
    let e = recv_event(&mut ws).await.unwrap();
    assert_eq!(e["code"], "malformed");
    assert!(recv_event(&mut ws).await.is_none(), "closed");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_binary_frame_is_malformed() {
    let r = rig().await;
    let (mut ws, _) = engaged(&r).await;
    ws.send(Message::binary(vec![1u8, 2, 3])).await.unwrap();
    let e = recv_event(&mut ws).await.unwrap();
    assert_eq!(e["code"], "malformed");
    wait_until("force off", Duration::from_millis(50), || force(&r) == 0.).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_second_controller_is_busy_and_cannot_disturb_the_first() {
    let r = rig().await;
    let (mut first, seq) = engaged(&r).await;
    let (mut second, challenge) = connect(&r).await;
    send(
        &mut second,
        json!({"v":1,"type":"auth","event":auth_event(&r.keys, &challenge)}),
    )
    .await;
    assert_eq!(recv_event(&mut second).await.unwrap()["code"], "busy");
    send(&mut first, target(seq, [0.01, 0., 0.])).await;
    let s = recv(&mut first).await.unwrap();
    assert_eq!(s["reason"], "engaged", "the first controller is unaffected");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn silence_past_the_idle_timeout_closes_and_disarms() {
    let r = rig().await;
    let (mut ws, _) = engaged(&r).await;
    let mut code = None;
    while let Some(m) = recv(&mut ws).await {
        if m["type"] == "error" {
            code = m["code"].as_str().map(str::to_owned);
        }
    }
    assert_eq!(code.as_deref(), Some("idle"));
    wait_until("disarmed", Duration::from_millis(50), || {
        !r.fake.snapshot().armed
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn disarm_and_a_null_target_each_stop_force() {
    let r = rig().await;
    let (mut ws, seq) = engaged(&r).await;
    send(&mut ws, json!({"v":1,"type":"target","seq":seq,"p":null})).await;
    wait_until(
        "force off on a null target",
        Duration::from_millis(50),
        || force(&r) == 0.,
    )
    .await;
    send(&mut ws, target(seq + 1, [0.01, 0., 0.])).await;
    wait_until("force back", Duration::from_millis(500), || force(&r) > 0.).await;
    send(&mut ws, json!({"v":1,"type":"disarm"})).await;
    wait_until("disarmed", Duration::from_millis(50), || {
        let s = r.fake.snapshot();
        !s.armed && s.force == [0.; 3]
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_clean_close_zeroes_force_promptly() {
    let r = rig().await;
    let (mut ws, _) = engaged(&r).await;
    ws.close(None).await.unwrap();
    let took = wait_until(
        "force off and disarmed after a close",
        Duration::from_millis(500),
        || {
            let s = r.fake.snapshot();
            s.force == [0.; 3] && !s.armed
        },
    )
    .await;
    assert!(took < Duration::from_millis(TARGET_TIMEOUT_MS), "{took:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_browser_origin_not_on_the_list_is_refused() {
    use tokio_tungstenite::tungstenite::client::IntoClientRequest;
    let r = rig().await;
    let mut req = r.url.as_str().into_client_request().unwrap();
    req.headers_mut()
        .insert("origin", "http://evil.example".parse().unwrap());
    assert!(connect_async(req).await.is_err(), "handshake refused");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_signature_for_another_url_is_refused() {
    let r = rig().await;
    let (mut ws, challenge) = connect(&r).await;
    let e = EventBuilder::new(Kind::Authentication, "")
        .tags([
            Tag::custom("challenge", [challenge.as_str()]),
            Tag::custom("relay", ["ws://some-other-service"]),
        ])
        .finalize(&r.keys)
        .unwrap();
    let event: Value = serde_json::from_str(&e.as_json()).unwrap();
    send(&mut ws, json!({"v":1,"type":"auth","event":event})).await;
    assert_eq!(recv_event(&mut ws).await.unwrap()["code"], "unauthorised");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_scene_target_is_mapped_through_the_view() {
    let r = rig().await;
    let (mut ws, mut seq) = engaged(&r).await;
    // A quarter turn about +Y: scene -Z is pen +X. Scale 1000 units per metre.
    let h = std::f64::consts::FRAC_PI_4;
    let view = json!({"origin":[300.,0.,0.],"rotation":[0.,h.sin(),0.,h.cos()],"scale":1000.});
    let first = seq + 1;
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        assert!(
            Instant::now() < deadline,
            "never settled on the mapped target"
        );
        seq += 1;
        send(
            &mut ws,
            json!({"v":1,"type":"target","seq":seq,"p":[300.,0.,-10.],"view":view}),
        )
        .await;
        let s = recv(&mut ws).await.unwrap();
        if s["type"] == "status" && s["target_seq"].as_u64().is_some_and(|q| q >= first) {
            let anchor = s["anchor"][0].as_f64().unwrap_or(0.);
            if (anchor - 0.01).abs() < 1e-4 {
                assert!(s["force"][0].as_f64().unwrap() > 0., "{s}");
                // The pen (at the model origin) reported in the scene.
                assert_eq!(s["pen_scene"], json!([300., 0., 0.]));
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
}
