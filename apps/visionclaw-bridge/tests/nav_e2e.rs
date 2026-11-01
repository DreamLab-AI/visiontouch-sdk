//! SpaceMouse navigation end to end: a real WebSocket on 127.0.0.1, the simulated arm,
//! and a fake spacenavd on a Unix socket. Nothing here opens hardware.
use futures_util::{SinkExt, StreamExt};
use nostr::prelude::*;
use serde_json::{Value, json};
use spnav_client::testing::FakeDaemon;
use spnav_client::{Health, Monitor};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::{MaybeTlsStream, WebSocketStream, connect_async};
use visionclaw_bridge::config::Config;
use visionclaw_bridge::fake::FakeUnit;
use visionclaw_bridge::nav::NavSource;
use visionclaw_bridge::server::Bridge;
use visionclaw_bridge::servo::{self, Inbox, Servo};

type Ws = WebSocketStream<MaybeTlsStream<TcpStream>>;

/// Whether the bridge is configured to offer navigation.
#[derive(Clone, Copy, PartialEq)]
enum Nav {
    Disabled,
    Enabled { plugged: bool },
}

struct Rig {
    url: String,
    keys: Keys,
    fake: FakeUnit,
    pad: FakeDaemon,
    stop: Arc<AtomicBool>,
    shutdown: tokio::sync::watch::Sender<bool>,
    _monitor: Option<Monitor>,
}

impl Drop for Rig {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.shutdown.send(true);
    }
}

async fn rig(nav: Nav) -> Rig {
    let keys = Keys::generate();
    let pad = FakeDaemon::at(spnav_client::testing::unique_socket_path()).unwrap();
    let spacemouse = if matches!(nav, Nav::Enabled { .. }) {
        format!(
            "[spacemouse]\nenabled = true\nsocket = \"{}\"\nrate_hz = 60\npoll_ms = 200\n",
            pad.path().display()
        )
    } else {
        String::new()
    };
    let config = Config::from_toml(&format!(
        "url = \"ws://bridge\"\nauthorised_keys = [\"{}\"]\n[link]\ntarget_timeout_ms = 100\n\
         idle_timeout_ms = 400\nauth_timeout_ms = 1000\nstatus_hz = 100\n{spacemouse}",
        keys.public_key().to_hex()
    ))
    .unwrap();
    let monitor = match nav {
        Nav::Enabled { plugged } => {
            if plugged {
                pad.plug();
            }
            Some(
                Monitor::spawn(
                    config.spacemouse.socket.clone(),
                    Duration::from_millis(config.spacemouse.poll_ms),
                )
                .unwrap(),
            )
        }
        Nav::Disabled => None,
    };
    if let Some(m) = &monitor {
        // Subscriptions are seeded from the presence at subscribe time, so wait for the
        // monitor to have connected and learned the device state before any test
        // subscribes.
        let latest = m.latest();
        let want = if nav == (Nav::Enabled { plugged: true }) {
            Health::Ready
        } else {
            Health::NoDevice
        };
        wait_until("daemon state learned", || latest.get().health == want).await;
    }
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
    // Disabled in the config means no handle is offered, whatever the caller passes.
    let source = monitor
        .as_ref()
        .map(|m| NavSource::new(m.latest(), config.spacemouse.settings()));
    let bridge = Bridge::with_nav(config, inbox, rx, source).unwrap();
    tokio::spawn(bridge.serve(listener, shutdown_rx));
    Rig {
        url,
        keys,
        fake,
        pad,
        stop,
        shutdown,
        _monitor: monitor,
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

/// Receive until a message of one of `types`, skipping status.
async fn recv_type(ws: &mut Ws, types: &[&str]) -> Option<Value> {
    loop {
        let m = recv(ws).await?;
        if types.iter().any(|t| m["type"] == *t) {
            return Some(m);
        }
        assert_ne!(m["type"], "error", "unexpected error: {m}");
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
    assert_eq!(hello["type"], "hello");
    let challenge = hello["challenge"].as_str().unwrap().to_owned();
    (ws, challenge)
}

async fn authed(r: &Rig) -> Ws {
    let (mut ws, challenge) = connect(r).await;
    send(
        &mut ws,
        json!({"v":1,"type":"auth","event":auth_event(&r.keys, &challenge)}),
    )
    .await;
    assert_eq!(
        recv_type(&mut ws, &["auth_ok"]).await.unwrap()["type"],
        "auth_ok"
    );
    ws
}

async fn subscribe(ws: &mut Ws) -> Value {
    send(ws, json!({"v":1,"type":"nav_subscribe"})).await;
    recv_type(ws, &["nav_ok"]).await.unwrap()
}

/// For `dur`, keep the session alive with a target every 30 ms (`p` and `view` as
/// given) and collect everything received.
async fn pump(ws: &mut Ws, dur: Duration, seq: &mut u64, p: Option<[f64; 3]>) -> Vec<Value> {
    let end = Instant::now() + dur;
    let mut out = Vec::new();
    let mut next_send = Instant::now();
    while Instant::now() < end {
        if Instant::now() >= next_send {
            *seq += 1;
            send(ws, json!({"v":1,"type":"target","seq":*seq,"p":p})).await;
            next_send += Duration::from_millis(30);
        }
        match tokio::time::timeout(Duration::from_millis(4), ws.next()).await {
            Ok(Some(Ok(Message::Text(t)))) => out.push(serde_json::from_str(&t).unwrap()),
            Ok(Some(Ok(_))) => {}
            Ok(_) => break,
            Err(_) => {}
        }
    }
    out
}

fn navs(msgs: &[Value]) -> Vec<&Value> {
    msgs.iter().filter(|m| m["type"] == "nav").collect()
}

async fn wait_until(what: &str, mut ok: impl FnMut() -> bool) {
    let start = Instant::now();
    while !ok() {
        assert!(start.elapsed() < Duration::from_secs(3), "{what}");
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_authenticated_subscriber_gets_bounded_frames_then_one_zero_frame() {
    let r = rig(Nav::Enabled { plugged: true }).await;
    let mut ws = authed(&r).await;
    let ok = subscribe(&mut ws).await;
    assert_eq!(ok["v"], 1);
    assert_eq!(
        (ok["enabled"].clone(), ok["hz"].clone()),
        (json!(true), json!(60))
    );
    wait_until("device read", || {
        r._monitor.as_ref().unwrap().latest().get().health.present()
    })
    .await;

    let mut seq = 0;
    let quiet = pump(&mut ws, Duration::from_millis(200), &mut seq, None).await;
    assert!(navs(&quiet).is_empty(), "a device at rest sends nothing");

    r.pad.push_translation(350, 0, -175);
    r.pad.push_rotation(0, 0, 350);
    let held = pump(&mut ws, Duration::from_millis(500), &mut seq, None).await;
    let frames = navs(&held);
    assert!(
        (10..=36).contains(&frames.len()),
        "about 60 Hz over 0.5 s, never faster: {}",
        frames.len()
    );
    let last = frames.last().unwrap();
    assert_eq!(last["v"], 1);
    assert_eq!(last["tx"], 1.0);
    assert!(last["tz"].as_f64().unwrap() < -0.4);
    assert_eq!(last["rz"], 1.0);
    let seqs: Vec<u64> = frames.iter().map(|f| f["seq"].as_u64().unwrap()).collect();
    assert!(seqs.windows(2).all(|w| w[1] == w[0] + 1), "{seqs:?}");
    let ts: Vec<u64> = frames.iter().map(|f| f["t"].as_u64().unwrap()).collect();
    assert!(ts.windows(2).all(|w| w[1] >= w[0]));

    r.pad.push_translation(0, 0, 0);
    r.pad.push_rotation(0, 0, 0);
    let after = pump(&mut ws, Duration::from_millis(400), &mut seq, None).await;
    let after_navs = navs(&after);
    let zero_frames = after_navs
        .iter()
        .filter(|f| {
            ["tx", "ty", "tz", "rx", "ry", "rz"]
                .iter()
                .all(|k| f[*k] == 0.0)
        })
        .count();
    assert_eq!(
        zero_frames, 1,
        "exactly one zero frame on release: {after_navs:?}"
    );
    assert_eq!(
        after_navs.last().unwrap()["tx"],
        0.0,
        "and nothing follows it"
    );
    let tail = pump(&mut ws, Duration::from_millis(200), &mut seq, None).await;
    assert!(navs(&tail).is_empty());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_unauthenticated_client_gets_nothing() {
    let r = rig(Nav::Enabled { plugged: true }).await;
    wait_until("device read", || {
        r._monitor.as_ref().unwrap().latest().get().health.present()
    })
    .await;
    r.pad.push_translation(350, 350, 350);

    // Silent after the hello: no nav, however hard the device is deflected.
    let (mut idle, _) = connect(&r).await;
    let got = tokio::time::timeout(Duration::from_millis(300), idle.next()).await;
    assert!(got.is_err(), "nothing is sent before auth: {got:?}");

    // Asking for navigation first is refused and closes the socket.
    let (mut eager, _) = connect(&r).await;
    send(&mut eager, json!({"v":1,"type":"nav_subscribe"})).await;
    let e = recv(&mut eager).await.unwrap();
    assert_eq!(
        (e["type"].as_str(), e["code"].as_str()),
        (Some("error"), Some("unauthorised"))
    );
    assert!(recv(&mut eager).await.is_none());

    // A stranger's key fares the same.
    let (mut stranger, challenge) = connect(&r).await;
    send(
        &mut stranger,
        json!({"v":1,"type":"auth","event":auth_event(&Keys::generate(), &challenge)}),
    )
    .await;
    assert_eq!(recv(&mut stranger).await.unwrap()["code"], "unauthorised");
    assert!(recv(&mut stranger).await.is_none());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_v1_client_that_never_subscribes_sees_no_nav_message() {
    let r = rig(Nav::Enabled { plugged: true }).await;
    let mut ws = authed(&r).await;
    wait_until("device read", || {
        r._monitor.as_ref().unwrap().latest().get().health.present()
    })
    .await;
    r.pad.push_translation(350, 0, 0);
    let mut seq = 0;
    let msgs = pump(&mut ws, Duration::from_millis(600), &mut seq, None).await;
    assert!(msgs.len() > 10, "the session is live: {}", msgs.len());
    for m in &msgs {
        assert_eq!(m["type"], "status", "only v1 messages: {m}");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_busy_second_client_gets_no_nav() {
    let r = rig(Nav::Enabled { plugged: true }).await;
    let mut first = authed(&r).await;
    subscribe(&mut first).await;
    wait_until("device read", || {
        r._monitor.as_ref().unwrap().latest().get().health.present()
    })
    .await;
    r.pad.push_translation(350, 0, 0);
    let (mut second, challenge) = connect(&r).await;
    send(
        &mut second,
        json!({"v":1,"type":"auth","event":auth_event(&r.keys, &challenge)}),
    )
    .await;
    assert_eq!(recv(&mut second).await.unwrap()["code"], "busy");
    assert!(recv(&mut second).await.is_none());
    let mut seq = 0;
    let msgs = pump(&mut first, Duration::from_millis(200), &mut seq, None).await;
    assert!(!navs(&msgs).is_empty(), "the controller is undisturbed");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn no_device_then_a_device_then_none_again() {
    let r = rig(Nav::Enabled { plugged: false }).await;
    let mut ws = authed(&r).await;
    let ok = subscribe(&mut ws).await;
    assert_eq!(ok["present"], false, "absent is an answer, not an error");
    let mut seq = 0;
    let none = pump(&mut ws, Duration::from_millis(300), &mut seq, None).await;
    assert!(navs(&none).is_empty());
    assert!(
        none.iter().all(|m| m["type"] == "status"),
        "the bridge carries on"
    );

    r.pad.plug();
    wait_until("hot-plugged", || {
        r._monitor.as_ref().unwrap().latest().get().health.present()
    })
    .await;
    let plugged = pump(&mut ws, Duration::from_millis(200), &mut seq, None).await;
    let n = navs(&plugged);
    assert_eq!(n.len(), 1, "one frame announces the plug: {n:?}");
    assert_eq!(n[0]["present"], true);
    assert!(
        ["tx", "ty", "tz", "rx", "ry", "rz"]
            .iter()
            .all(|k| n[0][*k] == 0.0)
    );
    r.pad.push_translation(0, 350, 0);
    let held = pump(&mut ws, Duration::from_millis(300), &mut seq, None).await;
    assert_eq!(navs(&held).last().unwrap()["ty"], 1.0);
    assert!(navs(&held).iter().all(|f| f["present"] == true));

    r.pad.unplug();
    let gone = pump(&mut ws, Duration::from_millis(400), &mut seq, None).await;
    let n = navs(&gone);
    assert_eq!(
        n.last().unwrap()["ty"],
        0.0,
        "unplug mid-deflection ends in a zero frame"
    );
    assert_eq!(
        n.last().unwrap()["present"],
        false,
        "and it says the device is gone"
    );
    let tail = pump(&mut ws, Duration::from_millis(200), &mut seq, None).await;
    assert!(navs(&tail).is_empty());
    // The session is still alive and answers a new subscription.
    assert_eq!(subscribe(&mut ws).await["present"], false);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_bridge_without_spacemouse_answers_disabled_and_sends_no_frames() {
    let r = rig(Nav::Disabled).await;
    let mut ws = authed(&r).await;
    let ok = subscribe(&mut ws).await;
    assert_eq!(
        (
            ok["enabled"].clone(),
            ok["hz"].clone(),
            ok["present"].clone()
        ),
        (json!(false), json!(0), json!(false))
    );
    let mut seq = 0;
    let msgs = pump(&mut ws, Duration::from_millis(250), &mut seq, None).await;
    assert!(navs(&msgs).is_empty());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn unsubscribing_stops_the_frames() {
    let r = rig(Nav::Enabled { plugged: true }).await;
    let mut ws = authed(&r).await;
    subscribe(&mut ws).await;
    wait_until("device read", || {
        r._monitor.as_ref().unwrap().latest().get().health.present()
    })
    .await;
    r.pad.push_translation(350, 0, 0);
    let mut seq = 0;
    let on = pump(&mut ws, Duration::from_millis(200), &mut seq, None).await;
    assert!(!navs(&on).is_empty());
    send(&mut ws, json!({"v":1,"type":"nav_unsubscribe"})).await;
    let _ = pump(&mut ws, Duration::from_millis(60), &mut seq, None).await;
    let off = pump(&mut ws, Duration::from_millis(250), &mut seq, None).await;
    assert!(navs(&off).is_empty());
}

/// Authenticate, arm, release-then-press and hold a target `p` until force flows.
async fn engaged(r: &Rig, p: [f64; 3]) -> (Ws, u64) {
    let mut ws = authed(r).await;
    send(&mut ws, json!({"v":1,"type":"arm"})).await;
    r.fake.set_button(false);
    let mut seq = 0;
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        assert!(Instant::now() < deadline, "never engaged");
        seq += 1;
        send(&mut ws, json!({"v":1,"type":"target","seq":seq,"p":p})).await;
        let s = recv_type(&mut ws, &["status"]).await.unwrap();
        if s["armed"] == true && s["deadman_ready"] == true {
            r.fake.set_button(true);
        }
        if s["reason"] == "engaged" && r.fake.snapshot().max_force_n > 0.5 {
            return (ws, seq);
        }
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
}

fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    norm([a[0] - b[0], a[1] - b[1], a[2] - b[2]])
}

fn vec3(v: &Value) -> [f64; 3] {
    [
        v[0].as_f64().unwrap(),
        v[1].as_f64().unwrap(),
        v[2].as_f64().unwrap(),
    ]
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn navigation_during_a_hold_never_steps_force() {
    let r = rig(Nav::Enabled { plugged: true }).await;
    // 2 mm at 0.4 N/mm is about 0.8 N: well under the 1.41 N cap, so a target shifted by
    // even 0.1 mm would show.
    let target = [0.002, 0., 0.];
    let (mut ws, mut seq) = engaged(&r, target).await;
    subscribe(&mut ws).await;
    wait_until("device read", || {
        r._monitor.as_ref().unwrap().latest().get().health.present()
    })
    .await;
    let settle = pump(&mut ws, Duration::from_millis(900), &mut seq, Some(target)).await;
    let f0 = r.fake.snapshot().force;
    assert!(
        norm(f0) > 0.3 && norm(f0) < 1.3,
        "settled below the cap: {f0:?}"
    );
    let anchor0 = vec3(&settle.iter().rev().find(|m| m["type"] == "status").unwrap()["anchor"]);

    // Every axis and two buttons, fully deflected.
    r.pad.push_translation(350, -350, 350);
    r.pad.push_rotation(350, 350, -350);
    r.pad.push_buttons(0b11);

    let start = Instant::now();
    let mut prev = (r.fake.snapshot().force, Instant::now());
    let mut nav_frames = 0;
    let mut worst_step = 0.0f64;
    let mut worst_drift = 0.0f64;
    while start.elapsed() < Duration::from_millis(700) {
        let msgs = pump(&mut ws, Duration::from_millis(4), &mut seq, Some(target)).await;
        let f = r.fake.snapshot().force;
        let now = Instant::now();
        let dt = now.duration_since(prev.1).as_secs_f64();
        worst_step = worst_step.max(dist(f, prev.0) - 20. * (dt + 0.003));
        worst_drift = worst_drift.max(dist(f, f0));
        prev = (f, now);
        for m in &msgs {
            if m["type"] == "nav" {
                nav_frames += 1;
            }
            if m["type"] == "status" {
                assert_eq!(m["reason"], "engaged", "{m}");
                assert!(
                    dist(vec3(&m["anchor"]), anchor0) < 1e-6,
                    "the anchor never moves: {m}"
                );
                assert!(m["target_seq"].as_u64().unwrap() <= seq);
            }
        }
    }
    assert!(
        nav_frames >= 15,
        "navigation really was flowing: {nav_frames}"
    );
    assert!(
        worst_step <= 0.01,
        "no force step beyond the rate limit: {worst_step}"
    );
    assert!(
        worst_drift < 0.02,
        "force follows the target alone: {worst_drift} N"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_view_steered_by_navigation_under_a_hold_stays_within_the_force_limits() {
    let r = rig(Nav::Enabled { plugged: true }).await;
    let scale = 1000.; // scene units per metre
    // The held node sits at a fixed scene point, 2 mm (2 scene units) from the view's
    // starting origin. When navigation moves the view, the same scene point lands
    // somewhere else in the pen's workspace.
    let held_scene = [2., 0., 0.];
    let target_msg = |seq: u64, origin_x: f64| {
        json!({"v":1,"type":"target","seq":seq,"p":held_scene,
               "view":{"origin":[origin_x,0.,0.],"rotation":[0.,0.,0.,1.],"scale":scale}})
    };
    let (mut ws, mut seq) = engaged(&r, [0.002, 0., 0.]).await;
    subscribe(&mut ws).await;
    wait_until("device read", || {
        r._monitor.as_ref().unwrap().latest().get().health.present()
    })
    .await;

    // The test client stands in for a camera: it integrates nav into its own view
    // at 2 m of scene travel per second at full deflection (about as brisk as a
    // SpaceMouse throw), and resends the held scene point through the moving view.
    r.pad.push_translation(350, 0, 0);
    let mut origin_x = 0.;
    let mut anchors: Vec<(u64, [f64; 3])> = Vec::new();
    let mut worst_step = 0.0f64;
    let mut peak = 0.0f64;
    let mut prev = (r.fake.snapshot().force, Instant::now());
    let mut next_send = Instant::now();
    let start = Instant::now();
    let mut moved = false;
    while start.elapsed() < Duration::from_millis(900) {
        if Instant::now() >= next_send {
            seq += 1;
            send(&mut ws, target_msg(seq, origin_x)).await;
            next_send += Duration::from_millis(15);
        }
        if let Ok(Some(Ok(Message::Text(t)))) =
            tokio::time::timeout(Duration::from_millis(3), ws.next()).await
        {
            let m: Value = serde_json::from_str(&t).unwrap();
            if m["type"] == "nav" {
                origin_x += m["tx"].as_f64().unwrap() * 2. * scale / 60.;
                moved |= origin_x.abs() > 100.;
            } else if m["type"] == "status" && !m["anchor"].is_null() {
                anchors.push((m["t_ms"].as_u64().unwrap(), vec3(&m["anchor"])));
            }
        }
        let f = r.fake.snapshot().force;
        let now = Instant::now();
        let dt = now.duration_since(prev.1).as_secs_f64();
        worst_step = worst_step.max(dist(f, prev.0) - 20. * (dt + 0.003));
        peak = peak.max(norm(f));
        prev = (f, now);
    }
    assert!(moved, "the view really travelled: {origin_x}");
    assert!(
        worst_step <= 0.01,
        "force-rate limit holds under a moving view: {worst_step}"
    );
    assert!(
        peak <= visiontouch_device::unit::CONTINUOUS_N + 1e-9,
        "cap holds: {peak}"
    );
    let mut checked = 0;
    for (i, (ta, a)) in anchors.iter().enumerate() {
        for (tb, b) in &anchors[i + 1..] {
            if tb - ta >= 40 {
                checked += 1;
                assert!(
                    dist(*a, *b) <= 0.25 * (*tb - *ta) as f64 / 1000. * 1.05 + 1e-4,
                    "anchor slew limit holds: {a:?} to {b:?} in {} ms",
                    tb - ta
                );
            }
        }
    }
    assert!(checked > 50, "enough anchor samples: {checked}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_absent_spacenavd_is_a_reason_in_nav_ok_and_status_and_the_bridge_carries_on() {
    let mut r = rig(Nav::Enabled { plugged: false }).await;
    r.pad.halt();
    let latest = r._monitor.as_ref().unwrap().latest();
    wait_until("daemon seen to be gone", || {
        latest.get().health == Health::DaemonAbsent
    })
    .await;
    let mut ws = authed(&r).await;
    let ok = subscribe(&mut ws).await;
    assert_eq!(ok["present"], false);
    assert_eq!(ok["reason"], "spacenavd_absent");
    let mut seq = 0;
    let got = pump(&mut ws, Duration::from_millis(300), &mut seq, None).await;
    assert!(navs(&got).is_empty(), "no nav frames without a daemon");
    let status = got.iter().rev().find(|m| m["type"] == "status").unwrap();
    assert_eq!(status["nav_input"], "spacenavd_absent");
    assert!(status["t_ms"].is_u64(), "the servo status is still flowing");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_spacenavd_that_dies_after_subscribing_ends_in_a_zero_absent_frame_and_a_status_reason() {
    let mut r = rig(Nav::Enabled { plugged: true }).await;
    let mut ws = authed(&r).await;
    let ok = subscribe(&mut ws).await;
    assert_eq!(
        (ok["present"].clone(), ok["reason"].clone()),
        (json!(true), Value::Null)
    );
    r.pad.push_translation(0, 0, 350);
    let mut seq = 0;
    let live = pump(&mut ws, Duration::from_millis(200), &mut seq, None).await;
    assert_eq!(navs(&live).last().unwrap()["tz"], 1.0);
    assert!(
        live.iter()
            .filter(|m| m["type"] == "status")
            .all(|m| m["nav_input"].is_null()),
        "a healthy link reports null"
    );
    r.pad.halt();
    let gone = pump(&mut ws, Duration::from_millis(500), &mut seq, None).await;
    let last = *navs(&gone).last().unwrap();
    assert_eq!(
        (last["tz"].clone(), last["present"].clone()),
        (json!(0.0), json!(false))
    );
    let status = gone.iter().rev().find(|m| m["type"] == "status").unwrap();
    assert!(
        ["spacenavd_lost", "spacenavd_absent"].contains(&status["nav_input"].as_str().unwrap()),
        "{status}"
    );
}
