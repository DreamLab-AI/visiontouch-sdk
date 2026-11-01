//! The monitor against a fake spacenavd on a Unix socket. Nothing here opens hardware.
use spnav_client::testing::{FakeDaemon, unique_socket_path};
use spnav_client::{Health, Latest, Monitor, Snapshot};
use std::io::{Read, Write};
use std::os::unix::net::UnixListener;
use std::thread;
use std::time::{Duration, Instant};

const POLL: Duration = Duration::from_millis(20);

fn wait(what: &str, latest: &Latest, mut ok: impl FnMut(&Snapshot) -> bool) {
    let start = Instant::now();
    while !ok(&latest.get()) {
        assert!(
            start.elapsed() < Duration::from_secs(3),
            "{what}: {:?}",
            latest.get()
        );
        thread::sleep(Duration::from_millis(2));
    }
}

fn monitor(d: &FakeDaemon) -> Monitor {
    Monitor::spawn(d.path(), POLL).unwrap()
}

#[test]
fn motion_is_latched_and_both_halves_combine() {
    let d = FakeDaemon::start();
    let m = monitor(&d);
    let l = m.latest();
    wait("ready", &l, |s| s.health == Health::Ready);
    d.push_translation(350, 0, -175);
    wait("translation", &l, |s| s.axes == [350, 0, -175, 0, 0, 0]);
    d.push_rotation(0, 40, -350);
    wait("rotation joins", &l, |s| {
        s.axes == [350, 0, -175, 0, 40, -350]
    });
    d.push_translation(0, 0, 0);
    d.push_rotation(0, 0, 0);
    wait("released", &l, |s| s.axes == [0; 6]);
}

#[test]
fn a_button_press_and_release_set_and_clear_its_bit() {
    let d = FakeDaemon::start();
    let m = monitor(&d);
    let l = m.latest();
    wait("ready", &l, |s| s.health == Health::Ready);
    d.push_buttons(0b101);
    wait("press", &l, |s| s.buttons == 0b101);
    d.push_buttons(0b100);
    wait("release one", &l, |s| s.buttons == 0b100);
    d.push_buttons(0);
    wait("release all", &l, |s| s.buttons == 0);
}

#[test]
fn a_hostile_button_number_is_ignored_not_shifted() {
    let d = FakeDaemon::start();
    let m = monitor(&d);
    let l = m.latest();
    wait("ready", &l, |s| s.health == Health::Ready);
    for n in [-1, 32, 33, i32::MAX, i32::MIN] {
        d.push_raw([1, n, 0, 0, 0, 0, 0, 0]);
    }
    d.push_raw([1, 3, 0, 0, 0, 0, 0, 0]);
    wait("button 3", &l, |s| s.buttons == 0b1000);
}

#[test]
fn no_daemon_is_a_quiet_zero_snapshot() {
    let path = unique_socket_path();
    let m = Monitor::spawn(&path, POLL).unwrap();
    thread::sleep(Duration::from_millis(100));
    let s = m.latest().get();
    assert_eq!(s.health, Health::DaemonAbsent);
    assert_eq!((s.axes, s.buttons, s.health.present()), ([0; 6], 0, false));
    assert_eq!(s.health.reason(), Some("spacenavd_absent"));
}

#[test]
fn a_stale_socket_file_is_the_same_as_no_daemon() {
    let path = unique_socket_path();
    drop(UnixListener::bind(&path).unwrap()); // leaves the file behind: ECONNREFUSED
    assert!(path.exists());
    let m = Monitor::spawn(&path, POLL).unwrap();
    thread::sleep(Duration::from_millis(100));
    assert_eq!(m.latest().get().health, Health::DaemonAbsent);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn a_daemon_that_appears_later_is_picked_up() {
    let path = unique_socket_path();
    let m = Monitor::spawn(&path, POLL).unwrap();
    let l = m.latest();
    thread::sleep(Duration::from_millis(80));
    assert_eq!(l.get().health, Health::DaemonAbsent);
    let d = FakeDaemon::at(&path).unwrap();
    wait("daemon found, no device yet", &l, |s| {
        s.health == Health::NoDevice
    });
    d.plug();
    wait("device", &l, |s| s.health == Health::Ready);
    d.push_translation(0, 350, 0);
    wait("reads", &l, |s| s.axes[1] == 350);
}

#[test]
fn a_device_already_absent_at_connect_is_learned_from_the_query() {
    let path = unique_socket_path();
    let d = FakeDaemon::at(&path).unwrap(); // up, no device
    let m = monitor(&d);
    let l = m.latest();
    wait("no device", &l, |s| s.health == Health::NoDevice);
    assert_eq!(l.get().health.reason(), Some("no_device"));
    assert_eq!(d.connections(), 1);
}

#[test]
fn an_unplug_mid_stream_zeroes_the_state_and_a_replug_resumes_without_reconnecting() {
    let d = FakeDaemon::start();
    let m = monitor(&d);
    let l = m.latest();
    wait("ready", &l, |s| s.health == Health::Ready);
    d.push_translation(350, 350, 350);
    d.push_buttons(0b11);
    wait("deflected", &l, |s| s.axes[2] == 350 && s.buttons == 0b11);
    d.unplug();
    wait("absent and still", &l, |s| {
        s.health == Health::NoDevice && s.axes == [0; 6] && s.buttons == 0
    });
    d.plug();
    wait("present again", &l, |s| s.health == Health::Ready);
    assert_eq!(l.get().axes, [0; 6], "no stale deflection");
    d.push_translation(0, 350, 0);
    wait("reads again", &l, |s| s.axes[1] == 350);
    assert_eq!(d.connections(), 1, "the daemon never went away");
}

#[test]
fn a_daemon_that_dies_mid_stream_zeroes_the_state_and_a_reconnect_resumes() {
    let d = FakeDaemon::start();
    let m = monitor(&d);
    let l = m.latest();
    wait("ready", &l, |s| s.health == Health::Ready);
    d.push_translation(350, 0, 0);
    wait("deflected", &l, |s| s.axes[0] == 350);
    d.disconnect_client();
    wait("lost or reconnected", &l, |_| d.connections() >= 2);
    wait("ready again with no stale motion", &l, |s| {
        s.health == Health::Ready && s.axes == [0; 6]
    });
    d.push_translation(0, 0, 350);
    wait("reads again", &l, |s| s.axes[2] == 350);
}

#[test]
fn a_dropped_connection_is_reported_lost_with_zero_input_until_the_retry() {
    let d = FakeDaemon::start();
    let m = Monitor::spawn(d.path(), Duration::from_millis(600)).unwrap();
    let l = m.latest();
    wait("ready", &l, |s| s.health == Health::Ready);
    d.push_translation(350, 0, 0);
    d.push_buttons(1);
    wait("deflected", &l, |s| s.axes[0] == 350 && s.buttons == 1);
    d.disconnect_client();
    wait("lost, zeroed", &l, |s| {
        s.health == Health::DaemonLost && s.axes == [0; 6] && s.buttons == 0
    });
    assert_eq!(l.get().health.reason(), Some("spacenavd_lost"));
    wait("retried", &l, |s| s.health == Health::Ready);
}

#[test]
fn a_halted_daemon_is_reported_lost_then_absent() {
    let mut d = FakeDaemon::start();
    let m = monitor(&d);
    let l = m.latest();
    wait("ready", &l, |s| s.health == Health::Ready);
    d.push_rotation(5, 5, 5);
    wait("deflected", &l, |s| s.axes[3] == 5);
    d.halt();
    wait("absent and still", &l, |s| {
        s.health == Health::DaemonAbsent && s.axes == [0; 6]
    });
}

#[test]
fn an_old_daemon_gets_no_handshake_reply_and_reads_as_protocol_zero() {
    let d = FakeDaemon::start().protocol0();
    let m = monitor(&d);
    let l = m.latest();
    wait("v0 ready", &l, |s| s.health == Health::ReadyV0);
    assert!(
        l.get().health.present(),
        "presence is unknown, input is live"
    );
    d.push_translation(350, 0, 0);
    wait("reads", &l, |s| s.axes[0] == 350);
}

#[test]
fn a_frame_split_across_reads_is_reassembled() {
    let path = unique_socket_path();
    let listener = UnixListener::bind(&path).unwrap();
    let server = thread::spawn(move || {
        let (mut s, _) = listener.accept().unwrap();
        let mut hello = [0u8; 4];
        s.read_exact(&mut hello).unwrap();
        s.write_all(&hello).unwrap(); // accept protocol 1
        let mut req = [0u8; 32];
        s.read_exact(&mut req).unwrap(); // the device query; answer: a device exists
        let mut reply = [0u8; 32];
        reply[..4].copy_from_slice(&(0x7faa_2002i32).to_ne_bytes());
        s.write_all(&reply).unwrap();
        let mut motion = [0u8; 32];
        motion[4..8].copy_from_slice(&123i32.to_ne_bytes());
        motion[24..28].copy_from_slice(&(-9i32).to_ne_bytes());
        s.write_all(&motion[..13]).unwrap();
        s.flush().unwrap();
        thread::sleep(Duration::from_millis(80));
        s.write_all(&motion[13..]).unwrap();
        thread::sleep(Duration::from_millis(300));
    });
    let m = Monitor::spawn(&path, POLL).unwrap();
    let l = m.latest();
    wait("whole frame decoded", &l, |s| {
        s.axes == [123, 0, 0, 0, 0, -9]
    });
    drop(m);
    server.join().unwrap();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn dropping_the_monitor_stops_the_thread_and_leaves_zero() {
    let d = FakeDaemon::start();
    let m = monitor(&d);
    let l = m.latest();
    wait("ready", &l, |s| s.health == Health::Ready);
    d.push_translation(350, 0, 0);
    wait("deflected", &l, |s| s.axes[0] == 350);
    drop(m);
    let s = l.get();
    assert_eq!((s.axes, s.health.present()), ([0; 6], false));
}
