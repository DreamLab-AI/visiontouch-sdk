//! The hot-plug monitor: one thread that finds a device, reads it, and publishes the
//! latest filtered state.
//!
//! The thread loops: scan, open the first device found, read reports until the device
//! goes, publish a zero snapshot, wait one poll interval, repeat. Readers poll with
//! non-blocking reads every [`READ_POLL`], so the thread notices a stop request
//! promptly and [`Monitor`]'s drop joins it.
//!
//! **Failing soft.** An empty scan, a device that cannot be opened (for example for want
//! of permission), a read error and an unplug all end in a [`Snapshot`] that is
//! not present and all-zero. Open failures are logged once per distinct message. Even
//! a panic on the thread leaves a zero, absent snapshot behind (a drop guard), so a
//! published motion can never be left latched.
//!
//! Consumers hold a [`Latest`] and read it whenever they like; there is no queue, so a
//! slow consumer sees only the newest state, which is what a motion stream wants.
use crate::device::Opener;
use crate::filter::{Nav, Settings};
use crate::report::{RawState, parse};
use std::io::{self, Read};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// How long a reader sleeps when no report is waiting.
pub const READ_POLL: Duration = Duration::from_millis(2);

/// What the monitor last saw.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Snapshot {
    /// A device is open and being read.
    pub present: bool,
    /// The filtered state. All zero whenever `present` is false.
    pub nav: Nav,
    /// Incremented on every published change; lets a consumer notice updates cheaply.
    pub version: u64,
}

/// A cheap handle to the newest [`Snapshot`].
#[derive(Clone, Debug, Default)]
pub struct Latest(Arc<Mutex<Snapshot>>);

impl Latest {
    /// The newest snapshot. A poisoned lock reads as absent and still.
    pub fn get(&self) -> Snapshot {
        match self.0.lock() {
            Ok(g) => *g,
            Err(_) => Snapshot::default(),
        }
    }

    fn publish(&self, present: bool, nav: Nav) {
        let mut g = self.0.lock().unwrap_or_else(|e| e.into_inner());
        if g.present != present || g.nav != nav {
            g.present = present;
            g.nav = nav;
            g.version += 1;
        }
    }
}

/// Resets the snapshot when the thread ends, however it ends.
struct Reset(Latest);
impl Drop for Reset {
    fn drop(&mut self) {
        self.0.publish(false, Nav::default());
    }
}

/// The running monitor. Dropping it stops the thread.
pub struct Monitor {
    latest: Latest,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Monitor {
    /// Start the monitor thread. `poll` is the wait between scans while no device is
    /// open. The only error is failing to start the thread.
    pub fn spawn(opener: Box<dyn Opener>, settings: Settings, poll: Duration) -> io::Result<Self> {
        let latest = Latest::default();
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let (latest, stop) = (latest.clone(), stop.clone());
            thread::Builder::new()
                .name("spacemouse".into())
                .spawn(move || run(opener.as_ref(), &settings, poll, &latest, &stop))?
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

fn run(
    opener: &dyn Opener,
    settings: &Settings,
    poll: Duration,
    latest: &Latest,
    stop: &AtomicBool,
) {
    let _reset = Reset(latest.clone());
    let mut last_error = String::new();
    while !stop.load(Ordering::Relaxed) {
        if let Some(info) = opener.scan().into_iter().next() {
            match opener.open(&info) {
                Ok(mut reader) => {
                    last_error.clear();
                    eprintln!(
                        "visiontouch-spacemouse: {} ({:04x}:{:04x}) connected",
                        info.name, info.vendor, info.product
                    );
                    latest.publish(true, Nav::default());
                    read_until_gone(reader.as_mut(), settings, latest, stop);
                    latest.publish(false, Nav::default());
                    eprintln!("visiontouch-spacemouse: {} disconnected", info.name);
                }
                Err(e) => {
                    let msg = format!("{}: {e}", info.path.display());
                    if msg != last_error {
                        eprintln!("visiontouch-spacemouse: cannot open {msg}");
                        last_error = msg;
                    }
                }
            }
        }
        sleep_unless_stopped(poll, stop);
    }
}

fn read_until_gone(reader: &mut dyn Read, settings: &Settings, latest: &Latest, stop: &AtomicBool) {
    let mut raw = RawState::default();
    let mut buf = [0u8; 64];
    while !stop.load(Ordering::Relaxed) {
        match reader.read(&mut buf) {
            Ok(0) => return,
            Ok(n) => {
                if let Some(report) = parse(&buf[..n]) {
                    raw.apply(report);
                    latest.publish(true, settings.apply(&raw));
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => thread::sleep(READ_POLL),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(_) => return,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::ScriptedDevice;
    use std::time::Instant;

    const POLL: Duration = Duration::from_millis(20);

    fn wait(what: &str, mut ok: impl FnMut() -> bool) {
        let start = Instant::now();
        while !ok() {
            assert!(start.elapsed() < Duration::from_secs(3), "{what}");
            thread::sleep(Duration::from_millis(2));
        }
    }

    fn monitor(dev: &ScriptedDevice) -> Monitor {
        let settings = Settings {
            deadzone: 0.,
            ..Settings::default()
        };
        Monitor::spawn(dev.opener(), settings, POLL).unwrap()
    }

    #[test]
    fn no_device_is_a_quiet_zero_snapshot() {
        let dev = ScriptedDevice::new();
        let m = monitor(&dev);
        thread::sleep(Duration::from_millis(100));
        assert_eq!(m.latest().get(), Snapshot::default());
        assert_eq!(dev.opens(), 0);
    }

    #[test]
    fn a_device_plugged_in_later_is_picked_up_and_read() {
        let dev = ScriptedDevice::new();
        let m = monitor(&dev);
        let latest = m.latest();
        thread::sleep(Duration::from_millis(60));
        assert!(!latest.get().present);
        dev.plug();
        wait("device present", || latest.get().present);
        dev.push_translation(350, 0, 0);
        wait("x deflected", || latest.get().nav.axes[0] == 1.);
        dev.push_rotation(0, -175, 0);
        dev.push_buttons(0b10);
        wait("ry and buttons", || {
            let n = latest.get().nav;
            n.axes[4] == -0.5 && n.buttons == 0b10 && n.axes[0] == 1.
        });
    }

    #[test]
    fn an_unplug_zeroes_the_state_and_a_replug_resumes() {
        let dev = ScriptedDevice::new();
        dev.plug();
        let m = monitor(&dev);
        let latest = m.latest();
        wait("present", || latest.get().present);
        dev.push_translation(350, 350, 350);
        wait("deflected", || latest.get().nav.axes[2] == 1.);
        dev.unplug();
        wait("absent and still", || {
            let s = latest.get();
            !s.present && s.nav == Nav::default()
        });
        dev.plug();
        wait("present again", || latest.get().present);
        assert_eq!(latest.get().nav, Nav::default(), "no stale deflection");
        dev.push_translation(0, 350, 0);
        wait("reads again", || latest.get().nav.axes[1] == 1.);
        assert_eq!(dev.opens(), 2);
    }

    #[test]
    fn unrecognised_reports_change_nothing() {
        let dev = ScriptedDevice::new();
        dev.plug();
        let m = monitor(&dev);
        let latest = m.latest();
        wait("present", || latest.get().present);
        let before = latest.get();
        dev.push(&[0x17, 1, 2, 3]);
        dev.push(&[1, 0, 0]);
        thread::sleep(Duration::from_millis(50));
        assert_eq!(latest.get(), before);
    }

    #[test]
    fn dropping_the_monitor_stops_the_thread_and_leaves_zero() {
        let dev = ScriptedDevice::new();
        dev.plug();
        let m = monitor(&dev);
        let latest = m.latest();
        wait("present", || latest.get().present);
        dev.push_translation(350, 0, 0);
        wait("deflected", || latest.get().nav.axes[0] == 1.);
        drop(m);
        assert_eq!(latest.get().nav, Nav::default());
        assert!(!latest.get().present);
    }
}
