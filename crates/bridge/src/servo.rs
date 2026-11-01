//! The servo thread: device service, gate, coupling, at the device rate.
//!
//! The servo loop owns the device and runs on a plain OS thread, never on the async
//! runtime. It reads the network's latest wishes from a [`SharedInbox`] (a mutex held
//! for a copy, never across I/O) and publishes its [`Status`] through a
//! `tokio::sync::watch` channel, which is non-blocking from a plain thread. Nothing on
//! the network side can stall it.
//!
//! Arming follows the network: the device is armed when an authenticated controller
//! asks, and disarmed as soon as the controller disarms, disconnects or is dropped.
//! A device error is final: output has been shut down, the loop exits and the bridge
//! needs a restart at the arm.
use crate::config::Config;
use crate::coupling::{Coupling, TargetFilter, VelocityFilter};
use crate::gate::{Gate, GateIn, Reason};
use crate::protocol::Status;
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use visiontouch_device::unit::{State, Unit};

/// Servo period: the 2.4 ms the transport sustains (about 400 Hz).
pub const SERVO_PERIOD: Duration = Duration::from_micros(2400);
/// Longest interval fed to the filters, so a stalled tick cannot make a large step.
const MAX_DT_S: f64 = 0.025;
/// Pen velocity low-pass time constant, s.
const PEN_VELOCITY_TAU_S: f64 = 0.01;

/// What the servo loop needs from a device. Implemented for
/// [`visiontouch_device::unit::Unit`], and by [`crate::fake::FakeUnit`] for tests.
pub trait Haptic {
    /// Arm output (zero-only handshake on the real unit).
    fn arm(&mut self) -> io::Result<()>;
    /// Turn output off.
    fn disarm(&mut self) -> io::Result<()>;
    /// Set the delivered force for the next service, newtons, model frame.
    fn set_force(&mut self, force_n: [f64; 3]);
    /// Send the current force and read one sample.
    fn service(&mut self) -> io::Result<State>;
}

impl Haptic for Unit {
    fn arm(&mut self) -> io::Result<()> {
        Unit::arm(self)
    }
    fn disarm(&mut self) -> io::Result<()> {
        Unit::disarm(self)
    }
    fn set_force(&mut self, force_n: [f64; 3]) {
        Unit::set_force(self, force_n)
    }
    fn service(&mut self) -> io::Result<State> {
        Unit::service(self)
    }
}

/// A coupling target as received, stamped with the bridge's receive time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Target {
    /// Position, metres, model frame.
    pub p: [f64; 3],
    /// When it arrived.
    pub at: Instant,
}

/// The network's latest wishes. Latest value only: nothing queues.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Inbox {
    /// The authenticated controller's connection id, while connected.
    pub controller: Option<u64>,
    /// The controller wants force armed.
    pub arm_wanted: bool,
    /// Incremented on every `arm` request; a latched fault clears on the next one.
    pub arm_epoch: u64,
    /// The current target; `None` when nothing is held.
    pub target: Option<Target>,
    /// Sequence number of the last target message, `null` targets included.
    pub target_seq: Option<u64>,
}

impl Inbox {
    /// Drop the controller: no link, no arm, no target.
    pub fn release(&mut self) {
        self.controller = None;
        self.arm_wanted = false;
        self.target = None;
        self.target_seq = None;
    }
}

/// The inbox shared between the network and the servo thread.
pub type SharedInbox = Arc<Mutex<Inbox>>;

/// Read the inbox. A poisoned lock (a network task panicked mid-write) reads as no
/// controller, so force stops.
pub fn snapshot(inbox: &SharedInbox) -> Inbox {
    match inbox.lock() {
        Ok(g) => *g,
        Err(_) => Inbox::default(),
    }
}

/// Status for a device that has faulted.
pub fn fault_status(t_ms: u64) -> Status {
    Status {
        t_ms,
        pen: None,
        button: false,
        deadman_ready: false,
        armed: false,
        reason: Reason::Fault,
        anchor: None,
        force: [0.; 3],
        applied_n: 0.,
        heat_n2s: 0.,
        heat_limited: false,
        target_seq: None,
        target_age_ms: None,
        device_fault: true,
    }
}

/// One servo loop's state. Drive it with [`Servo::step`]; [`spawn`] runs it on a thread.
pub struct Servo<H> {
    dev: H,
    coupling: Coupling,
    filter: TargetFilter,
    gate: Gate,
    pen_v: VelocityFilter,
    start: Instant,
    last_tick: Option<Instant>,
    fault_epoch: Option<u64>,
    arm_attempt_epoch: Option<u64>,
}

impl<H: Haptic> Servo<H> {
    /// A servo loop for `dev` with the configured coupling and timeouts. `start` is the
    /// zero of `Status::t_ms`.
    pub fn new(dev: H, config: &Config, start: Instant) -> Self {
        Self {
            dev,
            coupling: config.coupling(),
            filter: config.target_filter(),
            gate: Gate::new(config.target_timeout(), config.ramp()),
            pen_v: VelocityFilter::new(PEN_VELOCITY_TAU_S),
            start,
            last_tick: None,
            fault_epoch: None,
            arm_attempt_epoch: None,
        }
    }

    /// The device.
    pub fn device(&self) -> &H {
        &self.dev
    }

    /// Give the device back.
    pub fn into_device(self) -> H {
        self.dev
    }

    /// Zero the force and disarm. Used on shutdown.
    pub fn shutdown(&mut self) -> io::Result<()> {
        self.dev.set_force([0.; 3]);
        self.dev.disarm()
    }

    /// One tick at time `now` with the network's latest `inbox`. Any error is a device
    /// fault: output is off and the loop must stop.
    pub fn step(&mut self, now: Instant, inbox: Inbox) -> io::Result<Status> {
        let dt = self
            .last_tick
            .map(|t| now.saturating_duration_since(t).as_secs_f64().min(MAX_DT_S))
            .unwrap_or(0.);
        self.last_tick = Some(now);

        let state = self.dev.service()?;
        let epoch = inbox.arm_epoch;
        let link_up = inbox.controller.is_some();
        let fault = self.fault_epoch == Some(epoch);
        let want = link_up && inbox.arm_wanted && !fault;
        if want && !state.armed && self.arm_attempt_epoch != Some(epoch) {
            self.arm_attempt_epoch = Some(epoch);
            if self.dev.arm().is_err() {
                self.fault_epoch = Some(epoch);
            }
        }
        if !want && state.armed {
            self.dev.set_force([0.; 3]);
            self.dev.disarm()?;
        }

        let pen_v = match state.position_m {
            Some(p) => self.pen_v.step(p, dt),
            None => {
                self.pen_v.clear();
                [0.; 3]
            }
        };
        let decision = self.gate.evaluate(GateIn {
            now,
            link_up,
            arm_wanted: inbox.arm_wanted,
            device_armed: state.armed && want,
            pen_valid: state.position_m.is_some(),
            fault: self.fault_epoch == Some(epoch),
            target_at: inbox.target.map(|t| t.at),
            button: state.button,
        });

        let mut force = [0.; 3];
        let mut anchor = None;
        let mut reason = decision.reason;
        if let (Reason::Engaged, Some(pen), Some(target)) =
            (decision.reason, state.position_m, inbox.target)
        {
            if decision.engaged_now {
                self.filter.reset(pen);
            }
            match self.filter.step(target.p, dt).and_then(|(a, av)| {
                Some((a, self.coupling.force(pen, pen_v, a, av, decision.ramp)?))
            }) {
                Some((a, f)) => {
                    force = f;
                    anchor = Some(a);
                }
                None => {
                    // Non-finite somewhere: zero force and latch a fault until re-arm.
                    self.fault_epoch = Some(epoch);
                    reason = Reason::Fault;
                    self.filter.clear();
                }
            }
        } else {
            self.filter.clear();
        }
        self.dev.set_force(force);

        Ok(Status {
            t_ms: now.saturating_duration_since(self.start).as_millis() as u64,
            pen: state.position_m,
            button: state.button,
            deadman_ready: self.gate.seen_released(),
            armed: state.armed,
            reason,
            anchor,
            force,
            applied_n: state.applied_n,
            heat_n2s: state.heat_n2s,
            heat_limited: state.heat_limited,
            target_seq: inbox.target_seq,
            target_age_ms: inbox
                .target
                .map(|t| now.saturating_duration_since(t.at).as_millis() as u64),
            device_fault: false,
        })
    }
}

/// Run `servo` on its own thread every `period` until `stop` is set or the device
/// faults, publishing status to `status`. On a normal stop the force is zeroed and
/// output disarmed. Returns the device and how the loop ended.
pub fn spawn<H: Haptic + Send + 'static>(
    mut servo: Servo<H>,
    inbox: SharedInbox,
    status: tokio::sync::watch::Sender<Status>,
    stop: Arc<AtomicBool>,
    period: Duration,
) -> io::Result<JoinHandle<(H, io::Result<()>)>> {
    std::thread::Builder::new()
        .name("visiontouch-servo".into())
        .spawn(move || {
            let mut next = Instant::now();
            let result = loop {
                if stop.load(Ordering::Relaxed) {
                    break servo.shutdown();
                }
                let now = Instant::now();
                match servo.step(now, snapshot(&inbox)) {
                    Ok(s) => {
                        status.send_replace(s);
                    }
                    Err(e) => {
                        let t = now.saturating_duration_since(servo.start).as_millis() as u64;
                        status.send_replace(fault_status(t));
                        // Output is already shut down by the device on a fault; try
                        // once more to be sure, ignoring a second error.
                        let _ = servo.shutdown();
                        break Err(e);
                    }
                }
                next += period;
                match next.checked_duration_since(Instant::now()) {
                    Some(wait) => std::thread::sleep(wait),
                    None => next = Instant::now(),
                }
            };
            (servo.into_device(), result)
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fake::FakeUnit;

    const KEY: &str = "79be667ef9dcbbac55a06295ce870b07029bfcdb2dce28d959f2815b16f81798";

    fn config() -> Config {
        Config::from_toml(&format!("authorised_keys = [\"{KEY}\"]\n")).unwrap()
    }

    struct Rig {
        servo: Servo<FakeUnit>,
        fake: FakeUnit,
        inbox: Inbox,
        t: Instant,
    }

    impl Rig {
        fn new() -> Self {
            let fake = FakeUnit::registered();
            let t = Instant::now();
            Self {
                servo: Servo::new(fake.clone(), &config(), t),
                fake,
                inbox: Inbox::default(),
                t,
            }
        }
        /// Advance one servo period, refreshing the target if one is set.
        fn tick(&mut self) -> Status {
            self.t += SERVO_PERIOD;
            if let Some(tg) = &mut self.inbox.target {
                tg.at = self.t;
            }
            self.servo.step(self.t, self.inbox).unwrap()
        }
        fn ticks(&mut self, n: usize) -> Status {
            let mut s = self.tick();
            for _ in 1..n {
                s = self.tick();
            }
            s
        }
        fn connect_and_arm(&mut self) {
            self.inbox.controller = Some(1);
            self.inbox.arm_wanted = true;
            self.inbox.arm_epoch += 1;
        }
        /// Connected, armed, released then pressed, target 10 mm from the pen.
        fn engaged(&mut self) -> Status {
            self.connect_and_arm();
            self.fake.set_button(false);
            self.ticks(3);
            self.fake.set_button(true);
            self.inbox.target = Some(Target {
                p: [0.01, 0., 0.],
                at: self.t,
            });
            self.ticks(400)
        }
    }

    fn mag(f: [f64; 3]) -> f64 {
        f.iter().map(|x| x * x).sum::<f64>().sqrt()
    }

    #[test]
    fn nothing_moves_without_a_controller() {
        let mut r = Rig::new();
        r.fake.set_button(true);
        r.inbox.target = Some(Target {
            p: [0.01, 0., 0.],
            at: r.t,
        });
        let s = r.ticks(100);
        assert_eq!(s.reason, Reason::NoLink);
        assert_eq!(r.fake.snapshot().arm_calls, 0);
        assert_eq!(r.fake.snapshot().max_force_n, 0.);
    }

    #[test]
    fn the_controller_arms_and_the_deadman_must_be_released_first() {
        let mut r = Rig::new();
        r.fake.set_button(true);
        r.connect_and_arm();
        r.inbox.target = Some(Target {
            p: [0.01, 0., 0.],
            at: r.t,
        });
        let s = r.ticks(200);
        assert!(s.armed);
        assert_eq!(r.fake.snapshot().arm_calls, 1, "armed once per request");
        assert_eq!(s.reason, Reason::AwaitRelease);
        assert_eq!(r.fake.snapshot().max_force_n, 0.);
    }

    #[test]
    fn engaged_force_pulls_the_pen_towards_the_target_and_ramps_in() {
        let mut r = Rig::new();
        r.connect_and_arm();
        r.fake.set_button(false);
        r.ticks(3);
        r.fake.set_button(true);
        r.inbox.target = Some(Target {
            p: [0.01, 0., 0.],
            at: r.t,
        });
        let first = r.tick();
        assert_eq!(first.reason, Reason::Engaged);
        assert!(
            mag(first.force) < 1e-9,
            "first engaged tick is zero: {:?}",
            first.force
        );
        let s = r.ticks(400);
        // Pen at the origin, anchor arrived at +10 mm, 0.4 N/mm: 4 N, capped at 1.41 N, +x.
        assert_eq!(s.reason, Reason::Engaged);
        assert!((s.anchor.unwrap()[0] - 0.01).abs() < 1e-6);
        assert!((mag(s.force) - CONTINUOUS).abs() < 1e-9, "{:?}", s.force);
        assert!(s.force[0] > 0.);
        assert_eq!(
            r.fake.snapshot().force,
            s.force,
            "what was sent is what is reported"
        );
    }

    const CONTINUOUS: f64 = visiontouch_device::unit::CONTINUOUS_N;

    #[test]
    fn a_stale_target_zeroes_force_at_the_timeout() {
        let mut r = Rig::new();
        r.engaged();
        let at = r.t;
        let mut target = r.inbox.target.take().unwrap();
        target.at = at;
        // Hold the target without refreshing it.
        let timeout = config().target_timeout();
        let mut t = at;
        let mut cut_after = None;
        for i in 1..200 {
            t = at + SERVO_PERIOD * i;
            r.inbox.target = Some(target);
            let s = r.servo.step(t, r.inbox).unwrap();
            if s.reason == Reason::TargetStale {
                assert_eq!(s.force, [0.; 3]);
                assert_eq!(r.fake.snapshot().force, [0.; 3]);
                cut_after = Some(t - at);
                break;
            }
        }
        let cut = cut_after.expect("force cut");
        assert!(cut > timeout && cut <= timeout + SERVO_PERIOD, "{cut:?}");
        let _ = t;
    }

    #[test]
    fn link_loss_zeroes_force_and_disarms_on_the_next_tick() {
        let mut r = Rig::new();
        assert!(mag(r.engaged().force) > 0.);
        r.inbox.release();
        let s = r.tick();
        assert_eq!(s.reason, Reason::NoLink);
        assert_eq!(s.force, [0.; 3]);
        assert_eq!(r.fake.snapshot().force, [0.; 3]);
        assert_eq!(r.fake.snapshot().disarm_calls, 1);
        assert!(!r.tick().armed);
    }

    #[test]
    fn disarm_zeroes_force_and_disarms() {
        let mut r = Rig::new();
        r.engaged();
        r.inbox.arm_wanted = false;
        let s = r.tick();
        assert_eq!((s.reason, s.force), (Reason::Disarmed, [0.; 3]));
        assert_eq!(r.fake.snapshot().disarm_calls, 1);
    }

    #[test]
    fn a_reconnect_needs_a_new_release() {
        let mut r = Rig::new();
        r.engaged();
        r.inbox.release();
        r.tick();
        // The same hand, button still held, reconnects and arms.
        r.connect_and_arm();
        r.inbox.target = Some(Target {
            p: [0.01, 0., 0.],
            at: r.t,
        });
        let s = r.ticks(100);
        assert_eq!(s.reason, Reason::AwaitRelease);
        assert_eq!(s.force, [0.; 3]);
    }

    #[test]
    fn an_arm_refusal_latches_a_fault_until_the_next_arm_request() {
        let mut r = Rig::new();
        r.fake.update(|s| s.fail_arm = true);
        r.connect_and_arm();
        let s = r.ticks(10);
        assert_eq!(s.reason, Reason::Fault);
        assert_eq!(r.fake.snapshot().arm_calls, 1, "no retry loop");
        r.fake.update(|s| s.fail_arm = false);
        assert_eq!(r.ticks(10).reason, Reason::Fault, "still latched");
        r.inbox.arm_epoch += 1;
        let s = r.ticks(3);
        assert!(s.armed);
    }

    #[test]
    fn an_unregistered_unit_gives_no_force() {
        let mut r = Rig::new();
        r.fake.update(|s| s.registered = false);
        r.connect_and_arm();
        let s = r.ticks(10);
        // The fake, like the unit, refuses to arm unregistered.
        assert_eq!(s.reason, Reason::Fault);
        assert_eq!(s.pen, None);
        assert_eq!(r.fake.snapshot().max_force_n, 0.);
    }

    #[test]
    fn a_device_fault_is_an_error() {
        let mut r = Rig::new();
        r.engaged();
        r.fake.update(|s| s.fail_service = true);
        r.t += SERVO_PERIOD;
        assert!(r.servo.step(r.t, r.inbox).is_err());
    }

    #[test]
    fn force_never_exceeds_the_cap_whatever_the_target_does() {
        let mut r = Rig::new();
        r.engaged();
        for i in 0..2000 {
            let x = if i % 7 == 0 { 0.9 } else { -0.9 };
            r.inbox.target = Some(Target {
                p: [x, -x, x],
                at: r.t,
            });
            let s = r.tick();
            assert!(mag(s.force) <= CONTINUOUS + 1e-9);
        }
        assert!(r.fake.snapshot().max_force_n <= CONTINUOUS + 1e-9);
    }

    #[test]
    fn the_thread_stops_cleanly_and_zeroes_output() {
        let fake = FakeUnit::registered();
        let inbox: SharedInbox = Arc::default();
        {
            let mut i = inbox.lock().unwrap();
            i.controller = Some(1);
            i.arm_wanted = true;
            i.arm_epoch = 1;
        }
        let (tx, rx) = tokio::sync::watch::channel(fault_status(0));
        let stop = Arc::new(AtomicBool::new(false));
        let h = spawn(
            Servo::new(fake.clone(), &config(), Instant::now()),
            inbox,
            tx,
            stop.clone(),
            Duration::from_millis(1),
        )
        .unwrap();
        while !rx.borrow().armed {
            std::thread::sleep(Duration::from_millis(1));
        }
        stop.store(true, Ordering::Relaxed);
        let (_dev, result) = h.join().unwrap();
        assert!(result.is_ok());
        let s = fake.snapshot();
        assert!(!s.armed);
        assert_eq!(s.force, [0.; 3]);
    }

    #[test]
    fn the_thread_reports_a_device_fault_and_exits() {
        let fake = FakeUnit::registered();
        let mut initial = fault_status(0);
        initial.device_fault = false;
        let (tx, rx) = tokio::sync::watch::channel(initial);
        let h = spawn(
            Servo::new(fake.clone(), &config(), Instant::now()),
            Arc::default(),
            tx,
            Arc::new(AtomicBool::new(false)),
            Duration::from_millis(1),
        )
        .unwrap();
        fake.update(|s| s.fail_service = true);
        let (_dev, result) = h.join().unwrap();
        assert!(result.is_err());
        assert!(rx.borrow().device_fault);
    }
}
