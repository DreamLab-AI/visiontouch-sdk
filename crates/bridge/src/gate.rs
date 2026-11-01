//! The force gate: every condition that must hold before the coupling may push.
//!
//! [`Gate::evaluate`] is pure apart from its own latch and takes the time as an
//! argument, so each fail-safe is tested without a clock, a socket or a device.
//! Force is allowed only when **all** of these hold on this tick:
//!
//! 1. an authenticated controller is connected (the link is up);
//! 2. that controller has asked for arm, and the device reports armed;
//! 3. the pen position is available (the unit is registered);
//! 4. no fault is latched since the last arm request;
//! 5. a target is set and younger than the target timeout;
//! 6. the pen button is held **and** has been seen released since force last became
//!    possible (the deadman latch: a button that reads as pressed because the wiring
//!    failed can never start output).
//!
//! Losing any of 1–4 also clears the deadman latch, so after a reconnect, a re-arm or
//! a fault the operator must release and press the button again. When every condition
//! holds, [`GateOut::ramp`] rises from 0 to 1 over the engage time.
use serde::Serialize;
use std::time::{Duration, Instant};

/// Why force is or is not flowing. Sent in every status message.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Reason {
    /// No authenticated controller is connected.
    NoLink,
    /// The controller has not asked for arm, or has disarmed.
    Disarmed,
    /// Arm was asked for and the device is arming or refused.
    DeviceNotArmed,
    /// The unit is not registered, so the pen position is unknown.
    NotRegistered,
    /// A fault is latched until the next arm request.
    Fault,
    /// No target is set (nothing is held).
    NoTarget,
    /// The last target is older than the target timeout.
    TargetStale,
    /// The pen button has not been seen released since arming.
    AwaitRelease,
    /// The pen button is released.
    DeadmanReleased,
    /// Force is flowing.
    Engaged,
}

/// One tick's inputs to the gate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GateIn {
    /// Time of this tick.
    pub now: Instant,
    /// An authenticated controller is connected.
    pub link_up: bool,
    /// The controller wants force armed.
    pub arm_wanted: bool,
    /// The device reports armed.
    pub device_armed: bool,
    /// The pen position is available.
    pub pen_valid: bool,
    /// A fault is latched.
    pub fault: bool,
    /// When the current target arrived, if any.
    pub target_at: Option<Instant>,
    /// The pen button is held.
    pub button: bool,
}

/// The gate's decision for one tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GateOut {
    /// Why, as sent to the client.
    pub reason: Reason,
    /// Force scale in `[0, 1]`; zero unless [`Reason::Engaged`].
    pub ramp: f64,
    /// Force has just become allowed this tick: place the anchor at the pen.
    pub engaged_now: bool,
}

/// The gate and its latches.
#[derive(Clone, Debug)]
pub struct Gate {
    target_timeout: Duration,
    ramp_time: Duration,
    seen_released: bool,
    engaged_since: Option<Instant>,
}

impl Gate {
    /// A gate with the given target timeout and engage ramp time.
    pub fn new(target_timeout: Duration, ramp_time: Duration) -> Self {
        Self {
            target_timeout,
            ramp_time,
            seen_released: false,
            engaged_since: None,
        }
    }

    /// Whether the deadman latch has seen a release.
    pub fn seen_released(&self) -> bool {
        self.seen_released
    }

    fn off(&mut self, reason: Reason, clear_latch: bool) -> GateOut {
        if clear_latch {
            self.seen_released = false;
        }
        self.engaged_since = None;
        GateOut {
            reason,
            ramp: 0.,
            engaged_now: false,
        }
    }

    /// Decide this tick.
    pub fn evaluate(&mut self, i: GateIn) -> GateOut {
        if !i.link_up {
            return self.off(Reason::NoLink, true);
        }
        if !i.arm_wanted {
            return self.off(Reason::Disarmed, true);
        }
        if i.fault {
            return self.off(Reason::Fault, true);
        }
        if !i.device_armed {
            return self.off(Reason::DeviceNotArmed, true);
        }
        if !i.pen_valid {
            return self.off(Reason::NotRegistered, true);
        }
        // The deadman latch is tracked before the target checks, so a release while
        // nothing is held still counts.
        if !i.button {
            self.seen_released = true;
            return self.off(Reason::DeadmanReleased, false);
        }
        if !self.seen_released {
            return self.off(Reason::AwaitRelease, false);
        }
        let Some(at) = i.target_at else {
            return self.off(Reason::NoTarget, false);
        };
        if i.now.saturating_duration_since(at) > self.target_timeout {
            return self.off(Reason::TargetStale, false);
        }
        let engaged_now = self.engaged_since.is_none();
        let since = *self.engaged_since.get_or_insert(i.now);
        let ramp = if self.ramp_time.is_zero() {
            1.
        } else {
            (i.now.saturating_duration_since(since).as_secs_f64() / self.ramp_time.as_secs_f64())
                .clamp(0., 1.)
        };
        GateOut {
            reason: Reason::Engaged,
            ramp,
            engaged_now,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TIMEOUT: Duration = Duration::from_millis(150);
    const RAMP: Duration = Duration::from_millis(200);

    fn ready(now: Instant) -> GateIn {
        GateIn {
            now,
            link_up: true,
            arm_wanted: true,
            device_armed: true,
            pen_valid: true,
            fault: false,
            target_at: Some(now),
            button: true,
        }
    }

    /// A gate that has seen a release and is engaged at `t`.
    fn engaged(t: Instant) -> Gate {
        let mut g = Gate::new(TIMEOUT, RAMP);
        g.evaluate(GateIn {
            button: false,
            ..ready(t)
        });
        assert_eq!(g.evaluate(ready(t)).reason, Reason::Engaged);
        g
    }

    #[test]
    fn a_held_button_never_seen_released_gives_no_force() {
        let t = Instant::now();
        let mut g = Gate::new(TIMEOUT, RAMP);
        for ms in 0..1000 {
            let o = g.evaluate(ready(t + Duration::from_millis(ms)));
            assert_eq!(o.reason, Reason::AwaitRelease);
            assert_eq!(o.ramp, 0.);
        }
    }

    #[test]
    fn release_then_press_engages_with_a_ramp_from_zero() {
        let t = Instant::now();
        let mut g = Gate::new(TIMEOUT, RAMP);
        let o = g.evaluate(GateIn {
            button: false,
            ..ready(t)
        });
        assert_eq!(o.reason, Reason::DeadmanReleased);
        let o = g.evaluate(ready(t));
        assert_eq!(o.reason, Reason::Engaged);
        assert!(o.engaged_now);
        assert_eq!(o.ramp, 0., "first engaged tick has zero force");
        let later = t + Duration::from_millis(100);
        let o = g.evaluate(GateIn {
            target_at: Some(later),
            ..ready(later)
        });
        assert!(!o.engaged_now);
        assert!((o.ramp - 0.5).abs() < 1e-9);
        let later = t + Duration::from_millis(300);
        let o = g.evaluate(GateIn {
            target_at: Some(later),
            ..ready(later)
        });
        assert_eq!(o.ramp, 1.);
    }

    #[test]
    fn releasing_the_button_cuts_force_and_repressing_ramps_again() {
        let t = Instant::now();
        let mut g = engaged(t);
        let o = g.evaluate(GateIn {
            button: false,
            ..ready(t)
        });
        assert_eq!((o.reason, o.ramp), (Reason::DeadmanReleased, 0.));
        let o = g.evaluate(ready(t));
        assert!(o.engaged_now && o.ramp == 0.);
    }

    #[test]
    fn a_stale_target_cuts_force_at_the_timeout() {
        let t = Instant::now();
        let mut g = engaged(t);
        let at_limit = GateIn {
            target_at: Some(t),
            ..ready(t + TIMEOUT)
        };
        assert_eq!(g.evaluate(at_limit).reason, Reason::Engaged);
        let past = GateIn {
            target_at: Some(t),
            ..ready(t + TIMEOUT + Duration::from_millis(1))
        };
        let o = g.evaluate(past);
        assert_eq!((o.reason, o.ramp), (Reason::TargetStale, 0.));
        // A fresh target re-engages without a new release (the hand never let go)…
        let fresh = t + Duration::from_millis(400);
        let o = g.evaluate(ready(fresh));
        assert_eq!(o.reason, Reason::Engaged);
        // …but from a zero ramp.
        assert!(o.engaged_now && o.ramp == 0.);
    }

    #[test]
    fn no_target_gives_no_force() {
        let t = Instant::now();
        let mut g = engaged(t);
        let o = g.evaluate(GateIn {
            target_at: None,
            ..ready(t)
        });
        assert_eq!((o.reason, o.ramp), (Reason::NoTarget, 0.));
    }

    #[test]
    fn link_loss_disarm_fault_and_device_loss_each_cut_force_and_clear_the_latch() {
        let t = Instant::now();
        type Break = fn(GateIn) -> GateIn;
        let cases: [(Break, Reason); 5] = [
            (
                |i| GateIn {
                    link_up: false,
                    ..i
                },
                Reason::NoLink,
            ),
            (
                |i| GateIn {
                    arm_wanted: false,
                    ..i
                },
                Reason::Disarmed,
            ),
            (|i| GateIn { fault: true, ..i }, Reason::Fault),
            (
                |i| GateIn {
                    device_armed: false,
                    ..i
                },
                Reason::DeviceNotArmed,
            ),
            (
                |i| GateIn {
                    pen_valid: false,
                    ..i
                },
                Reason::NotRegistered,
            ),
        ];
        for (break_it, reason) in cases {
            let mut g = engaged(t);
            let o = g.evaluate(break_it(ready(t)));
            assert_eq!((o.reason, o.ramp), (reason, 0.));
            assert!(!g.seen_released(), "{reason:?} clears the deadman latch");
            // Restored, with the button still held: no force until a release.
            assert_eq!(g.evaluate(ready(t)).reason, Reason::AwaitRelease);
        }
    }

    #[test]
    fn a_release_while_nothing_is_held_still_counts() {
        let t = Instant::now();
        let mut g = Gate::new(TIMEOUT, RAMP);
        g.evaluate(GateIn {
            button: false,
            target_at: None,
            ..ready(t)
        });
        assert!(g.seen_released());
        assert_eq!(g.evaluate(ready(t)).reason, Reason::Engaged);
    }

    #[test]
    fn a_target_from_the_future_is_not_stale_and_time_going_backwards_is_safe() {
        let t = Instant::now();
        let mut g = engaged(t);
        let o = g.evaluate(GateIn {
            target_at: Some(t + Duration::from_secs(1)),
            ..ready(t)
        });
        assert_eq!(o.reason, Reason::Engaged);
        assert!((0. ..=1.).contains(&o.ramp));
    }
}
