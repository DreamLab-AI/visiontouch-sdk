//! A simulated unit for tests and client development. It never opens hardware.
//!
//! [`FakeUnit`] behaves like `visiontouch_device::unit::Unit` as the bridge sees it:
//! it refuses to arm unregistered, reports armed only after [`Haptic::arm`], and
//! records every force it is sent. The pen does not move by itself; tests and the
//! binary's `--fake` mode move it through [`FakeUnit::update`]. Clones share state.
use crate::servo::Haptic;
use std::io;
use std::sync::{Arc, Mutex};
use visiontouch_device::unit::{PEAK_N, State};

/// The simulated unit's state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FakeState {
    /// Pen position, metres, model frame.
    pub pen: [f64; 3],
    /// Pen button held.
    pub button: bool,
    /// Registered (pen position is reported).
    pub registered: bool,
    /// Output armed.
    pub armed: bool,
    /// The last force sent with [`Haptic::set_force`].
    pub force: [f64; 3],
    /// The largest force magnitude delivered while armed, N.
    pub max_force_n: f64,
    /// Calls to [`Haptic::arm`].
    pub arm_calls: u32,
    /// Calls to [`Haptic::disarm`].
    pub disarm_calls: u32,
    /// Make [`Haptic::arm`] fail.
    pub fail_arm: bool,
    /// Make [`Haptic::service`] fail.
    pub fail_service: bool,
}

/// A simulated unit. Clones share one [`FakeState`].
#[derive(Clone, Debug, Default)]
pub struct FakeUnit {
    state: Arc<Mutex<FakeState>>,
}

impl FakeUnit {
    /// A registered unit, pen at the origin, button released, output off.
    pub fn registered() -> Self {
        let f = Self::default();
        f.update(|s| s.registered = true);
        f
    }
    /// Change the state.
    pub fn update(&self, change: impl FnOnce(&mut FakeState)) {
        let mut g = self.state.lock().unwrap_or_else(|e| e.into_inner());
        change(&mut g);
    }
    /// A copy of the state.
    pub fn snapshot(&self) -> FakeState {
        *self.state.lock().unwrap_or_else(|e| e.into_inner())
    }
    /// Press or release the pen button.
    pub fn set_button(&self, held: bool) {
        self.update(|s| s.button = held);
    }
    /// Move the pen.
    pub fn set_pen(&self, p: [f64; 3]) {
        self.update(|s| s.pen = p);
    }
}

impl Haptic for FakeUnit {
    fn arm(&mut self) -> io::Result<()> {
        let mut s = self.state.lock().unwrap_or_else(|e| e.into_inner());
        s.arm_calls += 1;
        if !s.registered {
            return Err(io::Error::other("register with the pen in its seat first"));
        }
        if s.fail_arm {
            return Err(io::Error::other("simulated arm failure"));
        }
        s.force = [0.; 3];
        s.armed = true;
        Ok(())
    }
    fn disarm(&mut self) -> io::Result<()> {
        let mut s = self.state.lock().unwrap_or_else(|e| e.into_inner());
        s.disarm_calls += 1;
        s.force = [0.; 3];
        s.armed = false;
        Ok(())
    }
    fn set_force(&mut self, force_n: [f64; 3]) {
        self.update(|s| s.force = force_n);
    }
    fn service(&mut self) -> io::Result<State> {
        let mut s = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if s.fail_service {
            s.armed = false;
            s.force = [0.; 3];
            return Err(io::Error::other("simulated transport fault"));
        }
        let applied_n = if s.armed {
            let m = s.force.iter().map(|x| x * x).sum::<f64>().sqrt();
            if m.is_finite() { m.min(PEAK_N) } else { 0. }
        } else {
            0.
        };
        s.max_force_n = s.max_force_n.max(applied_n);
        Ok(State {
            position_m: s.registered.then_some(s.pen),
            button: s.button,
            applied_n,
            heat_n2s: 0.,
            heat_limited: false,
            armed: s.armed,
        })
    }
}
