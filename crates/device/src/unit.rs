//! Single-unit profile for lab integration.
//!
//! This module is specific to the owner's identified unit. It replaces the staged
//! verification ladder with one registration click and full output, using that unit's
//! **measured** gain rather than the reference model's newtons, and a heat
//! budget derived from the published ratings.
//!
//! # Lifecycle
//!
//! 1. [`Unit::connect`] opens the port with output off.
//! 2. With the pen in its seat (the reset pose), call [`Unit::register_here`]. Do this
//!    once per connection: saved counts cannot reproduce a reference.
//! 3. [`Unit::arm`] runs the zero-only handshake and opens a full-span session.
//! 4. Every loop iteration (about 2.4 ms), set the force with [`Unit::set_force`]
//!    and call [`Unit::service`]. Never stop calling `service` while armed: a request
//!    older than 25 ms, or a gap that long, yields zero output.
//! 5. [`Unit::disarm`] or [`Unit::close`] when done. Drop also attempts shutdown.
//!
//! **The caller supplies the deadman.** The profile has no built-in operator hold;
//! the held pen button ([`State::button`]) is the recommended one. Keep virtual
//! stiffness near [`STABLE_STIFFNESS_N_MM`]: this loop runs at about 400 Hz and the
//! force lands two samples after it is computed, so stiffer walls bounce.
//!
//! ```no_run
//! use visiontouch_device::unit::Unit;
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut unit = Unit::connect("/dev/parport0")?;
//! // ... with the pen seated at the reset pose:
//! unit.register_here()?;
//! unit.arm()?;
//! loop {
//!     let state = unit.service()?;
//!     let Some(p) = state.position_m else { break };
//!     // A 2 N/mm floor at y = -30 mm, live only while the pen button is held.
//!     let depth = -0.030 - p[1];
//!     let f = if state.button && depth > 0. { 2000. * depth } else { 0. };
//!     unit.set_force([0., f, 0.]);
//! #   break;
//! }
//! unit.close()?;
//! # Ok(()) }
//! ```
use crate::{Demand, Device, MAX_CHARACTERISE_OFFSET, motor_demand_counts};
use std::io;
use std::path::Path;
use std::time::Instant;

/// Delivered newtons per count on the largest motor channel: the settled 542 g at
/// 2047 counts pushing down. About 0.0026 N per count.
pub const N_PER_COUNT: f64 = 0.542 * 9.80665 / 2047.;
/// Output ceiling of the profile, counts on the largest channel: the full half-span.
pub const PEAK_COUNTS: i16 = MAX_CHARACTERISE_OFFSET;
/// Delivered force at [`PEAK_COUNTS`], newtons (about 5.3 N).
pub const PEAK_N: f64 = PEAK_COUNTS as f64 * N_PER_COUNT;
/// Output slew of the profile's session, counts per second (about 120 per tick).
pub const SLEW_COUNTS_S: f64 = 50_000.;
/// Virtual wall stiffness found stable on this unit and loop, delivered N/mm.
pub const STABLE_STIFFNESS_N_MM: f64 = 2.;
/// Heat model decay time constant, seconds.
pub const HEAT_TAU_S: f64 = 600.;
/// Heat budget, delivered N²s: half the 2 N, 600 s hold that raised the casing 34.5 C
/// with the stock amplifier.
pub const HEAT_BUDGET_N2S: f64 = 1200.;
/// The force that holds the heat total exactly at the budget, newtons: about 1.41 N,
/// the published continuous rating. Output is capped here once the budget
/// is spent.
pub const CONTINUOUS_N: f64 = std::f64::consts::SQRT_2; // sqrt(HEAT_BUDGET_N2S / HEAT_TAU_S)
const _: () = assert!(CONTINUOUS_N * CONTINUOUS_N * HEAT_TAU_S <= HEAT_BUDGET_N2S + 1e-6);

/// Leaky heat accumulator: force squared over time, decaying with [`HEAT_TAU_S`].
///
/// Below [`CONTINUOUS_N`] the total never reaches the budget, so low continuous force
/// is unlimited. Any force up to [`PEAK_N`] is allowed until the total reaches
/// [`HEAT_BUDGET_N2S`] (about 40 s at full force from cool); then output is capped at
/// [`CONTINUOUS_N`] until it cools below the budget. This is a model from published
/// ratings, not a temperature measurement.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Heat {
    total_n2s: f64,
    last: Option<Instant>,
}
impl Heat {
    /// Add `delivered_n` held since the previous update, after decaying to `now`.
    /// Non-finite or negative force counts as zero.
    pub fn update(&mut self, now: Instant, delivered_n: f64) {
        let f = if delivered_n.is_finite() {
            delivered_n.max(0.)
        } else {
            0.
        };
        if let Some(last) = self.last {
            let dt = now.saturating_duration_since(last).as_secs_f64();
            let decay = (-dt / HEAT_TAU_S).exp();
            // Exact for a force held constant over the interval.
            self.total_n2s = self.total_n2s * decay + f * f * HEAT_TAU_S * (1. - decay);
        }
        self.last = Some(now);
    }
    /// The heat total, N²s.
    pub fn total_n2s(&self) -> f64 {
        self.total_n2s
    }
    /// Whether the budget is spent and output is held at [`CONTINUOUS_N`].
    pub fn limiting(&self) -> bool {
        self.total_n2s >= HEAT_BUDGET_N2S
    }
    /// The largest force magnitude allowed now, newtons.
    pub fn allowed_n(&self) -> f64 {
        if self.limiting() {
            CONTINUOUS_N
        } else {
            PEAK_N
        }
    }
}

/// What one [`Unit::service`] call observed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct State {
    /// Pen position in metres relative to the registered reset pose, model frame;
    /// `None` until registered.
    pub position_m: Option<[f64; 3]>,
    /// Pen button held (active-low decode; a wiring fault reads as pressed).
    pub button: bool,
    /// Delivered-force estimate of the output just sent, newtons.
    pub applied_n: f64,
    /// Heat total after this sample, N²s.
    pub heat_n2s: f64,
    /// The heat budget is capping output at [`CONTINUOUS_N`].
    pub heat_limited: bool,
    /// Output is armed.
    pub armed: bool,
}

/// The demand that delivers `force_n` newtons, limited to `allowed_n`. Pure: the
/// direction is mapped through the Jacobian by the guard at registered counts.
pub fn demand_for(force_n: [f64; 3], allowed_n: f64) -> Demand {
    let finite = force_n.iter().all(|x| x.is_finite());
    let magnitude = if finite {
        force_n.iter().map(|x| x * x).sum::<f64>().sqrt()
    } else {
        0.
    };
    let magnitude = magnitude.min(allowed_n.max(0.));
    Demand::Characterise {
        direction: if finite { force_n } else { [0.; 3] },
        counts: magnitude / N_PER_COUNT,
    }
}

/// The single-unit device: [`Device`] plus the measured profile and heat budget.
pub struct Unit {
    device: Device,
    heat: Heat,
    force_n: [f64; 3],
}
impl Unit {
    /// Open the port and take one sample, with output off.
    pub fn connect(path: impl AsRef<Path>) -> io::Result<Self> {
        let mut device = Device::connect(path)?;
        device.sample()?;
        Ok(Self {
            device,
            heat: Heat::default(),
            force_n: [0.; 3],
        })
    }
    /// Make the current pose the origin. Call with the pen in its seat. Turns output
    /// off; call [`Unit::arm`] again afterwards.
    pub fn register_here(&mut self) -> Result<(), &'static str> {
        self.force_n = [0.; 3];
        self.device.register_reset().map(|_| ())
    }
    /// Run the zero-only handshake and open the full-span session. Requires a
    /// registration on this connection.
    pub fn arm(&mut self) -> io::Result<()> {
        if self.device.state.registration().is_none() {
            return Err(io::Error::other("register with the pen in its seat first"));
        }
        self.force_n = [0.; 3];
        if self.device.feedback_enabled() {
            self.device.disable_feedback()?;
        }
        self.device
            .enable_characterisation(PEAK_COUNTS, SLEW_COUNTS_S)
    }
    /// Turn output off.
    pub fn disarm(&mut self) -> io::Result<()> {
        self.force_n = [0.; 3];
        self.device.disable_feedback()
    }
    /// Set the delivered force in newtons, model frame. Applied from the next
    /// [`Unit::service`]; non-finite input sends zero.
    pub fn set_force(&mut self, force_n: [f64; 3]) {
        self.force_n = force_n;
    }
    /// Queue the current force, read one sample, service the guard and update the
    /// heat model. Any error is a transport fault: output was shut down; close and
    /// reconnect.
    pub fn service(&mut self) -> io::Result<State> {
        let armed = self.device.feedback_enabled();
        if armed {
            let demand = demand_for(self.force_n, self.heat.allowed_n());
            self.device
                .request(demand, Instant::now())
                .map_err(|e| io::Error::other(format!("{e:?}")))?;
        }
        self.device.sample()?;
        let t = self.device.guard.telemetry();
        let largest = t
            .applied_motor_offsets
            .iter()
            .map(|x| x.unsigned_abs())
            .max()
            .unwrap_or(0);
        let applied_n = f64::from(largest) * N_PER_COUNT;
        self.heat.update(Instant::now(), applied_n);
        Ok(State {
            position_m: self.device.state.position_m(),
            button: self.device.state.button_pressed,
            applied_n,
            heat_n2s: self.heat.total_n2s(),
            heat_limited: self.heat.limiting(),
            armed: self.device.feedback_enabled(),
        })
    }
    /// The heat model.
    pub fn heat(&self) -> Heat {
        self.heat
    }
    /// Channel counts the current force would command at the registered pose, before
    /// guard limits; for display only.
    pub fn preview_counts(&self) -> Option<[f64; 3]> {
        let c = self.device.state.registered_counts()?;
        Some(motor_demand_counts(
            std::array::from_fn(|i| c[i] as f64),
            self.force_n,
        ))
    }
    /// The underlying device, for encoder state and telemetry.
    pub fn device(&self) -> &Device {
        &self.device
    }
    /// Shut output down and release the port.
    pub fn close(self) -> io::Result<()> {
        self.device.close()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn constants_match_the_measurement_and_ratings() {
        assert!((N_PER_COUNT - 0.002597).abs() < 1e-5);
        assert!((PEAK_N - 5.315).abs() < 0.01);
        assert!((CONTINUOUS_N - (HEAT_BUDGET_N2S / HEAT_TAU_S).sqrt()).abs() < 1e-12);
        assert!(
            (CONTINUOUS_N - 1.4).abs() < 0.02,
            "the published continuous rating"
        );
    }

    #[test]
    fn low_continuous_force_never_fills_the_budget() {
        let t = Instant::now();
        let mut h = Heat::default();
        for s in 0..=20_000 {
            h.update(t + Duration::from_secs(s), 1.0);
        }
        assert!(!h.limiting());
        assert!((h.total_n2s() - 600.).abs() < 1., "settles at F² τ");
        assert_eq!(h.allowed_n(), PEAK_N);
    }

    #[test]
    fn full_force_is_allowed_in_bursts_then_capped_and_recovers() {
        let t = Instant::now();
        let mut h = Heat::default();
        h.update(t, PEAK_N);
        let mut ms = 0;
        while !h.limiting() {
            ms += 10;
            h.update(t + Duration::from_millis(ms), PEAK_N);
            assert!(ms < 120_000);
        }
        let s = ms as f64 / 1000.;
        assert!(
            (38. ..48.).contains(&s),
            "about 40 s of full force from cool: {s}"
        );
        assert_eq!(h.allowed_n(), CONTINUOUS_N);
        // A lighter force lets it cool back below the budget.
        let mut now = t + Duration::from_millis(ms);
        for _ in 0..600 {
            now += Duration::from_secs(1);
            h.update(now, 0.5);
        }
        assert!(!h.limiting(), "cooling restores full force");
        assert_eq!(h.allowed_n(), PEAK_N);
    }

    #[test]
    fn demand_converts_delivered_newtons_and_respects_the_cap() {
        match demand_for([0., 1.0, 0.], PEAK_N) {
            Demand::Characterise { direction, counts } => {
                assert_eq!(direction, [0., 1., 0.]);
                assert!((counts - 1.0 / N_PER_COUNT).abs() < 1e-9);
            }
            other => panic!("{other:?}"),
        }
        match demand_for([3., 4., 0.], CONTINUOUS_N) {
            Demand::Characterise { counts, .. } => {
                assert!((counts * N_PER_COUNT - CONTINUOUS_N).abs() < 1e-9, "capped");
            }
            other => panic!("{other:?}"),
        }
        match demand_for([f64::NAN, 1., 0.], PEAK_N) {
            Demand::Characterise { counts, .. } => assert_eq!(counts, 0.),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn heat_ignores_bad_input_and_backwards_time() {
        let t = Instant::now();
        let mut h = Heat::default();
        h.update(t, f64::NAN);
        h.update(t + Duration::from_secs(1), -3.);
        assert_eq!(h.total_n2s(), 0.);
        h.update(t + Duration::from_secs(2), 2.);
        let before = h.total_n2s();
        h.update(t, 2.);
        assert!(h.total_n2s() >= before - 1e-12, "no negative intervals");
    }
}
