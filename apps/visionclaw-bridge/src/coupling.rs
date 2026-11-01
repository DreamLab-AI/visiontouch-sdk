//! The local virtual coupling: target smoothing and the spring-damper force.
//!
//! Everything here is pure and runs on the servo thread at the device rate (about
//! 400 Hz). The network only moves the *target*; the force is computed here from the
//! pen measured on this tick, so network jitter changes where the spring is anchored,
//! never how hard the motors push from one tick to the next.
//!
//! The coupling is `F = -k·(pen − target) − c·(v_pen − v_target)`, with the damping
//! term capped on its own and the total capped at the configured force cap. Targets
//! arrive at 10–30 Hz; [`TargetFilter`] turns those steps into a smooth path with a
//! workspace clamp, a slew limit and a first-order low-pass, so the anchor never jumps.
//! [`Coupling::force`] then multiplies by an engage ramp so force grows from zero when
//! the gate opens.

/// A 3-vector in metres (or newtons, or metres per second).
pub type V3 = [f64; 3];

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn norm(a: V3) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}
fn finite(a: V3) -> bool {
    a.iter().all(|x| x.is_finite())
}

/// Scale `a` down uniformly so its magnitude is at most `cap` (direction kept).
/// A non-positive or non-finite cap yields zero.
pub fn limit(a: V3, cap: f64) -> V3 {
    if !(cap.is_finite() && cap > 0.) {
        return [0.; 3];
    }
    let n = norm(a);
    if n > cap { scale(a, cap / n) } else { a }
}

/// Spring-damper gains and caps for the coupling, in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coupling {
    /// Spring stiffness, N/m.
    pub stiffness_n_m: f64,
    /// Damping on the pen's velocity relative to the target, N·s/m.
    pub damping_n_s_m: f64,
    /// Cap on the damping term alone, N.
    pub damping_cap_n: f64,
    /// Cap on the total force magnitude, N. The SDK's heat budget applies on top.
    pub force_cap_n: f64,
}

impl Coupling {
    /// The coupling force for a pen at `pen` moving at `pen_v`, anchored to a target at
    /// `target` moving at `target_v`, scaled by the engage `ramp` in `[0, 1]`.
    ///
    /// Returns `None` if any input or the result is not finite: the caller must send
    /// zero force and treat it as a fault.
    pub fn force(&self, pen: V3, pen_v: V3, target: V3, target_v: V3, ramp: f64) -> Option<V3> {
        if !(finite(pen) && finite(pen_v) && finite(target) && finite(target_v)) {
            return None;
        }
        if !ramp.is_finite() {
            return None;
        }
        let ramp = ramp.clamp(0., 1.);
        let spring = scale(sub(pen, target), -self.stiffness_n_m);
        let damper = limit(
            scale(sub(pen_v, target_v), -self.damping_n_s_m),
            self.damping_cap_n,
        );
        let total = limit(add(spring, damper), self.force_cap_n);
        let f = scale(total, ramp);
        finite(f).then_some(f)
    }
}

/// Smooths network targets into a continuous anchor path.
///
/// Each tick: clamp the raw target into a sphere of `workspace_radius_m` about the
/// registered origin, move towards it no faster than `max_speed_m_s`, then low-pass
/// with time constant `tau_s`. The anchor's velocity is reported for the damper.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TargetFilter {
    /// Radius of the sphere, about the registered origin, that targets are clamped to, m.
    pub workspace_radius_m: f64,
    /// Fastest the slewed anchor may move, m/s.
    pub max_speed_m_s: f64,
    /// Low-pass time constant, s.
    pub tau_s: f64,
    slewed: Option<V3>,
    smoothed: Option<V3>,
    velocity: V3,
}

impl TargetFilter {
    /// A filter with no state; the first [`TargetFilter::reset`] places the anchor.
    pub fn new(workspace_radius_m: f64, max_speed_m_s: f64, tau_s: f64) -> Self {
        Self {
            workspace_radius_m,
            max_speed_m_s,
            tau_s,
            slewed: None,
            smoothed: None,
            velocity: [0.; 3],
        }
    }

    /// Place the anchor at `at` with zero velocity. Called when force engages, with the
    /// pen position, so the spring starts relaxed and the anchor then travels to the
    /// target at the slew limit.
    pub fn reset(&mut self, at: V3) {
        let at = self.clamp(at);
        self.slewed = Some(at);
        self.smoothed = Some(at);
        self.velocity = [0.; 3];
    }

    /// Forget the anchor (force is off).
    pub fn clear(&mut self) {
        self.slewed = None;
        self.smoothed = None;
        self.velocity = [0.; 3];
    }

    /// The current anchor and its velocity, if placed.
    pub fn anchor(&self) -> Option<(V3, V3)> {
        self.smoothed.map(|p| (p, self.velocity))
    }

    fn clamp(&self, p: V3) -> V3 {
        limit(p, self.workspace_radius_m)
    }

    /// Advance by `dt_s` towards `raw`. Returns the anchor and its velocity, or `None`
    /// if the filter has not been placed or `raw` is not finite.
    pub fn step(&mut self, raw: V3, dt_s: f64) -> Option<(V3, V3)> {
        if !finite(raw) || !(dt_s.is_finite() && dt_s >= 0.) {
            return None;
        }
        let (slewed, smoothed) = (self.slewed?, self.smoothed?);
        let goal = self.clamp(raw);
        let step = limit(sub(goal, slewed), self.max_speed_m_s * dt_s);
        let slewed = add(slewed, step);
        let alpha = if self.tau_s > 0. {
            1. - (-dt_s / self.tau_s).exp()
        } else {
            1.
        };
        let next = add(smoothed, scale(sub(slewed, smoothed), alpha));
        self.velocity = if dt_s > 0. {
            scale(sub(next, smoothed), 1. / dt_s)
        } else {
            self.velocity
        };
        self.slewed = Some(slewed);
        self.smoothed = Some(next);
        Some((next, self.velocity))
    }
}

/// First-order filtered velocity from successive positions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelocityFilter {
    /// Low-pass time constant, s.
    pub tau_s: f64,
    last: Option<V3>,
    velocity: V3,
}

impl VelocityFilter {
    /// A filter with no history.
    pub fn new(tau_s: f64) -> Self {
        Self {
            tau_s,
            last: None,
            velocity: [0.; 3],
        }
    }
    /// Forget history (the pen position became unavailable).
    pub fn clear(&mut self) {
        self.last = None;
        self.velocity = [0.; 3];
    }
    /// Feed a position `dt_s` after the previous one and return the filtered velocity.
    /// The first sample, a zero interval or a non-finite input gives the previous value.
    pub fn step(&mut self, p: V3, dt_s: f64) -> V3 {
        if !finite(p) {
            return self.velocity;
        }
        if let Some(last) = self.last
            && dt_s.is_finite()
            && dt_s > 0.
        {
            let raw = scale(sub(p, last), 1. / dt_s);
            let alpha = if self.tau_s > 0. {
                1. - (-dt_s / self.tau_s).exp()
            } else {
                1.
            };
            self.velocity = add(self.velocity, scale(sub(raw, self.velocity), alpha));
        }
        self.last = Some(p);
        self.velocity
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn coupling() -> Coupling {
        Coupling {
            stiffness_n_m: 400.,
            damping_n_s_m: 2.,
            damping_cap_n: 0.5,
            force_cap_n: 1.4,
        }
    }

    fn close(a: V3, b: V3) -> bool {
        norm(sub(a, b)) < 1e-9
    }

    #[test]
    fn zero_force_at_the_target() {
        let f = coupling().force([0.01, 0.02, 0.], [0.; 3], [0.01, 0.02, 0.], [0.; 3], 1.);
        assert!(close(f.unwrap(), [0.; 3]));
    }

    #[test]
    fn hookes_law_below_the_cap_pulls_towards_the_target() {
        // 1 mm off in +x at 400 N/m is 0.4 N back towards the target.
        let f = coupling().force([0.001, 0., 0.], [0.; 3], [0.; 3], [0.; 3], 1.);
        assert!(close(f.unwrap(), [-0.4, 0., 0.]));
    }

    #[test]
    fn the_total_is_capped_with_its_direction_kept() {
        let f = coupling()
            .force([0.03, 0.04, 0.], [0.; 3], [0.; 3], [0.; 3], 1.)
            .unwrap();
        assert!((norm(f) - 1.4).abs() < 1e-9);
        assert!(close(scale(f, 1. / 1.4), [-0.6, -0.8, 0.]));
    }

    #[test]
    fn damping_opposes_relative_velocity_and_is_capped_alone() {
        let c = coupling();
        let f = c
            .force([0.; 3], [0.1, 0., 0.], [0.; 3], [0.; 3], 1.)
            .unwrap();
        assert!(close(f, [-0.2, 0., 0.]));
        // Moving with the target: no damping.
        let f = c
            .force([0.; 3], [0.1, 0., 0.], [0.; 3], [0.1, 0., 0.], 1.)
            .unwrap();
        assert!(close(f, [0.; 3]));
        // A fast flick is capped at the damping cap, not the total cap.
        let f = c
            .force([0.; 3], [5., 0., 0.], [0.; 3], [0.; 3], 1.)
            .unwrap();
        assert!(close(f, [-0.5, 0., 0.]));
    }

    #[test]
    fn the_ramp_scales_and_is_clamped() {
        let c = coupling();
        let full = c
            .force([0.001, 0., 0.], [0.; 3], [0.; 3], [0.; 3], 1.)
            .unwrap();
        let half = c
            .force([0.001, 0., 0.], [0.; 3], [0.; 3], [0.; 3], 0.5)
            .unwrap();
        assert!(close(half, scale(full, 0.5)));
        assert!(close(
            c.force([0.001, 0., 0.], [0.; 3], [0.; 3], [0.; 3], 7.)
                .unwrap(),
            full
        ));
        assert!(close(
            c.force([0.001, 0., 0.], [0.; 3], [0.; 3], [0.; 3], -1.)
                .unwrap(),
            [0.; 3]
        ));
    }

    #[test]
    fn non_finite_input_is_refused() {
        let c = coupling();
        assert!(
            c.force([f64::NAN, 0., 0.], [0.; 3], [0.; 3], [0.; 3], 1.)
                .is_none()
        );
        assert!(
            c.force([0.; 3], [0.; 3], [f64::INFINITY, 0., 0.], [0.; 3], 1.)
                .is_none()
        );
        assert!(
            c.force([0.; 3], [0.; 3], [0.; 3], [0.; 3], f64::NAN)
                .is_none()
        );
    }

    #[test]
    fn limit_handles_bad_caps() {
        assert_eq!(limit([1., 0., 0.], 0.), [0.; 3]);
        assert_eq!(limit([1., 0., 0.], f64::NAN), [0.; 3]);
        assert_eq!(limit([0.5, 0., 0.], 1.), [0.5, 0., 0.]);
    }

    #[test]
    fn the_target_filter_starts_where_it_is_placed_and_slews() {
        let mut f = TargetFilter::new(0.15, 0.25, 0.);
        assert!(f.step([0.; 3], 0.0025).is_none(), "unplaced");
        f.reset([0.; 3]);
        // A 30 mm jump at 0.25 m/s takes 120 ms; after one 2.5 ms tick it has moved 0.625 mm.
        let (p, v) = f.step([0.03, 0., 0.], 0.0025).unwrap();
        assert!((p[0] - 0.000625).abs() < 1e-12);
        assert!((v[0] - 0.25).abs() < 1e-9);
        let mut last = p;
        for _ in 0..47 {
            last = f.step([0.03, 0., 0.], 0.0025).unwrap().0;
        }
        assert!(
            (last[0] - 0.03).abs() < 1e-9,
            "arrives after 48 ticks: {last:?}"
        );
    }

    #[test]
    fn the_target_filter_clamps_to_the_workspace() {
        let mut f = TargetFilter::new(0.1, 100., 0.);
        f.reset([0.; 3]);
        let (p, _) = f.step([1., 0., 0.], 0.01).unwrap();
        assert!((norm(p) - 0.1).abs() < 1e-12);
        f.reset([0., 5., 0.]);
        assert!((norm(f.anchor().unwrap().0) - 0.1).abs() < 1e-12);
    }

    #[test]
    fn the_low_pass_interpolates_a_10_hz_staircase_smoothly() {
        // Targets step 5 mm every 100 ms (a 10 Hz stream); the anchor's per-tick motion
        // never exceeds the slew limit and it tracks the ramp within a few mm.
        let mut f = TargetFilter::new(0.15, 0.25, 0.03);
        f.reset([0.; 3]);
        let dt = 0.0025;
        let mut prev = 0.;
        let mut max_step: f64 = 0.;
        for tick in 0..400 {
            let raw = 0.005 * ((tick / 40) as f64);
            let (p, _) = f.step([raw, 0., 0.], dt).unwrap();
            max_step = max_step.max((p[0] - prev).abs());
            prev = p[0];
            if tick > 80 {
                assert!((p[0] - raw).abs() < 0.008, "tracks: {} vs {raw}", p[0]);
            }
        }
        assert!(max_step <= 0.25 * dt + 1e-12);
        assert!(f.step([f64::NAN, 0., 0.], dt).is_none());
    }

    #[test]
    fn the_velocity_filter_estimates_steady_motion() {
        let mut v = VelocityFilter::new(0.01);
        let mut out = [0.; 3];
        for i in 0..400 {
            out = v.step([0.1 * i as f64 * 0.0025, 0., 0.], 0.0025);
        }
        assert!((out[0] - 0.1).abs() < 1e-6);
        assert_eq!(v.step([f64::NAN, 0., 0.], 0.0025), out);
        v.clear();
        assert_eq!(v.step([1., 1., 1.], 0.0025), [0.; 3]);
    }
}
