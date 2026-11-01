//! Deadzone, per-axis sensitivity and clamping: raw counts to normalised motion.
//!
//! For each axis the spacenavd count is divided by [`Settings::deflection_max`] and clamped to
//! `[-1, 1]`. Inside the deadzone it is zero; outside, the remainder is rescaled so the
//! output rises smoothly from zero at the edge of the deadzone to one at full
//! deflection, then multiplied by the axis's sensitivity and clamped to `[-1, 1]`
//! again. A negative sensitivity inverts an axis.
/// Number of motion axes: `x, y, z, rx, ry, rz`.
pub const AXES: usize = 6;

/// The default for [`Settings::deflection_max`]. spacenavd does not normalise: its
/// output range follows its own sensitivity setting, so this is a starting value to
/// tune against the running daemon, not a property of any device.
pub const DEFAULT_DEFLECTION_MAX: f64 = 350.;

/// Filter settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// Deadzone as a fraction of full scale, in `[0, 0.9]`.
    pub deadzone: f64,
    /// Multiplier per axis (`x, y, z, rx, ry, rz`); magnitude in `[0.01, 10]`, sign
    /// inverts the axis.
    pub sensitivity: [f64; AXES],
    /// The spacenavd count that maps to full deflection (1.0), in `[1, 1000000]`.
    pub deflection_max: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            deadzone: 0.05,
            sensitivity: [1.; AXES],
            deflection_max: DEFAULT_DEFLECTION_MAX,
        }
    }
}

/// Motion and buttons after filtering.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Nav {
    /// `x, y, z, rx, ry, rz`, each in `[-1, 1]`, in the device's own sign convention.
    pub axes: [f64; AXES],
    /// The button bitmask.
    pub buttons: u32,
}

impl Nav {
    /// Every axis is exactly zero (buttons are not motion).
    pub fn is_still(&self) -> bool {
        self.axes.iter().all(|a| *a == 0.)
    }
}

impl Settings {
    /// Check every field against its documented range.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.deadzone.is_finite() && (0. ..=0.9).contains(&self.deadzone)) {
            return Err(format!("deadzone {} must be in 0..=0.9", self.deadzone));
        }
        if !(self.deflection_max.is_finite() && (1. ..=1_000_000.).contains(&self.deflection_max)) {
            return Err(format!(
                "deflection_max {} must be in 1..=1000000",
                self.deflection_max
            ));
        }
        for (i, s) in self.sensitivity.iter().enumerate() {
            if !(s.is_finite() && (0.01..=10.).contains(&s.abs())) {
                return Err(format!(
                    "sensitivity[{i}] {s} must have a magnitude in 0.01..=10"
                ));
            }
        }
        Ok(())
    }

    /// Filter spacenavd axes and buttons. Settings are assumed valid (see
    /// [`Settings::validate`]); the output is always finite and within `[-1, 1]`.
    pub fn apply(&self, raw: &[i32; AXES], buttons: u32) -> Nav {
        let mut axes = [0.; AXES];
        for ((out, count), gain) in axes.iter_mut().zip(raw).zip(self.sensitivity) {
            let n = (f64::from(*count) / self.deflection_max).clamp(-1., 1.);
            let m = n.abs();
            if m <= self.deadzone {
                continue;
            }
            let shaped = (m - self.deadzone) / (1. - self.deadzone);
            *out = (n.signum() * shaped * gain).clamp(-1., 1.);
        }
        Nav { axes, buttons }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raw(x: i32) -> [i32; AXES] {
        [x, 0, 0, 0, 0, 0]
    }

    #[test]
    fn inside_the_deadzone_is_exactly_zero() {
        let s = Settings::default();
        // 5 % of 350 = 17.5: 17 is inside, 18 is out.
        assert_eq!(s.apply(&raw(17), 0).axes[0], 0.);
        assert_eq!(s.apply(&raw(-17), 0).axes[0], 0.);
        assert!(s.apply(&raw(18), 0).axes[0] > 0.);
        assert!(
            s.apply(&raw(18), 0).axes[0] < 0.01,
            "rises from zero, no jump"
        );
    }

    #[test]
    fn full_deflection_is_one_and_overrange_is_clamped() {
        let s = Settings::default();
        assert_eq!(s.apply(&raw(350), 0).axes[0], 1.);
        assert_eq!(s.apply(&raw(-350), 0).axes[0], -1.);
        assert_eq!(s.apply(&raw(i32::MAX), 0).axes[0], 1.);
        assert_eq!(s.apply(&raw(i32::MIN), 0).axes[0], -1.);
    }

    #[test]
    fn sensitivity_is_per_axis_and_a_negative_sign_inverts() {
        let s = Settings {
            deadzone: 0.,
            sensitivity: [0.5, -1., 2., 1., 1., 1.],
            deflection_max: 100.,
        };
        let n = s.apply(&[100, 100, 30, 0, 0, 0], 0);
        assert_eq!(n.axes[0], 0.5);
        assert_eq!(n.axes[1], -1.);
        assert!((n.axes[2] - 0.6).abs() < 1e-12);
        assert_eq!(n.axes[3], 0.);
    }

    #[test]
    fn gain_cannot_push_the_output_past_one() {
        let s = Settings {
            sensitivity: [10.; 6],
            ..Settings::default()
        };
        assert_eq!(s.apply(&raw(300), 0).axes[0], 1.);
    }

    #[test]
    fn buttons_pass_through_and_are_not_motion() {
        let n = Settings::default().apply(&[0; 6], 5);
        assert_eq!(n.buttons, 5);
        assert!(n.is_still());
    }

    #[test]
    fn validation_refuses_out_of_range_settings() {
        assert!(Settings::default().validate().is_ok());
        for bad in [
            Settings {
                deadzone: -0.1,
                ..Settings::default()
            },
            Settings {
                deadzone: 0.95,
                ..Settings::default()
            },
            Settings {
                deadzone: f64::NAN,
                ..Settings::default()
            },
            Settings {
                deflection_max: 0.,
                ..Settings::default()
            },
            Settings {
                deflection_max: 2_000_000.,
                ..Settings::default()
            },
            Settings {
                sensitivity: [1., 1., 0., 1., 1., 1.],
                ..Settings::default()
            },
            Settings {
                sensitivity: [1., 1., 1., 11., 1., 1.],
                ..Settings::default()
            },
        ] {
            assert!(bad.validate().is_err(), "{bad:?}");
        }
    }
}
