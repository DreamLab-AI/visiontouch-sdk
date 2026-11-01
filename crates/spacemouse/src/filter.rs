//! Deadzone, per-axis sensitivity and clamping: raw counts to normalised motion.
//!
//! For each axis the raw count is divided by [`Settings::full_scale`] and clamped to
//! `[-1, 1]`. Inside the deadzone it is zero; outside, the remainder is rescaled so the
//! output rises smoothly from zero at the edge of the deadzone to one at full
//! deflection, then multiplied by the axis's sensitivity and clamped to `[-1, 1]`
//! again. A negative sensitivity inverts an axis.
use crate::report::{AXES, RawState};

/// Full-scale deflection of the current models, raw counts. Public driver sources
/// describe about ±350; not verified on a device here, hence configurable.
pub const DEFAULT_FULL_SCALE: f64 = 350.;

/// Filter settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// Deadzone as a fraction of full scale, in `[0, 0.9]`.
    pub deadzone: f64,
    /// Multiplier per axis (`x, y, z, rx, ry, rz`); magnitude in `[0.01, 10]`, sign
    /// inverts the axis.
    pub sensitivity: [f64; AXES],
    /// The raw count that counts as full deflection, in `[1, 32767]`.
    pub full_scale: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            deadzone: 0.05,
            sensitivity: [1.; AXES],
            full_scale: DEFAULT_FULL_SCALE,
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
        if !(self.full_scale.is_finite() && (1. ..=32767.).contains(&self.full_scale)) {
            return Err(format!(
                "full_scale {} must be in 1..=32767",
                self.full_scale
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

    /// Filter a raw state. Settings are assumed valid (see [`Settings::validate`]);
    /// the output is always finite and within `[-1, 1]`.
    pub fn apply(&self, raw: &RawState) -> Nav {
        let mut axes = [0.; AXES];
        for ((out, count), gain) in axes.iter_mut().zip(raw.axes).zip(self.sensitivity) {
            let n = (f64::from(count) / self.full_scale).clamp(-1., 1.);
            let m = n.abs();
            if m <= self.deadzone {
                continue;
            }
            let shaped = (m - self.deadzone) / (1. - self.deadzone);
            *out = (n.signum() * shaped * gain).clamp(-1., 1.);
        }
        Nav {
            axes,
            buttons: raw.buttons,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raw(x: i16) -> RawState {
        RawState {
            axes: [x, 0, 0, 0, 0, 0],
            buttons: 0,
        }
    }

    #[test]
    fn inside_the_deadzone_is_exactly_zero() {
        let s = Settings::default();
        // 5 % of 350 = 17.5: 17 is inside, 18 is out.
        assert_eq!(s.apply(&raw(17)).axes[0], 0.);
        assert_eq!(s.apply(&raw(-17)).axes[0], 0.);
        assert!(s.apply(&raw(18)).axes[0] > 0.);
        assert!(s.apply(&raw(18)).axes[0] < 0.01, "rises from zero, no jump");
    }

    #[test]
    fn full_deflection_is_one_and_overrange_is_clamped() {
        let s = Settings::default();
        assert_eq!(s.apply(&raw(350)).axes[0], 1.);
        assert_eq!(s.apply(&raw(-350)).axes[0], -1.);
        assert_eq!(s.apply(&raw(i16::MAX)).axes[0], 1.);
        assert_eq!(s.apply(&raw(i16::MIN)).axes[0], -1.);
    }

    #[test]
    fn sensitivity_is_per_axis_and_a_negative_sign_inverts() {
        let s = Settings {
            deadzone: 0.,
            sensitivity: [0.5, -1., 2., 1., 1., 1.],
            full_scale: 100.,
        };
        let n = s.apply(&RawState {
            axes: [100, 100, 30, 0, 0, 0],
            buttons: 0,
        });
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
        assert_eq!(s.apply(&raw(300)).axes[0], 1.);
    }

    #[test]
    fn buttons_pass_through_and_are_not_motion() {
        let n = Settings::default().apply(&RawState {
            axes: [0; 6],
            buttons: 5,
        });
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
                full_scale: 0.,
                ..Settings::default()
            },
            Settings {
                full_scale: 40000.,
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
