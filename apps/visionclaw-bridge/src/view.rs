//! The view transform: how the pen's workspace sits in the client's scene.
//!
//! A client that holds things in its own scene sends, with every target, the transform
//! that places the pen's model frame in that scene:
//!
//! ```text
//! p_scene = origin + scale · R(rotation) · p_device
//! ```
//!
//! - `origin`: the scene point where the pen's registered origin sits, scene units;
//! - `rotation`: a unit quaternion `[x, y, z, w]` turning the pen's axes (X, Y up, Z)
//!   into scene axes;
//! - `scale`: scene units per metre of pen travel, positive.
//!
//! The client composes everything between its scene coordinates and the pen into this
//! one similarity transform: camera, navigation device, any transform of the content
//! (a moved, resized or rearranged volume), entering a room. The bridge maps scene
//! targets into the model frame with the inverse, and reports the pen back in scene
//! coordinates. When the view moves, the mapped target moves, so the coupling makes the
//! hand feel scene motion; the anchor's slew limit and the force-rate limit keep that
//! motion from ever becoming a force step.
//!
//! ```
//! use visionclaw_bridge::view::View;
//! // Scene is 1000 units per metre, origin at (5, 0, 0), no rotation.
//! let v = View::new([5., 0., 0.], [0., 0., 0., 1.], 1000.).unwrap();
//! let d = v.to_device([15., 0., 0.]);
//! assert!((d[0] - 0.01).abs() < 1e-12);
//! let s = v.to_scene(d);
//! assert!((s[0] - 15.).abs() < 1e-9);
//! ```
use crate::coupling::V3;

/// Largest accepted scene coordinate magnitude, scene units.
pub const MAX_SCENE_COORD: f64 = 1e7;
/// Smallest and largest accepted `scale`, scene units per metre.
pub const SCALE_RANGE: (f64, f64) = (1e-3, 1e9);
/// How far from 1 the quaternion's length may be before it is refused (it is then
/// normalised).
pub const QUATERNION_TOLERANCE: f64 = 1e-3;

/// A validated view transform. See the module docs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct View {
    origin: V3,
    /// Unit quaternion `[x, y, z, w]`.
    rotation: [f64; 4],
    scale: f64,
}

fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Rotate `v` by the unit quaternion `q` (`v' = v + 2w(u×v) + 2u×(u×v)`).
fn rotate(q: [f64; 4], v: V3) -> V3 {
    let u = [q[0], q[1], q[2]];
    let w = q[3];
    let t = cross(u, v).map(|c| 2. * c);
    let ut = cross(u, t);
    [
        v[0] + w * t[0] + ut[0],
        v[1] + w * t[1] + ut[1],
        v[2] + w * t[2] + ut[2],
    ]
}

impl View {
    /// Validate and build. Refuses non-finite values, a scene origin beyond
    /// [`MAX_SCENE_COORD`], a scale outside [`SCALE_RANGE`], or a quaternion whose
    /// length is more than [`QUATERNION_TOLERANCE`] from 1.
    pub fn new(origin: V3, rotation: [f64; 4], scale: f64) -> Result<Self, String> {
        if !origin
            .iter()
            .all(|x| x.is_finite() && x.abs() <= MAX_SCENE_COORD)
        {
            return Err("view origin out of range".into());
        }
        if !(scale.is_finite() && scale >= SCALE_RANGE.0 && scale <= SCALE_RANGE.1) {
            return Err("view scale out of range".into());
        }
        if !rotation.iter().all(|x| x.is_finite()) {
            return Err("view rotation not finite".into());
        }
        let n = rotation.iter().map(|x| x * x).sum::<f64>().sqrt();
        if (n - 1.).abs() > QUATERNION_TOLERANCE {
            return Err("view rotation is not a unit quaternion".into());
        }
        Ok(Self {
            origin,
            rotation: rotation.map(|x| x / n),
            scale,
        })
    }

    /// Scene point to the pen's model frame, metres.
    pub fn to_device(&self, p: V3) -> V3 {
        let d = [
            p[0] - self.origin[0],
            p[1] - self.origin[1],
            p[2] - self.origin[2],
        ];
        let q = self.rotation;
        let inverse = [-q[0], -q[1], -q[2], q[3]];
        rotate(inverse, d).map(|x| x / self.scale)
    }

    /// Model-frame point, metres, to the scene.
    pub fn to_scene(&self, p: V3) -> V3 {
        let r = rotate(self.rotation, p);
        [
            self.origin[0] + self.scale * r[0],
            self.origin[1] + self.scale * r[1],
            self.origin[2] + self.scale * r[2],
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: V3, b: V3, tol: f64) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < tol)
    }

    #[test]
    fn identity_scales_and_offsets() {
        let v = View::new([1., 2., 3.], [0., 0., 0., 1.], 100.).unwrap();
        assert!(close(v.to_device([2., 2., 3.]), [0.01, 0., 0.], 1e-12));
        assert!(close(v.to_scene([0., 0.02, 0.]), [1., 4., 3.], 1e-12));
    }

    #[test]
    fn a_quarter_turn_about_y_maps_the_axes() {
        // 90° about +Y: pen +X becomes scene -Z, pen +Z becomes scene +X.
        let h = std::f64::consts::FRAC_PI_4;
        let v = View::new([0.; 3], [0., h.sin(), 0., h.cos()], 1.).unwrap();
        assert!(close(v.to_scene([1., 0., 0.]), [0., 0., -1.], 1e-12));
        assert!(close(v.to_scene([0., 0., 1.]), [1., 0., 0.], 1e-12));
        assert!(close(v.to_scene([0., 1., 0.]), [0., 1., 0.], 1e-12));
    }

    #[test]
    fn round_trips_through_an_arbitrary_view() {
        let q = [0.3, -0.5, 0.2, 0.787_400_787_401_181];
        let v = View::new([120., -40., 7.5], q, 2500.).unwrap();
        for p in [[0.01, 0.02, -0.03], [0., 0., 0.], [-0.1, 0.05, 0.07]] {
            assert!(close(v.to_device(v.to_scene(p)), p, 1e-12));
        }
        let s = [130., -35., 0.];
        assert!(close(v.to_scene(v.to_device(s)), s, 1e-9));
    }

    #[test]
    fn invalid_views_are_refused() {
        let id = [0., 0., 0., 1.];
        assert!(View::new([f64::NAN, 0., 0.], id, 1.).is_err());
        assert!(View::new([2e7, 0., 0.], id, 1.).is_err());
        assert!(View::new([0.; 3], id, 0.).is_err());
        assert!(View::new([0.; 3], id, -1.).is_err());
        assert!(View::new([0.; 3], id, f64::INFINITY).is_err());
        assert!(View::new([0.; 3], [0., 0., 0., 0.], 1.).is_err());
        assert!(View::new([0.; 3], [0., 0., 0., 1.1], 1.).is_err());
        assert!(View::new([0.; 3], [0., 0., 0., f64::NAN], 1.).is_err());
        assert!(
            View::new([0.; 3], [0., 0., 0., 1.0005], 1.).is_ok(),
            "normalised"
        );
    }
}
