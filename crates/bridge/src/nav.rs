//! SpaceMouse navigation over the bridge's socket.
//!
//! A USB SpaceMouse plugged into the bridge's machine is read by
//! [`visiontouch_spacemouse`]; this module decides what goes on the wire. Navigation
//! is **view input only**:
//!
//! - It lives entirely outside the force path. Nothing here is passed to
//!   [`crate::servo`], the [`crate::servo::Inbox`], the gate or the coupling, so a
//!   deflected SpaceMouse cannot change a target, an anchor or a force. The only thing a
//!   client can do with navigation is move its own view; the target it then sends, and
//!   the anchor slew, force-rate and force-cap limits that already bound a view moving
//!   under a hold, are the same as for any other camera motion.
//! - It uses the same socket and the same authentication. Frames go to the
//!   authenticated controller only, and only after that connection has sent
//!   `nav_subscribe`. A v1 client that never subscribes sees no new message type.
//!
//! # Streaming rule
//!
//! At [`crate::config::SpacemouseConfig::rate_hz`] the session asks [`NavStream::next`]
//! for a frame. It yields one when any axis is non-zero (so a held deflection is
//! refreshed at the full rate, which lets a client treat a gap as "released"), and
//! otherwise only when something changed since the last frame sent: exactly one all-zero
//! frame when the puck is released, one when the device is plugged in and one when it
//! is unplugged (each carries the new `present`), and one frame per button change. A
//! still, unchanged device sends nothing. Nav sends do not count as client activity
//! for the idle timeout.
//!
//! Presence is part of the stream because a client that only learns it from `nav_ok`
//! cannot tell that a device was plugged in, or removed, after it subscribed.
use crate::protocol::NavFrame;
use visiontouch_spacemouse::filter::Nav;
use visiontouch_spacemouse::monitor::Snapshot;

/// Per-subscription streaming state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NavStream {
    last: Nav,
    present: bool,
    seq: u64,
}

impl NavStream {
    /// A fresh subscription. The client has just been told `present` in `nav_ok` and
    /// the last frame is taken to be all-zero, so a client subscribing while the device
    /// is at rest hears nothing until something changes.
    pub fn new(present: bool) -> Self {
        Self {
            last: Nav::default(),
            present,
            seq: 0,
        }
    }

    /// The frame to send now for `snapshot` at `t_ms`, if any.
    pub fn next(&mut self, snapshot: &Snapshot, t_ms: u64) -> Option<NavFrame> {
        let nav = snapshot.nav;
        if nav.is_still() && nav == self.last && snapshot.present == self.present {
            return None;
        }
        self.last = nav;
        self.present = snapshot.present;
        self.seq += 1;
        let [tx, ty, tz, rx, ry, rz] = nav.axes;
        Some(NavFrame {
            seq: self.seq,
            t: t_ms,
            tx,
            ty,
            tz,
            rx,
            ry,
            rz,
            buttons: nav.buttons,
            present: snapshot.present,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snap(axes: [f64; 6], buttons: u32) -> Snapshot {
        Snapshot {
            present: true,
            nav: Nav { axes, buttons },
            version: 0,
        }
    }

    #[test]
    fn a_device_at_rest_sends_nothing() {
        let mut s = NavStream::new(true);
        for t in 0..10 {
            assert_eq!(s.next(&snap([0.; 6], 0), t), None);
        }
    }

    #[test]
    fn a_held_deflection_streams_every_tick_then_one_zero_frame() {
        let mut s = NavStream::new(true);
        let held = snap([0.5, 0., 0., 0., 0., 0.], 0);
        let a = s.next(&held, 10).unwrap();
        let b = s.next(&held, 26).unwrap();
        assert_eq!(
            (a.seq, b.seq),
            (1, 2),
            "unchanged non-zero input is refreshed"
        );
        assert_eq!(b.tx, 0.5);
        let zero = s.next(&snap([0.; 6], 0), 42).unwrap();
        assert_eq!((zero.seq, zero.tx), (3, 0.));
        assert_eq!(
            s.next(&snap([0.; 6], 0), 58),
            None,
            "exactly one zero frame"
        );
    }

    #[test]
    fn a_button_change_is_one_frame_even_without_motion() {
        let mut s = NavStream::new(true);
        let f = s.next(&snap([0.; 6], 0b10), 5).unwrap();
        assert_eq!((f.buttons, f.tx), (0b10, 0.));
        assert_eq!(
            s.next(&snap([0.; 6], 0b10), 21),
            None,
            "a held button is quiet"
        );
        assert_eq!(s.next(&snap([0.; 6], 0), 37).unwrap().buttons, 0);
    }

    #[test]
    fn an_unplug_mid_deflection_ends_in_a_zero_frame_that_says_absent() {
        let mut s = NavStream::new(true);
        let live = s.next(&snap([0., 0., 1., 0., 0., 0.], 0), 1).unwrap();
        assert!(live.present);
        let gone = Snapshot::default();
        let f = s.next(&gone, 17).unwrap();
        assert_eq!((f.tz, f.buttons, f.present), (0., 0, false));
        assert_eq!(s.next(&gone, 33), None);
    }

    #[test]
    fn plugging_and_unplugging_each_send_one_frame_with_the_new_presence() {
        let mut s = NavStream::new(false);
        let absent = Snapshot::default();
        assert_eq!(
            s.next(&absent, 1),
            None,
            "absent and still, as told in nav_ok"
        );
        let plugged = snap([0.; 6], 0);
        let f = s.next(&plugged, 17).unwrap();
        assert!(f.present && f.tx == 0.);
        assert_eq!(s.next(&plugged, 33), None);
        let f = s.next(&absent, 49).unwrap();
        assert!(!f.present);
        assert_eq!(s.next(&absent, 65), None);
    }

    #[test]
    fn a_subscription_seeded_present_hears_nothing_about_the_device_it_was_told_of() {
        let mut s = NavStream::new(true);
        assert_eq!(s.next(&snap([0.; 6], 0), 1), None);
    }
}
