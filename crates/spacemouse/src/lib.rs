//! A direct Linux reader for USB 3Dconnexion SpaceMouse devices.
//!
//! The crate reads `/dev/hidrawN` itself, so it needs no `spacenavd` daemon. It is
//! split into pure parts that tests drive with synthesised reports and a thin part that
//! touches the system:
//!
//! | Module | Role |
//! |---|---|
//! | [`report`] | Decode one raw HID input report into an axis or button update (pure) |
//! | [`filter`] | Deadzone, per-axis sensitivity and clamp: raw counts to a [`filter::Nav`] (pure) |
//! | [`device`] | The [`device::Opener`] seam and [`device::HidrawOpener`], which finds devices through sysfs |
//! | [`monitor`] | The hot-plug thread that publishes the latest state to a [`monitor::Latest`] handle |
//! | [`testing`] | A scripted device for tests and client development; it opens nothing |
//!
//! # Failing soft
//!
//! No device, an unreadable device node, a device unplugged mid-stream and a decoder
//! that does not recognise a report are all ordinary states, not errors: the monitor
//! publishes an all-zero, not-present snapshot and keeps polling. Nothing here returns
//! an error that a caller must act on after start-up.
//!
//! # Example
//!
//! ```
//! use visiontouch_spacemouse::filter::Settings;
//! use visiontouch_spacemouse::report::{Report, parse};
//!
//! // Report ID 1, three little-endian i16: x = 175, y = 0, z = -350.
//! let bytes = [1, 175, 0, 0, 0, 0xA2, 0xFE];
//! let Some(Report::Translation(t)) = parse(&bytes) else { panic!() };
//! assert_eq!(t, [175, 0, -350]);
//!
//! let settings = Settings::default();
//! let mut raw = visiontouch_spacemouse::report::RawState::default();
//! raw.apply(Report::Translation(t));
//! let nav = settings.apply(&raw);
//! assert!(nav.axes[0] > 0.4 && nav.axes[0] < 0.5);
//! assert_eq!(nav.axes[2], -1.0);
//! ```
#![forbid(unsafe_code)]
#![warn(missing_docs)]
pub mod device;
pub mod filter;
pub mod monitor;
pub mod report;
pub mod testing;
