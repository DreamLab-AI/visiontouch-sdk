//! A pure-Rust client of **spacenavd**'s AF_UNIX protocol.
//!
//! spacenavd is the Linux driver for 3Dconnexion SpaceMouse devices. It owns the device
//! and serves motion and button events on a Unix stream socket, `/var/run/spnav.sock`
//! by default. This crate speaks that protocol directly, so nothing here reads
//! `/dev` nodes or decodes device reports, and the C `libspnav` is not linked.
//! libspnav is not needed: the whole client side is one 4-byte handshake, one optional
//! 32-byte request and a stream of 32-byte event frames.
//!
//! # The protocol as implemented
//!
//! Implemented from `libspnav/src/spnav.c` and `libspnav/src/proto.h` (libspnav at
//! `476a17b6087146f95799e3a2a12b4d13cb4d2818`) and the daemon side in
//! `spacenavd/src/proto_unix.c` (spacenavd at
//! `05cc67938e77ac7ad9c5b34267984d944cfd57dd`), both from the FreeSpacenav project.
//! Integers are `int32` in native byte order.
//!
//! | Step | Bytes | Source |
//! |---|---|---|
//! | Handshake, client to daemon | one `int32`, `0x7faa5500 \| 1` (`REQ_TAG \| REQ_CHANGE_PROTO \| version`) | `spnav_open`, `spnav.c` |
//! | Handshake reply | the same `int32` with the version the daemon accepted in its low byte; the daemon switches that client to protocol 1 and sets the event mask to motion, button and device events | `proto_unix.c`, `case 0` |
//! | Old daemon | no reply: it read the handshake as a sensitivity `float`. libspnav waits 300 ms, then writes `1.0f32` to restore it and continues with protocol 0 | `spnav_open` |
//! | Event | 8 × `int32`; word 0 is the type: `0` motion (words 1..=6 are x, y, z, rx, ry, rz and word 7 the period in ms), `1` button press and `2` button release (word 1 is the button number), `3` device change (word 1 is `0` for add, `1` for remove) | `proc_event`, `spnav.c`; `send_uevent`, `proto_unix.c` |
//! | Device query | a 32-byte request whose type is `REQ_TAG \| REQ_DEV_NAXES` (`0x7faa2002`); the 32-byte reply has the same type and the last word (`data[6]`) is `0` on success and `-1` when the daemon has no device | `request`, `spnav.c`; `handle_request`, `proto_unix.c` |
//!
//! Motion values are the daemon's own output: they are scaled by the daemon's
//! sensitivity and are **not** normalised to a fixed range; the range is whatever the
//! daemon is configured to emit.
//!
//! # Failing soft
//!
//! [`Monitor`] runs one thread that connects, reads and reconnects for as long as it
//! lives. A daemon that is not running, a stale socket file, a daemon that goes away,
//! a device that is unplugged and a protocol-0 daemon are all ordinary states,
//! reported through [`Snapshot::health`]. The published axes and buttons are zero
//! whenever [`Health::present`] is false, so a motion can never be left latched.
//!
//! # Example
//!
//! ```
//! use spnav_client::{Event, decode};
//!
//! // A motion frame: type 0, x = 120, the rest 0, period 16 ms.
//! let mut frame = [0u8; 32];
//! frame[4..8].copy_from_slice(&120i32.to_ne_bytes());
//! frame[28..32].copy_from_slice(&16i32.to_ne_bytes());
//! assert_eq!(decode(&frame), Some(Event::Motion { axes: [120, 0, 0, 0, 0, 0] }));
//! ```
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod monitor;
pub mod testing;
pub mod wire;

pub use monitor::{Health, Latest, Monitor, Snapshot};
pub use wire::{DEFAULT_SOCKET, Event, decode};
