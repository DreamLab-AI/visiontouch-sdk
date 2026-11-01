//! Wire constants and frame decoding. See the [crate documentation](crate) for the
//! protocol and its sources.

/// The default socket path of spacenavd (`SPNAV_SOCK_PATH` in `spnav.c`).
pub const DEFAULT_SOCKET: &str = "/var/run/spnav.sock";

/// Bytes in one event or request frame: eight `int32`.
pub const FRAME_LEN: usize = 32;

/// The tag spacenavd puts in the high bits of request and reply types
/// (`REQ_TAG`, `proto.h`). It puts a reply outside the event type range.
pub const REQ_TAG: i32 = 0x7faa_0000;

/// `REQ_CHANGE_PROTO` (`proto.h`): the handshake request.
pub const REQ_CHANGE_PROTO: i32 = 0x5500;

/// `REQ_DEV_NAXES` (`proto.h`): ask for the number of axes of the daemon's device.
pub const REQ_DEV_NAXES: i32 = 0x2002;

/// The newest protocol version this client asks for (`MAX_PROTO_VER`, `proto.h`).
pub const PROTO_VERSION: i32 = 1;

/// The 4-byte handshake a client sends first.
pub const HANDSHAKE: i32 = REQ_TAG | REQ_CHANGE_PROTO | PROTO_VERSION;

const UEV_MOTION: i32 = 0;
const UEV_PRESS: i32 = 1;
const UEV_RELEASE: i32 = 2;
const UEV_DEV: i32 = 3;
const DEV_RM: i32 = 1;

/// One event frame from the daemon.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// Six-axis motion, `x, y, z, rx, ry, rz`, in the daemon's units. The daemon
    /// sends the full state on every change, so a release is an all-zero motion.
    Motion {
        /// The six axes.
        axes: [i32; 6],
    },
    /// A button went down or up.
    Button {
        /// Button number, from 0.
        number: i32,
        /// `true` for a press.
        press: bool,
    },
    /// The daemon's device was added (`true`) or removed (`false`).
    Device {
        /// Whether a device is now attached.
        added: bool,
    },
}

/// What one 32-byte frame is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Frame {
    Event(Event),
    /// The reply to a request: its type (with the tag) and its status word.
    Reply {
        kind: i32,
        status: i32,
    },
    /// An event type this client does not use.
    Ignored,
}

fn word(frame: &[u8; FRAME_LEN], i: usize) -> i32 {
    let mut b = [0u8; 4];
    b.copy_from_slice(&frame[i * 4..i * 4 + 4]);
    i32::from_ne_bytes(b)
}

/// Decode one event frame. Returns `None` for replies and for event types this client
/// does not use (configuration changes, raw axes, raw buttons).
pub fn decode(frame: &[u8; FRAME_LEN]) -> Option<Event> {
    match classify(frame) {
        Frame::Event(e) => Some(e),
        _ => None,
    }
}

pub(crate) fn classify(frame: &[u8; FRAME_LEN]) -> Frame {
    let kind = word(frame, 0);
    match kind {
        UEV_MOTION => Frame::Event(Event::Motion {
            axes: std::array::from_fn(|i| word(frame, i + 1)),
        }),
        UEV_PRESS | UEV_RELEASE => Frame::Event(Event::Button {
            number: word(frame, 1),
            press: kind == UEV_PRESS,
        }),
        UEV_DEV => Frame::Event(Event::Device {
            added: word(frame, 1) != DEV_RM,
        }),
        k if k & !0xffff == REQ_TAG => Frame::Reply {
            kind: k,
            status: word(frame, 7),
        },
        _ => Frame::Ignored,
    }
}

/// The 32-byte `REQ_DEV_NAXES` request.
pub(crate) fn naxes_request() -> [u8; FRAME_LEN] {
    let mut out = [0u8; FRAME_LEN];
    out[..4].copy_from_slice(&(REQ_TAG | REQ_DEV_NAXES).to_ne_bytes());
    out
}

/// Build an event frame (used by the test daemon and the tests).
pub(crate) fn frame(words: [i32; 8]) -> [u8; FRAME_LEN] {
    let mut out = [0u8; FRAME_LEN];
    for (i, w) in words.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&w.to_ne_bytes());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_decodes_all_six_axes_in_order() {
        let f = frame([0, 1, -2, 3, -4, 5, -6, 16]);
        assert_eq!(
            decode(&f),
            Some(Event::Motion {
                axes: [1, -2, 3, -4, 5, -6]
            })
        );
    }

    #[test]
    fn press_and_release_are_types_one_and_two() {
        assert_eq!(
            decode(&frame([1, 7, 0, 0, 0, 0, 0, 0])),
            Some(Event::Button {
                number: 7,
                press: true
            })
        );
        assert_eq!(
            decode(&frame([2, 7, 0, 0, 0, 0, 0, 0])),
            Some(Event::Button {
                number: 7,
                press: false
            })
        );
    }

    #[test]
    fn device_add_is_op_zero_and_remove_is_op_one() {
        assert_eq!(
            decode(&frame([3, 0, 0, 0, 0, 0, 0, 0])),
            Some(Event::Device { added: true })
        );
        assert_eq!(
            decode(&frame([3, 1, 0, 0, 0, 0, 0, 0])),
            Some(Event::Device { added: false })
        );
    }

    #[test]
    fn replies_and_unused_types_are_not_events() {
        let reply = frame([REQ_TAG | REQ_DEV_NAXES, 6, 0, 0, 0, 0, 0, 0]);
        assert_eq!(decode(&reply), None);
        assert_eq!(
            classify(&reply),
            Frame::Reply {
                kind: REQ_TAG | REQ_DEV_NAXES,
                status: 0
            }
        );
        for kind in [4, 5, 6, 99, -1] {
            assert_eq!(
                classify(&frame([kind, 0, 0, 0, 0, 0, 0, 0])),
                Frame::Ignored
            );
        }
    }

    #[test]
    fn the_handshake_is_the_documented_constant() {
        assert_eq!(HANDSHAKE, 0x7faa_5501);
    }
}
