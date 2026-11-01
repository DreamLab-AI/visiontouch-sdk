//! The versioned wire schema (JSON text frames over WebSocket).
//!
//! Every message is a JSON object carrying `"v": 1` and a `"type"`. Unknown types,
//! unknown fields, a different `v`, binary frames, non-finite or out-of-range numbers
//! are all **malformed**: the bridge answers with an `error`, cuts force and closes the
//! connection.
//!
//! Device positions are metres in the model frame, relative to the registered reset
//! pose, with Y up (the frame of `visiontouch_device::unit::State::position_m`). A
//! client may instead send targets in its own scene coordinates together with the view
//! transform that places the pen's workspace in that scene; see [`crate::view`].
//!
//! # Client to bridge
//!
//! | `type` | Fields | Meaning |
//! |---|---|---|
//! | `auth` | `event`: a signed Nostr event | Must be the first message, within the auth deadline. See [`crate::auth`] |
//! | `arm` | — | Ask for force to be armed. The deadman still gates it |
//! | `disarm` | — | Turn force off |
//! | `target` | `seq`: integer, `p`: `[x, y, z]` or `null`, optional `view` | The anchor for the coupling; `null` means nothing is held. Without `view`, `p` is in the model frame (metres). With `view` (`{"origin": [x, y, z], "rotation": [x, y, z, w], "scale": s}`, see [`crate::view`]), `p` is in the client's scene and the bridge maps it; the view is also used to report the pen in scene coordinates. Send at 20–30 Hz while connected, `null` included: it is also the liveness signal |
//!
//! | `nav_subscribe` | — | Ask for SpaceMouse navigation frames (controller only). Additive: a v1 client that never sends it never sees a `nav` message. See [`crate::nav`] |
//! | `nav_unsubscribe` | — | Stop them |
//!
//! # Bridge to client
//!
//! | `type` | Fields | Meaning |
//! |---|---|---|
//! | `hello` | `schema`, `challenge`, `limits` | Sent on connect. `challenge` is the nonce to sign |
//! | `auth_ok` | `pubkey` | This connection is now the controller |
//! | `status` | see [`Status`] | Pen pose, button, arm state, gate reason, force and heat, at the configured rate |
//! | `error` | `code`, `message` | Sent before the bridge closes the connection |
//! | `nav_ok` | `enabled`, `hz`, `present`, `reason` | Answer to `nav_subscribe`: whether the bridge offers navigation, the frame rate, whether a device is attached now, and why not if it is not (`spacenavd_absent`, `spacenavd_lost`, `no_device`; else `null`). A subscribed connection's `status` messages also carry `nav_input`, the same reason or `null`, so a change after subscribing reaches the client |
//! | `nav` | see [`NavFrame`] | One navigation frame, only after `nav_subscribe`. It carries `present` too, so a plug or unplug after subscribing is seen in the stream |
//!
//! ```
//! use visionclaw_bridge::protocol::{ClientMsg, parse_client};
//! let m = parse_client(r#"{"v":1,"type":"target","seq":7,"p":[0.01,-0.02,0.0]}"#).unwrap();
//! assert_eq!(m, ClientMsg::Target { seq: 7, p: Some([0.01, -0.02, 0.0]), view: None });
//! assert!(parse_client(r#"{"v":2,"type":"arm"}"#).is_err());
//! ```
use crate::gate::Reason;
use crate::view::{MAX_SCENE_COORD, View};
use serde::Serialize;
use serde_json::{Map, Value};

/// The schema version every message carries in `v`.
pub const SCHEMA_VERSION: u64 = 1;
/// Largest accepted target coordinate magnitude, metres. The coupling clamps targets to
/// a much smaller workspace; this only rejects nonsense.
pub const MAX_COORD_M: f64 = 1.0;

/// A parsed client message.
#[derive(Clone, Debug, PartialEq)]
pub enum ClientMsg {
    /// Authentication: a signed Nostr event answering the challenge.
    Auth {
        /// The event, verified by [`crate::auth::verify`].
        event: Box<nostr::event::Event>,
    },
    /// Ask for force to be armed.
    Arm,
    /// Turn force off.
    Disarm,
    /// Start receiving navigation frames.
    NavSubscribe,
    /// Stop receiving navigation frames.
    NavUnsubscribe,
    /// A new coupling target, or `None` for "nothing held".
    Target {
        /// Client sequence number, echoed in status.
        seq: u64,
        /// Target position: model-frame metres, or scene units when `view` is set.
        p: Option<[f64; 3]>,
        /// The view transform, when `p` is in scene coordinates.
        view: Option<View>,
    },
}

impl ClientMsg {
    /// The message's `type` string.
    pub fn kind(&self) -> &'static str {
        match self {
            ClientMsg::Auth { .. } => "auth",
            ClientMsg::Arm => "arm",
            ClientMsg::Disarm => "disarm",
            ClientMsg::NavSubscribe => "nav_subscribe",
            ClientMsg::NavUnsubscribe => "nav_unsubscribe",
            ClientMsg::Target { .. } => "target",
        }
    }
}

/// Why a message was refused.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Malformed(pub String);

impl std::fmt::Display for Malformed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for Malformed {}

fn bad(why: impl Into<String>) -> Malformed {
    Malformed(why.into())
}

fn only_fields(obj: &Map<String, Value>, allowed: &[&str]) -> Result<(), Malformed> {
    for k in obj.keys() {
        if !allowed.contains(&k.as_str()) {
            return Err(bad(format!("unknown field `{k}`")));
        }
    }
    Ok(())
}

fn parse_numbers<const N: usize>(v: &Value, name: &str, max: f64) -> Result<[f64; N], Malformed> {
    let arr = v
        .as_array()
        .ok_or_else(|| bad(format!("`{name}` must be an array of {N} numbers")))?;
    if arr.len() != N {
        return Err(bad(format!("`{name}` must have {N} numbers")));
    }
    let mut out = [0.; N];
    for (slot, x) in out.iter_mut().zip(arr) {
        let x = x
            .as_f64()
            .ok_or_else(|| bad(format!("`{name}` must contain numbers")))?;
        if !x.is_finite() || x.abs() > max {
            return Err(bad(format!("`{name}` value out of range")));
        }
        *slot = x;
    }
    Ok(out)
}

fn parse_view(v: &Value) -> Result<View, Malformed> {
    let obj = v
        .as_object()
        .ok_or_else(|| bad("`view` must be an object"))?;
    only_fields(obj, &["origin", "rotation", "scale"])?;
    let field = |k: &str| {
        obj.get(k)
            .ok_or_else(|| bad(format!("`view` is missing `{k}`")))
    };
    let origin = parse_numbers::<3>(field("origin")?, "origin", MAX_SCENE_COORD)?;
    let rotation = parse_numbers::<4>(field("rotation")?, "rotation", 2.)?;
    let scale = field("scale")?
        .as_f64()
        .ok_or_else(|| bad("`scale` must be a number"))?;
    View::new(origin, rotation, scale).map_err(bad)
}

/// Parse one client text frame. Strict: see the module docs.
pub fn parse_client(text: &str) -> Result<ClientMsg, Malformed> {
    let value: Value = serde_json::from_str(text).map_err(|e| bad(format!("not JSON: {e}")))?;
    let obj = value.as_object().ok_or_else(|| bad("not a JSON object"))?;
    match obj.get("v").and_then(Value::as_u64) {
        Some(SCHEMA_VERSION) => {}
        Some(v) => return Err(bad(format!("unsupported schema version {v}"))),
        None => return Err(bad("missing `v`")),
    }
    let kind = obj
        .get("type")
        .and_then(Value::as_str)
        .ok_or_else(|| bad("missing `type`"))?;
    match kind {
        "auth" => {
            only_fields(obj, &["v", "type", "event"])?;
            let event = obj.get("event").ok_or_else(|| bad("missing `event`"))?;
            let event: nostr::event::Event = serde_json::from_value(event.clone())
                .map_err(|e| bad(format!("`event` is not a Nostr event: {e}")))?;
            Ok(ClientMsg::Auth {
                event: Box::new(event),
            })
        }
        "arm" => {
            only_fields(obj, &["v", "type"])?;
            Ok(ClientMsg::Arm)
        }
        "disarm" => {
            only_fields(obj, &["v", "type"])?;
            Ok(ClientMsg::Disarm)
        }
        "nav_subscribe" => {
            only_fields(obj, &["v", "type"])?;
            Ok(ClientMsg::NavSubscribe)
        }
        "nav_unsubscribe" => {
            only_fields(obj, &["v", "type"])?;
            Ok(ClientMsg::NavUnsubscribe)
        }
        "target" => {
            only_fields(obj, &["v", "type", "seq", "p", "view"])?;
            let seq = obj
                .get("seq")
                .and_then(Value::as_u64)
                .ok_or_else(|| bad("`seq` must be a non-negative integer"))?;
            let view = obj.get("view").map(parse_view).transpose()?;
            let max = if view.is_some() {
                MAX_SCENE_COORD
            } else {
                MAX_COORD_M
            };
            let p = match obj.get("p") {
                None => return Err(bad("missing `p`")),
                Some(Value::Null) => None,
                Some(v) => Some(parse_numbers::<3>(v, "p", max)?),
            };
            Ok(ClientMsg::Target { seq, p, view })
        }
        other => Err(bad(format!("unknown type `{other}`"))),
    }
}

/// Limits the bridge is running with, announced in `hello`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Limits {
    /// Coupling stiffness, N/mm.
    pub stiffness_n_mm: f64,
    /// Coupling damping, N·s/m.
    pub damping_n_s_m: f64,
    /// Total force cap, N.
    pub force_cap_n: f64,
    /// Target timeout, ms: force stops if no target arrives for this long.
    pub target_timeout_ms: u64,
    /// Idle timeout, ms: the connection is closed if no message arrives for this long.
    pub idle_timeout_ms: u64,
    /// Target clamp radius about the registered origin, m.
    pub workspace_radius_m: f64,
    /// Status messages per second.
    pub status_hz: u32,
}

/// The servo loop's state, as sent in `status` messages.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Status {
    /// Milliseconds since the bridge started (monotonic).
    pub t_ms: u64,
    /// Pen position, metres, model frame; `null` until registered.
    pub pen: Option<[f64; 3]>,
    /// Pen position in scene coordinates through the last view sent; `null` without a
    /// view.
    pub pen_scene: Option<[f64; 3]>,
    /// Pen button held.
    pub button: bool,
    /// Whether the deadman has seen a release since force last became possible.
    pub deadman_ready: bool,
    /// The device reports armed.
    pub armed: bool,
    /// Gate decision.
    pub reason: Reason,
    /// The smoothed anchor the coupling pulls towards, while engaged.
    pub anchor: Option<[f64; 3]>,
    /// Force requested this tick, N, model frame.
    pub force: [f64; 3],
    /// The SDK's delivered-force estimate of the output just sent, N.
    pub applied_n: f64,
    /// Heat total, N²s.
    pub heat_n2s: f64,
    /// The heat budget is capping output.
    pub heat_limited: bool,
    /// Sequence number of the last target applied, if any.
    pub target_seq: Option<u64>,
    /// Age of the last target, ms, if any.
    pub target_age_ms: Option<u64>,
    /// The device has faulted; the bridge needs a restart at the arm.
    pub device_fault: bool,
}

/// One SpaceMouse navigation frame, sent as `nav`.
///
/// Axes are normalised to `[-1, 1]` after the bridge's deadzone and sensitivity, in the
/// device's own sign convention (the same signs a local USB SpaceMouse reports to the
/// client's existing driver). `buttons` is the device's button bitmask, bit 0 = button 1.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct NavFrame {
    /// Frame counter for this subscription, starting at 1.
    pub seq: u64,
    /// Monotonic milliseconds since the bridge began serving. Not the clock `status.t_ms`
    /// uses; compare frames with each other only.
    pub t: u64,
    /// Translation x (right).
    pub tx: f64,
    /// Translation y.
    pub ty: f64,
    /// Translation z.
    pub tz: f64,
    /// Rotation about x.
    pub rx: f64,
    /// Rotation about y.
    pub ry: f64,
    /// Rotation about z.
    pub rz: f64,
    /// Button bitmask.
    pub buttons: u32,
    /// A device is attached. Every change of this (a plug or an unplug) is sent as an
    /// all-zero frame, so a client can follow hot-plug from the stream alone.
    pub present: bool,
}

/// Error codes sent before a close.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ErrorCode {
    /// The message did not parse or broke the schema.
    Malformed,
    /// Authentication failed.
    Unauthorised,
    /// Another controller is connected.
    Busy,
    /// No valid `auth` within the deadline.
    AuthTimeout,
    /// No message within the idle timeout.
    Idle,
    /// The bridge is shutting down.
    Shutdown,
}

/// A bridge-to-client message.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMsg {
    /// Sent on connect.
    Hello {
        /// Schema version.
        schema: u64,
        /// The nonce to sign, hex.
        challenge: String,
        /// The limits in force.
        limits: Limits,
    },
    /// Authenticated; this connection is the controller.
    AuthOk {
        /// The authenticated public key, hex.
        pubkey: String,
    },
    /// Servo state.
    Status(Status),
    /// A `status` message sent to a connection that has subscribed to navigation. It is
    /// the same message plus `nav_input`, so a client learns that navigation went away
    /// (or came back) from the stream it is already reading. Added by the session
    /// layer; the servo thread does not know about navigation.
    #[serde(rename = "status")]
    StatusNav {
        /// The servo state.
        #[serde(flatten)]
        status: Status,
        /// `null` while navigation input is available, else why not: `spacenavd_absent`,
        /// `spacenavd_lost` or `no_device`.
        nav_input: Option<&'static str>,
    },
    /// The answer to `nav_subscribe`.
    NavOk {
        /// The bridge has SpaceMouse navigation enabled. When false no `nav` frame will
        /// ever follow.
        enabled: bool,
        /// Frames per second while the input is non-zero (0 when not enabled).
        hz: u32,
        /// A device is attached right now. Devices come and go; this is a snapshot.
        present: bool,
        /// `null` while input is available, else why not: `spacenavd_absent`,
        /// `spacenavd_lost` or `no_device`. Later changes arrive as `nav_input` in
        /// `status`.
        reason: Option<&'static str>,
    },
    /// One navigation frame.
    Nav(NavFrame),
    /// Sent before a close.
    Error {
        /// Machine-readable code.
        code: ErrorCode,
        /// Human-readable detail.
        message: String,
    },
}

impl ServerMsg {
    /// Encode as a text frame with `"v": 1`.
    pub fn to_text(&self) -> String {
        let mut value = serde_json::to_value(self).unwrap_or(Value::Null);
        if let Value::Object(obj) = &mut value {
            obj.insert("v".into(), Value::from(SCHEMA_VERSION));
        }
        value.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_messages_parse() {
        assert_eq!(parse_client(r#"{"v":1,"type":"arm"}"#), Ok(ClientMsg::Arm));
        assert_eq!(
            parse_client(r#"{"type":"disarm","v":1}"#),
            Ok(ClientMsg::Disarm)
        );
        assert_eq!(
            parse_client(r#"{"v":1,"type":"target","seq":0,"p":null}"#),
            Ok(ClientMsg::Target {
                seq: 0,
                p: None,
                view: None
            })
        );
        assert_eq!(
            parse_client(r#"{"v":1,"type":"target","seq":3,"p":[0,0.5,-1]}"#),
            Ok(ClientMsg::Target {
                seq: 3,
                p: Some([0., 0.5, -1.]),
                view: None
            })
        );
    }

    #[test]
    fn every_schema_breach_is_malformed() {
        for text in [
            "",
            "not json",
            "[]",
            r#"{"type":"arm"}"#,
            r#"{"v":2,"type":"arm"}"#,
            r#"{"v":"1","type":"arm"}"#,
            r#"{"v":1}"#,
            r#"{"v":1,"type":"fire"}"#,
            r#"{"v":1,"type":"arm","force":5}"#,
            r#"{"v":1,"type":"target","seq":1}"#,
            r#"{"v":1,"type":"target","p":null}"#,
            r#"{"v":1,"type":"target","seq":-1,"p":null}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,0]}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,0,0,0]}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,"0",0]}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,1.5,0]}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,1e999,0]}"#,
            r#"{"v":1,"type":"target","seq":1,"p":{"x":0}}"#,
            r#"{"v":1,"type":"auth"}"#,
            r#"{"v":1,"type":"auth","event":{"kind":1}}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,0,0],"view":{}}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,0,0],"view":{"origin":[0,0,0],"rotation":[0,0,0,1]}}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,0,0],"view":{"origin":[0,0,0],"rotation":[0,0,0,2],"scale":1}}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,0,0],"view":{"origin":[0,0,0],"rotation":[0,0,0,1],"scale":0}}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,0,0],"view":{"origin":[0,0],"rotation":[0,0,0,1],"scale":1}}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,0,0],"view":{"origin":[0,0,0],"rotation":[0,0,0,1],"scale":1,"skew":1}}"#,
            r#"{"v":1,"type":"target","seq":1,"p":[0,2e7,0],"view":{"origin":[0,0,0],"rotation":[0,0,0,1],"scale":1}}"#,
        ] {
            assert!(parse_client(text).is_err(), "accepted: {text}");
        }
    }

    #[test]
    fn scene_targets_carry_a_view() {
        let m = parse_client(
            r#"{"v":1,"type":"target","seq":4,"p":[250,-30,12],
                "view":{"origin":[200,0,0],"rotation":[0,0,0,1],"scale":1000}}"#,
        )
        .unwrap();
        let ClientMsg::Target { p, view, .. } = m else {
            panic!("{m:?}")
        };
        assert_eq!(
            p,
            Some([250., -30., 12.]),
            "scene coordinates may exceed 1 m"
        );
        let d = view.unwrap().to_device(p.unwrap());
        assert!((d[0] - 0.05).abs() < 1e-12 && (d[1] + 0.03).abs() < 1e-12);
        // A view with a null target is valid: it keeps the scene pen reported.
        assert!(
            parse_client(
                r#"{"v":1,"type":"target","seq":5,"p":null,
                    "view":{"origin":[0,0,0],"rotation":[0,0,0,1],"scale":1}}"#
            )
            .is_ok()
        );
    }

    #[test]
    fn nav_messages_parse_strictly() {
        assert_eq!(
            parse_client(r#"{"v":1,"type":"nav_subscribe"}"#),
            Ok(ClientMsg::NavSubscribe)
        );
        assert_eq!(
            parse_client(r#"{"v":1,"type":"nav_unsubscribe"}"#),
            Ok(ClientMsg::NavUnsubscribe)
        );
        for text in [
            r#"{"v":1,"type":"nav_subscribe","hz":500}"#,
            r#"{"v":2,"type":"nav_subscribe"}"#,
            r#"{"type":"nav_subscribe"}"#,
        ] {
            assert!(parse_client(text).is_err(), "accepted: {text}");
        }
    }

    #[test]
    fn nav_frames_serialise_flat_with_the_version() {
        let f = NavFrame {
            seq: 3,
            t: 1200,
            tx: 0.5,
            ty: 0.,
            tz: -1.,
            rx: 0.,
            ry: 0.25,
            rz: 0.,
            buttons: 2,
            present: true,
        };
        let v: Value = serde_json::from_str(&ServerMsg::Nav(f).to_text()).unwrap();
        assert_eq!(v["v"], 1);
        assert_eq!(v["type"], "nav");
        assert_eq!(v["seq"], 3);
        assert_eq!(v["tz"], -1.0);
        assert_eq!(v["buttons"], 2);
        assert_eq!(v["present"], true);
        let ok: Value = serde_json::from_str(
            &ServerMsg::NavOk {
                enabled: true,
                hz: 60,
                present: false,
                reason: Some("spacenavd_absent"),
            }
            .to_text(),
        )
        .unwrap();
        assert_eq!(ok["type"], "nav_ok");
        assert_eq!(ok["present"], false);
        assert_eq!(ok["reason"], "spacenavd_absent");
    }

    #[test]
    fn server_messages_carry_the_version_and_type() {
        let text = ServerMsg::Error {
            code: ErrorCode::Busy,
            message: "x".into(),
        }
        .to_text();
        let v: Value = serde_json::from_str(&text).unwrap();
        assert_eq!(v["v"], 1);
        assert_eq!(v["type"], "error");
        assert_eq!(v["code"], "busy");
    }

    #[test]
    fn status_serialises_flat() {
        let s = Status {
            t_ms: 1,
            pen: Some([0.; 3]),
            pen_scene: None,
            button: true,
            deadman_ready: false,
            armed: true,
            reason: Reason::AwaitRelease,
            anchor: None,
            force: [0.; 3],
            applied_n: 0.,
            heat_n2s: 0.,
            heat_limited: false,
            target_seq: None,
            target_age_ms: None,
            device_fault: false,
        };
        let v: Value = serde_json::from_str(&ServerMsg::Status(s).to_text()).unwrap();
        assert_eq!(v["type"], "status");
        assert_eq!(v["reason"], "await_release");
        assert_eq!(v["pen"], serde_json::json!([0., 0., 0.]));
    }
}
