//! The versioned wire schema (JSON text frames over WebSocket).
//!
//! Every message is a JSON object carrying `"v": 1` and a `"type"`. Unknown types,
//! unknown fields, a different `v`, binary frames, non-finite or out-of-range numbers
//! are all **malformed**: the bridge answers with an `error`, cuts force and closes the
//! connection.
//!
//! Positions are metres in the device's model frame, relative to the registered reset
//! pose, with Y up (the frame of `visiontouch_device::unit::State::position_m`). The
//! client maps between that frame and its own scene; the bridge knows nothing about it.
//!
//! # Client to bridge
//!
//! | `type` | Fields | Meaning |
//! |---|---|---|
//! | `auth` | `event`: a signed Nostr event | Must be the first message, within the auth deadline. See [`crate::auth`] |
//! | `arm` | — | Ask for force to be armed. The deadman still gates it |
//! | `disarm` | — | Turn force off |
//! | `target` | `seq`: integer, `p`: `[x, y, z]` or `null` | The anchor for the coupling; `null` means nothing is held. Send at 20–30 Hz while connected, `null` included: it is also the liveness signal |
//!
//! # Bridge to client
//!
//! | `type` | Fields | Meaning |
//! |---|---|---|
//! | `hello` | `schema`, `challenge`, `limits` | Sent on connect. `challenge` is the nonce to sign |
//! | `auth_ok` | `pubkey` | This connection is now the controller |
//! | `status` | see [`Status`] | Pen pose, button, arm state, gate reason, force and heat, at the configured rate |
//! | `error` | `code`, `message` | Sent before the bridge closes the connection |
//!
//! ```
//! use visiontouch_bridge::protocol::{ClientMsg, parse_client};
//! let m = parse_client(r#"{"v":1,"type":"target","seq":7,"p":[0.01,-0.02,0.0]}"#).unwrap();
//! assert_eq!(m, ClientMsg::Target { seq: 7, p: Some([0.01, -0.02, 0.0]) });
//! assert!(parse_client(r#"{"v":2,"type":"arm"}"#).is_err());
//! ```
use crate::gate::Reason;
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
    /// A new coupling target, or `None` for "nothing held".
    Target {
        /// Client sequence number, echoed in status.
        seq: u64,
        /// Target position, metres, model frame.
        p: Option<[f64; 3]>,
    },
}

impl ClientMsg {
    /// The message's `type` string.
    pub fn kind(&self) -> &'static str {
        match self {
            ClientMsg::Auth { .. } => "auth",
            ClientMsg::Arm => "arm",
            ClientMsg::Disarm => "disarm",
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

fn parse_point(v: &Value) -> Result<[f64; 3], Malformed> {
    let arr = v
        .as_array()
        .ok_or_else(|| bad("`p` must be [x, y, z] or null"))?;
    if arr.len() != 3 {
        return Err(bad("`p` must have three coordinates"));
    }
    let mut p = [0.; 3];
    for (slot, x) in p.iter_mut().zip(arr) {
        let x = x
            .as_f64()
            .ok_or_else(|| bad("`p` coordinates must be numbers"))?;
        if !x.is_finite() || x.abs() > MAX_COORD_M {
            return Err(bad("`p` coordinate out of range"));
        }
        *slot = x;
    }
    Ok(p)
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
        "target" => {
            only_fields(obj, &["v", "type", "seq", "p"])?;
            let seq = obj
                .get("seq")
                .and_then(Value::as_u64)
                .ok_or_else(|| bad("`seq` must be a non-negative integer"))?;
            let p = match obj.get("p") {
                None => return Err(bad("missing `p`")),
                Some(Value::Null) => None,
                Some(v) => Some(parse_point(v)?),
            };
            Ok(ClientMsg::Target { seq, p })
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
            Ok(ClientMsg::Target { seq: 0, p: None })
        );
        assert_eq!(
            parse_client(r#"{"v":1,"type":"target","seq":3,"p":[0,0.5,-1]}"#),
            Ok(ClientMsg::Target {
                seq: 3,
                p: Some([0., 0.5, -1.])
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
        ] {
            assert!(parse_client(text).is_err(), "accepted: {text}");
        }
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
