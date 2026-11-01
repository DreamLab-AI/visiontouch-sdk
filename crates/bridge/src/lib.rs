//! A network WebSocket bridge for the single-unit haptic arm.
//!
//! The bridge runs on the machine the arm is plugged into. It owns the
//! [`visiontouch_device::unit::Unit`] on a servo thread at about 400 Hz and talks to
//! one authenticated client over a WebSocket. **The servo loop never crosses the
//! network**: the client sends a *target* (where the thing it holds is), and the bridge
//! computes the force locally from a virtual coupling,
//! `F = -k·(pen − target) − c·(v_pen − v_target)`, from the pen it has just measured.
//! The client receives the pen pose, the button, the arm state and the heat budget.
//!
//! # Safety model
//!
//! - **Fail safe on every link failure.** Force stops on the next servo tick if the
//!   socket drops, if no message arrives within the idle timeout, if no target has
//!   arrived within the target timeout (default 200 ms), or on any malformed message;
//!   zero is queued before the next device service, and a servo stall over 10 ms also
//!   cuts force. The SDK's own 25 ms stale-output rule stays the inner guard. See
//!   [`gate`] and [`servo`].
//! - **Deadman.** Force flows only while the pen button is held, and only after it has
//!   been seen released since the last arm request, reconnect, fault or stall.
//! - **Gentle engage.** When force becomes allowed the anchor starts at the pen, travels
//!   to the target at a bounded speed, and the force ramps from zero.
//! - **Bounded output.** Stiffness is capped at half the unit's stable 2 N/mm, force at
//!   the configured cap (default the 1.41 N continuous level), and the SDK's heat
//!   budget applies on top. See [`config`].
//! - **Authenticated control.** A Nostr-key challenge ([`auth`]) against an allowlist,
//!   one controller at a time, LAN-only by default ([`netpolicy`]), browser origins
//!   only when listed. No message before
//!   authentication can arm anything, and taking control never arms by itself.
//!
//! # Wire schema
//!
//! Versioned JSON over WebSocket text frames: see [`protocol`].
//!
//! # Layout
//!
//! | Module | Role |
//! |---|---|
//! | [`coupling`] | Target smoothing and the spring-damper force (pure) |
//! | [`gate`] | Every condition force needs, with the deadman latch (pure) |
//! | [`servo`] | The servo thread and the [`servo::Haptic`] device seam |
//! | [`server`] | The WebSocket server |
//! | [`protocol`] | The message schema |
//! | [`auth`] | Challenge and signature verification |
//! | [`netpolicy`] | LAN-only bind and peer rules |
//! | [`config`] | TOML configuration and its limits |
//! | [`view`] | The view transform between the pen's workspace and a client's scene |
//! | [`fake`] | A simulated unit for tests and client development |
#![warn(missing_docs)]
pub mod auth;
pub mod config;
pub mod coupling;
pub mod fake;
pub mod gate;
pub mod netpolicy;
pub mod protocol;
pub mod server;
pub mod servo;
pub mod view;
