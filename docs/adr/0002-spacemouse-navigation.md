---
id: ADR-0002
title: Relay a SpaceMouse over the bridge for viewport navigation, through spacenavd
date: 2026-11-01
decision_status: proposed
implementation_status: complete
activation_status: inactive
supersedes: []
superseded_by: []
verified_paths: [apps/spnav-client/src, apps/spnav-client/tests/daemon.rs, apps/visionclaw-bridge/src/nav.rs, apps/visionclaw-bridge/src/filter.rs, apps/visionclaw-bridge/tests/nav_e2e.rs]
review_trigger: the first read of a real SpaceMouse through a running spacenavd on the haptics machine, a spacenavd release that changes the socket protocol, a client that needs a different frame rate or axis convention, or any change that would let navigation reach the force path
---

# ADR-0002: Relay a USB SpaceMouse over the bridge for viewport navigation

## Context

The arm's host is the machine at the bench, and the client it serves is a browser on
another desktop. The person at the bench has one hand on the pen and wants the other on
a SpaceMouse to steer the client's viewport. A SpaceMouse plugged into the desktop
works today only if the person is at the desktop. Plugged into the bench machine, it
needs a path to the client, and the bridge already has an authenticated socket to
exactly that client.

Three constraints shape the answer. The device is read on the bridge's machine, so the
reader must work headless, with no desktop session. The stream rides the bridge's socket and its
authentication, not a second listener. And a view that moves must never be able to push
force: navigation is view input, and the force path is the bridge's job alone.

## Decision

The bridge is a **demo app** beside the SDK (see ADR-0001), so everything here lives in
`apps/` and is `publish = false`.

1. **Use the Linux driver, not raw HID.** The owner's instruction was to use the
   existing Linux drivers, with no browser HID. The driver for 3Dconnexion devices on
   Linux is **spacenavd**, which owns the device and serves events on a Unix socket
   (`/var/run/spnav.sock`). The bridge is a client of that socket and decodes no device
   reports of its own; spacenavd already does that work, for far more models than a
   table here could list. (An earlier revision of this branch read the device
   directly. It is superseded by this record and removed from the tree.)
2. **A small pure-Rust client crate, `apps/spnav-client`** (`publish = false`). It
   implements the socket protocol from `libspnav/src/spnav.c` and `proto.h` and
   `spacenavd/src/proto_unix.c` (commit hashes in the crate documentation): a 4-byte
   protocol-1 handshake, 32-byte event frames (motion, button press and release, device
   add and remove), and the `REQ_DEV_NAXES` query to learn at connect whether a device
   exists. libspnav (C) is not linked: the client side is a few dozen lines, and a C
   dependency would add a build requirement and an FFI boundary for no gain. It is a
   separate crate from the bridge because it knows nothing of the bridge, the arm or
   the SDK, and a test double for the daemon (`spnav_client::testing::FakeDaemon`)
   serves both its own tests and the bridge's end-to-end tests. It is `publish = false`
   because it is demo support, not an offering.
3. **Failing soft.** One thread connects, reads and reconnects every `poll_ms`. spacenavd
   not running, a stale socket file, the daemon dying mid-stream, a device unplugged and a
   thread panic all leave a zero snapshot with a reason (`spacenavd_absent`,
   `spacenavd_lost`, `no_device`). No `nav` frame is sent without input; the bridge keeps
   serving the arm; and the monitor has no link to the servo thread or the shutdown
   watcher. It notices the daemon coming back on its next attempt.
4. **The reason reaches the client.** In `nav_ok.reason` at subscribe time and, because
   `nav_ok` is a one-shot reply, in `nav_input` on every `status` message of a
   subscribed connection. The session layer merges it; the servo thread does not know
   about navigation.
5. **Deadzone and per-axis sensitivity stay in the bridge** (`[spacemouse]`:
   `deadzone`, `sensitivity`, `deflection_max`, `rate_hz`, `poll_ms`, `socket`,
   `enabled`, default `false`), range-checked at load. spacenavd emits unbounded
   integers scaled by its own sensitivity, and the wire contract is `[-1, 1]`, so one
   divisor is unavoidable: `deflection_max`, default 350, is the spacenavd count that
   maps to 1.0. **It is a tunable, not a property of any device**, and has not been
   checked against a running daemon here.
6. **Schema (additive to v1).** `nav_subscribe` and `nav_unsubscribe` from the
   authenticated controller; `nav_ok {enabled, hz, present, reason}` in reply; `nav`
   frames `{seq, t, tx…rz, buttons, present}`; and `nav_input` on `status` while
   subscribed. `present` is in the stream, not only in `nav_ok`, because a device can
   come and go after a client subscribed. A client that never subscribes receives
   nothing new.
7. **Streaming rule.** At `rate_hz` (about 60 Hz) a frame is sent while any axis is
   non-zero, even if unchanged, so the client can treat a gap as a release. When all
   axes are zero, a frame is sent only if something changed since the last one: exactly
   one all-zero frame on release, one when input appears and one when it goes (each
   with the new `present`), one frame per button change, and nothing from a device at
   rest. Nav sends do not reset the connection's idle timer.
8. **Same socket, same authentication.** The nav ticker exists only inside the
   authenticated controller's session loop. `nav_subscribe` before authentication is
   `unauthorised` and closes; a second client is closed `busy` before it can subscribe.
9. **Navigation never touches the force path.** The nav code takes no reference to the
   servo thread, the inbox, the gate or the coupling. A client moves its own view; the
   target it then sends is limited as for any scene motion (anchor slew 0.25 m/s, force
   rate 20 N/s, force cap), exactly as ADR-0001 already requires.

## What is evidence and what is unknown

The protocol was read from the FreeSpacenav sources (libspnav
`476a17b6087146f95799e3a2a12b4d13cb4d2818`, spacenavd
`05cc67938e77ac7ad9c5b34267984d944cfd57dd`), which this work fetched and read. It was
**not run against a real spacenavd or a physical SpaceMouse**; the tests run against a
fake daemon written from the same reading, so they prove the client matches the reading,
not that the reading matches a daemon. Specifically unverified:

- spacenavd's actual output range at its default sensitivity (hence `deflection_max`);
- axis signs and which physical motion each axis is (the frame carries spacenavd's own
  signs; negative sensitivities fix a wrong sign without code);
- that a real daemon answers `REQ_DEV_NAXES` with a failure status when no device is
  attached (read in `proto_unix.c`: it does, through `get_client_device`);
- the behaviour with a spacenavd older than protocol 1: the client waits 300 ms for the
  handshake reply, restores the daemon's sensitivity to 1.0 as libspnav does, and
  carries on with protocol 0, where device changes are not reported and `present`
  means only that the daemon is up;
- the socket's permissions on the haptics machine (an owner action).

## Consequences

- A person at the bench steers the client's viewport from the bench machine. The client
  needs a source that turns `nav` frames into its existing navigation input; that is a
  decision for the client's own records.
- The machine needs spacenavd installed and running, which is a package and a service,
  not a device-permission rule. Without it the bridge works and reports why.
- Subscribing is per connection and the controller is the only connection that can
  subscribe.
- A silent client still times out after the idle timeout even while nav frames flow, by
  design; it must keep sending `target`.
- A held deflection is refreshed at the full rate, about 60 small frames a second.
- The bridge's deadzone stacks on spacenavd's and on any deadzone the client applies.
  Set the bridge's to zero to leave the others alone.

## Verification

Hardware-free, in `cargo test --workspace`:

- Wire: every frame shape, replies and unused types, the handshake constant.
- Client against a fake spacenavd on a Unix socket (`apps/spnav-client/tests/daemon.rs`):
  motion with latching, button press and release, hostile button numbers, no daemon, a
  stale socket file, a daemon that appears later, a device absent at connect, an unplug
  and replug mid-stream, the daemon dying mid-stream and reconnecting, a halted daemon,
  a protocol-0 daemon, a frame split across reads, and a clean stop.
- Filter: deadzone edge, full-deflection and over-range clamping, per-axis and negative
  sensitivity, validation.
- Streaming rule: rest sends nothing, a held deflection streams then ends in exactly one
  zero frame, button-only changes, input going away mid-deflection.
- Over a real local WebSocket (`apps/visionclaw-bridge/tests/nav_e2e.rs`): an
  authenticated subscriber gets frames at no more than the rate and one zero frame; an
  unauthenticated client, a client asking before authenticating and a stranger's key get
  nothing; a client that never subscribes sees only `status`; a `busy` client gets
  nothing; no device, hot-plug and unplug; an absent spacenavd, and one that dies after
  subscribing, reported in `nav_ok` and `status`; a bridge without `[spacemouse]` answers
  disabled; unsubscribe stops frames.
- Force: with the arm engaged on a target well under the cap, a fully deflected
  SpaceMouse (all six axes, two buttons) leaves the anchor and `target_seq` alone, keeps
  the force within 0.02 N of its settled value and every per-sample change within the
  force-rate limit; a second test has a client integrate the frames into a moving view
  over a fixed scene point and checks the force-rate limit, the cap and the anchor slew
  limit.
- Activation is inactive: no real spacenavd or SpaceMouse has been used.
