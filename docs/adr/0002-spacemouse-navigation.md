---
id: ADR-0002
title: Relay a USB SpaceMouse over the bridge for viewport navigation
date: 2026-11-01
decision_status: proposed
implementation_status: complete
activation_status: inactive
supersedes: []
superseded_by: []
verified_paths: [crates/spacemouse/src, crates/spacemouse/tests/models.rs, crates/bridge/src/nav.rs, crates/bridge/tests/nav_e2e.rs]
review_trigger: the first read of a real SpaceMouse on the haptics machine, a model whose reports the decoder does not recognise, a client that needs a different frame rate or axis convention, or any change that would let navigation reach the force path
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
reader must work without a desktop daemon. The stream rides the bridge's socket and its
authentication, not a second listener. And a view that moves must never be able to push
force: navigation is view input, and the force path is the bridge's job alone.

## Decision

1. **A new crate, `visiontouch-spacemouse`, reads the device directly.** It scans
   `/sys/class/hidraw`, matches vendor and product against a table, prefers the
   interface whose report descriptor opens with Generic Desktop / Multi-axis Controller
   (`05 01 09 08`), and reads `/dev/hidrawN` with non-blocking reads. There is **no
   dependency on `spacenavd`**, and no feature-gated fallback to it: hidraw access is
   practical (a udev rule gives read access, as the README shows) and `spacenavd` would
   add a daemon and a socket to a machine that otherwise runs one process. If a model
   proves unreadable through hidraw, a `spacenavd` fallback behind a feature is the
   remedy, and this ADR should be amended then.
2. **Reports are decoded by (report ID, payload length), not by product.** Report 1
   with 6 bytes is translation, report 1 with 12 bytes is all six axes, report 2 with 6
   bytes is rotation, report 3 with 1 to 8 bytes is a little-endian button mask, and
   everything else is ignored. One decoder serves every model, and each model is just a
   fixture.
3. **Supported models** (vendor:product from `usb.ids`, hwdata 0.412): SpaceMouse
   Compact `256f:c635`; SpaceMouse Wireless cabled `256f:c62e` and its receiver
   `256f:c62f`; the Universal Receiver `256f:c652`; SpaceMouse Pro `046d:c62b`; Pro
   Wireless `256f:c631` and receiver `256f:c632`; SpaceMouse Enterprise `256f:c633`. The
   SpacePilot family is not supported here.
4. **Deadzone and per-axis sensitivity are configuration.** `[spacemouse]` in the bridge
   configuration: `deadzone` (fraction of full scale), `sensitivity` (six multipliers;
   a negative sign inverts an axis), `full_scale` (raw count), `rate_hz`, `poll_ms`,
   `enabled` (default `false`). All are range-checked at load. After the deadzone the
   axis is rescaled so the output rises from zero at the edge, not with a jump.
5. **Hot-plug is a poll, and absence is a state.** One thread scans every `poll_ms`
   while no device is open, reads while one is, and publishes the latest filtered state
   to a shared handle. No device, an unreadable node, an unplug and a thread panic all
   leave a zero, not-present snapshot (a drop guard guarantees it). Nothing about the
   monitor can stop the bridge: it has no link to the servo thread or the shutdown
   watcher.
6. **Schema (additive to v1).** `{"v":1,"type":"nav_subscribe"}` and
   `{"v":1,"type":"nav_unsubscribe"}` from the authenticated controller;
   `{"v":1,"type":"nav_ok","enabled":bool,"hz":n,"present":bool}` in reply; and
   `{"v":1,"type":"nav","seq":n,"t":ms,"tx":…,"ty":…,"tz":…,"rx":…,"ry":…,"rz":…,"buttons":mask,"present":bool}`
   frames. `present` is in the stream, not only in `nav_ok`, because a device can be
   plugged in or removed after a client subscribed; without it the client could not
   tell. `hello`, `status` and every other existing message are unchanged. A client
   that never subscribes receives nothing new; a v1 client therefore cannot meet an
   unknown `type`.
7. **Streaming rule.** At `rate_hz` (about 60 Hz) a frame is sent while any axis is
   non-zero, even if unchanged, so the client can treat a gap as a release. When all
   axes are zero, a frame is sent only if something changed since the last one: so
   exactly one all-zero frame on release, one on plug and one on unplug (each with the
   new `present`), one frame per button change, and nothing from a device at rest. Nav sends do not reset the connection's idle timer;
   the client's 30 Hz `target` messages do.
8. **Same socket, same authentication.** The nav ticker exists only inside the
   authenticated controller's session loop. Before authentication there is no
   subscription to make (`nav_subscribe` as a first message is `unauthorised` and
   closes), and a second client is closed `busy` before it can subscribe.
9. **Navigation never touches the force path.** The nav code takes no reference to the
   servo thread, the inbox, the gate or the coupling. A client moves its own view; the
   target it then sends is limited as for any scene motion (anchor slew 0.25 m/s,
   force rate 20 N/s, force cap), exactly as ADR-0001 already requires for a view moving
   under a hold.

## What is evidence and what is unknown

The product IDs are from the `usb.ids` database. The report layouts (IDs 1, 2 and 3 and
their lengths) match what an existing browser (WebHID) decoder for these devices relies
on and what public drivers (spacenavd, pyspacemouse) are recalled to describe;
**none was checked against a physical device or a captured report here, and
the sources were not re-read in this work.** Specifically unverified:

- the report-descriptor prefix on each model, and which hidraw node carries motion on
  models that expose several (the scan falls back to accepting a node with no readable
  descriptor);
- the full-scale count (about ±350 is assumed; `full_scale` is configurable);
- axis signs and which physical motion each axis is (the frame carries the device's own
  signs; negative sensitivities fix a wrong sign without code);
- the button-mask width per model (up to 4 bytes are kept, which covers the Enterprise's
  31 buttons if it uses a plain mask) and whether the Enterprise or the wireless models
  send extra reports (battery, LED echo), which the decoder ignores;
- whether the wireless models send the same reports over Bluetooth;
- whether a running `spacenavd` interferes. It reads evdev, not hidraw, so it should
  not, but both can see the device.

## Consequences

- A person at the bench steers the client's viewport from the bench machine. The client
  needs a source that turns `nav` frames into its existing navigation input; that is a
  decision for the client's own records.
- Subscribing is per connection and the controller is the only connection that can
  subscribe. A `busy` client gets nothing and cannot observe the device.
- A silent client still times out after the idle timeout even while nav frames flow, by
  design; it must keep sending `target`.
- A held deflection is refreshed at the full rate, which costs about 60 small frames a
  second. That is the price of letting the client decide a gap means "released".
- The bridge's deadzone stacks on any deadzone the client applies. Set the bridge's to
  zero to leave the client's alone.
- Direct hidraw access needs a udev rule on the machine, which is an owner action.

## Verification

Hardware-free, in `cargo test --workspace`:

- Decoder: every report shape, malformed and unknown reports, state accumulation, and a
  fixture per supported model (`tests/models.rs`) through uevent scan, descriptor
  filtering and decoding.
- Filter: deadzone edge, full-scale and over-range clamping, per-axis and negative
  sensitivity, validation.
- Device and monitor: absent then present, present then unplugged then replugged (no
  stale deflection), unknown reports ignored, clean stop.
- Streaming rule: rest sends nothing, a held deflection streams then ends in exactly one
  zero frame, button-only changes, unplug mid-deflection.
- Over a real local WebSocket (`crates/bridge/tests/nav_e2e.rs`): an authenticated
  subscriber gets frames at no more than the rate and one zero frame; an
  unauthenticated client, a client asking for navigation before authenticating and a
  stranger's key get nothing; a v1 client that never subscribes sees only `status`; a
  `busy` client gets nothing; no device, hot-plug and unplug; a bridge without
  `[spacemouse]` answers disabled; unsubscribe stops frames.
- Force: with the arm engaged on a target well under the cap, a fully deflected
  SpaceMouse (all six axes, two buttons) leaves the anchor and `target_seq` alone,
  keeps the force within 0.02 N of its settled value and every per-sample change within
  the force-rate limit. A second test has a client integrate the frames into a moving
  view over a fixed scene point (a 2 m/s throw) and checks the force-rate limit, the
  cap and the anchor slew limit hold. The first test fails if navigation is wired into
  the target (checked by mutation), and the second fails if the force-rate limit is
  removed (checked by mutation).
- Activation is inactive: no real SpaceMouse has been read. The `record` example is
  provided for the owner to run on the device.
