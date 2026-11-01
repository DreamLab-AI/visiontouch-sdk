---
id: ADR-0001
title: Serve the arm over a WebSocket with a local virtual coupling
date: 2026-11-01
decision_status: proposed
implementation_status: complete
activation_status: inactive
supersedes: []
superseded_by: []
verified_paths: [crates/bridge/src, crates/bridge/tests/e2e.rs, crates/bridge/bridge.example.toml]
review_trigger: the first attended session with the arm on the bridge, any change to the coupling limits, a client that needs a coupling the bridge does not offer, or a request for a second simultaneous controller
---

# ADR-0001: Serve the arm over a WebSocket with a local virtual coupling

This is the first decision record in this repository; it starts `docs/adr/`.

## Context

The SDK's `Unit` must be serviced every 2.4 ms and drops output if a request is older
than 25 ms. Lab applications that want the arm are increasingly not on the machine it is
plugged into: a browser application on a desktop wants a single point of contact, a
held object it can feel, while a separate device steers the view. Such an application
can reach the arm only over the network, where delivery is 10 to 30 Hz with tens of
milliseconds of jitter and can stop at any moment.

Sending force over that link would hand the motors the link's jitter and gaps. Sending
pen pose one way and a *target* the other way, and closing the force loop beside the
arm, keeps the 400 Hz loop local and turns network trouble into a slower-moving anchor,
not a rougher force.

## Decision

A new crate, `visiontouch-bridge` (library and binary), in this workspace. `Unit` and
the other crates are unchanged.

1. **Local coupling.** The bridge runs `Unit` on its own OS thread at the SDK's pace.
   Each tick it computes `F = -k·(pen − anchor) − c·(v_pen − v_anchor)`, damping capped
   on its own, total capped, from the pen just measured. The anchor follows the
   client's target through a workspace clamp, a slew limit and a low-pass, which
   interpolates a 10 to 30 Hz stream into a smooth path without a playout delay.
2. **Limits.** Stiffness is at most half the unit's stable 2 N/mm (default 0.4 N/mm),
   because the target is the far end of a delayed loop through the network and the
   client's own simulation, and a stiff coupling feeds that loop's energy back into the
   hand. Force is capped at the 1.41 N continuous level by default, configurable to the
   peak; `Unit`'s heat budget applies on top.
3. **Fail-safe gate.** Force needs, on every tick: an authenticated controller
   connected; an `arm` from it and the device armed; the unit registered; no latched
   fault; a target younger than the target timeout (default 200 ms, configurable from
   50 to 250 ms); and the deadman. A socket close, a malformed message (also closing the
   connection) or silence past the idle timeout releases control and disarms. When
   force becomes allowed the anchor starts at the pen and the force ramps from zero.
   The SDK's 25 ms rule stays the inner guard.
4. **Deadman.** The pen button, held, and only after it has been seen released since
   the last arm, reconnect or fault, as the README requires of every integrator.
5. **Authentication.** On connect the bridge sends a 32-byte random challenge; the
   client must answer first, within the auth timeout, with a Nostr event of kind 22242
   (the NIP-42 client-authentication kind) that carries the challenge and the bridge
   URL, signed by a key in the configured allowlist. Verification uses the maintained
   `nostr` crate; nothing cryptographic is implemented here. A NIP-98 header was
   considered and rejected: browsers cannot set headers on a WebSocket upgrade, so the
   event would have to travel in the URL, logged and replayable within its time window,
   where a per-connection nonce makes every signature single-use. One controller at a
   time; a second authenticated client is told `busy`. Taking control never arms.
6. **Network policy.** The listen address is set in the configuration. A wildcard or
   public address, and any non-loopback, non-private, non-link-local peer, is refused
   unless `allow_public = true`. An optional browser `Origin` allowlist guards against a
   page the operator did not intend. Frames are capped at 8 KiB.
7. **Schema.** Versioned JSON text frames, `"v": 1`. In: `auth`, `arm`, `disarm`,
   `target {seq, p | null}`. Out: `hello {challenge, limits}`, `auth_ok`, `status`
   (pen, button, deadman, arm, gate reason, anchor, force, heat, target age), `error`.
   Unknown types or fields, another version, binary frames and non-finite or
   out-of-range numbers are malformed. Targets are metres in the model frame; mapping
   to and from a client's scene is the client's job.

## Consequences

- A client never commands force directly, so it cannot make the arm push harder than
  the coupling and its caps allow, and a network fault can only remove force.
- What the hand feels is the spring between the pen and the client's target. A client
  that wants the user to feel its simulation must let that simulation move the target
  away from the pen; a target that simply copies the pen yields only a light drag
  proportional to the round-trip delay.
- Transport is plain `ws://`. A client page served over `https` needs a TLS proxy in
  front of the bridge, or a later TLS option.
- The client's clock does not matter: the bridge stamps targets on arrival, so the
  target timeout measures the link as the bridge sees it.
- Registration stays a local act: the operator clicks the pen in its seat when the
  bridge starts. The network can arm and disarm, never register.

## Verification

`cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings` and
`RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --workspace` pass. Unit tests cover the
coupling maths and every cap, the target filter (slew, clamp, staircase smoothing), each
gate rule including the deadman latch and its reset on link loss, disarm, fault, device
loss and registration, the servo loop against a simulated unit (no force without a
controller, arming once per request, ramp-in from zero, stale-target cut-off within one
tick of the timeout, link loss disarming, refusal latching, the cap under adversarial
targets, clean thread stop and fault exit), the schema's strictness, the configuration
limits, the LAN policy and every authentication refusal. An end-to-end suite runs a real
WebSocket on 127.0.0.1 against the simulated unit: an unauthenticated `arm`, an unknown
key and a replayed signature are refused; the happy path delivers force towards the
target within the cap; a dropped socket, a clean close, a stalled target, a malformed
text or binary message, idle silence, `disarm` and a null target each stop force; a
second controller gets `busy` without disturbing the first. The guards were also
removed one at a time and the matching tests seen to fail. Activation is inactive: the
bridge has not run on the arm, and no test opens the port.
