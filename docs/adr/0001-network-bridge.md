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
   peak; `Unit`'s heat budget applies on top. While engaged, the force vector changes by
   at most `force_rate_n_s` (default 20 N/s, so 0 to 1.41 N takes about 70 ms); cutting
   force is never rate-limited.
3. **Fail-safe gate.** Force needs, on every tick: an authenticated controller
   connected; an `arm` from it and the device armed; the unit registered; no latched
   fault; a target younger than the target timeout (default 200 ms, configurable from
   50 to 250 ms); and the deadman. A socket close, a malformed message (also closing the
   connection) or silence past the idle timeout releases control and disarms. Zero is
   queued before the next device service whenever the link, the arm request or the
   target has gone, so a lost link never gets one more period of the old force. A
   servo tick more than 10 ms late counts as a stall: zero first, and the deadman must
   be released again. A panic in the servo loop is caught, zeroes and disarms, and ends
   the bridge. When force becomes allowed the anchor starts at the pen and the force
   ramps from zero. The SDK's 25 ms rule stays the inner guard.
4. **Deadman.** The pen button, held, and only after it has been seen released since
   the last arm request, reconnect, fault or stall, as the README requires of every
   integrator. Arm attempts are spaced at least 500 ms apart.
5. **Authentication.** On connect the bridge sends a 32-byte random challenge; the
   client must answer first, within the auth timeout, with a Nostr event of kind 22242
   (the NIP-42 client-authentication kind) that carries the challenge and the bridge
   URL (the configured `url`, required, so a challenge relayed through another
   service is refused), signed by a key in the configured allowlist. Verification uses the maintained
   `nostr` crate; nothing cryptographic is implemented here. A NIP-98 header was
   considered and rejected: browsers cannot set headers on a WebSocket upgrade, so the
   event would have to travel in the URL, logged and replayable within its time window,
   where a per-connection nonce makes every signature single-use. One controller at a
   time; a second authenticated client is told `busy`. Taking control never arms.
6. **Network policy.** The listen address is set in the configuration. A wildcard or
   public address, and any non-loopback, non-private, non-link-local peer, is refused
   unless `allow_public = true`. A request carrying a browser `Origin` is refused
   unless that origin is listed, so a page the operator did not choose cannot reach the
   arm through a permissive signer. Frames are capped at 8 KiB and connections at 8.
7. **Schema.** Versioned JSON text frames, `"v": 1`. In: `auth`, `arm`, `disarm`,
   `target {seq, p | null}`. Out: `hello {challenge, limits}`, `auth_ok`, `status`
   (pen, button, deadman, arm, gate reason, anchor, force, heat, target age), `error`.
   Unknown types or fields, another version, binary frames and non-finite or
   out-of-range numbers are malformed. A target is either metres in the model frame or,
   with a `view` in the same message, a point in the client's scene. The view is the
   similarity transform `p_scene = origin + scale·R(rotation)·p_device` (unit
   quaternion, scale in scene units per metre) into which the client folds its camera,
   navigation and any transform of its content. The bridge maps scene targets into the
   model frame with the inverse and reports the pen in scene coordinates as well.
8. **Scene motion.** Because the view arrives with every target, moving the view moves
   the target even when the held thing is still in the scene, and the hand feels it.
   Navigation alone never produces a force step: the anchor moves no faster than its
   slew limit, the force changes no faster than its rate limit, and both stay within
   the caps. A view jump (a teleport) becomes a ramp of about 70 ms to the cap.

## Consequences

- A client never commands force directly, so it cannot make the arm push harder than
  the coupling and its caps allow, and a network fault can only remove force.
- What the hand feels is the spring between the pen and the client's target. A client
  that wants the user to feel its simulation must let that simulation move the target
  away from the pen. A target that copies the pen through a delay `τ` yields a drag of
  about `k·v·τ`, and that is not light: at the default 0.4 N/mm with `τ` = 100–200 ms it
  reaches the 1.41 N cap at 2 to 4 cm/s of hand speed. (A pure delay is dissipative
  below `1/(2τ)` and adds energy above it.) Such a client must compensate the delay
  (send the simulation's offset from the hand position it answers to, added to the
  current pen) or run a much lower stiffness. A target mode in which the bridge itself
  adds a client-sent offset to the pen it has just measured would remove the residual
  link delay; it is a candidate for schema version 2.
- Transport is plain `ws://` and authentication covers the connection, not each
  message, so the LAN segment must be trusted: an attacker able to hijack the TCP
  stream after authentication could send targets (still bounded by the caps and the
  deadman). A client page served over `https` needs a TLS proxy in front of the
  bridge, which also closes that gap; a built-in TLS option is follow-up work.
- Eight silent unauthenticated sockets can hold every connection slot for the auth
  timeout; that is an availability cost on the LAN, not a safety one.
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
controller, arming once per request and rate-limited, ramp-in from zero, stale-target
cut-off within one tick of the timeout, zero sent before the service after link loss,
stall cut-off with a fresh release, a disarm and re-arm inside one tick restarting the
deadman, link loss disarming, refusal latching, the cap under adversarial targets, an
invalid configuration refused, clean thread stop and fault exit; a view teleport and a
2-turn-per-second view rotation each keep every per-tick force change within the rate
limit and the force within the cap, while the force cut stays immediate), the view
transform (axes, round trips, refusals), the schema's view rules, the schema's strictness, the configuration
limits, the LAN policy and every authentication refusal. An end-to-end suite runs a real
WebSocket on 127.0.0.1 against the simulated unit: an unauthenticated `arm`, an unknown
key, a replayed signature, a signature naming another URL and an unlisted browser
origin are refused; a scene target through a rotated view pulls along the mapped
direction and the pen comes back in scene coordinates; the happy path delivers force towards the
target within the cap; a dropped socket, a clean close, a stalled target, a malformed
text or binary message, idle silence, `disarm` and a null target each stop force; a
second controller gets `busy` without disturbing the first. The guards were also
removed one at a time and the matching tests seen to fail. Activation is inactive: the
bridge has not run on the arm, and no test opens the port. An independent security
review of the crate found no path to force without authentication or past the caps;
the defects it found (a servo panic going unnoticed, no stall cut-off, a one-period
window after link loss, a fast disarm and re-arm skipping the deadman reset, a leaked
connection slot on panic, unescaped client text in logs) are fixed and tested, and
its remaining hardening notes are recorded under Consequences.
