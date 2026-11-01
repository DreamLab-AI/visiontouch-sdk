# visiontouch-sdk

A small Rust SDK for the lab's 1990s parallel-port haptic arm (a PHANTOM 1.0A on an EPP
parallel port), for use from other lab projects on Linux.

Connect, click once with the pen in its seat, then read the pen position and button and
command force in real newtons, up to about 5.3 N, with a heat budget that protects the
motors.

This SDK is built for **one specific, lawfully owned unit**. It uses that unit's measured
gain and limits. It is not a general or supported driver for other devices. It is
published as a clean-room showcase: a working driver for a 25-year-old device built
from observation and measurement, largely through agentic engineering.

## Crates

| Crate | What it does |
|---|---|
| `visiontouch-hal` | Linux ppdev transport: software EPP cycles, 16-byte input packets, 8-byte motor packets with acknowledgement, the zero-only arming handshake, a transport clamp on every motor channel |
| `visiontouch-device` | The SDK: encoder state and unwrapping, reset-pose registration, kinematics, a force guard with expiry and limits, and `unit`, the single-unit profile most projects should use |
| `visiontouch-effects` | Hardware-free force models (cube, plane, water, balloon, springs, viscous field, detents) for building haptic scenes |
| `visiontouch-bridge` | A WebSocket bridge that serves the arm to one authenticated network client: the client sends a target, the bridge computes the force locally |

## Requirements

- Linux with the `parport_pc` and `ppdev` kernel modules loaded, and the arm on
  `/dev/parport0`.
- Read and write access to `/dev/parport0`. Usually this means being in the `lp` group:
  `sudo usermod -aG lp $USER`, then log in again.
- Rust 1.85 or later (edition 2024).

## Add it to a project

```toml
[dependencies]
visiontouch-device = { git = "https://github.com/DreamLab-AI/visiontouch-sdk" }
```

A local checkout also works:
`visiontouch-device = { path = "../visiontouch-sdk/crates/device" }`.

## Quick start

```rust,no_run
use visiontouch_device::unit::Unit;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    visiontouch_device::shutdown::install()?; // Ctrl+C turns output off cleanly
    let mut unit = Unit::connect("/dev/parport0")?;

    // Rest the pen in its seat (the reset pose), then:
    unit.register_here()?; // once per connection
    unit.arm()?;

    while !visiontouch_device::shutdown::requested() {
        let s = unit.service()?; // call every ~2.4 ms; never pause while armed
        let Some(p) = s.position_m else { continue };
        // A 2 N/mm floor 30 mm below the seat, live only while the pen button is held.
        let depth = -0.030 - p[1];
        let f = if s.button && depth > 0. { 2000. * depth } else { 0. };
        unit.set_force([0., f, 0.]); // delivered newtons
    }
    unit.close()?;
    Ok(())
}
```

A complete version, with a click to register and a live readout, is in
`crates/device/examples/unit-floor.rs`:

```sh
cargo run -p visiontouch-device --example unit-floor
```

Run the servo loop on its own thread and keep it going. The rest of your program (a
renderer, a network client) should exchange the latest position and desired force with
that thread.

## How `Unit` behaves

| | |
|---|---|
| Coordinates | Metres in the device's model frame, relative to the registered reset pose. Y is up (gravity pulls towards -Y) |
| Registration | `register_here` makes the current pose the origin. It is needed once per connection, because the encoders count from wherever the arm was at power-on. Until then `position_m` is `None` and `arm` refuses |
| Force units | Delivered newtons. Converted at 0.0026 N per count on the largest motor channel, measured with the pen pushing down on a scale. Accurate within a few percent pushing down near the measured poses, approximate in other directions |
| Peak force | About 5.3 N (`PEAK_N`), the full output range |
| Heat budget | See below. Up to about 1.41 N (`CONTINUOUS_N`) can be held indefinitely. Full force lasts about 40 s from cool. Once the budget is spent, output is capped at 1.41 N until the motors cool |
| Stale output | If a request is older than 25 ms, or the loop stalls that long, output goes to zero at once. Keep calling `service` |
| Faults | Any error from `service` is a transport fault. Output has been shut down; close and reconnect |
| Deadman | **Not built in. Supply one.** The held pen button (`State::button`) works well. It reads as pressed if the wiring fails, so only start output after you have seen it released |
| Wall stiffness | About 2 N/mm is stable (`STABLE_STIFFNESS_N_MM`). The loop runs at about 400 Hz and force arrives two samples late, so stiffer walls bounce |
| Torque | None. The pen and wrist encoders are inputs only |

### Heat budget

The motors can deliver far more than their continuous rating, but only briefly. `Heat`
keeps a running total of force squared over time, which decays with a 10-minute time
constant:

- **Low continuous force is free.** Below about 1.41 N the total never reaches the
  1200 N²s budget, however long it is held. This matches the published continuous
  rating of about 1.4 N.
- **High impulse is fine.** Any force up to the 5.3 N peak is allowed while the total is
  under budget. From cool, that is about 40 s of full force, or far longer in short
  bursts.
- **When the budget is spent**, the force magnitude is capped at 1.41 N, not cut off, and
  full force returns as the total decays.

The budget is half the heat of a published test: 2 N held for 10 minutes raised the
casing by 34.5 °C on the stock amplifier. It is a model from published ratings, not a
temperature sensor. `State::heat_n2s` and `State::heat_limited` report it every loop.

## Measured characteristics of this unit

Recorded in attended sessions on 1 November 2026, with the pen pressing down on a
digital kitchen scale.

| Output (largest channel) | Scale reading |
|---|---|
| 959 counts | 250 g |
| 1593 counts | 460 g |
| 2047 counts (full) | 596 to 600 g at the peak of a ramp, 542 g settled during a hold |

- Full output delivers about 5.3 to 5.9 N.
- Repeat readings agreed within 1 percent, at two different arm poses.
- The settled 542 g sets the SDK's conversion factor, since it is the conservative
  figure.
- On this loop, a virtual wall is stable at about 2 N/mm and bounces at 5 N/mm and
  above.

## Network bridge

`visiontouch-bridge` runs on the machine the arm is plugged into and serves it to one
client over a WebSocket, for example a browser application on another desktop. **The
servo loop never crosses the network.** The bridge runs `Unit` on its own thread at
about 400 Hz. The client sends a *target* (where the thing it holds is) at 20 to 30 Hz;
the bridge pulls the pen towards it with a local virtual coupling,
`F = -k·(pen − target) − c·(v_pen − v_target)`, from the pen it has just measured, and
streams back the pen pose, the button, the arm state and the heat budget.

```sh
cp crates/bridge/bridge.example.toml bridge.toml   # set bind, url, authorised_keys, allowed_origins
cargo run -p visiontouch-bridge -- --config bridge.toml          # the arm
cargo run -p visiontouch-bridge -- --config bridge.toml --fake   # a simulated arm
```

With the arm, rest the pen in its seat and click the button once to register. Output
stays off until an authenticated client sends `arm`, and then flows only while the pen
button is held, after it has been seen released.

| | |
|---|---|
| Schema | Versioned JSON text frames (`"v": 1`): `auth`, `arm`, `disarm`, `target` in; `hello`, `auth_ok`, `status`, `error` out. Documented in the crate's `protocol` module |
| Authentication | The bridge sends a random challenge; the client answers with a Nostr event of kind 22242 (NIP-42) naming the bridge's `url`, signed by a key in `authorised_keys`. One controller at a time |
| Network | Binds to a loopback or LAN address and accepts LAN peers only, unless `allow_public = true`. A browser page must be listed in `allowed_origins`. Plain `ws://`: the link must be trusted, and a page served over `https` needs a TLS proxy in front |
| Fail-safe | Force stops on the next servo tick when the socket closes, when a malformed message arrives (the connection is also closed), when no message arrives within the idle timeout, or when no target arrives within the target timeout (default 200 ms). Zero goes out before the next service, and a servo stall over 10 ms also cuts force and needs a fresh button release. The SDK's 25 ms stale-output rule stays underneath |
| Limits | Stiffness at most 1 N/mm (half the stable 2 N/mm; default 0.4), force capped at the 1.41 N continuous level by default (configurable to the peak), and the heat budget on top. Force ramps in from zero when it engages, with the anchor starting at the pen, and while engaged it changes by at most 20 N/s (`force_rate_n_s`), so no target motion can step it; cutting force is immediate |
| Targets | Metres in the model frame, or in the client's scene with a `view` transform (`origin`, unit-quaternion `rotation`, `scale` in scene units per metre) that places the pen's workspace in that scene. The bridge maps scene targets through the view, reports the pen in scene coordinates too, clamps targets to a sphere about the origin and smooths them with a slew limit and a low-pass. A moving view moves the target, so the hand feels scene motion |

The design and its trade-offs are recorded in
[docs/adr/0001-network-bridge.md](docs/adr/0001-network-bridge.md).

## Lower-level API

Most projects only need `unit`. Underneath it:

- `Device`: connect, `sample`, `register_reset`, `enable_feedback`, `request`, `close`.
  `sample` reads the encoders and services the guard. A demand must be re-sent at least
  every 25 ms.
- `Demand::Force` uses the reference kinematic model's newtons, which on this unit
  over-state delivered force about threefold. It is capped by `ForceLimits` (default
  0.5 N model force and 96 counts; ceilings 2 N and 300 counts).
- `Demand::Channels` drives raw motor channels for liveness tests, capped at 96 counts.
- `Demand::Characterise` and `Demand::CharacteriseChannels` give count-based output up
  to the full 2047 counts. They only work inside a session opened with
  `Device::enable_characterisation`, which `Unit::arm` uses.
- `EncoderState` exposes all six raw and unwrapped encoder channels, including the
  three wrist and pen encoders, and the button.
- `position_from_counts` and `motor_demand_counts` expose the kinematics.

`cargo doc --open` builds the full API documentation.

## Safety

The arm pushes on a hand with up to about 6 N.

- Output must be armed explicitly, and drops to zero within 25 ms if the program stops
  servicing it.
- The transport clamps every motor channel. The clamp is 300 counts normally, and the
  full range is only reachable inside an armed full-span session. The clamp goes back to
  300 whenever output stops.
- Software cannot cover a kernel stall or a killed process. **Keep the amplifier's power
  switch within reach**, use a deadman, and keep the workspace clear while output is
  armed.

## Building and testing

```sh
cargo build
cargo test        # no hardware needed; nothing in the tests opens the port
cargo clippy --all-targets -- -D warnings
```

## Provenance

This SDK was written independently. Its sources were observation of the device's own
port traffic and behaviour, published literature about this device family, and attended
measurements of this unit. It contains no vendor code, firmware, headers or documents,
and nothing derived from decompilation. It is not affiliated with or endorsed by the
original manufacturer or its successors; product names belong to their owners and are
used only to identify the hardware.

Contributions must keep it that way: please do not add anything taken from vendor
software, documentation or reverse-engineered binaries.

## Licence

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE)), or
- MIT licence ([LICENSE-MIT](LICENSE-MIT))

at your option. Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in this work, as defined in the Apache-2.0 licence, shall be
dual licensed as above, without any additional terms or conditions.

The software drives a motorised device that pushes on a hand. It is provided as is,
without warranty of any kind; see the licence texts. Use it with a deadman and the
amplifier power switch within reach.
