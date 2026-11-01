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
