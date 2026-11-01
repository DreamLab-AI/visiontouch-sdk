//! Single-unit profile example: a 2 N/mm floor 30 mm below the seat.
//!
//! 1. Run with the pen resting in its seat. Click the pen button once: that pose is
//!    registered as the origin and output is armed.
//! 2. Hold the pen button to energise the floor (deadman); release to cut it.
//! 3. Ctrl+C stops and shuts output down.
//!
//! Prints position, delivered-force estimate and heat about twice a second.
use std::time::{Duration, Instant};
use visiontouch_device::unit::{STABLE_STIFFNESS_N_MM, Unit};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    visiontouch_device::shutdown::install()?;
    let mut unit = Unit::connect("/dev/parport0")?;
    println!("Pen in its seat, then click the pen button once to register and arm.");
    // Wait for a release followed by a press, so a stuck button cannot register.
    let mut released = false;
    loop {
        let s = unit.service()?;
        released |= !s.button;
        if released && s.button {
            break;
        }
        if visiontouch_device::shutdown::requested() {
            return Ok(unit.close()?);
        }
        std::thread::sleep(Duration::from_micros(2400));
    }
    unit.register_here()?;
    unit.arm()?;
    println!("Armed. Hold the pen button to feel the floor 30 mm below the seat.");
    let k = STABLE_STIFFNESS_N_MM * 1000.;
    let mut shown = Instant::now();
    let mut next = Instant::now();
    while !visiontouch_device::shutdown::requested() {
        let s = unit.service()?;
        let force = match s.position_m {
            Some(p) if s.button => {
                let depth = -0.030 - p[1];
                if depth > 0. {
                    [0., k * depth, 0.]
                } else {
                    [0.; 3]
                }
            }
            _ => [0.; 3],
        };
        unit.set_force(force);
        if shown.elapsed() > Duration::from_millis(500) {
            shown = Instant::now();
            if let Some(p) = s.position_m {
                println!(
                    "y {:+6.1} mm  force {:4.2} N  heat {:5.0} N²s{}",
                    p[1] * 1e3,
                    s.applied_n,
                    s.heat_n2s,
                    if s.heat_limited {
                        "  (capped at continuous)"
                    } else {
                        ""
                    }
                );
            }
        }
        // Pace the loop near the 2.4 ms the transport sustains.
        next += Duration::from_micros(2400);
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        } else {
            next = Instant::now();
        }
    }
    unit.close()?;
    Ok(())
}
