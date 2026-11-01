//! `visionclaw-bridge`: serve the arm over a WebSocket.
//!
//! ```text
//! visionclaw-bridge --config bridge.toml          # the real unit
//! visionclaw-bridge --config bridge.toml --fake   # a simulated unit, no hardware
//! ```
//!
//! With the real unit, rest the pen in its seat and click the pen button once to
//! register; the bridge then serves clients with output off until a client arms.
//! Ctrl+C stops it and shuts output down.
use spnav_client::Monitor;
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use visionclaw_bridge::config::Config;
use visionclaw_bridge::fake::FakeUnit;
use visionclaw_bridge::nav::NavSource;
use visionclaw_bridge::server::Bridge;
use visionclaw_bridge::servo::{self, Haptic, Inbox, SERVO_PERIOD, Servo};
use visiontouch_device::shutdown;
use visiontouch_device::unit::Unit;

const USAGE: &str = "usage: visionclaw-bridge --config <file.toml> [--fake]";

fn main() {
    if let Err(e) = run() {
        eprintln!("visionclaw-bridge: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let mut config_path = None;
    let mut fake = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--config" => config_path = args.next(),
            "--fake" => fake = true,
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            other => return Err(format!("unknown argument `{other}`\n{USAGE}")),
        }
    }
    let path = config_path.ok_or(USAGE)?;
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{path}: {e}"))?;
    let config = Config::from_toml(&text).map_err(|e| format!("{path}: {e}"))?;
    shutdown::install().map_err(|e| format!("signal handler: {e}"))?;

    if fake {
        let unit = FakeUnit::registered();
        let script = unit.clone();
        std::thread::spawn(move || fake_hand(script));
        eprintln!("visionclaw-bridge: simulated unit (no hardware)");
        serve(unit, config).map(|_| ())
    } else {
        let mut unit = Unit::connect(&config.device)
            .map_err(|e| format!("{}: {e}", config.device.display()))?;
        if !register_by_click(&mut unit)? {
            return unit.close().map_err(|e| e.to_string());
        }
        let unit = serve(unit, config)?;
        unit.close().map_err(|e| format!("close: {e}"))
    }
}

/// Wait for a release then a press with the pen seated, then register. Output stays off.
fn register_by_click(unit: &mut Unit) -> Result<bool, String> {
    eprintln!("Rest the pen in its seat, then click the pen button once to register.");
    let mut released = false;
    loop {
        let s = unit.service().map_err(|e| format!("service: {e}"))?;
        released |= !s.button;
        if released && s.button {
            break;
        }
        if shutdown::requested() {
            return Ok(false);
        }
        std::thread::sleep(SERVO_PERIOD);
    }
    unit.register_here().map_err(|e| e.to_string())?;
    eprintln!("Registered. Output is off until a client arms; hold the pen button for force.");
    Ok(true)
}

/// A simulated hand: button released for a second, then held; the pen circles slowly.
fn fake_hand(unit: FakeUnit) {
    let start = Instant::now();
    while !shutdown::requested() {
        let t = start.elapsed().as_secs_f64();
        let a = std::f64::consts::TAU * 0.1 * t;
        unit.set_pen([0.02 * a.cos(), 0., 0.02 * a.sin()]);
        unit.set_button(t > 1.);
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Run the servo thread and the server until Ctrl+C or a device fault.
fn serve<H: Haptic + Send + 'static>(unit: H, config: Config) -> Result<H, String> {
    let start = Instant::now();
    let inbox = Arc::new(Mutex::new(Inbox::default()));
    let mut initial = servo::fault_status(0);
    initial.device_fault = false;
    initial.reason = visionclaw_bridge::gate::Reason::NoLink;
    let (status_tx, status_rx) = tokio::sync::watch::channel(initial);
    let stop = Arc::new(AtomicBool::new(false));
    let servo_thread = servo::spawn(
        Servo::new(unit, &config, start)?,
        inbox.clone(),
        status_tx,
        stop.clone(),
        SERVO_PERIOD,
    )
    .map_err(|e| format!("servo thread: {e}"))?;

    let bind = config.bind;
    // SpaceMouse navigation is optional and fails soft: its monitor thread has no link
    // to the servo thread or the shutdown watcher below, and no spacenavd or no device
    // just means no navigation frames. The monitor must outlive the server.
    let mouse = if config.spacemouse.enabled {
        match Monitor::spawn(
            config.spacemouse.socket.clone(),
            Duration::from_millis(config.spacemouse.poll_ms),
        ) {
            Ok(m) => Some(m),
            Err(e) => {
                eprintln!("visionclaw-bridge: spacemouse disabled: {e}");
                None
            }
        }
    } else {
        None
    };
    let source = mouse
        .as_ref()
        .map(|m| NavSource::new(m.latest(), config.spacemouse.settings()));
    let bridge = Bridge::with_nav(config, inbox, status_rx.clone(), source)?;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .map_err(|e| format!("runtime: {e}"))?;
    let served: io::Result<()> = runtime.block_on(async move {
        let listener = tokio::net::TcpListener::bind(bind).await?;
        eprintln!(
            "visionclaw-bridge: listening on ws://{}",
            listener.local_addr()?
        );
        let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_millis(50)).await;
                // A dropped sender means the servo thread has ended.
                let servo_gone = status_rx.has_changed().is_err();
                if shutdown::requested() || servo_gone || status_rx.borrow().device_fault {
                    let _ = shutdown_tx.send(true);
                    break;
                }
            }
        });
        bridge.serve(listener, shutdown_rx).await
    });
    drop(mouse);
    stop.store(true, Ordering::Relaxed);
    let (unit, ended) = servo_thread
        .join()
        .map_err(|_| "servo thread panicked".to_string())?;
    served.map_err(|e| format!("server: {e}"))?;
    ended.map_err(|e| format!("device fault, output is off; restart at the arm: {e}"))?;
    Ok(unit)
}
