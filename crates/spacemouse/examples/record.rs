//! Record raw reports from the first SpaceMouse found, for fixtures and to check the
//! decoder against a real device.
//!
//! This example **opens a real `/dev/hidrawN`**. Run it by hand on the machine the
//! SpaceMouse is plugged into, move the puck, press the buttons, then press Ctrl+C:
//!
//! ```text
//! cargo run -p visiontouch-spacemouse --example record
//! ```
//!
//! It prints the device, the path of its report descriptor (dump that file with
//! `xxd` and keep it with the fixtures), then one line per report: the hex bytes and
//! what the decoder made of it. Reports the decoder does not know print `?`; those are
//! the ones to send back.
use std::io::Read;
use std::time::Duration;
use visiontouch_spacemouse::device::{HidrawOpener, Opener};
use visiontouch_spacemouse::report::parse;

fn main() {
    let opener = HidrawOpener::system();
    let Some(info) = opener.scan().into_iter().next() else {
        eprintln!("no supported SpaceMouse found under /sys/class/hidraw");
        return;
    };
    let node = info
        .path
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("hidrawN");
    println!(
        "{} {:04x}:{:04x} at {}",
        info.name,
        info.vendor,
        info.product,
        info.path.display()
    );
    println!("descriptor: /sys/class/hidraw/{node}/device/report_descriptor");
    let mut reader = match opener.open(&info) {
        Ok(r) => r,
        Err(e) => {
            eprintln!(
                "cannot open {}: {e} (is there a udev rule for read access?)",
                info.path.display()
            );
            return;
        }
    };
    let mut buf = [0u8; 64];
    loop {
        match reader.read(&mut buf) {
            Ok(0) => return,
            Ok(n) => {
                let hex: Vec<String> = buf[..n].iter().map(|b| format!("{b:02x}")).collect();
                match parse(&buf[..n]) {
                    Some(r) => println!("{}  {r:?}", hex.join(" ")),
                    None => println!("{}  ?", hex.join(" ")),
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(2));
            }
            Err(e) => {
                eprintln!("read: {e}");
                return;
            }
        }
    }
}
