//! Finding and opening SpaceMouse devices.
//!
//! [`Opener`] is the seam: the monitor asks it to [`scan`](Opener::scan) for devices
//! and to [`open`](Opener::open) one as a byte stream of input reports.
//! [`HidrawOpener`] is the Linux implementation; its sysfs and `/dev` roots are
//! parameters, so tests point it at a temporary directory and never touch a real node.
//!
//! # How a device is recognised
//!
//! For each `class/hidraw/hidrawN` the opener reads `device/uevent` and takes
//! `HID_ID=BBBB:VVVVVVVV:PPPPPPPP` and `HID_NAME`. The vendor and product must be in
//! [`SUPPORTED`]. When `device/report_descriptor` is readable it must also begin with
//! the Generic Desktop page and the Multi-axis Controller usage (`05 01 09 08`), which
//! picks the motion interface out of a device that exposes several hidraw nodes. An
//! unreadable or absent descriptor does not exclude a node.
//!
//! The vendor and product IDs below are from the `usb.ids` database (hwdata 0.412). The
//! descriptor prefix is the HID Usage Tables definition; that the real devices lead
//! with it has not been checked here.
//!
//! The SpacePilot family (`046d:c625`, `046d:c629`) is deliberately absent: it has its
//! own browser path.
use std::fs::{self, File, OpenOptions};
use std::io::{self, Read};
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};

/// A supported device: vendor ID, product ID and model name.
pub type Model = (u16, u16, &'static str);

/// Supported models (`usb.ids`). Wireless models appear once per connection route.
pub const SUPPORTED: &[Model] = &[
    (0x256f, 0xc635, "SpaceMouse Compact"),
    (0x256f, 0xc62e, "SpaceMouse Wireless (cabled)"),
    (0x256f, 0xc62f, "SpaceMouse Wireless Receiver"),
    (0x256f, 0xc652, "Universal Receiver"),
    (0x046d, 0xc62b, "SpaceMouse Pro"),
    (0x256f, 0xc631, "SpaceMouse Pro Wireless (cabled)"),
    (0x256f, 0xc632, "SpaceMouse Pro Wireless Receiver"),
    (0x256f, 0xc633, "SpaceMouse Enterprise"),
];

/// A device the opener found.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceInfo {
    /// The node to open, e.g. `/dev/hidraw3`.
    pub path: PathBuf,
    /// USB vendor ID.
    pub vendor: u16,
    /// USB product ID.
    pub product: u16,
    /// The kernel's name for the device.
    pub name: String,
}

/// Finds and opens devices. Implementations must not block for long in `scan`.
pub trait Opener: Send + Sync {
    /// The supported devices currently present, motion interfaces only.
    fn scan(&self) -> Vec<DeviceInfo>;
    /// Open one for reading. The reader must not block indefinitely: it returns
    /// [`io::ErrorKind::WouldBlock`] when no report is waiting, `Ok(0)` or an error
    /// once the device has gone.
    fn open(&self, device: &DeviceInfo) -> io::Result<Box<dyn Read + Send>>;
}

/// The Linux hidraw implementation of [`Opener`].
#[derive(Clone, Debug)]
pub struct HidrawOpener {
    sysfs: PathBuf,
    dev: PathBuf,
}

impl HidrawOpener {
    /// Use the real `/sys` and `/dev`.
    pub fn system() -> Self {
        Self::with_roots("/sys", "/dev")
    }

    /// Use other roots, for fixtures.
    pub fn with_roots(sysfs: impl Into<PathBuf>, dev: impl Into<PathBuf>) -> Self {
        Self {
            sysfs: sysfs.into(),
            dev: dev.into(),
        }
    }
}

/// Parse a `uevent` file: the `(vendor, product, name)` of a HID device.
pub fn parse_uevent(text: &str) -> Option<(u16, u16, String)> {
    let mut ids = None;
    let mut name = String::new();
    for line in text.lines() {
        if let Some(v) = line.strip_prefix("HID_ID=") {
            let mut parts = v.split(':');
            let _bus = parts.next()?;
            let vendor = u32::from_str_radix(parts.next()?, 16).ok()?;
            let product = u32::from_str_radix(parts.next()?, 16).ok()?;
            ids = Some((u16::try_from(vendor).ok()?, u16::try_from(product).ok()?));
        } else if let Some(v) = line.strip_prefix("HID_NAME=") {
            name = v.to_owned();
        }
    }
    ids.map(|(v, p)| (v, p, name))
}

/// Whether a report descriptor opens with Generic Desktop / Multi-axis Controller.
pub fn is_multi_axis(descriptor: &[u8]) -> bool {
    descriptor.starts_with(&[0x05, 0x01, 0x09, 0x08])
}

impl Opener for HidrawOpener {
    fn scan(&self) -> Vec<DeviceInfo> {
        let class = self.sysfs.join("class/hidraw");
        let Ok(entries) = fs::read_dir(&class) else {
            return Vec::new();
        };
        let mut found = Vec::new();
        for entry in entries.flatten() {
            let node = entry.file_name();
            let Some(node_name) = node.to_str().filter(|n| n.starts_with("hidraw")) else {
                continue;
            };
            let dir = entry.path().join("device");
            let Ok(uevent) = fs::read_to_string(dir.join("uevent")) else {
                continue;
            };
            let Some((vendor, product, name)) = parse_uevent(&uevent) else {
                continue;
            };
            if !SUPPORTED.iter().any(|m| m.0 == vendor && m.1 == product) {
                continue;
            }
            if let Ok(desc) = fs::read(dir.join("report_descriptor"))
                && !is_multi_axis(&desc)
            {
                continue;
            }
            found.push(DeviceInfo {
                path: self.dev.join(node_name),
                vendor,
                product,
                name,
            });
        }
        found.sort_by(|a, b| a.path.cmp(&b.path));
        found
    }

    fn open(&self, device: &DeviceInfo) -> io::Result<Box<dyn Read + Send>> {
        open_nonblocking(&device.path).map(|f| Box::new(f) as Box<dyn Read + Send>)
    }
}

fn open_nonblocking(path: &Path) -> io::Result<File> {
    OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NONBLOCK)
        .open(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(root: &Path, n: u32, hid_id: &str, name: &str, descriptor: Option<&[u8]>) {
        let dir = root.join(format!("class/hidraw/hidraw{n}/device"));
        fs::create_dir_all(&dir).unwrap();
        fs::write(
            dir.join("uevent"),
            format!("DRIVER=hid-generic\nHID_ID={hid_id}\nHID_NAME={name}\nHID_PHYS=usb-0000\n"),
        )
        .unwrap();
        if let Some(d) = descriptor {
            fs::write(dir.join("report_descriptor"), d).unwrap();
        }
    }

    fn scratch(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("vt-sm-{tag}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn uevent_parsing_reads_ids_and_name() {
        let t =
            "DRIVER=x\nHID_ID=0003:0000256F:0000C635\nHID_NAME=3Dconnexion SpaceMouse Compact\n";
        assert_eq!(
            parse_uevent(t),
            Some((0x256f, 0xc635, "3Dconnexion SpaceMouse Compact".into()))
        );
        assert_eq!(parse_uevent("HID_NAME=x"), None);
        assert_eq!(parse_uevent("HID_ID=0003:zz:1"), None);
        assert_eq!(
            parse_uevent("HID_ID=0003:12345678:0001"),
            None,
            "wider than u16"
        );
    }

    #[test]
    fn scan_finds_every_supported_model_and_nothing_else() {
        let root = scratch("scan");
        node(&root, 0, "0003:0000256F:0000C635", "Compact", None);
        node(&root, 1, "0003:0000256F:0000C62E", "Wireless", None);
        node(&root, 2, "0003:0000046D:0000C62B", "Pro", None);
        node(&root, 3, "0003:0000256F:0000C633", "Enterprise", None);
        node(&root, 4, "0003:0000046D:0000C625", "SpacePilot", None);
        node(
            &root,
            5,
            "0003:0000046D:0000C52B",
            "Keyboard receiver",
            None,
        );
        let found = HidrawOpener::with_roots(&root, "/devroot").scan();
        let names: Vec<_> = found.iter().map(|d| d.name.as_str()).collect();
        assert_eq!(names, ["Compact", "Wireless", "Pro", "Enterprise"]);
        assert_eq!(found[0].path, Path::new("/devroot/hidraw0"));
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn only_the_motion_interface_is_chosen() {
        let root = scratch("iface");
        node(
            &root,
            0,
            "0003:0000256F:0000C633",
            "kbd",
            Some(&[0x05, 0x01, 0x09, 0x06]),
        );
        node(
            &root,
            1,
            "0003:0000256F:0000C633",
            "motion",
            Some(&[0x05, 0x01, 0x09, 0x08, 0xA1]),
        );
        let found = HidrawOpener::with_roots(&root, "/d").scan();
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].name, "motion");
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_missing_sysfs_is_an_empty_scan() {
        let none = scratch("none").join("absent");
        assert!(HidrawOpener::with_roots(&none, "/d").scan().is_empty());
    }

    #[test]
    fn open_reads_a_regular_file_fixture_and_reports_a_missing_node() {
        let root = scratch("open");
        let path = root.join("hidraw0");
        fs::write(&path, [1u8, 2, 3]).unwrap();
        let opener = HidrawOpener::with_roots(&root, &root);
        let info = DeviceInfo {
            path: path.clone(),
            vendor: 0x256f,
            product: 0xc635,
            name: String::new(),
        };
        let mut r = opener.open(&info).unwrap();
        let mut buf = [0u8; 8];
        assert_eq!(r.read(&mut buf).unwrap(), 3);
        assert_eq!(
            r.read(&mut buf).unwrap(),
            0,
            "end of file reads as a gone device"
        );
        let gone = DeviceInfo {
            path: root.join("hidraw9"),
            ..info
        };
        assert!(opener.open(&gone).is_err());
        fs::remove_dir_all(&root).unwrap();
    }
}
