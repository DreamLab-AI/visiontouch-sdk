//! A scripted device for tests and client development. It opens nothing.
//!
//! [`ScriptedDevice`] stands in for a SpaceMouse: [`plug`](ScriptedDevice::plug) and
//! [`unplug`](ScriptedDevice::unplug) model hot-plug, and
//! [`push`](ScriptedDevice::push) queues raw HID input reports as `hidraw` would return
//! them. Its [`opener`](ScriptedDevice::opener) is an [`Opener`] to hand to a
//! [`crate::monitor::Monitor`]. Clones share one device.
use crate::device::{DeviceInfo, Opener};
use std::collections::VecDeque;
use std::io::{self, Read};
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};

#[derive(Default)]
struct Inner {
    plugged: bool,
    generation: u64,
    queue: VecDeque<Vec<u8>>,
    opens: u32,
}

/// A scripted SpaceMouse. See the module docs.
#[derive(Clone, Default)]
pub struct ScriptedDevice {
    inner: Arc<Mutex<Inner>>,
}

impl ScriptedDevice {
    /// A device that is not plugged in.
    pub fn new() -> Self {
        Self::default()
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Plug the device in.
    pub fn plug(&self) {
        let mut g = self.lock();
        g.plugged = true;
        g.generation += 1;
        g.queue.clear();
    }

    /// Unplug it. A reader that is open sees end of file.
    pub fn unplug(&self) {
        let mut g = self.lock();
        g.plugged = false;
        g.generation += 1;
        g.queue.clear();
    }

    /// Queue one raw input report (report ID first).
    pub fn push(&self, report: &[u8]) {
        self.lock().queue.push_back(report.to_vec());
    }

    /// Queue a translation report (ID 1).
    pub fn push_translation(&self, x: i16, y: i16, z: i16) {
        self.push(&axes_report(1, [x, y, z]));
    }

    /// Queue a rotation report (ID 2).
    pub fn push_rotation(&self, rx: i16, ry: i16, rz: i16) {
        self.push(&axes_report(2, [rx, ry, rz]));
    }

    /// Queue a button report (ID 3) with a two-byte mask.
    pub fn push_buttons(&self, mask: u16) {
        let m = mask.to_le_bytes();
        self.push(&[3, m[0], m[1]]);
    }

    /// How many times a reader has been opened.
    pub fn opens(&self) -> u32 {
        self.lock().opens
    }

    /// An [`Opener`] over this device.
    pub fn opener(&self) -> Box<dyn Opener> {
        Box::new(self.clone())
    }
}

/// A translation or rotation report for the given report ID.
pub fn axes_report(id: u8, v: [i16; 3]) -> Vec<u8> {
    let mut out = vec![id];
    out.extend(v.iter().flat_map(|x| x.to_le_bytes()));
    out
}

struct Reader {
    device: ScriptedDevice,
    generation: u64,
}

impl Read for Reader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut g = self.device.lock();
        if g.generation != self.generation || !g.plugged {
            return Ok(0);
        }
        match g.queue.pop_front() {
            Some(r) => {
                let n = r.len().min(buf.len());
                buf[..n].copy_from_slice(&r[..n]);
                Ok(n)
            }
            None => Err(io::ErrorKind::WouldBlock.into()),
        }
    }
}

impl Opener for ScriptedDevice {
    fn scan(&self) -> Vec<DeviceInfo> {
        if !self.lock().plugged {
            return Vec::new();
        }
        vec![DeviceInfo {
            path: PathBuf::from("/scripted/hidraw0"),
            vendor: 0x256f,
            product: 0xc635,
            name: "Scripted SpaceMouse".into(),
        }]
    }

    fn open(&self, _device: &DeviceInfo) -> io::Result<Box<dyn Read + Send>> {
        let mut g = self.lock();
        if !g.plugged {
            return Err(io::ErrorKind::NotFound.into());
        }
        g.opens += 1;
        Ok(Box::new(Reader {
            device: self.clone(),
            generation: g.generation,
        }))
    }
}
