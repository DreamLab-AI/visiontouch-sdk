//! A fake spacenavd for tests and client development.
//!
//! [`FakeDaemon`] listens on a Unix socket in the temporary directory and speaks the
//! daemon side of the protocol described in the [crate documentation](crate): it
//! answers the protocol-1 handshake (or stays silent, as an old daemon does), answers
//! the device query, and sends motion, button and device events. It opens no hardware
//! and no `/dev` node. One client is served at a time; clones share one daemon.
use crate::wire::{FRAME_LEN, HANDSHAKE, REQ_DEV_NAXES, REQ_TAG, frame};
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::Duration;

static NEXT: AtomicU32 = AtomicU32::new(0);

#[derive(Default)]
struct Inner {
    plugged: bool,
    protocol0: bool,
    axes: [i32; 6],
    buttons: u32,
    out: VecDeque<[u8; FRAME_LEN]>,
    kick: bool,
    connections: u32,
}

/// A fake spacenavd. See the module docs. Dropping it stops it and removes its socket.
pub struct FakeDaemon {
    path: PathBuf,
    inner: Arc<Mutex<Inner>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

/// A short, unique socket path (`sun_path` holds only about 100 bytes).
pub fn unique_socket_path() -> PathBuf {
    let name = format!(
        "spnav-{}-{}.sock",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    );
    let tmp = std::env::temp_dir();
    if tmp.as_os_str().len() + name.len() < 90 {
        tmp.join(name)
    } else {
        PathBuf::from("/tmp").join(name)
    }
}

impl FakeDaemon {
    /// Start a daemon on a fresh socket path, with a device attached.
    pub fn start() -> Self {
        let d = Self::at(unique_socket_path()).expect("fake spnav socket");
        d.plug();
        d
    }

    /// Start a daemon on `path` with no device attached.
    pub fn at(path: impl Into<PathBuf>) -> io::Result<Self> {
        let path = path.into();
        let _ = std::fs::remove_file(&path);
        let listener = UnixListener::bind(&path)?;
        listener.set_nonblocking(true)?;
        let inner = Arc::new(Mutex::new(Inner::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let (inner, stop) = (inner.clone(), stop.clone());
            thread::Builder::new()
                .name("fake-spnav".into())
                .spawn(move || serve(&listener, &inner, &stop))?
        };
        Ok(Self {
            path,
            inner,
            stop,
            thread: Some(thread),
        })
    }

    /// Behave as an old (protocol 0) daemon: no handshake reply, no device events.
    pub fn protocol0(self) -> Self {
        self.lock().protocol0 = true;
        self
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// The socket path.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// How many clients have connected so far.
    pub fn connections(&self) -> u32 {
        self.lock().connections
    }

    /// Attach the device; a connected client is sent a device-add event.
    pub fn plug(&self) {
        let mut g = self.lock();
        g.plugged = true;
        g.axes = [0; 6];
        g.buttons = 0;
        if !g.protocol0 {
            g.out.push_back(frame([3, 0, 0, 0, 0, 0, 0, 0]));
        }
    }

    /// Remove the device; a connected client is sent a device-remove event.
    pub fn unplug(&self) {
        let mut g = self.lock();
        g.plugged = false;
        g.axes = [0; 6];
        g.buttons = 0;
        if !g.protocol0 {
            g.out.push_back(frame([3, 1, 0, 0, 0, 0, 0, 0]));
        }
    }

    /// Drop the connected client's connection without a word (the daemon dies).
    pub fn disconnect_client(&self) {
        self.lock().kick = true;
    }

    fn send_motion(g: &mut Inner) {
        let a = g.axes;
        g.out
            .push_back(frame([0, a[0], a[1], a[2], a[3], a[4], a[5], 16]));
    }

    /// Set the translation axes and send a motion event with the current rotation.
    pub fn push_translation(&self, x: i32, y: i32, z: i32) {
        let mut g = self.lock();
        g.axes[..3].copy_from_slice(&[x, y, z]);
        Self::send_motion(&mut g);
    }

    /// Set the rotation axes and send a motion event with the current translation.
    pub fn push_rotation(&self, rx: i32, ry: i32, rz: i32) {
        let mut g = self.lock();
        g.axes[3..].copy_from_slice(&[rx, ry, rz]);
        Self::send_motion(&mut g);
    }

    /// Set the button state to `mask`: one press or release event per changed bit.
    pub fn push_buttons(&self, mask: u32) {
        let mut g = self.lock();
        let changed = g.buttons ^ mask;
        for n in 0..32 {
            if changed & (1 << n) != 0 {
                let kind = if mask & (1 << n) != 0 { 1 } else { 2 };
                g.out.push_back(frame([kind, n, 0, 0, 0, 0, 0, 0]));
            }
        }
        g.buttons = mask;
    }

    /// Send raw words as one frame (for protocol edge cases).
    pub fn push_raw(&self, words: [i32; 8]) {
        self.lock().out.push_back(frame(words));
    }
}

impl FakeDaemon {
    /// Stop the daemon now: the connected client is dropped and the socket file is
    /// removed, as when spacenavd is stopped. Idempotent; also done on drop.
    pub fn halt(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        let _ = std::fs::remove_file(&self.path);
    }
}

impl Drop for FakeDaemon {
    fn drop(&mut self) {
        self.halt();
    }
}

fn serve(listener: &UnixListener, inner: &Mutex<Inner>, stop: &AtomicBool) {
    let lock = || inner.lock().unwrap_or_else(|e| e.into_inner());
    while !stop.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, _)) => {
                {
                    let mut g = lock();
                    g.out.clear();
                    g.kick = false;
                    g.connections += 1;
                }
                client(stream, &lock, stop);
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(2));
            }
            Err(_) => thread::sleep(Duration::from_millis(2)),
        }
    }
}

fn client<'a>(mut s: UnixStream, lock: &impl Fn() -> MutexGuard<'a, Inner>, stop: &AtomicBool) {
    if s.set_nonblocking(true).is_err() {
        return;
    }
    let mut got: Vec<u8> = Vec::new();
    let mut greeted = false;
    let mut buf = [0u8; 256];
    while !stop.load(Ordering::Relaxed) {
        match s.read(&mut buf) {
            Ok(0) => return,
            Ok(n) => got.extend_from_slice(&buf[..n]),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {}
            Err(_) => return,
        }
        if !greeted && got.len() >= 4 {
            let hello = i32::from_ne_bytes([got[0], got[1], got[2], got[3]]);
            got.drain(..4);
            greeted = true;
            let protocol0 = lock().protocol0;
            if hello == HANDSHAKE && !protocol0 {
                // A real daemon replies with the version it accepted.
                if s.write_all(&HANDSHAKE.to_ne_bytes()).is_err() {
                    return;
                }
            }
        }
        while greeted && got.len() >= FRAME_LEN {
            let kind = i32::from_ne_bytes([got[0], got[1], got[2], got[3]]);
            got.drain(..FRAME_LEN);
            if kind == REQ_TAG | REQ_DEV_NAXES {
                let status = if lock().plugged { 0 } else { -1 };
                let reply = frame([kind, 6, 0, 0, 0, 0, 0, status]);
                if s.write_all(&reply).is_err() {
                    return;
                }
            }
        }
        let batch: Vec<_> = {
            let mut g = lock();
            if g.kick {
                return;
            }
            if !greeted {
                Vec::new()
            } else {
                g.out.drain(..).collect()
            }
        };
        for f in batch {
            if s.write_all(&f).is_err() {
                return;
            }
        }
        thread::sleep(Duration::from_millis(1));
    }
}
