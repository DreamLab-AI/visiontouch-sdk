//! Bridge configuration (TOML), with every limit validated at load.
//!
//! ```toml
//! bind = "192.168.1.20:7341"        # this machine's LAN address; default 127.0.0.1:7341
//! url = "ws://192.168.1.20:7341"    # required: the URL clients must name when signing
//! authorised_keys = ["npub1..."]    # required: hex or npub keys allowed to control the arm
//! # allowed_origins = ["http://desktop.lan:3001"]  # gate for browser clients only
//! # allow_public = false            # wildcard or public bind and public peers
//! # device = "/dev/parport0"
//!
//! [coupling]
//! stiffness_n_mm = 0.4              # at most 1.0 (half the unit's stable 2 N/mm)
//! damping_n_s_m = 1.0
//! damping_cap_n = 0.5
//! force_cap_n = 1.41                # at most the unit's peak; default its continuous level
//! workspace_radius_m = 0.12
//! target_max_speed_m_s = 0.25
//! target_tau_ms = 30
//! ramp_ms = 250
//! force_rate_n_s = 20               # largest force change per second while engaged
//!
//! [link]
//! target_timeout_ms = 200           # 50..=250
//! idle_timeout_ms = 1000
//! auth_timeout_ms = 5000
//! status_hz = 60
//!
//! [spacemouse]                      # optional SpaceMouse navigation through spacenavd; see `nav`
//! enabled = false
//! socket = "/var/run/spnav.sock"    # spacenavd's socket
//! deadzone = 0.05                   # fraction of full scale, 0..=0.9
//! sensitivity = [1, 1, 1, 1, 1, 1]  # x y z rx ry rz; magnitude 0.01..=10, negative inverts
//! deflection_max = 350              # spacenavd count that maps to 1.0, 1..=1000000
//! rate_hz = 60                      # nav frames per second, 10..=120
//! poll_ms = 1000                    # reconnect interval while spacenavd is absent
//! ```
use crate::auth::AuthPolicy;
use crate::coupling::{Coupling, TargetFilter};
use crate::filter::Settings;
use crate::netpolicy;
use crate::protocol::Limits;
use serde::Deserialize;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::time::Duration;
use visiontouch_device::unit::{CONTINUOUS_N, PEAK_N, STABLE_STIFFNESS_N_MM};

/// Highest coupling stiffness the bridge accepts, N/mm: half the unit's stable figure.
///
/// The servo loop itself is local, so its stability margin is the SDK's 2 N/mm. The
/// margin is halved because the target is the far end of a slower loop through the
/// network and the client's simulation; a stiffer coupling feeds more of that delayed
/// loop's energy back into the hand.
pub const MAX_STIFFNESS_N_MM: f64 = STABLE_STIFFNESS_N_MM / 2.;
/// Default coupling stiffness, N/mm.
pub const DEFAULT_STIFFNESS_N_MM: f64 = 0.4;
/// Shortest and longest accepted target timeout, ms.
pub const TARGET_TIMEOUT_RANGE_MS: (u64, u64) = (50, 250);
/// Default listen address: loopback.
pub const DEFAULT_BIND: &str = "127.0.0.1:7341";

/// The coupling section.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields, default)]
pub struct CouplingConfig {
    /// Spring stiffness, N/mm, in `(0, MAX_STIFFNESS_N_MM]`.
    pub stiffness_n_mm: f64,
    /// Damping on relative velocity, N·s/m, in `[0, 10]`.
    pub damping_n_s_m: f64,
    /// Cap on the damping term, N, at most `force_cap_n`.
    pub damping_cap_n: f64,
    /// Cap on total force, N, in `(0, PEAK_N]`.
    pub force_cap_n: f64,
    /// Target clamp radius about the registered origin, m, in `(0, 0.2]`.
    pub workspace_radius_m: f64,
    /// Anchor slew limit, m/s, in `(0, 1]`.
    pub target_max_speed_m_s: f64,
    /// Anchor low-pass time constant, ms, at most 200.
    pub target_tau_ms: u64,
    /// Engage ramp, ms, in `[50, 2000]`.
    pub ramp_ms: u64,
    /// Largest change of the force vector per second while engaged, N/s, in
    /// `[5, 200]`. Bounds how fast target motion (physics or scene navigation) can
    /// change the force; cutting force is never rate-limited.
    pub force_rate_n_s: f64,
}

impl Default for CouplingConfig {
    fn default() -> Self {
        Self {
            stiffness_n_mm: DEFAULT_STIFFNESS_N_MM,
            damping_n_s_m: 1.0,
            damping_cap_n: 0.5,
            force_cap_n: CONTINUOUS_N,
            workspace_radius_m: 0.12,
            target_max_speed_m_s: 0.25,
            target_tau_ms: 30,
            ramp_ms: 250,
            force_rate_n_s: 20.,
        }
    }
}

/// The link section.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields, default)]
pub struct LinkConfig {
    /// Force stops when no target has arrived for this long, ms.
    pub target_timeout_ms: u64,
    /// The connection is closed when no message has arrived for this long, ms.
    pub idle_timeout_ms: u64,
    /// A connection must authenticate within this long, ms.
    pub auth_timeout_ms: u64,
    /// Status messages per second, in `[1, 200]`.
    pub status_hz: u32,
}

impl Default for LinkConfig {
    fn default() -> Self {
        Self {
            target_timeout_ms: 200,
            idle_timeout_ms: 1000,
            auth_timeout_ms: 5000,
            status_hz: 60,
        }
    }
}

/// The `[spacemouse]` section: optional navigation from a SpaceMouse plugged into this
/// machine and served by spacenavd. Disabled unless `enabled = true`; an enabled
/// bridge with no spacenavd or no device simply sends no navigation, and says why.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields, default)]
pub struct SpacemouseConfig {
    /// Read a SpaceMouse through spacenavd and offer navigation to subscribed clients.
    pub enabled: bool,
    /// The spacenavd socket.
    pub socket: PathBuf,
    /// Deadzone as a fraction of full scale, in `[0, 0.9]`.
    pub deadzone: f64,
    /// Per-axis multiplier `x, y, z, rx, ry, rz`; magnitude in `[0.01, 10]`, a negative
    /// sign inverts the axis.
    pub sensitivity: [f64; 6],
    /// The spacenavd count that maps to full deflection (1.0), in `[1, 1000000]`.
    /// spacenavd does not normalise; its range follows its own sensitivity setting.
    pub deflection_max: f64,
    /// Navigation frames per second while the input is non-zero, in `[10, 120]`.
    pub rate_hz: u32,
    /// Milliseconds between connection attempts while spacenavd is absent or lost, in
    /// `[200, 10000]`.
    pub poll_ms: u64,
}

impl Default for SpacemouseConfig {
    fn default() -> Self {
        let s = Settings::default();
        Self {
            enabled: false,
            socket: PathBuf::from(spnav_client::DEFAULT_SOCKET),
            deadzone: s.deadzone,
            sensitivity: s.sensitivity,
            deflection_max: s.deflection_max,
            rate_hz: 60,
            poll_ms: 1000,
        }
    }
}

impl SpacemouseConfig {
    /// The filter settings.
    pub fn settings(&self) -> Settings {
        Settings {
            deadzone: self.deadzone,
            sensitivity: self.sensitivity,
            deflection_max: self.deflection_max,
        }
    }
}

/// The whole configuration file.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// Listen address.
    #[serde(default = "default_bind")]
    pub bind: SocketAddr,
    /// Allow a wildcard or public bind and public peers.
    #[serde(default)]
    pub allow_public: bool,
    /// The URL clients must name in their signed `relay` tag. Required: without it a
    /// hostile service could relay this bridge's challenge to a user and replay the
    /// signature here.
    pub url: String,
    /// Keys (hex or npub) allowed to control the arm. Must not be empty.
    pub authorised_keys: Vec<String>,
    /// Browser `Origin` values allowed to connect. This is a gate for browser clients
    /// only: a request carrying an `Origin` header not in this list is refused, so an
    /// empty list admits only clients that send none. A native client is not subject
    /// to it and is authenticated by the signed challenge alone.
    #[serde(default)]
    pub allowed_origins: Vec<String>,
    /// Parallel port device.
    #[serde(default = "default_device")]
    pub device: PathBuf,
    /// Coupling gains and caps.
    #[serde(default)]
    pub coupling: CouplingConfig,
    /// Link timeouts and rates.
    #[serde(default)]
    pub link: LinkConfig,
    /// Optional SpaceMouse navigation.
    #[serde(default)]
    pub spacemouse: SpacemouseConfig,
}

fn default_bind() -> SocketAddr {
    DEFAULT_BIND.parse().expect("valid default")
}
fn default_device() -> PathBuf {
    PathBuf::from("/dev/parport0")
}

fn range(name: &str, v: f64, lo: f64, hi: f64, lo_open: bool) -> Result<(), String> {
    let ok = v.is_finite() && (if lo_open { v > lo } else { v >= lo }) && v <= hi;
    if ok {
        Ok(())
    } else {
        let open = if lo_open { "(" } else { "[" };
        Err(format!("{name} = {v} is outside {open}{lo}, {hi}]"))
    }
}

impl Config {
    /// Parse and validate a TOML document.
    pub fn from_toml(text: &str) -> Result<Self, String> {
        let c: Config = toml::from_str(text).map_err(|e| e.to_string())?;
        c.validate()?;
        Ok(c)
    }

    /// Check every limit, the bind policy and the key list.
    pub fn validate(&self) -> Result<(), String> {
        let k = &self.coupling;
        range(
            "stiffness_n_mm",
            k.stiffness_n_mm,
            0.,
            MAX_STIFFNESS_N_MM,
            true,
        )?;
        range("damping_n_s_m", k.damping_n_s_m, 0., 10., false)?;
        range("force_cap_n", k.force_cap_n, 0., PEAK_N, true)?;
        range("damping_cap_n", k.damping_cap_n, 0., k.force_cap_n, false)?;
        range("workspace_radius_m", k.workspace_radius_m, 0., 0.2, true)?;
        range("target_max_speed_m_s", k.target_max_speed_m_s, 0., 1., true)?;
        range("target_tau_ms", k.target_tau_ms as f64, 0., 200., false)?;
        range("ramp_ms", k.ramp_ms as f64, 50., 2000., false)?;
        range("force_rate_n_s", k.force_rate_n_s, 5., 200., false)?;
        let l = &self.link;
        let (lo, hi) = TARGET_TIMEOUT_RANGE_MS;
        range(
            "target_timeout_ms",
            l.target_timeout_ms as f64,
            lo as f64,
            hi as f64,
            false,
        )?;
        range(
            "idle_timeout_ms",
            l.idle_timeout_ms as f64,
            l.target_timeout_ms as f64,
            10_000.,
            true,
        )?;
        range(
            "auth_timeout_ms",
            l.auth_timeout_ms as f64,
            1000.,
            30_000.,
            false,
        )?;
        range("status_hz", l.status_hz as f64, 1., 200., false)?;
        let m = &self.spacemouse;
        m.settings()
            .validate()
            .map_err(|e| format!("spacemouse: {e}"))?;
        range("rate_hz", f64::from(m.rate_hz), 10., 120., false)?;
        range("poll_ms", m.poll_ms as f64, 200., 10_000., false)?;
        netpolicy::check_bind(self.bind, self.allow_public)?;
        self.auth_policy()?;
        Ok(())
    }

    /// The authentication policy: parsed keys and the URL.
    pub fn auth_policy(&self) -> Result<AuthPolicy, String> {
        if self.authorised_keys.is_empty() {
            return Err("authorised_keys is empty: nobody could control the arm".into());
        }
        let allowed = self
            .authorised_keys
            .iter()
            .map(|k| {
                nostr::key::PublicKey::parse(k).map_err(|e| format!("authorised key `{k}`: {e}"))
            })
            .collect::<Result<_, _>>()?;
        if !(self.url.starts_with("ws://") || self.url.starts_with("wss://")) {
            return Err(format!(
                "url `{}` must start with ws:// or wss://",
                self.url
            ));
        }
        Ok(AuthPolicy {
            allowed,
            url: Some(self.url.clone()),
        })
    }

    /// The coupling gains in SI units.
    pub fn coupling(&self) -> Coupling {
        let k = &self.coupling;
        Coupling {
            stiffness_n_m: k.stiffness_n_mm * 1000.,
            damping_n_s_m: k.damping_n_s_m,
            damping_cap_n: k.damping_cap_n,
            force_cap_n: k.force_cap_n,
        }
    }

    /// A fresh target filter.
    pub fn target_filter(&self) -> TargetFilter {
        let k = &self.coupling;
        TargetFilter::new(
            k.workspace_radius_m,
            k.target_max_speed_m_s,
            k.target_tau_ms as f64 / 1000.,
        )
    }

    /// Target timeout.
    pub fn target_timeout(&self) -> Duration {
        Duration::from_millis(self.link.target_timeout_ms)
    }
    /// Engage ramp time.
    pub fn ramp(&self) -> Duration {
        Duration::from_millis(self.coupling.ramp_ms)
    }

    /// The limits announced in `hello`.
    pub fn limits(&self) -> Limits {
        Limits {
            stiffness_n_mm: self.coupling.stiffness_n_mm,
            damping_n_s_m: self.coupling.damping_n_s_m,
            force_cap_n: self.coupling.force_cap_n,
            target_timeout_ms: self.link.target_timeout_ms,
            idle_timeout_ms: self.link.idle_timeout_ms,
            workspace_radius_m: self.coupling.workspace_radius_m,
            status_hz: self.link.status_hz,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: &str = "79be667ef9dcbbac55a06295ce870b07029bfcdb2dce28d959f2815b16f81798";

    fn minimal() -> String {
        format!("url = \"ws://bridge:7341\"\nauthorised_keys = [\"{KEY}\"]\n")
    }

    #[test]
    fn the_minimal_file_gets_safe_defaults() {
        let c = Config::from_toml(&minimal()).unwrap();
        assert_eq!(c.bind, DEFAULT_BIND.parse().unwrap());
        assert!(!c.allow_public);
        assert_eq!(c.coupling.stiffness_n_mm, DEFAULT_STIFFNESS_N_MM);
        assert!(c.coupling.stiffness_n_mm <= MAX_STIFFNESS_N_MM);
        assert_eq!(c.coupling.force_cap_n, CONTINUOUS_N);
        assert_eq!(c.link.target_timeout_ms, 200);
        assert_eq!(c.coupling().stiffness_n_m, 400.);
    }

    #[test]
    fn stiffness_above_half_the_stable_figure_is_refused() {
        assert_eq!(MAX_STIFFNESS_N_MM, 1.0);
        let ok = format!("{}[coupling]\nstiffness_n_mm = 1.0\n", minimal());
        assert!(Config::from_toml(&ok).is_ok());
        for bad in ["1.01", "2.0", "0", "-0.5", "nan"] {
            let t = format!("{}[coupling]\nstiffness_n_mm = {bad}\n", minimal());
            assert!(Config::from_toml(&t).is_err(), "{bad}");
        }
    }

    #[test]
    fn force_cap_is_bounded_by_the_peak() {
        let t = format!("{}[coupling]\nforce_cap_n = {}\n", minimal(), PEAK_N + 0.01);
        assert!(Config::from_toml(&t).is_err());
        let t = format!("{}[coupling]\nforce_cap_n = 5.0\n", minimal());
        assert!(Config::from_toml(&t).is_ok());
        let t = format!(
            "{}[coupling]\nforce_cap_n = 1.0\ndamping_cap_n = 1.5\n",
            minimal()
        );
        assert!(
            Config::from_toml(&t).is_err(),
            "damping cap above total cap"
        );
    }

    #[test]
    fn the_force_rate_is_bounded() {
        assert_eq!(
            Config::from_toml(&minimal())
                .unwrap()
                .coupling
                .force_rate_n_s,
            20.
        );
        for (v, ok) in [("4.9", false), ("5", true), ("200", true), ("201", false)] {
            let t = format!("{}[coupling]\nforce_rate_n_s = {v}\n", minimal());
            assert_eq!(Config::from_toml(&t).is_ok(), ok, "{v}");
        }
    }

    #[test]
    fn timeouts_are_bounded() {
        for (ms, ok) in [(49, false), (50, true), (250, true), (251, false)] {
            let t = format!("{}[link]\ntarget_timeout_ms = {ms}\n", minimal());
            assert_eq!(Config::from_toml(&t).is_ok(), ok, "{ms}");
        }
        let t = format!(
            "{}[link]\ntarget_timeout_ms = 200\nidle_timeout_ms = 200\n",
            minimal()
        );
        assert!(
            Config::from_toml(&t).is_err(),
            "idle must exceed target timeout"
        );
    }

    #[test]
    fn keys_bind_and_unknown_fields_are_checked() {
        assert!(Config::from_toml("url = \"ws://b\"\nauthorised_keys = []\n").is_err());
        assert!(Config::from_toml("url = \"ws://b\"\nauthorised_keys = [\"nope\"]\n").is_err());
        let no_url = format!("authorised_keys = [\"{KEY}\"]\n");
        assert!(Config::from_toml(&no_url).is_err(), "url is required");
        let bad_url = format!("url = \"http://b\"\nauthorised_keys = [\"{KEY}\"]\n");
        assert!(Config::from_toml(&bad_url).is_err());
        assert!(Config::from_toml("").is_err(), "keys are required");
        let t = format!("bind = \"0.0.0.0:7341\"\n{}", minimal());
        assert!(Config::from_toml(&t).is_err());
        let t = format!(
            "bind = \"0.0.0.0:7341\"\nallow_public = true\n{}",
            minimal()
        );
        assert!(Config::from_toml(&t).is_ok());
        let t = format!("{}stifness = 3\n", minimal());
        assert!(
            Config::from_toml(&t).is_err(),
            "typo is not silently ignored"
        );
    }

    #[test]
    fn spacemouse_is_off_by_default_and_its_limits_are_checked() {
        let c = Config::from_toml(&minimal()).unwrap();
        assert!(!c.spacemouse.enabled);
        assert_eq!(c.spacemouse.rate_hz, 60);
        for bad in [
            "deadzone = 0.95",
            "deadzone = -0.1",
            "sensitivity = [1, 1, 1, 1, 1, 0]",
            "sensitivity = [1, 1, 1, 1, 1, 11]",
            "deflection_max = 0",
            "deflection_max = 2000000",
            "rate_hz = 9",
            "rate_hz = 121",
            "poll_ms = 100",
            "unknown = 1",
        ] {
            let text = format!("{}\n[spacemouse]\n{bad}\n", minimal());
            assert!(Config::from_toml(&text).is_err(), "accepted: {bad}");
        }
        let text = format!(
            "{}\n[spacemouse]\nenabled = true\nsensitivity = [1, -1, 1, 0.5, 0.5, 2]\n",
            minimal()
        );
        let c = Config::from_toml(&text).unwrap();
        assert!(c.spacemouse.enabled);
        assert_eq!(c.spacemouse.settings().sensitivity[1], -1.);
    }
}
