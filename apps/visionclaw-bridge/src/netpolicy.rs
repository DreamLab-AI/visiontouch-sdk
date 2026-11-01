//! LAN-only network policy for the listening socket and its peers.
//!
//! By default the bridge binds only to a loopback or private-network address and
//! accepts connections only from loopback, private and link-local peers. Binding to a
//! wildcard (`0.0.0.0`, `::`) or a public address, or accepting public peers, needs
//! `allow_public = true` in the configuration. This is defence in depth: the
//! authentication in [`crate::auth`] is the access control.
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};

fn lan_v4(ip: Ipv4Addr) -> bool {
    ip.is_loopback() || ip.is_private() || ip.is_link_local()
}

fn lan_v6(ip: Ipv6Addr) -> bool {
    if let Some(v4) = ip.to_ipv4_mapped() {
        return lan_v4(v4);
    }
    let first = ip.segments()[0];
    ip.is_loopback()
        || (first & 0xfe00) == 0xfc00 // unique local, fc00::/7
        || (first & 0xffc0) == 0xfe80 // link-local, fe80::/10
}

/// Whether `ip` is loopback, private (RFC 1918, IPv6 unique local) or link-local.
pub fn is_lan(ip: IpAddr) -> bool {
    match ip {
        IpAddr::V4(v4) => lan_v4(v4),
        IpAddr::V6(v6) => lan_v6(v6),
    }
}

/// Check the configured listen address. A wildcard or public address is refused unless
/// `allow_public`.
pub fn check_bind(addr: SocketAddr, allow_public: bool) -> Result<(), String> {
    if allow_public {
        return Ok(());
    }
    let ip = addr.ip();
    if ip.is_unspecified() {
        return Err(format!(
            "refusing to listen on the wildcard address {ip}: set `bind` to this machine's \
             LAN or loopback address, or set `allow_public = true`"
        ));
    }
    if !is_lan(ip) {
        return Err(format!(
            "refusing to listen on the non-LAN address {ip}; set `allow_public = true` to override"
        ));
    }
    Ok(())
}

/// Whether to accept a connection from `peer`.
pub fn accept_peer(peer: SocketAddr, allow_public: bool) -> bool {
    allow_public || is_lan(peer.ip())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ip(s: &str) -> IpAddr {
        s.parse().unwrap()
    }

    #[test]
    fn lan_addresses() {
        for s in [
            "127.0.0.1",
            "10.1.2.3",
            "172.16.0.1",
            "172.31.255.255",
            "192.168.1.40",
            "169.254.1.1",
            "::1",
            "fd00::1",
            "fc12::1",
            "fe80::1",
            "::ffff:192.168.1.1",
        ] {
            assert!(is_lan(ip(s)), "{s}");
        }
        for s in [
            "8.8.8.8",
            "172.32.0.1",
            "100.64.0.1",
            "0.0.0.0",
            "::",
            "2001:db8::1",
            "::ffff:8.8.8.8",
            "255.255.255.255",
        ] {
            assert!(!is_lan(ip(s)), "{s}");
        }
    }

    #[test]
    fn bind_policy() {
        let a = |s: &str| s.parse::<SocketAddr>().unwrap();
        assert!(check_bind(a("127.0.0.1:7341"), false).is_ok());
        assert!(check_bind(a("192.168.1.20:7341"), false).is_ok());
        assert!(check_bind(a("0.0.0.0:7341"), false).is_err());
        assert!(check_bind(a("[::]:7341"), false).is_err());
        assert!(check_bind(a("203.0.113.9:7341"), false).is_err());
        assert!(check_bind(a("0.0.0.0:7341"), true).is_ok());
    }

    #[test]
    fn peer_policy() {
        let a = |s: &str| s.parse::<SocketAddr>().unwrap();
        assert!(accept_peer(a("192.168.1.5:5000"), false));
        assert!(!accept_peer(a("8.8.8.8:5000"), false));
        assert!(accept_peer(a("8.8.8.8:5000"), true));
    }
}
