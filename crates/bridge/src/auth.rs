//! Authentication: a signed challenge, NIP-42 style, against an allowlist of keys.
//!
//! On connect the bridge sends a fresh 32-byte random challenge in `hello`. The client
//! answers with a Nostr event of kind 22242 (the NIP-42 client-authentication kind)
//! carrying a `challenge` tag with that nonce and a `relay` tag with the bridge URL,
//! signed by a key in the bridge's allowlist. The bridge checks the event id and
//! BIP-340 signature (with the maintained `nostr` crate; no cryptography is
//! implemented here), the kind, the challenge, the signing time and the key.
//!
//! Why a challenge and not a NIP-98 header: browsers cannot set an `Authorization`
//! header on a WebSocket upgrade, so a NIP-98 event could only travel in the URL,
//! where it is logged and replayable for its whole time window. A per-connection
//! nonce makes every signature single-use. The identity is the same `did:nostr` key
//! a NIP-98 client already holds, and browser signers (NIP-07) sign kind 22242.
use nostr::event::{Event, Kind};
use nostr::key::PublicKey;
use std::collections::HashSet;

/// The NIP-42 client-authentication event kind.
pub const AUTH_KIND: u16 = 22242;
/// Largest accepted difference between the event's `created_at` and the bridge clock.
pub const MAX_CLOCK_SKEW_S: u64 = 60;

/// A fresh random challenge: 32 bytes from the operating system, hex.
pub fn new_challenge() -> std::io::Result<String> {
    let mut bytes = [0u8; 32];
    getrandom::fill(&mut bytes).map_err(|e| std::io::Error::other(e.to_string()))?;
    Ok(bytes.iter().map(|b| format!("{b:02x}")).collect())
}

/// Who may connect, and where.
#[derive(Clone, Debug, Default)]
pub struct AuthPolicy {
    /// Keys allowed to control the arm.
    pub allowed: HashSet<PublicKey>,
    /// If set, the event's `relay` tag must name this URL (trailing `/` ignored).
    pub url: Option<String>,
}

/// Why authentication failed. The detail is logged at the bridge; the client is told
/// only that it was refused.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthError {
    /// The event is not kind 22242.
    WrongKind,
    /// The id or signature does not verify.
    BadSignature,
    /// The `challenge` tag is missing or does not match.
    WrongChallenge,
    /// `created_at` is too far from the bridge clock.
    Stale,
    /// The key is not in the allowlist.
    NotAllowed,
    /// The `relay` tag is missing or names another URL.
    WrongUrl,
}

impl std::fmt::Display for AuthError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            AuthError::WrongKind => "event is not kind 22242",
            AuthError::BadSignature => "event id or signature does not verify",
            AuthError::WrongChallenge => "challenge tag missing or wrong",
            AuthError::Stale => "created_at outside the allowed clock skew",
            AuthError::NotAllowed => "key not in the allowlist",
            AuthError::WrongUrl => "relay tag missing or names another URL",
        })
    }
}
impl std::error::Error for AuthError {}

fn same_url(a: &str, b: &str) -> bool {
    a.trim_end_matches('/') == b.trim_end_matches('/')
}

/// Verify `event` against the `challenge` this connection was sent, the policy and the
/// current Unix time in seconds. Returns the authenticated key.
pub fn verify(
    event: &Event,
    challenge: &str,
    policy: &AuthPolicy,
    now_unix_s: u64,
) -> Result<PublicKey, AuthError> {
    if event.kind != Kind::Custom(AUTH_KIND) && event.kind != Kind::Authentication {
        return Err(AuthError::WrongKind);
    }
    event.verify().map_err(|_| AuthError::BadSignature)?;
    if event.tags.challenge().as_deref() != Some(challenge) {
        return Err(AuthError::WrongChallenge);
    }
    if event.created_at.as_secs().abs_diff(now_unix_s) > MAX_CLOCK_SKEW_S {
        return Err(AuthError::Stale);
    }
    if let Some(url) = &policy.url {
        let named = event
            .tags
            .iter()
            .any(|t| t.kind() == "relay" && t.content().is_some_and(|u| same_url(u, url)));
        if !named {
            return Err(AuthError::WrongUrl);
        }
    }
    if !policy.allowed.contains(&event.pubkey) {
        return Err(AuthError::NotAllowed);
    }
    Ok(event.pubkey)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nostr::prelude::*;

    const URL: &str = "ws://10.0.0.5:7341";

    fn now() -> u64 {
        Timestamp::now().as_secs()
    }

    fn signed(keys: &Keys, kind: u16, challenge: &str, url: &str, at: u64) -> Event {
        EventBuilder::new(Kind::from(kind), "")
            .tags([
                Tag::custom("challenge", [challenge]),
                Tag::custom("relay", [url]),
            ])
            .custom_created_at(Timestamp::from(at))
            .finalize(keys)
            .unwrap()
    }

    fn policy(keys: &Keys) -> AuthPolicy {
        AuthPolicy {
            allowed: [keys.public_key()].into(),
            url: Some(URL.into()),
        }
    }

    #[test]
    fn challenges_are_fresh_32_byte_hex() {
        let a = new_challenge().unwrap();
        let b = new_challenge().unwrap();
        assert_eq!(a.len(), 64);
        assert!(a.chars().all(|c| c.is_ascii_hexdigit()));
        assert_ne!(a, b);
    }

    #[test]
    fn a_correct_answer_is_accepted() {
        let k = Keys::generate();
        let e = signed(&k, AUTH_KIND, "abc", &format!("{URL}/"), now());
        assert_eq!(verify(&e, "abc", &policy(&k), now()), Ok(k.public_key()));
        // Without a configured URL the relay tag is not checked.
        let p = AuthPolicy {
            url: None,
            ..policy(&k)
        };
        let e = signed(&k, AUTH_KIND, "abc", "ws://elsewhere", now());
        assert!(verify(&e, "abc", &p, now()).is_ok());
    }

    #[test]
    fn each_wrong_answer_is_refused() {
        let k = Keys::generate();
        let p = policy(&k);
        let t = now();
        assert_eq!(
            verify(&signed(&k, 1, "abc", URL, t), "abc", &p, t),
            Err(AuthError::WrongKind)
        );
        assert_eq!(
            verify(&signed(&k, AUTH_KIND, "abd", URL, t), "abc", &p, t),
            Err(AuthError::WrongChallenge)
        );
        assert_eq!(
            verify(&signed(&k, AUTH_KIND, "abc", URL, t - 61), "abc", &p, t),
            Err(AuthError::Stale)
        );
        assert_eq!(
            verify(&signed(&k, AUTH_KIND, "abc", URL, t + 61), "abc", &p, t),
            Err(AuthError::Stale)
        );
        assert_eq!(
            verify(
                &signed(&k, AUTH_KIND, "abc", "ws://other:7341", t),
                "abc",
                &p,
                t
            ),
            Err(AuthError::WrongUrl)
        );
        let stranger = Keys::generate();
        assert_eq!(
            verify(&signed(&stranger, AUTH_KIND, "abc", URL, t), "abc", &p, t),
            Err(AuthError::NotAllowed)
        );
    }

    #[test]
    fn a_tampered_event_fails_signature_checks() {
        let k = Keys::generate();
        let t = now();
        let e = signed(&k, AUTH_KIND, "abc", URL, t);
        // Re-point the challenge without re-signing.
        let mut json: serde_json::Value = serde_json::from_str(&e.as_json()).unwrap();
        json["tags"][0][1] = "xyz".into();
        let forged: Event = serde_json::from_value(json).unwrap();
        assert_eq!(
            verify(&forged, "xyz", &policy(&k), t),
            Err(AuthError::BadSignature)
        );
        // Claim an allowed key on someone else's signature.
        let other = Keys::generate();
        let mut json: serde_json::Value =
            serde_json::from_str(&signed(&other, AUTH_KIND, "abc", URL, t).as_json()).unwrap();
        json["pubkey"] = k.public_key().to_hex().into();
        let forged: Event = serde_json::from_value(json).unwrap();
        assert_eq!(
            verify(&forged, "abc", &policy(&k), t),
            Err(AuthError::BadSignature)
        );
    }
}
