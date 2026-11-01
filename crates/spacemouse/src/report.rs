//! Decoding of HID input reports.
//!
//! A hidraw `read` returns the report ID as the first byte, followed by the payload.
//! (WebHID strips the ID; this module does not expect that form.) Reports are
//! recognised by **(report ID, payload length)**, never by product ID, so every model
//! that sends the same layout shares one decoder and unknown reports are ignored
//! rather than treated as errors.
//!
//! | Report ID | Payload | Meaning |
//! |---|---|---|
//! | 1 | 6 bytes | translation `x, y, z`, three little-endian `i16` |
//! | 1 | 12 bytes | all six axes in one report: `x, y, z, rx, ry, rz` |
//! | 2 | 6 bytes | rotation `rx, ry, rz`, three little-endian `i16` |
//! | 3 | 1–8 bytes | button bitmask, little-endian; bit 0 is button 1 |
//! | anything else | — | ignored (battery level, LED echo, vendor reports) |
//!
//! The first three rows are what an existing browser (WebHID) decoder for these devices
//! relies on, and what public driver sources are recalled to describe for the
//! SpaceMouse Compact, Wireless, Pro and Enterprise. Those sources were not re-read
//! here, and the layouts have **not** been checked against a physical device; see
//! ADR 0002 for the verification step.
//!
//! Only the low 32 buttons are kept: a bitmask wider than four bytes loses its upper
//! bytes.

/// Number of motion axes: `x, y, z, rx, ry, rz`.
pub const AXES: usize = 6;

/// One recognised input report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Report {
    /// Translation `x, y, z` in raw counts.
    Translation([i16; 3]),
    /// Rotation `rx, ry, rz` in raw counts.
    Rotation([i16; 3]),
    /// All six axes in raw counts.
    Combined([i16; AXES]),
    /// The button bitmask, bit 0 = button 1.
    Buttons(u32),
}

/// The device's last known raw state, accumulated from reports.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawState {
    /// `x, y, z, rx, ry, rz` in raw counts.
    pub axes: [i16; AXES],
    /// The button bitmask.
    pub buttons: u32,
}

impl RawState {
    /// Fold one report into the state.
    pub fn apply(&mut self, report: Report) {
        match report {
            Report::Translation(t) => self.axes[..3].copy_from_slice(&t),
            Report::Rotation(r) => self.axes[3..].copy_from_slice(&r),
            Report::Combined(c) => self.axes = c,
            Report::Buttons(b) => self.buttons = b,
        }
    }
}

fn axes<const N: usize>(payload: &[u8]) -> [i16; N] {
    let mut out = [0i16; N];
    for (slot, pair) in out.iter_mut().zip(payload.chunks_exact(2)) {
        *slot = i16::from_le_bytes([pair[0], pair[1]]);
    }
    out
}

/// Decode one hidraw input report (report ID first). `None` for anything the table in
/// the module docs does not list.
pub fn parse(report: &[u8]) -> Option<Report> {
    let (&id, payload) = report.split_first()?;
    match (id, payload.len()) {
        (1, 6) => Some(Report::Translation(axes(payload))),
        (1, 12) => Some(Report::Combined(axes(payload))),
        (2, 6) => Some(Report::Rotation(axes(payload))),
        (3, 1..=8) => {
            let mut word = [0u8; 4];
            for (slot, b) in word.iter_mut().zip(payload) {
                *slot = *b;
            }
            Some(Report::Buttons(u32::from_le_bytes(word)))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn le(v: &[i16]) -> Vec<u8> {
        v.iter().flat_map(|x| x.to_le_bytes()).collect()
    }

    #[test]
    fn translation_and_rotation_are_little_endian_i16() {
        let mut r = vec![1];
        r.extend(le(&[350, -350, 1]));
        assert_eq!(parse(&r), Some(Report::Translation([350, -350, 1])));
        let mut r = vec![2];
        r.extend(le(&[-1, 2, i16::MIN]));
        assert_eq!(parse(&r), Some(Report::Rotation([-1, 2, i16::MIN])));
    }

    #[test]
    fn a_twelve_byte_report_one_carries_all_six_axes() {
        let mut r = vec![1];
        r.extend(le(&[1, 2, 3, 4, 5, 6]));
        assert_eq!(parse(&r), Some(Report::Combined([1, 2, 3, 4, 5, 6])));
    }

    #[test]
    fn buttons_are_a_little_endian_mask_of_any_width_up_to_eight() {
        assert_eq!(parse(&[3, 0b101]), Some(Report::Buttons(0b101)));
        assert_eq!(parse(&[3, 0x00, 0x80]), Some(Report::Buttons(0x8000)));
        assert_eq!(
            parse(&[3, 1, 0, 0, 0x80, 0xFF, 0xFF, 0xFF, 0xFF]),
            Some(Report::Buttons(0x8000_0001)),
            "bytes above the fourth are dropped"
        );
        assert_eq!(parse(&[3]), None, "an empty mask is not a report");
        assert_eq!(parse(&[3, 0, 0, 0, 0, 0, 0, 0, 0, 0]), None);
    }

    #[test]
    fn unknown_ids_and_lengths_are_ignored_not_errors() {
        for r in [
            &[][..],
            &[0],
            &[1],
            &[1, 0, 0],
            &[1, 0, 0, 0, 0, 0, 0, 0],
            &[2, 0, 0, 0, 0, 0],
            &[4, 90],
            &[0x17, 1, 2, 3],
        ] {
            assert_eq!(parse(r), None, "{r:?}");
        }
    }

    #[test]
    fn state_accumulates_across_reports() {
        let mut s = RawState::default();
        s.apply(Report::Translation([1, 2, 3]));
        s.apply(Report::Rotation([4, 5, 6]));
        s.apply(Report::Buttons(2));
        assert_eq!(
            s,
            RawState {
                axes: [1, 2, 3, 4, 5, 6],
                buttons: 2
            }
        );
        s.apply(Report::Combined([0; 6]));
        assert_eq!(s.axes, [0; 6]);
        assert_eq!(s.buttons, 2);
    }
}
