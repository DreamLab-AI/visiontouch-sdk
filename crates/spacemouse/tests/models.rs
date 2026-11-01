//! Every supported model through the whole read path, on synthesised reports.
//!
//! No hardware: a temporary directory stands in for sysfs, and a scripted reader for the
//! hidraw node. The report layouts per model are the ones recorded in `report`'s docs;
//! they are synthesised from public driver sources, not captured from devices.
use std::fs;
use std::path::PathBuf;
use visiontouch_spacemouse::device::{HidrawOpener, Opener, SUPPORTED, is_multi_axis};
use visiontouch_spacemouse::report::{RawState, Report, parse};

/// How a model is expected to report, as far as public sources say.
struct Layout {
    pid: u16,
    /// Button mask bytes it sends (Compact 2 buttons, Pro 15, Enterprise 31).
    mask_bytes: usize,
    /// Whether it sends one 12-byte report 1 rather than reports 1 and 2.
    combined: bool,
}

const LAYOUTS: &[Layout] = &[
    Layout {
        pid: 0xc635,
        mask_bytes: 1,
        combined: false,
    },
    Layout {
        pid: 0xc62e,
        mask_bytes: 2,
        combined: false,
    },
    Layout {
        pid: 0xc62f,
        mask_bytes: 2,
        combined: false,
    },
    Layout {
        pid: 0xc652,
        mask_bytes: 2,
        combined: false,
    },
    Layout {
        pid: 0xc62b,
        mask_bytes: 2,
        combined: false,
    },
    Layout {
        pid: 0xc631,
        mask_bytes: 2,
        combined: true,
    },
    Layout {
        pid: 0xc632,
        mask_bytes: 2,
        combined: true,
    },
    Layout {
        pid: 0xc633,
        mask_bytes: 4,
        combined: false,
    },
];

fn le(v: &[i16]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}

fn fold(reports: &[Vec<u8>]) -> RawState {
    let mut s = RawState::default();
    for r in reports {
        s.apply(parse(r).unwrap_or_else(|| panic!("unrecognised {r:?}")));
    }
    s
}

#[test]
fn the_layout_table_covers_every_supported_model() {
    let mut a: Vec<u16> = SUPPORTED.iter().map(|m| m.1).collect();
    let mut b: Vec<u16> = LAYOUTS.iter().map(|l| l.pid).collect();
    a.sort_unstable();
    b.sort_unstable();
    assert_eq!(a, b);
}

#[test]
fn each_model_is_found_and_its_reports_decode_to_the_same_state() {
    for (n, layout) in LAYOUTS.iter().enumerate() {
        let (vendor, product, name) = *SUPPORTED.iter().find(|m| m.1 == layout.pid).unwrap();
        let root: PathBuf =
            std::env::temp_dir().join(format!("vt-sm-model-{}-{n}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let dir = root.join("class/hidraw/hidraw0/device");
        fs::create_dir_all(&dir).unwrap();
        fs::write(
            dir.join("uevent"),
            format!("HID_ID=0003:{vendor:08X}:{product:08X}\nHID_NAME={name}\n"),
        )
        .unwrap();
        fs::write(
            dir.join("report_descriptor"),
            [0x05, 0x01, 0x09, 0x08, 0xA1, 0x01],
        )
        .unwrap();
        assert!(is_multi_axis(
            &fs::read(dir.join("report_descriptor")).unwrap()
        ));
        let found = HidrawOpener::with_roots(&root, "/dev").scan();
        assert_eq!(found.len(), 1, "{name}");
        assert_eq!((found[0].vendor, found[0].product), (vendor, product));

        let t = [120, -240, 350];
        let r = [-10, 20, -350];
        let mask = 0b1011u32;
        let mut reports = Vec::new();
        if layout.combined {
            let mut all = vec![1];
            all.extend(le(&[t[0], t[1], t[2], r[0], r[1], r[2]]));
            reports.push(all);
        } else {
            let mut a = vec![1];
            a.extend(le(&t));
            let mut b = vec![2];
            b.extend(le(&r));
            reports.push(a);
            reports.push(b);
        }
        let mut buttons = vec![3];
        buttons.extend(&mask.to_le_bytes()[..layout.mask_bytes]);
        reports.push(buttons);
        // A report this decoder does not know is ignored, not an error.
        assert_eq!(parse(&[4, 90]), None, "{name}");

        let state = fold(&reports);
        assert_eq!(state.axes, [120, -240, 350, -10, 20, -350], "{name}");
        assert_eq!(state.buttons, mask, "{name}");
        fs::remove_dir_all(&root).unwrap();
    }
}

#[test]
fn a_release_report_clears_the_axes() {
    let mut s = RawState::default();
    s.apply(Report::Combined([5, 6, 7, 8, 9, 10]));
    s.apply(parse(&[1, 0, 0, 0, 0, 0, 0]).unwrap());
    s.apply(parse(&[2, 0, 0, 0, 0, 0, 0]).unwrap());
    assert_eq!(s.axes, [0; 6]);
}
