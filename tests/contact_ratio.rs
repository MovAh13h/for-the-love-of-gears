use for_the_love_of_gears::{
    contact_ratio::{OverlapRatio, TotalContactRatio, TransverseContactRatio},
    gear::Gear,
    helical::{HelicalGear, HelixHand},
    module::Module,
};
use std::f64::consts::PI;

fn spur_pair() -> (Gear, Gear) {
    let g1 = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap();
    let g2 = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .build()
        .unwrap();
    (g1, g2)
}

fn helical_pair_no_fw() -> (HelicalGear, HelicalGear) {
    let g1 = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .build()
        .unwrap();
    let g2 = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Left)
        .build()
        .unwrap();
    (g1, g2)
}

fn helical_pair_fw30() -> (HelicalGear, HelicalGear) {
    let g1 = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .face_width(30.0)
        .build()
        .unwrap();
    let g2 = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Left)
        .face_width(30.0)
        .build()
        .unwrap();
    (g1, g2)
}

// --- Spur gear contact ratio ---

#[test]
fn spur_known_pair() {
    // m=2, z1=20, z2=40, α=20° → εα ≈ 1.635
    let (g1, g2) = spur_pair();
    assert!((g1.contact_ratio_with(&g2).value() - 1.635).abs() < 0.001);
}

#[test]
fn spur_contact_ratio_symmetric() {
    let (g1, g2) = spur_pair();
    assert_eq!(
        g1.contact_ratio_with(&g2).value(),
        g2.contact_ratio_with(&g1).value()
    );
}

#[test]
fn spur_contact_ratio_greater_than_one() {
    let (g1, g2) = spur_pair();
    assert!(g1.contact_ratio_with(&g2).value() > 1.0);
}

#[test]
fn spur_equal_gear_contact_ratio() {
    let g = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap();
    assert!(g.contact_ratio_with(&g).value() > 1.0);
}

// --- TransverseContactRatio::from_geometry ---

#[test]
fn from_geometry_matches_gear_method() {
    let (g1, g2) = spur_pair();
    let ra1 = g1.tip_diameter().value() / 2.0;
    let rb1 = g1.base_diameter().value() / 2.0;
    let ra2 = g2.tip_diameter().value() / 2.0;
    let rb2 = g2.base_diameter().value() / 2.0;
    let a = g1.center_distance_to(&g2).value();
    let alpha = g1.pressure_angle();
    let pb = PI * g1.module().value() * alpha.to_radians().cos();
    let manual = TransverseContactRatio::from_geometry(ra1, rb1, ra2, rb2, a, alpha, pb);
    assert_eq!(manual.value(), g1.contact_ratio_with(&g2).value());
}

// --- OverlapRatio ---

#[test]
fn overlap_ratio_formula() {
    // b=30mm, ψ=20°, mn=2mm
    let eb = OverlapRatio::new(30.0, 20.0, 2.0);
    let expected = 30.0 * 20.0_f64.to_radians().sin() / (PI * 2.0);
    assert!((eb.value() - expected).abs() < 1e-10);
}

#[test]
fn overlap_ratio_none_without_face_width() {
    let (g1, _) = helical_pair_no_fw();
    assert!(g1.overlap_ratio().is_none());
}

#[test]
fn overlap_ratio_some_with_face_width() {
    let (g1, _) = helical_pair_fw30();
    assert!(g1.overlap_ratio().is_some());
}

// --- TotalContactRatio ---

#[test]
fn total_contact_ratio_equals_transverse_plus_overlap() {
    let (g1, g2) = helical_pair_fw30();
    let ea = g1.transverse_contact_ratio_with(&g2).value();
    let eb = g1.overlap_ratio().unwrap().value();
    let eg = g1.total_contact_ratio_with(&g2).unwrap().value();
    assert!((eg - (ea + eb)).abs() < 1e-10);
}

#[test]
fn total_contact_ratio_none_without_face_width() {
    let (g1, g2) = helical_pair_no_fw();
    assert!(g1.total_contact_ratio_with(&g2).is_none());
}

#[test]
fn total_contact_ratio_greater_than_transverse() {
    let (g1, g2) = helical_pair_fw30();
    let ea = g1.transverse_contact_ratio_with(&g2).value();
    let eg = g1.total_contact_ratio_with(&g2).unwrap().value();
    assert!(eg > ea);
}

#[test]
fn helical_transverse_contact_ratio_symmetric() {
    let (g1, g2) = helical_pair_no_fw();
    assert_eq!(
        g1.transverse_contact_ratio_with(&g2).value(),
        g2.transverse_contact_ratio_with(&g1).value()
    );
}

// --- TotalContactRatio::new ---

#[test]
fn total_contact_ratio_new_sums_components() {
    let ra1 = 22.0_f64;
    let rb1 = 40.0_f64 * 20.0_f64.to_radians().cos() / 2.0;
    let ra2 = 42.0_f64;
    let rb2 = 80.0_f64 * 20.0_f64.to_radians().cos() / 2.0;
    let pb = PI * 2.0 * 20.0_f64.to_radians().cos();
    let ea = TransverseContactRatio::from_geometry(ra1, rb1, ra2, rb2, 60.0, 20.0, pb);
    let eb = OverlapRatio::new(30.0, 20.0, 2.0);
    let eg = TotalContactRatio::new(ea, eb);
    assert!((eg.value() - (ea.value() + eb.value())).abs() < 1e-10);
}
