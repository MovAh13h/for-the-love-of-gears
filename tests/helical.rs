use for_the_love_of_gears::{
    gear::GearError,
    helical::{HelicalGear, HelixHand},
    module::Module,
};
use std::f64::consts::PI;

fn gear_mn2_z20_psi20_right() -> HelicalGear {
    HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .build()
        .unwrap()
}

// --- Builder: required fields ---

#[test]
fn build_requires_module() {
    assert_eq!(
        HelicalGear::builder()
            .teeth(20)
            .helix_angle(20.0)
            .helix_hand(HelixHand::Right)
            .build(),
        Err(GearError::ModuleRequired)
    );
}

#[test]
fn build_requires_teeth() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(2.0))
            .helix_angle(20.0)
            .helix_hand(HelixHand::Right)
            .build(),
        Err(GearError::TeethRequired)
    );
}

#[test]
fn build_requires_helix_angle() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .helix_hand(HelixHand::Right)
            .build(),
        Err(GearError::HelixAngleRequired)
    );
}

#[test]
fn build_requires_helix_hand() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .helix_angle(20.0)
            .build(),
        Err(GearError::HelixHandRequired)
    );
}

// --- Builder: validation ---

#[test]
fn build_rejects_zero_module() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(0.0))
            .teeth(20)
            .helix_angle(20.0)
            .helix_hand(HelixHand::Right)
            .build(),
        Err(GearError::ModuleMustBePositive)
    );
}

#[test]
fn build_rejects_zero_teeth() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(2.0))
            .teeth(0)
            .helix_angle(20.0)
            .helix_hand(HelixHand::Right)
            .build(),
        Err(GearError::TeethMustBePositive)
    );
}

#[test]
fn build_rejects_zero_helix_angle() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .helix_angle(0.0)
            .helix_hand(HelixHand::Right)
            .build(),
        Err(GearError::HelixAngleMustBePositive)
    );
}

#[test]
fn build_rejects_negative_helix_angle() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .helix_angle(-10.0)
            .helix_hand(HelixHand::Right)
            .build(),
        Err(GearError::HelixAngleMustBePositive)
    );
}

#[test]
fn build_rejects_helix_angle_of_90() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .helix_angle(90.0)
            .helix_hand(HelixHand::Right)
            .build(),
        Err(GearError::HelixAngleMustBeLessThan90)
    );
}

#[test]
fn build_rejects_zero_pressure_angle() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .helix_angle(20.0)
            .helix_hand(HelixHand::Right)
            .normal_pressure_angle(0.0)
            .build(),
        Err(GearError::PressureAngleMustBePositive)
    );
}

#[test]
fn build_rejects_zero_face_width() {
    assert_eq!(
        HelicalGear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .helix_angle(20.0)
            .helix_hand(HelixHand::Right)
            .face_width(0.0)
            .build(),
        Err(GearError::FaceWidthMustBePositive)
    );
}

// --- Builder: defaults and optional fields ---

#[test]
fn build_defaults_pressure_angle_to_20() {
    assert_eq!(gear_mn2_z20_psi20_right().normal_pressure_angle(), 20.0);
}

#[test]
fn build_face_width_is_none_by_default() {
    assert_eq!(gear_mn2_z20_psi20_right().face_width(), None);
}

#[test]
fn build_stores_face_width() {
    let gear = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .face_width(30.0)
        .build()
        .unwrap();
    assert_eq!(gear.face_width(), Some(30.0));
}

// --- Derived — transverse values ---

#[test]
fn transverse_module() {
    let g = gear_mn2_z20_psi20_right();
    let expected = 2.0 / 20.0_f64.to_radians().cos();
    assert!((g.transverse_module() - expected).abs() < 1e-10);
}

#[test]
fn transverse_module_greater_than_normal() {
    let g = gear_mn2_z20_psi20_right();
    assert!(g.transverse_module() > g.normal_module().value());
}

#[test]
fn transverse_pressure_angle() {
    let g = gear_mn2_z20_psi20_right();
    let alpha_n = 20.0_f64.to_radians();
    let psi = 20.0_f64.to_radians();
    let expected = (alpha_n.tan() / psi.cos()).atan().to_degrees();
    assert!((g.transverse_pressure_angle() - expected).abs() < 1e-10);
}

#[test]
fn transverse_pressure_angle_greater_than_normal() {
    let g = gear_mn2_z20_psi20_right();
    assert!(g.transverse_pressure_angle() > g.normal_pressure_angle());
}

// --- Derived geometry ---

#[test]
fn reference_diameter_uses_transverse_module() {
    let g = gear_mn2_z20_psi20_right();
    let expected = g.transverse_module() * 20.0;
    assert!((g.reference_diameter().value() - expected).abs() < 1e-10);
}

#[test]
fn tip_diameter() {
    let g = gear_mn2_z20_psi20_right();
    let expected = g.reference_diameter().value() + 2.0 * g.normal_module().value();
    assert!((g.tip_diameter().value() - expected).abs() < 1e-10);
}

#[test]
fn root_diameter() {
    let g = gear_mn2_z20_psi20_right();
    let expected = g.reference_diameter().value() - 2.5 * g.normal_module().value();
    assert!((g.root_diameter().value() - expected).abs() < 1e-10);
}

#[test]
fn base_diameter() {
    let g = gear_mn2_z20_psi20_right();
    let expected =
        g.reference_diameter().value() * g.transverse_pressure_angle().to_radians().cos();
    assert!((g.base_diameter().value() - expected).abs() < 1e-10);
}

#[test]
fn addendum_uses_normal_module() {
    assert_eq!(gear_mn2_z20_psi20_right().addendum().value(), 2.0);
}

#[test]
fn dedendum_uses_normal_module() {
    assert_eq!(gear_mn2_z20_psi20_right().dedendum().value(), 2.5);
}

#[test]
fn tooth_depth_uses_normal_module() {
    assert_eq!(gear_mn2_z20_psi20_right().tooth_depth().value(), 4.5);
}

#[test]
fn clearance_uses_normal_module() {
    assert_eq!(gear_mn2_z20_psi20_right().clearance().value(), 0.5);
}

#[test]
fn normal_circular_pitch() {
    let g = gear_mn2_z20_psi20_right();
    let expected = std::f64::consts::PI * 2.0;
    assert!((g.normal_circular_pitch().value() - expected).abs() < 1e-10);
}

#[test]
fn transverse_circular_pitch_greater_than_normal() {
    let g = gear_mn2_z20_psi20_right();
    assert!(g.transverse_circular_pitch().value() > g.normal_circular_pitch().value());
}

#[test]
fn lead_equals_axial_pitch_times_teeth() {
    let g = gear_mn2_z20_psi20_right();
    let expected = g.axial_pitch().value() * 20.0;
    assert!((g.lead().value() - expected).abs() < 1e-10);
}

// --- Gear pair ---

#[test]
fn can_mesh_with_opposite_hand() {
    let right = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .build()
        .unwrap();
    let left = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Left)
        .build()
        .unwrap();
    assert!(right.can_mesh_with(&left));
}

#[test]
fn cannot_mesh_with_same_hand() {
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
        .helix_hand(HelixHand::Right)
        .build()
        .unwrap();
    assert!(!g1.can_mesh_with(&g2));
}

#[test]
fn cannot_mesh_with_different_module() {
    let g1 = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .build()
        .unwrap();
    let g2 = HelicalGear::builder()
        .module(Module::Specified(3.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Left)
        .build()
        .unwrap();
    assert!(!g1.can_mesh_with(&g2));
}

#[test]
fn cannot_mesh_with_different_helix_angle() {
    let g1 = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .build()
        .unwrap();
    let g2 = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(25.0)
        .helix_hand(HelixHand::Left)
        .build()
        .unwrap();
    assert!(!g1.can_mesh_with(&g2));
}

#[test]
fn gear_ratio() {
    let driver = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .build()
        .unwrap();
    let driven = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Left)
        .build()
        .unwrap();
    assert_eq!(driver.gear_ratio_to(&driven), 2.0);
    assert_eq!(driven.gear_ratio_to(&driver), 0.5);
}

#[test]
fn center_distance() {
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
    let expected = (g1.reference_diameter().value() + g2.reference_diameter().value()) / 2.0;
    assert!((g1.center_distance_to(&g2).value() - expected).abs() < 1e-10);
}

// --- Contact ratio ---

#[test]
fn transverse_contact_ratio_positive() {
    let g1 = gear_mn2_z20_psi20_right();
    let g2 = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Left)
        .build()
        .unwrap();
    assert!(g1.transverse_contact_ratio_with(&g2).value() > 1.0);
}

#[test]
fn overlap_ratio_requires_face_width() {
    assert!(gear_mn2_z20_psi20_right().overlap_ratio().is_none());
}

#[test]
fn overlap_ratio_formula() {
    // b=30mm, ψ=20°, mn=2mm → εβ = 30·sin(20°) / (π·2)
    let g = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .face_width(30.0)
        .build()
        .unwrap();
    let expected = 30.0 * 20.0_f64.to_radians().sin() / (PI * 2.0);
    assert!((g.overlap_ratio().unwrap().value() - expected).abs() < 1e-10);
}

#[test]
fn total_contact_ratio_requires_face_width() {
    let g1 = gear_mn2_z20_psi20_right();
    let g2 = HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Left)
        .build()
        .unwrap();
    assert!(g1.total_contact_ratio_with(&g2).is_none());
}

#[test]
fn total_contact_ratio_greater_than_transverse() {
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
    let ea = g1.transverse_contact_ratio_with(&g2).value();
    let eg = g1.total_contact_ratio_with(&g2).unwrap().value();
    assert!(eg > ea);
}
