use for_the_love_of_gears::{
    gear::{Gear, GearError},
    module::Module,
};
use std::f64::consts::PI;

fn gear_m2_z20() -> Gear {
    Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap()
}

// --- Builder: required fields ---

#[test]
fn build_requires_module() {
    assert_eq!(
        Gear::builder().teeth(20).build(),
        Err(GearError::ModuleRequired)
    );
}

#[test]
fn build_requires_teeth() {
    assert_eq!(
        Gear::builder().module(Module::Specified(2.0)).build(),
        Err(GearError::TeethRequired)
    );
}

// --- Builder: validation ---

#[test]
fn build_rejects_zero_module() {
    assert_eq!(
        Gear::builder()
            .module(Module::Specified(0.0))
            .teeth(20)
            .build(),
        Err(GearError::ModuleMustBePositive)
    );
}

#[test]
fn build_rejects_negative_module() {
    assert_eq!(
        Gear::builder()
            .module(Module::Specified(-1.0))
            .teeth(20)
            .build(),
        Err(GearError::ModuleMustBePositive)
    );
}

#[test]
fn build_rejects_zero_teeth() {
    assert_eq!(
        Gear::builder()
            .module(Module::Specified(2.0))
            .teeth(0)
            .build(),
        Err(GearError::TeethMustBePositive)
    );
}

#[test]
fn build_rejects_zero_pressure_angle() {
    assert_eq!(
        Gear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .pressure_angle(0.0)
            .build(),
        Err(GearError::PressureAngleMustBePositive)
    );
}

#[test]
fn build_rejects_zero_face_width() {
    assert_eq!(
        Gear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .face_width(0.0)
            .build(),
        Err(GearError::FaceWidthMustBePositive)
    );
}

#[test]
fn build_rejects_negative_face_width() {
    assert_eq!(
        Gear::builder()
            .module(Module::Specified(2.0))
            .teeth(20)
            .face_width(-5.0)
            .build(),
        Err(GearError::FaceWidthMustBePositive)
    );
}

// --- Builder: defaults and optional fields ---

#[test]
fn build_defaults_pressure_angle_to_20() {
    assert_eq!(gear_m2_z20().pressure_angle(), 20.0);
}

#[test]
fn build_accepts_custom_pressure_angle() {
    let gear = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .pressure_angle(14.5)
        .build()
        .unwrap();
    assert_eq!(gear.pressure_angle(), 14.5);
}

#[test]
fn build_face_width_is_none_by_default() {
    assert_eq!(gear_m2_z20().face_width(), None);
}

#[test]
fn build_stores_face_width() {
    let gear = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .face_width(25.0)
        .build()
        .unwrap();
    assert_eq!(gear.face_width(), Some(25.0));
}

// --- Derived geometry ---

#[test]
fn reference_diameter() {
    assert_eq!(gear_m2_z20().reference_diameter().value(), 40.0);
}

#[test]
fn tip_diameter() {
    assert_eq!(gear_m2_z20().tip_diameter().value(), 44.0);
}

#[test]
fn root_diameter() {
    assert_eq!(gear_m2_z20().root_diameter().value(), 35.0);
}

#[test]
fn base_diameter() {
    let expected = 40.0 * 20.0_f64.to_radians().cos();
    assert!((gear_m2_z20().base_diameter().value() - expected).abs() < 1e-10);
}

#[test]
fn addendum() {
    assert_eq!(gear_m2_z20().addendum().value(), 2.0);
}

#[test]
fn dedendum() {
    assert_eq!(gear_m2_z20().dedendum().value(), 2.5);
}

#[test]
fn tooth_depth() {
    assert_eq!(gear_m2_z20().tooth_depth().value(), 4.5);
}

#[test]
fn tooth_thickness() {
    assert!((gear_m2_z20().tooth_thickness().value() - PI).abs() < 1e-10);
}

#[test]
fn clearance() {
    assert_eq!(gear_m2_z20().clearance().value(), 0.5);
}

#[test]
fn circular_pitch() {
    assert!((gear_m2_z20().circular_pitch().value() - 2.0 * PI).abs() < 1e-10);
}

// --- Gear pair ---

#[test]
fn can_mesh_with_same_module() {
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
    assert!(g1.can_mesh_with(&g2));
}

#[test]
fn cannot_mesh_with_different_module() {
    let g1 = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap();
    let g2 = Gear::builder()
        .module(Module::Specified(3.0))
        .teeth(20)
        .build()
        .unwrap();
    assert!(!g1.can_mesh_with(&g2));
}

#[test]
fn center_distance_to() {
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
    // d1=40, d2=80 → a=60
    assert_eq!(g1.center_distance_to(&g2).value(), 60.0);
}

#[test]
fn gear_ratio_reduction() {
    let driver = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap();
    let driven = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .build()
        .unwrap();
    assert_eq!(driver.gear_ratio_to(&driven), 2.0);
}

#[test]
fn gear_ratio_increase() {
    let driver = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .build()
        .unwrap();
    let driven = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap();
    assert_eq!(driver.gear_ratio_to(&driven), 0.5);
}
