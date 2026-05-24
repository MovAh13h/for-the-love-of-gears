use for_the_love_of_gears::{gear::Gear, module::Module};
use std::f64::consts::PI;

fn gear_m2_z20() -> Gear {
    Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap()
}

// --- Builder ---

#[test]
fn builder_requires_module() {
    let result = Gear::builder().teeth(20).build();
    assert_eq!(result, Err("module is required"));
}

#[test]
fn builder_requires_teeth() {
    let result = Gear::builder().module(Module::Specified(2.0)).build();
    assert_eq!(result, Err("teeth is required"));
}

#[test]
fn builder_defaults_pressure_angle_to_20() {
    let gear = gear_m2_z20();
    assert_eq!(gear.pressure_angle(), 20.0);
}

#[test]
fn builder_accepts_custom_pressure_angle() {
    let gear = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .pressure_angle(14.5)
        .build()
        .unwrap();
    assert_eq!(gear.pressure_angle(), 14.5);
}

#[test]
fn builder_face_width_is_optional() {
    let gear = gear_m2_z20();
    assert_eq!(gear.face_width(), None);
}

#[test]
fn builder_stores_face_width() {
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
fn center_distance_to() {
    let g1 = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap();
    let g2 = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(30)
        .build()
        .unwrap();
    // a = (40 + 60) / 2 = 50
    assert_eq!(g1.center_distance_to(&g2).value(), 50.0);
}

#[test]
fn center_distance_symmetric() {
    let g1 = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap();
    let g2 = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(30)
        .build()
        .unwrap();
    assert_eq!(
        g1.center_distance_to(&g2).value(),
        g2.center_distance_to(&g1).value()
    );
}
