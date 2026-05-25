use for_the_love_of_gears::gear::{Gear, GearError};
use std::f64::consts::PI;

fn gear_m2_z20() -> Gear {
    Gear::builder().module(2.0).teeth(20).build().unwrap()
}

// --- Builder: required fields ---

#[test]
fn build_requires_module() {
    assert_eq!(Gear::builder().teeth(20).build(), Err(GearError::ModuleRequired));
}

#[test]
fn build_requires_teeth() {
    assert_eq!(Gear::builder().module(2.0).build(), Err(GearError::TeethRequired));
}

// --- Builder: validation ---

#[test]
fn build_rejects_zero_module() {
    assert_eq!(
        Gear::builder().module(0.0).teeth(20).build(),
        Err(GearError::ModuleMustBePositive)
    );
}

#[test]
fn build_rejects_negative_module() {
    assert_eq!(
        Gear::builder().module(-1.0).teeth(20).build(),
        Err(GearError::ModuleMustBePositive)
    );
}

#[test]
fn build_rejects_zero_teeth() {
    assert_eq!(
        Gear::builder().module(2.0).teeth(0).build(),
        Err(GearError::TeethMustBePositive)
    );
}

#[test]
fn build_rejects_one_tooth() {
    assert_eq!(
        Gear::builder().module(2.0).teeth(1).build(),
        Err(GearError::TeethTooFew)
    );
}

#[test]
fn build_rejects_two_teeth() {
    assert_eq!(
        Gear::builder().module(2.0).teeth(2).build(),
        Err(GearError::TeethTooFew)
    );
}

#[test]
fn build_accepts_three_teeth() {
    assert!(Gear::builder().module(2.0).teeth(3).build().is_ok());
}

#[test]
fn build_rejects_zero_pressure_angle() {
    assert_eq!(
        Gear::builder().module(2.0).teeth(20).pressure_angle(0.0).build(),
        Err(GearError::PressureAngleMustBePositive)
    );
}

// --- Builder: defaults ---

#[test]
fn build_defaults_pressure_angle_to_20() {
    assert_eq!(gear_m2_z20().pressure_angle(), 20.0);
}

#[test]
fn build_accepts_custom_pressure_angle() {
    let gear = Gear::builder().module(2.0).teeth(20).pressure_angle(14.5).build().unwrap();
    assert_eq!(gear.pressure_angle(), 14.5);
}

// --- Derived geometry ---

#[test]
fn reference_diameter() {
    assert_eq!(gear_m2_z20().reference_diameter(), 40.0);
}

#[test]
fn tip_diameter() {
    assert_eq!(gear_m2_z20().tip_diameter(), 44.0);
}

#[test]
fn root_diameter() {
    assert_eq!(gear_m2_z20().root_diameter(), 35.0);
}

#[test]
fn base_diameter() {
    let expected = 40.0 * 20.0_f64.to_radians().cos();
    assert!((gear_m2_z20().base_diameter() - expected).abs() < 1e-10);
}

#[test]
fn addendum() {
    assert_eq!(gear_m2_z20().addendum(), 2.0);
}

#[test]
fn dedendum() {
    assert_eq!(gear_m2_z20().dedendum(), 2.5);
}

#[test]
fn tooth_depth() {
    assert_eq!(gear_m2_z20().tooth_depth(), 4.5);
}

#[test]
fn tooth_thickness() {
    assert!((gear_m2_z20().tooth_thickness() - PI).abs() < 1e-10);
}

#[test]
fn clearance() {
    assert_eq!(gear_m2_z20().clearance(), 0.5);
}

#[test]
fn circular_pitch() {
    assert!((gear_m2_z20().circular_pitch() - 2.0 * PI).abs() < 1e-10);
}

#[test]
fn diametral_pitch() {
    assert!((gear_m2_z20().diametral_pitch() - 12.7).abs() < 1e-10);
}

// --- Gear pair ---

#[test]
fn can_mesh_with_same_module() {
    let g1 = Gear::builder().module(2.0).teeth(20).build().unwrap();
    let g2 = Gear::builder().module(2.0).teeth(40).build().unwrap();
    assert!(g1.can_mesh_with(&g2));
}

#[test]
fn cannot_mesh_with_different_module() {
    let g1 = Gear::builder().module(2.0).teeth(20).build().unwrap();
    let g2 = Gear::builder().module(3.0).teeth(20).build().unwrap();
    assert!(!g1.can_mesh_with(&g2));
}

#[test]
fn cannot_mesh_with_different_pressure_angle() {
    let g1 = Gear::builder().module(2.0).teeth(20).pressure_angle(20.0).build().unwrap();
    let g2 = Gear::builder().module(2.0).teeth(40).pressure_angle(14.5).build().unwrap();
    assert!(!g1.can_mesh_with(&g2));
}

#[test]
fn center_distance_to() {
    let g1 = Gear::builder().module(2.0).teeth(20).build().unwrap();
    let g2 = Gear::builder().module(2.0).teeth(40).build().unwrap();
    assert_eq!(g1.center_distance_to(&g2), 60.0);
}

#[test]
fn gear_ratio_reduction() {
    let driver = Gear::builder().module(2.0).teeth(20).build().unwrap();
    let driven = Gear::builder().module(2.0).teeth(40).build().unwrap();
    assert_eq!(driver.gear_ratio_to(&driven), 2.0);
}

#[test]
fn gear_ratio_increase() {
    let driver = Gear::builder().module(2.0).teeth(40).build().unwrap();
    let driven = Gear::builder().module(2.0).teeth(20).build().unwrap();
    assert_eq!(driver.gear_ratio_to(&driven), 0.5);
}

// --- Backlash ---

#[test]
fn thinned_tooth_thickness_err_on_negative_backlash() {
    use for_the_love_of_gears::BacklashError;
    assert_eq!(gear_m2_z20().thinned_tooth_thickness(-0.01), Err(BacklashError::NegativeBacklash));
}

#[test]
fn normal_backlash_err_on_negative_backlash() {
    use for_the_love_of_gears::BacklashError;
    assert_eq!(gear_m2_z20().normal_backlash(-0.01), Err(BacklashError::NegativeBacklash));
}

#[test]
fn thinned_tooth_thickness() {
    let g = gear_m2_z20();
    let s = g.thinned_tooth_thickness(0.08).unwrap();
    assert!((s - (PI - 0.04)).abs() < 1e-10);
}

#[test]
fn normal_backlash() {
    let g = gear_m2_z20();
    let jn = g.normal_backlash(0.08).unwrap();
    let expected = 0.08 * 20.0_f64.to_radians().cos();
    assert!((jn - expected).abs() < 1e-10);
}

#[test]
fn normal_backlash_less_than_circular() {
    let g = gear_m2_z20();
    assert!(g.normal_backlash(0.08).unwrap() < 0.08);
}
