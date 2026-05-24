use for_the_love_of_gears::diameter::{BaseDiameter, ReferenceDiameter, RootDiameter, TipDiameter};

#[test]
fn reference_diameter_m2_z20() {
    assert_eq!(ReferenceDiameter::new(2.0, 20).value(), 40.0);
}

#[test]
fn tip_diameter_m2_z20() {
    assert_eq!(TipDiameter::new(2.0, 20).value(), 44.0);
}

#[test]
fn root_diameter_m2_z20() {
    assert_eq!(RootDiameter::new(2.0, 20).value(), 35.0);
}

#[test]
fn base_diameter_20deg_m2_z20() {
    let db = BaseDiameter::new(2.0, 20, 20.0).value();
    let expected = 40.0 * 20.0_f64.to_radians().cos();
    assert!((db - expected).abs() < 1e-10);
}
