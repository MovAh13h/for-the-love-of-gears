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
fn tip_diameter_equals_reference_plus_two_addenda() {
    let (m, z) = (3.0, 15);
    let d = ReferenceDiameter::new(m, z).value();
    assert!((TipDiameter::new(m, z).value() - (d + 2.0 * m)).abs() < 1e-10);
}

#[test]
fn root_diameter_equals_reference_minus_two_dedenda() {
    let (m, z) = (3.0, 15);
    let d = ReferenceDiameter::new(m, z).value();
    assert!((RootDiameter::new(m, z).value() - (d - 2.0 * 1.25 * m)).abs() < 1e-10);
}

#[test]
fn base_diameter_20deg_m2_z20() {
    let db = BaseDiameter::new(2.0, 20, 20.0).value();
    let expected = 40.0 * 20.0_f64.to_radians().cos();
    assert!((db - expected).abs() < 1e-10);
}
