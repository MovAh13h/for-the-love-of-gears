use for_the_love_of_gears::center_distance::CenterDistance;

#[test]
fn from_reference_diameters() {
    assert_eq!(CenterDistance::from_reference_diameters(40.0, 60.0).value(), 50.0);
}

#[test]
fn from_module_and_teeth() {
    assert_eq!(CenterDistance::from_module_and_teeth(2.0, 20, 30).value(), 50.0);
}

#[test]
fn both_methods_agree() {
    let (m, z1, z2) = (2.0, 20_u32, 30_u32);
    let a1 = CenterDistance::from_reference_diameters(m * z1 as f64, m * z2 as f64).value();
    let a2 = CenterDistance::from_module_and_teeth(m, z1, z2).value();
    assert!((a1 - a2).abs() < 1e-10);
}

#[test]
fn equal_gears_center_distance_equals_reference_diameter() {
    let (m, z) = (2.0, 20);
    assert_eq!(CenterDistance::from_module_and_teeth(m, z, z).value(), m * z as f64);
}
