use for_the_love_of_gears::center_distance::CenterDistance;

#[test]
fn from_reference_diameters() {
    assert_eq!(
        CenterDistance::from_reference_diameters(40.0, 60.0).value(),
        50.0
    );
}

#[test]
fn from_module_and_teeth() {
    assert_eq!(
        CenterDistance::from_module_and_teeth(2.0, 20, 30).value(),
        50.0
    );
}
