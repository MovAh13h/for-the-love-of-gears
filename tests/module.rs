use for_the_love_of_gears::module;
use std::f64::consts::PI;

#[test]
fn from_circular_pitch() {
    let p = 2.0 * PI;
    assert!((module::from_circular_pitch(p) - 2.0).abs() < 1e-10);
}

#[test]
fn from_diameter() {
    assert_eq!(module::from_diameter(40.0, 20), 2.0);
}
