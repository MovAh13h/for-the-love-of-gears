use for_the_love_of_gears::module::Module;
use std::f64::consts::PI;

#[test]
fn specified_returns_value() {
    assert_eq!(Module::Specified(2.0).value(), 2.0);
}

#[test]
fn from_circular_pitch() {
    let p = 2.0 * PI;
    assert!((Module::FromCircularPitch(p).value() - 2.0).abs() < 1e-10);
}

#[test]
fn from_pitch_circle_diameter() {
    // d=40, z=20 → m=2
    assert_eq!(Module::FromPitchCircleDiameter { pitch_circle_diameter: 40.0, teeth: 20 }.value(), 2.0);
}

#[test]
fn from_f64() {
    let m: Module = 4.0_f64.into();
    assert_eq!(m, Module::Specified(4.0));
}
