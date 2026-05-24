use for_the_love_of_gears::pitch::{CircularPitch, DiametralPitch};
use std::f64::consts::PI;

#[test]
fn circular_pitch_m1() {
    assert!((CircularPitch::from_module(1.0).value() - PI).abs() < 1e-10);
}

#[test]
fn circular_pitch_m2() {
    assert!((CircularPitch::from_module(2.0).value() - 2.0 * PI).abs() < 1e-10);
}

#[test]
fn diametral_pitch_m1() {
    assert!((DiametralPitch::from_module(1.0).value() - 25.4).abs() < 1e-10);
}

#[test]
fn diametral_pitch_m2() {
    assert!((DiametralPitch::from_module(2.0).value() - 12.7).abs() < 1e-10);
}
