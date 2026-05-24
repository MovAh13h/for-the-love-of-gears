use for_the_love_of_gears::tooth::{Addendum, Clearance, Dedendum, ToothDepth, ToothThickness};
use std::f64::consts::PI;

#[test]
fn addendum_m2() {
    assert_eq!(Addendum::from_module(2.0).value(), 2.0);
}

#[test]
fn dedendum_m2() {
    assert_eq!(Dedendum::from_module(2.0).value(), 2.5);
}

#[test]
fn tooth_depth_m2() {
    assert_eq!(ToothDepth::from_module(2.0).value(), 4.5);
}

#[test]
fn tooth_depth_equals_addendum_plus_dedendum() {
    let m = 3.0;
    let sum = Addendum::from_module(m).value() + Dedendum::from_module(m).value();
    assert!((ToothDepth::from_module(m).value() - sum).abs() < 1e-10);
}

#[test]
fn tooth_thickness_m2() {
    assert!((ToothThickness::from_module(2.0).value() - PI).abs() < 1e-10);
}

#[test]
fn clearance_m4() {
    assert_eq!(Clearance::from_module(4.0).value(), 1.0);
}

#[test]
fn clearance_equals_dedendum_minus_addendum() {
    let m = 5.0;
    let diff = Dedendum::from_module(m).value() - Addendum::from_module(m).value();
    assert!((Clearance::from_module(m).value() - diff).abs() < 1e-10);
}
