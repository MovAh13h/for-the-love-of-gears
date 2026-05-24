use for_the_love_of_gears::{
    backlash::{Backlash, NormalBacklash},
    gear::Gear,
    helical::{HelicalGear, HelixHand},
    module::Module,
};
use std::f64::consts::PI;

fn spur() -> Gear {
    Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .build()
        .unwrap()
}

fn helical() -> HelicalGear {
    HelicalGear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .build()
        .unwrap()
}

// --- Backlash ---

#[test]
fn backlash_value() {
    assert_eq!(Backlash::new(0.08).value(), 0.08);
}

#[test]
fn per_gear_thinning_is_half_backlash() {
    let jt = Backlash::new(0.08);
    assert!((jt.per_gear_thinning() - 0.04).abs() < 1e-10);
}

// --- NormalBacklash ---

#[test]
fn normal_backlash_spur_formula() {
    let jt = Backlash::new(0.08);
    let jn = NormalBacklash::from_spur(jt, 20.0);
    let expected = 0.08 * 20.0_f64.to_radians().cos();
    assert!((jn.value() - expected).abs() < 1e-10);
}

#[test]
fn normal_backlash_spur_less_than_circular() {
    let jt = Backlash::new(0.08);
    assert!(NormalBacklash::from_spur(jt, 20.0).value() < jt.value());
}

#[test]
fn normal_backlash_helical_formula() {
    let jt = Backlash::new(0.08);
    let alpha_t = 20.0_f64; // simplified — real αt would be computed from αn and ψ
    let psi = 20.0_f64;
    let jn = NormalBacklash::from_helical(jt, alpha_t, psi);
    let expected = 0.08 * alpha_t.to_radians().cos() * psi.to_radians().cos();
    assert!((jn.value() - expected).abs() < 1e-10);
}

#[test]
fn normal_backlash_helical_less_than_spur() {
    // helical has an extra cos(ψ) factor so jn_helical < jn_spur for same jt and α
    let jt = Backlash::new(0.08);
    let alpha_t = 20.0_f64;
    let psi = 20.0_f64;
    let jn_spur = NormalBacklash::from_spur(jt, alpha_t).value();
    let jn_helical = NormalBacklash::from_helical(jt, alpha_t, psi).value();
    assert!(jn_helical < jn_spur);
}

// --- Gear::thinned_tooth_thickness ---

#[test]
fn spur_thinned_tooth_thickness_formula() {
    // s' = πm/2 − jt/2 = π − 0.04
    let jt = Backlash::new(0.08);
    let s_prime = spur().thinned_tooth_thickness(jt).value();
    assert!((s_prime - (PI * 2.0 / 2.0 - 0.04)).abs() < 1e-10);
}

#[test]
fn spur_thinned_less_than_theoretical() {
    let jt = Backlash::new(0.08);
    let g = spur();
    assert!(g.thinned_tooth_thickness(jt).value() < g.tooth_thickness().value());
}

#[test]
fn spur_pair_thinning_sums_to_backlash() {
    // two identical gears each thinned by jt/2 → total gap = jt
    let jt = Backlash::new(0.08);
    let g = spur();
    let theoretical = g.tooth_thickness().value();
    let thinned = g.thinned_tooth_thickness(jt).value();
    let total_gap = 2.0 * (theoretical - thinned);
    assert!((total_gap - jt.value()).abs() < 1e-10);
}

#[test]
fn spur_zero_backlash_equals_theoretical() {
    let g = spur();
    let s = g.thinned_tooth_thickness(Backlash::new(0.0)).value();
    assert!((s - g.tooth_thickness().value()).abs() < 1e-10);
}

// --- Gear::normal_backlash ---

#[test]
fn spur_normal_backlash_matches_formula() {
    let jt = Backlash::new(0.08);
    let g = spur();
    let expected = 0.08 * 20.0_f64.to_radians().cos();
    assert!((g.normal_backlash(jt).value() - expected).abs() < 1e-10);
}

// --- HelicalGear::thinned_tooth_thickness ---

#[test]
fn helical_thinned_tooth_thickness_formula() {
    // sn' = πmn/2 − (jt/2)·cos(ψ)
    let jt = Backlash::new(0.08);
    let g = helical();
    let expected = PI * 2.0 / 2.0 - 0.04 * 20.0_f64.to_radians().cos();
    assert!((g.thinned_tooth_thickness(jt).value() - expected).abs() < 1e-10);
}

#[test]
fn helical_thinned_less_than_theoretical() {
    let jt = Backlash::new(0.08);
    let g = helical();
    assert!(g.thinned_tooth_thickness(jt).value() < g.tooth_thickness().value());
}

#[test]
fn helical_zero_backlash_equals_theoretical() {
    let g = helical();
    let s = g.thinned_tooth_thickness(Backlash::new(0.0)).value();
    assert!((s - g.tooth_thickness().value()).abs() < 1e-10);
}

// --- HelicalGear::normal_backlash ---

#[test]
fn helical_normal_backlash_less_than_circular() {
    let jt = Backlash::new(0.08);
    assert!(helical().normal_backlash(jt).value() < jt.value());
}

#[test]
fn helical_normal_backlash_less_than_spur_normal_backlash() {
    // helical has extra cos(ψ) factor on top of spur's cos(α)
    let jt = Backlash::new(0.08);
    let g_spur = spur();
    let g_helical = helical();
    assert!(g_helical.normal_backlash(jt).value() < g_spur.normal_backlash(jt).value());
}
