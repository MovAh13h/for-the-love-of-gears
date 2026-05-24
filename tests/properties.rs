use for_the_love_of_gears::{
    center_distance::CenterDistance,
    diameter::{BaseDiameter, ReferenceDiameter, RootDiameter, TipDiameter},
    gear::Gear,
    module::Module,
    pitch::{CircularPitch, DiametralPitch},
    tooth::{Addendum, Clearance, Dedendum, ToothDepth},
};
use proptest::prelude::*;
use std::f64::consts::PI;

fn pos_module() -> impl Strategy<Value = f64> {
    1e-3f64..=100.0f64
}

fn any_teeth() -> impl Strategy<Value = u32> {
    1u32..=1000u32
}

fn teeth_min3() -> impl Strategy<Value = u32> {
    3u32..=1000u32
}

fn pressure_angle_deg() -> impl Strategy<Value = f64> {
    0.1f64..89.9f64
}

proptest! {
    // --- Tooth identities ---

    #[test]
    fn prop_tooth_depth_equals_addendum_plus_dedendum(m in pos_module()) {
        let h  = ToothDepth::from_module(m).value();
        let ha = Addendum::from_module(m).value();
        let hf = Dedendum::from_module(m).value();
        prop_assert!((h - (ha + hf)).abs() < 1e-10);
    }

    #[test]
    fn prop_clearance_equals_dedendum_minus_addendum(m in pos_module()) {
        let c  = Clearance::from_module(m).value();
        let ha = Addendum::from_module(m).value();
        let hf = Dedendum::from_module(m).value();
        prop_assert!((c - (hf - ha)).abs() < 1e-10);
    }

    // --- Pitch product ---

    #[test]
    fn prop_circular_diametral_product(m in pos_module()) {
        let p  = CircularPitch::from_module(m).value();
        let dp = DiametralPitch::from_module(m).value();
        prop_assert!((p * dp - PI * 25.4).abs() < 1e-6);
    }

    // --- Diameter relations ---

    #[test]
    fn prop_tip_diameter_equals_reference_plus_two_addenda(m in pos_module(), z in any_teeth()) {
        let d  = ReferenceDiameter::new(m, z).value();
        let da = TipDiameter::new(m, z).value();
        let ha = Addendum::from_module(m).value();
        prop_assert!((da - (d + 2.0 * ha)).abs() < 1e-10);
    }

    #[test]
    fn prop_root_diameter_equals_reference_minus_two_dedenda(m in pos_module(), z in any_teeth()) {
        let d  = ReferenceDiameter::new(m, z).value();
        let df = RootDiameter::new(m, z).value();
        let hf = Dedendum::from_module(m).value();
        prop_assert!((df - (d - 2.0 * hf)).abs() < 1e-10);
    }

    #[test]
    fn prop_base_diameter_equals_reference_times_cos_alpha(
        m     in pos_module(),
        z     in any_teeth(),
        alpha in pressure_angle_deg(),
    ) {
        let d  = ReferenceDiameter::new(m, z).value();
        let db = BaseDiameter::new(m, z, alpha).value();
        prop_assert!((db - d * alpha.to_radians().cos()).abs() < 1e-10);
    }

    // df < d < da for z ≥ 3 (below 3 the root circle goes negative — not a meaningful gear)
    #[test]
    fn prop_diameter_ordering(m in pos_module(), z in teeth_min3()) {
        let d  = ReferenceDiameter::new(m, z).value();
        let da = TipDiameter::new(m, z).value();
        let df = RootDiameter::new(m, z).value();
        prop_assert!(df < d, "root {} should be < pitch {}", df, d);
        prop_assert!(d < da, "pitch {} should be < tip {}", d, da);
    }

    #[test]
    fn prop_base_diameter_less_than_reference(
        m     in pos_module(),
        z     in any_teeth(),
        alpha in pressure_angle_deg(),
    ) {
        let d  = ReferenceDiameter::new(m, z).value();
        let db = BaseDiameter::new(m, z, alpha).value();
        prop_assert!(db < d, "base {} should be < reference {}", db, d);
    }

    // --- Module roundtrips ---

    #[test]
    fn prop_module_specified_roundtrip(m in pos_module()) {
        prop_assert_eq!(Module::Specified(m).value(), m);
    }

    #[test]
    fn prop_module_from_circular_pitch_roundtrip(m in pos_module()) {
        let p = PI * m;
        prop_assert!((Module::FromCircularPitch(p).value() - m).abs() < 1e-10);
    }

    #[test]
    fn prop_module_from_pitch_circle_diameter_roundtrip(m in pos_module(), z in any_teeth()) {
        let d = m * z as f64;
        let computed = Module::FromPitchCircleDiameter { pitch_circle_diameter: d, teeth: z }.value();
        prop_assert!((computed - m).abs() < 1e-10);
    }

    // --- Center distance ---

    #[test]
    fn prop_center_distance_symmetric(d1 in pos_module(), d2 in pos_module()) {
        let a1 = CenterDistance::from_reference_diameters(d1, d2).value();
        let a2 = CenterDistance::from_reference_diameters(d2, d1).value();
        prop_assert_eq!(a1, a2);
    }

    #[test]
    fn prop_center_distance_both_methods_agree(m in pos_module(), z1 in any_teeth(), z2 in any_teeth()) {
        let a1 = CenterDistance::from_reference_diameters(m * z1 as f64, m * z2 as f64).value();
        let a2 = CenterDistance::from_module_and_teeth(m, z1, z2).value();
        prop_assert!((a1 - a2).abs() < 1e-10);
    }

    #[test]
    fn prop_center_distance_equal_gears_equals_reference_diameter(m in pos_module(), z in any_teeth()) {
        let a = CenterDistance::from_module_and_teeth(m, z, z).value();
        let d = ReferenceDiameter::new(m, z).value();
        prop_assert!((a - d).abs() < 1e-10);
    }

    // --- Gear pair ---

    #[test]
    fn prop_gear_center_distance_symmetric(
        m  in 0.1f64..=50.0f64,
        z1 in 1u32..=200u32,
        z2 in 1u32..=200u32,
    ) {
        let g1 = Gear::builder().module(Module::Specified(m)).teeth(z1).build().unwrap();
        let g2 = Gear::builder().module(Module::Specified(m)).teeth(z2).build().unwrap();
        prop_assert_eq!(
            g1.center_distance_to(&g2).value(),
            g2.center_distance_to(&g1).value()
        );
    }

    #[test]
    fn prop_gear_ratio_reciprocal(
        m  in 0.1f64..=50.0f64,
        z1 in 1u32..=200u32,
        z2 in 1u32..=200u32,
    ) {
        let g1 = Gear::builder().module(Module::Specified(m)).teeth(z1).build().unwrap();
        let g2 = Gear::builder().module(Module::Specified(m)).teeth(z2).build().unwrap();
        let product = g1.gear_ratio_to(&g2) * g2.gear_ratio_to(&g1);
        prop_assert!((product - 1.0).abs() < 1e-10);
    }

    #[test]
    fn prop_gear_ratio_self_is_one(m in 0.1f64..=50.0f64, z in 1u32..=200u32) {
        let g = Gear::builder().module(Module::Specified(m)).teeth(z).build().unwrap();
        prop_assert_eq!(g.gear_ratio_to(&g), 1.0);
    }
}
