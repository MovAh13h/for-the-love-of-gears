use for_the_love_of_gears::{
    backlash::Backlash,
    center_distance::CenterDistance,
    contact_ratio::OverlapRatio,
    diameter::{BaseDiameter, ReferenceDiameter, RootDiameter, TipDiameter},
    gear::Gear,
    helical::{HelicalGear, HelixHand},
    module::Module,
    pitch::{CircularPitch, DiametralPitch},
    tooth::{Addendum, Clearance, Dedendum, ToothDepth, ToothThickness},
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

fn helical_gear(m: f64, z: u32, psi: f64, hand: HelixHand) -> HelicalGear {
    HelicalGear::builder()
        .module(Module::Specified(m))
        .teeth(z)
        .helix_angle(psi)
        .helix_hand(hand)
        .build()
        .unwrap()
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

    #[test]
    fn prop_tooth_thickness_is_half_circular_pitch(m in pos_module()) {
        let s = ToothThickness::from_module(m).value();
        let p = CircularPitch::from_module(m).value();
        prop_assert!((s - p / 2.0).abs() < 1e-10);
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

    // --- Helical gear invariants ---

    #[test]
    fn prop_helical_transverse_module_formula(
        m   in 0.1f64..=50.0f64,
        z   in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let expected = m / psi.to_radians().cos();
        prop_assert!((g.transverse_module() - expected).abs() < 1e-10);
        prop_assert!(g.transverse_module() > m);
    }

    #[test]
    fn prop_helical_transverse_pressure_angle_greater_than_normal(
        m   in 0.1f64..=50.0f64,
        z   in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!(g.transverse_pressure_angle() > g.normal_pressure_angle());
    }

    #[test]
    fn prop_helical_tooth_profile_uses_normal_module(
        m   in 0.1f64..=50.0f64,
        z   in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!((g.addendum().value()    - m).abs()        < 1e-10);
        prop_assert!((g.dedendum().value()    - 1.25 * m).abs() < 1e-10);
        prop_assert!((g.tooth_depth().value() - 2.25 * m).abs() < 1e-10);
        prop_assert!((g.clearance().value()   - 0.25 * m).abs() < 1e-10);
        prop_assert!((g.tooth_thickness().value() - PI * m / 2.0).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_diameter_ordering(
        m   in 0.1f64..=50.0f64,
        z   in teeth_min3(),
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let d  = g.reference_diameter().value();
        let da = g.tip_diameter().value();
        let df = g.root_diameter().value();
        prop_assert!(df < d,  "root {} mm should be < pitch {} mm", df, d);
        prop_assert!(d  < da, "pitch {} mm should be < tip {} mm", d, da);
    }

    #[test]
    fn prop_helical_base_diameter_formula(
        m   in 0.1f64..=50.0f64,
        z   in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let expected = g.reference_diameter().value()
            * g.transverse_pressure_angle().to_radians().cos();
        prop_assert!((g.base_diameter().value() - expected).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_tip_diameter_equals_reference_plus_two_addenda(
        m   in 0.1f64..=50.0f64,
        z   in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let d  = g.reference_diameter().value();
        let da = g.tip_diameter().value();
        let ha = g.addendum().value();
        prop_assert!((da - (d + 2.0 * ha)).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_root_diameter_equals_reference_minus_two_dedenda(
        m   in 0.1f64..=50.0f64,
        z   in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let d  = g.reference_diameter().value();
        let df = g.root_diameter().value();
        let hf = g.dedendum().value();
        prop_assert!((df - (d - 2.0 * hf)).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_transverse_circular_pitch_formula(
        m   in 0.1f64..=50.0f64,
        z   in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let expected = PI * g.transverse_module();
        prop_assert!((g.transverse_circular_pitch().value() - expected).abs() < 1e-10);
        prop_assert!(g.transverse_circular_pitch().value() > g.normal_circular_pitch().value());
    }

    #[test]
    fn prop_helical_axial_pitch_formula(
        m   in 0.1f64..=50.0f64,
        z   in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let expected = PI * m / psi.to_radians().sin();
        prop_assert!((g.axial_pitch().value() - expected).abs() < 1e-6);
    }

    #[test]
    fn prop_helical_lead_equals_axial_pitch_times_teeth(
        m   in 0.1f64..=50.0f64,
        z   in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let expected = g.axial_pitch().value() * z as f64;
        prop_assert!((g.lead().value() - expected).abs() < 1e-6);
    }

    #[test]
    fn prop_helical_center_distance_symmetric(
        m   in 0.1f64..=50.0f64,
        z1  in 1u32..=200u32,
        z2  in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g1 = helical_gear(m, z1, psi, HelixHand::Right);
        let g2 = helical_gear(m, z2, psi, HelixHand::Left);
        prop_assert_eq!(
            g1.center_distance_to(&g2).value(),
            g2.center_distance_to(&g1).value()
        );
    }

    #[test]
    fn prop_helical_gear_ratio_reciprocal(
        m   in 0.1f64..=50.0f64,
        z1  in 1u32..=200u32,
        z2  in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g1 = helical_gear(m, z1, psi, HelixHand::Right);
        let g2 = helical_gear(m, z2, psi, HelixHand::Left);
        let product = g1.gear_ratio_to(&g2) * g2.gear_ratio_to(&g1);
        prop_assert!((product - 1.0).abs() < 1e-10);
    }

    // --- Contact ratio ---

    #[test]
    fn prop_spur_contact_ratio_symmetric(
        m  in 0.1f64..=50.0f64,
        z1 in 1u32..=200u32,
        z2 in 1u32..=200u32,
    ) {
        let g1 = Gear::builder().module(Module::Specified(m)).teeth(z1).build().unwrap();
        let g2 = Gear::builder().module(Module::Specified(m)).teeth(z2).build().unwrap();
        prop_assert_eq!(
            g1.contact_ratio_with(&g2).value(),
            g2.contact_ratio_with(&g1).value()
        );
    }

    #[test]
    fn prop_spur_contact_ratio_positive(
        m  in 0.1f64..=50.0f64,
        z1 in 1u32..=200u32,
        z2 in 1u32..=200u32,
    ) {
        let g1 = Gear::builder().module(Module::Specified(m)).teeth(z1).build().unwrap();
        let g2 = Gear::builder().module(Module::Specified(m)).teeth(z2).build().unwrap();
        prop_assert!(g1.contact_ratio_with(&g2).value() > 0.0);
    }

    #[test]
    fn prop_overlap_ratio_formula(
        b   in 1.0f64..=200.0f64,
        psi in 0.1f64..89.9f64,
        m   in 0.1f64..=50.0f64,
    ) {
        let eb = OverlapRatio::new(b, psi, m);
        let expected = b * psi.to_radians().sin() / (PI * m);
        prop_assert!((eb.value() - expected).abs() < 1e-10);
    }

    #[test]
    fn prop_total_contact_ratio_equals_transverse_plus_overlap(
        m   in 0.1f64..=50.0f64,
        z1  in 1u32..=200u32,
        z2  in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
        b   in 1.0f64..=200.0f64,
    ) {
        let g1 = HelicalGear::builder()
            .module(Module::Specified(m)).teeth(z1)
            .helix_angle(psi).helix_hand(HelixHand::Right)
            .face_width(b)
            .build().unwrap();
        let g2 = HelicalGear::builder()
            .module(Module::Specified(m)).teeth(z2)
            .helix_angle(psi).helix_hand(HelixHand::Left)
            .face_width(b)
            .build().unwrap();
        let ea = g1.transverse_contact_ratio_with(&g2).value();
        let eb = g1.overlap_ratio().unwrap().value();
        let eg = g1.total_contact_ratio_with(&g2).unwrap().value();
        prop_assert!((eg - (ea + eb)).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_transverse_contact_ratio_symmetric(
        m   in 0.1f64..=50.0f64,
        z1  in 1u32..=200u32,
        z2  in 1u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g1 = helical_gear(m, z1, psi, HelixHand::Right);
        let g2 = helical_gear(m, z2, psi, HelixHand::Left);
        prop_assert_eq!(
            g1.transverse_contact_ratio_with(&g2).value(),
            g2.transverse_contact_ratio_with(&g1).value()
        );
    }

    // --- Backlash ---

    #[test]
    fn prop_spur_pair_thinning_sums_to_backlash(
        m  in pos_module(),
        z  in any_teeth(),
        jt in 0.0f64..=1.0f64,
    ) {
        let g = Gear::builder().module(Module::Specified(m)).teeth(z).build().unwrap();
        let backlash = Backlash::new(jt);
        let theoretical = g.tooth_thickness().value();
        let thinned = g.thinned_tooth_thickness(backlash).value();
        prop_assert!((2.0 * (theoretical - thinned) - jt).abs() < 1e-10);
    }

    #[test]
    fn prop_spur_thinned_thickness_less_than_theoretical(
        m  in pos_module(),
        z  in any_teeth(),
        jt in 1e-6f64..=1.0f64,
    ) {
        let g = Gear::builder().module(Module::Specified(m)).teeth(z).build().unwrap();
        let backlash = Backlash::new(jt);
        prop_assert!(g.thinned_tooth_thickness(backlash).value() < g.tooth_thickness().value());
    }

    #[test]
    fn prop_spur_normal_backlash_less_than_circular(
        m  in pos_module(),
        z  in any_teeth(),
        jt in 1e-6f64..=1.0f64,
    ) {
        let g = Gear::builder().module(Module::Specified(m)).teeth(z).build().unwrap();
        let backlash = Backlash::new(jt);
        prop_assert!(g.normal_backlash(backlash).value() < backlash.value());
    }

    #[test]
    fn prop_helical_normal_backlash_less_than_spur_normal_backlash(
        m   in pos_module(),
        z   in any_teeth(),
        psi in 0.1f64..89.9f64,
        jt  in 1e-6f64..=1.0f64,
    ) {
        let g_spur = Gear::builder().module(Module::Specified(m)).teeth(z).build().unwrap();
        let g_helical = helical_gear(m, z, psi, HelixHand::Right);
        let backlash = Backlash::new(jt);
        prop_assert!(
            g_helical.normal_backlash(backlash).value()
                < g_spur.normal_backlash(backlash).value()
        );
    }
}
