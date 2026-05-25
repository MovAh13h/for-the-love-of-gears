use for_the_love_of_gears::{
    gear::Gear,
    helical::{HelicalGear, HelixHand},
    module,
    scene::{AnyGear, GearScene},
};
use proptest::prelude::*;
use std::f64::consts::PI;

fn pos_module() -> impl Strategy<Value = f64> {
    1e-3f64..=100.0f64
}

fn any_teeth() -> impl Strategy<Value = u32> {
    3u32..=1000u32
}

fn pressure_angle_deg() -> impl Strategy<Value = f64> {
    0.1f64..89.9f64
}

fn helical_gear(m: f64, z: u32, psi: f64, hand: HelixHand) -> HelicalGear {
    HelicalGear::builder()
        .module(m)
        .teeth(z)
        .helix_angle(psi)
        .helix_hand(hand)
        .build()
        .unwrap()
}

proptest! {
    // --- Module helpers ---

    #[test]
    fn prop_module_from_circular_pitch_roundtrip(m in pos_module()) {
        let p = PI * m;
        prop_assert!((module::from_circular_pitch(p) - m).abs() < 1e-10);
    }

    #[test]
    fn prop_module_from_diameter_roundtrip(m in pos_module(), z in any_teeth()) {
        let d = m * z as f64;
        prop_assert!((module::from_diameter(d, z) - m).abs() < 1e-10);
    }

    // --- Spur gear tooth identities ---

    #[test]
    fn prop_tooth_depth_equals_addendum_plus_dedendum(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.tooth_depth() - (g.addendum() + g.dedendum())).abs() < 1e-10);
    }

    #[test]
    fn prop_clearance_equals_dedendum_minus_addendum(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.clearance() - (g.dedendum() - g.addendum())).abs() < 1e-10);
    }

    #[test]
    fn prop_tooth_thickness_is_half_circular_pitch(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.tooth_thickness() - g.circular_pitch() / 2.0).abs() < 1e-10);
    }

    #[test]
    fn prop_circular_diametral_product(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.circular_pitch() * g.diametral_pitch() - PI * 25.4).abs() < 1e-6);
    }

    // --- Spur diameter relations ---

    #[test]
    fn prop_tip_diameter_equals_reference_plus_two_addenda(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.tip_diameter() - (g.reference_diameter() + 2.0 * g.addendum())).abs() < 1e-10);
    }

    #[test]
    fn prop_root_diameter_equals_reference_minus_two_dedenda(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.root_diameter() - (g.reference_diameter() - 2.0 * g.dedendum())).abs() < 1e-10);
    }

    #[test]
    fn prop_base_diameter_equals_reference_times_cos_alpha(
        m     in pos_module(),
        z     in any_teeth(),
        alpha in pressure_angle_deg(),
    ) {
        let g = Gear::builder().module(m).teeth(z).pressure_angle(alpha).build().unwrap();
        prop_assert!((g.base_diameter() - g.reference_diameter() * alpha.to_radians().cos()).abs() < 1e-10);
    }

    #[test]
    fn prop_diameter_ordering(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!(g.root_diameter() < g.reference_diameter(), "root < pitch");
        prop_assert!(g.reference_diameter() < g.tip_diameter(), "pitch < tip");
    }

    #[test]
    fn prop_base_diameter_less_than_reference(
        m     in pos_module(),
        z     in any_teeth(),
        alpha in pressure_angle_deg(),
    ) {
        let g = Gear::builder().module(m).teeth(z).pressure_angle(alpha).build().unwrap();
        prop_assert!(g.base_diameter() < g.reference_diameter(), "base < reference");
    }

    // --- Spur gear pair ---

    #[test]
    fn prop_spur_addendum_equals_module(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.addendum() - m).abs() < 1e-10);
    }

    #[test]
    fn prop_spur_dedendum_equals_1_25_module(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.dedendum() - 1.25 * m).abs() < 1e-10);
    }

    #[test]
    fn prop_spur_tooth_depth_equals_2_25_module(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.tooth_depth() - 2.25 * m).abs() < 1e-10);
    }

    #[test]
    fn prop_spur_clearance_equals_0_25_module(m in pos_module(), z in any_teeth()) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!((g.clearance() - 0.25 * m).abs() < 1e-10);
    }

    #[test]
    fn prop_gear_center_distance_symmetric(
        m  in 0.1f64..=50.0f64,
        z1 in 3u32..=200u32,
        z2 in 3u32..=200u32,
    ) {
        let g1 = Gear::builder().module(m).teeth(z1).build().unwrap();
        let g2 = Gear::builder().module(m).teeth(z2).build().unwrap();
        prop_assert_eq!(g1.center_distance_to(&g2), g2.center_distance_to(&g1));
    }

    #[test]
    fn prop_gear_ratio_reciprocal(
        m  in 0.1f64..=50.0f64,
        z1 in 3u32..=200u32,
        z2 in 3u32..=200u32,
    ) {
        let g1 = Gear::builder().module(m).teeth(z1).build().unwrap();
        let g2 = Gear::builder().module(m).teeth(z2).build().unwrap();
        let product = g1.gear_ratio_to(&g2) * g2.gear_ratio_to(&g1);
        prop_assert!((product - 1.0).abs() < 1e-10);
    }

    #[test]
    fn prop_gear_ratio_self_is_one(m in 0.1f64..=50.0f64, z in 3u32..=200u32) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert_eq!(g.gear_ratio_to(&g), 1.0);
    }

    // --- Spur contact ratio ---

    #[test]
    fn prop_spur_contact_ratio_symmetric(
        m  in 0.1f64..=50.0f64,
        z1 in 3u32..=200u32,
        z2 in 3u32..=200u32,
    ) {
        let g1 = Gear::builder().module(m).teeth(z1).build().unwrap();
        let g2 = Gear::builder().module(m).teeth(z2).build().unwrap();
        prop_assert_eq!(g1.contact_ratio_with(&g2), g2.contact_ratio_with(&g1));
    }

    #[test]
    fn prop_spur_contact_ratio_positive(
        m  in 0.1f64..=50.0f64,
        z1 in 3u32..=200u32,
        z2 in 3u32..=200u32,
    ) {
        let g1 = Gear::builder().module(m).teeth(z1).build().unwrap();
        let g2 = Gear::builder().module(m).teeth(z2).build().unwrap();
        prop_assert!(g1.contact_ratio_with(&g2) > 0.0);
    }

    // --- Spur backlash ---

    #[test]
    fn prop_spur_pair_thinning_sums_to_backlash(
        m  in pos_module(),
        z  in any_teeth(),
        jt in 0.0f64..=1.0f64,
    ) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        let thinned = g.thinned_tooth_thickness(jt).unwrap();
        prop_assert!((2.0 * (g.tooth_thickness() - thinned) - jt).abs() < 1e-10);
    }

    #[test]
    fn prop_spur_thinned_thickness_less_than_theoretical(
        m  in pos_module(),
        z  in any_teeth(),
        jt in 1e-6f64..=1.0f64,
    ) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!(g.thinned_tooth_thickness(jt).unwrap() < g.tooth_thickness());
    }

    #[test]
    fn prop_spur_normal_backlash_less_than_circular(
        m  in pos_module(),
        z  in any_teeth(),
        jt in 1e-6f64..=1.0f64,
    ) {
        let g = Gear::builder().module(m).teeth(z).build().unwrap();
        prop_assert!(g.normal_backlash(jt).unwrap() < jt);
    }

    // --- Helical gear invariants ---

    #[test]
    fn prop_helical_transverse_module_formula(
        m   in 0.1f64..=50.0f64,
        z   in 3u32..=200u32,
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
        z   in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!(g.transverse_pressure_angle() > g.normal_pressure_angle());
    }

    #[test]
    fn prop_helical_tooth_profile_uses_normal_module(
        m   in 0.1f64..=50.0f64,
        z   in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!((g.addendum()    - m).abs()        < 1e-10);
        prop_assert!((g.dedendum()    - 1.25 * m).abs() < 1e-10);
        prop_assert!((g.tooth_depth() - 2.25 * m).abs() < 1e-10);
        prop_assert!((g.clearance()   - 0.25 * m).abs() < 1e-10);
        prop_assert!((g.tooth_thickness() - PI * m / 2.0).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_diameter_ordering(
        m   in 0.1f64..=50.0f64,
        z   in any_teeth(),
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!(g.root_diameter() < g.reference_diameter(), "root < pitch");
        prop_assert!(g.reference_diameter() < g.tip_diameter(), "pitch < tip");
    }

    #[test]
    fn prop_helical_base_diameter_formula(
        m   in 0.1f64..=50.0f64,
        z   in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let expected = g.reference_diameter() * g.transverse_pressure_angle().to_radians().cos();
        prop_assert!((g.base_diameter() - expected).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_tip_diameter_equals_reference_plus_two_addenda(
        m   in 0.1f64..=50.0f64,
        z   in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!((g.tip_diameter() - (g.reference_diameter() + 2.0 * g.addendum())).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_root_diameter_equals_reference_minus_two_dedenda(
        m   in 0.1f64..=50.0f64,
        z   in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!((g.root_diameter() - (g.reference_diameter() - 2.0 * g.dedendum())).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_transverse_circular_pitch_formula(
        m   in 0.1f64..=50.0f64,
        z   in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!((g.transverse_circular_pitch() - PI * g.transverse_module()).abs() < 1e-10);
        prop_assert!(g.transverse_circular_pitch() > g.normal_circular_pitch());
    }

    #[test]
    fn prop_helical_axial_pitch_formula(
        m   in 0.1f64..=50.0f64,
        z   in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        let expected = PI * m / psi.to_radians().sin();
        prop_assert!((g.axial_pitch() - expected).abs() < 1e-6);
    }

    #[test]
    fn prop_helical_lead_equals_axial_pitch_times_teeth(
        m   in 0.1f64..=50.0f64,
        z   in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!((g.lead() - g.axial_pitch() * z as f64).abs() < 1e-6);
    }

    #[test]
    fn prop_helical_center_distance_symmetric(
        m   in 0.1f64..=50.0f64,
        z1  in 3u32..=200u32,
        z2  in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g1 = helical_gear(m, z1, psi, HelixHand::Right);
        let g2 = helical_gear(m, z2, psi, HelixHand::Left);
        prop_assert_eq!(g1.center_distance_to(&g2), g2.center_distance_to(&g1));
    }

    #[test]
    fn prop_helical_gear_ratio_reciprocal(
        m   in 0.1f64..=50.0f64,
        z1  in 3u32..=200u32,
        z2  in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g1 = helical_gear(m, z1, psi, HelixHand::Right);
        let g2 = helical_gear(m, z2, psi, HelixHand::Left);
        let product = g1.gear_ratio_to(&g2) * g2.gear_ratio_to(&g1);
        prop_assert!((product - 1.0).abs() < 1e-10);
    }

    // --- Helical overlap and total contact ratio ---

    #[test]
    fn prop_overlap_ratio_formula(
        b   in 1.0f64..=200.0f64,
        psi in 0.1f64..89.9f64,
        m   in 0.1f64..=50.0f64,
        z   in 3u32..=200u32,
    ) {
        let g = HelicalGear::builder()
            .module(m).teeth(z).helix_angle(psi).helix_hand(HelixHand::Right)
            .face_width(b).build().unwrap();
        let expected = b * psi.to_radians().sin() / (PI * m);
        prop_assert!((g.overlap_ratio().unwrap() - expected).abs() < 1e-10);
    }

    #[test]
    fn prop_total_contact_ratio_equals_transverse_plus_overlap(
        m   in 0.1f64..=50.0f64,
        z1  in 3u32..=200u32,
        z2  in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
        b   in 1.0f64..=200.0f64,
    ) {
        let g1 = HelicalGear::builder()
            .module(m).teeth(z1).helix_angle(psi).helix_hand(HelixHand::Right)
            .face_width(b).build().unwrap();
        let g2 = HelicalGear::builder()
            .module(m).teeth(z2).helix_angle(psi).helix_hand(HelixHand::Left)
            .face_width(b).build().unwrap();
        let ea = g1.transverse_contact_ratio_with(&g2);
        let eb = g1.overlap_ratio().unwrap();
        let eg = g1.total_contact_ratio_with(&g2).unwrap();
        prop_assert!((eg - (ea + eb)).abs() < 1e-10);
    }

    #[test]
    fn prop_helical_transverse_contact_ratio_symmetric(
        m   in 0.1f64..=50.0f64,
        z1  in 3u32..=200u32,
        z2  in 3u32..=200u32,
        psi in 0.1f64..89.9f64,
    ) {
        let g1 = helical_gear(m, z1, psi, HelixHand::Right);
        let g2 = helical_gear(m, z2, psi, HelixHand::Left);
        prop_assert_eq!(
            g1.transverse_contact_ratio_with(&g2),
            g2.transverse_contact_ratio_with(&g1)
        );
    }

    #[test]
    fn prop_helical_normal_backlash_less_than_spur_normal_backlash(
        m   in pos_module(),
        z   in any_teeth(),
        psi in 0.1f64..89.9f64,
        jt  in 1e-6f64..=1.0f64,
    ) {
        let g_spur = Gear::builder().module(m).teeth(z).build().unwrap();
        let g_helical = helical_gear(m, z, psi, HelixHand::Right);
        prop_assert!(g_helical.normal_backlash(jt).unwrap() < g_spur.normal_backlash(jt).unwrap());
    }

    // --- Scene: ratio propagation ---

    #[test]
    fn prop_scene_two_gear_ratio(
        m  in 0.1f64..=50.0f64,
        z1 in 3u32..=200u32,
        z2 in 3u32..=200u32,
    ) {
        let ga = AnyGear::from(Gear::builder().module(m).teeth(z1).build().unwrap());
        let gb = AnyGear::from(Gear::builder().module(m).teeth(z2).build().unwrap());
        let scene = GearScene::builder()
            .shaft("driver", vec![("a", ga)])
            .shaft("driven", vec![("b", gb)])
            .mesh("a", "b")
            .driver("driver")
            .build()
            .unwrap();
        let sim = scene.run(100.0).unwrap();
        let expected = z2 as f64 / z1 as f64;
        prop_assert!((sim.ratio_to("driven").unwrap() - expected).abs() < 1e-9);
    }

    #[test]
    fn prop_scene_rpm_scales_linearly(
        m   in 0.1f64..=50.0f64,
        z1  in 3u32..=200u32,
        z2  in 3u32..=200u32,
        rpm in 1.0f64..=10000.0f64,
    ) {
        let ga = AnyGear::from(Gear::builder().module(m).teeth(z1).build().unwrap());
        let gb = AnyGear::from(Gear::builder().module(m).teeth(z2).build().unwrap());
        let scene = GearScene::builder()
            .shaft("driver", vec![("a", ga)])
            .shaft("driven", vec![("b", gb)])
            .mesh("a", "b")
            .driver("driver")
            .build()
            .unwrap();
        let sim1 = scene.run(rpm).unwrap();
        let sim2 = scene.run(rpm * 2.0).unwrap();
        prop_assert!((sim2.rpm("driven").unwrap() - 2.0 * sim1.rpm("driven").unwrap()).abs() < 1e-6);
    }
}
