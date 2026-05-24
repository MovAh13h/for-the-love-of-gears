use for_the_love_of_gears::{
    gear::Gear,
    helical::{HelicalGear, HelixHand},
    module::Module,
    scene::{AnyGear, Direction, GearScene, GearSceneError},
};

fn spur(module: f64, teeth: u32) -> AnyGear {
    AnyGear::from(
        Gear::builder()
            .module(Module::Specified(module))
            .teeth(teeth)
            .build()
            .unwrap(),
    )
}

fn helical(module: f64, teeth: u32, helix_deg: f64, hand: HelixHand) -> AnyGear {
    AnyGear::from(
        HelicalGear::builder()
            .module(Module::Specified(module))
            .teeth(teeth)
            .helix_angle(helix_deg)
            .helix_hand(hand)
            .build()
            .unwrap(),
    )
}

// ── Basic two-gear pair ───────────────────────────────────────────────────────

#[test]
fn two_gear_pair_rpm() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1000.0, 10.0).unwrap();
    assert!((sim.rpm("input") - 1000.0).abs() < 1e-9);
    assert!((sim.rpm("output") - 500.0).abs() < 1e-9);
}

#[test]
fn two_gear_pair_ratio() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1000.0, 10.0).unwrap();
    assert!((sim.ratio_to("output") - 2.0).abs() < 1e-9);
    assert!((sim.ratio_to("input") - 1.0).abs() < 1e-9);
}

#[test]
fn two_gear_pair_directions() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1000.0, 10.0).unwrap();
    assert_eq!(sim.direction("input"), Direction::Clockwise);
    assert_eq!(sim.direction("output"), Direction::CounterClockwise);
}

// ── Speed-up pair (fewer teeth on driven) ────────────────────────────────────

#[test]
fn speed_up_pair() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 40))])
        .shaft("output", vec![("b", spur(2.0, 20))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(500.0, 1.0).unwrap();
    assert!((sim.rpm("output") - 1000.0).abs() < 1e-9);
    assert!((sim.ratio_to("output") - 0.5).abs() < 1e-9);
}

// ── Compound gear train (three shafts, 6:1 total) ────────────────────────────

#[test]
fn compound_train_rpm() {
    // Stage 1: 20t → 40t (2:1). Stage 2: 20t → 60t (3:1). Total: 6:1.
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft(
            "intermediate",
            vec![("b", spur(2.0, 40)), ("c", spur(3.0, 20))],
        )
        .shaft("output", vec![("d", spur(3.0, 60))])
        .mesh("a", "b")
        .mesh("c", "d")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1200.0, 60.0).unwrap();
    assert!((sim.rpm("output") - 200.0).abs() < 1e-9);
    assert!((sim.ratio_to("output") - 6.0).abs() < 1e-9);
}

#[test]
fn compound_train_directions() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft(
            "intermediate",
            vec![("b", spur(2.0, 40)), ("c", spur(3.0, 20))],
        )
        .shaft("output", vec![("d", spur(3.0, 60))])
        .mesh("a", "b")
        .mesh("c", "d")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1200.0, 60.0).unwrap();
    // input → intermediate: direction flips (CW → CCW)
    // intermediate → output: direction flips again (CCW → CW)
    assert_eq!(sim.direction("input"), Direction::Clockwise);
    assert_eq!(sim.direction("intermediate"), Direction::CounterClockwise);
    assert_eq!(sim.direction("output"), Direction::Clockwise);
}

// ── Kinematics ────────────────────────────────────────────────────────────────

#[test]
fn total_rotations() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(60.0, 60.0).unwrap(); // 60 rpm for 60 s = 60 rotations
    assert!((sim.total_rotations("input") - 60.0).abs() < 1e-9);
    assert!((sim.total_rotations("output") - 30.0).abs() < 1e-9);
}

#[test]
fn angle_at_t_zero_is_zero() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1000.0, 10.0).unwrap();
    assert!((sim.angle_deg("input", 0.0)).abs() < 1e-9);
    assert!((sim.angle_deg("output", 0.0)).abs() < 1e-9);
}

#[test]
fn angle_wraps_at_360() {
    // 60 rpm → 1 revolution per second → 360°/s
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(60.0, 100.0).unwrap();
    // At t = 1s the input has made exactly one full revolution → 0°
    assert!(sim.angle_deg("input", 1.0).abs() < 1e-9);
    // At t = 0.5s input is at 180°
    assert!((sim.angle_deg("input", 0.5) - 180.0).abs() < 1e-9);
}

// ── Animation frames ──────────────────────────────────────────────────────────

#[test]
fn frames_count() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1000.0, 1.0).unwrap(); // 1 second
    let frames = sim.frames(24.0); // 24 fps → 24 intervals → 25 frames
    assert_eq!(frames.len(), 25);
}

#[test]
fn frames_first_time_is_zero() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1000.0, 2.0).unwrap();
    let frames = sim.frames(10.0);
    assert!(frames[0].time_secs.abs() < 1e-12);
}

#[test]
fn frames_each_has_all_shafts() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1000.0, 1.0).unwrap();
    for frame in sim.frames(10.0) {
        assert!(frame.shaft_angles.contains_key("input"));
        assert!(frame.shaft_angles.contains_key("output"));
    }
}

// ── GearScene metadata ────────────────────────────────────────────────────────

#[test]
fn shaft_names_sorted() {
    let scene = GearScene::builder()
        .shaft("z_shaft", vec![("za", spur(2.0, 20))])
        .shaft("a_shaft", vec![("aa", spur(2.0, 40))])
        .mesh("za", "aa")
        .driver("z_shaft")
        .build()
        .unwrap();

    assert_eq!(scene.shaft_names(), vec!["a_shaft", "z_shaft"]);
}

#[test]
fn driver_and_duration_accessors() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(300.0, 45.0).unwrap();
    assert_eq!(sim.driver_shaft(), "input");
    assert!((sim.driver_rpm() - 300.0).abs() < 1e-9);
    assert!((sim.duration_secs() - 45.0).abs() < 1e-9);
}

// ── Helical gear mesh ─────────────────────────────────────────────────────────

#[test]
fn helical_pair_rpm() {
    let scene = GearScene::builder()
        .shaft(
            "input",
            vec![("a", helical(2.0, 20, 15.0, HelixHand::Right))],
        )
        .shaft(
            "output",
            vec![("b", helical(2.0, 40, 15.0, HelixHand::Left))],
        )
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1000.0, 5.0).unwrap();
    assert!((sim.rpm("output") - 500.0).abs() < 1e-9);
}

// ── Error cases ───────────────────────────────────────────────────────────────

#[test]
fn error_no_driver() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .build();

    assert_eq!(result.unwrap_err(), GearSceneError::DriverShaftRequired);
}

#[test]
fn error_driver_not_found() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("nonexistent")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::DriverShaftNotFound("nonexistent".to_string())
    );
}

#[test]
fn error_duplicate_shaft_name() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("input", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::DuplicateShaftName("input".to_string())
    );
}

#[test]
fn error_duplicate_gear_name() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("a", spur(2.0, 40))]) // "a" used twice
        .mesh("a", "a")
        .driver("input")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::DuplicateGearName("a".to_string())
    );
}

#[test]
fn error_gear_not_found_in_mesh() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "ghost") // "ghost" does not exist
        .driver("input")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::GearNotFound("ghost".to_string())
    );
}

#[test]
fn error_mesh_gears_on_same_shaft() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20)), ("b", spur(2.0, 40))])
        .driver("input")
        .mesh("a", "b") // both on the same shaft
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::MeshGearsOnSameShaft {
            gear_a: "a".to_string(),
            gear_b: "b".to_string(),
        }
    );
}

#[test]
fn error_incompatible_mesh_different_module() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(3.0, 40))]) // module mismatch
        .mesh("a", "b")
        .driver("input")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::IncompatibleMesh {
            gear_a: "a".to_string(),
            gear_b: "b".to_string(),
        }
    );
}

#[test]
fn error_incompatible_mesh_spur_with_helical() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft(
            "output",
            vec![("b", helical(2.0, 40, 15.0, HelixHand::Left))],
        )
        .mesh("a", "b")
        .driver("input")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::IncompatibleMesh {
            gear_a: "a".to_string(),
            gear_b: "b".to_string(),
        }
    );
}

#[test]
fn error_disconnected_shaft() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .shaft("island", vec![("c", spur(2.0, 20))]) // no mesh connecting it
        .mesh("a", "b")
        .driver("input")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::DisconnectedShaft("island".to_string())
    );
}

#[test]
fn error_driver_rpm_not_positive() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    assert_eq!(
        scene.run(0.0, 10.0).unwrap_err(),
        GearSceneError::DriverRpmMustBePositive
    );
    assert_eq!(
        scene.run(-100.0, 10.0).unwrap_err(),
        GearSceneError::DriverRpmMustBePositive
    );
}

#[test]
fn error_duration_not_positive() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    assert_eq!(
        scene.run(1000.0, 0.0).unwrap_err(),
        GearSceneError::DurationMustBePositive
    );
    assert_eq!(
        scene.run(1000.0, -5.0).unwrap_err(),
        GearSceneError::DurationMustBePositive
    );
}

// ── Same scene, multiple run speeds ──────────────────────────────────────────

#[test]
fn scene_reuse_at_different_speeds() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim1 = scene.run(1000.0, 1.0).unwrap();
    let sim2 = scene.run(500.0, 1.0).unwrap();

    assert!((sim1.rpm("output") - 500.0).abs() < 1e-9);
    assert!((sim2.rpm("output") - 250.0).abs() < 1e-9);
    // Ratio is independent of speed
    assert!((sim1.ratio_to("output") - sim2.ratio_to("output")).abs() < 1e-9);
}

// ── Over-constrained shaft ────────────────────────────────────────────────────

#[test]
fn error_over_constrained_shaft() {
    // Shafts: A (driver), B, C
    // A→B via gears 20t/40t  → B = A/2
    // B→C via gears 20t/30t  → C = B × (20/30) = A/3
    // A→C via gears 20t/30t  → C = A × (20/30) = 2A/3
    // A/3 ≠ 2A/3 → OverConstrainedShaft on "c"
    let result = GearScene::builder()
        .shaft("a", vec![("a1", spur(2.0, 20)), ("a2", spur(3.0, 20))])
        .shaft("b", vec![("b1", spur(2.0, 40)), ("b2", spur(3.0, 20))])
        .shaft("c", vec![("c1", spur(3.0, 30)), ("c2", spur(3.0, 30))])
        .mesh("a1", "b1") // A→B: ratio 20/40 = 0.5
        .mesh("b2", "c1") // B→C: ratio 20/30 ≈ 0.667, so C_via_B = 0.5 × 0.667 = 0.333 × A
        .mesh("a2", "c2") // A→C direct: ratio 20/30 ≈ 0.667 × A — contradicts 0.333
        .driver("a")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::OverConstrainedShaft("c".to_string())
    );
}

// ── Extended helical scene tests ──────────────────────────────────────────────

#[test]
fn helical_pair_directions() {
    let scene = GearScene::builder()
        .shaft(
            "input",
            vec![("a", helical(2.0, 20, 15.0, HelixHand::Right))],
        )
        .shaft(
            "output",
            vec![("b", helical(2.0, 40, 15.0, HelixHand::Left))],
        )
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(600.0, 5.0).unwrap();
    assert_eq!(sim.direction("input"), Direction::Clockwise);
    assert_eq!(sim.direction("output"), Direction::CounterClockwise);
}

#[test]
fn helical_pair_ratio() {
    let scene = GearScene::builder()
        .shaft(
            "input",
            vec![("a", helical(2.0, 20, 15.0, HelixHand::Right))],
        )
        .shaft(
            "output",
            vec![("b", helical(2.0, 40, 15.0, HelixHand::Left))],
        )
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1000.0, 5.0).unwrap();
    assert!((sim.ratio_to("output") - 2.0).abs() < 1e-9);
}

#[test]
fn helical_compound_train() {
    // Two-stage helical reduction: 20t→40t (2:1) then 15t→45t (3:1) = 6:1
    let scene = GearScene::builder()
        .shaft(
            "input",
            vec![("a", helical(2.0, 20, 15.0, HelixHand::Right))],
        )
        .shaft(
            "intermediate",
            vec![
                ("b", helical(2.0, 40, 15.0, HelixHand::Left)),
                ("c", helical(3.0, 15, 20.0, HelixHand::Right)),
            ],
        )
        .shaft(
            "output",
            vec![("d", helical(3.0, 45, 20.0, HelixHand::Left))],
        )
        .mesh("a", "b")
        .mesh("c", "d")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1200.0, 10.0).unwrap();
    assert!((sim.rpm("output") - 200.0).abs() < 1e-9);
    assert!((sim.ratio_to("output") - 6.0).abs() < 1e-9);
}

// ── AnyGear accessors ─────────────────────────────────────────────────────────

#[test]
fn anygear_teeth_and_module() {
    let g = spur(3.0, 24);
    assert_eq!(g.teeth(), 24);
    assert!((g.module() - 3.0).abs() < 1e-9);
}

#[test]
fn anygear_face_width_none_by_default() {
    let g = spur(2.0, 20);
    assert!(g.face_width().is_none());
}

#[test]
fn anygear_helical_module_is_normal_module() {
    let g = helical(2.0, 20, 15.0, HelixHand::Right);
    assert!((g.module() - 2.0).abs() < 1e-9); // normal module, not transverse
    assert_eq!(g.teeth(), 20);
}
