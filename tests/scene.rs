use for_the_love_of_gears::{
    gear::Gear,
    helical::{HelicalGear, HelixHand},
    scene::{AnyGear, Direction, GearScene, GearSceneError},
    traits::GearGeometry,
};

fn spur(module: f64, teeth: u32) -> AnyGear {
    AnyGear::from(Gear::builder().module(module).teeth(teeth).build().unwrap())
}

fn helical(module: f64, teeth: u32, helix_deg: f64, hand: HelixHand) -> AnyGear {
    AnyGear::from(
        HelicalGear::builder()
            .module(module)
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

    let sim = scene.run(1000.0).unwrap();
    assert!((sim.rpm("input").unwrap() - 1000.0).abs() < 1e-9);
    assert!((sim.rpm("output").unwrap() - 500.0).abs() < 1e-9);
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

    let sim = scene.run(1000.0).unwrap();
    assert!((sim.ratio_to("output").unwrap() - 2.0).abs() < 1e-9);
    assert!((sim.ratio_to("input").unwrap() - 1.0).abs() < 1e-9);
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

    let sim = scene.run(1000.0).unwrap();
    assert_eq!(sim.direction("input"), Some(Direction::Clockwise));
    assert_eq!(sim.direction("output"), Some(Direction::CounterClockwise));
}

// ── Speed-up pair ─────────────────────────────────────────────────────────────

#[test]
fn speed_up_pair() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 40))])
        .shaft("output", vec![("b", spur(2.0, 20))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(500.0).unwrap();
    assert!((sim.rpm("output").unwrap() - 1000.0).abs() < 1e-9);
    assert!((sim.ratio_to("output").unwrap() - 0.5).abs() < 1e-9);
}

// ── Compound gear train ───────────────────────────────────────────────────────

#[test]
fn compound_train_rpm() {
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

    let sim = scene.run(1200.0).unwrap();
    assert!((sim.rpm("output").unwrap() - 200.0).abs() < 1e-9);
    assert!((sim.ratio_to("output").unwrap() - 6.0).abs() < 1e-9);
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

    let sim = scene.run(1200.0).unwrap();
    assert_eq!(sim.direction("input"), Some(Direction::Clockwise));
    assert_eq!(
        sim.direction("intermediate"),
        Some(Direction::CounterClockwise)
    );
    assert_eq!(sim.direction("output"), Some(Direction::Clockwise));
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

    // 60 rpm for 60 s = 60 rotations on input
    let sim = scene.run(60.0).unwrap();
    assert!((sim.total_rotations("input", 60.0).unwrap() - 60.0).abs() < 1e-9);
    assert!((sim.total_rotations("output", 60.0).unwrap() - 30.0).abs() < 1e-9);
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

    let sim = scene.run(1000.0).unwrap();
    assert!((sim.angle_deg("input", 0.0).unwrap()).abs() < 1e-9);
    assert!((sim.angle_deg("output", 0.0).unwrap()).abs() < 1e-9);
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

    let sim = scene.run(60.0).unwrap();
    assert!(sim.angle_deg("input", 1.0).unwrap().abs() < 1e-9);
    assert!((sim.angle_deg("input", 0.5).unwrap() - 180.0).abs() < 1e-9);
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

    let sim = scene.run(1000.0).unwrap();
    let frames = sim.frames(24.0, 1.0); // 24 fps, 1 second → 25 frames
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

    let sim = scene.run(1000.0).unwrap();
    let frames = sim.frames(10.0, 2.0);
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

    let sim = scene.run(1000.0).unwrap();
    let i_in = sim.shaft_index("input").unwrap();
    let i_out = sim.shaft_index("output").unwrap();
    for frame in sim.frames(10.0, 1.0) {
        assert!(frame.shaft_angles.get(i_in).is_some());
        assert!(frame.shaft_angles.get(i_out).is_some());
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
fn driver_and_rpm_accessors() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(300.0).unwrap();
    assert_eq!(sim.driver_shaft(), "input");
    assert!((sim.driver_rpm() - 300.0).abs() < 1e-9);
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

    let sim = scene.run(1000.0).unwrap();
    assert!((sim.rpm("output").unwrap() - 500.0).abs() < 1e-9);
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
        .mesh("a", "ghost")
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
        .mesh("a", "b")
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
        .shaft("output", vec![("b", spur(3.0, 40))])
        .mesh("a", "b")
        .driver("input")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::MeshParameterMismatch {
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
        GearSceneError::MeshTypeMismatch {
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
        .shaft("island", vec![("c", spur(2.0, 20))]) // no mesh
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
        scene.run(0.0).unwrap_err(),
        GearSceneError::DriverRpmMustBePositive
    );
    assert_eq!(
        scene.run(-100.0).unwrap_err(),
        GearSceneError::DriverRpmMustBePositive
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

    let sim1 = scene.run(1000.0).unwrap();
    let sim2 = scene.run(500.0).unwrap();

    assert!((sim1.rpm("output").unwrap() - 500.0).abs() < 1e-9);
    assert!((sim2.rpm("output").unwrap() - 250.0).abs() < 1e-9);
    assert!((sim1.ratio_to("output").unwrap() - sim2.ratio_to("output").unwrap()).abs() < 1e-9);
}

// ── Over-constrained shaft ────────────────────────────────────────────────────

#[test]
fn error_over_constrained_shaft() {
    let result = GearScene::builder()
        .shaft("a", vec![("a1", spur(2.0, 20)), ("a2", spur(3.0, 20))])
        .shaft("b", vec![("b1", spur(2.0, 40)), ("b2", spur(3.0, 20))])
        .shaft("c", vec![("c1", spur(3.0, 30)), ("c2", spur(3.0, 30))])
        .mesh("a1", "b1")
        .mesh("b2", "c1")
        .mesh("a2", "c2")
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

    let sim = scene.run(600.0).unwrap();
    assert_eq!(sim.direction("input"), Some(Direction::Clockwise));
    assert_eq!(sim.direction("output"), Some(Direction::CounterClockwise));
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

    let sim = scene.run(1000.0).unwrap();
    assert!((sim.ratio_to("output").unwrap() - 2.0).abs() < 1e-9);
}

#[test]
fn helical_compound_train() {
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

    let sim = scene.run(1200.0).unwrap();
    assert!((sim.rpm("output").unwrap() - 200.0).abs() < 1e-9);
    assert!((sim.ratio_to("output").unwrap() - 6.0).abs() < 1e-9);
}

// ── Closed-loop direction paradox ────────────────────────────────────────────

#[test]
fn error_closed_loop_direction_paradox() {
    // Three equal spur gears in a triangle: A→B, A→C, C→B.
    // All ratios are 1:1, so the RPM check alone passes, but the direction
    // constraint is unsatisfiable — B is driven CCW by A yet must be CW from C.
    let result = GearScene::builder()
        .shaft("a", vec![("ag", spur(2.0, 20))])
        .shaft("b", vec![("bg", spur(2.0, 20))])
        .shaft("c", vec![("cg", spur(2.0, 20))])
        .mesh("ag", "bg")
        .mesh("ag", "cg")
        .mesh("cg", "bg")
        .driver("a")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::OverConstrainedShaft("c".to_string())
    );
}

// ── Duplicate mesh ────────────────────────────────────────────────────────────

#[test]
fn error_duplicate_mesh_same_order() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .mesh("a", "b")
        .driver("input")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::DuplicateMesh {
            gear_a: "a".to_string(),
            gear_b: "b".to_string(),
        }
    );
}

#[test]
fn error_duplicate_mesh_reversed_order() {
    let result = GearScene::builder()
        .shaft("input", vec![("a", spur(2.0, 20))])
        .shaft("output", vec![("b", spur(2.0, 40))])
        .mesh("a", "b")
        .mesh("b", "a")
        .driver("input")
        .build();

    assert_eq!(
        result.unwrap_err(),
        GearSceneError::DuplicateMesh {
            gear_a: "b".to_string(),
            gear_b: "a".to_string(),
        }
    );
}

// ── AnyGear accessors ─────────────────────────────────────────────────────────

#[test]
fn anygear_teeth_and_module() {
    let g = spur(3.0, 24);
    assert_eq!(g.teeth(), 24);
    assert!((g.normal_module() - 3.0).abs() < 1e-9);
}

#[test]
fn anygear_helical_module_is_normal_module() {
    let g = helical(2.0, 20, 15.0, HelixHand::Right);
    assert!((g.normal_module() - 2.0).abs() < 1e-9);
    assert_eq!(g.teeth(), 20);
}
