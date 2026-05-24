//! Advanced — helical gear train simulation.
//!
//! Two-stage helical reduction, 6:1 total:
//!   input → intermediate: 20t / 40t = 2:1   (m=2, ψ=15°)
//!   intermediate → output: 18t / 54t = 3:1   (m=3, ψ=20°)
//!
//! Opposing helix hands are required within each mesh.
//! Different modules are fine across stages.
//!
//! Run with: `cargo run --example 04_helical_scene`

use for_the_love_of_gears::{
    helical::{HelicalGear, HelixHand},
    module::Module,
    scene::{AnyGear, Direction, GearScene},
};

fn main() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", hg(2.0, 20, 15.0, HelixHand::Right))])
        .shaft(
            "intermediate",
            vec![
                ("b", hg(2.0, 40, 15.0, HelixHand::Left)), // driven by a (2:1)
                ("c", hg(3.0, 18, 20.0, HelixHand::Right)), // drives d (3:1)
            ],
        )
        .shaft("output", vec![("d", hg(3.0, 54, 20.0, HelixHand::Left))])
        .mesh("a", "b")
        .mesh("c", "d")
        .driver("input")
        .build()
        .unwrap();

    let sim = scene.run(1800.0, 5.0).unwrap();

    println!("HELICAL GEAR TRAIN  (2-stage, 6:1)");
    println!();
    println!(
        "  {:<14}  {:>10}  {:>5}  {:>12}  {:>12}",
        "shaft", "rpm", "dir", "rot (5 s)", "angle @1.15 s"
    );
    println!("  {}", "─".repeat(60));
    for shaft in scene.shaft_names() {
        let dir = match sim.direction(shaft) {
            Direction::Clockwise => "CW",
            Direction::CounterClockwise => "CCW",
        };
        println!(
            "  {:<14}  {:>10.3}  {:>5}  {:>12.3}  {:>11.1}°",
            shaft,
            sim.rpm(shaft),
            dir,
            sim.total_rotations(shaft),
            sim.angle_deg(shaft, 1.15),
        );
    }

    println!();
    println!("  Overall ratio  {:.0}:1", sim.ratio_to("output"));

    // Contact ratio for each mesh
    let a = helical(2.0, 20, 15.0, HelixHand::Right, 25.0);
    let b = helical(2.0, 40, 15.0, HelixHand::Left, 25.0);
    let c = helical(3.0, 18, 20.0, HelixHand::Right, 30.0);
    let d = helical(3.0, 54, 20.0, HelixHand::Left, 30.0);

    println!();
    println!("CONTACT RATIOS");
    println!("  {:<22}  {:>6}  {:>6}  {:>6}", "mesh", "εα", "εβ", "εγ");
    println!("  {}", "─".repeat(46));
    contact_row("input → intermediate", &a, &b);
    contact_row("intermediate → output", &c, &d);
}

fn contact_row(label: &str, g1: &HelicalGear, g2: &HelicalGear) {
    let ea = g1.transverse_contact_ratio_with(g2).value();
    let eb = g1.overlap_ratio().map(|r| r.value()).unwrap_or(0.0);
    let eg = g1
        .total_contact_ratio_with(g2)
        .map(|r| r.value())
        .unwrap_or(ea);
    println!("  {:<22}  {:>6.3}  {:>6.3}  {:>6.3}", label, ea, eb, eg);
}

fn hg(module: f64, teeth: u32, helix_deg: f64, hand: HelixHand) -> AnyGear {
    AnyGear::from(helical(module, teeth, helix_deg, hand, 0.0))
}

fn helical(
    module: f64,
    teeth: u32,
    helix_deg: f64,
    hand: HelixHand,
    face_width: f64,
) -> HelicalGear {
    let mut b = HelicalGear::builder()
        .module(Module::Specified(module))
        .teeth(teeth)
        .helix_angle(helix_deg)
        .helix_hand(hand);
    if face_width > 0.0 {
        b = b.face_width(face_width);
    }
    b.build().unwrap()
}
