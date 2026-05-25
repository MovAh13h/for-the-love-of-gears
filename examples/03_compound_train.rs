//! Advanced — compound gear train simulation with animation frames.
//!
//! Three-stage reduction, 60:1 total:
//!   A (input) → B: 20t/60t = 3:1
//!   B → C:         20t/80t = 4:1
//!   C → D (output):15t/75t = 5:1
//!
//! Run with: `cargo run --example 03_compound_train`

use for_the_love_of_gears::{
    gear::Gear,
    scene::{AnyGear, Direction, GearScene},
};

fn main() {
    let scene = GearScene::builder()
        .shaft("A_input",        vec![("a1", g(2.0, 20))])
        .shaft("B_intermediate", vec![("b1", g(2.0, 60)), ("b2", g(3.0, 20))])
        .shaft("C_intermediate", vec![("c1", g(3.0, 80)), ("c2", g(4.0, 15))])
        .shaft("D_output",       vec![("d1", g(4.0, 75))])
        .mesh("a1", "b1")
        .mesh("b2", "c1")
        .mesh("c2", "d1")
        .driver("A_input")
        .build()
        .unwrap();

    let duration = 2.0; // seconds
    let sim = scene.run(1500.0 /* rpm */).unwrap();

    println!("SHAFTS");
    println!("  {:<18}  {:>10}  {:>5}  {:>15}", "shaft", "rpm", "dir", "rotations (2 s)");
    println!("  {}", "─".repeat(56));
    for shaft in scene.shaft_names() {
        let dir = match sim.direction(shaft).unwrap() {
            Direction::Clockwise => "CW",
            Direction::CounterClockwise => "CCW",
        };
        println!(
            "  {:<18}  {:>10.3}  {:>5}  {:>15.3}",
            shaft,
            sim.rpm(shaft).unwrap(),
            dir,
            sim.total_rotations(shaft, duration).unwrap()
        );
    }
    println!();
    println!("  Overall ratio  {:.0}:1", sim.ratio_to("D_output").unwrap());

    println!();
    println!("FRAMES  (10 fps, first 0.5 s)");
    println!(
        "  {:<6}  {:>12}  {:>14}  {:>14}  {:>10}",
        "t", "A_input", "B_intermediate", "C_intermediate", "D_output"
    );
    println!("  {}", "─".repeat(62));
    for frame in sim.frames(10.0, duration).iter().filter(|f| f.time_secs <= 0.5) {
        println!(
            "  {:<6.2}  {:>11.1}°  {:>13.1}°  {:>13.1}°  {:>9.1}°",
            frame.time_secs,
            frame.shaft_angles["A_input"],
            frame.shaft_angles["B_intermediate"],
            frame.shaft_angles["C_intermediate"],
            frame.shaft_angles["D_output"],
        );
    }
}

fn g(module: f64, teeth: u32) -> AnyGear {
    AnyGear::from(Gear::builder().module(module).teeth(teeth).build().unwrap())
}
