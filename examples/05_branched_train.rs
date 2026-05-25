//! Branched gear train — one motor driving two independent output shafts.
//!
//! A single 3 600 rpm motor shaft carries two gears of different modules,
//! each independently driving a separate output shaft at a different ratio:
//!
//!   motor (30t, m=2) → spindle   (15t, m=2):  2:1 step-up  → 7 200 rpm
//!   motor (20t, m=3) → coolant   (80t, m=3):  4:1 reduction → 900 rpm
//!
//! Both output shafts are driven from the same motor shaft; neither output
//! drives the other.  Branched topologies are just ordinary scenes with more
//! than one gear on the driver shaft.
//!
//! Run with: `cargo run --example 05_branched_train`

use for_the_love_of_gears::{
    gear::Gear,
    scene::{AnyGear, Direction, GearScene},
};

fn main() {
    let scene = GearScene::builder()
        .shaft("motor", vec![
            ("m1", g(2.0, 30)),  // drives spindle
            ("m2", g(3.0, 20)),  // drives coolant pump
        ])
        .shaft("spindle",       vec![("s1", g(2.0, 15))])
        .shaft("coolant_pump",  vec![("p1", g(3.0, 80))])
        .mesh("m1", "s1")
        .mesh("m2", "p1")
        .driver("motor")
        .build()
        .unwrap();

    let driver_rpm = 3_600.0; // rpm
    let duration   = 1.0;     // seconds
    let sim = scene.run(driver_rpm).unwrap();

    println!("BRANCHED GEAR TRAIN");
    println!("  Motor: {driver_rpm:.0} rpm driving spindle (step-up 2:1) + coolant pump (reduction 4:1)");
    println!();
    println!(
        "  {:<14}  {:>10}  {:>5}  {:>14}  {:>14}  {:>10}",
        "shaft", "rpm", "dir", "rot (1 s)", "period (s)", "ω (rad/s)"
    );
    println!("  {}", "─".repeat(74));

    for shaft in scene.shaft_names() {
        let rpm    = sim.rpm(shaft).unwrap();
        let dir    = fmt_dir(sim.direction(shaft).unwrap());
        let rots   = sim.total_rotations(shaft, duration).unwrap();
        let period = 60.0 / rpm; // seconds per revolution
        let omega  = sim.angular_velocity_rad_s(shaft).unwrap();
        println!(
            "  {:<14}  {:>10.1}  {:>5}  {:>14.3}  {:>14.4}  {:>10.3}",
            shaft, rpm, dir, rots, period, omega
        );
    }

    println!();
    println!("  Spindle  ratio: {:.1}:1  (step-up)", sim.ratio_to("spindle").unwrap());
    println!("  Coolant  ratio: {:.1}:1  (reduction)", sim.ratio_to("coolant_pump").unwrap());

    println!();
    println!("ANIMATION FRAMES  (12 fps, first 0.25 s)");
    println!(
        "  {:<7}  {:>10}  {:>12}  {:>14}",
        "t (s)", "motor°", "spindle°", "coolant_pump°"
    );
    println!("  {}", "─".repeat(50));
    for frame in sim.frames(12.0, duration).iter().filter(|f| f.time_secs <= 0.25) {
        println!(
            "  {:<7.4}  {:>9.1}°  {:>11.1}°  {:>13.1}°",
            frame.time_secs,
            frame.shaft_angles["motor"],
            frame.shaft_angles["spindle"],
            frame.shaft_angles["coolant_pump"],
        );
    }
}

fn g(module: f64, teeth: u32) -> AnyGear {
    AnyGear::from(Gear::builder().module(module).teeth(teeth).build().unwrap())
}

fn fmt_dir(d: Direction) -> &'static str {
    match d {
        Direction::Clockwise => "CW",
        Direction::CounterClockwise => "CCW",
    }
}
