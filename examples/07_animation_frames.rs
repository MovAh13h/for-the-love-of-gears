//! Animation frame deep-dive — periods, angular velocities, and sync analysis.
//!
//! Two-stage spur reduction, 6:1 total:
//!   input       (20t m=2)
//!   → intermediate (60t m=2, 20t m=3)   first stage 3:1
//!   → output    (40t m=3)                second stage 2:1
//!
//! Driver: 180 rpm (one revolution every 0.333 s).
//!   input:        180 rpm  — period  0.333 s
//!   intermediate:  60 rpm  — period  1.000 s
//!   output:        30 rpm  — period  2.000 s
//!
//! At 24 fps the output shaft completes exactly one revolution every 48 frames.
//!
//! Run with: `cargo run --example 07_animation_frames`

use std::f64::consts::TAU;

use for_the_love_of_gears::{
    gear::Gear,
    scene::{AnyGear, Direction, GearScene},
};

fn main() {
    let scene = GearScene::builder()
        .shaft("input", vec![("a", g(2.0, 20))])
        .shaft("intermediate", vec![("b", g(2.0, 60)), ("c", g(3.0, 20))])
        .shaft("output", vec![("d", g(3.0, 40))])
        .mesh("a", "b")
        .mesh("c", "d")
        .driver("input")
        .build()
        .unwrap();

    let driver_rpm = 180.0; // rpm
    let fps = 24.0; // frames per second
    let duration = 2.0; // seconds — one full output revolution

    let sim = scene.run(driver_rpm).unwrap();

    // ── Shaft summary ────────────────────────────────────────────────────────
    println!("6:1 SPUR REDUCTION — SHAFT SUMMARY");
    println!();
    println!(
        "  {:<14}  {:>8}  {:>5}  {:>10}  {:>10}  {:>10}",
        "shaft", "rpm", "dir", "period (s)", "ω (rad/s)", "ω (°/s)"
    );
    println!("  {}", "─".repeat(66));

    for shaft in scene.shaft_names() {
        let rpm = sim.rpm(shaft).unwrap();
        let dir = fmt_dir(sim.direction(shaft).unwrap());
        let period = 60.0 / rpm; // seconds per revolution
        let omega = rpm * TAU / 60.0; // rad/s
        let omega_deg = rpm * 360.0 / 60.0; // degrees/s

        println!(
            "  {:<14}  {:>8.3}  {:>5}  {:>10.4}  {:>10.4}  {:>10.3}",
            shaft, rpm, dir, period, omega, omega_deg
        );
    }

    println!();
    println!("  Overall ratio: {:.0}:1", sim.ratio_to("output").unwrap());

    // ── Frame-count-to-revolution synchronisation ────────────────────────────
    println!();
    println!("SYNCHRONISATION  (at {fps:.0} fps)");
    for shaft in scene.shaft_names() {
        let rpm = sim.rpm(shaft).unwrap();
        let period = 60.0 / rpm;
        let frames_per_rev = period * fps;
        println!(
            "  {:<14}  one revolution every {:>6.2} frames  ({period:.4} s)",
            shaft, frames_per_rev
        );
    }

    // ── Animation frames table ───────────────────────────────────────────────
    println!();
    println!(
        "ANIMATION FRAMES  ({fps:.0} fps, {duration:.1} s total — {} frames)",
        (duration * fps) as usize + 1
    );
    println!(
        "  {:>5}  {:<7}  {:>10}  {:>16}  {:>10}",
        "frame", "t (s)", "input°", "intermediate°", "output°"
    );
    println!("  {}", "─".repeat(56));

    let i_in = sim.shaft_index("input").unwrap();
    let i_mid = sim.shaft_index("intermediate").unwrap();
    let i_out = sim.shaft_index("output").unwrap();

    let frames = sim.frames(fps, duration);
    for (i, frame) in frames.iter().enumerate() {
        println!(
            "  {:>5}  {:<7.4}  {:>9.1}°  {:>15.1}°  {:>9.1}°",
            i,
            frame.time_secs,
            frame.shaft_angles[i_in],
            frame.shaft_angles[i_mid],
            frame.shaft_angles[i_out],
        );
    }

    // ── Verify angle formula ─────────────────────────────────────────────────
    println!();
    println!("SPOT CHECKS  (angle = rpm/60 × t × 360°  mod 360°)");
    for &t in &[0.0_f64, 1.0 / fps, 0.5, 1.0, duration] {
        let input_angle = sim.angle_deg("input", t).unwrap();
        let out_angle = sim.angle_deg("output", t).unwrap();
        println!(
            "  t = {:>6.4} s  →  input {:>7.2}°   output {:>7.2}°",
            t, input_angle, out_angle
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
