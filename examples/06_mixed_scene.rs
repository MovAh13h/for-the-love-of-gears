//! Mixed spur + helical compound train — two gear types in one scene.
//!
//! Stage 1  (spur):    input  25t m=2 → intermediate 75t m=2  →  3:1 reduction
//! Stage 2  (helical): inter  20t m=2.5 ψ=20° RH → output 60t m=2.5 ψ=20° LH  →  3:1 reduction
//!
//! Total: 9:1 at 1 800 rpm input → 200 rpm output.
//!
//! The intermediate compound shaft carries one spur gear (driven by stage 1)
//! and one helical gear (driving stage 2).  The two gear types do not mesh with
//! each other — they are on the same shaft, so they rotate together.
//!
//! Run with: `cargo run --example 06_mixed_scene`

use for_the_love_of_gears::{
    gear::Gear,
    helical::{HelicalGear, HelixHand},
    scene::{AnyGear, Direction, GearScene},
};

fn main() {
    let scene = GearScene::builder()
        .shaft("input", vec![
            ("a", spur(2.0, 25)),
        ])
        .shaft("intermediate", vec![
            ("b", spur(2.0, 75)),                                        // stage 1 driven
            ("c", helical(2.5, 20, 20.0, HelixHand::Right)),             // stage 2 driver
        ])
        .shaft("output", vec![
            ("d", helical(2.5, 60, 20.0, HelixHand::Left)),
        ])
        .mesh("a", "b")   // spur stage
        .mesh("c", "d")   // helical stage
        .driver("input")
        .build()
        .unwrap();

    let driver_rpm = 1_800.0; // rpm
    let duration   = 3.0;     // seconds
    let sim = scene.run(driver_rpm).unwrap();

    println!("MIXED SPUR + HELICAL COMPOUND TRAIN  (9:1)");
    println!();
    println!(
        "  {:<14}  {:>10}  {:>5}  {:>14}  {:>10}",
        "shaft", "rpm", "dir", "rot (3 s)", "ω (rad/s)"
    );
    println!("  {}", "─".repeat(60));

    for shaft in scene.shaft_names() {
        let rpm   = sim.rpm(shaft).unwrap();
        let dir   = fmt_dir(sim.direction(shaft).unwrap());
        let rots  = sim.total_rotations(shaft, duration).unwrap();
        let omega = sim.angular_velocity_rad_s(shaft).unwrap();
        println!(
            "  {:<14}  {:>10.3}  {:>5}  {:>14.3}  {:>10.4}",
            shaft, rpm, dir, rots, omega
        );
    }

    println!();
    println!("  Stage 1 ratio (spur):    {:.1}:1", sim.ratio_to("intermediate").unwrap());
    println!("  Stage 2 ratio (helical): {:.1}:1", sim.ratio_to("output").unwrap() / sim.ratio_to("intermediate").unwrap());
    println!("  Overall ratio:           {:.1}:1", sim.ratio_to("output").unwrap());

    // Build gear objects for contact-ratio queries (need face_width for εβ).
    let ga = HelicalGear::builder().module(2.5).teeth(20).helix_angle(20.0).helix_hand(HelixHand::Right).face_width(30.0).build().unwrap();
    let gb = HelicalGear::builder().module(2.5).teeth(60).helix_angle(20.0).helix_hand(HelixHand::Left).face_width(30.0).build().unwrap();
    let spur_a = Gear::builder().module(2.0).teeth(25).build().unwrap();
    let spur_b = Gear::builder().module(2.0).teeth(75).build().unwrap();

    println!();
    println!("CONTACT RATIOS");
    println!("  {:<26}  {:>6}  {:>6}  {:>6}", "mesh", "εα", "εβ", "εγ");
    println!("  {}", "─".repeat(50));

    let ea_spur = spur_a.contact_ratio_with(&spur_b).unwrap();
    println!("  {:<26}  {:>6.3}  {:>6}  {:>6.3}", "input→intermediate (spur)", ea_spur, "—", ea_spur);

    let ea_hel = ga.transverse_contact_ratio_with(&gb).unwrap();
    let eb_hel = ga.overlap_ratio().unwrap_or(0.0);
    let eg_hel = ga.total_contact_ratio_with(&gb).unwrap_or(ea_hel);
    println!("  {:<26}  {:>6.3}  {:>6.3}  {:>6.3}", "intermediate→output (helical)", ea_hel, eb_hel, eg_hel);

    println!();
    let i_in  = sim.shaft_index("input").unwrap();
    let i_mid = sim.shaft_index("intermediate").unwrap();
    let i_out = sim.shaft_index("output").unwrap();

    println!("ANIMATION FRAMES  (10 fps, first 0.3 s)");
    println!(
        "  {:<7}  {:>10}  {:>16}  {:>10}",
        "t (s)", "input°", "intermediate°", "output°"
    );
    println!("  {}", "─".repeat(52));
    for frame in sim.frames(10.0, duration).iter().filter(|f| f.time_secs <= 0.3) {
        println!(
            "  {:<7.2}  {:>9.1}°  {:>15.1}°  {:>9.1}°",
            frame.time_secs,
            frame.shaft_angles[i_in],
            frame.shaft_angles[i_mid],
            frame.shaft_angles[i_out],
        );
    }
}

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

fn fmt_dir(d: Direction) -> &'static str {
    match d {
        Direction::Clockwise => "CW",
        Direction::CounterClockwise => "CCW",
    }
}
