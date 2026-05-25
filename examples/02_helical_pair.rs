//! Medium — helical gear pair with contact ratio and backlash.
//!
//! Run with: `cargo run --example 02_helical_pair`

use for_the_love_of_gears::helical::{HelicalGear, HelixHand};

fn main() {
    let pinion = HelicalGear::builder()
        .module(2.0)
        .teeth(20)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Right)
        .face_width(30.0)
        .build()
        .unwrap();

    let wheel = HelicalGear::builder()
        .module(2.0)
        .teeth(40)
        .helix_angle(20.0)
        .helix_hand(HelixHand::Left)
        .face_width(30.0)
        .build()
        .unwrap();

    println!(
        "PINION  mn={} z={} ψ={}° RH",
        pinion.normal_module(), pinion.teeth(), pinion.helix_angle()
    );
    println!(
        "WHEEL   mn={} z={} ψ={}° LH",
        wheel.normal_module(), wheel.teeth(), wheel.helix_angle()
    );
    println!();

    let w = 30;
    println!("  {:<w$}  {:>8}  {:>8}  note", "dimension", "pinion", "wheel");
    println!("  {}", "─".repeat(w + 30));
    hrow("Normal module mn",            pinion.normal_module(),            wheel.normal_module(),            "mm    input", w);
    hrow("Transverse module mt",        pinion.transverse_module(),        wheel.transverse_module(),        "mm    mn / cos ψ", w);
    hrow("Normal pressure angle αn",    pinion.normal_pressure_angle(),    wheel.normal_pressure_angle(),    "°     input", w);
    hrow("Transverse pressure angle αt",pinion.transverse_pressure_angle(),wheel.transverse_pressure_angle(),"°     derived", w);
    println!();
    hrow("Reference diameter", pinion.reference_diameter(), wheel.reference_diameter(), "mm    mt·z", w);
    hrow("Tip diameter",       pinion.tip_diameter(),       wheel.tip_diameter(),       "mm", w);
    hrow("Root diameter",      pinion.root_diameter(),      wheel.root_diameter(),      "mm", w);
    println!();
    hrow("Axial pitch px", pinion.axial_pitch(), wheel.axial_pitch(), "mm    πmn / sin ψ", w);
    hrow("Lead",           pinion.lead(),        wheel.lead(),        "mm    px · z", w);

    println!();
    println!("PAIR");
    println!("  Center distance       {:>8.3} mm", pinion.center_distance_to(&wheel));
    println!("  Gear ratio            {:>8.3}:1",  pinion.gear_ratio_to(&wheel));

    let ea = pinion.transverse_contact_ratio_with(&wheel);
    let eb = pinion.overlap_ratio().unwrap_or(0.0);
    let eg = pinion.total_contact_ratio_with(&wheel).unwrap_or(ea);
    println!();
    println!("  Contact ratio εα      {:>8.3}  transverse (profile overlap)", ea);
    println!("  Contact ratio εβ      {:>8.3}  overlap (helix length)", eb);
    println!("  Total εγ = εα + εβ    {:>8.3}", eg);

    println!();
    println!("BACKLASH  jt = 0.08 mm");
    let jt = 0.08;
    println!("  Theoretical thickness {:>8.4} mm  s = πmn/2", pinion.tooth_thickness());
    println!("  Thinned thickness     {:>8.4} mm  s' = s − (jt/2)·cos ψ", pinion.thinned_tooth_thickness(jt));
    println!("  Normal backlash jn    {:>8.4} mm  jt·cos αt·cos ψ", pinion.normal_backlash(jt));
}

fn hrow(label: &str, a: f64, b: f64, note: &str, w: usize) {
    println!("  {:<w$}  {:>8.4}  {:>8.4}  {}", label, a, b, note);
}
