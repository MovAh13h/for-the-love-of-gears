//! Easy — two spur gears meshing together.
//!
//! Run with: `cargo run --example 01_spur_pair`

use for_the_love_of_gears::gear::Gear;

fn main() {
    let driver = Gear::builder().module(2.0).teeth(20).build().unwrap();
    let driven = Gear::builder().module(2.0).teeth(40).build().unwrap();

    println!("DRIVER  m={} z={}", driver.module(), driver.teeth());
    println!("DRIVEN  m={} z={}", driven.module(), driven.teeth());
    println!();

    let w = 28; // label column width
    println!("  {:<w$}  {:>8}  {:>8}", "", "driver", "driven");
    println!("  {}", "─".repeat(w + 20));
    row("Reference diameter", driver.reference_diameter(), driven.reference_diameter(), "mm    d = mz", w);
    row("Tip diameter",       driver.tip_diameter(),       driven.tip_diameter(),       "mm    da = m(z+2)", w);
    row("Root diameter",      driver.root_diameter(),      driven.root_diameter(),      "mm    df = m(z−2.5)", w);
    row("Base diameter",      driver.base_diameter(),      driven.base_diameter(),      "mm    db = d·cos α", w);
    println!();
    row("Addendum",       driver.addendum(),       driven.addendum(),       "mm    ha = m", w);
    row("Dedendum",       driver.dedendum(),       driven.dedendum(),       "mm    hf = 1.25m", w);
    row("Tooth depth",    driver.tooth_depth(),    driven.tooth_depth(),    "mm    h = 2.25m", w);
    row("Clearance",      driver.clearance(),      driven.clearance(),      "mm    c = 0.25m", w);
    row("Tooth thickness",driver.tooth_thickness(),driven.tooth_thickness(),"mm    s = πm/2", w);
    println!();
    row("Circular pitch", driver.circular_pitch(), driven.circular_pitch(), "mm    p = πm", w);
    row("Diametral pitch",driver.diametral_pitch(),driven.diametral_pitch(),"t/in  DP = 25.4/m", w);

    println!();
    println!("PAIR");
    println!("  Center distance   {:>8.3} mm",  driver.center_distance_to(&driven));
    println!("  Gear ratio        {:>8.3}:1",   driver.gear_ratio_to(&driven));
    println!("  Contact ratio εα  {:>8.3}",     driver.contact_ratio_with(&driven));
}

fn row(label: &str, a: f64, b: f64, note: &str, w: usize) {
    println!("  {:<w$}  {:>8.3}  {:>8.3}  {}", label, a, b, note);
}
