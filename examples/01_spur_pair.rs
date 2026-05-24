//! Easy — two spur gears meshing together.
//!
//! Run with: `cargo run --example 01_spur_pair`

use for_the_love_of_gears::{gear::Gear, module::Module};

fn main() {
    let driver = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(20)
        .face_width(20.0)
        .build()
        .unwrap();

    let driven = Gear::builder()
        .module(Module::Specified(2.0))
        .teeth(40)
        .face_width(20.0)
        .build()
        .unwrap();

    println!("DRIVER  m={} z={}", driver.module().value(), driver.teeth());
    println!("DRIVEN  m={} z={}", driven.module().value(), driven.teeth());
    println!();

    let w = 28; // label column width
    println!("  {:<w$}  {:>8}  {:>8}", "", "driver", "driven");
    println!("  {}", "─".repeat(w + 20));
    row(
        "Reference diameter",
        driver.reference_diameter().value(),
        driven.reference_diameter().value(),
        "mm    d = mz",
        w,
    );
    row(
        "Tip diameter",
        driver.tip_diameter().value(),
        driven.tip_diameter().value(),
        "mm    da = m(z+2)",
        w,
    );
    row(
        "Root diameter",
        driver.root_diameter().value(),
        driven.root_diameter().value(),
        "mm    df = m(z−2.5)",
        w,
    );
    row(
        "Base diameter",
        driver.base_diameter().value(),
        driven.base_diameter().value(),
        "mm    db = d·cos α",
        w,
    );
    println!();
    row(
        "Addendum",
        driver.addendum().value(),
        driven.addendum().value(),
        "mm    ha = m",
        w,
    );
    row(
        "Dedendum",
        driver.dedendum().value(),
        driven.dedendum().value(),
        "mm    hf = 1.25m",
        w,
    );
    row(
        "Tooth depth",
        driver.tooth_depth().value(),
        driven.tooth_depth().value(),
        "mm    h = 2.25m",
        w,
    );
    row(
        "Clearance",
        driver.clearance().value(),
        driven.clearance().value(),
        "mm    c = 0.25m",
        w,
    );
    row(
        "Tooth thickness",
        driver.tooth_thickness().value(),
        driven.tooth_thickness().value(),
        "mm    s = πm/2",
        w,
    );
    println!();
    row(
        "Circular pitch",
        driver.circular_pitch().value(),
        driven.circular_pitch().value(),
        "mm    p = πm",
        w,
    );
    row(
        "Diametral pitch",
        driver.diametral_pitch().value(),
        driven.diametral_pitch().value(),
        "t/in  DP = 25.4/m",
        w,
    );

    println!();
    println!("PAIR");
    println!(
        "  Center distance   {:>8.3} mm",
        driver.center_distance_to(&driven).value()
    );
    println!(
        "  Gear ratio        {:>8.3}:1",
        driver.gear_ratio_to(&driven)
    );
    println!(
        "  Contact ratio εα  {:>8.3}",
        driver.contact_ratio_with(&driven).value()
    );
}

fn row(label: &str, a: f64, b: f64, note: &str, w: usize) {
    println!("  {:<w$}  {:>8.3}  {:>8.3}  {}", label, a, b, note);
}
