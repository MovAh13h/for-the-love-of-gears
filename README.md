# For the love of Gears!

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-dark.svg">
  <img alt="for-the-love-of-gears" src="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-light.svg" width="1280">
</picture>

[![CI](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml/badge.svg)](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml)
[![Crates.io](https://img.shields.io/crates/v/for_the_love_of_gears.svg)](https://crates.io/crates/for_the_love_of_gears)
[![Docs.rs](https://docs.rs/for_the_love_of_gears/badge.svg)](https://docs.rs/for_the_love_of_gears)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE)

ISO-standard gear geometry for Rust. Give a gear its module and tooth count; get back every dimension — diameters, tooth profile, pitch, contact ratio, and backlash — as typed millimetre values. Compose gears into a `GearScene` to simulate multi-stage gear trains.

**[Full API reference on docs.rs →](https://docs.rs/for_the_love_of_gears)**

## Installation

```toml
[dependencies]
for_the_love_of_gears = "0.1"
```

## What's included

- **Spur gears** — reference, tip, root, and base diameters; addendum, dedendum, tooth depth, clearance, tooth thickness, circular and diametral pitch
- **Helical gears** — normal and transverse module, transverse pressure angle, axial pitch, lead; all spur dimensions in both planes
- **Contact ratio** — transverse εα, overlap εβ (helical), total εγ
- **Backlash** — circular and normal backlash; thinned tooth thickness
- **Gear scenes** — mount gears on named shafts, declare meshes, run a kinematic simulation to query RPM, rotation direction, and angular position
- **Type-safe outputs** — every result is a distinct type (`ReferenceDiameter`, `ToothThickness`, …) so you can't accidentally mix up values

## Usage

```rust
use for_the_love_of_gears::{gear::Gear, module::Module, scene::{AnyGear, GearScene}};

// A 2:1 spur reduction
let pinion = Gear::builder().module(Module::Specified(2.0)).teeth(20).build()?;
let wheel  = Gear::builder().module(Module::Specified(2.0)).teeth(40).build()?;

println!("pitch diameter:  {} mm",  pinion.reference_diameter().value()); // 40 mm
println!("centre distance: {} mm",  pinion.center_distance_to(&wheel).value()); // 60 mm
println!("contact ratio:   {:.3}",  pinion.contact_ratio_with(&wheel).value()); // 1.635

// Simulate a two-stage 6:1 compound train
let scene = GearScene::builder()
    .shaft("input",        vec![("a", AnyGear::from(pinion))])
    .shaft("intermediate", vec![
        ("b", AnyGear::from(wheel)),
        ("c", AnyGear::from(Gear::builder().module(Module::Specified(3.0)).teeth(20).build()?)),
    ])
    .shaft("output", vec![("d", AnyGear::from(
        Gear::builder().module(Module::Specified(3.0)).teeth(60).build()?
    ))])
    .mesh("a", "b").mesh("c", "d")
    .driver("input")
    .build()?;

let sim = scene.run(1200.0, 10.0)?;
println!("output: {:.0} rpm", sim.rpm("output")); // 200 rpm
```

## License

[MIT](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE) — Tanishq Jain
