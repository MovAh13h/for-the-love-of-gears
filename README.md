# For the love of Gears!

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-dark.svg">
  <img alt="for-the-love-of-gears" src="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-light.svg" width="1280">
</picture>

[![CI](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml/badge.svg)](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml)
[![Crates.io](https://img.shields.io/crates/v/for_the_love_of_gears.svg)](https://crates.io/crates/for_the_love_of_gears)
[![Docs.rs](https://docs.rs/for_the_love_of_gears/badge.svg)](https://docs.rs/for_the_love_of_gears)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE)

ISO-standard spur and helical gear geometry in Rust. Define a gear from its module and tooth count, then read off every dimension — diameters, tooth profile, pitch, contact ratio, and backlash — as typed values in millimetres. Compose gears into a `GearScene` to simulate multi-stage gear trains and query per-shaft RPM, direction, and angular position.

---

## Features

- **Spur gears** — full ISO geometry: 4 diameters, tooth profile, pitch, contact ratio, backlash
- **Helical gears** — normal/transverse module, helix geometry, axial pitch, lead, overlap ratio
- **Gear scenes** — mount gears on shafts, declare meshes, run kinematic simulation
- **Backlash** — circular and normal backlash, thinned tooth thickness
- **Contact ratio** — transverse εα, overlap εβ, total εγ
- **Typed values** — every output is a newtype (`ReferenceDiameter`, `ToothThickness`, …), not a bare `f64`
- **Builder pattern** — validated construction with clear errors for missing or invalid inputs
- **245 tests** — unit, integration, and property-based (proptest)

---

## Installation

```toml
[dependencies]
for_the_love_of_gears = "0.1"
```

---

## Quick start

### Spur gear

```rust
use for_the_love_of_gears::{gear::Gear, module::Module};

let gear = Gear::builder()
    .module(Module::Specified(2.0))
    .teeth(20)
    .build()?;

assert_eq!(gear.reference_diameter().value(), 40.0); // d  = mz
assert_eq!(gear.tip_diameter().value(),       44.0); // da = m(z + 2)
assert_eq!(gear.root_diameter().value(),      35.0); // df = m(z − 2.5)
assert_eq!(gear.addendum().value(),    2.0);          // ha = m
assert_eq!(gear.dedendum().value(),    2.5);          // hf = 1.25m
assert_eq!(gear.tooth_depth().value(), 4.5);          // h  = 2.25m

let pinion = Gear::builder().module(Module::Specified(2.0)).teeth(20).build()?;
let wheel  = Gear::builder().module(Module::Specified(2.0)).teeth(40).build()?;

assert!(pinion.can_mesh_with(&wheel));
assert_eq!(pinion.center_distance_to(&wheel).value(), 60.0);
assert_eq!(pinion.gear_ratio_to(&wheel), 2.0);
```

### Helical gear

```rust
use for_the_love_of_gears::{helical::{HelicalGear, HelixHand}, module::Module};

// Opposite hands required for parallel-shaft meshing
let driver = HelicalGear::builder()
    .module(Module::Specified(2.0))
    .teeth(20)
    .helix_angle(20.0)
    .helix_hand(HelixHand::Right)
    .face_width(30.0)
    .build()?;

let driven = HelicalGear::builder()
    .module(Module::Specified(2.0))
    .teeth(40)
    .helix_angle(20.0)
    .helix_hand(HelixHand::Left)
    .face_width(30.0)
    .build()?;

let ea = driver.transverse_contact_ratio_with(&driven);
let eg = driver.total_contact_ratio_with(&driven).unwrap(); // εα + εβ
assert!(eg.value() > ea.value());
```

### Gear scene (multi-stage simulation)

```rust
use for_the_love_of_gears::{gear::Gear, module::Module, scene::{AnyGear, GearScene}};

// Two-stage spur reduction: 20t→40t (2:1) then 20t→60t (3:1) = 6:1 total
let scene = GearScene::builder()
    .shaft("input",        vec![("a", AnyGear::from(Gear::builder().module(Module::Specified(2.0)).teeth(20).build()?))])
    .shaft("intermediate", vec![
        ("b", AnyGear::from(Gear::builder().module(Module::Specified(2.0)).teeth(40).build()?)),
        ("c", AnyGear::from(Gear::builder().module(Module::Specified(3.0)).teeth(20).build()?)),
    ])
    .shaft("output",       vec![("d", AnyGear::from(Gear::builder().module(Module::Specified(3.0)).teeth(60).build()?))])
    .mesh("a", "b")
    .mesh("c", "d")
    .driver("input")
    .build()?;

let sim = scene.run(1200.0, 10.0)?; // 1200 rpm driver, 10 seconds

assert!((sim.rpm("output")       - 200.0).abs() < 1e-9); // 1200 / 6
assert!((sim.ratio_to("output")  - 6.0).abs()   < 1e-9);
assert!((sim.total_rotations("output") - 200.0 * 10.0 / 60.0).abs() < 1e-9);
```

---

## License

[MIT](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE) — Tanishq Jain
