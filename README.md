# For the love of Gears!

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-dark.svg">
  <img alt="for-the-love-of-gears" src="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-light.svg" width="1280">
</picture>

[![CI](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml/badge.svg)](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml)
[![Crates.io](https://img.shields.io/crates/v/for_the_love_of_gears.svg)](https://crates.io/crates/for_the_love_of_gears)
[![Docs.rs](https://docs.rs/for_the_love_of_gears/badge.svg)](https://docs.rs/for_the_love_of_gears)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE)

ISO-standard spur and helical gear geometry in Rust. Define a gear from its module and tooth count, then read off every dimension — diameters, tooth profile, pitch, and gear pair properties — as typed values in millimetres.

---

## Installation

```toml
[dependencies]
for_the_love_of_gears = "0.1"
```

---

## Quick start

```rust
use for_the_love_of_gears::{gear::Gear, module::Module};

// Define a gear — module and teeth are required
let gear = Gear::builder()
    .module(Module::Specified(2.0))
    .teeth(20)
    .build()?;

// Diameters (mm)
assert_eq!(gear.reference_diameter().value(), 40.0); // d  = mz
assert_eq!(gear.tip_diameter().value(),       44.0); // da = m(z + 2)
assert_eq!(gear.root_diameter().value(),      35.0); // df = m(z − 2.5)

// Tooth profile (mm)
assert_eq!(gear.addendum().value(),    2.0); // ha = m
assert_eq!(gear.dedendum().value(),    2.5); // hf = 1.25m
assert_eq!(gear.tooth_depth().value(), 4.5); // h  = 2.25m
assert_eq!(gear.clearance().value(),   0.5); // c  = 0.25m

// Gear pair
let pinion = Gear::builder().module(Module::Specified(2.0)).teeth(20).build()?;
let wheel  = Gear::builder().module(Module::Specified(2.0)).teeth(40).build()?;

assert!(pinion.can_mesh_with(&wheel));
assert_eq!(pinion.center_distance_to(&wheel).value(), 60.0); // (40 + 80) / 2
assert_eq!(pinion.gear_ratio_to(&wheel), 2.0);               // 40 / 20
```

```rust
use for_the_love_of_gears::{
    helical::{HelicalGear, HelixHand},
    module::Module,
};

// Helical gear pair — opposite hands required for parallel-shaft meshing
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

assert!(driver.can_mesh_with(&driven));

// Transverse contact ratio (how many tooth pairs share load on average)
let ea = driver.transverse_contact_ratio_with(&driven);
assert!(ea.value() > 1.0);

// Total contact ratio (transverse + axial overlap — requires face width)
let eg = driver.total_contact_ratio_with(&driven).unwrap();
assert!(eg.value() > ea.value()); // helical overlap adds smoothness
```

---

## License

[MIT](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE) — Tanishq Jain
