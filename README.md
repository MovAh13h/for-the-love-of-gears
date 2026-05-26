# For the love of Gears!

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-dark.svg">
  <img alt="for-the-love-of-gears" src="https://raw.githubusercontent.com/MovAh13h/for-the-love-of-gears/master/assets/banner-light.svg" width="1280">
</picture>

[![CI](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml/badge.svg)](https://github.com/MovAh13h/for-the-love-of-gears/actions/workflows/ci.yml)
[![Crates.io](https://img.shields.io/crates/v/for_the_love_of_gears.svg)](https://crates.io/crates/for_the_love_of_gears)
[![Docs.rs](https://docs.rs/for_the_love_of_gears/badge.svg)](https://docs.rs/for_the_love_of_gears)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE)

ISO-standard involute gear geometry for Rust. Model spur and helical gears, check mesh compatibility, compute contact ratios and backlash, then compose gears into named gear trains and simulate kinematics — RPM, direction, and angular position at any point in time.

**[Full API reference on docs.rs →](https://docs.rs/for_the_love_of_gears)**

## Installation

```toml
[dependencies]
for_the_love_of_gears = "0.1"
```

---

## Spur gears

A spur gear is the simplest kind of gear — the one you picture when someone says "gear". The teeth run straight across the face, parallel to the shaft. They're cheap to make, easy to understand, and produce no sideways (axial) force on their bearings. The trade-off is that they can be noisy at high speed, because all the teeth slam into contact at once rather than gradually.

Every spur gear is described by two numbers:
- **Module** (`m`) — controls how big the teeth are. A module-2 gear has teeth twice as large as a module-1 gear. Two gears must share the same module to mesh.
- **Tooth count** (`z`) — how many teeth are on the gear. Together with the module, this sets the size of the gear: pitch diameter = `m × z`.

```rust
use for_the_love_of_gears::gear::Gear;

let gear = Gear::builder()
    .module(2.0)   // tooth size in mm
    .teeth(20)
    .build()?;

// Diameters
assert_eq!(gear.reference_diameter(), 40.0); // d  = m·z
assert_eq!(gear.tip_diameter(),       44.0); // da = m(z + 2)
assert_eq!(gear.root_diameter(),      35.0); // df = m(z − 2.5)

// Tooth profile
assert_eq!(gear.addendum(),    2.0);  // ha = m       — tooth height above pitch circle
assert_eq!(gear.dedendum(),    2.5);  // hf = 1.25·m  — tooth depth below pitch circle
assert_eq!(gear.tooth_depth(), 4.5);  // h  = 2.25·m  — full tooth height
assert_eq!(gear.clearance(),   0.5);  // c  = 0.25·m  — gap between tip and root of mate
```

---

## Helical gears

A helical gear is like a spur gear whose teeth have been twisted along the shaft — imagine a spiral staircase instead of a straight staircase. Because of the twist, each tooth slides into contact gradually from one end to the other rather than all at once. This makes helical gears quieter and smoother than spur gears.

The twist comes with one trade-off: it produces an axial thrust force (a push along the shaft) that the bearings must handle. The steeper the twist (larger helix angle), the smoother the gear — but the more thrust it generates. Most practical helical gears use a helix angle between 15° and 30°.

Helical gears also have a direction: **right-hand** teeth spiral upward to the right (like a right-handed screw), and **left-hand** teeth spiral the other way. Two gears on parallel shafts must have **opposite hands** to mesh — a right-hand gear pairs with a left-hand gear.

```rust
use for_the_love_of_gears::helical::{HelicalGear, HelixHand};

let gear = HelicalGear::builder()
    .module(2.0)               // normal module mn — the tooth size parameter
    .teeth(20)
    .helix_angle(20.0)         // ψ in degrees — the twist angle
    .helix_hand(HelixHand::Right)
    .face_width(30.0)          // axial length of the gear — needed for overlap ratio
    .build()?;

// The helix stretches the apparent pitch in the rotation plane
let mt = gear.transverse_module();           // mn / cos ψ  — always larger than mn
let at = gear.transverse_pressure_angle();   // atan(tan αn / cos ψ)

// Axial geometry
let pa = gear.axial_pitch();   // distance along the shaft between teeth
let L  = gear.lead();          // distance one tooth travels axially in one full revolution
```

---

## Gear pairs

Two gears mesh when their teeth interlock and one drives the other to rotate. For this to work cleanly the gears must be **compatible** — same module, same pressure angle, and for helical gears the same helix angle magnitude with opposite hands.

The key relationship is the **gear ratio**: the bigger gear rotates slower, the smaller gear rotates faster. If the driving gear has 20 teeth and the driven gear has 40 teeth, the driven gear spins at half the speed — a 2:1 reduction.

```rust
use for_the_love_of_gears::gear::Gear;

let pinion = Gear::builder().module(2.0).teeth(20).build()?;
let wheel  = Gear::builder().module(2.0).teeth(40).build()?;

assert!(pinion.can_mesh_with(&wheel));               // compatible?
assert_eq!(pinion.gear_ratio_to(&wheel), 2.0);       // 2:1 reduction
assert_eq!(pinion.center_distance_to(&wheel), 60.0); // shaft spacing in mm
```

### Contact ratio

The **contact ratio** is the average number of tooth pairs sharing the load at any instant. A ratio of 1.0 means exactly one pair of teeth is in contact at all times — the load transfers fully from one pair to the next with no overlap. A ratio of 1.6 means the gears spend 60% of their time with two pairs sharing the load and 40% with one. Higher is smoother and quieter; anything below 1.2 is rough and noisy.

```rust
let cr = pinion.contact_ratio_with(&wheel).unwrap(); // aim for > 1.2

// Helical gears add an overlap ratio εβ from the helix sweeping across the face
let ea = g1.transverse_contact_ratio_with(&g2).unwrap(); // profile overlap
let eb = g1.overlap_ratio().unwrap();                     // helix contribution
let eg = g1.total_contact_ratio_with(&g2).unwrap();      // εα + εβ — often > 2.0
```

---

## Interference and undercutting

### Undercutting

When a gear has very few teeth, the cutting tool that shapes the teeth has to dig into the base of the tooth to clear the mating gear's tip. This gouging is called **undercutting**. It weakens the tooth at its root and can cause interference with the mating gear. For a standard 20° pressure angle, gears with fewer than 17 teeth are at risk.

Helical gears are more forgiving — the helix effectively increases the virtual tooth count, so a helical gear with 14 teeth might be perfectly fine where an equivalent spur gear would undercut.

```rust
use for_the_love_of_gears::gear::Gear;

let small = Gear::builder().module(2.0).teeth(14).build()?;
let large = Gear::builder().module(2.0).teeth(40).build()?;

assert!(small.is_undercut());   // too few teeth — cutting tool gouges the root
assert!(!large.is_undercut());
```

### Interference

Interference is a more severe problem: the tip of one gear reaches past the **base circle** of the other, into the zone where the involute tooth profile doesn't exist. The tip physically digs into the root flank of the mating gear — the teeth lock up. This typically happens when a small pinion meshes with a much larger wheel.

```rust
let pinion = Gear::builder().module(2.0).teeth(12).build()?;
let wheel  = Gear::builder().module(2.0).teeth(60).build()?;
assert!(pinion.interferes_with(&wheel).unwrap()); // tip digs into root — bad

let g20 = Gear::builder().module(2.0).teeth(20).build()?;
let g40 = Gear::builder().module(2.0).teeth(40).build()?;
assert!(!g20.interferes_with(&g40).unwrap());     // safe pair
```

---

## Backlash

Backlash is the small intentional gap between the drive flank of one tooth and the coast flank of the mating tooth. Think of it as the play you feel when you reverse the direction of a hand drill — there's a tiny amount of rotation before the gears re-engage in the new direction.

Backlash is not a flaw. Without it, thermal expansion could cause the teeth to jam as the gear train heats up, and there would be no room for a lubricant film between the flanks. Precision gearboxes use 0.05–0.15 mm; coarse industrial drives allow up to 0.5 mm.

Backlash is created by thinning each tooth slightly. The total gap `jt` is split equally between the two gears, so each is thinned by `jt / 2`.

```rust
use for_the_love_of_gears::gear::Gear;

let gear = Gear::builder().module(2.0).teeth(20).build()?;
let jt = 0.08; // 80 µm total transverse backlash

// Theoretical tooth thickness is π·m/2; backlash thins it by jt/2
let s_prime = gear.thinned_tooth_thickness(jt).unwrap();

// Normal backlash — what a feeler gauge actually reads at the tooth flank
let jn = gear.normal_backlash(jt).unwrap();
assert!(jn < jt); // projection through the pressure angle makes it smaller
```

---

## Gear scenes and kinematic simulation

A **gear scene** is a collection of shafts, each carrying one or more gears, connected by mesh declarations. Once built, the scene pre-computes the speed ratio of every shaft relative to the driver. Calling `run` at any RPM is then instant — it just scales those pre-computed ratios.

### Simple pair

```rust
use for_the_love_of_gears::{gear::Gear, scene::{AnyGear, GearScene}};

let scene = GearScene::builder()
    .shaft("input",  vec![("a", AnyGear::from(Gear::builder().module(2.0).teeth(20).build()?))])
    .shaft("output", vec![("b", AnyGear::from(Gear::builder().module(2.0).teeth(40).build()?))])
    .mesh("a", "b")
    .driver("input")
    .build()?;

let sim = scene.run(1000.0)?;                       // 1 000 rpm input
assert_eq!(sim.rpm("output"),      Some(500.0));    // 2:1 reduction
assert_eq!(sim.ratio_to("output"), Some(2.0));
```

### Compound trains

A **compound shaft** carries two gears that rotate together. The first gear is driven by the previous stage; the second gear drives the next stage. This multiplies the reduction ratios of each stage together, achieving large overall ratios in a compact package.

```rust
use for_the_love_of_gears::{gear::Gear, scene::{AnyGear, GearScene}};

// Stage 1: 20t → 40t (2:1).  Stage 2: 20t → 60t (3:1).  Total: 6:1.
let scene = GearScene::builder()
    .shaft("input",        vec![("a", AnyGear::from(Gear::builder().module(2.0).teeth(20).build()?))])
    .shaft("intermediate", vec![
        ("b", AnyGear::from(Gear::builder().module(2.0).teeth(40).build()?)),
        ("c", AnyGear::from(Gear::builder().module(3.0).teeth(20).build()?)),
    ])
    .shaft("output",       vec![("d", AnyGear::from(Gear::builder().module(3.0).teeth(60).build()?))])
    .mesh("a", "b")
    .mesh("c", "d")
    .driver("input")
    .build()?;

let sim = scene.run(1200.0)?;
// 1 200 rpm ÷ 6 = 200 rpm output
assert!((sim.rpm("output").unwrap() - 200.0).abs() < 1e-9);
```

### Querying the simulation

```rust
// Speed and direction
let rpm   = sim.rpm("output").unwrap();
let dir   = sim.direction("output").unwrap(); // Direction::Clockwise / CounterClockwise
let omega = sim.angular_velocity_rad_s("output").unwrap(); // rad/s

// Where is the shaft pointing right now?
let angle = sim.angle_deg("output", 1.5).unwrap(); // degrees in [0°, 360°)

// How many full turns in 5 seconds?
let revs  = sim.total_rotations("output", 5.0).unwrap();

// What is the overall reduction from driver to this shaft?
let ratio = sim.ratio_to("output").unwrap(); // driver_rpm / shaft_rpm
```

---

## Animation frames

`frames` generates a keyframe for every tick of a renderer — each frame contains the angular position of every shaft at that instant. Look up each shaft's index once and reuse it across all frames.

```rust
let frames   = sim.frames(24.0, 2.0); // 24 fps, 2 seconds → 49 frames
let i_output = sim.shaft_index("output").unwrap();

for frame in &frames {
    let t     = frame.time_secs;
    let angle = frame.shaft_angles[i_output]; // degrees, [0°, 360°)
    // pass to your renderer here
}
```

---

## Display

Every gear type implements `Display` for quick human-readable output — useful for logging, debugging, or printing gear specs to a user.

```rust
use for_the_love_of_gears::gear::Gear;
use for_the_love_of_gears::helical::{HelicalGear, HelixHand};

let g = Gear::builder().module(2.0).teeth(20).build()?;
println!("{g}");
// Gear { m=2, z=20, α=20° }

let h = HelicalGear::builder()
    .module(2.0).teeth(20)
    .helix_angle(20.0).helix_hand(HelixHand::Right)
    .face_width(30.0)
    .build()?;
println!("{h}");
// HelicalGear { mn=2, z=20, ψ=20° right-hand, αn=20°, b=30 }
```

---

## Serialisation (serde)

Enable the `serde` feature to derive `Serialize` and `Deserialize` for all public types:

```toml
[dependencies]
for_the_love_of_gears = { version = "0.1", features = ["serde"] }
```

```rust
use for_the_love_of_gears::gear::Gear;

let g = Gear::builder().module(2.0).teeth(20).build()?;
let json = serde_json::to_string(&g)?;
let back: Gear = serde_json::from_str(&json)?;
assert_eq!(g, back);
```

All gear types (`Gear`, `HelicalGear`, `HelixHand`, `AnyGear`), direction and frame types (`Direction`, `SimFrame`), and error types (`GearError`, `GearSceneError`) are covered.

---

## Generic code over any gear type

`GearGeometry` is implemented by `Gear`, `HelicalGear`, and `AnyGear`. Write functions that accept any gear without caring which type it is:

```rust
use for_the_love_of_gears::traits::GearGeometry;

fn print_profile(label: &str, g: &impl GearGeometry) {
    println!("{label}: d={:.1}  da={:.1}  df={:.1}  h={:.2}",
        g.reference_diameter(),
        g.tip_diameter(),
        g.root_diameter(),
        g.tooth_depth(),
    );
}
```

---

## Examples

The `examples/` directory contains runnable programs covering spur pairs, helical pairs, compound trains, branched trains, mixed spur+helical scenes, and animation frame output.

```sh
cargo run --example 01_spur_pair
```

---

## License

[MIT](https://github.com/MovAh13h/for-the-love-of-gears/blob/master/LICENSE) — Tanishq Jain
