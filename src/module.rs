//! Gear module — the tooth size unit.
//!
//! Module `m` is the fundamental size unit of a gear tooth. Every linear gear
//! dimension scales with it: pitch diameter = `m × z`, addendum = `m`,
//! dedendum = `1.25 × m`. Two gears can mesh only if they share the same module.
//!
//! # Specifying module
//!
//! Pass `m` as a plain `f64` to the gear builders — for example `.module(2.0)`.
//! Use the helpers below only when you need to derive `m` from other measurements.
//!
//! # ISO preferred values (mm)
//!
//! ```text
//! 1, 1.25, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10, 12, 16, 20
//! ```
//!
//! Always choose from ISO preferred values — they ensure off-the-shelf tooling.

use std::f64::consts::PI;

/// Derive module from circular pitch `p` (mm): `m = p / π`.
///
/// Use this when you know the arc length between adjacent teeth but not `m` directly.
///
/// ```
/// use for_the_love_of_gears::module;
/// use std::f64::consts::PI;
/// let m = module::from_circular_pitch(2.0 * PI); // p = 2π → m = 2
/// assert!((m - 2.0).abs() < 1e-10);
/// ```
pub fn from_circular_pitch(p: f64) -> f64 {
    p / PI
}

/// Derive module from pitch circle diameter `d` (mm) and tooth count `z`: `m = d / z`.
///
/// Use this when reverse-engineering an existing gear — measure the pitch circle
/// diameter and count the teeth.
///
/// ```
/// use for_the_love_of_gears::module;
/// let m = module::from_diameter(40.0, 20);
/// assert_eq!(m, 2.0); // 40 / 20
/// ```
pub fn from_diameter(pitch_circle_diameter: f64, teeth: u32) -> f64 {
    pitch_circle_diameter / teeth as f64
}
