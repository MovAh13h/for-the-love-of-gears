//! Gear geometry library following ISO standards.
//!
//! This library models the geometry of involute spur and helical gears,
//! composed gear trains, and basic kinematic simulation. It is designed to
//! be learnable as well as practical — every formula is explained in its
//! doc comment alongside the ISO standard that defines it.
//!
//! # Module guide
//!
//! | Module | What it provides |
//! |---|---|
//! | [`gear`] | Spur gear geometry — the simplest involute gear |
//! | [`helical`] | Helical gear geometry — quieter, with axial thrust |
//! | [`scene`] | Compose gear trains and simulate RPMs and angles |
//! | [`module`] | Convert between module, circular pitch, and diameter |
//! | [`constants`] | Named ISO coefficients (addendum, dedendum, pressure angle…) |
//! | [`traits`] | [`GearGeometry`] — shared interface for generic gear code |
//!
//! # The pitch circle — the central concept
//!
//! The **pitch circle** is the most important concept in gear geometry.
//! Everything else is defined relative to it.
//!
//! When two gears mesh, each has an imaginary circle that rolls against the
//! other's without slipping. These are the pitch circles. The point where they
//! touch is called the **pitch point** and lies on the straight line between
//! the two gear centres:
//!
//! ```text
//!    c₁       pitch point      c₂
//!     ●────────────●────────────●
//!     ←── d₁/2 ───→←── d₂/2 ───→
//!     ←─────────── a ────────────→
//! ```
//!
//! The pitch circle diameter is `d = m × z` (module × tooth count). Every
//! other dimension is anchored to it:
//!
//! - **Addendum** (`ha = m`) — tooth extends one module above the pitch circle
//! - **Dedendum** (`hf = 1.25m`) — tooth extends 1.25 modules below the pitch circle
//! - **Tip circle** — pitch circle radius + addendum
//! - **Root circle** — pitch circle radius − dedendum
//! - **Base circle** — `d · cos(α)`; the involute tooth profile unrolls from here
//!
//! Two gears can only mesh if their pitch circles are tangent at the pitch
//! point — the centre distance must equal exactly `(d₁ + d₂) / 2`.
//!
//! # Involute geometry in brief
//!
//! All gears in this library use an **involute** tooth profile. The involute
//! of a circle is the curve traced by unwrapping a taut string from that
//! circle. Its key property: two involute gears produce a **constant velocity
//! ratio** regardless of small centre-distance errors. No other tooth form has
//! this property, which is why involute gearing dominates industry.
//!
//! The involute starts at the base circle and runs outward to the tip. The
//! region below the base circle is a fillet — straight or curved — that the
//! cutting tool leaves behind.
//!
//! # Units
//!
//! All linear dimensions are in **millimetres (mm)** (ISO 54, ISO 1328).
//! Angles are in **degrees** at API boundaries; radians are used internally.
//!
//! # Quick start
//!
//! ```
//! use for_the_love_of_gears::gear::Gear;
//!
//! let driver = Gear::builder().module(2.0).teeth(20).build().unwrap();
//! let driven = Gear::builder().module(2.0).teeth(40).build().unwrap();
//!
//! assert!(driver.can_mesh_with(&driven));
//! assert_eq!(driver.gear_ratio_to(&driven), 2.0);       // 2:1 reduction
//! assert_eq!(driver.center_distance_to(&driven), 60.0); // shaft spacing
//! ```
//!
//! [`GearGeometry`]: crate::traits::GearGeometry

use std::fmt;

pub(crate) const MESH_TOLERANCE: f64 = 1e-9;

pub(crate) mod contact_ratio;
pub mod constants;
pub mod gear;
pub mod helical;
pub mod module;
pub mod scene;
pub mod traits;

/// Error returned by backlash methods when the supplied backlash value is negative.
#[derive(Debug, PartialEq)]
#[non_exhaustive]
pub enum BacklashError {
    /// The supplied backlash value was negative.
    NegativeBacklash,
}

impl fmt::Display for BacklashError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BacklashError::NegativeBacklash => f.write_str("backlash must be non-negative"),
        }
    }
}

impl std::error::Error for BacklashError {}
