//! Named constants for ISO gear geometry.
//!
//! These are the standard coefficients defined in **ISO 21771** (geometry of
//! cylindrical involute gears) and **ISO 54** (module series). Referencing them
//! by name instead of raw literals makes the intent clear and connects the code
//! directly to the standards.
//!
//! # Standard tooth proportions
//!
//! ISO defines tooth proportions as multiples of the **module** `m`. The module
//! is the fundamental size parameter — doubling the module doubles every linear
//! dimension of the tooth while keeping the tooth shape identical.
//!
//! ```text
//! Dimension         ISO symbol   Coefficient   Formula
//! ──────────────── ──────────── ────────────  ──────────────
//! Addendum          ha*          1.000         ha = 1.000 · m
//! Dedendum          hf*          1.250         hf = 1.250 · m
//! Whole depth       h*           2.250         h  = 2.250 · m
//! Clearance         c*           0.250         c  = 0.250 · m
//! ```
//!
//! The clearance (`c* = 0.25`) is the extra 0.25·m of dedendum beyond the
//! addendum. It prevents the tip of the mating gear from bottoming out in the
//! root — a safety margin for thermal expansion, lubricant film, and
//! manufacturing tolerances.

/// Standard pressure angle in degrees (ISO 21771 §5.1).
///
/// `20°` is the worldwide default for modern general-purpose spur and helical
/// gears. It replaced the older `14.5°` standard (still found on legacy
/// machinery) because 20° produces a stronger tooth form, a smaller base
/// circle relative to the pitch circle, and better load distribution. For
/// fine-pitch instrument gears a 25° angle is sometimes specified.
///
/// This value is used as the builder default in [`Gear`] and [`HelicalGear`].
///
/// [`Gear`]: crate::gear::Gear
/// [`HelicalGear`]: crate::helical::HelicalGear
pub const ISO_PRESSURE_ANGLE_DEG: f64 = 20.0;

/// Addendum coefficient `ha* = 1.0` (ISO 21771 §5.3).
///
/// The addendum is `ha = ha* · m`. A coefficient of exactly 1 means the tip
/// circle is one module above the pitch circle. This value is fixed by the
/// standard so that racks and gears cut with standard tooling interchangeably.
pub const ADDENDUM_COEFFICIENT: f64 = 1.0;

/// Dedendum coefficient `hf* = 1.25` (ISO 21771 §5.3).
///
/// The dedendum is `hf = hf* · m`. The extra 0.25·m beyond the addendum
/// becomes the **clearance gap** — the space between the tip of one gear
/// and the root of its mating gear. Without this gap, thermal expansion or
/// manufacturing tolerances can cause the teeth to jam.
pub const DEDENDUM_COEFFICIENT: f64 = 1.25;

/// Whole-depth coefficient `h* = 2.25` (ISO 21771 §5.3).
///
/// Equal to [`ADDENDUM_COEFFICIENT`] + [`DEDENDUM_COEFFICIENT`].
/// The whole depth is `h = h* · m = ha + hf`.
pub const WHOLE_DEPTH_COEFFICIENT: f64 = 2.25;

/// Clearance coefficient `c* = 0.25` (ISO 21771 §5.3).
///
/// Equal to [`DEDENDUM_COEFFICIENT`] − [`ADDENDUM_COEFFICIENT`].
/// Clearance is `c = c* · m` — the radial gap between the tip circle of one
/// gear and the root circle of its mate at the pitch point. It accommodates
/// lubricant film, thermal growth, and root-fillet radius.
pub const CLEARANCE_COEFFICIENT: f64 = 0.25;

/// Millimetres per inch, used for diametral pitch conversion.
///
/// Diametral pitch (`DP`) is the imperial-system equivalent of module:
/// `DP = 25.4 / m`. It counts the number of teeth per inch of pitch diameter.
/// A module-2 gear has `DP ≈ 12.7 t/in`; a module-3 gear has `DP ≈ 8.5 t/in`.
pub const MM_PER_INCH: f64 = 25.4;

/// Minimum tooth count that produces a positive root diameter.
///
/// With the standard dedendum coefficient of 1.25 the root diameter formula
/// `df = m(z − 2.5)` equals zero at `z = 2.5` and goes negative for smaller
/// values. Since tooth counts are integers, `z ≥ 3` is the minimum for a
/// geometrically valid root circle under ISO standard proportions.
pub const MIN_TEETH: u32 = 3;
