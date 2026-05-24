//! Tooth spacing — circular pitch and diametral pitch.
//!
//! Two different ways to express how far apart adjacent teeth are:
//!
//! - **Circular pitch** (`p`, mm) — arc length between adjacent teeth along the pitch
//!   circle. `p = π × m`. This is the natural companion to module in metric systems.
//!
//! - **Diametral pitch** (`DP`, teeth/in) — number of teeth per inch of pitch diameter.
//!   `DP = 25.4 / m`. Used in North American and some aerospace standards.
//!
//! The two are reciprocal: `p × DP = π × 25.4 ≈ 79.8`.
//!
//! For metric design work you rarely need these types directly —
//! [`crate::module::Module`] encodes tooth size completely. They are useful when
//! interfacing with inch-unit tooling or comparing datasheets from mixed-unit systems.

use std::f64::consts::PI;

const MM_PER_INCH: f64 = 25.4;

/// The arc length between two adjacent teeth, measured along the pitch circle.
///
/// Circular pitch `p` is the metric way to express tooth spacing. It is directly
/// proportional to module: a larger module means larger, more widely spaced teeth.
///
/// ```text
/// p = π × m
/// ```
///
/// # Visualisation
///
/// ```text
///        p
///   ◄─────────►
///   │    │    │
///  ─┴────┴────┴─  ← pitch circle
///    tooth  tooth
/// ```
///
/// Compare with [`DiametralPitch`], which expresses the same concept in the
/// imperial system as teeth-per-inch rather than mm-per-tooth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircularPitch(f64);

impl CircularPitch {
    /// Compute circular pitch from module: `p = π × m`.
    ///
    /// ```
    /// use for_the_love_of_gears::pitch::CircularPitch;
    /// use std::f64::consts::PI;
    /// let p = CircularPitch::from_module(1.0);
    /// assert!((p.value() - PI).abs() < 1e-10);
    /// ```
    pub fn from_module(m: f64) -> Self {
        Self(PI * m)
    }

    /// Returns the circular pitch in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// The number of teeth per inch of pitch circle diameter (imperial system).
///
/// "Diametral" means *of or relating to a diameter* — `DP` counts how many
/// teeth fit per inch of pitch circle diameter. It is a ratio (teeth/inch),
/// not a length. Do not confuse it with [`CircularPitch`], which is an arc
/// length in mm between adjacent teeth.
///
/// `DP` runs *inversely* to module and tooth size — a higher `DP` means
/// smaller, finer teeth. Common in inch-unit industries (North America,
/// some aerospace).
///
/// ```text
/// DP = z / d_inches = 25.4 / m      (25.4 mm = 1 inch)
/// ```
///
/// # Comparison
///
/// | Module (mm) | Diametral Pitch (teeth/in) | Tooth size |
/// |---|---|---|
/// | 1 | 25.4 | fine |
/// | 2 | 12.7 | medium |
/// | 4 | 6.35 | coarse |
///
/// For metric work, use [`crate::module::Module`] directly — module is the
/// metric tooth size unit and is all you need.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiametralPitch(f64);

impl DiametralPitch {
    /// Compute diametral pitch from module: `DP = 25.4 / m`.
    ///
    /// ```
    /// use for_the_love_of_gears::pitch::DiametralPitch;
    /// let dp = DiametralPitch::from_module(1.0);
    /// assert!((dp.value() - 25.4).abs() < 1e-10);
    /// ```
    pub fn from_module(m: f64) -> Self {
        Self(MM_PER_INCH / m)
    }

    /// Returns the diametral pitch in teeth per inch.
    pub fn value(self) -> f64 {
        self.0
    }
}
