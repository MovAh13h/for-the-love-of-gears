//! Shared trait for gear geometry.
//!
//! [`GearGeometry`] abstracts the properties common to all gear types, allowing
//! generic code to work uniformly with [`Gear`], [`HelicalGear`], and any future
//! gear variant.
//!
//! # Example
//!
//! ```
//! use for_the_love_of_gears::{
//!     gear::Gear,
//!     helical::{HelicalGear, HelixHand},
//!     traits::GearGeometry,
//! };
//!
//! fn summarise(g: &impl GearGeometry) {
//!     println!(
//!         "z={} d={:.1} da={:.1} df={:.1}",
//!         g.teeth(), g.reference_diameter(), g.tip_diameter(), g.root_diameter()
//!     );
//! }
//!
//! let spur = Gear::builder().module(2.0).teeth(20).build().unwrap();
//! let helical = HelicalGear::builder()
//!     .module(2.0).teeth(20).helix_angle(20.0).helix_hand(HelixHand::Right)
//!     .build().unwrap();
//!
//! summarise(&spur);
//! summarise(&helical);
//! ```
//!
//! [`Gear`]: crate::gear::Gear
//! [`HelicalGear`]: crate::helical::HelicalGear

/// Geometric properties shared by all gear types.
///
/// Implemented by [`Gear`], [`HelicalGear`], and [`AnyGear`].
///
/// For helical gears, all module- and pitch-based dimensions use the
/// **normal module** (`mn`), while diameters use the transverse plane.
/// The trait surfaces the same conceptual properties regardless of gear type.
///
/// # Provided default methods
///
/// [`gear_ratio_to`] and [`center_distance_to`] have default implementations
/// derived from `teeth()` and `reference_diameter()`. Types may override them
/// if their definition diverges from the standard formulas.
///
/// [`Gear`]: crate::gear::Gear
/// [`HelicalGear`]: crate::helical::HelicalGear
/// [`AnyGear`]: crate::scene::AnyGear
/// [`gear_ratio_to`]: GearGeometry::gear_ratio_to
/// [`center_distance_to`]: GearGeometry::center_distance_to
pub trait GearGeometry {
    // ── Inputs ────────────────────────────────────────────────────────────────

    /// Number of teeth.
    fn teeth(&self) -> u32;

    // ── Diameters (mm) ────────────────────────────────────────────────────────

    /// Pitch circle diameter in mm.
    fn reference_diameter(&self) -> f64;

    /// Tip (outer) diameter in mm.
    fn tip_diameter(&self) -> f64;

    /// Root diameter in mm.
    fn root_diameter(&self) -> f64;

    /// Base circle diameter in mm.
    fn base_diameter(&self) -> f64;

    // ── Tooth profile (mm) ────────────────────────────────────────────────────

    /// Addendum — radial height above the pitch circle in mm.
    fn addendum(&self) -> f64;

    /// Dedendum — radial depth below the pitch circle in mm.
    fn dedendum(&self) -> f64;

    /// Full tooth height root-to-tip in mm.
    fn tooth_depth(&self) -> f64;

    /// Tip-to-root radial clearance in mm.
    fn clearance(&self) -> f64;

    /// Theoretical tooth thickness in mm.
    fn tooth_thickness(&self) -> f64;

    // ── Pitch ─────────────────────────────────────────────────────────────────

    /// Teeth per inch of pitch diameter (imperial).
    fn diametral_pitch(&self) -> f64;

    // ── Backlash ──────────────────────────────────────────────────────────────

    /// Thinned tooth thickness after applying pair backlash `jt` in mm.
    ///
    /// # Panics
    ///
    /// Panics if `backlash_mm < 0.0`.
    fn thinned_tooth_thickness(&self, backlash_mm: f64) -> f64;

    /// Normal backlash from circular backlash `jt` in mm.
    ///
    /// # Panics
    ///
    /// Panics if `backlash_mm < 0.0`.
    fn normal_backlash(&self, backlash_mm: f64) -> f64;

    // ── Pair operations (default implementations) ─────────────────────────────

    /// Speed ratio to `other`: `i = z_other / z_self`.
    ///
    /// Greater than 1 → reduction (other is slower). Less than 1 → step-up.
    fn gear_ratio_to(&self, other: &Self) -> f64
    where
        Self: Sized,
    {
        other.teeth() as f64 / self.teeth() as f64
    }

    /// Centre distance to `other` in mm: `a = (d₁ + d₂) / 2`.
    fn center_distance_to(&self, other: &Self) -> f64
    where
        Self: Sized,
    {
        (self.reference_diameter() + other.reference_diameter()) / 2.0
    }
}
