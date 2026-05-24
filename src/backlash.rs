//! Backlash — the intentional clearance between mating gear teeth.
//!
//! # What is backlash?
//!
//! In a perfect gear pair, tooth thickness exactly equals the tooth space width
//! at the pitch circle. In practice, teeth are cut slightly thinner than
//! theoretical so that the gears do not jam due to thermal expansion,
//! lubrication film thickness, or manufacturing tolerances. The resulting gap
//! between a tooth flank and the opposing tooth space is **backlash**.
//!
//! ```text
//!  ──── pitch circle ────────────────────────────────────
//!
//!       │← s₁ →│    ← tooth 1 (thinned)
//!                │←jt→│   ← backlash gap
//!                      │← s₂ →│   ← tooth 2 (thinned)
//!
//!  │←────────── p = πm ────────────→│   one full pitch
//!
//!  jt = p − s₁ − s₂ = Δs₁ + Δs₂
//! ```
//!
//! Backlash is a **pair-level** property: it is the sum of the tooth thinning
//! contributed by each gear. Standard practice distributes it equally — each
//! gear is thinned by `jt / 2` — though in principle all thinning can be
//! applied to one gear.
//!
//! # Two ways to measure backlash
//!
//! [`Backlash`] (`jt`) is the gap measured **along the pitch circle** (arc
//! length, mm). This is the design input and what most engineers specify.
//!
//! [`NormalBacklash`] (`jn`) is the gap measured **perpendicular to the tooth
//! flank**. It is smaller than `jt` and is what a feeler gauge or dial
//! indicator measures when the probe is held normal to the tooth surface:
//!
//! ```text
//!  jn = jt · cos(α)              (spur)
//!  jn = jt · cos(αt) · cos(ψ)   (helical)
//! ```
//!
//! # Effect on tooth thickness
//!
//! The theoretical tooth thickness at the pitch circle is `s = πm / 2`.
//! Applying backlash `jt` equally to both gears gives each gear a thinned
//! tooth thickness of `s' = πm / 2 − jt / 2`. Use
//! [`crate::gear::Gear::thinned_tooth_thickness`] or
//! [`crate::helical::HelicalGear::thinned_tooth_thickness`] to compute this.
//!
//! # Typical values
//!
//! Backlash scales with module. For general industrial machinery a circular
//! backlash of roughly `0.04 × m` to `0.06 × m` mm is common, but the exact
//! value depends on speed, temperature range, lubrication, and the required
//! positioning accuracy.

use std::f64::consts::PI;

/// Total circular backlash of a gear pair: the arc-length gap between mating
/// teeth measured along the pitch circle in **millimetres**.
///
/// Backlash is a pair-level value — it is the sum of the tooth thinning from
/// both gears. See the [module-level documentation](crate::backlash) for a
/// full explanation.
///
/// Use [`crate::gear::Gear::thinned_tooth_thickness`] or
/// [`crate::gear::Gear::normal_backlash`] to apply this to a specific gear.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Backlash(f64);

impl Backlash {
    /// Create a backlash value from the total circular gap `jt` in **millimetres**.
    ///
    /// ```
    /// use for_the_love_of_gears::backlash::Backlash;
    /// let jt = Backlash::new(0.08); // 0.08 mm circular backlash
    /// assert_eq!(jt.value(), 0.08);
    /// ```
    pub fn new(jt_mm: f64) -> Self {
        Self(jt_mm)
    }

    /// Returns the circular backlash in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }

    /// Tooth thinning applied to each gear when backlash is distributed equally:
    /// `Δs = jt / 2`.
    ///
    /// The two gears together produce the full backlash gap; each contributes
    /// half under the standard equal-distribution assumption.
    ///
    /// ```
    /// use for_the_love_of_gears::backlash::Backlash;
    /// let jt = Backlash::new(0.08);
    /// assert!((jt.per_gear_thinning() - 0.04).abs() < 1e-10);
    /// ```
    pub fn per_gear_thinning(self) -> f64 {
        self.0 / 2.0
    }
}

/// Backlash measured perpendicular to the tooth flank in **millimetres**.
///
/// Normal backlash `jn` is smaller than circular backlash `jt` — the tooth
/// flank is angled relative to the pitch circle by the pressure angle, so the
/// gap appears narrower when measured normal to the surface.
///
/// Use [`NormalBacklash::from_spur`] or [`NormalBacklash::from_helical`]
/// directly, or call [`crate::gear::Gear::normal_backlash`] /
/// [`crate::helical::HelicalGear::normal_backlash`] on a built gear.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NormalBacklash(f64);

impl NormalBacklash {
    /// Compute normal backlash for a spur gear: `jn = jt · cos(α)`.
    ///
    /// `pressure_angle_deg` is in **degrees**.
    ///
    /// ```
    /// use for_the_love_of_gears::backlash::{Backlash, NormalBacklash};
    /// let jt = Backlash::new(0.08);
    /// let jn = NormalBacklash::from_spur(jt, 20.0);
    /// let expected = 0.08 * 20.0_f64.to_radians().cos();
    /// assert!((jn.value() - expected).abs() < 1e-10);
    /// ```
    pub fn from_spur(backlash: Backlash, pressure_angle_deg: f64) -> Self {
        Self(backlash.value() * pressure_angle_deg.to_radians().cos())
    }

    /// Compute normal backlash for a helical gear: `jn = jt · cos(αt) · cos(ψ)`.
    ///
    /// `transverse_pressure_angle_deg` and `helix_angle_deg` are in **degrees**.
    ///
    /// ```
    /// use for_the_love_of_gears::backlash::{Backlash, NormalBacklash};
    /// let jt = Backlash::new(0.08);
    /// let jn = NormalBacklash::from_helical(jt, 21.17, 20.0); // αt≈21.17° for αn=20°, ψ=20°
    /// assert!(jn.value() < jt.value());
    /// ```
    pub fn from_helical(
        backlash: Backlash,
        transverse_pressure_angle_deg: f64,
        helix_angle_deg: f64,
    ) -> Self {
        let cos_at = transverse_pressure_angle_deg.to_radians().cos();
        let cos_psi = helix_angle_deg.to_radians().cos();
        Self(backlash.value() * cos_at * cos_psi)
    }

    /// Returns the normal backlash in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Thinned tooth thickness: `s' = πm / 2 − jt / 2` (spur, in the pitch plane).
///
/// This is a low-level constructor used by [`crate::gear::Gear::thinned_tooth_thickness`]
/// and [`crate::helical::HelicalGear::thinned_tooth_thickness`]. Prefer those
/// methods over calling this directly.
pub(crate) fn thinned_thickness_spur(module: f64, backlash: Backlash) -> f64 {
    PI * module / 2.0 - backlash.per_gear_thinning()
}

/// Thinned normal-plane tooth thickness for helical: `sn' = πmn / 2 − (jt / 2) · cos(ψ)`.
///
/// The thinning is applied in the transverse plane (`Δst = jt/2`) and projected
/// into the normal plane via `cos(ψ)`.
pub(crate) fn thinned_thickness_helical(
    normal_module: f64,
    backlash: Backlash,
    helix_angle_deg: f64,
) -> f64 {
    PI * normal_module / 2.0 - backlash.per_gear_thinning() * helix_angle_deg.to_radians().cos()
}
