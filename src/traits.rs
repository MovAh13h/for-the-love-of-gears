//! Shared trait for gear geometry — write once, works for any gear type.
//!
//! [`GearGeometry`] defines the geometric properties that are conceptually
//! identical across spur gears, helical gears, and any future gear types.
//! By programming to this trait rather than a concrete type, you can write
//! functions and data structures that are generic over gear variety.
//!
//! # Why this trait exists
//!
//! [`Gear`] and [`HelicalGear`] share the same tooth-profile geometry —
//! addendum, dedendum, tooth depth, clearance, tooth thickness, and all four
//! characteristic diameters are defined the same way for both. Without a
//! shared trait, every function that operates on "any gear" would need to
//! duplicate code or resort to a runtime match on [`AnyGear`].
//!
//! # Example — generic over gear type
//!
//! ```
//! use for_the_love_of_gears::{
//!     gear::Gear,
//!     helical::{HelicalGear, HelixHand},
//!     traits::GearGeometry,
//! };
//!
//! /// Print a summary valid for any gear type.
//! fn print_summary(label: &str, g: &impl GearGeometry) {
//!     println!(
//!         "{}: z={} d={:.2} da={:.2} df={:.2}",
//!         label,
//!         g.teeth(),
//!         g.reference_diameter(),
//!         g.tip_diameter(),
//!         g.root_diameter(),
//!     );
//! }
//!
//! let spur = Gear::builder().module(2.0).teeth(20).build().unwrap();
//! let helical = HelicalGear::builder()
//!     .module(2.0).teeth(20).helix_angle(20.0).helix_hand(HelixHand::Right)
//!     .build().unwrap();
//!
//! print_summary("spur",    &spur);
//! print_summary("helical", &helical);
//! ```
//!
//! # What is intentionally excluded
//!
//! The trait does **not** include:
//!
//! - **Pressure angle** — the names differ (`pressure_angle` on `Gear`;
//!   `normal_pressure_angle` on `HelicalGear`). The trait omits these because
//!   the values are not directly comparable across gear types.
//! - **Contact ratio** — the spur and helical methods have different signatures
//!   (`contact_ratio_with` vs `transverse_contact_ratio_with`) and pair methods
//!   require two gears of the *same* type, which is hard to abstract cleanly
//!   without associated types or GATs.
//! - **Helical-only quantities** — helix angle, hand, axial pitch, lead, and
//!   overlap ratio have no spur-gear equivalent.
//!
//! # Default pair methods
//!
//! [`gear_ratio_to`] and [`center_distance_to`] are provided as default
//! implementations because their formulas depend only on `teeth()` and
//! `reference_diameter()`, which are already required by the trait:
//!
//! ```text
//! gear_ratio_to(other)     =  other.teeth() / self.teeth()
//! center_distance_to(other) = (self.reference_diameter() + other.reference_diameter()) / 2
//! ```
//!
//! [`Gear`]: crate::gear::Gear
//! [`HelicalGear`]: crate::helical::HelicalGear
//! [`AnyGear`]: crate::scene::AnyGear
//! [`gear_ratio_to`]: GearGeometry::gear_ratio_to
//! [`center_distance_to`]: GearGeometry::center_distance_to

/// Geometric properties shared by all involute gear types.
///
/// Implemented by [`Gear`], [`HelicalGear`], and [`AnyGear`].
///
/// All lengths are in **millimetres (mm)**; angles are not part of this trait
/// because the naming conventions differ between gear types.
///
/// For helical gears, tooth-profile dimensions (addendum, dedendum, tooth
/// thickness, etc.) use the **normal module** `mn`; pitch-circle dimensions
/// use the **transverse module** `mt`. This is transparent to code that works
/// through the trait — the correct value is returned by the implementation
/// without the caller needing to know which plane it came from.
///
/// [`Gear`]: crate::gear::Gear
/// [`HelicalGear`]: crate::helical::HelicalGear
/// [`AnyGear`]: crate::scene::AnyGear
pub trait GearGeometry {
    // ── Inputs ────────────────────────────────────────────────────────────────

    /// Number of teeth.
    ///
    /// Together with the module, this fully determines the pitch circle
    /// diameter: `d = m · z`. The gear ratio between two meshing gears is
    /// `z_other / z_self`.
    fn teeth(&self) -> u32;

    /// Normal module in mm.
    ///
    /// The normal module is the fundamental tooth-size parameter measured
    /// perpendicular to the tooth helix. For spur gears the normal and
    /// transverse modules are identical. For helical gears:
    /// `mn = mt · cos(ψ)`.
    ///
    /// Two gears can only mesh if they share the same normal module.
    fn normal_module(&self) -> f64;

    // ── Diameters (mm) ────────────────────────────────────────────────────────

    /// Pitch circle diameter in mm: `d = m · z`.
    ///
    /// The pitch circle is the reference circle from which all tooth dimensions
    /// are measured. When two gears mesh, their pitch circles are tangent. For
    /// helical gears `m` here is the transverse module `mt = mn / cos(ψ)`.
    fn reference_diameter(&self) -> f64;

    /// Tip (outer) diameter in mm: `da = d + 2 · ha`.
    ///
    /// The tip circle bounds the gear from the outside. The addendum `ha` is
    /// one (normal) module above the pitch circle, so `da = d + 2 · m`.
    fn tip_diameter(&self) -> f64;

    /// Root diameter in mm: `df = d − 2 · hf`.
    ///
    /// The root circle is where the tooth base meets the gear body, `hf = 1.25 · m`
    /// below the pitch circle: `df = d − 2.5 · m`. A positive root diameter
    /// is enforced by the `MIN_TEETH = 3` constraint in the builders.
    fn root_diameter(&self) -> f64;

    /// Base circle diameter in mm: `db = d · cos(α)`.
    ///
    /// The involute tooth profile unrolls from this circle. For spur gears
    /// `α` is the pressure angle; for helical gears it is the transverse
    /// pressure angle `αt`. The base circle is always smaller than the pitch
    /// circle and larger than the root circle for most practical tooth counts.
    fn base_diameter(&self) -> f64;

    // ── Tooth profile (mm) ────────────────────────────────────────────────────

    /// Addendum — radial height above the pitch circle in mm: `ha = m`.
    ///
    /// Exactly one (normal) module. The ISO standard fixes this so that gears
    /// cut with standard tooling are interchangeable across manufacturers.
    fn addendum(&self) -> f64 {
        crate::constants::ADDENDUM_COEFFICIENT * self.normal_module()
    }

    /// Dedendum — radial depth below the pitch circle in mm: `hf = 1.25 · m`.
    ///
    /// One module of working depth plus 0.25 · m of clearance. The clearance
    /// prevents the tip of the mating gear from bottoming out in the root.
    fn dedendum(&self) -> f64 {
        crate::constants::DEDENDUM_COEFFICIENT * self.normal_module()
    }

    /// Full tooth height from root to tip in mm: `h = ha + hf = 2.25 · m`.
    ///
    /// Every standard involute gear of the same module has the same tooth
    /// height, regardless of tooth count.
    fn tooth_depth(&self) -> f64 {
        crate::constants::WHOLE_DEPTH_COEFFICIENT * self.normal_module()
    }

    /// Tip-to-root radial clearance in mm: `c = hf − ha = 0.25 · m`.
    ///
    /// The gap between the tip of one gear and the root of its mate. It
    /// accommodates lubricant film, thermal expansion, and the root fillet.
    fn clearance(&self) -> f64 {
        crate::constants::CLEARANCE_COEFFICIENT * self.normal_module()
    }

    /// Theoretical tooth thickness along the pitch circle in mm: `s = π · m / 2`.
    ///
    /// On a standard gear the tooth and the gap are equal on the pitch circle,
    /// so each is half the circular pitch. Real gears are thinned slightly to
    /// produce backlash — see [`thinned_tooth_thickness`].
    ///
    /// [`thinned_tooth_thickness`]: GearGeometry::thinned_tooth_thickness
    fn tooth_thickness(&self) -> f64 {
        std::f64::consts::PI * self.normal_module() / 2.0
    }

    // ── Pitch ─────────────────────────────────────────────────────────────────

    /// Teeth per inch of pitch diameter (imperial): `DP = 25.4 / m`.
    ///
    /// The inch-unit analogue of module. A large DP means fine (small) teeth;
    /// a small DP means coarse (large) teeth — the opposite sense from module.
    /// Only relevant when interfacing with inch-unit gear catalogues.
    fn diametral_pitch(&self) -> f64 {
        crate::constants::MM_PER_INCH / self.normal_module()
    }

    // ── Backlash ──────────────────────────────────────────────────────────────

    /// Thinned tooth thickness after applying pair backlash `jt` in mm.
    ///
    /// The total transverse backlash `jt` is split equally between the two
    /// gears, thinning each by `jt / 2`. For spur gears: `s' = π·m/2 − jt/2`.
    /// For helical gears the projection also involves `cos(ψ)`:
    /// `sn' = π·mn/2 − (jt/2)·cos(ψ)`.
    ///
    /// Returns `None` if `backlash_mm < 0.0`.
    fn thinned_tooth_thickness(&self, backlash_mm: f64) -> Option<f64>;

    /// Normal backlash from transverse backlash `jt` in mm.
    ///
    /// The transverse gap `jt` is projected to the normal plane, which is
    /// where a feeler gauge actually measures it. For spur gears:
    /// `jn = jt · cos(α)`. For helical gears the helix angle adds another
    /// cosine: `jn = jt · cos(αt) · cos(ψ)`.
    ///
    /// Returns `None` if `backlash_mm < 0.0`.
    fn normal_backlash(&self, backlash_mm: f64) -> Option<f64>;

    // ── Pair operations (default implementations) ─────────────────────────────

    /// Speed ratio to `other`: `i = z_other / z_self`.
    ///
    /// Greater than 1 means `other` rotates slower (speed reduction).
    /// Less than 1 means `other` rotates faster (step-up).
    ///
    /// This default implementation is correct for all standard involute gears
    /// because the ratio depends only on tooth count.
    fn gear_ratio_to(&self, other: &Self) -> f64
    where
        Self: Sized,
    {
        other.teeth() as f64 / self.teeth() as f64
    }

    /// Centre distance to `other` in mm: `a = (d₁ + d₂) / 2`.
    ///
    /// The required shaft-to-shaft distance for the two pitch circles to be
    /// tangent. At this exact distance the gear ratio equals `z_other / z_self`
    /// and the tooth contact follows the standard involute path.
    ///
    /// This default implementation uses [`reference_diameter`] and is correct
    /// for both spur and helical gears on parallel shafts.
    ///
    /// [`reference_diameter`]: GearGeometry::reference_diameter
    fn center_distance_to(&self, other: &Self) -> f64
    where
        Self: Sized,
    {
        (self.reference_diameter() + other.reference_diameter()) / 2.0
    }
}
