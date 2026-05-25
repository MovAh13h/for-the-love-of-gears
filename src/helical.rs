//! Helical gears — teeth cut at an angle to the rotation axis.
//!
//! A helical gear is like a spur gear whose teeth are twisted along the shaft
//! axis by the **helix angle** `ψ`. Because each tooth enters contact gradually
//! rather than all at once, helical gears run quieter and smoother than spur
//! gears. The trade-off is an **axial thrust force** proportional to `tan(ψ)`,
//! which the shaft bearings must absorb.
//!
//! # Parameters
//!
//! | Parameter | Symbol | Unit | Default |
//! |---|---|---|---|
//! | Normal module | `mn` | mm | required |
//! | Number of teeth | `z` | — | required (min 3) |
//! | Normal pressure angle | `αn` | degrees | 20° ([`ISO_PRESSURE_ANGLE_DEG`]) |
//! | Helix angle | `ψ` | degrees | required (0°, 90°) |
//! | Hand | — | — | required |
//! | Face width | `b` | mm | optional |
//!
//! Typical helix angles are **15°–30°**. Below 15° the noise advantage over
//! spur gears is marginal. Above 30° the axial thrust force becomes large
//! enough that the bearing selection dominates the design.
//!
//! # Normal plane vs transverse plane
//!
//! Helical gears live in two planes that have different geometry values:
//!
//! - **Normal plane** (`n`): perpendicular to the tooth helix. The cutting tool
//!   works in this plane, so `mn` and `αn` are the *design* inputs. Tooth
//!   profile dimensions (addendum, dedendum, clearance, tooth thickness) are
//!   measured here.
//! - **Transverse plane** (`t`): perpendicular to the shaft axis — the plane of
//!   rotation. Pitch diameters and contact geometry live here.
//!
//! The two planes are related by:
//!
//! ```text
//! mt  = mn / cos(ψ)                     transverse module
//! tan(αt) = tan(αn) / cos(ψ)            transverse pressure angle
//! ```
//!
//! # Helix hand and meshing
//!
//! Two helical gears on parallel shafts must have **opposite hands** — a
//! right-hand gear meshes with a left-hand gear. They must also share the same
//! normal module, normal pressure angle, and helix angle magnitude. Crossed-axis
//! (skewed shaft) arrangements are outside the scope of this library.
//!
//! # Contact ratios
//!
//! Helical gears have **two** contact ratio components:
//!
//! - **Transverse contact ratio `εα`** — same concept as spur gears; computed
//!   in the transverse plane.
//! - **Overlap ratio `εβ`** — the extra contact from the helical tooth sweep
//!   along the face width: `εβ = b · sin(ψ) / (π · mn)`. This requires knowing
//!   the face width.
//! - **Total contact ratio `εγ = εα + εβ`** — the effective average number of
//!   tooth pairs in contact. Helical gears routinely achieve `εγ > 2.0`, which
//!   is why they run so smoothly.
//!
//! # Quick start
//!
//! ```
//! use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
//!
//! let gear = HelicalGear::builder()
//!     .module(2.0)
//!     .teeth(20)
//!     .helix_angle(20.0)
//!     .helix_hand(HelixHand::Right)
//!     .build()
//!     .unwrap();
//!
//! let mt = gear.transverse_module();
//! assert!((mt - 2.0 / 20.0_f64.to_radians().cos()).abs() < 1e-10);
//! ```
//!
//! [`ISO_PRESSURE_ANGLE_DEG`]: crate::constants::ISO_PRESSURE_ANGLE_DEG

use std::f64::consts::PI;
use std::fmt;

use crate::{
    constants::{
        ADDENDUM_COEFFICIENT, CLEARANCE_COEFFICIENT, DEDENDUM_COEFFICIENT, MIN_TEETH,
        MM_PER_INCH, WHOLE_DEPTH_COEFFICIENT,
    },
    traits::GearGeometry,
};

/// The ISO standard normal pressure angle, used when the caller does not specify one.
/// See [`crate::constants::ISO_PRESSURE_ANGLE_DEG`].
const DEFAULT_NORMAL_PRESSURE_ANGLE: f64 = crate::constants::ISO_PRESSURE_ANGLE_DEG;

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors returned by [`HelicalGearBuilder::build`].
#[derive(Debug, PartialEq)]
#[non_exhaustive]
pub enum HelicalGearError {
    /// `.module()` was not called on the builder.
    ModuleRequired,

    /// `.teeth()` was not called on the builder.
    TeethRequired,

    /// `.helix_angle()` was not called on the builder.
    HelixAngleRequired,

    /// `.helix_hand()` was not called on the builder.
    HelixHandRequired,

    /// Module must be strictly greater than zero.
    ///
    /// The normal module `mn` is the tooth-size parameter measured in the normal
    /// plane. Like the spur-gear module it must be a positive length in mm.
    ModuleMustBePositive,

    /// Tooth count must be at least 1.
    ///
    /// A gear with zero teeth cannot transmit motion.
    TeethMustBePositive,

    /// Tooth count must be at least [`MIN_TEETH`] (3).
    ///
    /// The root diameter `df = mt·z − 2.5·mn` must be positive. For any
    /// practical helix angle where `cos(ψ) < 1`, the transverse module
    /// `mt = mn / cos(ψ)` is larger than `mn`, which relaxes the limit
    /// slightly — but `z ≥ 3` is the safe, conservative minimum for all
    /// standard helix angles.
    ///
    /// [`MIN_TEETH`]: crate::constants::MIN_TEETH
    TeethTooFew,

    /// Helix angle must be strictly greater than zero degrees.
    ///
    /// A zero helix angle is a spur gear — use [`crate::gear::Gear`] for that
    /// case. The helix angle must be in the open interval `(0°, 90°)`.
    HelixAngleMustBePositive,

    /// Helix angle must be strictly less than 90 degrees.
    ///
    /// At exactly 90° the tooth helix runs parallel to the shaft axis, meaning
    /// the tooth never progresses across the face — a degenerate geometry that
    /// cannot transmit motion. In practice angles above ~35° already produce
    /// very large axial thrust forces.
    HelixAngleMustBeLessThan90,

    /// Normal pressure angle must be strictly greater than zero degrees.
    ///
    /// A zero normal pressure angle would produce a vertical tooth flank in the
    /// normal plane — one that can transmit no tangential force. Negative values
    /// have no physical meaning.
    PressureAngleMustBePositive,

    /// Face width must be strictly greater than zero.
    ///
    /// Face width is an optional parameter, but when provided it must be a
    /// positive length in mm. It is required for computing the overlap ratio
    /// `εβ` and total contact ratio `εγ`.
    FaceWidthMustBePositive,
}

impl std::error::Error for HelicalGearError {}

impl fmt::Display for HelicalGearError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ModuleRequired => write!(f, "module is required"),
            Self::TeethRequired => write!(f, "teeth count is required"),
            Self::HelixAngleRequired => write!(f, "helix angle is required"),
            Self::HelixHandRequired => write!(f, "helix hand is required"),
            Self::ModuleMustBePositive => write!(f, "module must be greater than zero"),
            Self::TeethMustBePositive => write!(f, "teeth count must be at least 1"),
            Self::TeethTooFew => write!(
                f,
                "teeth count must be at least 3 (fewer teeth produce a non-positive root diameter)"
            ),
            Self::HelixAngleMustBePositive => {
                write!(f, "helix angle must be greater than zero degrees")
            }
            Self::HelixAngleMustBeLessThan90 => {
                write!(f, "helix angle must be less than 90 degrees")
            }
            Self::PressureAngleMustBePositive => {
                write!(f, "pressure angle must be greater than zero degrees")
            }
            Self::FaceWidthMustBePositive => write!(f, "face width must be greater than zero"),
        }
    }
}

// ── Helix hand ────────────────────────────────────────────────────────────────

/// The winding direction of the tooth helix.
///
/// Hold a right-hand helical gear in front of you: the teeth rise from
/// lower-left to upper-right, like a right-handed screw. A left-hand gear
/// rises from lower-right to upper-left.
///
/// Two helical gears on parallel shafts must have **opposite hands** to mesh.
/// On crossed shafts (not supported by this library) the hands can be the same.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HelixHand {
    /// Teeth wind upward to the left (like a left-handed screw).
    Left,
    /// Teeth wind upward to the right (like a right-handed screw).
    Right,
}

impl HelixHand {
    fn opposite(self) -> Self {
        match self {
            Self::Left => Self::Right,
            Self::Right => Self::Left,
        }
    }
}

// ── Gear type ─────────────────────────────────────────────────────────────────

/// A fully defined helical gear.
///
/// All geometry methods return values in **millimetres** (or degrees /
/// dimensionless where noted). Build via [`HelicalGear::builder()`].
///
/// # Two modules, one gear
///
/// Helical gears have two module values that appear in different formulas:
///
/// - **Normal module `mn`** — measured perpendicular to the tooth helix. This
///   is the cutting-tool parameter and the design input. Tooth depth dimensions
///   (addendum, dedendum, clearance) all use `mn`.
/// - **Transverse module `mt = mn / cos(ψ)`** — measured in the rotation plane.
///   Because the helix "stretches" the apparent tooth pitch, `mt > mn`. Pitch
///   circle diameter uses `mt`: `d = mt · z`.
///
/// # Building a helical gear
///
/// ```
/// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
///
/// let gear = HelicalGear::builder()
///     .module(2.0)
///     .teeth(20)
///     .helix_angle(20.0)
///     .helix_hand(HelixHand::Right)
///     .build()
///     .unwrap();
///
/// let mt = gear.transverse_module();
/// let at = gear.transverse_pressure_angle();
/// ```
///
/// # Gear pairs
///
/// ```
/// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
///
/// let driver = HelicalGear::builder()
///     .module(2.0).teeth(20)
///     .helix_angle(20.0).helix_hand(HelixHand::Right)
///     .build().unwrap();
///
/// let driven = HelicalGear::builder()
///     .module(2.0).teeth(40)
///     .helix_angle(20.0).helix_hand(HelixHand::Left)
///     .build().unwrap();
///
/// assert!(driver.can_mesh_with(&driven));
/// assert_eq!(driver.gear_ratio_to(&driven), 2.0);
/// ```
///
/// # Backlash
///
/// Backlash `jt` is the circular gap in the **transverse plane**. When projected
/// into the **normal plane**, the thinning per gear is `(jt / 2) · cos(ψ)`:
///
/// ```text
///  thinned normal tooth thickness: sn' = π·mn / 2 − (jt / 2) · cos(ψ)
///  normal backlash:                jn  = jt · cos(αt) · cos(ψ)
/// ```
///
/// ```
/// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
/// use std::f64::consts::PI;
///
/// let g = HelicalGear::builder()
///     .module(2.0).teeth(20)
///     .helix_angle(20.0).helix_hand(HelixHand::Right)
///     .build().unwrap();
///
/// let jt = 0.08;
/// let s_prime = g.thinned_tooth_thickness(jt).unwrap();
/// let expected = PI * 2.0 / 2.0 - 0.04 * 20.0_f64.to_radians().cos();
/// assert!((s_prime - expected).abs() < 1e-10);
///
/// assert!(g.normal_backlash(jt).unwrap() < jt);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct HelicalGear {
    module: f64,
    teeth: u32,
    helix_angle: f64,
    helix_hand: HelixHand,
    normal_pressure_angle: f64,
    face_width: Option<f64>,
}

impl HelicalGear {
    /// Start building a [`HelicalGear`]. See [`HelicalGearBuilder`] for all options.
    pub fn builder() -> HelicalGearBuilder {
        HelicalGearBuilder::default()
    }

    // ── Inputs ────────────────────────────────────────────────────────────────

    /// Normal module `mn` (tooth size in the normal plane) in mm.
    ///
    /// The normal module is the design input and matches the cutting tool. It
    /// controls tooth height: addendum = `mn`, dedendum = 1.25·`mn`.
    /// Two helical gears must share the same normal module to mesh.
    ///
    /// See also [`transverse_module`] for the rotation-plane equivalent.
    ///
    /// [`transverse_module`]: HelicalGear::transverse_module
    pub fn normal_module(&self) -> f64 {
        self.module
    }

    /// Number of teeth.
    pub fn teeth(&self) -> u32 {
        self.teeth
    }

    /// Helix angle `ψ` in degrees.
    ///
    /// The angle between the tooth helix and a plane perpendicular to the shaft
    /// axis. A larger helix angle means:
    /// - **More overlap ratio** `εβ` — smoother, quieter operation
    /// - **Higher axial thrust** — larger bearing loads
    /// - **Stronger tooth** — more tooth material in cross-section
    ///
    /// Typical range: 15°–30°.
    pub fn helix_angle(&self) -> f64 {
        self.helix_angle
    }

    /// Winding direction of the helix.
    ///
    /// See [`HelixHand`] for the convention. Two gears on parallel shafts must
    /// have opposite hands.
    pub fn helix_hand(&self) -> HelixHand {
        self.helix_hand
    }

    /// Normal pressure angle `αn` in degrees.
    ///
    /// The pressure angle measured in the **normal plane** — the plane of the
    /// cutting tool. Defaults to [`ISO_PRESSURE_ANGLE_DEG`] (20°). Always
    /// smaller than the transverse pressure angle `αt` for ψ > 0.
    ///
    /// [`ISO_PRESSURE_ANGLE_DEG`]: crate::constants::ISO_PRESSURE_ANGLE_DEG
    pub fn normal_pressure_angle(&self) -> f64 {
        self.normal_pressure_angle
    }

    /// Face width `b` in millimetres, if provided.
    ///
    /// Face width is the axial length of the gear tooth. It is optional because
    /// many calculations do not need it. It is required for the overlap ratio
    /// `εβ` and total contact ratio `εγ`.
    pub fn face_width(&self) -> Option<f64> {
        self.face_width
    }

    // ── Derived — transverse plane ────────────────────────────────────────────

    /// Transverse module `mt` in mm: `mt = mn / cos(ψ)`.
    ///
    /// The helix stretches the apparent tooth pitch when viewed in the rotation
    /// plane, so `mt > mn` for any ψ > 0. The pitch circle diameter is computed
    /// in the transverse plane: `d = mt · z`.
    ///
    /// This is the reason a helical gear with the same `mn` and `z` as a spur
    /// gear will have a **larger pitch diameter** — the pitch circles are farther
    /// apart even though the teeth are the same size.
    pub fn transverse_module(&self) -> f64 {
        self.module / self.helix_angle.to_radians().cos()
    }

    /// Transverse pressure angle `αt` in degrees: `αt = atan(tan(αn) / cos(ψ))`.
    ///
    /// The pressure angle seen in the rotation plane. Always larger than `αn`
    /// because the helix projects the normal-plane angle into a steeper
    /// transverse-plane angle. The base circle diameter is computed using `αt`.
    pub fn transverse_pressure_angle(&self) -> f64 {
        let alpha_n = self.normal_pressure_angle.to_radians();
        let psi = self.helix_angle.to_radians();
        (alpha_n.tan() / psi.cos()).atan().to_degrees()
    }

    // ── Diameters (mm) ────────────────────────────────────────────────────────

    /// Pitch circle diameter in mm: `d = mt · z`.
    ///
    /// The transverse module `mt = mn / cos(ψ)` is larger than `mn`, so a
    /// helical gear with the same tooth count has a larger pitch diameter than
    /// an equivalent spur gear. The centre distance between two helical gears
    /// is `(d₁ + d₂) / 2`, using these transverse pitch circles.
    pub fn reference_diameter(&self) -> f64 {
        self.transverse_module() * self.teeth as f64
    }

    /// Tip (outer) diameter in mm: `da = mt·z + 2·mn`.
    ///
    /// Tooth height is governed by the **normal module** (addendum = `mn`),
    /// while the pitch circle is governed by the **transverse module**:
    /// `da = d + 2·ha = mt·z + 2·mn`.
    pub fn tip_diameter(&self) -> f64 {
        self.transverse_module() * self.teeth as f64 + 2.0 * ADDENDUM_COEFFICIENT * self.module
    }

    /// Root diameter in mm: `df = mt·z − 2.5·mn`.
    ///
    /// The dedendum depth `hf = 1.25·mn` is also in the normal module:
    /// `df = d − 2·hf = mt·z − 2.5·mn`.
    pub fn root_diameter(&self) -> f64 {
        self.transverse_module() * self.teeth as f64 - 2.0 * DEDENDUM_COEFFICIENT * self.module
    }

    /// Base circle diameter in mm: `db = d · cos(αt)`.
    ///
    /// The involute tooth profile is defined in the **transverse plane**, so the
    /// base circle uses the transverse pressure angle `αt`. The involute unrolls
    /// from this circle upward to the tip circle, forming the active tooth flank.
    pub fn base_diameter(&self) -> f64 {
        self.reference_diameter() * self.transverse_pressure_angle().to_radians().cos()
    }

    // ── Tooth profile (mm) ────────────────────────────────────────────────────

    /// Addendum in mm: `ha = mn`.
    ///
    /// Measured in the **normal plane** — the plane of the cutting tool. Equal
    /// to the normal module.
    pub fn addendum(&self) -> f64 {
        ADDENDUM_COEFFICIENT * self.module
    }

    /// Dedendum in mm: `hf = 1.25 · mn`.
    ///
    /// Measured in the **normal plane**. The extra 0.25·`mn` beyond the
    /// addendum is the clearance gap.
    pub fn dedendum(&self) -> f64 {
        DEDENDUM_COEFFICIENT * self.module
    }

    /// Full tooth height root-to-tip in mm: `h = 2.25 · mn`.
    ///
    /// Sum of addendum and dedendum, both measured in the normal plane.
    pub fn tooth_depth(&self) -> f64 {
        WHOLE_DEPTH_COEFFICIENT * self.module
    }

    /// Tip-to-root radial clearance in mm: `c = 0.25 · mn`.
    ///
    /// Same physical meaning as for spur gears — the gap between the tip of one
    /// gear and the root of its mate — but measured in the normal plane.
    pub fn clearance(&self) -> f64 {
        CLEARANCE_COEFFICIENT * self.module
    }

    /// Theoretical tooth thickness in the **normal plane** in mm: `sn = π · mn / 2`.
    ///
    /// On the pitch cylinder, the tooth and space are equal in the normal plane,
    /// so each occupies half the normal circular pitch. For the thinned value
    /// used in real pairs see [`thinned_tooth_thickness`].
    ///
    /// [`thinned_tooth_thickness`]: HelicalGear::thinned_tooth_thickness
    pub fn tooth_thickness(&self) -> f64 {
        PI * self.module / 2.0
    }

    // ── Pitch ─────────────────────────────────────────────────────────────────

    /// Arc length between teeth in the **normal plane** in mm: `pn = π · mn`.
    ///
    /// This is the pitch of the cutting tool and determines how teeth from
    /// different gears mesh in the normal plane. Two gears with the same `pn`
    /// (same `mn`) can mesh regardless of helix angle.
    pub fn normal_circular_pitch(&self) -> f64 {
        PI * self.module
    }

    /// Arc length between teeth in the **transverse plane** in mm: `pt = π · mt`.
    ///
    /// Always larger than the normal circular pitch because the helix stretches
    /// the apparent pitch in the rotation plane. This is the pitch that
    /// determines the base pitch used in the transverse contact ratio formula.
    pub fn transverse_circular_pitch(&self) -> f64 {
        PI * self.transverse_module()
    }

    /// Teeth per inch of pitch diameter (imperial): `DP = 25.4 / mn`.
    pub fn diametral_pitch(&self) -> f64 {
        MM_PER_INCH / self.module
    }

    /// Tooth spacing along the rotation axis in mm: `pa = π · mn / sin(ψ)`.
    ///
    /// The axial pitch is the distance along the shaft axis from one tooth to
    /// the next. Combined with the face width, it determines how many teeth are
    /// simultaneously in contact along the face (the overlap ratio).
    pub fn axial_pitch(&self) -> f64 {
        PI * self.module / self.helix_angle.to_radians().sin()
    }

    /// Axial distance for one full helix revolution in mm: `L = pa · z`.
    ///
    /// The lead is the distance a point on a tooth helix advances along the
    /// shaft axis in one full revolution of the gear. It is the axial pitch
    /// multiplied by the tooth count.
    pub fn lead(&self) -> f64 {
        self.axial_pitch() * self.teeth as f64
    }

    // ── Gear pair ─────────────────────────────────────────────────────────────

    /// `true` if this gear can mesh with `other` on parallel shafts.
    ///
    /// Four conditions must hold simultaneously:
    /// 1. **Equal normal modules** — the cutting tool pitch must match.
    /// 2. **Equal normal pressure angles** — the tooth flank angle in the normal
    ///    plane must match; different angles produce incompatible flanks.
    /// 3. **Equal helix angle magnitudes** — the tooth helices must be conjugate.
    /// 4. **Opposite helix hands** — one right-hand and one left-hand; same-hand
    ///    gears cannot be assembled on parallel shafts.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    ///
    /// let g1 = HelicalGear::builder()
    ///     .module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right)
    ///     .build().unwrap();
    ///
    /// let g2 = HelicalGear::builder()
    ///     .module(2.0).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left)
    ///     .build().unwrap();
    ///
    /// assert!(g1.can_mesh_with(&g2));
    ///
    /// // Same-hand gears do not mesh
    /// let g3 = HelicalGear::builder()
    ///     .module(2.0).teeth(30)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right)
    ///     .build().unwrap();
    /// assert!(!g1.can_mesh_with(&g3));
    ///
    /// // Different pressure angle gears do not mesh
    /// let g4 = HelicalGear::builder()
    ///     .module(2.0).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left)
    ///     .normal_pressure_angle(14.5)
    ///     .build().unwrap();
    /// assert!(!g1.can_mesh_with(&g4));
    /// ```
    pub fn can_mesh_with(&self, other: &HelicalGear) -> bool {
        let same_module = (self.module - other.module).abs() < crate::MESH_TOLERANCE;
        let same_pressure_angle = (self.normal_pressure_angle - other.normal_pressure_angle).abs()
            < crate::MESH_TOLERANCE;
        let same_angle = (self.helix_angle - other.helix_angle).abs() < crate::MESH_TOLERANCE;
        let opposite_hand = other.helix_hand == self.helix_hand.opposite();
        same_module && same_pressure_angle && same_angle && opposite_hand
    }

    /// Centre distance between the two gear axes in mm: `a = (d₁ + d₂) / 2`.
    ///
    /// Uses the transverse pitch circle diameters.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    /// let g1 = HelicalGear::builder().module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// let g2 = HelicalGear::builder().module(2.0).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).build().unwrap();
    /// let a = g1.center_distance_to(&g2);
    /// let expected = (g1.reference_diameter() + g2.reference_diameter()) / 2.0;
    /// assert!((a - expected).abs() < 1e-10);
    /// ```
    pub fn center_distance_to(&self, other: &HelicalGear) -> f64 {
        (self.reference_diameter() + other.reference_diameter()) / 2.0
    }

    /// Speed ratio to `other`: `i = z_other / z_self` (dimensionless).
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    /// let driver = HelicalGear::builder().module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// let driven = HelicalGear::builder().module(2.0).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).build().unwrap();
    /// assert_eq!(driver.gear_ratio_to(&driven), 2.0);
    /// assert_eq!(driven.gear_ratio_to(&driver), 0.5);
    /// ```
    pub fn gear_ratio_to(&self, other: &HelicalGear) -> f64 {
        other.teeth as f64 / self.teeth as f64
    }

    /// Transverse contact ratio `εα` between this gear and `other`.
    ///
    /// Computed in the transverse plane using the same path-of-contact formula
    /// as spur gears, but with transverse quantities (`mt`, `αt`, transverse
    /// pitch circle, etc.). For the total contact ratio including axial overlap,
    /// use [`total_contact_ratio_with`] (requires face width).
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    ///
    /// let g1 = HelicalGear::builder().module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// let g2 = HelicalGear::builder().module(2.0).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).build().unwrap();
    /// assert!(g1.transverse_contact_ratio_with(&g2) > 1.0);
    /// ```
    ///
    /// [`total_contact_ratio_with`]: HelicalGear::total_contact_ratio_with
    pub fn transverse_contact_ratio_with(&self, other: &HelicalGear) -> f64 {
        crate::contact_ratio::transverse(
            self.tip_diameter() / 2.0,
            self.base_diameter() / 2.0,
            other.tip_diameter() / 2.0,
            other.base_diameter() / 2.0,
            self.center_distance_to(other),
            self.transverse_pressure_angle(),
            self.transverse_module(),
        )
    }

    /// Overlap ratio `εβ = b · sin(ψ) / (π · mn)` (dimensionless).
    ///
    /// The overlap ratio is the extra contact coverage produced by the helical
    /// tooth sweeping across the face width. Intuitively: when `εβ = 1`, the
    /// helix completes exactly one tooth pitch across the face — meaning every
    /// point of the tooth profile is in contact at some axial position.
    ///
    /// Returns `None` if no face width was set — face width is required to
    /// compute the axial sweep.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    ///
    /// let g = HelicalGear::builder().module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right)
    ///     .face_width(30.0).build().unwrap();
    /// assert!(g.overlap_ratio().is_some());
    ///
    /// let g_no_fw = HelicalGear::builder().module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// assert!(g_no_fw.overlap_ratio().is_none());
    /// ```
    pub fn overlap_ratio(&self) -> Option<f64> {
        self.face_width
            .map(|b| crate::contact_ratio::overlap(b, self.helix_angle, self.module))
    }

    /// Total contact ratio `εγ = εα + εβ` (dimensionless).
    ///
    /// The total contact ratio is the sum of the transverse contact ratio and
    /// the overlap ratio. For well-designed helical gears `εγ > 2.0` is common,
    /// meaning more than two tooth pairs are in contact on average — this is
    /// the primary reason helical gears run quieter than spur gears.
    ///
    /// The overlap ratio `εβ` is bounded by the **shorter** of the two face
    /// widths: axial contact cannot extend beyond the narrower gear. This method
    /// therefore requires face widths on **both** gears and uses
    /// `min(b_self, b_other)` as the effective face width.
    ///
    /// Returns `None` if either gear is missing a face width.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    ///
    /// let g1 = HelicalGear::builder().module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).face_width(30.0).build().unwrap();
    /// let g2 = HelicalGear::builder().module(2.0).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).face_width(30.0).build().unwrap();
    ///
    /// // Equal face widths: εγ = εα + εβ (using the common width).
    /// let eg = g1.total_contact_ratio_with(&g2).unwrap();
    /// let ea = g1.transverse_contact_ratio_with(&g2);
    /// let eb = g1.overlap_ratio().unwrap();
    /// assert!((eg - (ea + eb)).abs() < 1e-10);
    ///
    /// // Unequal face widths: the narrower gear limits εβ.
    /// let g3 = HelicalGear::builder().module(2.0).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).face_width(10.0).build().unwrap();
    /// let eg_narrow = g1.total_contact_ratio_with(&g3).unwrap();
    /// assert!(eg_narrow < eg); // narrower wheel reduces total contact ratio
    /// ```
    pub fn total_contact_ratio_with(&self, other: &HelicalGear) -> Option<f64> {
        let b1 = self.face_width?;
        let b2 = other.face_width?;
        let b_eff = b1.min(b2);
        let eb = crate::contact_ratio::overlap(b_eff, self.helix_angle, self.module);
        Some(self.transverse_contact_ratio_with(other) + eb)
    }

    // ── Backlash ──────────────────────────────────────────────────────────────

    /// Normal-plane tooth thickness after applying transverse backlash `jt` (mm):
    /// `sn' = π·mn / 2 − (jt / 2) · cos(ψ)`.
    ///
    /// The transverse backlash `jt` lives in the rotation plane. Projecting it
    /// into the normal plane introduces `cos(ψ)`: `sn' = sn − (jt / 2) · cos(ψ)`.
    /// Each gear in a pair is thinned by half the total backlash.
    ///
    /// # Errors
    ///
    /// Returns [`BacklashError::NegativeBacklash`] if `backlash_mm < 0.0`.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    /// use std::f64::consts::PI;
    ///
    /// let g = HelicalGear::builder()
    ///     .module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right)
    ///     .build().unwrap();
    /// let s = g.thinned_tooth_thickness(0.08).unwrap();
    /// let expected = PI * 2.0 / 2.0 - 0.04 * 20.0_f64.to_radians().cos();
    /// assert!((s - expected).abs() < 1e-10);
    /// ```
    pub fn thinned_tooth_thickness(&self, backlash_mm: f64) -> Result<f64, crate::BacklashError> {
        if backlash_mm < 0.0 {
            return Err(crate::BacklashError::NegativeBacklash);
        }
        Ok(PI * self.module / 2.0 - backlash_mm / 2.0 * self.helix_angle.to_radians().cos())
    }

    /// Normal backlash from transverse backlash `jt` (mm):
    /// `jn = jt · cos(αt) · cos(ψ)`.
    ///
    /// The transverse gap `jt` is projected first through the transverse
    /// pressure angle and then through the helix angle to obtain the gap
    /// measured perpendicular to the tooth flank in the normal plane. `jn`
    /// is always smaller than `jt` — the projection shrinks it twice.
    ///
    /// # Errors
    ///
    /// Returns [`BacklashError::NegativeBacklash`] if `backlash_mm < 0.0`.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    ///
    /// let g = HelicalGear::builder()
    ///     .module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right)
    ///     .build().unwrap();
    /// assert!(g.normal_backlash(0.08).unwrap() < 0.08);
    /// ```
    pub fn normal_backlash(&self, backlash_mm: f64) -> Result<f64, crate::BacklashError> {
        if backlash_mm < 0.0 {
            return Err(crate::BacklashError::NegativeBacklash);
        }
        Ok(backlash_mm
            * self.transverse_pressure_angle().to_radians().cos()
            * self.helix_angle.to_radians().cos())
    }

    /// Returns `true` if this gear would be undercut when hobbed with a standard rack tool.
    ///
    /// For helical gears, undercutting is assessed on the **virtual (equivalent)
    /// spur gear** in the normal plane. The virtual tooth count is
    /// `z_v = z / cos³(ψ)` (Tregold's approximation). The gear undercuts if
    /// `z_v · sin²(αn) < 2`, where `αn` is the normal pressure angle.
    ///
    /// Because the helix increases the virtual tooth count, a helical gear with
    /// a given `z` undercuts at a lower threshold than an equivalent spur gear.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    ///
    /// // 14 teeth spur-equivalent would undercut, but helix saves it.
    /// let g = HelicalGear::builder()
    ///     .module(2.0).teeth(14)
    ///     .helix_angle(30.0).helix_hand(HelixHand::Right)
    ///     .build().unwrap();
    /// assert!(!g.is_undercut()); // z_v = 14/cos³(30°) ≈ 24.2 > 17
    /// ```
    pub fn is_undercut(&self) -> bool {
        let psi = self.helix_angle.to_radians();
        let cos_psi = psi.cos();
        let z_v = (self.teeth as f64) / (cos_psi * cos_psi * cos_psi);
        let sin_alpha_n = self.normal_pressure_angle.to_radians().sin();
        z_v * sin_alpha_n * sin_alpha_n < 2.0
    }

    /// Returns `true` if the tip of either gear extends past the base circle of
    /// the other in the transverse plane, causing involute interference.
    ///
    /// Helical gear interference is evaluated in the **transverse plane** using
    /// the transverse pressure angle `αt`:
    ///
    /// ```text
    /// approach limit:  a · sin(αt)
    /// gear-2 tip reach: √(ra2² − rb2²)
    /// gear-1 tip reach: √(ra1² − rb1²)
    ///
    /// interferes if either tip reach > approach limit
    /// ```
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    ///
    /// let pinion = HelicalGear::builder().module(2.0).teeth(12)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// let wheel = HelicalGear::builder().module(2.0).teeth(60)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).build().unwrap();
    /// assert!(pinion.interferes_with(&wheel));
    ///
    /// let g20 = HelicalGear::builder().module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// let g40 = HelicalGear::builder().module(2.0).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).build().unwrap();
    /// assert!(!g20.interferes_with(&g40));
    /// ```
    pub fn interferes_with(&self, other: &HelicalGear) -> bool {
        let alpha_t = self.transverse_pressure_angle().to_radians();
        let a = self.center_distance_to(other);
        let limit = a * alpha_t.sin();

        let ra1 = self.tip_diameter()   / 2.0;
        let rb1 = self.base_diameter()  / 2.0;
        let ra2 = other.tip_diameter()  / 2.0;
        let rb2 = other.base_diameter() / 2.0;

        let reach1 = if ra1 > rb1 { (ra1 * ra1 - rb1 * rb1).sqrt() } else { 0.0 };
        let reach2 = if ra2 > rb2 { (ra2 * ra2 - rb2 * rb2).sqrt() } else { 0.0 };

        reach1 > limit || reach2 > limit
    }
}

// ── GearGeometry trait impl ───────────────────────────────────────────────────

impl GearGeometry for HelicalGear {
    fn teeth(&self) -> u32 {
        self.teeth
    }

    fn normal_module(&self) -> f64 {
        self.module
    }

    fn reference_diameter(&self) -> f64 {
        self.reference_diameter()
    }

    fn tip_diameter(&self) -> f64 {
        self.tip_diameter()
    }

    fn root_diameter(&self) -> f64 {
        self.root_diameter()
    }

    fn base_diameter(&self) -> f64 {
        self.base_diameter()
    }

    fn addendum(&self) -> f64 {
        self.addendum()
    }

    fn dedendum(&self) -> f64 {
        self.dedendum()
    }

    fn tooth_depth(&self) -> f64 {
        self.tooth_depth()
    }

    fn clearance(&self) -> f64 {
        self.clearance()
    }

    fn tooth_thickness(&self) -> f64 {
        self.tooth_thickness()
    }

    fn diametral_pitch(&self) -> f64 {
        self.diametral_pitch()
    }

    fn thinned_tooth_thickness(&self, backlash_mm: f64) -> Result<f64, crate::BacklashError> {
        self.thinned_tooth_thickness(backlash_mm)
    }

    fn normal_backlash(&self, backlash_mm: f64) -> Result<f64, crate::BacklashError> {
        self.normal_backlash(backlash_mm)
    }
}

// ── Builder ───────────────────────────────────────────────────────────────────

/// Builder for [`HelicalGear`]. Obtain via [`HelicalGear::builder()`].
///
/// `module`, `teeth`, `helix_angle`, and `helix_hand` are required.
/// `normal_pressure_angle` defaults to [`ISO_PRESSURE_ANGLE_DEG`] (20°).
/// `face_width` is optional (needed for overlap and total contact ratios).
///
/// # Errors
///
/// [`build`] returns a [`HelicalGearError`] if any constraint is violated:
///
/// ```
/// use for_the_love_of_gears::helical::{HelicalGear, HelicalGearError, HelixHand};
///
/// assert_eq!(
///     HelicalGear::builder().module(2.0).teeth(20).helix_angle(20.0).build(),
///     Err(HelicalGearError::HelixHandRequired)
/// );
/// assert_eq!(
///     HelicalGear::builder().module(0.0).teeth(20).helix_angle(20.0)
///         .helix_hand(HelixHand::Right).build(),
///     Err(HelicalGearError::ModuleMustBePositive)
/// );
/// ```
///
/// [`ISO_PRESSURE_ANGLE_DEG`]: crate::constants::ISO_PRESSURE_ANGLE_DEG
/// [`build`]: HelicalGearBuilder::build
#[derive(Debug, Default)]
pub struct HelicalGearBuilder {
    module: Option<f64>,
    teeth: Option<u32>,
    helix_angle: Option<f64>,
    helix_hand: Option<HelixHand>,
    normal_pressure_angle: Option<f64>,
    face_width: Option<f64>,
}

impl HelicalGearBuilder {
    /// Set the normal module `mn` (tooth size in the normal plane) in mm.
    ///
    /// Must be positive. See [`HelicalGear::normal_module`].
    pub fn module(mut self, m: f64) -> Self {
        self.module = Some(m);
        self
    }

    /// Set the number of teeth.
    ///
    /// Must be at least [`MIN_TEETH`] (3). See [`HelicalGear::teeth`].
    ///
    /// [`MIN_TEETH`]: crate::constants::MIN_TEETH
    pub fn teeth(mut self, z: u32) -> Self {
        self.teeth = Some(z);
        self
    }

    /// Set the helix angle `ψ` in degrees. Must be in the open interval `(0°, 90°)`.
    ///
    /// `ψ = 0°` is a spur gear — use [`crate::gear::Gear`] for that case.
    pub fn helix_angle(mut self, degrees: f64) -> Self {
        self.helix_angle = Some(degrees);
        self
    }

    /// Set the helix hand.
    ///
    /// Two gears on parallel shafts must have opposite hands. See [`HelixHand`].
    pub fn helix_hand(mut self, hand: HelixHand) -> Self {
        self.helix_hand = Some(hand);
        self
    }

    /// Set the normal pressure angle `αn` in degrees.
    ///
    /// Defaults to [`ISO_PRESSURE_ANGLE_DEG`] (20°) if not called. Must be
    /// positive. See [`HelicalGear::normal_pressure_angle`].
    ///
    /// [`ISO_PRESSURE_ANGLE_DEG`]: crate::constants::ISO_PRESSURE_ANGLE_DEG
    pub fn normal_pressure_angle(mut self, degrees: f64) -> Self {
        self.normal_pressure_angle = Some(degrees);
        self
    }

    /// Set the face width in millimetres.
    ///
    /// Optional. Required for [`HelicalGear::overlap_ratio`] and
    /// [`HelicalGear::total_contact_ratio_with`]. Must be positive.
    pub fn face_width(mut self, mm: f64) -> Self {
        self.face_width = Some(mm);
        self
    }

    /// Build the [`HelicalGear`], validating all inputs.
    pub fn build(self) -> Result<HelicalGear, HelicalGearError> {
        let module = self.module.ok_or(HelicalGearError::ModuleRequired)?;
        let teeth = self.teeth.ok_or(HelicalGearError::TeethRequired)?;
        let helix_angle = self.helix_angle.ok_or(HelicalGearError::HelixAngleRequired)?;
        let helix_hand = self.helix_hand.ok_or(HelicalGearError::HelixHandRequired)?;

        if module <= 0.0 {
            return Err(HelicalGearError::ModuleMustBePositive);
        }
        if teeth == 0 {
            return Err(HelicalGearError::TeethMustBePositive);
        }
        if teeth < MIN_TEETH {
            return Err(HelicalGearError::TeethTooFew);
        }
        if helix_angle <= 0.0 {
            return Err(HelicalGearError::HelixAngleMustBePositive);
        }
        if helix_angle >= 90.0 {
            return Err(HelicalGearError::HelixAngleMustBeLessThan90);
        }

        let normal_pressure_angle =
            self.normal_pressure_angle.unwrap_or(DEFAULT_NORMAL_PRESSURE_ANGLE);
        if normal_pressure_angle <= 0.0 {
            return Err(HelicalGearError::PressureAngleMustBePositive);
        }

        if self.face_width.is_some_and(|fw| fw <= 0.0) {
            return Err(HelicalGearError::FaceWidthMustBePositive);
        }

        Ok(HelicalGear {
            module,
            teeth,
            helix_angle,
            helix_hand,
            normal_pressure_angle,
            face_width: self.face_width,
        })
    }
}
