//! Spur gears — teeth parallel to the rotation axis.
//!
//! A spur gear is the simplest form of involute gear: teeth run straight across
//! the face, parallel to the shaft. They are inexpensive to manufacture and
//! produce no axial (thrust) forces, at the cost of more noise at high speed
//! compared to helical gears.
//!
//! # Involute tooth profile
//!
//! Every spur gear in this library uses an **involute** profile. The involute of
//! a circle is the curve traced by the end of a taut string unwrapped from that
//! circle — the **base circle**. This geometric property ensures that, as two
//! involute gears rotate together, the contact point travels along a straight
//! line called the **line of action**, producing a constant velocity ratio
//! regardless of small centre-distance errors. No other tooth form has this
//! property.
//!
//! # Parameters
//!
//! A spur gear is fully defined by three values:
//!
//! | Parameter | Symbol | Unit | Default |
//! |---|---|---|---|
//! | Module | `m` | mm | required |
//! | Number of teeth | `z` | — | required (min 3) |
//! | Pressure angle | `α` | degrees | 20° ([`ISO_PRESSURE_ANGLE_DEG`]) |
//!
//! **Module** is the fundamental tooth-size parameter. It equals the pitch
//! circle diameter divided by the tooth count: `m = d / z`. Two gears can
//! only mesh if they share the same module — this is the single most important
//! compatibility constraint. All tooth dimensions are proportional to `m`.
//!
//! **Pressure angle** is the angle between the line of action and the common
//! tangent to the two pitch circles at the pitch point. It determines the
//! tooth flank angle and controls the balance between smooth operation (smaller
//! α) and load-carrying capacity (larger α). ISO standardises on 20°.
//!
//! # Quick start
//!
//! ```
//! use for_the_love_of_gears::gear::Gear;
//!
//! let gear = Gear::builder().module(2.0).teeth(20).build().unwrap();
//!
//! assert_eq!(gear.reference_diameter(), 40.0); // d = mz
//! assert_eq!(gear.addendum(), 2.0);             // ha = m
//! assert_eq!(gear.dedendum(), 2.5);             // hf = 1.25m
//! ```
//!
//! # Gear pairs
//!
//! ```
//! use for_the_love_of_gears::gear::Gear;
//!
//! let driver = Gear::builder().module(2.0).teeth(20).build().unwrap();
//! let driven = Gear::builder().module(2.0).teeth(40).build().unwrap();
//!
//! assert!(driver.can_mesh_with(&driven));
//! assert_eq!(driver.gear_ratio_to(&driven), 2.0);
//! assert_eq!(driver.center_distance_to(&driven), 60.0);
//! ```
//!
//! [`ISO_PRESSURE_ANGLE_DEG`]: crate::constants::ISO_PRESSURE_ANGLE_DEG

use std::{f64::consts::PI, fmt};

use crate::{
    constants::{
        ADDENDUM_COEFFICIENT, CLEARANCE_COEFFICIENT, DEDENDUM_COEFFICIENT, MIN_TEETH,
        MM_PER_INCH, WHOLE_DEPTH_COEFFICIENT,
    },
    traits::GearGeometry,
};

/// The ISO standard pressure angle in degrees, used when the caller does not
/// specify one. See [`crate::constants::ISO_PRESSURE_ANGLE_DEG`].
const DEFAULT_PRESSURE_ANGLE: f64 = crate::constants::ISO_PRESSURE_ANGLE_DEG;

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors returned by [`GearBuilder::build`].
#[derive(Debug, PartialEq)]
#[non_exhaustive]
pub enum GearError {
    /// `.module()` was not called on the builder.
    ModuleRequired,

    /// `.teeth()` was not called on the builder.
    TeethRequired,

    /// Module must be strictly greater than zero.
    ///
    /// A zero or negative module has no physical meaning — it would imply a
    /// gear with zero or inverted tooth size. The module is the ratio of pitch
    /// diameter to tooth count and must be a positive length in mm.
    ModuleMustBePositive,

    /// Tooth count must be at least 1.
    ///
    /// A gear with zero teeth cannot transmit motion.
    TeethMustBePositive,

    /// Tooth count must be at least [`MIN_TEETH`] (3).
    ///
    /// With the standard dedendum coefficient of 1.25, the root diameter
    /// formula `df = m(z − 2.5)` equals zero at `z = 2.5` and goes negative
    /// for `z = 1` or `z = 2`. A negative root diameter is geometrically
    /// invalid — the teeth would extend past the centre of the gear.
    ///
    /// [`MIN_TEETH`]: crate::constants::MIN_TEETH
    TeethTooFew,

    /// Pressure angle must be strictly greater than zero degrees.
    ///
    /// A zero pressure angle would produce a vertical tooth flank — one that
    /// transmits force purely radially with no tangential component. The gear
    /// could not drive a load. Negative values have no physical meaning.
    PressureAngleMustBePositive,
}

impl std::error::Error for GearError {}

impl fmt::Display for GearError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ModuleRequired => write!(f, "module is required"),
            Self::TeethRequired => write!(f, "teeth count is required"),
            Self::ModuleMustBePositive => write!(f, "module must be greater than zero"),
            Self::TeethMustBePositive => write!(f, "teeth count must be at least 1"),
            Self::TeethTooFew => write!(
                f,
                "teeth count must be at least 3 (fewer teeth produce a non-positive root diameter)"
            ),
            Self::PressureAngleMustBePositive => {
                write!(f, "pressure angle must be greater than zero degrees")
            }
        }
    }
}

// ── Gear type ─────────────────────────────────────────────────────────────────

/// A fully defined spur gear.
///
/// All geometry methods return values in **millimetres** (or degrees /
/// dimensionless where noted). Build via [`Gear::builder()`].
///
/// # Dimension overview
///
/// ```text
///            ┌──── tip circle (da) ────┐
///        ┌───┤                         ├───┐
///       /    │   ┌─── pitch circle (d)    │   \
///      │     │   │                    │   │    │  ← addendum (ha = m)
///      │     ╔═══╧════════════════════╧═══╗    │
///      │     ║     tooth cross-section    ║    │
///      │     ╚═══╤════════════════════╤═══╝    │
///      │     │   │                    │   │    │  ← dedendum (hf = 1.25m)
///       \    │   └─── root circle (df)    │   /
///        └───┤                         ├───┘
///            └──── root circle (df) ───┘
/// ```
///
/// # Example
///
/// ```
/// use for_the_love_of_gears::gear::Gear;
///
/// let g = Gear::builder().module(2.0).teeth(20).build().unwrap();
///
/// assert_eq!(g.reference_diameter(), 40.0); // d  = mz
/// assert_eq!(g.tip_diameter(),       44.0); // da = m(z+2)
/// assert_eq!(g.root_diameter(),      35.0); // df = m(z-2.5)
/// assert_eq!(g.addendum(),    2.0);  // ha = m
/// assert_eq!(g.dedendum(),    2.5);  // hf = 1.25m
/// assert_eq!(g.tooth_depth(), 4.5);  // h  = 2.25m
/// assert_eq!(g.clearance(),   0.5);  // c  = 0.25m
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Gear {
    module: f64,
    teeth: u32,
    pressure_angle: f64,
}

impl Gear {
    /// Start building a [`Gear`].
    pub fn builder() -> GearBuilder {
        GearBuilder::default()
    }

    // ── Inputs ────────────────────────────────────────────────────────────────

    /// Module (tooth size) in mm.
    ///
    /// Module is defined as `m = d / z` — the pitch circle diameter divided by
    /// the tooth count. It controls the physical size of every tooth:
    /// doubling the module doubles the tooth height, pitch, and all diameters.
    ///
    /// Common ISO module values: 1, 1.25, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10, 12,
    /// 16, 20 (ISO 54 series). Two gears must share the same module to mesh.
    pub fn module(&self) -> f64 {
        self.module
    }

    /// Normal module in mm — alias for [`module`] for symmetry with `HelicalGear`.
    ///
    /// For spur gears there is only one module (the normal and transverse planes
    /// coincide). Use this name when writing code that is generic over both
    /// spur and helical gears.
    ///
    /// [`module`]: Gear::module
    pub fn normal_module(&self) -> f64 {
        self.module
    }

    /// Number of teeth.
    ///
    /// Together with the module, the tooth count fully determines the pitch
    /// circle diameter: `d = m · z`. The gear ratio between two meshing gears
    /// is simply the ratio of their tooth counts.
    pub fn teeth(&self) -> u32 {
        self.teeth
    }

    /// Pressure angle `α` in degrees.
    ///
    /// Default is [`DEFAULT_PRESSURE_ANGLE`] (20°, per ISO 21771). The pressure
    /// angle is the angle between the tooth normal force and the tangent to the
    /// pitch circle. A larger angle means:
    /// - **Stronger teeth** — more material in the root cross-section
    /// - **Higher radial load** on the bearings — force has a larger radial component
    /// - **Less risk of undercutting** on small tooth counts
    pub fn pressure_angle(&self) -> f64 {
        self.pressure_angle
    }

    /// Normal pressure angle in degrees — alias for [`pressure_angle`] for symmetry with `HelicalGear`.
    ///
    /// [`pressure_angle`]: Gear::pressure_angle
    pub fn normal_pressure_angle(&self) -> f64 {
        self.pressure_angle
    }

    // ── Diameters (mm) ────────────────────────────────────────────────────────

    /// Pitch circle diameter in mm: `d = m · z`.
    ///
    /// The pitch circle is the reference circle from which all tooth proportions
    /// are measured. When two gears mesh, their pitch circles are tangent at the
    /// **pitch point** — the contact point on the line of centres. The centre
    /// distance between two gears equals `(d₁ + d₂) / 2`.
    pub fn reference_diameter(&self) -> f64 {
        self.module * self.teeth as f64
    }

    /// Tip (outer) diameter in mm: `da = m(z + 2)`.
    ///
    /// The tip circle bounds the gear tooth from the outside. It equals the
    /// pitch diameter plus two addenda: `da = d + 2·ha = m·z + 2·m`.
    /// The addendum height of `1·m` above the pitch circle is set by the ISO
    /// standard to ensure adequate contact ratio with a mating gear.
    pub fn tip_diameter(&self) -> f64 {
        self.module * (self.teeth as f64 + 2.0 * ADDENDUM_COEFFICIENT)
    }

    /// Root diameter in mm: `df = m(z − 2.5)`.
    ///
    /// The root circle is where the tooth meets the gear body. It lies
    /// `hf = 1.25·m` below the pitch circle — one addendum plus the clearance
    /// gap: `df = d − 2·hf = m·z − 2.5·m`. The root diameter must be positive,
    /// which is why [`MIN_TEETH`] = 3 is enforced by the builder.
    ///
    /// **Practical note:** For very small tooth counts (z = 3–5) the base circle
    /// may extend below the root circle, meaning the involute profile does not
    /// reach all the way down. Undercutting is a manufacturing concern in those
    /// cases.
    ///
    /// [`MIN_TEETH`]: crate::constants::MIN_TEETH
    pub fn root_diameter(&self) -> f64 {
        self.module * (self.teeth as f64 - 2.0 * DEDENDUM_COEFFICIENT)
    }

    /// Base circle diameter in mm: `db = d · cos(α)`.
    ///
    /// The base circle is where the involute tooth profile originates. The
    /// involute curve is traced by a point on a taut string unwrapping from
    /// this circle. The size of the base circle relative to the pitch circle is
    /// controlled entirely by the pressure angle: `db = d · cos(α)`. A larger
    /// pressure angle produces a smaller base circle, and therefore a steeper
    /// tooth flank.
    ///
    /// **Important:** if `db > df` (base circle larger than root circle), part
    /// of the root is not an involute — it becomes a straight radial fillet cut
    /// by the tool. This is normal for small gears (z < ~17 at 20°) and does
    /// not prevent meshing, but it does affect the effective contact ratio.
    pub fn base_diameter(&self) -> f64 {
        self.reference_diameter() * self.pressure_angle.to_radians().cos()
    }

    // ── Tooth profile (mm) ────────────────────────────────────────────────────

    /// Addendum — radial height above pitch circle in mm: `ha = m`.
    ///
    /// The addendum is exactly one module. This means the tip circle is always
    /// one module above the pitch circle: `ra_tip = d/2 + m`. The ISO standard
    /// sets `ha = m` (addendum coefficient = 1) so that standard rack cutters
    /// and gear blanks are interchangeable across manufacturers.
    pub fn addendum(&self) -> f64 {
        ADDENDUM_COEFFICIENT * self.module
    }

    /// Dedendum — radial depth below pitch circle in mm: `hf = 1.25 · m`.
    ///
    /// The dedendum is 1.25 modules — one module of working depth (to match the
    /// mating gear's addendum) plus 0.25 modules of **clearance**. The extra
    /// 0.25·m ensures the tip of the mating gear never contacts the root, even
    /// accounting for manufacturing tolerances and thermal growth.
    pub fn dedendum(&self) -> f64 {
        DEDENDUM_COEFFICIENT * self.module
    }

    /// Full tooth height root-to-tip in mm: `h = 2.25 · m`.
    ///
    /// This is the sum of addendum and dedendum: `h = ha + hf = m + 1.25·m`.
    /// All standard involute gears have this tooth height regardless of tooth
    /// count, so a module-2 gear always has 4.5 mm deep teeth whether it has
    /// 20 or 200 teeth.
    pub fn tooth_depth(&self) -> f64 {
        WHOLE_DEPTH_COEFFICIENT * self.module
    }

    /// Tip-to-root radial clearance in mm: `c = 0.25 · m`.
    ///
    /// Clearance is the gap between the tip of one gear and the root of its
    /// mate: `c = hf − ha = 0.25·m`. It serves three purposes:
    /// 1. Prevents tip-to-root jamming from thermal expansion
    /// 2. Provides a channel for lubricant to reach the contact zone
    /// 3. Accommodates the root-fillet radius without interference
    pub fn clearance(&self) -> f64 {
        CLEARANCE_COEFFICIENT * self.module
    }

    /// Theoretical tooth thickness along pitch circle in mm: `s = π · m / 2`.
    ///
    /// On a standard gear, the tooth and space are equal on the pitch circle, so
    /// each occupies half the circular pitch: `s = p / 2 = π·m / 2`. Real gears
    /// have a slightly thinned tooth to create backlash — see
    /// [`thinned_tooth_thickness`].
    ///
    /// [`thinned_tooth_thickness`]: Gear::thinned_tooth_thickness
    pub fn tooth_thickness(&self) -> f64 {
        PI * self.module / 2.0
    }

    // ── Pitch ─────────────────────────────────────────────────────────────────

    /// Arc length between adjacent teeth along the pitch circle in mm: `p = π · m`.
    ///
    /// The pitch circle has circumference `π · d = π · m · z`. Dividing by `z`
    /// gives `p = π · m` per tooth. Two gears can only mesh if their pitches
    /// are equal — which is equivalent to requiring equal modules.
    pub fn circular_pitch(&self) -> f64 {
        PI * self.module
    }

    /// Normal circular pitch in mm — alias for [`circular_pitch`] for symmetry with `HelicalGear`.
    ///
    /// [`circular_pitch`]: Gear::circular_pitch
    pub fn normal_circular_pitch(&self) -> f64 {
        self.circular_pitch()
    }

    /// Teeth per inch of pitch diameter (imperial): `DP = 25.4 / m`.
    ///
    /// Diametral pitch is the inch-unit analogue of module. It counts the number
    /// of teeth per inch of pitch diameter. A large DP means fine (small) teeth;
    /// a small DP means coarse (large) teeth — the opposite of module. Common
    /// DP values: 4, 6, 8, 10, 12, 16, 20, 24, 32, 48.
    ///
    /// Only relevant when interfacing with inch-unit systems. For SI work, use
    /// [`module`].
    ///
    /// [`module`]: Gear::module
    pub fn diametral_pitch(&self) -> f64 {
        MM_PER_INCH / self.module
    }

    // ── Gear pair ─────────────────────────────────────────────────────────────

    /// `true` if this gear can mesh with `other` (same module and pressure angle,
    /// within floating-point tolerance).
    ///
    /// Two spur gears mesh correctly only when:
    /// - Their **modules are equal** — otherwise tooth pitch does not match and
    ///   the teeth will jam or skip.
    /// - Their **pressure angles are equal** — otherwise the tooth flanks have
    ///   different angles, preventing smooth rolling contact along the line of
    ///   action.
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    ///
    /// let g1 = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// let g2 = Gear::builder().module(2.0).teeth(40).build().unwrap();
    /// let g3 = Gear::builder().module(3.0).teeth(20).build().unwrap();
    /// let g4 = Gear::builder().module(2.0).teeth(20).pressure_angle(14.5).build().unwrap();
    ///
    /// assert!(g1.can_mesh_with(&g2));
    /// assert!(!g1.can_mesh_with(&g3)); // different module
    /// assert!(!g1.can_mesh_with(&g4)); // different pressure angle
    /// ```
    pub fn can_mesh_with(&self, other: &Gear) -> bool {
        (self.module - other.module).abs() < crate::MESH_TOLERANCE
            && (self.pressure_angle - other.pressure_angle).abs() < crate::MESH_TOLERANCE
    }

    /// Centre distance in mm: `a = (d₁ + d₂) / 2`.
    ///
    /// This is the required shaft-to-shaft distance for the two gears to mesh
    /// at their standard pitch circles. At this distance the pitch circles are
    /// tangent and the gear ratio equals exactly `z₂ / z₁`. Mounting the gears
    /// closer increases backlash interference; mounting them farther apart
    /// increases backlash — both degrade tooth contact.
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    ///
    /// let g1 = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// let g2 = Gear::builder().module(2.0).teeth(40).build().unwrap();
    /// assert_eq!(g1.center_distance_to(&g2), 60.0);
    /// ```
    pub fn center_distance_to(&self, other: &Gear) -> f64 {
        (self.reference_diameter() + other.reference_diameter()) / 2.0
    }

    /// Speed ratio to `other`: `i = z_other / z_self` (dimensionless).
    ///
    /// Greater than 1 means `other` rotates slower than `self` (speed
    /// reduction, torque multiplication). Less than 1 means `other` rotates
    /// faster (speed increase, torque reduction).
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    ///
    /// let driver = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// let driven = Gear::builder().module(2.0).teeth(40).build().unwrap();
    ///
    /// assert_eq!(driver.gear_ratio_to(&driven), 2.0);
    /// assert_eq!(driven.gear_ratio_to(&driver), 0.5);
    /// ```
    pub fn gear_ratio_to(&self, other: &Gear) -> f64 {
        other.teeth as f64 / self.teeth as f64
    }

    /// Transverse contact ratio `εα` with `other` (dimensionless).
    ///
    /// The contact ratio is the average number of tooth pairs simultaneously
    /// in contact. It is computed from the **path of contact** — the arc along
    /// which the teeth actually touch — divided by the base pitch `pb = π·m·cos(α)`.
    ///
    /// | Range | Interpretation |
    /// |---|---|
    /// | < 1.2 | Avoid — noisy, high impact loads |
    /// | 1.2 – 1.4 | Minimum for general machinery |
    /// | 1.4 – 1.8 | Good industrial practice |
    /// | > 2.0 | Achievable with many teeth or large addenda |
    ///
    /// A value of 1.6, for example, means the gear pair spends 60% of the time
    /// with two pairs of teeth in contact and 40% with only one pair — the load
    /// is shared most of the time.
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    ///
    /// let g1 = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// let g2 = Gear::builder().module(2.0).teeth(40).build().unwrap();
    /// assert!((g1.contact_ratio_with(&g2) - 1.635).abs() < 0.001);
    /// ```
    pub fn contact_ratio_with(&self, other: &Gear) -> f64 {
        crate::contact_ratio::spur(
            self.tip_diameter() / 2.0,
            self.base_diameter() / 2.0,
            other.tip_diameter() / 2.0,
            other.base_diameter() / 2.0,
            self.center_distance_to(other),
            self.pressure_angle,
            self.module,
        )
    }

    /// Transverse contact ratio `εα` with `other` — alias for [`contact_ratio_with`] for
    /// symmetry with `HelicalGear::transverse_contact_ratio_with`.
    ///
    /// [`contact_ratio_with`]: Gear::contact_ratio_with
    pub fn transverse_contact_ratio_with(&self, other: &Gear) -> f64 {
        self.contact_ratio_with(other)
    }

    // ── Backlash ──────────────────────────────────────────────────────────────

    /// Thinned tooth thickness after applying pair backlash `jt` (mm):
    /// `s' = π·m / 2 − jt / 2`.
    ///
    /// **Backlash** is the intentional gap between mating tooth flanks when the
    /// drive flank is in contact. It prevents jamming from thermal expansion and
    /// allows a lubricant film to form. Backlash is a **pair property**: the
    /// total gap `jt` is split equally, thinning each gear by `jt / 2`.
    ///
    /// Typical values: 0.05–0.15 mm for precision gearboxes; up to 0.5 mm for
    /// coarse industrial drives.
    ///
    /// # Errors
    ///
    /// Returns [`BacklashError::NegativeBacklash`] if `backlash_mm < 0.0`.
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    /// use std::f64::consts::PI;
    ///
    /// let g = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// let s = g.thinned_tooth_thickness(0.08).unwrap();
    /// assert!((s - (PI - 0.04)).abs() < 1e-10);
    /// ```
    pub fn thinned_tooth_thickness(&self, backlash_mm: f64) -> Result<f64, crate::BacklashError> {
        if backlash_mm < 0.0 {
            return Err(crate::BacklashError::NegativeBacklash);
        }
        Ok(PI * self.module / 2.0 - backlash_mm / 2.0)
    }

    /// Normal backlash from transverse backlash `jt` (mm): `jn = jt · cos(α)`.
    ///
    /// The **transverse backlash** `jt` is the gap measured along the pitch
    /// circle (an arc length). The **normal backlash** `jn` is what a feeler
    /// gauge reads when inserted perpendicular to the tooth flank — it is
    /// smaller than `jt` by the cosine of the pressure angle.
    ///
    /// Normal backlash is used in inspection because it can be measured directly
    /// with a feeler gauge at any point on the tooth face.
    ///
    /// # Errors
    ///
    /// Returns [`BacklashError::NegativeBacklash`] if `backlash_mm < 0.0`.
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    ///
    /// let g = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// let jn = g.normal_backlash(0.08).unwrap();
    /// let expected = 0.08 * 20.0_f64.to_radians().cos();
    /// assert!((jn - expected).abs() < 1e-10);
    /// ```
    pub fn normal_backlash(&self, backlash_mm: f64) -> Result<f64, crate::BacklashError> {
        if backlash_mm < 0.0 {
            return Err(crate::BacklashError::NegativeBacklash);
        }
        Ok(backlash_mm * self.pressure_angle.to_radians().cos())
    }

    /// Returns `true` if this gear would be undercut when hobbed with a standard rack tool.
    ///
    /// Undercutting occurs when the tooth count is below the minimum for the
    /// pressure angle: `z_min = 2 / sin²(α)`. For the standard 20° angle this
    /// gives `z_min ≈ 17`. Undercut teeth are weaker (the root fillet encroaches
    /// on the involute) and may cause interference with the mating gear.
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    ///
    /// let g16 = Gear::builder().module(2.0).teeth(16).build().unwrap();
    /// let g20 = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// assert!(g16.is_undercut());
    /// assert!(!g20.is_undercut());
    /// ```
    pub fn is_undercut(&self) -> bool {
        let sin_alpha = self.pressure_angle.to_radians().sin();
        (self.teeth as f64) * sin_alpha * sin_alpha < 2.0
    }

    /// Returns `true` if the tip of either gear in the pair extends past the
    /// base circle of the other, causing involute interference.
    ///
    /// Interference means the tip of one gear contacts the flank of the other
    /// below the base circle, where the involute does not exist. The test is:
    ///
    /// ```text
    /// approach limit:  a · sin(α)
    /// gear-2 tip reach: √(ra2² − rb2²)
    /// gear-1 tip reach: √(ra1² − rb1²)
    ///
    /// interferes if either tip reach > approach limit
    /// ```
    ///
    /// This requires both gears to have the same module and pressure angle
    /// (i.e., `self.can_mesh_with(other)` is true).
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    ///
    /// let pinion = Gear::builder().module(2.0).teeth(12).build().unwrap();
    /// let wheel  = Gear::builder().module(2.0).teeth(60).build().unwrap();
    /// // Large ratio: pinion is small enough to interfere into the wheel root.
    /// assert!(pinion.interferes_with(&wheel));
    ///
    /// let g20 = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// let g40 = Gear::builder().module(2.0).teeth(40).build().unwrap();
    /// assert!(!g20.interferes_with(&g40));
    /// ```
    pub fn interferes_with(&self, other: &Gear) -> bool {
        let alpha = self.pressure_angle.to_radians();
        let a = self.center_distance_to(other);
        let limit = a * alpha.sin();

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

impl GearGeometry for Gear {
    fn teeth(&self) -> u32 {
        self.teeth
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

/// Builder for [`Gear`]. Obtain via [`Gear::builder()`].
///
/// `module` and `teeth` are required. `pressure_angle` defaults to
/// [`ISO_PRESSURE_ANGLE_DEG`] (20°).
///
/// # Errors
///
/// [`build`] returns a [`GearError`] if any constraint is violated:
///
/// ```
/// use for_the_love_of_gears::gear::{Gear, GearError};
///
/// assert_eq!(
///     Gear::builder().module(2.0).build(),
///     Err(GearError::TeethRequired)
/// );
/// assert_eq!(
///     Gear::builder().module(0.0).teeth(20).build(),
///     Err(GearError::ModuleMustBePositive)
/// );
/// assert_eq!(
///     Gear::builder().module(2.0).teeth(2).build(),
///     Err(GearError::TeethTooFew)
/// );
/// ```
///
/// [`ISO_PRESSURE_ANGLE_DEG`]: crate::constants::ISO_PRESSURE_ANGLE_DEG
/// [`build`]: GearBuilder::build
#[derive(Debug, Default)]
pub struct GearBuilder {
    module: Option<f64>,
    teeth: Option<u32>,
    pressure_angle: Option<f64>,
}

impl GearBuilder {
    /// Set the module (tooth size) in mm.
    ///
    /// See [`Gear::module`] for a full description. Must be positive.
    pub fn module(mut self, m: f64) -> Self {
        self.module = Some(m);
        self
    }

    /// Set the number of teeth.
    ///
    /// Must be at least [`MIN_TEETH`] (3). See [`Gear::teeth`].
    ///
    /// [`MIN_TEETH`]: crate::constants::MIN_TEETH
    pub fn teeth(mut self, z: u32) -> Self {
        self.teeth = Some(z);
        self
    }

    /// Set the pressure angle in degrees.
    ///
    /// Defaults to [`ISO_PRESSURE_ANGLE_DEG`] (20°) if not called. Must be
    /// positive. See [`Gear::pressure_angle`] for the physical meaning.
    ///
    /// [`ISO_PRESSURE_ANGLE_DEG`]: crate::constants::ISO_PRESSURE_ANGLE_DEG
    pub fn pressure_angle(mut self, degrees: f64) -> Self {
        self.pressure_angle = Some(degrees);
        self
    }

    /// Build the [`Gear`], validating all inputs.
    pub fn build(self) -> Result<Gear, GearError> {
        let module = self.module.ok_or(GearError::ModuleRequired)?;
        let teeth = self.teeth.ok_or(GearError::TeethRequired)?;

        if module <= 0.0 {
            return Err(GearError::ModuleMustBePositive);
        }
        if teeth == 0 {
            return Err(GearError::TeethMustBePositive);
        }
        if teeth < MIN_TEETH {
            return Err(GearError::TeethTooFew);
        }

        let pressure_angle = self.pressure_angle.unwrap_or(DEFAULT_PRESSURE_ANGLE);
        if pressure_angle <= 0.0 {
            return Err(GearError::PressureAngleMustBePositive);
        }

        Ok(Gear { module, teeth, pressure_angle })
    }
}
