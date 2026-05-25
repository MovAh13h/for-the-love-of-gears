//! Helical gears — teeth cut at an angle to the rotation axis.
//!
//! A helical gear is like a spur gear whose teeth are twisted along the shaft
//! axis by the **helix angle** `ψ`. This twist means several teeth share the
//! load at once, producing smoother and quieter operation than spur gears at
//! the cost of an axial thrust force.
//!
//! # Key parameters
//!
//! | Parameter | Symbol | Unit | Note |
//! |---|---|---|---|
//! | Normal module | `mn` | mm | Tooth size in the normal plane; determines tooling |
//! | Number of teeth | `z` | — | Integer count |
//! | Normal pressure angle | `αn` | degrees | 20° standard |
//! | Helix angle | `ψ` | degrees | Typically 15–30° |
//! | Hand | — | — | Right-hand (RH) or left-hand (LH) |
//!
//! Mating helical gears must have the same normal module, same normal pressure
//! angle, equal helix angles, and **opposite hands** (one RH, one LH).
//!
//! # Normal plane vs transverse plane
//!
//! Helical gears have two sets of measurements:
//! - **Normal plane** (`n`): perpendicular to the tooth flank helix. This is
//!   the plane of the cutting tool and where `mn` and `αn` are defined.
//! - **Transverse plane** (`t`): perpendicular to the shaft axis. This is
//!   where `mt`, `αt`, and the pitch diameter live.
//!
//! ```text
//! mt = mn / cos(ψ)
//! tan(αt) = tan(αn) / cos(ψ)
//! ```
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

use std::f64::consts::PI;
use std::fmt;

/// The standard normal pressure angle for helical gears in degrees.
const DEFAULT_NORMAL_PRESSURE_ANGLE: f64 = 20.0;

/// Errors returned by [`HelicalGearBuilder::build`].
#[derive(Debug, PartialEq)]
#[non_exhaustive]
pub enum HelicalGearError {
    /// `.module()` was not called.
    ModuleRequired,
    /// `.teeth()` was not called.
    TeethRequired,
    /// `.helix_angle()` was not called.
    HelixAngleRequired,
    /// `.helix_hand()` was not called.
    HelixHandRequired,
    /// Module must be greater than zero.
    ModuleMustBePositive,
    /// Tooth count must be at least 1.
    TeethMustBePositive,
    /// Tooth count must be at least 3 to produce a positive root diameter.
    ///
    /// With fewer than 3 teeth the dedendum exceeds the pitch radius and
    /// `df = mt·z − 2.5·mn` becomes zero or negative for any practical helix angle.
    TeethTooFew,
    /// Helix angle must be greater than zero degrees.
    HelixAngleMustBePositive,
    /// Helix angle must be less than 90 degrees.
    HelixAngleMustBeLessThan90,
    /// Pressure angle must be greater than zero degrees.
    PressureAngleMustBePositive,
    /// Face width must be greater than zero.
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

/// The winding direction of the tooth helix.
///
/// Two helical gears on parallel shafts must have opposite hands to mesh —
/// a left-hand gear meshes only with a right-hand gear. See [`HelicalGear::can_mesh_with`].
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

/// A fully defined helical gear.
///
/// All geometry methods return values in **millimetres** (or degrees / dimensionless
/// where noted). Build with [`HelicalGear::builder()`].
///
/// # Two modules, one gear
///
/// Helical gears have two module values:
///
/// - **Normal module** `mn` — measured perpendicular to the tooth helix. This is
///   the design input and matches the cutting tool.
/// - **Transverse module** `mt = mn / cos(ψ)` — measured in the rotation plane.
///   This determines the pitch circle diameter: `d = mt · z`.
///
/// Tooth profile dimensions (addendum, dedendum, depth, clearance) all use `mn`.
/// Pitch circle and diameter dimensions use `mt`.
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
///  thinned normal tooth thickness: sn' = πmn / 2 − (jt / 2) · cos(ψ)
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
/// let s_prime = g.thinned_tooth_thickness(jt);
/// let expected = PI * 2.0 / 2.0 - 0.04 * 20.0_f64.to_radians().cos();
/// assert!((s_prime - expected).abs() < 1e-10);
///
/// assert!(g.normal_backlash(jt) < jt);
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
    pub fn normal_module(&self) -> f64 {
        self.module
    }

    /// Number of teeth.
    pub fn teeth(&self) -> u32 {
        self.teeth
    }

    /// Helix angle `ψ` in degrees.
    pub fn helix_angle(&self) -> f64 {
        self.helix_angle
    }

    /// Winding direction of the helix.
    pub fn helix_hand(&self) -> HelixHand {
        self.helix_hand
    }

    /// Normal pressure angle `αn` in degrees.
    pub fn normal_pressure_angle(&self) -> f64 {
        self.normal_pressure_angle
    }

    /// Face width in millimetres, if provided.
    pub fn face_width(&self) -> Option<f64> {
        self.face_width
    }

    // ── Derived — transverse plane ────────────────────────────────────────────

    /// Transverse module in mm: `mt = mn / cos(ψ)`.
    ///
    /// Always larger than the normal module — the helix stretches the apparent
    /// tooth pitch when viewed in the rotation plane.
    pub fn transverse_module(&self) -> f64 {
        self.module / self.helix_angle.to_radians().cos()
    }

    /// Transverse pressure angle in degrees: `αt = atan(tan(αn) / cos(ψ))`.
    ///
    /// Always larger than `αn`.
    pub fn transverse_pressure_angle(&self) -> f64 {
        let alpha_n = self.normal_pressure_angle.to_radians();
        let psi = self.helix_angle.to_radians();
        (alpha_n.tan() / psi.cos()).atan().to_degrees()
    }

    // ── Diameters (mm) ────────────────────────────────────────────────────────

    /// Pitch circle diameter in mm: `d = mt · z`.
    pub fn reference_diameter(&self) -> f64 {
        self.transverse_module() * self.teeth as f64
    }

    /// Tip (outer) diameter in mm: `da = mt·z + 2·mn`.
    ///
    /// Tooth height is governed by the normal module; pitch diameter by the transverse module.
    pub fn tip_diameter(&self) -> f64 {
        self.transverse_module() * self.teeth as f64 + 2.0 * self.module
    }

    /// Root diameter in mm: `df = mt·z − 2.5·mn`.
    pub fn root_diameter(&self) -> f64 {
        self.transverse_module() * self.teeth as f64 - 2.5 * self.module
    }

    /// Base circle diameter in mm: `db = d · cos(αt)`.
    ///
    /// Uses the transverse pressure angle because the involute is defined in the transverse plane.
    pub fn base_diameter(&self) -> f64 {
        self.reference_diameter() * self.transverse_pressure_angle().to_radians().cos()
    }

    // ── Tooth profile (mm) ────────────────────────────────────────────────────

    /// Radial height above pitch circle in mm: `ha = mn`.
    pub fn addendum(&self) -> f64 {
        self.module
    }

    /// Radial depth below pitch circle in mm: `hf = 1.25·mn`.
    pub fn dedendum(&self) -> f64 {
        1.25 * self.module
    }

    /// Full tooth height root-to-tip in mm: `h = 2.25·mn`.
    pub fn tooth_depth(&self) -> f64 {
        2.25 * self.module
    }

    /// Tip-to-root radial clearance in mm: `c = 0.25·mn`.
    pub fn clearance(&self) -> f64 {
        0.25 * self.module
    }

    /// Theoretical tooth thickness in the normal plane in mm: `sn = π·mn / 2`.
    ///
    /// For the thinned value used in real pairs see [`HelicalGear::thinned_tooth_thickness`].
    pub fn tooth_thickness(&self) -> f64 {
        PI * self.module / 2.0
    }

    // ── Pitch ─────────────────────────────────────────────────────────────────

    /// Arc length between teeth in the normal plane in mm: `pn = π·mn`.
    pub fn normal_circular_pitch(&self) -> f64 {
        PI * self.module
    }

    /// Arc length between teeth in the transverse plane in mm: `pt = π·mt`.
    pub fn transverse_circular_pitch(&self) -> f64 {
        PI * self.transverse_module()
    }

    /// Teeth per inch of pitch diameter (imperial): `DP = 25.4 / mn`.
    pub fn diametral_pitch(&self) -> f64 {
        25.4 / self.module
    }

    /// Tooth spacing along the rotation axis in mm: `pa = π·mn / sin(ψ)`.
    pub fn axial_pitch(&self) -> f64 {
        PI * self.module / self.helix_angle.to_radians().sin()
    }

    /// Axial distance for one full helix revolution in mm: `L = pa · z`.
    pub fn lead(&self) -> f64 {
        self.axial_pitch() * self.teeth as f64
    }

    // ── Gear pair ─────────────────────────────────────────────────────────────

    /// `true` if this gear can mesh with `other` on parallel shafts.
    ///
    /// Requires equal normal modules, equal normal pressure angles, equal helix angle magnitudes,
    /// and opposite helix hands.
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

    /// Centre distance between the two gear axes in mm: `a = (d1 + d2) / 2`.
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

    /// Speed ratio to `other`: `i = z_other / z_self`.
    ///
    /// Greater than 1 → `other` is slower (reduction). Less than 1 → faster (step-up).
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
    /// Computed in the transverse plane. For the total contact ratio (including
    /// axial overlap), use [`HelicalGear::total_contact_ratio_with`].
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
    pub fn transverse_contact_ratio_with(&self, other: &HelicalGear) -> f64 {
        crate::contact_ratio::helical_transverse(
            self.tip_diameter() / 2.0,
            self.base_diameter() / 2.0,
            other.tip_diameter() / 2.0,
            other.base_diameter() / 2.0,
            self.center_distance_to(other),
            self.transverse_pressure_angle(),
            self.transverse_module(),
        )
    }

    /// Overlap ratio from the helical tooth sweep: `εβ = b·sin(ψ) / (π·mn)`.
    ///
    /// Returns `None` if no face width was set — face width is required to
    /// compute the axial overlap.
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

    /// Total contact ratio: `εγ = εα + εβ`.
    ///
    /// Returns `None` if no face width was set on this gear.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    ///
    /// let g1 = HelicalGear::builder().module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).face_width(30.0).build().unwrap();
    /// let g2 = HelicalGear::builder().module(2.0).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).face_width(30.0).build().unwrap();
    ///
    /// let eg = g1.total_contact_ratio_with(&g2).unwrap();
    /// let ea = g1.transverse_contact_ratio_with(&g2);
    /// let eb = g1.overlap_ratio().unwrap();
    /// assert!((eg - (ea + eb)).abs() < 1e-10);
    /// ```
    pub fn total_contact_ratio_with(&self, other: &HelicalGear) -> Option<f64> {
        self.overlap_ratio()
            .map(|eb| self.transverse_contact_ratio_with(other) + eb)
    }

    // ── Backlash ──────────────────────────────────────────────────────────────

    /// Normal-plane tooth thickness after applying backlash `jt` (mm):
    /// `sn' = πmn / 2 − (jt / 2) · cos(ψ)`.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    /// use std::f64::consts::PI;
    ///
    /// let g = HelicalGear::builder()
    ///     .module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right)
    ///     .build().unwrap();
    /// let s = g.thinned_tooth_thickness(0.08);
    /// let expected = PI * 2.0 / 2.0 - 0.04 * 20.0_f64.to_radians().cos();
    /// assert!((s - expected).abs() < 1e-10);
    /// ```
    pub fn thinned_tooth_thickness(&self, backlash_mm: f64) -> f64 {
        assert!(backlash_mm >= 0.0, "backlash must be non-negative, got {backlash_mm}");
        PI * self.module / 2.0 - backlash_mm / 2.0 * self.helix_angle.to_radians().cos()
    }

    /// Normal backlash from circular backlash `jt` (mm): `jn = jt · cos(αt) · cos(ψ)`.
    ///
    /// Smaller than `jt` due to both pressure angle and helix angle projections.
    ///
    /// ```
    /// use for_the_love_of_gears::helical::{HelicalGear, HelixHand};
    ///
    /// let g = HelicalGear::builder()
    ///     .module(2.0).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right)
    ///     .build().unwrap();
    /// assert!(g.normal_backlash(0.08) < 0.08);
    /// ```
    pub fn normal_backlash(&self, backlash_mm: f64) -> f64 {
        assert!(backlash_mm >= 0.0, "backlash must be non-negative, got {backlash_mm}");
        backlash_mm
            * self.transverse_pressure_angle().to_radians().cos()
            * self.helix_angle.to_radians().cos()
    }
}

/// Builder for [`HelicalGear`]. Obtain via [`HelicalGear::builder()`].
///
/// `module`, `teeth`, `helix_angle`, and `helix_hand` are required.
/// `normal_pressure_angle` defaults to `20.0°`. `face_width` is optional.
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
    /// Set the normal module `mn` (tooth size) in mm.
    pub fn module(mut self, m: f64) -> Self {
        self.module = Some(m);
        self
    }

    /// Set the number of teeth.
    pub fn teeth(mut self, z: u32) -> Self {
        self.teeth = Some(z);
        self
    }

    /// Set the helix angle `ψ` in degrees. Must be in `(0°, 90°)`.
    ///
    /// `ψ = 0°` is a spur gear — use [`crate::gear::Gear`] for that case.
    pub fn helix_angle(mut self, degrees: f64) -> Self {
        self.helix_angle = Some(degrees);
        self
    }

    /// Set the helix hand.
    pub fn helix_hand(mut self, hand: HelixHand) -> Self {
        self.helix_hand = Some(hand);
        self
    }

    /// Set the normal pressure angle `αn` in degrees. Defaults to `20.0°`.
    pub fn normal_pressure_angle(mut self, degrees: f64) -> Self {
        self.normal_pressure_angle = Some(degrees);
        self
    }

    /// Set the face width in millimetres. Required for overlap and total contact ratios.
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
        if teeth < 3 {
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
