//! Spur gears — teeth parallel to the rotation axis.
//!
//! A spur gear is fully defined by three parameters:
//!
//! | Parameter | Symbol | Unit | Default |
//! |---|---|---|---|
//! | Module | `m` | mm | required |
//! | Number of teeth | `z` | — | required |
//! | Pressure angle | `α` | degrees | 20° (ISO) |
//!
//! Two gears mesh when they share the same module. Size and ratio are independent.
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

use std::{f64::consts::PI, fmt};

use crate::traits::GearGeometry;

const DEFAULT_PRESSURE_ANGLE: f64 = 20.0;

/// Errors returned by [`GearBuilder::build`].
#[derive(Debug, PartialEq)]
#[non_exhaustive]
pub enum GearError {
    /// `.module()` was not called.
    ModuleRequired,
    /// `.teeth()` was not called.
    TeethRequired,
    /// Module must be greater than zero.
    ModuleMustBePositive,
    /// Tooth count must be at least 1.
    TeethMustBePositive,
    /// Tooth count must be at least 3 to produce a positive root diameter.
    ///
    /// With fewer than 3 teeth the dedendum exceeds the pitch radius and
    /// `df = m(z − 2.5)` becomes zero or negative, which is geometrically invalid.
    TeethTooFew,
    /// Pressure angle must be greater than zero degrees.
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

/// A fully defined spur gear.
///
/// All geometry methods return values in **millimetres** (or degrees / dimensionless
/// where noted). Build with [`Gear::builder()`].
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
    pub fn module(&self) -> f64 {
        self.module
    }

    /// Normal module in mm — alias for [`module`] for symmetry with `HelicalGear`.
    ///
    /// [`module`]: Gear::module
    pub fn normal_module(&self) -> f64 {
        self.module
    }

    /// Number of teeth.
    pub fn teeth(&self) -> u32 {
        self.teeth
    }

    /// Pressure angle in degrees. Default is `20.0°` (ISO standard).
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

    /// Pitch circle diameter in mm: `d = m·z`.
    pub fn reference_diameter(&self) -> f64 {
        self.module * self.teeth as f64
    }

    /// Tip (outer) diameter in mm: `da = m(z + 2)`.
    pub fn tip_diameter(&self) -> f64 {
        self.module * (self.teeth as f64 + 2.0)
    }

    /// Root diameter in mm: `df = m(z − 2.5)`.
    pub fn root_diameter(&self) -> f64 {
        self.module * (self.teeth as f64 - 2.5)
    }

    /// Base circle diameter in mm: `db = d · cos(α)`.
    ///
    /// The involute tooth profile unrolls from this circle.
    pub fn base_diameter(&self) -> f64 {
        self.reference_diameter() * self.pressure_angle.to_radians().cos()
    }

    // ── Tooth profile (mm) ────────────────────────────────────────────────────

    /// Addendum — radial height above pitch circle in mm: `ha = m`.
    pub fn addendum(&self) -> f64 {
        self.module
    }

    /// Dedendum — radial depth below pitch circle in mm: `hf = 1.25·m`.
    pub fn dedendum(&self) -> f64 {
        1.25 * self.module
    }

    /// Full tooth height root-to-tip in mm: `h = 2.25·m`.
    pub fn tooth_depth(&self) -> f64 {
        2.25 * self.module
    }

    /// Tip-to-root radial clearance in mm: `c = 0.25·m`.
    pub fn clearance(&self) -> f64 {
        0.25 * self.module
    }

    /// Theoretical tooth thickness along pitch circle in mm: `s = πm / 2`.
    ///
    /// For the thinned value used in real pairs see [`Gear::thinned_tooth_thickness`].
    pub fn tooth_thickness(&self) -> f64 {
        PI * self.module / 2.0
    }

    // ── Pitch ─────────────────────────────────────────────────────────────────

    /// Arc length between adjacent teeth along pitch circle in mm: `p = πm`.
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
    /// Only relevant when interfacing with inch-unit systems.
    pub fn diametral_pitch(&self) -> f64 {
        25.4 / self.module
    }

    // ── Gear pair ─────────────────────────────────────────────────────────────

    /// `true` if this gear can mesh with `other` (same module and pressure angle, within tolerance).
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

    /// Centre distance in mm: `a = (d1 + d2) / 2`.
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

    /// Speed ratio to `other`: `i = z_other / z_self`.
    ///
    /// Greater than 1 → `other` is slower (reduction). Less than 1 → faster (step-up).
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

    /// Transverse contact ratio `εα` with `other`.
    ///
    /// Values above `1.2` are required for smooth running; `1.4–1.8` is typical
    /// for general industrial machinery.
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

    /// Thinned tooth thickness after applying pair backlash `jt` (mm): `s' = πm/2 − jt/2`.
    ///
    /// Backlash is a pair property. Under equal distribution each gear is thinned
    /// by `jt / 2` so the two gears together produce the full gap.
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    /// use std::f64::consts::PI;
    ///
    /// let g = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// let s = g.thinned_tooth_thickness(0.08);
    /// assert!((s - (PI - 0.04)).abs() < 1e-10);
    /// ```
    pub fn thinned_tooth_thickness(&self, backlash_mm: f64) -> f64 {
        assert!(backlash_mm >= 0.0, "backlash must be non-negative, got {backlash_mm}");
        PI * self.module / 2.0 - backlash_mm / 2.0
    }

    /// Normal backlash from circular backlash `jt` (mm): `jn = jt · cos(α)`.
    ///
    /// Normal backlash is what a feeler gauge reads when held perpendicular to
    /// the tooth flank — it is smaller than the circular gap `jt`.
    ///
    /// ```
    /// use for_the_love_of_gears::gear::Gear;
    ///
    /// let g = Gear::builder().module(2.0).teeth(20).build().unwrap();
    /// let jn = g.normal_backlash(0.08);
    /// let expected = 0.08 * 20.0_f64.to_radians().cos();
    /// assert!((jn - expected).abs() < 1e-10);
    /// ```
    pub fn normal_backlash(&self, backlash_mm: f64) -> f64 {
        assert!(backlash_mm >= 0.0, "backlash must be non-negative, got {backlash_mm}");
        backlash_mm * self.pressure_angle.to_radians().cos()
    }
}

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

    fn thinned_tooth_thickness(&self, backlash_mm: f64) -> f64 {
        self.thinned_tooth_thickness(backlash_mm)
    }

    fn normal_backlash(&self, backlash_mm: f64) -> f64 {
        self.normal_backlash(backlash_mm)
    }
}

/// Builder for [`Gear`]. Obtain via [`Gear::builder()`].
///
/// `module` and `teeth` are required. `pressure_angle` defaults to `20.0°`.
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
#[derive(Debug, Default)]
pub struct GearBuilder {
    module: Option<f64>,
    teeth: Option<u32>,
    pressure_angle: Option<f64>,
}

impl GearBuilder {
    /// Set the module (tooth size) in mm.
    pub fn module(mut self, m: f64) -> Self {
        self.module = Some(m);
        self
    }

    /// Set the number of teeth.
    pub fn teeth(mut self, z: u32) -> Self {
        self.teeth = Some(z);
        self
    }

    /// Set the pressure angle in degrees. Defaults to `20.0°` if not called.
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
        if teeth < 3 {
            return Err(GearError::TeethTooFew);
        }

        let pressure_angle = self.pressure_angle.unwrap_or(DEFAULT_PRESSURE_ANGLE);
        if pressure_angle <= 0.0 {
            return Err(GearError::PressureAngleMustBePositive);
        }

        Ok(Gear { module, teeth, pressure_angle })
    }
}
