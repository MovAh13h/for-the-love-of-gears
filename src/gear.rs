use std::fmt;

use crate::{
    center_distance::CenterDistance,
    diameter::{BaseDiameter, ReferenceDiameter, RootDiameter, TipDiameter},
    module::Module,
    pitch::{CircularPitch, DiametralPitch},
    tooth::{Addendum, Clearance, Dedendum, ToothDepth, ToothThickness},
};

/// The standard pressure angle for ISO spur gears in degrees.
const DEFAULT_PRESSURE_ANGLE: f64 = 20.0;

/// Errors returned by [`GearBuilder::build`].
#[derive(Debug, PartialEq)]
pub enum GearError {
    /// No module was given to the builder.
    ModuleRequired,
    /// No tooth count was given to the builder.
    TeethRequired,
    /// Module value must be greater than zero.
    ModuleMustBePositive,
    /// Tooth count must be at least 1.
    TeethMustBePositive,
    /// Pressure angle must be greater than zero degrees.
    PressureAngleMustBePositive,
    /// Face width must be greater than zero millimetres.
    FaceWidthMustBePositive,
    /// No helix angle was given to the builder.
    HelixAngleRequired,
    /// No helix hand was given to the builder.
    HelixHandRequired,
    /// Helix angle must be greater than zero degrees.
    ///
    /// A helix angle of `0°` describes a spur gear — use [`Gear`] for that case.
    HelixAngleMustBePositive,
    /// Helix angle must be less than 90 degrees.
    HelixAngleMustBeLessThan90,
}

impl fmt::Display for GearError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ModuleRequired => write!(f, "module is required"),
            Self::TeethRequired => write!(f, "teeth count is required"),
            Self::ModuleMustBePositive => write!(f, "module value must be greater than zero"),
            Self::TeethMustBePositive => write!(f, "teeth count must be at least 1"),
            Self::PressureAngleMustBePositive => {
                write!(f, "pressure angle must be greater than zero degrees")
            }
            Self::FaceWidthMustBePositive => {
                write!(f, "face width must be greater than zero mm")
            }
            Self::HelixAngleRequired => write!(f, "helix angle is required"),
            Self::HelixHandRequired => write!(f, "helix hand is required"),
            Self::HelixAngleMustBePositive => {
                write!(f, "helix angle must be greater than zero degrees")
            }
            Self::HelixAngleMustBeLessThan90 => {
                write!(f, "helix angle must be less than 90 degrees")
            }
        }
    }
}

/// A fully defined spur gear.
///
/// A spur gear is the simplest gear type: a cylinder with teeth cut parallel
/// to its rotation axis. Three values fully determine all of its geometry:
///
/// | Parameter | Description | Default |
/// |---|---|---|
/// | `module` | Tooth size scale (ISO unit, mm) | required |
/// | `teeth` | Number of teeth | required |
/// | `pressure_angle` | Tooth flank angle in degrees | 20° (ISO standard) |
///
/// An optional `face_width` (axial length of the teeth, mm) can also be stored —
/// it cannot be derived from the other parameters and is needed for load calculations.
///
/// All derived dimensions (diameters, tooth profile, pitch) are methods on this
/// struct and return the corresponding typed value from this library.
///
/// # Building a gear
///
/// ```
/// use for_the_love_of_gears::{gear::Gear, module::Module};
///
/// // Minimum — module and teeth are required
/// let gear = Gear::builder()
///     .module(Module::Specified(2.0))
///     .teeth(20)
///     .build()
///     .unwrap();
///
/// // With all parameters
/// let gear = Gear::builder()
///     .module(Module::Specified(2.0))
///     .teeth(20)
///     .pressure_angle(20.0) // optional, defaults to 20°
///     .face_width(25.0)     // optional, in mm
///     .build()
///     .unwrap();
/// ```
///
/// # Derived geometry
///
/// ```
/// use for_the_love_of_gears::{gear::Gear, module::Module};
/// use std::f64::consts::PI;
///
/// let gear = Gear::builder()
///     .module(Module::Specified(2.0))
///     .teeth(20)
///     .build()
///     .unwrap();
///
/// // Diameters (mm)
/// assert_eq!(gear.reference_diameter().value(), 40.0);  // d  = mz
/// assert_eq!(gear.tip_diameter().value(),       44.0);  // da = m(z+2)
/// assert_eq!(gear.root_diameter().value(),      35.0);  // df = m(z-2.5)
///
/// // Tooth profile (mm)
/// assert_eq!(gear.addendum().value(),    2.0);   // ha = m
/// assert_eq!(gear.dedendum().value(),    2.5);   // hf = 1.25m
/// assert_eq!(gear.tooth_depth().value(), 4.5);   // h  = 2.25m
/// assert_eq!(gear.clearance().value(),   0.5);   // c  = 0.25m
/// ```
///
/// # Gear pairs
///
/// ```
/// use for_the_love_of_gears::{gear::Gear, module::Module};
///
/// let driver = Gear::builder()
///     .module(Module::Specified(2.0))
///     .teeth(20)
///     .build()
///     .unwrap();
///
/// let driven = Gear::builder()
///     .module(Module::Specified(2.0))
///     .teeth(40)
///     .build()
///     .unwrap();
///
/// assert!(driver.can_mesh_with(&driven));
/// assert_eq!(driver.center_distance_to(&driven).value(), 60.0); // (40+80)/2
/// assert_eq!(driver.gear_ratio_to(&driven), 2.0); // 40/20
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Gear {
    module: Module,
    teeth: u32,
    pressure_angle: f64,
    face_width: Option<f64>,
}

impl Gear {
    /// Start building a [`Gear`]. See [`GearBuilder`] for all options.
    pub fn builder() -> GearBuilder {
        GearBuilder::new()
    }

    // --- Inputs ---

    /// The module (tooth size) of this gear.
    ///
    /// Two gears can only mesh if their modules are equal — see [`Gear::can_mesh_with`].
    pub fn module(&self) -> Module {
        self.module
    }

    /// The number of teeth.
    pub fn teeth(&self) -> u32 {
        self.teeth
    }

    /// The pressure angle in degrees.
    ///
    /// Defines the angle of the tooth flank relative to the pitch circle tangent.
    /// The ISO standard value is `20.0°`. Older systems used `14.5°`.
    pub fn pressure_angle(&self) -> f64 {
        self.pressure_angle
    }

    /// The face width in millimetres, if provided.
    ///
    /// Face width is the axial length of the teeth — how wide the gear is along
    /// its rotation axis. It cannot be derived from module, teeth, or pressure
    /// angle; it is a design choice driven by load requirements.
    pub fn face_width(&self) -> Option<f64> {
        self.face_width
    }

    // --- Derived geometry (all values in mm unless noted) ---

    /// Diameter of the pitch circle in mm: `d = mz`.
    ///
    /// The pitch circle is the reference from which all other dimensions are
    /// measured. See the [crate-level documentation](crate) for a full explanation.
    pub fn reference_diameter(&self) -> ReferenceDiameter {
        ReferenceDiameter::new(self.module.value(), self.teeth)
    }

    /// Outer diameter of the gear in mm: `da = m(z + 2)`.
    ///
    /// The dimension a caliper measures across the gear blank.
    pub fn tip_diameter(&self) -> TipDiameter {
        TipDiameter::new(self.module.value(), self.teeth)
    }

    /// Diameter at the base of the tooth spaces in mm: `df = m(z − 2.5)`.
    pub fn root_diameter(&self) -> RootDiameter {
        RootDiameter::new(self.module.value(), self.teeth)
    }

    /// Diameter of the involute base circle in mm: `db = d · cos(α)`.
    ///
    /// The tooth flanks are involutes of this circle. Its size depends on the
    /// pressure angle — a larger angle produces a smaller base circle and steeper
    /// tooth flanks.
    pub fn base_diameter(&self) -> BaseDiameter {
        BaseDiameter::new(self.module.value(), self.teeth, self.pressure_angle)
    }

    /// Radial distance from pitch circle to tooth tip in mm: `ha = m`.
    pub fn addendum(&self) -> Addendum {
        Addendum::from_module(self.module.value())
    }

    /// Radial distance from pitch circle to tooth root in mm: `hf = 1.25m`.
    pub fn dedendum(&self) -> Dedendum {
        Dedendum::from_module(self.module.value())
    }

    /// Full tooth height (root to tip) in mm: `h = 2.25m`.
    pub fn tooth_depth(&self) -> ToothDepth {
        ToothDepth::from_module(self.module.value())
    }

    /// Tooth thickness measured along the pitch circle in mm: `s = πm / 2`.
    pub fn tooth_thickness(&self) -> ToothThickness {
        ToothThickness::from_module(self.module.value())
    }

    /// Radial gap between this gear's root and the mating gear's tip in mm: `c = 0.25m`.
    pub fn clearance(&self) -> Clearance {
        Clearance::from_module(self.module.value())
    }

    /// Arc length between adjacent teeth along the pitch circle in mm: `p = πm`.
    pub fn circular_pitch(&self) -> CircularPitch {
        CircularPitch::from_module(self.module.value())
    }

    /// Teeth per inch of pitch diameter (imperial, not mm): `DP = 25.4 / m`.
    ///
    /// Only relevant when interfacing with inch-unit systems. For metric work,
    /// [`Gear::module`] is the tooth size reference.
    pub fn diametral_pitch(&self) -> DiametralPitch {
        DiametralPitch::from_module(self.module.value())
    }

    // --- Gear pair ---

    /// Returns `true` if this gear can mesh with `other`.
    ///
    /// Two gears can mesh only if their modules are equal — identical tooth size
    /// is required for the teeth to fit together. This check uses a small
    /// floating-point tolerance to account for values derived from measurements.
    ///
    /// ```
    /// use for_the_love_of_gears::{gear::Gear, module::Module};
    ///
    /// let g1 = Gear::builder().module(Module::Specified(2.0)).teeth(20).build().unwrap();
    /// let g2 = Gear::builder().module(Module::Specified(2.0)).teeth(40).build().unwrap();
    /// let g3 = Gear::builder().module(Module::Specified(3.0)).teeth(20).build().unwrap();
    ///
    /// assert!(g1.can_mesh_with(&g2));
    /// assert!(!g1.can_mesh_with(&g3));
    /// ```
    pub fn can_mesh_with(&self, other: &Gear) -> bool {
        (self.module.value() - other.module.value()).abs() < 1e-9
    }

    /// Centre distance between the two gear axes in mm: `a = (d1 + d2) / 2`.
    ///
    /// This is the shaft spacing required for the two gears to mesh correctly.
    /// Only meaningful when both gears share the same module — use
    /// [`Gear::can_mesh_with`] to verify before calling this.
    ///
    /// ```
    /// use for_the_love_of_gears::{gear::Gear, module::Module};
    ///
    /// let g1 = Gear::builder().module(Module::Specified(2.0)).teeth(20).build().unwrap();
    /// let g2 = Gear::builder().module(Module::Specified(2.0)).teeth(40).build().unwrap();
    ///
    /// // d1 = 40mm, d2 = 80mm → a = 60mm
    /// assert_eq!(g1.center_distance_to(&g2).value(), 60.0);
    /// ```
    pub fn center_distance_to(&self, other: &Gear) -> CenterDistance {
        CenterDistance::from_reference_diameters(
            self.reference_diameter().value(),
            other.reference_diameter().value(),
        )
    }

    /// Speed ratio from this gear to `other`: `i = z_other / z_self`.
    ///
    /// A ratio greater than 1.0 means `other` rotates slower (speed reduction).
    /// A ratio less than 1.0 means `other` rotates faster (speed increase).
    ///
    /// ```
    /// use for_the_love_of_gears::{gear::Gear, module::Module};
    ///
    /// let driver = Gear::builder().module(Module::Specified(2.0)).teeth(20).build().unwrap();
    /// let driven = Gear::builder().module(Module::Specified(2.0)).teeth(40).build().unwrap();
    ///
    /// assert_eq!(driver.gear_ratio_to(&driven), 2.0); // driven turns at half the speed
    /// assert_eq!(driven.gear_ratio_to(&driver), 0.5); // driver turns at double the speed
    /// ```
    pub fn gear_ratio_to(&self, other: &Gear) -> f64 {
        other.teeth as f64 / self.teeth as f64
    }
}

/// Builder for [`Gear`]. Obtain one via [`Gear::builder()`].
///
/// `module` and `teeth` are required. `pressure_angle` defaults to `20.0°`
/// (ISO standard). `face_width` is optional.
///
/// [`GearBuilder::build`] validates all inputs and returns a [`GearError`] if
/// anything is missing or out of range.
///
/// ```
/// use for_the_love_of_gears::{gear::{Gear, GearError}, module::Module};
///
/// // Missing teeth — returns an error
/// let result = Gear::builder()
///     .module(Module::Specified(2.0))
///     .build();
/// assert_eq!(result, Err(GearError::TeethRequired));
///
/// // Invalid module — returns an error
/// let result = Gear::builder()
///     .module(Module::Specified(0.0))
///     .teeth(20)
///     .build();
/// assert_eq!(result, Err(GearError::ModuleMustBePositive));
/// ```
#[derive(Debug, Default)]
pub struct GearBuilder {
    module: Option<Module>,
    teeth: Option<u32>,
    pressure_angle: Option<f64>,
    face_width: Option<f64>,
}

impl GearBuilder {
    fn new() -> Self {
        Self::default()
    }

    /// Set the gear module (tooth size, in mm).
    pub fn module(mut self, module: Module) -> Self {
        self.module = Some(module);
        self
    }

    /// Set the number of teeth.
    pub fn teeth(mut self, teeth: u32) -> Self {
        self.teeth = Some(teeth);
        self
    }

    /// Set the pressure angle in degrees. Defaults to `20.0°` if not called.
    pub fn pressure_angle(mut self, degrees: f64) -> Self {
        self.pressure_angle = Some(degrees);
        self
    }

    /// Set the face width in millimetres.
    pub fn face_width(mut self, mm: f64) -> Self {
        self.face_width = Some(mm);
        self
    }

    /// Build the [`Gear`], validating all inputs.
    ///
    /// # Errors
    ///
    /// | Error | Cause |
    /// |---|---|
    /// | [`GearError::ModuleRequired`] | `.module()` was not called |
    /// | [`GearError::TeethRequired`] | `.teeth()` was not called |
    /// | [`GearError::ModuleMustBePositive`] | module value ≤ 0 |
    /// | [`GearError::TeethMustBePositive`] | teeth = 0 |
    /// | [`GearError::PressureAngleMustBePositive`] | pressure angle ≤ 0° |
    /// | [`GearError::FaceWidthMustBePositive`] | face width ≤ 0 mm |
    pub fn build(self) -> Result<Gear, GearError> {
        let module = self.module.ok_or(GearError::ModuleRequired)?;
        let teeth = self.teeth.ok_or(GearError::TeethRequired)?;

        if module.value() <= 0.0 {
            return Err(GearError::ModuleMustBePositive);
        }
        if teeth == 0 {
            return Err(GearError::TeethMustBePositive);
        }

        let pressure_angle = self.pressure_angle.unwrap_or(DEFAULT_PRESSURE_ANGLE);
        if pressure_angle <= 0.0 {
            return Err(GearError::PressureAngleMustBePositive);
        }

        if self.face_width.is_some_and(|fw| fw <= 0.0) {
            return Err(GearError::FaceWidthMustBePositive);
        }

        Ok(Gear {
            module,
            teeth,
            pressure_angle,
            face_width: self.face_width,
        })
    }
}
