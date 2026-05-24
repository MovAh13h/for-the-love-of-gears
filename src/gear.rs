use crate::{
    center_distance::CenterDistance,
    diameter::{BaseDiameter, ReferenceDiameter, RootDiameter, TipDiameter},
    module::Module,
    pitch::{CircularPitch, DiametralPitch},
    tooth::{Addendum, Clearance, Dedendum, ToothDepth, ToothThickness},
};

/// The standard pressure angle for ISO spur gears in degrees.
const DEFAULT_PRESSURE_ANGLE: f64 = 20.0;

/// A fully defined spur gear.
///
/// Construct with [`GearBuilder`] via [`Gear::builder()`]:
///
/// ```
/// use for_the_love_of_gears::{gear::Gear, module::Module};
///
/// let gear = Gear::builder()
///     .module(Module::Specified(2.0))
///     .teeth(20)
///     .build()
///     .unwrap();
///
/// assert_eq!(gear.reference_diameter().value(), 40.0);
/// assert_eq!(gear.tip_diameter().value(), 44.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Gear {
    module: Module,
    teeth: u32,
    pressure_angle: f64,
    face_width: Option<f64>,
}

impl Gear {
    /// Start building a [`Gear`].
    pub fn builder() -> GearBuilder {
        GearBuilder::new()
    }

    /// The module of this gear.
    pub fn module(&self) -> Module {
        self.module
    }

    /// The number of teeth.
    pub fn teeth(&self) -> u32 {
        self.teeth
    }

    /// The pressure angle in degrees.
    pub fn pressure_angle(&self) -> f64 {
        self.pressure_angle
    }

    /// The face width in millimetres, if specified.
    pub fn face_width(&self) -> Option<f64> {
        self.face_width
    }

    // --- Derived geometry ---

    /// Diameter of the pitch circle: `d = mz`
    pub fn reference_diameter(&self) -> ReferenceDiameter {
        ReferenceDiameter::new(self.module.value(), self.teeth)
    }

    /// Outer diameter of the gear: `da = m(z + 2)`
    pub fn tip_diameter(&self) -> TipDiameter {
        TipDiameter::new(self.module.value(), self.teeth)
    }

    /// Diameter at the tooth root: `df = m(z − 2.5)`
    pub fn root_diameter(&self) -> RootDiameter {
        RootDiameter::new(self.module.value(), self.teeth)
    }

    /// Diameter of the base circle: `db = d · cos(α)`
    pub fn base_diameter(&self) -> BaseDiameter {
        BaseDiameter::new(self.module.value(), self.teeth, self.pressure_angle)
    }

    /// Radial distance from pitch circle to tooth tip: `ha = m`
    pub fn addendum(&self) -> Addendum {
        Addendum::from_module(self.module.value())
    }

    /// Radial distance from pitch circle to tooth root: `hf = 1.25m`
    pub fn dedendum(&self) -> Dedendum {
        Dedendum::from_module(self.module.value())
    }

    /// Full tooth height: `h = 2.25m`
    pub fn tooth_depth(&self) -> ToothDepth {
        ToothDepth::from_module(self.module.value())
    }

    /// Tooth thickness along the pitch circle: `s = πm / 2`
    pub fn tooth_thickness(&self) -> ToothThickness {
        ToothThickness::from_module(self.module.value())
    }

    /// Radial clearance between mating gear tip and root: `c = 0.25m`
    pub fn clearance(&self) -> Clearance {
        Clearance::from_module(self.module.value())
    }

    /// Arc length between adjacent teeth: `p = πm`
    pub fn circular_pitch(&self) -> CircularPitch {
        CircularPitch::from_module(self.module.value())
    }

    /// Teeth per inch (imperial): `DP = 25.4 / m`
    pub fn diametral_pitch(&self) -> DiametralPitch {
        DiametralPitch::from_module(self.module.value())
    }

    /// Centre distance to a mating gear: `a = (d1 + d2) / 2`
    ///
    /// Both gears must share the same module to mesh.
    pub fn center_distance_to(&self, other: &Gear) -> CenterDistance {
        CenterDistance::from_reference_diameters(
            self.reference_diameter().value(),
            other.reference_diameter().value(),
        )
    }
}

/// Builder for [`Gear`].
///
/// `module` and `teeth` are required. All other fields are optional
/// and fall back to ISO defaults if not set.
///
/// ```
/// use for_the_love_of_gears::{gear::Gear, module::Module};
///
/// let gear = Gear::builder()
///     .module(Module::Specified(4.0))
///     .teeth(30)
///     .pressure_angle(20.0)  // optional — default is 20°
///     .face_width(40.0)      // optional
///     .build()
///     .unwrap();
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

    /// Set the gear module.
    pub fn module(mut self, module: Module) -> Self {
        self.module = Some(module);
        self
    }

    /// Set the number of teeth.
    pub fn teeth(mut self, teeth: u32) -> Self {
        self.teeth = Some(teeth);
        self
    }

    /// Set the pressure angle in degrees. Defaults to `20.0` if not called.
    pub fn pressure_angle(mut self, degrees: f64) -> Self {
        self.pressure_angle = Some(degrees);
        self
    }

    /// Set the face width in millimetres.
    pub fn face_width(mut self, mm: f64) -> Self {
        self.face_width = Some(mm);
        self
    }

    /// Build the [`Gear`].
    ///
    /// Returns `Err` if `module` or `teeth` were not set.
    pub fn build(self) -> Result<Gear, &'static str> {
        let module = self.module.ok_or("module is required")?;
        let teeth = self.teeth.ok_or("teeth is required")?;

        Ok(Gear {
            module,
            teeth,
            pressure_angle: self.pressure_angle.unwrap_or(DEFAULT_PRESSURE_ANGLE),
            face_width: self.face_width,
        })
    }
}
