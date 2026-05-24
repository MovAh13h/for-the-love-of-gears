use std::f64::consts::PI;

use crate::{
    center_distance::CenterDistance,
    contact_ratio::{OverlapRatio, TotalContactRatio, TransverseContactRatio},
    diameter::{BaseDiameter, ReferenceDiameter, RootDiameter, TipDiameter},
    gear::GearError,
    module::Module,
    pitch::{CircularPitch, DiametralPitch},
    tooth::{Addendum, Clearance, Dedendum, ToothDepth, ToothThickness},
};

/// The standard normal pressure angle for helical gears in degrees.
const DEFAULT_NORMAL_PRESSURE_ANGLE: f64 = 20.0;

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

/// Tooth spacing measured along the rotation axis: `pa = π·mn / sin(ψ)`.
///
/// Axial pitch is the distance along the shaft axis between corresponding points
/// on adjacent teeth. It grows as the helix angle `ψ` decreases — a shallow
/// helix has a very long axial pitch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxialPitch(f64);

impl AxialPitch {
    /// Compute axial pitch from normal module and helix angle: `pa = π·mn / sin(ψ)`.
    pub fn new(normal_module: f64, helix_angle_deg: f64) -> Self {
        Self(PI * normal_module / helix_angle_deg.to_radians().sin())
    }

    /// Returns the axial pitch in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Axial distance for one complete tooth helix revolution: `L = pa · z`.
///
/// Lead is how far along the shaft axis a tooth advances in one full rotation
/// of the gear. It is the axial equivalent of the pitch circle circumference.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lead(f64);

impl Lead {
    /// Compute lead from axial pitch and tooth count: `L = pa · z`.
    pub fn new(axial_pitch: f64, teeth: u32) -> Self {
        Self(axial_pitch * teeth as f64)
    }

    /// Returns the lead in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// A fully defined helical gear.
///
/// A helical gear is a cylinder with teeth cut at the **helix angle** `ψ` to the
/// rotation axis. Because the teeth are angled, multiple teeth are always in mesh
/// simultaneously — this distributes load, reduces noise, and increases capacity
/// compared with spur gears.
///
/// The trade-off is an **axial thrust force** proportional to `tan(ψ)` that the
/// shaft bearings must absorb.
///
/// # Relationship to spur gears
///
/// A spur gear is a helical gear with `ψ = 0°` — teeth parallel to the rotation
/// axis, no axial force, contact snapping tooth-to-tooth. [`HelicalGear`] requires
/// `ψ > 0°`; use [`crate::gear::Gear`] for the spur case.
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
/// # Parameters
///
/// | Parameter | Description | Default |
/// |---|---|---|
/// | `module` | Normal module `mn` — tooth size in mm | required |
/// | `teeth` | Number of teeth | required |
/// | `helix_angle` | Tooth helix angle `ψ` in degrees (typically 15–30°) | required |
/// | `helix_hand` | Winding direction — [`HelixHand::Left`] or [`HelixHand::Right`] | required |
/// | `normal_pressure_angle` | Pressure angle in the normal plane `αn` in degrees | 20° |
/// | `face_width` | Axial tooth length in mm — needed for load calculations | optional |
///
/// # Building a helical gear
///
/// ```
/// use for_the_love_of_gears::{
///     helical::{HelicalGear, HelixHand},
///     module::Module,
/// };
///
/// let gear = HelicalGear::builder()
///     .module(Module::Specified(2.0))
///     .teeth(20)
///     .helix_angle(20.0)
///     .helix_hand(HelixHand::Right)
///     .build()
///     .unwrap();
///
/// // Derived transverse values
/// let mt = gear.transverse_module();
/// let at = gear.transverse_pressure_angle();
/// ```
///
/// # Gear pairs
///
/// ```
/// use for_the_love_of_gears::{
///     helical::{HelicalGear, HelixHand},
///     module::Module,
/// };
///
/// let driver = HelicalGear::builder()
///     .module(Module::Specified(2.0))
///     .teeth(20)
///     .helix_angle(20.0)
///     .helix_hand(HelixHand::Right)
///     .build()
///     .unwrap();
///
/// let driven = HelicalGear::builder()
///     .module(Module::Specified(2.0))
///     .teeth(40)
///     .helix_angle(20.0)
///     .helix_hand(HelixHand::Left) // opposite hand required
///     .build()
///     .unwrap();
///
/// assert!(driver.can_mesh_with(&driven));
/// assert_eq!(driver.gear_ratio_to(&driven), 2.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct HelicalGear {
    module: Module,
    teeth: u32,
    helix_angle: f64,
    helix_hand: HelixHand,
    normal_pressure_angle: f64,
    face_width: Option<f64>,
}

impl HelicalGear {
    /// Start building a [`HelicalGear`]. See [`HelicalGearBuilder`] for all options.
    pub fn builder() -> HelicalGearBuilder {
        HelicalGearBuilder::new()
    }

    // --- Inputs ---

    /// The normal module `mn` (tooth size in the normal plane) in mm.
    pub fn normal_module(&self) -> Module {
        self.module
    }

    /// The number of teeth.
    pub fn teeth(&self) -> u32 {
        self.teeth
    }

    /// The helix angle `ψ` in degrees.
    pub fn helix_angle(&self) -> f64 {
        self.helix_angle
    }

    /// The winding direction of the helix.
    pub fn helix_hand(&self) -> HelixHand {
        self.helix_hand
    }

    /// The normal pressure angle `αn` in degrees.
    pub fn normal_pressure_angle(&self) -> f64 {
        self.normal_pressure_angle
    }

    /// The face width in millimetres, if provided.
    pub fn face_width(&self) -> Option<f64> {
        self.face_width
    }

    // --- Derived — transverse plane ---

    /// Transverse module in mm: `mt = mn / cos(ψ)`.
    ///
    /// Always larger than the normal module — the helix stretches the apparent
    /// tooth pitch when viewed in the rotation plane.
    pub fn transverse_module(&self) -> f64 {
        self.module.value() / self.helix_angle.to_radians().cos()
    }

    /// Transverse pressure angle in degrees: `αt = atan(tan(αn) / cos(ψ))`.
    ///
    /// The pressure angle as seen in the rotation plane. Always larger than `αn`.
    pub fn transverse_pressure_angle(&self) -> f64 {
        let alpha_n = self.normal_pressure_angle.to_radians();
        let psi = self.helix_angle.to_radians();
        (alpha_n.tan() / psi.cos()).atan().to_degrees()
    }

    // --- Derived geometry (all values in mm unless noted) ---

    /// Pitch circle diameter in mm: `d = mt · z`.
    pub fn reference_diameter(&self) -> ReferenceDiameter {
        ReferenceDiameter::new(self.transverse_module(), self.teeth)
    }

    /// Outer diameter in mm: `da = mt·z + 2·mn`.
    ///
    /// Tooth height is governed by the normal module; pitch diameter by the transverse module.
    pub fn tip_diameter(&self) -> TipDiameter {
        TipDiameter::from_helical(self.transverse_module(), self.module.value(), self.teeth)
    }

    /// Root diameter in mm: `df = mt·z − 2.5·mn`.
    pub fn root_diameter(&self) -> RootDiameter {
        RootDiameter::from_helical(self.transverse_module(), self.module.value(), self.teeth)
    }

    /// Base circle diameter in mm: `db = d · cos(αt)`.
    ///
    /// Uses the transverse pressure angle because the involute profile is defined
    /// in the transverse plane.
    pub fn base_diameter(&self) -> BaseDiameter {
        BaseDiameter::new(
            self.transverse_module(),
            self.teeth,
            self.transverse_pressure_angle(),
        )
    }

    // --- Derived — tooth profile (all use normal module mn) ---

    /// Radial distance from pitch circle to tooth tip in mm: `ha = mn`.
    pub fn addendum(&self) -> Addendum {
        Addendum::from_module(self.module.value())
    }

    /// Radial distance from pitch circle to tooth root in mm: `hf = 1.25·mn`.
    pub fn dedendum(&self) -> Dedendum {
        Dedendum::from_module(self.module.value())
    }

    /// Full tooth height in mm: `h = 2.25·mn`.
    pub fn tooth_depth(&self) -> ToothDepth {
        ToothDepth::from_module(self.module.value())
    }

    /// Tooth thickness in the normal plane in mm: `s = π·mn / 2`.
    pub fn tooth_thickness(&self) -> ToothThickness {
        ToothThickness::from_module(self.module.value())
    }

    /// Radial clearance gap in mm: `c = 0.25·mn`.
    pub fn clearance(&self) -> Clearance {
        Clearance::from_module(self.module.value())
    }

    // --- Derived — pitch ---

    /// Arc length between teeth in the normal plane in mm: `pn = π·mn`.
    pub fn normal_circular_pitch(&self) -> CircularPitch {
        CircularPitch::from_module(self.module.value())
    }

    /// Arc length between teeth in the transverse plane in mm: `pt = π·mt`.
    pub fn transverse_circular_pitch(&self) -> CircularPitch {
        CircularPitch::from_module(self.transverse_module())
    }

    /// Teeth per inch of pitch diameter (imperial): `DP = 25.4 / mn`.
    pub fn diametral_pitch(&self) -> DiametralPitch {
        DiametralPitch::from_module(self.module.value())
    }

    /// Tooth spacing along the rotation axis in mm: `pa = π·mn / sin(ψ)`.
    pub fn axial_pitch(&self) -> AxialPitch {
        AxialPitch::new(self.module.value(), self.helix_angle)
    }

    /// Axial distance for one full helix revolution in mm: `L = pa · z`.
    pub fn lead(&self) -> Lead {
        Lead::new(self.axial_pitch().value(), self.teeth)
    }

    // --- Gear pair ---

    /// Returns `true` if this gear can mesh with `other` on parallel shafts.
    ///
    /// Requires equal normal modules, equal helix angle magnitudes, and opposite helix hands.
    ///
    /// ```
    /// use for_the_love_of_gears::{helical::{HelicalGear, HelixHand}, module::Module};
    ///
    /// let g1 = HelicalGear::builder()
    ///     .module(Module::Specified(2.0)).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right)
    ///     .build().unwrap();
    ///
    /// let g2 = HelicalGear::builder()
    ///     .module(Module::Specified(2.0)).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left)
    ///     .build().unwrap();
    ///
    /// assert!(g1.can_mesh_with(&g2));
    /// ```
    pub fn can_mesh_with(&self, other: &HelicalGear) -> bool {
        let same_module = (self.module.value() - other.module.value()).abs() < 1e-9;
        let same_angle = (self.helix_angle - other.helix_angle).abs() < 1e-9;
        let opposite_hand = other.helix_hand == self.helix_hand.opposite();
        same_module && same_angle && opposite_hand
    }

    /// Centre distance between the two gear axes in mm: `a = (d1 + d2) / 2`.
    ///
    /// ```
    /// use for_the_love_of_gears::{helical::{HelicalGear, HelixHand}, module::Module};
    /// let g1 = HelicalGear::builder().module(Module::Specified(2.0)).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// let g2 = HelicalGear::builder().module(Module::Specified(2.0)).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).build().unwrap();
    /// let a = g1.center_distance_to(&g2).value();
    /// let expected = (g1.reference_diameter().value() + g2.reference_diameter().value()) / 2.0;
    /// assert!((a - expected).abs() < 1e-10);
    /// ```
    pub fn center_distance_to(&self, other: &HelicalGear) -> CenterDistance {
        CenterDistance::from_reference_diameters(
            self.reference_diameter().value(),
            other.reference_diameter().value(),
        )
    }

    /// Transverse contact ratio between this gear and `other`: `εα`.
    ///
    /// Computed in the transverse plane using the transverse pressure angle and
    /// transverse base pitch. For the total contact ratio (including the axial
    /// overlap contribution), use [`HelicalGear::total_contact_ratio_with`].
    ///
    /// ```
    /// use for_the_love_of_gears::{helical::{HelicalGear, HelixHand}, module::Module};
    ///
    /// let g1 = HelicalGear::builder().module(Module::Specified(2.0)).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// let g2 = HelicalGear::builder().module(Module::Specified(2.0)).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).build().unwrap();
    /// assert!(g1.transverse_contact_ratio_with(&g2).value() > 1.0);
    /// ```
    pub fn transverse_contact_ratio_with(&self, other: &HelicalGear) -> TransverseContactRatio {
        let ra1 = self.tip_diameter().value() / 2.0;
        let rb1 = self.base_diameter().value() / 2.0;
        let ra2 = other.tip_diameter().value() / 2.0;
        let rb2 = other.base_diameter().value() / 2.0;
        let a = self.center_distance_to(other).value();
        let alpha_t = self.transverse_pressure_angle();
        let pb_t = PI * self.transverse_module() * alpha_t.to_radians().cos();
        TransverseContactRatio::from_geometry(ra1, rb1, ra2, rb2, a, alpha_t, pb_t)
    }

    /// Overlap ratio from the helical tooth sweep: `εβ = b·sin(ψ) / (π·mn)`.
    ///
    /// Returns `None` if no face width was set on this gear — face width is
    /// required to compute the axial overlap.
    ///
    /// ```
    /// use for_the_love_of_gears::{helical::{HelicalGear, HelixHand}, module::Module};
    ///
    /// let g = HelicalGear::builder().module(Module::Specified(2.0)).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right)
    ///     .face_width(30.0)
    ///     .build().unwrap();
    /// assert!(g.overlap_ratio().is_some());
    ///
    /// let g_no_fw = HelicalGear::builder().module(Module::Specified(2.0)).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// assert!(g_no_fw.overlap_ratio().is_none());
    /// ```
    pub fn overlap_ratio(&self) -> Option<OverlapRatio> {
        self.face_width
            .map(|b| OverlapRatio::new(b, self.helix_angle, self.module.value()))
    }

    /// Total contact ratio: `εγ = εα + εβ`.
    ///
    /// Returns `None` if no face width was set on this gear, since the overlap
    /// ratio `εβ` cannot be computed without it.
    ///
    /// ```
    /// use for_the_love_of_gears::{helical::{HelicalGear, HelixHand}, module::Module};
    ///
    /// let g1 = HelicalGear::builder().module(Module::Specified(2.0)).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).face_width(30.0).build().unwrap();
    /// let g2 = HelicalGear::builder().module(Module::Specified(2.0)).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).face_width(30.0).build().unwrap();
    ///
    /// let eg = g1.total_contact_ratio_with(&g2).unwrap();
    /// let ea = g1.transverse_contact_ratio_with(&g2);
    /// let eb = g1.overlap_ratio().unwrap();
    /// assert!((eg.value() - (ea.value() + eb.value())).abs() < 1e-10);
    /// ```
    pub fn total_contact_ratio_with(&self, other: &HelicalGear) -> Option<TotalContactRatio> {
        self.overlap_ratio()
            .map(|eb| TotalContactRatio::new(self.transverse_contact_ratio_with(other), eb))
    }

    /// Speed ratio from this gear to `other`: `i = z_other / z_self`.
    ///
    /// A ratio greater than 1.0 means `other` rotates slower (speed reduction).
    /// A ratio less than 1.0 means `other` rotates faster (speed increase).
    ///
    /// ```
    /// use for_the_love_of_gears::{helical::{HelicalGear, HelixHand}, module::Module};
    /// let driver = HelicalGear::builder().module(Module::Specified(2.0)).teeth(20)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Right).build().unwrap();
    /// let driven = HelicalGear::builder().module(Module::Specified(2.0)).teeth(40)
    ///     .helix_angle(20.0).helix_hand(HelixHand::Left).build().unwrap();
    /// assert_eq!(driver.gear_ratio_to(&driven), 2.0);
    /// assert_eq!(driven.gear_ratio_to(&driver), 0.5);
    /// ```
    pub fn gear_ratio_to(&self, other: &HelicalGear) -> f64 {
        other.teeth as f64 / self.teeth as f64
    }
}

/// Builder for [`HelicalGear`]. Obtain one via [`HelicalGear::builder()`].
///
/// `module`, `teeth`, `helix_angle`, and `helix_hand` are required.
/// `normal_pressure_angle` defaults to `20.0°`. `face_width` is optional.
#[derive(Debug, Default)]
pub struct HelicalGearBuilder {
    module: Option<Module>,
    teeth: Option<u32>,
    helix_angle: Option<f64>,
    helix_hand: Option<HelixHand>,
    normal_pressure_angle: Option<f64>,
    face_width: Option<f64>,
}

impl HelicalGearBuilder {
    fn new() -> Self {
        Self::default()
    }

    /// Set the normal module `mn` (tooth size, in mm).
    pub fn module(mut self, module: Module) -> Self {
        self.module = Some(module);
        self
    }

    /// Set the number of teeth.
    pub fn teeth(mut self, teeth: u32) -> Self {
        self.teeth = Some(teeth);
        self
    }

    /// Set the helix angle `ψ` in degrees. Must be in the range `(0°, 90°)`.
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

    /// Set the normal pressure angle `αn` in degrees. Defaults to `20.0°` if not called.
    pub fn normal_pressure_angle(mut self, degrees: f64) -> Self {
        self.normal_pressure_angle = Some(degrees);
        self
    }

    /// Set the face width in millimetres.
    pub fn face_width(mut self, mm: f64) -> Self {
        self.face_width = Some(mm);
        self
    }

    /// Build the [`HelicalGear`], validating all inputs.
    ///
    /// # Errors
    ///
    /// | Error | Cause |
    /// |---|---|
    /// | [`GearError::ModuleRequired`] | `.module()` was not called |
    /// | [`GearError::TeethRequired`] | `.teeth()` was not called |
    /// | [`GearError::HelixAngleRequired`] | `.helix_angle()` was not called |
    /// | [`GearError::HelixHandRequired`] | `.helix_hand()` was not called |
    /// | [`GearError::ModuleMustBePositive`] | module value ≤ 0 |
    /// | [`GearError::TeethMustBePositive`] | teeth = 0 |
    /// | [`GearError::HelixAngleMustBePositive`] | helix angle ≤ 0° |
    /// | [`GearError::HelixAngleMustBeLessThan90`] | helix angle ≥ 90° |
    /// | [`GearError::PressureAngleMustBePositive`] | normal pressure angle ≤ 0° |
    /// | [`GearError::FaceWidthMustBePositive`] | face width ≤ 0 mm |
    pub fn build(self) -> Result<HelicalGear, GearError> {
        let module = self.module.ok_or(GearError::ModuleRequired)?;
        let teeth = self.teeth.ok_or(GearError::TeethRequired)?;
        let helix_angle = self.helix_angle.ok_or(GearError::HelixAngleRequired)?;
        let helix_hand = self.helix_hand.ok_or(GearError::HelixHandRequired)?;

        if module.value() <= 0.0 {
            return Err(GearError::ModuleMustBePositive);
        }
        if teeth == 0 {
            return Err(GearError::TeethMustBePositive);
        }
        if helix_angle <= 0.0 {
            return Err(GearError::HelixAngleMustBePositive);
        }
        if helix_angle >= 90.0 {
            return Err(GearError::HelixAngleMustBeLessThan90);
        }

        let normal_pressure_angle = self
            .normal_pressure_angle
            .unwrap_or(DEFAULT_NORMAL_PRESSURE_ANGLE);
        if normal_pressure_angle <= 0.0 {
            return Err(GearError::PressureAngleMustBePositive);
        }

        if self.face_width.is_some_and(|fw| fw <= 0.0) {
            return Err(GearError::FaceWidthMustBePositive);
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
