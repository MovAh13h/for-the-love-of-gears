//! Gear scene — compose shafts, mount gears, and simulate kinematics.
//!
//! # Core concepts
//!
//! ## Shafts, gears, and meshes
//!
//! A **shaft** is a rigid rotating axis. All gears mounted on the same shaft
//! are keyed to it and turn at exactly the same angular velocity — this
//! constraint is what makes compound gear trains possible.
//!
//! A **mesh** is a physical contact between one gear on one shaft and one gear
//! on a different shaft. Each mesh transfers motion and multiplies (or divides)
//! speed by the ratio of the two tooth counts.
//!
//! The **driver** shaft is the one receiving external power. Its speed is
//! specified at simulation time; every other shaft's speed is derived
//! automatically by following the mesh graph.
//!
//! ## How speed propagates — BFS over the mesh graph
//!
//! At build time, `GearSceneBuilder::build` performs a **breadth-first search**
//! starting at the driver shaft. For each mesh edge it visits:
//!
//! ```text
//! rpm_neighbor = rpm_current × (teeth_current / teeth_neighbor)
//! direction_neighbor = direction_current.flip()
//! ```
//!
//! The driver is assigned a normalised relative RPM of `1.0` and direction
//! `Clockwise`. All other shafts are expressed as multiples of the driver.
//! When `run(driver_rpm)` is called, every shaft's absolute RPM is obtained by
//! a simple multiplication — the topology work is done once, not every call.
//!
//! ## Compound gear trains
//!
//! A **compound shaft** (two gears on the same shaft) multiplies the reduction
//! ratios of adjacent stages:
//!
//! ```text
//! Stage 1: shaft_A (20t) → shaft_B (40t)   ratio = 40/20 = 2:1
//! Stage 2: shaft_B (20t) → shaft_C (60t)   ratio = 60/20 = 3:1
//! Overall:                                   total = 2 × 3 = 6:1
//! ```
//!
//! Both gears on shaft_B rotate together. The small gear on B drives the large
//! gear on C — its speed is already 1/2 the input, and the second stage
//! reduces it by another factor of 3.
//!
//! ## What the simulation computes
//!
//! [`GearSimulation`] answers purely **kinematic** questions — speed, angle,
//! direction, and time. It does not model forces, torques, stresses, lubrication,
//! or bearing loads. For those you need a separate mechanical analysis.
//!
//! Angular position at time `t` is exact — it is computed from the constant
//! RPM, not by numerical integration:
//!
//! ```text
//! angle_deg(t) = (rpm / 60 × t × 360°)  mod  360°
//! ```
//!
//! ## Direction convention
//!
//! The driver shaft is defined as [`Direction::Clockwise`]. Each mesh reverses
//! direction: the driven shaft in a single-stage pair is `CounterClockwise`.
//! In a two-stage compound train, the output is `Clockwise` again (two
//! reversals cancel). This matches real gear trains viewed from a fixed
//! vantage point along the shaft axis.
//!
//! # Quick start — simple pair
//!
//! ```
//! use for_the_love_of_gears::{
//!     gear::Gear,
//!     scene::{AnyGear, GearScene},
//! };
//!
//! let driver_gear = Gear::builder().module(2.0).teeth(20).build().unwrap();
//! let driven_gear = Gear::builder().module(2.0).teeth(40).build().unwrap();
//!
//! let scene = GearScene::builder()
//!     .shaft("input",  vec![("a", AnyGear::from(driver_gear))])
//!     .shaft("output", vec![("b", AnyGear::from(driven_gear))])
//!     .mesh("a", "b")
//!     .driver("input")
//!     .build()
//!     .unwrap();
//!
//! let sim = scene.run(1000.0).unwrap(); // 1000 rpm input
//! assert_eq!(sim.rpm("output"), Some(500.0));  // 2:1 reduction
//! assert_eq!(sim.ratio_to("output"), Some(2.0));
//! ```
//!
//! # Quick start — compound train
//!
//! ```
//! use for_the_love_of_gears::{
//!     gear::Gear,
//!     scene::{AnyGear, GearScene},
//! };
//!
//! // Stage 1: 20t → 40t (2:1).  Stage 2: 20t → 60t (3:1).  Total: 6:1.
//! let ga = Gear::builder().module(2.0).teeth(20).build().unwrap();
//! let gb = Gear::builder().module(2.0).teeth(40).build().unwrap();
//! let gc = Gear::builder().module(3.0).teeth(20).build().unwrap();
//! let gd = Gear::builder().module(3.0).teeth(60).build().unwrap();
//!
//! let scene = GearScene::builder()
//!     .shaft("input",        vec![("a", AnyGear::from(ga))])
//!     .shaft("intermediate", vec![("b", AnyGear::from(gb)), ("c", AnyGear::from(gc))])
//!     .shaft("output",       vec![("d", AnyGear::from(gd))])
//!     .mesh("a", "b")   // stage 1
//!     .mesh("c", "d")   // stage 2
//!     .driver("input")
//!     .build()
//!     .unwrap();
//!
//! let sim = scene.run(1000.0).unwrap();
//! assert!((sim.rpm("output").unwrap() - 1000.0 / 6.0).abs() < 1e-9);
//! assert!((sim.ratio_to("output").unwrap() - 6.0).abs() < 1e-9);
//! ```

use std::{
    collections::{HashMap, VecDeque},
    fmt,
};

use crate::{gear::Gear, helical::HelicalGear, traits::GearGeometry};

fn sorted_keys<V>(map: &HashMap<String, V>) -> Vec<&str> {
    let mut names: Vec<&str> = map.keys().map(|s| s.as_str()).collect();
    names.sort();
    names
}

#[derive(Debug, Clone, Copy)]
struct ShaftState {
    rpm: f64,
    direction: Direction,
}

/// A type-erased gear that can be mounted in a [`GearScene`] — either spur or helical.
///
/// [`GearScene`] needs to store gears of any type in a single collection.
/// `AnyGear` is the common container. Convert a concrete gear with
/// [`AnyGear::from`]:
///
/// ```
/// use for_the_love_of_gears::{
///     gear::Gear,
///     helical::{HelicalGear, HelixHand},
///     scene::AnyGear,
/// };
///
/// let spur = Gear::builder().module(2.0).teeth(20).build().unwrap();
/// let helical = HelicalGear::builder()
///     .module(2.0).teeth(30).helix_angle(20.0).helix_hand(HelixHand::Right)
///     .build().unwrap();
///
/// let a: AnyGear = AnyGear::from(spur);
/// let b: AnyGear = AnyGear::from(helical);
/// ```
///
/// For geometric queries on `AnyGear` use the [`GearGeometry`] trait, which
/// is implemented for this type.
///
/// [`GearGeometry`]: crate::traits::GearGeometry
#[derive(Debug, Clone)]
pub enum AnyGear {
    /// A standard spur gear.
    Spur(Gear),
    /// A helical gear.
    Helical(HelicalGear),
}

impl AnyGear {
    /// Number of teeth on this gear.
    ///
    /// Used by the scene builder to compute the mesh ratio between two shafts:
    /// `rpm_b = rpm_a × (teeth_a / teeth_b)`.
    pub fn teeth(&self) -> u32 {
        match self {
            Self::Spur(g) => g.teeth(),
            Self::Helical(g) => g.teeth(),
        }
    }

    /// Normal module in mm.
    ///
    /// For spur gears this is [`Gear::module`]; for helical gears it is
    /// [`HelicalGear::normal_module`]. The scene builder uses the module
    /// indirectly via `can_mesh_with` — two gears can only form a mesh if
    /// their modules (and other parameters) match.
    pub fn module(&self) -> f64 {
        match self {
            Self::Spur(g) => g.module(),
            Self::Helical(g) => g.normal_module(),
        }
    }

    fn can_mesh_with(&self, other: &AnyGear) -> bool {
        match (self, other) {
            (Self::Spur(a), Self::Spur(b)) => a.can_mesh_with(b),
            (Self::Helical(a), Self::Helical(b)) => a.can_mesh_with(b),
            _ => false,
        }
    }

    /// Transverse contact ratio `εα` for a meshing pair.
    ///
    /// Returns `None` if the two gears are of incompatible types (spur vs
    /// helical) — they cannot physically mesh. For helical pairs this is
    /// the transverse contact ratio only; add [`HelicalGear::overlap_ratio`]
    /// for the total `εγ`.
    ///
    /// ```
    /// use for_the_love_of_gears::{gear::Gear, scene::AnyGear};
    ///
    /// let a = AnyGear::from(Gear::builder().module(2.0).teeth(20).build().unwrap());
    /// let b = AnyGear::from(Gear::builder().module(2.0).teeth(40).build().unwrap());
    /// let cr = a.contact_ratio_with(&b);
    /// assert!(cr.is_some());
    /// assert!(cr.unwrap() > 1.0);
    /// ```
    pub fn contact_ratio_with(&self, other: &AnyGear) -> Option<f64> {
        match (self, other) {
            (Self::Spur(a), Self::Spur(b)) => Some(a.contact_ratio_with(b)),
            (Self::Helical(a), Self::Helical(b)) => Some(a.transverse_contact_ratio_with(b)),
            _ => None,
        }
    }
}

impl From<Gear> for AnyGear {
    fn from(g: Gear) -> Self {
        Self::Spur(g)
    }
}

impl From<HelicalGear> for AnyGear {
    fn from(g: HelicalGear) -> Self {
        Self::Helical(g)
    }
}

impl GearGeometry for AnyGear {
    fn teeth(&self) -> u32 {
        match self {
            Self::Spur(g) => g.teeth(),
            Self::Helical(g) => g.teeth(),
        }
    }

    fn normal_module(&self) -> f64 {
        match self {
            Self::Spur(g) => g.normal_module(),
            Self::Helical(g) => g.normal_module(),
        }
    }

    fn reference_diameter(&self) -> f64 {
        match self {
            Self::Spur(g) => g.reference_diameter(),
            Self::Helical(g) => g.reference_diameter(),
        }
    }

    fn tip_diameter(&self) -> f64 {
        match self {
            Self::Spur(g) => g.tip_diameter(),
            Self::Helical(g) => g.tip_diameter(),
        }
    }

    fn root_diameter(&self) -> f64 {
        match self {
            Self::Spur(g) => g.root_diameter(),
            Self::Helical(g) => g.root_diameter(),
        }
    }

    fn base_diameter(&self) -> f64 {
        match self {
            Self::Spur(g) => g.base_diameter(),
            Self::Helical(g) => g.base_diameter(),
        }
    }

    fn thinned_tooth_thickness(&self, backlash_mm: f64) -> Result<f64, crate::BacklashError> {
        match self {
            Self::Spur(g) => g.thinned_tooth_thickness(backlash_mm),
            Self::Helical(g) => g.thinned_tooth_thickness(backlash_mm),
        }
    }

    fn normal_backlash(&self, backlash_mm: f64) -> Result<f64, crate::BacklashError> {
        match self {
            Self::Spur(g) => g.normal_backlash(backlash_mm),
            Self::Helical(g) => g.normal_backlash(backlash_mm),
        }
    }
}

/// Rotation direction of a shaft, defined relative to the driver shaft.
///
/// The driver shaft is defined as [`Clockwise`]. Every mesh edge flips the
/// direction: the immediate driven shaft is `CounterClockwise`; the shaft
/// after that is `Clockwise` again, and so on.
///
/// This matches real gear trains when viewed from a fixed vantage point along
/// the shaft axes:
///
/// ```text
/// input (CW) ──mesh──> intermediate (CCW) ──mesh──> output (CW)
/// ```
///
/// The actual physical direction (which way "clockwise" is) depends on the
/// viewing angle and mounting — the library treats the driver as the
/// reference and tracks relative reversals.
///
/// [`Clockwise`]: Direction::Clockwise
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Same rotation sense as the driver shaft.
    ///
    /// The driver itself, and every shaft reached via an even number of mesh
    /// edges from the driver.
    Clockwise,
    /// Opposite rotation sense to the driver shaft.
    ///
    /// Every shaft reached via an odd number of mesh edges from the driver.
    CounterClockwise,
}

impl Direction {
    #[must_use]
    fn flip(self) -> Self {
        match self {
            Self::Clockwise => Self::CounterClockwise,
            Self::CounterClockwise => Self::Clockwise,
        }
    }
}

/// Errors returned when building or running a [`GearScene`].
#[derive(Debug, PartialEq)]
#[non_exhaustive]
pub enum GearSceneError {
    /// Two shafts were given the same name.
    ///
    /// Every shaft in a scene must have a unique string identifier. Duplicate
    /// names make it impossible to unambiguously reference a shaft in mesh
    /// definitions, driver selection, or simulation queries.
    DuplicateShaftName(String),

    /// Two gears were given the same name (gear names must be unique across the whole scene).
    ///
    /// Gear names are used in `.mesh()` calls to connect shafts. If two gears
    /// share a name, the simulator cannot determine which gear a mesh refers to.
    DuplicateGearName(String),

    /// No driver shaft was specified via `.driver()`.
    ///
    /// The driver shaft is the entry point for power and motion. Without one,
    /// the simulator has no starting RPM and no direction reference — it cannot
    /// propagate rotation to any other shaft.
    DriverShaftRequired,

    /// The driver shaft name does not match any shaft in the scene.
    ///
    /// The string is the name that was passed to `.driver()`. Check for
    /// typos or ensure the shaft was added before calling `.driver()`.
    DriverShaftNotFound(String),

    /// A gear referenced in a `.mesh()` call does not exist.
    ///
    /// The string is the unrecognised gear name. Check that you spelled the
    /// gear name correctly and that it was mounted on a shaft before being
    /// used in a mesh.
    GearNotFound(String),

    /// Both gears in a `.mesh()` call are on the same shaft.
    ///
    /// A gear cannot mesh with another gear on the same shaft — they rotate
    /// together and transmit no force between them. Connect gears on
    /// *different* shafts to transfer motion.
    MeshGearsOnSameShaft {
        /// Name of the first gear in the mesh.
        gear_a: String,
        /// Name of the second gear in the mesh.
        gear_b: String,
    },

    /// The two gears in a mesh are of different types (one spur, one helical).
    ///
    /// A spur gear and a helical gear have fundamentally different tooth
    /// geometries and cannot be paired. Use two spur gears or two helical
    /// gears with matching parameters and opposing helix hands.
    MeshTypeMismatch {
        /// Name of the first gear in the mesh.
        gear_a: String,
        /// Name of the second gear in the mesh.
        gear_b: String,
    },

    /// The two gears in a mesh share the same type but have incompatible
    /// parameters (different module, pressure angle, or — for helical gears —
    /// helix angle or same helix hand).
    ///
    /// For a mesh to be physically valid, the two gears must pass
    /// [`Gear::can_mesh_with`] or [`HelicalGear::can_mesh_with`].
    ///
    /// [`Gear::can_mesh_with`]: crate::gear::Gear::can_mesh_with
    /// [`HelicalGear::can_mesh_with`]: crate::helical::HelicalGear::can_mesh_with
    MeshParameterMismatch {
        /// Name of the first gear in the mesh.
        gear_a: String,
        /// Name of the second gear in the mesh.
        gear_b: String,
    },

    /// A shaft has no mesh connections leading back to the driver shaft.
    ///
    /// The string is the name of the disconnected shaft. Every shaft in the
    /// scene must be reachable from the driver through a chain of meshes.
    /// A disconnected shaft would receive no driving force and its RPM would
    /// be undefined.
    DisconnectedShaft(String),

    /// A shaft is reachable via two different mesh paths that imply conflicting RPM values.
    ///
    /// This happens in closed kinematic loops — for example, when gear A meshes
    /// with both B and C, and B also meshes with C. The loop constrains all
    /// three ratios simultaneously. Only chains (open paths from the driver)
    /// are currently supported.
    OverConstrainedShaft(String),

    /// Driver RPM must be strictly greater than zero.
    ///
    /// A non-positive driver RPM has no physical meaning — zero means the
    /// gear train is stationary and negative means reverse direction (not
    /// currently modelled). Pass a positive value to [`GearScene::run`].
    DriverRpmMustBePositive,
}

impl std::error::Error for GearSceneError {}

impl fmt::Display for GearSceneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateShaftName(s) => write!(f, "duplicate shaft name: '{s}'"),
            Self::DuplicateGearName(s) => write!(f, "duplicate gear name: '{s}'"),
            Self::DriverShaftRequired => {
                write!(f, "a driver shaft must be specified via .driver()")
            }
            Self::DriverShaftNotFound(s) => write!(f, "driver shaft not found: '{s}'"),
            Self::GearNotFound(s) => write!(f, "gear not found: '{s}'"),
            Self::MeshGearsOnSameShaft { gear_a, gear_b } => {
                write!(f, "gears '{gear_a}' and '{gear_b}' are on the same shaft")
            }
            Self::MeshTypeMismatch { gear_a, gear_b } => write!(
                f,
                "gears '{gear_a}' and '{gear_b}' cannot mesh — one is spur, the other helical"
            ),
            Self::MeshParameterMismatch { gear_a, gear_b } => write!(
                f,
                "gears '{gear_a}' and '{gear_b}' cannot mesh — incompatible module, pressure angle, or helix parameters"
            ),
            Self::DisconnectedShaft(s) => {
                write!(
                    f,
                    "shaft '{s}' is not connected to the driver via any mesh path"
                )
            }
            Self::OverConstrainedShaft(s) => write!(
                f,
                "shaft '{s}' is over-constrained — two mesh paths imply different RPM values"
            ),
            Self::DriverRpmMustBePositive => write!(f, "driver RPM must be greater than zero"),
        }
    }
}

#[derive(Debug)]
struct ShaftData {
    gears: Vec<(String, AnyGear)>,
}

/// A validated gear scene: shafts with mounted gears and mesh connections between them.
///
/// Build with [`GearScene::builder()`] and simulate with [`GearScene::run()`].
///
/// The scene is **immutable after build**: the topology is validated once and
/// relative speeds are pre-computed. Calling `run` at different driver RPMs is
/// cheap — it simply scales the pre-computed relative values. There is no
/// need to rebuild the scene to change speed.
///
/// See the [module-level documentation](crate::scene) for full examples and
/// an explanation of how speed propagates through the mesh graph.
#[derive(Debug)]
pub struct GearScene {
    driver_shaft: String,
    states: HashMap<String, ShaftState>,
    shafts: HashMap<String, Vec<(String, AnyGear)>>,
    meshes: Vec<(String, String)>,
}

impl GearScene {
    /// Start building a [`GearScene`].
    pub fn builder() -> GearSceneBuilder {
        GearSceneBuilder::default()
    }

    /// Run the scene at `driver_rpm` revolutions per minute.
    ///
    /// Returns a [`GearSimulation`] from which you can query RPM, direction,
    /// angle, total rotations, and animation frames for any shaft.
    ///
    /// Internally, each shaft's relative RPM (pre-computed at build time as a
    /// multiple of the driver) is simply multiplied by `driver_rpm` — this is
    /// an O(n) scale over the number of shafts, not another graph traversal.
    ///
    /// The same scene can be run at different speeds without rebuilding:
    ///
    /// ```
    /// # use for_the_love_of_gears::{gear::Gear, scene::{AnyGear, GearScene}};
    /// # let scene = GearScene::builder()
    /// #     .shaft("input",  vec![("a", AnyGear::from(Gear::builder().module(2.0).teeth(20).build().unwrap()))])
    /// #     .shaft("output", vec![("b", AnyGear::from(Gear::builder().module(2.0).teeth(40).build().unwrap()))])
    /// #     .mesh("a", "b").driver("input").build().unwrap();
    /// let slow = scene.run(500.0).unwrap();
    /// let fast = scene.run(3000.0).unwrap();
    /// assert_eq!(slow.rpm("output"),  Some(250.0));
    /// assert_eq!(fast.rpm("output"),  Some(1500.0));
    /// ```
    ///
    /// # Errors
    ///
    /// Returns [`GearSceneError::DriverRpmMustBePositive`] if `driver_rpm <= 0`.
    pub fn run(&self, driver_rpm: f64) -> Result<GearSimulation, GearSceneError> {
        if driver_rpm <= 0.0 {
            return Err(GearSceneError::DriverRpmMustBePositive);
        }

        let states: HashMap<String, ShaftState> = self
            .states
            .iter()
            .map(|(shaft, s)| {
                (shaft.clone(), ShaftState { rpm: s.rpm * driver_rpm, direction: s.direction })
            })
            .collect();

        let mut shaft_order: Vec<String> = states.keys().cloned().collect();
        shaft_order.sort();

        Ok(GearSimulation {
            states,
            shaft_order,
            driver_shaft: self.driver_shaft.clone(),
            driver_rpm,
        })
    }

    /// The names of all shafts in this scene, in **sorted** (alphabetical) order.
    ///
    /// The sorted order is stable regardless of the order shafts were added to
    /// the builder, making it safe to iterate and display results predictably.
    pub fn shaft_names(&self) -> Vec<&str> {
        sorted_keys(&self.states)
    }

    /// The gears mounted on `shaft`, in the order they were added.
    ///
    /// Returns `None` if `shaft` does not exist in the scene.
    /// Each element is a `(gear_name, gear)` pair.
    pub fn gears_on_shaft(&self, shaft: &str) -> Option<&[(String, AnyGear)]> {
        self.shafts.get(shaft).map(|v| v.as_slice())
    }

    /// Look up a single gear by name, returning a reference to it.
    ///
    /// Returns `None` if no gear with that name exists in the scene.
    pub fn gear(&self, gear_name: &str) -> Option<&AnyGear> {
        for gears in self.shafts.values() {
            if let Some((_, g)) = gears.iter().find(|(n, _)| n == gear_name) {
                return Some(g);
            }
        }
        None
    }

    /// All mesh pairs in the scene, as `(gear_a_name, gear_b_name)` tuples.
    pub fn meshes(&self) -> &[(String, String)] {
        &self.meshes
    }
}

/// Builder for [`GearScene`].
///
/// Obtain via [`GearScene::builder()`]. The typical call sequence is:
/// 1. Call `.shaft()` once per shaft (add gears at the same time)
/// 2. Call `.mesh()` for every gear pair that physically contacts
/// 3. Call `.driver()` to designate the power-input shaft
/// 4. Call `.build()` to validate and produce a [`GearScene`]
#[derive(Debug, Default)]
pub struct GearSceneBuilder {
    shafts: Vec<(String, Vec<(String, AnyGear)>)>,
    meshes: Vec<(String, String)>,
    driver_shaft: Option<String>,
}

impl GearSceneBuilder {
    /// Add a named shaft carrying the listed gears.
    ///
    /// `name` is the shaft identifier used in `.driver()` and in simulation
    /// queries. Each gear entry is `(gear_name, gear)` — the gear name is
    /// used only in `.mesh()` calls.
    ///
    /// **Gear names are global** across the whole scene: no two gears on any
    /// shaft may share a name, because mesh declarations reference gears by
    /// name and ambiguous names would make the scene undefined.
    ///
    /// **Compound shafts**: passing more than one gear for a shaft creates a
    /// compound shaft. All gears on the shaft rotate together. The small gear
    /// on the compound shaft is driven by the preceding stage, and its large
    /// partner drives the next stage, multiplying the overall reduction:
    ///
    /// ```
    /// # use for_the_love_of_gears::{gear::Gear, scene::{AnyGear, GearScene}};
    /// // Compound intermediate shaft: 40t receives input, 20t drives output.
    /// GearScene::builder()
    ///     .shaft("input",        vec![("a", AnyGear::from(Gear::builder().module(2.0).teeth(20).build().unwrap()))])
    ///     .shaft("intermediate", vec![
    ///         ("b", AnyGear::from(Gear::builder().module(2.0).teeth(40).build().unwrap())),
    ///         ("c", AnyGear::from(Gear::builder().module(3.0).teeth(20).build().unwrap())),
    ///     ])
    ///     .shaft("output",       vec![("d", AnyGear::from(Gear::builder().module(3.0).teeth(60).build().unwrap()))])
    ///     .mesh("a", "b")
    ///     .mesh("c", "d")
    ///     .driver("input")
    ///     .build()
    ///     .unwrap();
    /// ```
    pub fn shaft<S: Into<String>>(mut self, name: &str, gears: Vec<(S, AnyGear)>) -> Self {
        self.shafts.push((
            name.to_string(),
            gears.into_iter().map(|(n, g)| (n.into(), g)).collect(),
        ));
        self
    }

    /// Declare that two gears (identified by name) are in physical contact.
    ///
    /// Both gear names must have been registered via `.shaft()`. The two gears
    /// must be on **different** shafts and must pass the compatibility check of
    /// their underlying type (same module and pressure angle for spur gears;
    /// same module, pressure angle, helix angle, and opposite helix hand for
    /// helical gears). A spur gear and a helical gear cannot mesh.
    ///
    /// Mesh order does not matter — `.mesh("a", "b")` and `.mesh("b", "a")`
    /// produce the same result.
    pub fn mesh(mut self, gear_a: &str, gear_b: &str) -> Self {
        self.meshes.push((gear_a.to_string(), gear_b.to_string()));
        self
    }

    /// Designate which shaft receives the external input power.
    ///
    /// The driver shaft speed is specified when calling [`GearScene::run`].
    /// Every other shaft's speed is derived from it. Exactly one driver shaft
    /// is required — calling this method twice overwrites the previous choice.
    pub fn driver(mut self, shaft: &str) -> Self {
        self.driver_shaft = Some(shaft.to_string());
        self
    }

    /// Build and validate the [`GearScene`].
    ///
    /// # Validation steps
    ///
    /// 1. Check for duplicate shaft names and duplicate gear names.
    /// 2. Verify the driver shaft name matches a declared shaft.
    /// 3. For each mesh, verify both gears exist and are on different shafts,
    ///    then call the appropriate `can_mesh_with` to check geometric
    ///    compatibility.
    /// 4. Run a **BFS** from the driver shaft, computing the relative RPM
    ///    (`rpm_neighbor = rpm_current × teeth_current / teeth_neighbor`) and
    ///    direction (`flip` at each mesh) for every reachable shaft.
    /// 5. Detect disconnected shafts (never reached by BFS) and
    ///    over-constrained shafts (reached via two paths with inconsistent RPMs).
    ///
    /// On success, the relative RPMs and directions are stored. [`GearScene::run`]
    /// just multiplies them by the chosen driver RPM.
    pub fn build(self) -> Result<GearScene, GearSceneError> {
        let driver_shaft = self
            .driver_shaft
            .ok_or(GearSceneError::DriverShaftRequired)?;

        // --- Validate duplicate shaft names ---
        let mut seen_shafts = std::collections::HashSet::new();
        for (name, _) in &self.shafts {
            if !seen_shafts.insert(name.as_str()) {
                return Err(GearSceneError::DuplicateShaftName(name.clone()));
            }
        }

        // --- Build maps ---
        let mut shafts: HashMap<String, ShaftData> = HashMap::new();
        let mut gear_to_shaft: HashMap<String, String> = HashMap::new();

        for (shaft_name, gears) in self.shafts {
            for (gear_name, _) in &gears {
                if gear_to_shaft.contains_key(gear_name) {
                    return Err(GearSceneError::DuplicateGearName(gear_name.clone()));
                }
                gear_to_shaft.insert(gear_name.clone(), shaft_name.clone());
            }
            shafts.insert(shaft_name, ShaftData { gears });
        }

        // --- Validate driver shaft exists ---
        if !shafts.contains_key(&driver_shaft) {
            return Err(GearSceneError::DriverShaftNotFound(driver_shaft));
        }

        // --- Validate meshes and build shaft adjacency list ---
        // shaft_edges[shaft] = Vec<(other_shaft, ratio)>  where ratio = rpm_other / rpm_self
        let mut shaft_edges: HashMap<String, Vec<(String, f64)>> = HashMap::new();

        for (gear_a, gear_b) in &self.meshes {
            let shaft_a = gear_to_shaft
                .get(gear_a)
                .ok_or_else(|| GearSceneError::GearNotFound(gear_a.clone()))?
                .clone();
            let shaft_b = gear_to_shaft
                .get(gear_b)
                .ok_or_else(|| GearSceneError::GearNotFound(gear_b.clone()))?
                .clone();

            if shaft_a == shaft_b {
                return Err(GearSceneError::MeshGearsOnSameShaft {
                    gear_a: gear_a.clone(),
                    gear_b: gear_b.clone(),
                });
            }

            let g_a = shafts[&shaft_a]
                .gears
                .iter()
                .find(|(n, _)| n == gear_a)
                .map(|(_, g)| g)
                .unwrap();
            let g_b = shafts[&shaft_b]
                .gears
                .iter()
                .find(|(n, _)| n == gear_b)
                .map(|(_, g)| g)
                .unwrap();

            if !g_a.can_mesh_with(g_b) {
                let err = if matches!(
                    (g_a, g_b),
                    (AnyGear::Spur(_), AnyGear::Helical(_)) | (AnyGear::Helical(_), AnyGear::Spur(_))
                ) {
                    GearSceneError::MeshTypeMismatch {
                        gear_a: gear_a.clone(),
                        gear_b: gear_b.clone(),
                    }
                } else {
                    GearSceneError::MeshParameterMismatch {
                        gear_a: gear_a.clone(),
                        gear_b: gear_b.clone(),
                    }
                };
                return Err(err);
            }

            let ta = g_a.teeth() as f64;
            let tb = g_b.teeth() as f64;

            // rpm_b = rpm_a * (ta / tb)
            shaft_edges
                .entry(shaft_a.clone())
                .or_default()
                .push((shaft_b.clone(), ta / tb));
            // rpm_a = rpm_b * (tb / ta)
            shaft_edges
                .entry(shaft_b.clone())
                .or_default()
                .push((shaft_a.clone(), tb / ta));
        }

        // --- BFS: compute relative RPMs and directions ---
        // Driver shaft is normalised to relative RPM = 1.0 and Direction::Clockwise.
        let mut states: HashMap<String, ShaftState> = HashMap::new();
        states.insert(
            driver_shaft.clone(),
            ShaftState { rpm: 1.0, direction: Direction::Clockwise },
        );

        let mut queue: VecDeque<String> = VecDeque::new();
        queue.push_back(driver_shaft.clone());

        while let Some(current) = queue.pop_front() {
            let current_state = states[&current];

            if let Some(edges) = shaft_edges.get(&current) {
                for (neighbor, ratio) in edges {
                    let neighbor_rpm = current_state.rpm * ratio;
                    let neighbor_dir = current_state.direction.flip();

                    if let Some(existing) = states.get(neighbor) {
                        if (existing.rpm - neighbor_rpm).abs() > crate::MESH_TOLERANCE {
                            return Err(GearSceneError::OverConstrainedShaft(neighbor.clone()));
                        }
                    } else {
                        states.insert(
                            neighbor.clone(),
                            ShaftState { rpm: neighbor_rpm, direction: neighbor_dir },
                        );
                        queue.push_back(neighbor.clone());
                    }
                }
            }
        }

        // --- Ensure every shaft is reachable ---
        for shaft_name in shafts.keys() {
            if !states.contains_key(shaft_name) {
                return Err(GearSceneError::DisconnectedShaft(shaft_name.clone()));
            }
        }

        let shafts_data: HashMap<String, Vec<(String, AnyGear)>> = shafts
            .into_iter()
            .map(|(name, sd)| (name, sd.gears))
            .collect();

        Ok(GearScene {
            driver_shaft,
            states,
            shafts: shafts_data,
            meshes: self.meshes,
        })
    }
}

/// The kinematic result of running a [`GearScene`] at a given driver RPM.
///
/// All queries are `O(1)` — RPM and direction are pre-computed at build time;
/// `run(driver_rpm)` just scales them. Angular position at any time `t` is
/// computed from the constant RPM directly, with no numerical integration:
///
/// ```text
/// angle_deg(t) = (rpm / 60 × t × 360°)  mod  360°
/// ```
///
/// All `shaft` parameters accept the names declared in [`GearSceneBuilder::shaft`].
/// Methods return `None` rather than panicking when a shaft name is not found.
#[derive(Debug)]
pub struct GearSimulation {
    states: HashMap<String, ShaftState>,
    shaft_order: Vec<String>,
    driver_shaft: String,
    driver_rpm: f64,
}

impl GearSimulation {
    /// Revolutions per minute of the named shaft.
    ///
    /// For the driver shaft this is the RPM passed to [`GearScene::run`]. For
    /// all other shafts it is derived from the gear ratios along the mesh path:
    ///
    /// ```text
    /// rpm_shaft = driver_rpm × ∏(teeth_driving / teeth_driven)  along the path
    /// ```
    ///
    /// Returns `None` if `shaft` is not a known shaft name.
    pub fn rpm(&self, shaft: &str) -> Option<f64> {
        self.states.get(shaft).map(|s| s.rpm)
    }

    /// Rotation direction of the named shaft relative to the driver.
    ///
    /// The driver is always [`Direction::Clockwise`]. Each mesh edge in the
    /// path from the driver to this shaft flips the direction. See
    /// [`Direction`] for the full convention.
    ///
    /// Returns `None` if `shaft` is not a known shaft name.
    pub fn direction(&self, shaft: &str) -> Option<Direction> {
        self.states.get(shaft).map(|s| s.direction)
    }

    /// Total rotations completed over `duration_secs` seconds (dimensionless count).
    ///
    /// Formula: `rotations = rpm × duration_secs / 60`.
    ///
    /// A shaft at 1 800 rpm completes `1800 × 5 / 60 = 150` full turns in 5 s.
    /// `duration_secs` is in **seconds**; the result is a plain count of full revolutions.
    ///
    /// Returns `None` if `shaft` is not a known shaft name.
    pub fn total_rotations(&self, shaft: &str, duration_secs: f64) -> Option<f64> {
        self.rpm(shaft).map(|r| r * duration_secs / 60.0)
    }

    /// Angular position of the shaft at time `t` (**seconds**), in degrees `[0°, 360°)`.
    ///
    /// All shafts start at `0°` when `t = 0`. The angle advances linearly with
    /// time and wraps using `rem_euclid` so the result is always in `[0, 360)`:
    ///
    /// ```text
    /// angle = (rpm / 60 × t × 360°)  mod  360°
    /// ```
    ///
    /// Returns `None` if `shaft` is not a known shaft name.
    pub fn angle_deg(&self, shaft: &str, t: f64) -> Option<f64> {
        self.rpm(shaft)
            .map(|r| (r / 60.0 * t * 360.0).rem_euclid(360.0))
    }

    /// Angular velocity of the named shaft in radians per second.
    ///
    /// Formula: `ω = rpm × 2π / 60`.
    ///
    /// This is the SI unit angular velocity. Use it when interfacing with
    /// physics engines or dynamics calculations that require rad/s.
    ///
    /// Returns `None` if `shaft` is not a known shaft name.
    pub fn angular_velocity_rad_s(&self, shaft: &str) -> Option<f64> {
        self.rpm(shaft).map(|r| r * std::f64::consts::TAU / 60.0)
    }

    /// Overall speed ratio from the driver to the named shaft:
    /// `i = driver_rpm / shaft_rpm` (dimensionless).
    ///
    /// | Value | Meaning |
    /// |---|---|
    /// | `> 1.0` | Speed reduction — shaft is slower than driver |
    /// | `= 1.0` | Direct drive — shaft and driver turn at the same speed |
    /// | `< 1.0` | Speed increase — shaft is faster than driver |
    ///
    /// For a 6:1 compound train the output shaft returns `6.0` regardless of
    /// the actual driver RPM.
    ///
    /// Returns `None` if `shaft` is not a known shaft name.
    pub fn ratio_to(&self, shaft: &str) -> Option<f64> {
        self.rpm(shaft).map(|r| self.driver_rpm / r)
    }

    /// Generate animation keyframes at `fps` frames per second over `duration_secs`.
    ///
    /// Each [`SimFrame`] holds the angular position of every shaft at one
    /// instant. The frames are useful for driving gear-train visualisations:
    /// feed each frame's `shaft_angles` map to your renderer on each tick.
    ///
    /// Frame timestamps:
    /// - First frame: `t = 0` (all shafts at 0°)
    /// - Subsequent frames: `t = 1/fps, 2/fps, …`
    /// - Last frame: exactly `duration_secs` (clamped, not extrapolated)
    ///
    /// Total frame count: `⌈duration_secs × fps⌉ + 1`.
    ///
    /// Shaft angles within each frame are stored in a `Vec<f64>` in the same
    /// sorted (alphabetical) order as [`shaft_names`]. Use [`shaft_index`] to
    /// convert a shaft name to its index once, then index into every frame's
    /// `shaft_angles` by that index.
    ///
    /// [`shaft_names`]: GearSimulation::shaft_names
    /// [`shaft_index`]: GearSimulation::shaft_index
    pub fn frames(&self, fps: f64, duration_secs: f64) -> Vec<SimFrame> {
        let frame_count = (duration_secs * fps).ceil() as usize + 1;
        (0..frame_count)
            .map(|i| {
                let t = (i as f64 / fps).min(duration_secs);
                let shaft_angles = self
                    .shaft_order
                    .iter()
                    .map(|s| {
                        let rpm = self.states[s].rpm;
                        (rpm / 60.0 * t * 360.0).rem_euclid(360.0)
                    })
                    .collect();
                SimFrame { time_secs: t, shaft_angles }
            })
            .collect()
    }

    /// The names of all shafts in this simulation, in **sorted** order.
    ///
    /// Mirrors [`GearScene::shaft_names`] so callers don't need to keep the
    /// scene around just to enumerate shafts.
    pub fn shaft_names(&self) -> Vec<&str> {
        sorted_keys(&self.states)
    }

    /// Index of `shaft` in the sorted shaft order used by [`SimFrame::shaft_angles`].
    ///
    /// Use this to convert a shaft name to a `Vec` index without a string lookup
    /// on every frame:
    ///
    /// ```
    /// # use for_the_love_of_gears::{gear::Gear, scene::{AnyGear, GearScene}};
    /// # let scene = GearScene::builder()
    /// #     .shaft("input",  vec![("a", AnyGear::from(Gear::builder().module(2.0).teeth(20).build().unwrap()))])
    /// #     .shaft("output", vec![("b", AnyGear::from(Gear::builder().module(2.0).teeth(40).build().unwrap()))])
    /// #     .mesh("a", "b").driver("input").build().unwrap();
    /// let sim = scene.run(1000.0).unwrap();
    /// let idx = sim.shaft_index("output").unwrap();
    /// for frame in sim.frames(24.0, 1.0) {
    ///     let _angle = frame.shaft_angles[idx];
    /// }
    /// ```
    ///
    /// Returns `None` if `shaft` is not a known shaft name.
    pub fn shaft_index(&self, shaft: &str) -> Option<usize> {
        self.shaft_order.iter().position(|s| s == shaft)
    }

    /// The name of the driver shaft for this simulation.
    pub fn driver_shaft(&self) -> &str {
        &self.driver_shaft
    }

    /// The driver RPM this simulation was run at.
    ///
    /// Equivalent to `sim.rpm(sim.driver_shaft())`.
    pub fn driver_rpm(&self) -> f64 {
        self.driver_rpm
    }
}

/// One animation keyframe: angular positions of all shafts at a single instant.
///
/// Produced by [`GearSimulation::frames`]. Feed `shaft_angles` to your renderer
/// on each frame tick to animate a gear train.
///
/// `shaft_angles` is a `Vec<f64>` in the same sorted (alphabetical) order as
/// [`GearSimulation::shaft_names`]. Use [`GearSimulation::shaft_index`] to look
/// up a shaft's index once, then use that index into every frame's `shaft_angles`.
///
/// All angles are in degrees and wrapped to `[0°, 360°)`.
#[derive(Debug, Clone)]
pub struct SimFrame {
    /// Time of this frame in seconds from the start of the simulation (`t = 0`).
    pub time_secs: f64,
    /// Angular position of each shaft in degrees `[0°, 360°)`.
    ///
    /// Indexed in the same sorted (alphabetical) shaft order as
    /// [`GearSimulation::shaft_names`] and [`GearSimulation::shaft_index`].
    pub shaft_angles: Vec<f64>,
}
