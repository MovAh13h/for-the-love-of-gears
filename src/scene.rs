//! Gear scene — compose shafts, mount gears, and simulate kinematics.
//!
//! # Concepts
//!
//! A **shaft** holds one or more gears and defines an axis of rotation. All
//! gears on the same shaft turn at the same angular velocity — this is the
//! mechanism behind compound gear trains. A **mesh** is a connection between
//! one gear on one shaft and one gear on another; it is how motion and power
//! transfer between shafts.
//!
//! The **driver** shaft is the one with an external power input. Its RPM is
//! specified when running the simulation; every other shaft's RPM is derived
//! from it by following the mesh connections and applying gear ratios.
//!
//! # Building a scene
//!
//! ```
//! use for_the_love_of_gears::{
//!     gear::Gear,
//!     scene::{AnyGear, GearScene},
//! };
//!
//! // Two-gear pair: 20-tooth driver → 40-tooth driven (2:1 reduction)
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
//! let sim = scene.run(1000.0).unwrap(); // 1000 rpm
//! assert_eq!(sim.rpm("output"), Some(500.0));  // 2:1 reduction
//! assert_eq!(sim.ratio_to("output"), Some(2.0));
//! ```
//!
//! # Compound gear trains
//!
//! Mount two gears on the same shaft to build a multi-stage reduction. Both
//! gears rotate together — the ratio multiplies across stages.
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
    collections::{BTreeMap, HashMap, VecDeque},
    fmt,
};

use crate::{gear::Gear, helical::HelicalGear};

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

/// A gear that can be mounted in a [`GearScene`] — either spur or helical.
///
/// Use [`AnyGear::from`] to convert a [`Gear`] or [`HelicalGear`] into this type.
#[derive(Debug, Clone)]
pub enum AnyGear {
    /// A standard spur gear.
    Spur(Gear),
    /// A helical gear.
    Helical(HelicalGear),
}

impl AnyGear {
    /// Number of teeth on this gear.
    pub fn teeth(&self) -> u32 {
        match self {
            Self::Spur(g) => g.teeth(),
            Self::Helical(g) => g.teeth(),
        }
    }

    /// Module in mm (normal module for helical gears).
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

/// Rotation direction of a shaft, defined relative to the driver shaft (clockwise).
///
/// Each external gear mesh reverses direction. In a simple two-gear pair the
/// driven gear rotates opposite to the driver; in a compound three-shaft train
/// the output is the same direction as the input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Same rotation direction as the driver shaft.
    Clockwise,
    /// Opposite rotation direction to the driver shaft.
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
    DuplicateShaftName(String),
    /// Two gears were given the same name (gear names must be unique across the whole scene).
    DuplicateGearName(String),
    /// No driver shaft was specified via `.driver()`.
    DriverShaftRequired,
    /// The driver shaft name does not match any shaft in the scene.
    DriverShaftNotFound(String),
    /// A gear referenced in a `.mesh()` call does not exist.
    GearNotFound(String),
    /// Both gears in a `.mesh()` call are on the same shaft — a gear cannot mesh with itself.
    MeshGearsOnSameShaft {
        /// Name of the first gear in the mesh.
        gear_a: String,
        /// Name of the second gear in the mesh.
        gear_b: String,
    },
    /// The two gears in a mesh are incompatible (different module or gear type).
    IncompatibleMesh {
        /// Name of the first gear in the mesh.
        gear_a: String,
        /// Name of the second gear in the mesh.
        gear_b: String,
    },
    /// A shaft has no mesh connections leading back to the driver — it would never move.
    DisconnectedShaft(String),
    /// A shaft is reachable via two different mesh paths that imply different RPM values.
    OverConstrainedShaft(String),
    /// Driver RPM must be greater than zero.
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
            Self::IncompatibleMesh { gear_a, gear_b } => write!(
                f,
                "gears '{gear_a}' and '{gear_b}' cannot mesh — incompatible module or type"
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
/// The same scene can be run at different speeds without rebuilding.
///
/// See the [module-level documentation](crate::scene) for examples.
#[derive(Debug)]
pub struct GearScene {
    driver_shaft: String,
    states: HashMap<String, ShaftState>,
}

impl GearScene {
    /// Start building a [`GearScene`].
    pub fn builder() -> GearSceneBuilder {
        GearSceneBuilder::default()
    }

    /// Run the scene at `driver_rpm`.
    ///
    /// Returns a [`GearSimulation`] from which you can query RPM, direction,
    /// angle, total rotations, and animation frames for any shaft.
    ///
    /// The same scene can be run multiple times at different speeds.
    ///
    /// # Errors
    ///
    /// - [`GearSceneError::DriverRpmMustBePositive`] if `driver_rpm <= 0`
    pub fn run(&self, driver_rpm: f64) -> Result<GearSimulation, GearSceneError> {
        if driver_rpm <= 0.0 {
            return Err(GearSceneError::DriverRpmMustBePositive);
        }

        let states = self
            .states
            .iter()
            .map(|(shaft, s)| {
                (shaft.clone(), ShaftState { rpm: s.rpm * driver_rpm, direction: s.direction })
            })
            .collect();

        Ok(GearSimulation {
            states,
            driver_shaft: self.driver_shaft.clone(),
            driver_rpm,
        })
    }

    /// The names of all shafts in this scene, in sorted order.
    pub fn shaft_names(&self) -> Vec<&str> {
        sorted_keys(&self.states)
    }
}

/// Builder for [`GearScene`].
///
/// Obtain via [`GearScene::builder()`].
#[derive(Debug, Default)]
pub struct GearSceneBuilder {
    shafts: Vec<(String, Vec<(String, AnyGear)>)>,
    meshes: Vec<(String, String)>,
    driver_shaft: Option<String>,
}

impl GearSceneBuilder {
    /// Add a named shaft with the listed gears.
    ///
    /// Each gear entry is `(gear_name, gear)`. Gear names must be unique across
    /// the entire scene — no two gears on any shaft may share a name.
    pub fn shaft<S: Into<String>>(mut self, name: &str, gears: Vec<(S, AnyGear)>) -> Self {
        self.shafts.push((
            name.to_string(),
            gears.into_iter().map(|(n, g)| (n.into(), g)).collect(),
        ));
        self
    }

    /// Declare that two gears (identified by name) mesh with each other.
    ///
    /// The gears must be on different shafts, have compatible modules, and
    /// be compatible gear types (spur–spur or matching helical–helical).
    pub fn mesh(mut self, gear_a: &str, gear_b: &str) -> Self {
        self.meshes.push((gear_a.to_string(), gear_b.to_string()));
        self
    }

    /// Designate which shaft is the driver (the one with the external input RPM).
    ///
    /// Every scene must have exactly one driver. Call [`GearScene::run`] to
    /// specify the actual RPM at simulation time.
    pub fn driver(mut self, shaft: &str) -> Self {
        self.driver_shaft = Some(shaft.to_string());
        self
    }

    /// Build and validate the [`GearScene`].
    ///
    /// Validates names, mesh compatibility, and connectivity, then pre-computes
    /// relative RPMs and directions for all shafts so that [`GearScene::run`]
    /// is a simple scale operation.
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
                return Err(GearSceneError::IncompatibleMesh {
                    gear_a: gear_a.clone(),
                    gear_b: gear_b.clone(),
                });
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

        Ok(GearScene { driver_shaft, states })
    }
}

/// The kinematic result of running a [`GearScene`] at a given driver RPM.
///
/// All queries are `O(1)` — RPM and direction are pre-computed at build time
/// and scaled by the driver RPM at run time. Angular position at any time `t`
/// is exact (no numerical integration).
#[derive(Debug)]
pub struct GearSimulation {
    states: HashMap<String, ShaftState>,
    driver_shaft: String,
    driver_rpm: f64,
}

impl GearSimulation {
    /// RPM of the named shaft, or `None` if the shaft name does not exist.
    pub fn rpm(&self, shaft: &str) -> Option<f64> {
        self.states.get(shaft).map(|s| s.rpm)
    }

    /// Rotation direction of the named shaft relative to the driver,
    /// or `None` if the shaft name does not exist.
    pub fn direction(&self, shaft: &str) -> Option<Direction> {
        self.states.get(shaft).map(|s| s.direction)
    }

    /// Total rotations over `duration_secs` seconds,
    /// or `None` if the shaft name does not exist.
    ///
    /// `total_rotations = rpm × duration_secs / 60`
    pub fn total_rotations(&self, shaft: &str, duration_secs: f64) -> Option<f64> {
        self.rpm(shaft).map(|r| r * duration_secs / 60.0)
    }

    /// Angular position of the shaft at time `t` seconds, in degrees `[0, 360)`,
    /// or `None` if the shaft name does not exist.
    ///
    /// `t = 0` gives `0°`. The angle wraps at 360°.
    pub fn angle_deg(&self, shaft: &str, t: f64) -> Option<f64> {
        self.rpm(shaft)
            .map(|r| (r / 60.0 * t * 360.0).rem_euclid(360.0))
    }

    /// Angular velocity of the named shaft in radians per second,
    /// or `None` if the shaft name does not exist.
    ///
    /// `ω = rpm × 2π / 60`
    pub fn angular_velocity_rad_s(&self, shaft: &str) -> Option<f64> {
        self.rpm(shaft).map(|r| r * std::f64::consts::TAU / 60.0)
    }

    /// Speed ratio from the driver to the named shaft: `driver_rpm / shaft_rpm`,
    /// or `None` if the shaft name does not exist.
    ///
    /// A value greater than `1.0` means the shaft is slower (speed reduction).
    /// A value less than `1.0` means the shaft is faster (speed increase).
    pub fn ratio_to(&self, shaft: &str) -> Option<f64> {
        self.rpm(shaft).map(|r| self.driver_rpm / r)
    }

    /// Generate animation frames at `fps` frames per second over `duration_secs` seconds.
    ///
    /// Each frame contains the angular position (in degrees) of every shaft at
    /// that instant. The first frame is at `t = 0`; the last is at
    /// `duration_secs` or the nearest frame boundary.
    pub fn frames(&self, fps: f64, duration_secs: f64) -> Vec<SimFrame> {
        let frame_count = (duration_secs * fps).ceil() as usize + 1;
        (0..frame_count)
            .map(|i| {
                let t = (i as f64 / fps).min(duration_secs);
                let shaft_angles = self
                    .states
                    .iter()
                    .map(|(s, state)| {
                        (s.clone(), (state.rpm / 60.0 * t * 360.0).rem_euclid(360.0))
                    })
                    .collect();
                SimFrame { time_secs: t, shaft_angles }
            })
            .collect()
    }

    /// The names of all shafts in this simulation, in sorted order.
    ///
    /// Mirrors [`GearScene::shaft_names`] so callers don't need to keep the
    /// scene around just to enumerate shafts.
    pub fn shaft_names(&self) -> Vec<&str> {
        sorted_keys(&self.states)
    }

    /// The name of the driver shaft.
    pub fn driver_shaft(&self) -> &str {
        &self.driver_shaft
    }

    /// The driver RPM this simulation was run at.
    pub fn driver_rpm(&self) -> f64 {
        self.driver_rpm
    }
}

/// One animation frame: angular positions of all shafts at a single instant in time.
///
/// `shaft_angles` is a [`BTreeMap`] so shaft names are always in sorted order —
/// iteration order is deterministic regardless of how shafts were declared.
#[derive(Debug, Clone)]
pub struct SimFrame {
    /// Time of this frame in seconds from the start of the simulation.
    pub time_secs: f64,
    /// Angular position of each shaft in degrees `[0, 360)`, keyed by shaft name.
    pub shaft_angles: BTreeMap<String, f64>,
}
