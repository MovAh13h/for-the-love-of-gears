use std::f64::consts::PI;

// Tooth geometry — all dimensions relative to the pitch circle:
//
//  ──────┬───┬──────  tip circle    da = m(z + 2)
//        │   │                      ↑ addendum ha = m
//  ──────┴───┴──────  pitch circle  d  = mz        (reference line)
//       /     \                     ↓ dedendum hf = 1.25m
//  ────/───────\────  root circle   df = m(z − 2.5)
//
//  tooth depth  h = ha + hf = 2.25m
//  clearance    c = hf − ha = 0.25m   (gap between mating gear tip and root)
//  thickness    s = πm / 2            (arc length of one tooth at pitch circle)

/// Radial distance from the pitch circle up to the tooth tip: `ha = m`.
///
/// The addendum is exactly one module in height. It is the portion of the tooth
/// that extends radially *outward* from the pitch circle and meshes into
/// the dedendum space of the mating gear.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Addendum(f64);

impl Addendum {
    /// Compute addendum from module: `ha = m`.
    pub fn from_module(m: f64) -> Self {
        Self(m)
    }

    /// Returns the addendum in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Radial distance from the pitch circle down to the tooth root: `hf = 1.25m`.
///
/// The dedendum is 1.25 modules deep radially *inward* from the pitch circle —
/// slightly deeper than the addendum so that the tip of the mating gear never bottoms out. The extra 0.25m becomes
/// the [`Clearance`] gap.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dedendum(f64);

impl Dedendum {
    /// Compute dedendum from module: `hf = 1.25 × m`.
    pub fn from_module(m: f64) -> Self {
        Self(1.25 * m)
    }

    /// Returns the dedendum in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Full radial height of a tooth from root to tip: `h = 2.25m`.
///
/// Tooth depth is the sum of [`Addendum`] and [`Dedendum`]:
/// `h = ha + hf = m + 1.25m = 2.25m`.
///
/// This is the minimum radial space a tooth occupies and determines how deep
/// the gear blank must be cut.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToothDepth(f64);

impl ToothDepth {
    /// Compute tooth depth from module: `h = 2.25 × m`.
    pub fn from_module(m: f64) -> Self {
        Self(2.25 * m)
    }

    /// Returns the tooth depth in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Arc length of one tooth measured along the pitch circle: `s = πm / 2`.
///
/// At the pitch circle, tooth and gap are equal in width, so each occupies
/// half the circular pitch `p = πm`, giving `s = πm / 2`.
///
/// Tooth thickness matters for backlash calculations — reducing `s` slightly
/// below the theoretical value creates the small clearance that prevents gears
/// from jamming.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToothThickness(f64);

impl ToothThickness {
    /// Compute tooth thickness from module: `s = πm / 2`.
    pub fn from_module(m: f64) -> Self {
        Self(PI * m / 2.0)
    }

    /// Returns the tooth thickness in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Radial gap between the tip of one gear's tooth and the root of the mating gear: `c = 0.25 × m`.
///
/// Clearance scales with module — a larger gear has proportionally larger clearance.
/// The 0.25 is a fixed ISO ratio, not a fixed value: it falls out of the difference
/// between dedendum and addendum: `c = hf − ha = 1.25m − 1.00m = 0.25m`.
///
/// Clearance prevents the tip of one gear from pressing against the root fillet
/// of its mate, which would cause jamming and accelerated wear.
///
/// If clearance is zero or negative, the gear pair will bind.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Clearance(f64);

impl Clearance {
    /// Compute clearance from module: `c = 0.25 × m`.
    pub fn from_module(m: f64) -> Self {
        Self(0.25 * m)
    }

    /// Returns the clearance in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}
