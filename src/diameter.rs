//! The four concentric circles that define a gear's cross-section:
//!
//! ```text
//!  ┌──────────────────────────┐  tip circle    da = m(z + 2)
//!  │  ┌────────────────────┐  │  pitch circle  d  = mz
//!  │  │  ┌──────────────┐  │  │  root circle   df = m(z − 2.5)
//!  │  │  │  ┌────────┐  │  │  │  base circle   db = d · cos(α)
//!  │  │  │  │        │  │  │  │
//!  │  │  │  └────────┘  │  │  │
//!  │  │  └──────────────┘  │  │
//!  │  └────────────────────┘  │
//!  └──────────────────────────┘
//! ```
//!
//! The **pitch circle** is the reference — all other diameters are derived from
//! it. The **base circle** is unique: it is the origin of the involute tooth
//! profile and depends on the pressure angle.

/// Diameter of the pitch circle — the central reference of all gear geometry: `d = mz`.
///
/// This is the pitch circle diameter. See the [crate-level documentation](crate) for
/// a full explanation of what the pitch circle is and why it matters.
///
/// In short: the pitch circle is the imaginary circle that rolls against the
/// mating gear's pitch circle without slipping. It sits between the root and
/// tip circles, and every tooth dimension — [`crate::tooth::Addendum`],
/// [`crate::tooth::Dedendum`], all four diameters — is measured relative to it.
///
/// All other gear diameters derive from this one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReferenceDiameter(f64);

impl ReferenceDiameter {
    /// Compute reference diameter from module `m` and tooth count `z`: `d = mz`.
    ///
    /// ```
    /// use for_the_love_of_gears::diameter::ReferenceDiameter;
    /// let d = ReferenceDiameter::new(2.0, 20);
    /// assert_eq!(d.value(), 40.0); // 2 × 20
    /// ```
    pub fn new(module: f64, teeth: u32) -> Self {
        Self(module * teeth as f64)
    }

    /// Returns the reference diameter in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Outer diameter of the gear — the diameter you would measure with calipers: `da = m(z + 2)`.
///
/// The tip circle sits one [`crate::tooth::Addendum`] radially outward from the
/// pitch circle on each side. The `+2` comes from the addendum coefficient of
/// `1.00` applied on both sides: `2 × 1.00 = 2`.
/// This is the dimension that determines whether the gear fits in its housing.
///
/// ```text
///  pitch circle ─── d  = mz
///  tip circle   ─── da = m(z + 2) = d + 2·ha
///                         ↑
///                     addendum on each side
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TipDiameter(f64);

impl TipDiameter {
    /// Compute tip diameter from module `m` and tooth count `z`: `da = m(z + 2)`.
    ///
    /// ```
    /// use for_the_love_of_gears::diameter::TipDiameter;
    /// let da = TipDiameter::new(2.0, 20);
    /// assert_eq!(da.value(), 44.0); // 2 × (20 + 2)
    /// ```
    pub fn new(module: f64, teeth: u32) -> Self {
        Self(module * (teeth as f64 + 2.0))
    }

    /// Returns the tip diameter in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Diameter at the base of the tooth spaces: `df = m(z − 2.5)`.
///
/// The root circle sits one [`crate::tooth::Dedendum`] radially inward from
/// the pitch circle on each side. The `−2.5` comes from the dedendum coefficient
/// of `1.25` applied on both sides: `2 × 1.25 = 2.5`.
/// The root fillet lives at this depth.
///
/// ```text
///  pitch circle ─── d  = mz
///  root circle  ─── df = m(z − 2.5) = d − 2·hf
///                         ↑
///                     dedendum on each side
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RootDiameter(f64);

impl RootDiameter {
    /// Compute root diameter from module `m` and tooth count `z`: `df = m(z − 2.5)`.
    ///
    /// ```
    /// use for_the_love_of_gears::diameter::RootDiameter;
    /// let df = RootDiameter::new(2.0, 20);
    /// assert_eq!(df.value(), 35.0); // 2 × (20 − 2.5)
    /// ```
    pub fn new(module: f64, teeth: u32) -> Self {
        Self(module * (teeth as f64 - 2.0 * crate::tooth::DEDENDUM_COEFF))
    }

    /// Returns the root diameter in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Diameter of the base circle — the origin of the involute tooth profile: `db = d · cos(α)`.
///
/// Involute gear teeth are shaped as involutes of the base circle. The tooth
/// flanks "unroll" from this circle, which is why the pressure angle `α` appears
/// here: a larger pressure angle pulls the base circle inward (smaller `db`),
/// making the tooth flank steeper.
///
/// The base circle is always smaller than the root circle for standard pressure
/// angles (14.5°, 20°, 25°).
///
/// ```text
///  root circle  ─── df = m(z − 2.5)
///  base circle  ─── db = d · cos(α)     ← always inside root circle
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BaseDiameter(f64);

impl BaseDiameter {
    /// Compute base diameter from module `m`, tooth count `z`, and pressure angle `α`.
    ///
    /// `pressure_angle_deg` is in degrees. The standard value is `20.0`; older
    /// systems used `14.5`.
    ///
    /// ```
    /// use for_the_love_of_gears::diameter::BaseDiameter;
    /// let db = BaseDiameter::new(2.0, 20, 20.0);
    /// let expected = 40.0 * 20.0_f64.to_radians().cos();
    /// assert!((db.value() - expected).abs() < 1e-10);
    /// ```
    pub fn new(module: f64, teeth: u32, pressure_angle_deg: f64) -> Self {
        let d = module * teeth as f64;
        Self(d * pressure_angle_deg.to_radians().cos())
    }

    /// Returns the base diameter in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}
