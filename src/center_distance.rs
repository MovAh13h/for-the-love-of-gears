/// The distance between the rotation axes of two meshing gears: `a = (d1 + d2) / 2`.
///
/// Center distance determines where the two gear shafts must be mounted. If the
/// actual shaft distance differs from the theoretical value, the pitch circles no
/// longer meet correctly — too close causes binding, too far increases backlash
/// and reduces load capacity.
///
/// ```text
///      shaft 1            shaft 2
///        ●                  ●
///        │←────── a ────────│
///       (○)              (○)
///     gear 1           gear 2
///   d1 = m . z1       d2 = m . z2
/// ```
///
/// For a gear pair with the same module:
/// `a = m(z1 + z2) / 2`
///
/// # Meshing condition
///
/// Both gears **must share the same module** (same tooth size). This has nothing
/// to do with the gears being the same size — a small 20-tooth gear meshes with
/// a large 100-tooth gear without issue, as long as both have the same module.
/// What cannot mesh is two gears with *different* modules: their teeth are
/// physically different sizes and will not fit together regardless of center distance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CenterDistance(f64);

impl CenterDistance {
    /// Compute center distance from the two reference diameters: `a = (d1 + d2) / 2`.
    ///
    /// Use this when you already have [`crate::diameter::ReferenceDiameter`] values
    /// for both gears.
    ///
    /// ```
    /// use for_the_love_of_gears::center_distance::CenterDistance;
    /// let a = CenterDistance::from_reference_diameters(40.0, 60.0);
    /// assert_eq!(a.value(), 50.0);
    /// ```
    pub fn from_reference_diameters(d1: f64, d2: f64) -> Self {
        Self((d1 + d2) / 2.0)
    }

    /// Compute center distance from module and tooth counts: `a = m(z1 + z2) / 2`.
    ///
    /// Shorthand when both gears share the same module (required for meshing).
    ///
    /// ```
    /// use for_the_love_of_gears::center_distance::CenterDistance;
    /// let a = CenterDistance::from_module_and_teeth(2.0, 20, 30);
    /// assert_eq!(a.value(), 50.0); // 2 × (20 + 30) / 2
    /// ```
    pub fn from_module_and_teeth(module: f64, teeth1: u32, teeth2: u32) -> Self {
        Self(module * (teeth1 + teeth2) as f64 / 2.0)
    }

    /// Returns the center distance in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}
