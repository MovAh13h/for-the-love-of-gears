use std::f64::consts::PI;

/// The fundamental unit of gear tooth size, as defined by ISO standards.
///
/// Module `m` sets the scale of every gear dimension — doubling the module
/// doubles tooth height, pitch diameter, and every other linear measurement.
/// Two gears can only mesh together if they share the same module (same tooth
/// size). A large gear and a small gear mesh fine together; what matters is
/// that their teeth are the same size, not that the gears are the same size.
///
/// # How module relates to other measurements
///
/// ```text
/// m = d / z       pitch circle diameter ÷ number of teeth
/// m = p / π       circular pitch ÷ π
/// m = 25.4 / DP   25.4 mm/in ÷ diametral pitch (imperial)
/// ```
///
/// # ISO preferred values (mm)
///
/// ```text
/// 1, 1.25, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10, 12, 16, 20
/// ```
///
/// Always choose from these standard values when designing — they ensure
/// off-the-shelf tooling availability.
///
/// # Choosing a variant
///
/// | Situation | Variant |
/// |---|---|
/// | Designing a new gear | [`Module::Specified`] |
/// | Have circular pitch `p`, need module | [`Module::FromCircularPitch`] |
/// | Have a physical gear, measured `d` and counted `z` | [`Module::FromPitchCircleDiameter`] |
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Module {
    /// Module given directly as an ISO standard value or design choice.
    ///
    /// ```
    /// use for_the_love_of_gears::module::Module;
    /// let m = Module::Specified(2.0);
    /// assert_eq!(m.value(), 2.0);
    /// ```
    Specified(f64),

    /// Module derived from the circular pitch `p` (arc length between teeth): `m = p / π`.
    ///
    /// Use this when you know the spacing between teeth along the pitch circle
    /// but not the module directly.
    ///
    /// ```
    /// use for_the_love_of_gears::module::Module;
    /// use std::f64::consts::PI;
    /// let m = Module::FromCircularPitch(2.0 * PI); // p = 2π → m = 2
    /// assert!((m.value() - 2.0).abs() < 1e-10);
    /// ```
    FromCircularPitch(f64),

    /// Module derived from pitch circle diameter `d` and tooth count `z`: `m = d / z`.
    ///
    /// Use this when reverse-engineering an existing gear — measure the pitch
    /// circle diameter and count the teeth.
    ///
    /// ```
    /// use for_the_love_of_gears::module::Module;
    /// let m = Module::FromPitchCircleDiameter { pitch_circle_diameter: 40.0, teeth: 20 };
    /// assert_eq!(m.value(), 2.0); // 40 / 20 = 2
    /// ```
    FromPitchCircleDiameter { pitch_circle_diameter: f64, teeth: u32 },
}

impl Module {
    /// Returns the module value in millimetres, computing it from the source if needed.
    pub fn value(self) -> f64 {
        match self {
            Self::Specified(m) => m,
            Self::FromCircularPitch(p) => p / PI,
            Self::FromPitchCircleDiameter { pitch_circle_diameter, teeth } => {
                pitch_circle_diameter / teeth as f64
            }
        }
    }
}

impl From<f64> for Module {
    fn from(m: f64) -> Self {
        Self::Specified(m)
    }
}
