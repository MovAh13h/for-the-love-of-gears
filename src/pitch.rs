use std::f64::consts::PI;

/// The arc length between two adjacent teeth, measured along the pitch circle.
///
/// Circular pitch `p` is the metric way to express tooth spacing. It is directly
/// proportional to module: a larger module means larger, more widely spaced teeth.
///
/// ```text
/// p = π × m
/// ```
///
/// # Visualisation
///
/// ```text
///        p
///   ◄─────────►
///   │    │    │
///  ─┴────┴────┴─  ← pitch circle
///    tooth  tooth
/// ```
///
/// Compare with [`DiametralPitch`], which expresses the same concept in the
/// imperial system as teeth-per-inch rather than mm-per-tooth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircularPitch(f64);

impl CircularPitch {
    /// Compute circular pitch from module: `p = π × m`.
    ///
    /// ```
    /// use for_the_love_of_gears::pitch::CircularPitch;
    /// use std::f64::consts::PI;
    /// let p = CircularPitch::from_module(1.0);
    /// assert!((p.value() - PI).abs() < 1e-10);
    /// ```
    pub fn from_module(m: f64) -> Self {
        Self(PI * m)
    }

    /// Returns the circular pitch in millimetres.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// The number of teeth per inch of pitch circle diameter (imperial system).
///
/// Diametral pitch `DP` is the imperial counterpart to module. It runs
/// *inversely* to module and tooth size — a higher `DP` means smaller, finer
/// teeth. Common in inch-unit industries (North America, some aerospace).
///
/// ```text
/// DP = 25.4 / m      (25.4 mm = 1 inch)
/// ```
///
/// # Comparison
///
/// | Module (mm) | Diametral Pitch (teeth/in) | Tooth size |
/// |---|---|---|
/// | 1 | 25.4 | fine |
/// | 2 | 12.7 | medium |
/// | 4 | 6.35 | coarse |
///
/// For metric work, use [`CircularPitch`] instead.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiametralPitch(f64);

impl DiametralPitch {
    /// Compute diametral pitch from module: `DP = 25.4 / m`.
    ///
    /// ```
    /// use for_the_love_of_gears::pitch::DiametralPitch;
    /// let dp = DiametralPitch::from_module(1.0);
    /// assert!((dp.value() - 25.4).abs() < 1e-10);
    /// ```
    pub fn from_module(m: f64) -> Self {
        Self(25.4 / m)
    }

    /// Returns the diametral pitch in teeth per inch.
    pub fn value(self) -> f64 {
        self.0
    }
}
