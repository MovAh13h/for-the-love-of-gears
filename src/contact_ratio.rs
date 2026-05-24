//! Contact ratio — the average number of tooth pairs sharing load at any instant.
//!
//! # What is contact ratio?
//!
//! Imagine two gear teeth coming into mesh. One pair of teeth makes contact,
//! carries the full load, then disengages as the next pair takes over. But the
//! handoff is not instantaneous — there is a brief overlap window where **two
//! pairs of teeth are in contact simultaneously**, sharing the load between them.
//!
//! The contact ratio `εα` measures how long that overlap window is, expressed as
//! a fraction of one tooth pitch. A value of `1.6` means that, averaged over a
//! full rotation, 1.6 tooth pairs are in mesh at once:
//!
//! - For `1/0.6` of the cycle → one pair carries the load alone
//! - For `0.6/0.6` of the cycle → two pairs share the load
//!
//! ```text
//!  εα = 1.0  →  one pair, always alone   (noisy, high stress, theoretical minimum)
//!  εα = 1.6  →  typical spur gear pair   (industry norm for general machinery)
//!  εα = 2.0  →  two pairs, always in mesh (smooth, quiet, load well distributed)
//! ```
//!
//! The higher the contact ratio, the smoother and quieter the transmission, and
//! the lower the peak tooth stress.
//!
//! # Transverse contact ratio
//!
//! [`TransverseContactRatio`] (`εα`) is the contact ratio measured in the
//! transverse plane (the cross-section perpendicular to the gear axis). It is
//! the only contact ratio for spur gears; for helical gears it is the starting
//! point.
//!
//! The formula traces the **path of contact** — the line segment along which the
//! tooth flanks slide against each other — and divides it by the **base pitch**
//! (the tooth-to-tooth spacing along the base circle):
//!
//! ```text
//!  εα = (√(ra1² − rb1²) + √(ra2² − rb2²) − a·sin(α)) / pb
//!
//!  where  ra  = tip radius (mm)
//!         rb  = base circle radius (mm)
//!         a   = centre distance (mm)
//!         α   = pressure angle (degrees)
//!         pb  = base pitch = π·m·cos(α)  (mm)
//! ```
//!
//! # Overlap ratio (helical gears)
//!
//! Helical teeth do not snap in and out of contact the way spur teeth do —
//! because the tooth is angled, contact begins at one end of the face and sweeps
//! across to the other. This axial overlap adds a second contribution to smooth
//! transmission: the **overlap ratio** `εβ`.
//!
//! ```text
//!  εβ = b·sin(ψ) / (π·mn)
//!
//!  where  b   = face width (mm)
//!         ψ   = helix angle (degrees)
//!         mn  = normal module (mm)
//! ```
//!
//! # Total contact ratio (helical gears)
//!
//! The **total contact ratio** `εγ = εα + εβ` combines both contributions. A
//! helical gear pair with `εα = 1.6` and `εβ = 0.8` has `εγ = 2.4` — on average
//! 2.4 tooth-width segments are sharing load at any instant. This is why helical
//! gears run quieter and carry more load than an equivalent spur pair.

use std::f64::consts::PI;

/// Shared helper used by both [`crate::gear::Gear`] and [`crate::helical::HelicalGear`].
///
/// Computes the transverse contact ratio given tip/base radii for both gears,
/// the centre distance, the relevant pressure angle, and the relevant module
/// (transverse for helical, normal for spur). The base pitch is derived internally.
pub(crate) fn transverse_contact_ratio(
    tip_radius_1: f64,
    base_radius_1: f64,
    tip_radius_2: f64,
    base_radius_2: f64,
    center_distance: f64,
    pressure_angle_deg: f64,
    module: f64,
) -> TransverseContactRatio {
    let base_pitch = PI * module * pressure_angle_deg.to_radians().cos();
    TransverseContactRatio::from_geometry(
        tip_radius_1,
        base_radius_1,
        tip_radius_2,
        base_radius_2,
        center_distance,
        pressure_angle_deg,
        base_pitch,
    )
}

/// Average number of tooth pairs in contact in the transverse plane: `εα`.
///
/// Use [`crate::gear::Gear::contact_ratio_with`] for spur gear pairs, or
/// [`crate::helical::HelicalGear::transverse_contact_ratio_with`] for helical pairs.
/// [`TransverseContactRatio::from_geometry`] is the low-level constructor if you
/// already have the radii and centre distance.
///
/// See the [module-level documentation](crate::contact_ratio) for a full explanation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransverseContactRatio(f64);

impl TransverseContactRatio {
    /// Compute transverse contact ratio from raw gear geometry.
    ///
    /// All length inputs are in **millimetres**; `pressure_angle_deg` is in **degrees**.
    ///
    /// ```
    /// use for_the_love_of_gears::contact_ratio::TransverseContactRatio;
    /// use std::f64::consts::PI;
    ///
    /// // m=2, z1=20, z2=40, α=20°
    /// let ra1 = 22.0_f64; // tip radius gear 1 (mm)
    /// let rb1 = 40.0_f64 * 20.0_f64.to_radians().cos() / 2.0; // base radius gear 1 (mm)
    /// let ra2 = 42.0_f64;
    /// let rb2 = 80.0_f64 * 20.0_f64.to_radians().cos() / 2.0;
    /// let a   = 60.0_f64; // centre distance (mm)
    /// let pb  = PI * 2.0 * 20.0_f64.to_radians().cos(); // base pitch (mm)
    ///
    /// let cr = TransverseContactRatio::from_geometry(ra1, rb1, ra2, rb2, a, 20.0, pb);
    /// assert!((cr.value() - 1.635).abs() < 0.001);
    /// ```
    pub fn from_geometry(
        tip_radius_1: f64,
        base_radius_1: f64,
        tip_radius_2: f64,
        base_radius_2: f64,
        center_distance: f64,
        pressure_angle_deg: f64,
        base_pitch: f64,
    ) -> Self {
        let alpha = pressure_angle_deg.to_radians();
        let path_of_contact = (tip_radius_1.powi(2) - base_radius_1.powi(2)).sqrt()
            + (tip_radius_2.powi(2) - base_radius_2.powi(2)).sqrt()
            - center_distance * alpha.sin();
        Self(path_of_contact / base_pitch)
    }

    /// Returns the transverse contact ratio (dimensionless).
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Axial overlap contribution to contact ratio for helical gears: `εβ = b·sin(ψ) / (π·mn)`.
///
/// Requires a face width — use [`crate::helical::HelicalGear::overlap_ratio`] which
/// returns `None` when no face width was set.
///
/// See the [module-level documentation](crate::contact_ratio) for a full explanation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OverlapRatio(f64);

impl OverlapRatio {
    /// Compute overlap ratio from face width, helix angle, and normal module.
    ///
    /// `face_width_mm` and `normal_module_mm` are in **millimetres**;
    /// `helix_angle_deg` is in **degrees**.
    ///
    /// ```
    /// use for_the_love_of_gears::contact_ratio::OverlapRatio;
    ///
    /// // b=30mm, ψ=20°, mn=2mm
    /// let eb = OverlapRatio::new(30.0, 20.0, 2.0);
    /// let expected = 30.0 * 20.0_f64.to_radians().sin() / (std::f64::consts::PI * 2.0);
    /// assert!((eb.value() - expected).abs() < 1e-10);
    /// ```
    pub fn new(face_width_mm: f64, helix_angle_deg: f64, normal_module_mm: f64) -> Self {
        Self(face_width_mm * helix_angle_deg.to_radians().sin() / (PI * normal_module_mm))
    }

    /// Returns the overlap ratio (dimensionless).
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Total contact ratio for helical gears: `εγ = εα + εβ`.
///
/// Use [`crate::helical::HelicalGear::total_contact_ratio_with`], which returns
/// `None` when no face width was set on the gear.
///
/// See the [module-level documentation](crate::contact_ratio) for a full explanation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TotalContactRatio(f64);

impl TotalContactRatio {
    /// Sum transverse and overlap ratios into a total contact ratio.
    ///
    /// ```
    /// use for_the_love_of_gears::contact_ratio::{OverlapRatio, TotalContactRatio, TransverseContactRatio};
    /// use std::f64::consts::PI;
    ///
    /// let ra1 = 22.0_f64;
    /// let rb1 = 40.0_f64 * 20.0_f64.to_radians().cos() / 2.0;
    /// let ra2 = 42.0_f64;
    /// let rb2 = 80.0_f64 * 20.0_f64.to_radians().cos() / 2.0;
    /// let pb  = PI * 2.0 * 20.0_f64.to_radians().cos();
    /// let ea  = TransverseContactRatio::from_geometry(ra1, rb1, ra2, rb2, 60.0, 20.0, pb);
    /// let eb  = OverlapRatio::new(30.0, 20.0, 2.0);
    /// let eg  = TotalContactRatio::new(ea, eb);
    /// assert!((eg.value() - (ea.value() + eb.value())).abs() < 1e-10);
    /// ```
    pub fn new(transverse: TransverseContactRatio, overlap: OverlapRatio) -> Self {
        Self(transverse.value() + overlap.value())
    }

    /// Returns the total contact ratio (dimensionless).
    pub fn value(self) -> f64 {
        self.0
    }
}
