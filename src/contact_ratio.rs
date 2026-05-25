//! Contact ratio formulae — internal helpers used by [`crate::gear`] and [`crate::helical`].
//!
//! Contact ratio `εα` is the average number of tooth pairs sharing load at any instant.
//! The higher the value, the smoother and quieter the transmission.
//!
//! ```text
//! εα = (√(ra1²−rb1²) + √(ra2²−rb2²) − a·sin(α)) / pb
//!
//!   ra  = tip radius      rb  = base radius
//!   a   = centre distance  α  = pressure angle
//!   pb  = π · m · cos(α)   (base pitch)
//! ```
//!
//! For helical gears an additional **overlap ratio** `εβ = b·sin(ψ)/(π·mn)` adds
//! axial coverage. The total is `εγ = εα + εβ`.

use std::f64::consts::PI;

/// Transverse contact ratio for a spur gear pair.
pub(crate) fn spur(
    tip_radius_1: f64,
    base_radius_1: f64,
    tip_radius_2: f64,
    base_radius_2: f64,
    center_distance: f64,
    pressure_angle_deg: f64,
    module: f64,
) -> f64 {
    let alpha = pressure_angle_deg.to_radians();
    let base_pitch = PI * module * alpha.cos();
    path_of_contact(tip_radius_1, base_radius_1, tip_radius_2, base_radius_2, center_distance, alpha)
        / base_pitch
}

/// Transverse contact ratio for a helical gear pair (uses transverse module and pressure angle).
pub(crate) fn helical_transverse(
    tip_radius_1: f64,
    base_radius_1: f64,
    tip_radius_2: f64,
    base_radius_2: f64,
    center_distance: f64,
    transverse_pressure_angle_deg: f64,
    transverse_module: f64,
) -> f64 {
    let alpha_t = transverse_pressure_angle_deg.to_radians();
    let base_pitch = PI * transverse_module * alpha_t.cos();
    path_of_contact(tip_radius_1, base_radius_1, tip_radius_2, base_radius_2, center_distance, alpha_t)
        / base_pitch
}

/// Overlap ratio for a helical gear: `εβ = b·sin(ψ) / (π·mn)`.
pub(crate) fn overlap(face_width: f64, helix_angle_deg: f64, normal_module: f64) -> f64 {
    face_width * helix_angle_deg.to_radians().sin() / (PI * normal_module)
}

fn path_of_contact(ra1: f64, rb1: f64, ra2: f64, rb2: f64, a: f64, alpha: f64) -> f64 {
    // Tip radius must exceed base radius; if not (degenerate geometry), clamp to 0.
    let approach = if ra1 > rb1 { (ra1.powi(2) - rb1.powi(2)).sqrt() } else { 0.0 };
    let recess   = if ra2 > rb2 { (ra2.powi(2) - rb2.powi(2)).sqrt() } else { 0.0 };
    (approach + recess - a * alpha.sin()).max(0.0)
}
