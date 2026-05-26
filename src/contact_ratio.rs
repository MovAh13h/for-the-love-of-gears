//! Contact ratio and interference formulae — internal helpers used by [`crate::gear`] and [`crate::helical`].
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
//!
//! Involute interference is checked by comparing each gear's tip reach
//! `√(ra² − rb²)` against the approach limit `a·sin(α)`. If either tip reach
//! exceeds the limit the tip has dug past the mating gear's base circle where
//! no involute exists.

use std::f64::consts::PI;

/// Transverse contact ratio for a gear pair.
///
/// Works for both spur and helical gears — pass the transverse module and
/// transverse pressure angle. For spur gears these equal the normal module
/// and pressure angle directly.
pub(crate) fn transverse(
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
    path_of_contact(
        tip_radius_1,
        base_radius_1,
        tip_radius_2,
        base_radius_2,
        center_distance,
        alpha,
    ) / base_pitch
}

/// Overlap ratio for a helical gear: `εβ = b·sin(ψ) / (π·mn)`.
pub(crate) fn overlap(face_width: f64, helix_angle_deg: f64, normal_module: f64) -> f64 {
    face_width * helix_angle_deg.to_radians().sin() / (PI * normal_module)
}

/// Returns `true` if involute interference occurs between the two gears.
///
/// Each gear's **tip reach** `√(ra² − rb²)` is compared against the
/// **approach limit** `a · sin(α)`. When a tip reach exceeds the limit, the
/// tip has crossed the mating gear's base circle — no involute exists there.
///
/// For spur gears pass the normal pressure angle; for helical gears pass the
/// transverse pressure angle `αt`.
pub(crate) fn interference(
    tip_radius_1: f64,
    base_radius_1: f64,
    tip_radius_2: f64,
    base_radius_2: f64,
    center_distance: f64,
    pressure_angle_deg: f64,
) -> bool {
    let limit = center_distance * pressure_angle_deg.to_radians().sin();
    let reach = |ra: f64, rb: f64| {
        if ra > rb {
            (ra * ra - rb * rb).sqrt()
        } else {
            0.0
        }
    };
    reach(tip_radius_1, base_radius_1) > limit || reach(tip_radius_2, base_radius_2) > limit
}

fn path_of_contact(ra1: f64, rb1: f64, ra2: f64, rb2: f64, a: f64, alpha: f64) -> f64 {
    // Tip radius must exceed base radius; if not (degenerate geometry), clamp to 0.
    let approach = if ra1 > rb1 {
        (ra1 * ra1 - rb1 * rb1).sqrt()
    } else {
        0.0
    };
    let recess = if ra2 > rb2 {
        (ra2 * ra2 - rb2 * rb2).sqrt()
    } else {
        0.0
    };
    (approach + recess - a * alpha.sin()).max(0.0)
}
