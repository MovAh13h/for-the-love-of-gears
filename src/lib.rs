//! Gear geometry library following ISO standards.
//!
//! # The Pitch Circle
//!
//! The pitch circle is the single most important concept in gear geometry.
//! Everything else is defined relative to it.
//!
//! When two gears mesh, each has an imaginary circle that rolls against the
//! other's without slipping — these are the pitch circles. The point where
//! they touch is called the **pitch point**, and it always lies on the straight
//! line connecting the two gear centres.
//!
//! ```text
//!    c1       pitch point      c2
//!     ●────────────●────────────●
//!     ←── d1/2 ───→←── d2/2 ───→
//!     ←──────────── a ───────────→
//! ```
//!
//! The pitch point divides the centre line exactly at the boundary where the
//! two pitch circles touch — one radius from each centre.
//!
//! The pitch circle diameter is `d = m × z` (module × number of teeth). Every
//! other gear dimension is anchored to it:
//!
//! - **Addendum** — tooth extends 1× module radially *outward* from the pitch circle
//! - **Dedendum** — tooth extends 1.25× module radially *inward* from the pitch circle
//! - **Tip circle** — pitch circle + 2 addenda
//! - **Root circle** — pitch circle − 2 dedenda
//! - **Base circle** — derived from the pitch circle via the pressure angle
//!
//! Two gears can only mesh if their pitch circles are tangent at the pitch
//! point, which means their centre distance must equal `(d1 + d2) / 2`.
//!
//! # Units
//!
//! All linear dimensions are in **millimetres (mm)**, consistent with ISO gear
//! standards (ISO 54, ISO 1328). Angles are in **degrees** at API boundaries
//! and converted to radians internally.

pub(crate) const MESH_TOLERANCE: f64 = 1e-9;

pub(crate) mod contact_ratio;
pub mod gear;
pub mod helical;
pub mod module;
pub mod scene;
pub mod traits;
