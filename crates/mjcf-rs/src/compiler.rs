//! `<compiler>` and `<option>` representations.

use crate::types::Coordinate;

/// Resolved `<compiler>` settings.
#[derive(Clone, Debug)]
pub struct Compiler {
    /// `"degree"` (default) → multiply incoming angle attributes by π/180. The
    /// parser bakes this in: every angle on the AST is in **radians**.
    pub angle_is_degree: bool,
    /// Euler-axis sequence used by every `euler="…"` attribute (default `"xyz"`).
    pub eulerseq: String,
    /// Coordinate convention. Currently must be `Local`.
    pub coordinate: Coordinate,
    /// `<compiler autolimits>` (default `true` in MJCF v3+).
    pub autolimits: bool,
    /// Default density used for derived inertia (kg/m³).
    pub inertia_density: f64,
    /// Default mass-derivation flag for `<inertial>`-less bodies.
    pub inertia_from_geom: InertiaFromGeom,
    /// Mesh-asset directory, relative to the model file.
    pub mesh_dir: Option<String>,
    /// Texture-asset directory.
    pub texture_dir: Option<String>,
    /// Common asset-directory fallback.
    pub asset_dir: Option<String>,
    /// `<compiler strippath>` (default `false`).
    pub strip_path: bool,
    /// `<compiler discardvisual>` (default `false`).
    pub discard_visual: bool,
    /// `<compiler convexhull>` (default `true`).
    pub convex_hull: bool,
    /// `<compiler exactmeshinertia>` (default `false`).
    pub exact_mesh_inertia: bool,
    /// `<compiler balanceinertia>` (default `false`).
    pub balance_inertia: bool,
    /// `<compiler boundmass>` lower clamp on mass (default `0.0`).
    pub bound_mass: f64,
    /// `<compiler boundinertia>` lower clamp on diagonal inertia entries.
    pub bound_inertia: f64,
    /// `<compiler settotalmass>` rescaling target (default = -1, disabled).
    pub set_total_mass: f64,
}

/// `<compiler inertiafromgeom>` selector.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InertiaFromGeom {
    /// Always derive inertia from geoms (overrides any `<inertial>`).
    True,
    /// Never derive — use only `<inertial>`.
    False,
    /// Use the geoms only when the body has no `<inertial>` (default).
    Auto,
}

impl Default for Compiler {
    fn default() -> Self {
        Self {
            angle_is_degree: true,
            eulerseq: "xyz".to_string(),
            coordinate: Coordinate::Local,
            autolimits: true,
            inertia_density: 1000.0,
            inertia_from_geom: InertiaFromGeom::Auto,
            mesh_dir: None,
            texture_dir: None,
            asset_dir: None,
            strip_path: false,
            discard_visual: false,
            convex_hull: true,
            exact_mesh_inertia: false,
            balance_inertia: false,
            bound_mass: 0.0,
            bound_inertia: 0.0,
            set_total_mass: -1.0,
        }
    }
}

/// Pass-through copy of `<option>` attributes.
#[derive(Clone, Debug)]
pub struct SimOption {
    /// Simulation timestep in seconds (`<option timestep>`, default 0.002).
    pub timestep: f64,
    /// Gravity vector (`<option gravity>`, default `[0, 0, -9.81]`).
    pub gravity: [f64; 3],
}

impl Default for SimOption {
    fn default() -> Self {
        Self {
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }
    }
}
