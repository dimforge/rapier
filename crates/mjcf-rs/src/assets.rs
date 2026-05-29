//! `<asset>` library: meshes, hfields, textures, materials.

use crate::Pose;

/// Container for all assets defined in a model (and its includes).
#[derive(Default, Clone, Debug)]
pub struct Assets {
    /// `<mesh>` definitions, keyed by name.
    pub meshes: Vec<Mesh>,
    /// `<hfield>` definitions, keyed by name.
    pub hfields: Vec<Hfield>,
    /// `<texture>` definitions (recorded only — not loaded).
    pub textures: Vec<Texture>,
    /// `<material>` definitions (recorded only).
    pub materials: Vec<Material>,
}

impl Assets {
    /// Looks up a `<mesh>` by name.
    pub fn mesh(&self, name: &str) -> Option<&Mesh> {
        self.meshes.iter().find(|m| m.name.as_deref() == Some(name))
    }

    /// Looks up a `<hfield>` by name.
    pub fn hfield(&self, name: &str) -> Option<&Hfield> {
        self.hfields
            .iter()
            .find(|h| h.name.as_deref() == Some(name))
    }

    /// Looks up a `<material>` by name.
    pub fn material(&self, name: &str) -> Option<&Material> {
        self.materials
            .iter()
            .find(|m| m.name.as_deref() == Some(name))
    }

    /// Looks up a `<texture>` by name.
    pub fn texture(&self, name: &str) -> Option<&Texture> {
        self.textures
            .iter()
            .find(|t| t.name.as_deref() == Some(name))
    }
}

/// `<mesh>` asset. Either references a file or carries inline vertex/face data.
#[derive(Default, Clone, Debug)]
pub struct Mesh {
    /// Asset name.
    pub name: Option<String>,
    /// Class for default lookup.
    pub class: Option<String>,
    /// Mesh file path (relative to `meshdir` / model dir).
    pub file: Option<String>,
    /// Per-axis scale (default `[1, 1, 1]`).
    pub scale: [f64; 3],
    /// Reference pose applied to the source mesh before use.
    pub refpose: Pose,
    /// Inline vertex array (alternative to `file`). Flat `[x0, y0, z0, x1, …]`.
    pub inline_vertices: Option<Vec<f64>>,
    /// Inline normals. Optional even when `inline_vertices` is set.
    pub inline_normals: Option<Vec<f64>>,
    /// Inline triangle indices.
    pub inline_faces: Option<Vec<u32>>,
    /// `inertia` attribute: `"shell"`, `"convex"` (default), or `"exact"`.
    pub inertia: MeshInertia,
    /// Maximum number of vertices when computing the convex hull.
    pub max_hull_vert: Option<u32>,
    /// `smoothnormal` (visualisation only — recorded).
    pub smoothnormal: f64,
}

/// `<mesh inertia>` selector.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum MeshInertia {
    /// Surface integral.
    Shell,
    /// Convex-hull interior (default).
    #[default]
    Convex,
    /// Tetrahedral integration over all triangles.
    Exact,
    /// Treat as a point mass at the geometric centre.
    Legacy,
}

/// `<hfield>` heightfield asset.
#[derive(Default, Clone, Debug)]
pub struct Hfield {
    /// Asset name.
    pub name: Option<String>,
    /// Class.
    pub class: Option<String>,
    /// Number of grid rows.
    pub nrow: u32,
    /// Number of grid columns.
    pub ncol: u32,
    /// `(radius_x, radius_y, elevation_z, base_z)`.
    pub size: [f64; 4],
    /// File path (PNG / `.hfield`).
    pub file: Option<String>,
    /// Inline elevation data (`nrow * ncol` values, row-major).
    pub elevation: Option<Vec<f64>>,
}

/// `<texture>` asset (recorded only).
#[derive(Default, Clone, Debug)]
pub struct Texture {
    /// Asset name.
    pub name: Option<String>,
    /// Asset class.
    pub class: Option<String>,
    /// Texture type (`2d`, `cube`, `skybox`).
    pub type_: Option<String>,
    /// File path.
    pub file: Option<String>,
    /// `builtin` (`"none"`, `"gradient"`, `"checker"`, `"flat"`).
    pub builtin: Option<String>,
    /// Primary RGB.
    pub rgb1: Option<[f64; 3]>,
    /// Secondary RGB.
    pub rgb2: Option<[f64; 3]>,
}

/// `<material>` asset (recorded only).
#[derive(Default, Clone, Debug)]
pub struct Material {
    /// Asset name.
    pub name: Option<String>,
    /// Asset class.
    pub class: Option<String>,
    /// Texture reference.
    pub texture: Option<String>,
    /// `rgba`.
    pub rgba: Option<[f64; 4]>,
    /// `emission`.
    pub emission: f64,
    /// `specular`.
    pub specular: f64,
    /// `shininess`.
    pub shininess: f64,
    /// `roughness`.
    pub roughness: f64,
    /// `metallic`.
    pub metallic: f64,
}
