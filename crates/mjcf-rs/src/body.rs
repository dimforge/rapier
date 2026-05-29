//! `<worldbody>`, `<body>`, `<inertial>`, `<joint>`, `<freejoint>`, `<geom>`,
//! `<site>` AST types.

use crate::Pose;

/// A body in the kinematic tree. The implicit "world" body is represented as
/// an extra entry whose `parent` is `None` and whose `pose` is the identity.
#[derive(Clone, Debug, Default)]
pub struct Body {
    /// `name` attribute (`None` for the implicit world body).
    pub name: Option<String>,
    /// Local pose in the parent body's frame.
    pub pose: Pose,
    /// `<body mocap="true">` flag.
    pub mocap: bool,
    /// `<body gravcomp>` (default 0.0).
    pub gravcomp: f64,
    /// `<body sleep>` flag — allow this body to sleep (default false).
    pub sleep: bool,
    /// Default class for the body's children (`childclass="…"`).
    pub childclass: Option<String>,
    /// Class for the body itself.
    pub class: Option<String>,
    /// `<body user>` data passthrough.
    pub user: Vec<f64>,

    /// `<inertial>` element if any.
    pub inertial: Option<Inertial>,
    /// `<joint>` and `<freejoint>` elements that connect this body to its parent.
    /// In MJCF order — the parser preserves their declared order.
    pub joints: Vec<Joint>,
    /// `<geom>` elements declared inside this body.
    pub geoms: Vec<Geom>,
    /// `<site>` elements declared inside this body.
    pub sites: Vec<Site>,
}

/// `<inertial>` element.
#[derive(Clone, Debug, Default)]
pub struct Inertial {
    /// Pose of the inertial frame in the body frame.
    pub pose: Pose,
    /// Mass.
    pub mass: f64,
    /// Either `(ixx, iyy, izz)` of the diagonal inertia or
    /// `(ixx, iyy, izz, ixy, ixz, iyz)` of the full inertia.
    pub inertia: InertiaSpec,
}

/// Inertia specification.
#[derive(Clone, Debug)]
pub enum InertiaSpec {
    /// `<inertial diaginertia="ixx iyy izz">`.
    Diagonal([f64; 3]),
    /// `<inertial fullinertia="ixx iyy izz ixy ixz iyz">`.
    Full([f64; 6]),
}

impl Default for InertiaSpec {
    fn default() -> Self {
        InertiaSpec::Diagonal([0.0, 0.0, 0.0])
    }
}

/// A joint. `<freejoint>` is encoded as `Joint { type_: Free, … }`.
#[derive(Clone, Debug, Default)]
pub struct Joint {
    /// `name` attribute, if any.
    pub name: Option<String>,
    /// Default class assigned via `class="…"`.
    pub class: Option<String>,
    /// Joint type.
    pub type_: JointType,
    /// Joint anchor in the body's frame.
    pub pos: [f64; 3],
    /// Joint axis (in the body frame) for hinge/slide.
    pub axis: [f64; 3],
    /// `limited="true|false|auto"`.
    pub limited: crate::types::Tristate,
    /// `<joint range="lo hi">` (already in radians for hinges).
    pub range: Option<[f64; 2]>,
    /// `<joint stiffness>`.
    pub stiffness: f64,
    /// `<joint damping>`.
    pub damping: f64,
    /// `<joint springref>` (already in radians for hinges).
    pub springref: f64,
    /// `<joint springdamper>` `(timeconst, dampratio)`.
    pub springdamper: Option<[f64; 2]>,
    /// `<joint armature>`.
    pub armature: f64,
    /// `<joint frictionloss>`.
    pub frictionloss: f64,
    /// `<joint ref>` reference value (already in radians).
    pub ref_: f64,
    /// `<joint margin>`.
    pub margin: f64,
    /// `<joint user>` passthrough.
    pub user: Vec<f64>,
}

/// `<joint type="…">`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum JointType {
    /// Hinge / revolute (1 DoF).
    #[default]
    Hinge,
    /// Slide / prismatic (1 DoF).
    Slide,
    /// Ball / spherical (3 DoF).
    Ball,
    /// Free / 6 DoF.
    Free,
}

/// `<geom>` element.
#[derive(Clone, Debug, Default)]
pub struct Geom {
    /// `name` attribute, if any.
    pub name: Option<String>,
    /// Default class.
    pub class: Option<String>,
    /// Shape type.
    pub type_: GeomType,
    /// Shape size (interpretation depends on `type_`).
    pub size: [f64; 3],
    /// Local pose in the body frame.
    pub pose: Pose,
    /// Optional `fromto="x1 y1 z1 x2 y2 z2"` form.
    pub fromto: Option<[f64; 6]>,
    /// `<geom friction>` `(slide, spin, roll)` (default `[1.0, 0.005, 0.0001]` per MJCF).
    pub friction: [f64; 3],
    /// `<geom mass>` direct mass override.
    pub mass: Option<f64>,
    /// `<geom density>`.
    pub density: Option<f64>,
    /// `<geom margin>`.
    pub margin: f64,
    /// `<geom contype>` (default 1).
    pub contype: u32,
    /// `<geom conaffinity>` (default 1).
    pub conaffinity: u32,
    /// `<geom condim>` (default 3).
    pub condim: u32,
    /// `<geom group>` (default 0).
    pub group: i32,
    /// `<geom priority>` (default 0).
    pub priority: i32,
    /// `<geom mesh="…">` (asset reference).
    pub mesh: Option<String>,
    /// `<geom hfield="…">` (asset reference).
    pub hfield: Option<String>,
    /// `<geom material>` (visual).
    pub material: Option<String>,
    /// `<geom rgba>` (visual).
    pub rgba: Option<[f64; 4]>,
    /// `<geom user>` passthrough.
    pub user: Vec<f64>,
}

/// `<geom type="…">`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum GeomType {
    /// Plane (half-space).
    Plane,
    /// Heightfield.
    Hfield,
    /// Sphere — MJCF default.
    #[default]
    Sphere,
    /// Capsule along Z (rapier convention follows below in the loader).
    Capsule,
    /// Ellipsoid (approximated by the loader).
    Ellipsoid,
    /// Cylinder along Z.
    Cylinder,
    /// Box.
    Box,
    /// Triangle mesh.
    Mesh,
    /// SDF (out of scope; the loader will warn).
    Sdf,
}

/// `<site>` element. The parser records sites; the loader doesn't create
/// physics objects for them (sites are pure named frames).
#[derive(Clone, Debug, Default)]
pub struct Site {
    /// `name` attribute, if any.
    pub name: Option<String>,
    /// Default class.
    pub class: Option<String>,
    /// Local pose in the body frame.
    pub pose: Pose,
    /// `<site size>`.
    pub size: [f64; 3],
    /// `<site rgba>`.
    pub rgba: Option<[f64; 4]>,
    /// `<site material>`.
    pub material: Option<String>,
    /// `<site type>` (`sphere` / `capsule` — purely visual).
    pub type_: SiteType,
    /// `<site user>` passthrough.
    pub user: Vec<f64>,
}

/// `<site type>`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum SiteType {
    /// Sphere — MJCF default.
    #[default]
    Sphere,
    /// Capsule.
    Capsule,
    /// Ellipsoid.
    Ellipsoid,
    /// Cylinder.
    Cylinder,
    /// Box.
    Box,
}
