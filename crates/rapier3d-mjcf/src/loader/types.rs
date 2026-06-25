//! Plain-data types describing the loaded robot before insertion: bodies,
//! joints, equality constraints, actuator/sensor bindings, and the
//! [`MjcfRobot`] container that ties them together.

use std::collections::HashMap;
use std::path::PathBuf;

use mjcf_rs::contact::{ContactExclude as MjcfContactExclude, ContactPair as MjcfContactPair};
use mjcf_rs::extras::{Actuator as MjcfActuator, Keyframe, Sensor as MjcfSensor};
use mjcf_rs::model::BodyId;

use rapier3d::dynamics::{GenericJoint, RigidBody};
use rapier3d::geometry::{Collider, SharedShape};
use rapier3d::math::{Pose, Real, Vector};

/// One body from the MJCF model materialized as a rapier rigid-body and its colliders.
#[derive(Clone, Debug)]
pub struct MjcfBody {
    /// Original MJCF body name (or a synthesized name for intermediate bodies).
    pub name: Option<String>,
    /// `true` if this body was added to materialize one of several joints
    /// declared inside a single MJCF `<body>`.
    pub is_intermediate: bool,
    /// Index of the original MJCF body in [`Model::bodies`](mjcf_rs::model::Model::bodies)
    /// (the same index is used for both the body itself and any
    /// intermediates introduced for its joint chain). The world body has
    /// id 0.
    pub mjcf_body_id: BodyId,
    /// The rapier rigid-body.
    pub body: RigidBody,
    /// Colliders attached to this rigid-body.
    pub colliders: Vec<Collider>,
    /// `<body gravcomp>` value (0 = no compensation, 1 = full compensation).
    /// Recorded so [`MjcfRobotHandles::apply_gravity_compensation`](super::MjcfRobotHandles::apply_gravity_compensation)
    /// can re-apply the exact force per step for fractional values.
    pub gravcomp: f64,
    /// Mass derived from the MJCF model (`<inertial>` or derived from
    /// geoms when `inertiafromgeom` requests it). Zero when the body has
    /// no inertial information — e.g. intermediate spacer bodies, or
    /// when [`MjcfLoaderOptions::apply_imported_mass_props`](super::MjcfLoaderOptions::apply_imported_mass_props)
    /// is `false`.
    ///
    /// [`MjcfRobotHandles::apply_gravity_compensation`](super::MjcfRobotHandles::apply_gravity_compensation)
    /// reads this directly rather than calling `RigidBody::mass()`,
    /// because rapier's `mass()` returns 0 until the body has gone through
    /// one step (the `additional_mass_properties` doesn't combine into
    /// the queryable `local_mprops` until `update_world_mass_properties`
    /// runs).
    pub mass: Real,
    /// Render-only meshes derived from MJCF `<geom>` elements with
    /// `contype = conaffinity = 0`. Populated only when
    /// [`MjcfLoaderOptions::create_colliders_from_visual_shapes`](super::MjcfLoaderOptions::create_colliders_from_visual_shapes)
    /// is `false` — that flag controls whether visual geoms become
    /// colliders; when it's off, the geometry is preserved here instead
    /// so the user can hand it to a renderer.
    pub visual_meshes: Vec<MjcfVisualMesh>,
}

/// A render-only mesh declared by an MJCF `<geom>` (typically a
/// `<geom type="mesh">` with `contype = conaffinity = 0`). The loader
/// surfaces these so the caller can render them attached to the parent
/// body without inserting them into the physics collider set.
#[derive(Clone, Debug)]
pub struct MjcfVisualMesh {
    /// Shape, ready to be handed to a renderer.
    pub shape: SharedShape,
    /// Pose of the mesh in the body's local frame.
    pub local_pose: Pose,
    /// Resolved color, prioritized in this order: `<geom rgba>` over the
    /// MJCF `<material rgba>` over the OBJ MTL diffuse color. `None`
    /// when no source set one — the caller's fallback applies.
    pub rgba: Option<[f32; 4]>,
    /// Per-vertex texture coordinates parallel to the shape's vertex
    /// buffer. `Some` only for mesh geoms loaded from an asset that
    /// carried UV data; `None` for primitives and for meshes whose
    /// source didn't include texture coordinates.
    pub uvs: Option<Vec<[f32; 2]>>,
    /// Per-vertex normals parallel to the shape's vertex buffer. Computed
    /// from the geometry (crease-aware smoothing, honoring the asset's
    /// `smoothnormal`) the way MuJoCo generates mesh normals — faceted
    /// sources like STL carry no usable shared-vertex normals of their own.
    /// `Some` for mesh geoms; `None` for primitives. When present, a
    /// renderer should use these directly instead of recomputing flat
    /// per-face normals, so meshes render smooth-shaded like MuJoCo's
    /// viewer rather than faceted.
    pub normals: Option<Vec<[f32; 3]>>,
    /// Resolved filesystem path to a 2D color texture. Pulled from the
    /// MJCF `<material texture=…>` when set, otherwise from the OBJ
    /// MTL's `map_Kd`. `None` for geoms that aren't textured.
    pub texture: Option<PathBuf>,
    /// PBR shading parameters resolved from the geom's `<material>`. `None`
    /// when the geom references no material — the renderer then keeps its own
    /// default shading rather than being forced to a metallic-roughness look.
    pub material: Option<MjcfRenderMaterial>,
}

/// PBR shading parameters resolved from an MJCF `<material>`, expressed in the
/// metallic-roughness model a modern renderer consumes. MuJoCo's legacy Phong
/// `specular`/`shininess` are folded in here too: `reflectance` carries the
/// specular intensity, and `roughness` falls back to `1 − shininess` when the
/// material doesn't set `roughness` explicitly.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MjcfRenderMaterial {
    /// Metallic factor in `[0, 1]` (MuJoCo `metallic`; 0 = dielectric).
    pub metallic: f32,
    /// Surface roughness in `[0, 1]` (MuJoCo `roughness`, or `1 − shininess`).
    pub roughness: f32,
    /// Dielectric specular reflectance in `[0, 1]` (MuJoCo `specular`), which a
    /// renderer maps to the F0 Fresnel term.
    pub reflectance: f32,
    /// Emissive color `material.rgb × emission`; `[0, 0, 0]` for non-emissive.
    pub emissive: [f32; 3],
}

/// One joint from the MJCF model materialized as a rapier `GenericJoint`.
#[derive(Clone, Debug)]
pub struct MjcfJoint {
    /// MJCF joint name (`None` for synthesized fixed joints connecting
    /// no-joint bodies to their parent).
    pub name: Option<String>,
    /// Index of the parent rapier body in [`MjcfRobot::bodies`].
    pub link1: usize,
    /// Index of the child rapier body in [`MjcfRobot::bodies`].
    pub link2: usize,
    /// The joint description.
    pub joint: GenericJoint,
    /// MJCF `<joint damping>` value, preserved separately from the motor so
    /// the multibody insertion path can route it through per-DoF damping
    /// (which is numerically more stable than motor damping under stiff
    /// loads, and naturally covers all 3 DoFs of a ball joint instead of
    /// only the motorised AngX).
    pub damping_per_dof: Real,
    /// MJCF `<joint armature>` value (reflected rotor inertia). On the
    /// multibody insertion path this is added to the generalized mass-matrix
    /// diagonal of each of the joint's DoFs, matching MuJoCo's joint-space
    /// semantics. It is deliberately **not** baked into the link's spatial
    /// inertia tensor: doing so makes the inertia extremely anisotropic
    /// (huge along the joint axis, ~0 across it) and the multibody mass
    /// matrix ill-conditioned.
    pub armature_per_dof: Real,
    /// MJCF `<joint stiffness>` (passive spring). On the multibody path this
    /// can be integrated implicitly in the generalized dynamics (added to the
    /// mass-matrix diagonal as `dt²·k` with a force `-k·(q − ref)`), which is
    /// numerically stable for stiff springs on low-inertia links — unlike the
    /// explicit position motor used otherwise. `0` if the joint has no spring.
    pub spring_stiffness_per_dof: Real,
    /// Rest position of the spring (`springref − ref`, in rapier's joint
    /// coordinate convention). Used for both the explicit-stiffness spring and
    /// the `springdamper` spring below.
    pub spring_ref: Real,
    /// MJCF `<joint springdamper="timeconst dampratio">` (both positive). Unlike
    /// `spring_stiffness_per_dof`/`damping_per_dof` — which are physical
    /// coefficients known up front — a `springdamper` requests a spring with a
    /// given *time constant and damping ratio*, so its stiffness and damping
    /// must be computed from the assembled joint-space inertia (MuJoCo's
    /// `dof_invweight0`). The multibody insertion path resolves it in a
    /// post-assembly pass; a valid `springdamper` overrides the explicit
    /// stiffness/damping (which are then left at 0 here).
    pub springdamper: Option<(Real, Real)>,
}

/// One `<equality>` constraint materialized as an extra rapier joint.
#[derive(Clone, Debug)]
pub struct MjcfEqualityJoint {
    /// `name` attribute, if any.
    pub name: Option<String>,
    /// First body's index in [`MjcfRobot::bodies`].
    pub link1: usize,
    /// Second body's index in [`MjcfRobot::bodies`].
    pub link2: usize,
    /// Whether the constraint is active (mapped to `set_enabled`).
    pub active: bool,
    /// The joint description.
    pub joint: GenericJoint,
}

/// One `<equality><joint>` coupling, resolved to a *linear* relation between
/// two of [`MjcfRobot::joints`]: `q2 = coeff·q1 + offset` (rapier joint
/// coordinates). Higher-order `polycoef` terms are unsupported. Applied as a
/// multibody DoF coupling at insertion time.
#[derive(Copy, Clone, Debug)]
pub struct MjcfJointCoupling {
    /// Index of the first (independent) joint in [`MjcfRobot::joints`].
    pub joint1: usize,
    /// Index of the second (dependent) joint in [`MjcfRobot::joints`].
    pub joint2: usize,
    /// Linear coupling coefficient (`polycoef[1]`).
    pub coeff: Real,
    /// Constant offset (`polycoef[0]`).
    pub offset: Real,
    /// Whether the constraint is active.
    pub active: bool,
}

/// `<actuator>` ready to drive a rapier joint motor.
#[derive(Clone, Debug)]
pub struct MjcfActuatorBinding {
    /// Original actuator metadata.
    pub actuator: MjcfActuator,
    /// Index into [`MjcfRobot::joints`] of the joint this actuator drives,
    /// or `None` if the actuator references something rapier doesn't model.
    pub joint_index: Option<usize>,
}

/// `<sensor>` definition resolved against the rapier handles. The simulation
/// itself is up to the user — call [`MjcfRobot::read_sensor`] to query.
#[derive(Clone, Debug)]
pub struct MjcfSensorBinding {
    /// Original sensor metadata.
    pub sensor: MjcfSensor,
    /// Resolved object reference, when applicable.
    pub object: SensorObjectRef,
}

/// Resolved subject of a sensor.
#[derive(Clone, Debug, Default)]
pub enum SensorObjectRef {
    /// No (or unrecognized) object reference.
    #[default]
    None,
    /// Resolved to a body (rapier index in [`MjcfRobot::bodies`]).
    Body(usize),
    /// Resolved to a joint (rapier index in [`MjcfRobot::joints`]).
    Joint(usize),
    /// Resolved to a site (body index, site index in that body's `sites`).
    Site {
        /// Rapier body index this site belongs to.
        body: usize,
        /// Index into the original MJCF body's `sites` list (the loader does
        /// not create rapier objects for sites).
        site: usize,
    },
}

/// The MJCF joint kind backing one keyframe `qpos`/`qvel` slot. Decides how
/// many `qpos` / `qvel` scalars the slot consumes and how they are written to
/// the rapier model.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MjcfDofKind {
    /// 6-DoF free / floating base. Consumes 7 `qpos` (xyz position + wxyz
    /// quaternion, world frame) and 6 `qvel` (linear + angular). Maps to a
    /// free rapier body — there is no rapier joint, so the world pose is
    /// written directly (see [`MjcfQposDof::body`]).
    Free,
    /// 3-DoF ball. Consumes 4 `qpos` (wxyz quaternion) and 3 `qvel`.
    Ball,
    /// 1-DoF revolute. Consumes 1 `qpos` (angle) and 1 `qvel`.
    Hinge,
    /// 1-DoF prismatic. Consumes 1 `qpos` (length) and 1 `qvel`.
    Slide,
}

impl MjcfDofKind {
    /// Number of `qpos` scalars this DoF consumes.
    pub fn qpos_width(self) -> usize {
        match self {
            MjcfDofKind::Free => 7,
            MjcfDofKind::Ball => 4,
            MjcfDofKind::Hinge | MjcfDofKind::Slide => 1,
        }
    }

    /// Number of `qvel` scalars this DoF consumes.
    pub fn qvel_width(self) -> usize {
        match self {
            MjcfDofKind::Free => 6,
            MjcfDofKind::Ball => 3,
            MjcfDofKind::Hinge | MjcfDofKind::Slide => 1,
        }
    }
}

/// One MJCF joint's slot in a keyframe's `qpos` / `qvel` arrays, resolved to
/// where it must be written in the rapier model.
///
/// The entries in [`MjcfRobot::qpos_dofs`] are ordered exactly as MuJoCo lays
/// out `qpos` (and `qvel`): joints in kinematic-tree order — bodies in
/// declaration order, and within a body its joints in declaration order.
/// Welds and other 0-DoF couplings contribute nothing, matching MuJoCo. So
/// reading a keyframe is a single left-to-right walk of this list, advancing a
/// `qpos` cursor by [`MjcfDofKind::qpos_width`] (and a `qvel` cursor by
/// [`MjcfDofKind::qvel_width`]) per entry.
#[derive(Clone, Debug)]
pub struct MjcfQposDof {
    /// The joint kind — decides the `qpos`/`qvel` width and how the slot maps
    /// onto rapier.
    pub kind: MjcfDofKind,
    /// Index into [`MjcfRobot::joints`] of the rapier joint this slot drives.
    /// `None` for [`MjcfDofKind::Free`], which has no rapier joint — its pose
    /// is written to [`Self::body`] directly.
    pub joint: Option<usize>,
    /// Index into [`MjcfRobot::bodies`] of the child body this slot moves. For
    /// [`MjcfDofKind::Free`] this is the floating body whose world pose `qpos`
    /// sets.
    pub body: usize,
    /// MJCF `<joint ref>` (the joint's zero offset). Subtracted from `qpos`
    /// to convert MuJoCo's absolute joint coordinate to rapier's
    /// frame-relative one, matching the limit convention in the serial-joint
    /// builder. A length (pre-scale) for [`MjcfDofKind::Slide`]; unused for
    /// `Free`/`Ball`.
    pub reference: Real,
    /// Length scale (`MjcfLoaderOptions::scale`) applied to `Slide` coordinates
    /// and to the `Free` base translation, matching the scaling the loader
    /// applied to the model geometry.
    pub scale: Real,
}

/// A robot loaded from an MJCF file: a flat list of bodies, joints, and
/// extras.
#[derive(Clone, Debug, Default)]
pub struct MjcfRobot {
    /// Optional `<mujoco model="…">` name.
    pub name: Option<String>,
    /// All rapier rigid-bodies created from the MJCF model. Index 0 is
    /// reserved for the implicit world body but is never inserted into the
    /// rapier `RigidBodySet`. Original MJCF body indices map 1:1 onto
    /// entries with `is_intermediate == false`.
    pub bodies: Vec<MjcfBody>,
    /// All joints connecting one body to another.
    pub joints: Vec<MjcfJoint>,
    /// Extra joints created from `<equality>` constraints.
    pub equality_joints: Vec<MjcfEqualityJoint>,
    /// `<equality><joint>` couplings between two joints' coordinates.
    pub joint_couplings: Vec<MjcfJointCoupling>,
    /// Actuator bindings.
    pub actuators: Vec<MjcfActuatorBinding>,
    /// Sensor bindings.
    pub sensors: Vec<MjcfSensorBinding>,
    /// Keyframes preserved from the model.
    pub keyframes: Vec<Keyframe>,
    /// Layout that maps a keyframe's `qpos` / `qvel` arrays onto
    /// [`Self::joints`] / [`Self::bodies`], in MuJoCo's generalized-coordinate
    /// order. Drives [`MjcfRobotHandles::apply_keyframe`](super::MjcfRobotHandles::apply_keyframe).
    pub qpos_dofs: Vec<MjcfQposDof>,
    /// The `MjcfLoaderOptions::shift` the loader applied to every body pose.
    /// Re-applied when a keyframe writes a free body's absolute world pose so
    /// the floating base lands in the same frame as the rest of the model.
    pub base_shift: Pose,
    /// Resolved gravity vector (`<option gravity>`).
    pub gravity: Vector,
    /// Map from MJCF body name to index in [`Self::bodies`].
    pub body_name_to_idx: HashMap<String, usize>,
    /// Map from MJCF geom name to a `(body_index, collider_index_in_body)` pair.
    pub geom_name_to_collider: HashMap<String, (usize, usize)>,
    /// Map from MJCF joint name to index in [`Self::joints`].
    pub joint_name_to_idx: HashMap<String, usize>,
    /// `<contact><exclude>` entries (preserved verbatim).
    pub contact_excludes: Vec<MjcfContactExclude>,
    /// `<contact><pair>` entries (preserved verbatim).
    pub contact_pairs: Vec<MjcfContactPair>,
}
