//! Loader-wide configuration (`MjcfLoaderOptions`,
//! `MjcfMultibodyOptions`, `ContactFilterMode`).

use rapier3d::dynamics::RigidBodyBuilder;
#[cfg(feature = "__meshloader_is_enabled")]
use rapier3d::geometry::MeshConverter;
use rapier3d::geometry::{ColliderBuilder, TriMeshFlags};
use rapier3d::math::{Pose, Real};

bitflags::bitflags! {
    /// Options applied to multibody joints created from the MJCF joints.
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
    pub struct MjcfMultibodyOptions: u8 {
        /// If set, the created multibody joints are kinematic.
        const JOINTS_ARE_KINEMATIC = 0b0001;
        /// If set, contacts between two links of the same multibody are disabled.
        const DISABLE_SELF_CONTACTS = 0b0010;
        /// If set, `<equality><connect>` and `<equality><weld>` constraints
        /// are **not** inserted as impulse joints alongside the multibody.
        ///
        /// MuJoCo uses equalities to close kinematic loops (e.g. Cassie's
        /// achilles / plantar rods, four-bar mechanisms in parallel-link
        /// arms). Rapier's multibody is tree-structured, so the loader
        /// resorts to inserting those constraints as ordinary impulse
        /// joints. With stiff articulated chains, that impulse joint
        /// fights the multibody solver and can pull the chain hard
        /// toward the constraint anchor — useful escape hatch when
        /// debugging a chain that otherwise drifts under loop closure.
        const SKIP_LOOP_CLOSURES = 0b0100;
        /// If set, the motor entries baked into each joint by the loader
        /// (`<joint frictionloss>`, and the actuator motors set at runtime)
        /// are stripped before the joint is handed to the multibody. Only the
        /// multibody path is affected — the impulse-joint path still sees
        /// motors. Passive `<joint stiffness>` springs are **not** motors and
        /// are unaffected; use [`SKIP_JOINT_SPRINGS`](Self::SKIP_JOINT_SPRINGS)
        /// for those. Useful when comparing against a pure kinematic chain.
        const SKIP_JOINT_MOTORS = 0b1000;
        /// If set, the joint limits baked into each joint by the loader
        /// (`<joint range>`) are stripped before the joint is handed to
        /// the multibody. Only the multibody path is affected — the
        /// impulse-joint path still sees limits. Useful when a model's
        /// declared `range` interacts badly with the loader's resolved
        /// rest pose (the limit then fights the joint immediately at
        /// t=0).
        const SKIP_JOINT_LIMITS = 0b1_0000;
        /// If set, MJCF `<joint stiffness>` passive springs are **not**
        /// installed as implicit springs on the multibody. Only the multibody
        /// path is affected. The springs are integrated implicitly by default
        /// (stable for stiff springs on low-inertia links); set this to compare
        /// against a spring-free chain or to debug a model whose declared
        /// `springref`/`stiffness` is unexpected.
        const SKIP_JOINT_SPRINGS = 0b10_0000;
    }
}

/// How the loader maps MJCF `contype` / `conaffinity` to rapier
/// [`InteractionGroups`](rapier3d::geometry::InteractionGroups).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum ContactFilterMode {
    /// `memberships = filter = contype | conaffinity` (default). MJCF's
    /// "OR" rule is equivalent to rapier's "AND" rule on this encoding for
    /// the common case where each geom has `contype == conaffinity`.
    #[default]
    Symmetric,
    /// `memberships = contype`, `filter = conaffinity`. Matches MJCF only
    /// when geoms are deliberately partitioned into types vs. affinities.
    Asymmetric,
}

/// Configuration for [`MjcfRobot::from_model`](super::MjcfRobot::from_model).
#[derive(Clone, Debug)]
pub struct MjcfLoaderOptions {
    /// Build colliders for `<geom>` elements that participate in contact
    /// (`contype != 0` or `conaffinity != 0`). Default `true`.
    pub create_colliders_from_collision_shapes: bool,
    /// Build colliders for `<geom>` elements that don't participate in
    /// contact (visual-only, `contype == 0 && conaffinity == 0`). Default
    /// `false`.
    pub create_colliders_from_visual_shapes: bool,
    /// Use the model's `<inertial>` elements (or compiler-derived inertias)
    /// to set the rigid-body's mass properties. Default `true`.
    pub apply_imported_mass_props: bool,
    /// Allow contacts between two bodies sharing a joint. Default `false`.
    pub enable_joint_collisions: bool,
    /// If `true`, the rigid-bodies at the root of the kinematic chains are
    /// initialized as [`RigidBodyType::Fixed`](rapier3d::dynamics::RigidBodyType::Fixed). Default `false`.
    pub make_roots_fixed: bool,
    /// Trimesh flags applied to mesh colliders when the default mesh
    /// converter is used.
    pub trimesh_flags: TriMeshFlags,
    /// Custom mesh converter. Set to (e.g.) [`MeshConverter::ConvexHull`]
    /// to make every mesh collider a convex hull. Default `None` (use
    /// `TriMeshWithFlags(trimesh_flags)`).
    #[cfg(feature = "__meshloader_is_enabled")]
    pub mesh_converter: Option<MeshConverter>,
    /// Transform appended to every created rigid-body (default identity).
    pub shift: Pose,
    /// Uniform scale applied to lengths read from the MJCF (default `1.0`).
    pub scale: Real,
    /// Blueprint applied to every collider created by the loader (default
    /// has density 0 — physical mass comes from `<inertial>`).
    pub collider_blueprint: ColliderBuilder,
    /// Blueprint applied to every rigid-body created by the loader (default
    /// dynamic — root bodies are fixed if [`Self::make_roots_fixed`]).
    pub rigid_body_blueprint: RigidBodyBuilder,
    /// How to encode `contype`/`conaffinity` into rapier `InteractionGroups`.
    pub contact_filter_mode: ContactFilterMode,
    /// If `true`, `<geom type="plane">` elements are skipped entirely (no
    /// collider is created, the geom doesn't contribute to derived inertia,
    /// and the body it belonged to is unaffected).
    ///
    /// Default `true`.
    pub skip_plane_geoms: bool,
    /// If `true`, skip the per-joint **motor** setup that the loader
    /// normally applies for `<joint stiffness>`, `<joint springdamper>`,
    /// and `<joint frictionloss>` attributes (springs and Coulomb-friction
    /// approximations).
    ///
    /// `<joint damping>` is intentionally **not** affected by this flag —
    /// per-DoF damping is a dynamics-level friction term, not a motor, and
    /// the multibody-joint insertion path still routes `<joint damping>`
    /// values through the multibody's per-DoF damping vector. (The
    /// impulse-joint path has no per-DoF damping buffer, so damping is
    /// lost there when this flag is on.)
    ///
    /// The joint kinematic structure (type, axis, anchor, limits) is still
    /// honored. Useful when:
    ///
    /// - you want pure kinematic dynamics for debugging,
    /// - the model declares spring stiffness that's too high for your
    ///   simulation timestep,
    /// - you intend to drive every joint yourself through
    ///   [`MjcfRobotHandles::apply_controls`](super::MjcfRobotHandles::apply_controls)
    ///   and don't want the loader's default motors fighting your controller.
    ///
    /// Default `false`.
    pub disable_joint_motors: bool,
}

impl Default for MjcfLoaderOptions {
    fn default() -> Self {
        Self {
            create_colliders_from_collision_shapes: true,
            create_colliders_from_visual_shapes: false,
            apply_imported_mass_props: true,
            enable_joint_collisions: false,
            make_roots_fixed: false,
            trimesh_flags: TriMeshFlags::all(),
            #[cfg(feature = "__meshloader_is_enabled")]
            mesh_converter: None,
            shift: Pose::IDENTITY,
            scale: 1.0,
            collider_blueprint: ColliderBuilder::default().density(0.0),
            rigid_body_blueprint: RigidBodyBuilder::dynamic(),
            contact_filter_mode: ContactFilterMode::Symmetric,
            skip_plane_geoms: true,
            disable_joint_motors: false,
        }
    }
}
