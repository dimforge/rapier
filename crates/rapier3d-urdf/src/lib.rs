//! ## URDF loader for the Rapier physics engine
//!
//! Rapier is a set of 2D and 3D physics engines for games, animation, and robotics. The `rapier3d-urdf`
//! crate lets you convert an URDF file into a set of rigid-bodies, colliders, and joints, for usage with the
//! `rapier3d` physics engine.
//!
//! ## Optional cargo features
//!
//! - `stl`: enables loading STL meshes referenced by the URDF file.
//! - `collada`: enables loading Collada (`.dae`) meshes referenced by the URDF file.
//! - `wavefront`: enables loading Wavefront (`.obj`) meshes referenced by the URDF file.
//!
//! ## Limitations
//!
//! Are listed below some known limitations you might want to be aware of before picking this library. Contributions to
//! improve
//! these elements are very welcome!
//!
//! - Mesh file types are limited. Contributions are welcome. You may check the `rapier3d-meshloader`
//!   repository for an example of mesh loader.
//! - When inserting joints as multibody joints, they will be reset to their neutral position (all coordinates = 0).
//! - The following fields are currently ignored:
//!     - `Joint::dynamics`
//!     - `Joint::limit.effort` / `limit.velocity`
//!     - `Joint::mimic`
//!     - `Joint::safety_controller`

#![warn(missing_docs)]

use na::RealField;
use rapier3d::{
    dynamics::{
        GenericJoint, GenericJointBuilder, ImpulseJointHandle, ImpulseJointSet, JointAxesMask,
        JointAxis, MassProperties, MultibodyJointHandle, MultibodyJointSet, RigidBody,
        RigidBodyBuilder, RigidBodyHandle, RigidBodySet, RigidBodyType,
    },
    geometry::{
        Collider, ColliderBuilder, ColliderHandle, ColliderSet, MeshConverter, SharedShape,
        TriMeshFlags,
    },
    glamx::EulerRot,
    math::{Pose, Real, Rotation, Vector},
    na,
};
use std::collections::{HashMap, HashSet};
use std::path::Path;
use urdf_rs::{Geometry, Inertial, Joint, Pose as UrdfPose, Robot};

pub use urdf_rs;

#[cfg(doc)]
use rapier3d::dynamics::Multibody;

bitflags::bitflags! {
    /// Options applied to multibody joints created from the URDF joints.
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
    pub struct UrdfMultibodyOptions: u8 {
        /// If this flag is set, the created multibody joint will be marked as kinematic.
        ///
        /// A kinematic joint is entirely controlled by the user (it is not affected by any force).
        /// This particularly useful if you intend to control the robot through inverse-kinematics.
        const JOINTS_ARE_KINEMATIC = 0b0001;
        /// If enabled, any contact between two links belonging to the same generated multibody robot will
        /// be ignored.
        ///
        /// This is useful if the generated colliders are known to be overlapping (e.g. if creating colliders
        /// from visual meshes was enabled) or that collision detection is not needed a computationally
        /// expensive (e.g. if any of these colliders is a high-quality triangle mesh).
        const DISABLE_SELF_CONTACTS = 0b0010;
    }
}

/// The index of an urdf link.
pub type LinkId = usize;

/// A set of configurable options for loading URDF files.
#[derive(Clone, Debug)]
pub struct UrdfLoaderOptions {
    /// If `true` one collider will be created for each **collision** shape from the urdf file (default: `true`).
    pub create_colliders_from_collision_shapes: bool,
    /// If `true` one collider will be created for each **visual** shape from the urdf file (default: `false`).
    ///
    /// Note that visual shapes are usually significantly higher-resolution than collision shapes.
    /// Most of the time they might also overlap, or generate a lot of contacts due to them being
    /// thin triangle meshes.
    ///
    /// So if this option is set to `true`, it is recommended to also keep
    /// [`UrdfLoaderOptions::enable_joint_collisions`] set to `false`. If the model is then added
    /// to the physics sets using multibody joints, it is recommended to call
    /// [`UrdfRobot::insert_using_multibody_joints`] with the [`UrdfMultibodyOptions::DISABLE_SELF_CONTACTS`]
    /// flag enabled.
    pub create_colliders_from_visual_shapes: bool,
    /// If `true`, the mass properties (center-of-mass, mass, and angular inertia) read from the urdf
    /// file will be added to the corresponding rigid-body (default: `true`).
    ///
    /// Note that by default, all colliders created will be given a density of 0.0, meaning that,
    /// by default, the imported mass properties are the only ones added to the created rigid-bodies.
    /// To give colliders a non-zero density, see [`UrdfLoaderOptions::collider_blueprint`].
    pub apply_imported_mass_props: bool,
    /// If `true`, collisions between two links sharing a joint will be disabled (default: `false`).
    ///
    /// It is strongly recommended to leave this to `false` unless you are certain adjacent links
    /// colliders don’t overlap.
    pub enable_joint_collisions: bool,
    /// If `true`, the rigid-body at the root of the kinematic chains will be initialized as [`RigidBodyType::Fixed`]
    /// (default: `false`).
    pub make_roots_fixed: bool,
    /// This is the set of flags set on all the loaded triangle meshes (default: [`TriMeshFlags::all`]).
    ///
    /// Note that the default enables all the flags. This is operating under the assumption that the provided
    /// mesh are generally well-formed and properly oriented (2-manifolds with outward normals).
    ///
    /// Note that this field is only used when [`Self::mesh_converter`] is left at its default
    /// value. If a custom [`MeshConverter`] is supplied, these flags are ignored.
    pub trimesh_flags: TriMeshFlags,
    /// Controls how every mesh referenced by the URDF file is converted into a Rapier
    /// collider shape (default: `None`, in which case the loader falls back to
    /// [`MeshConverter::TriMeshWithFlags`] using [`Self::trimesh_flags`]).
    ///
    /// Set this to e.g. [`MeshConverter::Obb`] to get cheap proxy shapes for physics
    /// while keeping the original meshes available for rendering through other means.
    pub mesh_converter: Option<MeshConverter>,
    /// The transform appended to every created rigid-bodies (default: `Pose::IDENTITY`).
    pub shift: Pose,
    /// A uniform scale applied to every length read from the URDF file (default: `1.0`).
    ///
    /// This affects link positions, joint anchors, mesh scaling, primitive shape sizes,
    /// inertial offsets, and prismatic joint limits. Mass and inertia tensors are left
    /// unchanged. The [`Self::shift`] is applied *after* the scaling.
    pub scale: Real,
    /// A description of the collider properties that need to be applied to every collider created
    /// by the loader (default: `ColliderBuilder::default().density(0.0)`).
    ///
    /// This collider builder will be used for initializing every collider created by the loader.
    /// The shape specified by this builder isn’t important and will be replaced by the shape read
    /// from the urdf file.
    ///
    /// Note that by default, the collider is given a density of 0.0 so that it doesn’t contribute
    /// to its parent rigid-body’s mass properties (since they should be already provided by the
    /// urdf file assuming the [`UrdfLoaderOptions::apply_imported_mass_props`] wasn’t set `false`).
    pub collider_blueprint: ColliderBuilder,
    /// A description of the rigid-body properties that need to be applied to every rigid-body
    /// created by the loader (default: `RigidBodyBuilder::dynamic()`).
    ///
    /// This rigid-body builder will be used for initializing every rigid-body created by the loader.
    /// The rigid-body type is not important as it will always be set to [`RigidBodyType::Dynamic`]
    /// for non-root links. Root links will be set to [`RigidBodyType::Fixed`] instead of
    /// [`RigidBodyType::Dynamic`] if the [`UrdfLoaderOptions::make_roots_fixed`] is set to `true`.
    pub rigid_body_blueprint: RigidBodyBuilder,
    /// If `true` (default), URDF links that have no `<visual>`, `<collision>`, nor `<inertial>`
    /// element are removed and the joints around them are spliced together so the kinematic
    /// chain stays equivalent.
    ///
    /// Concretely:
    /// - When an empty link is attached to its parent by a **fixed** joint, the empty link
    ///   and that joint are dropped, and all its children (whatever their joint types) are
    ///   attached to its parent directly, with their joint origins composed so they end up at
    ///   the same pose.
    /// - When an empty link is attached to its parent by a **moving** joint and to at least
    ///   one child by a **fixed** joint, the empty link and its parent joint are dropped, and
    ///   one of these fixed children (preferably a non-empty one) becomes its representative:
    ///   its joint takes over the type, axis, limits, and pivot of the dropped parent joint.
    ///   The other children of the empty link (whatever their joint types) are attached to the
    ///   representative with their joint origins composed. This way, all the fixed children
    ///   keep moving together as one rigid group, driven by the single original moving joint.
    /// - An empty link attached to its parent and to all its children by moving joints is
    ///   kept, since it is needed to chain these joints.
    /// - When the empty link is the **root** and is connected to its child(ren) by a fixed
    ///   joint, the link and the joint are removed, and the child is forced to be a fixed
    ///   rigid-body (independently of [`Self::make_roots_fixed`]).
    /// - Empty leaves (no children) are simply dropped.
    ///
    /// A dropped parent joint is recorded in the [`UrdfJoint::merged_urdf_joint_indices`] of
    /// at most one joint: the joint of the representative (or of one of the fixed children if
    /// the dropped joint is fixed). Some dropped joints have no counterpart at all, e.g., the
    /// joint of an empty leaf, or the fixed joint of an empty link with only moving children.
    ///
    /// This avoids ending up with bodyless, mass-less rigid-bodies that exist only to act
    /// as named frames in the URDF (a common pattern for `world` anchors and `*_tcp`
    /// tool-center-points).
    pub squeeze_empty_fixed_links: bool,
}

impl Default for UrdfLoaderOptions {
    fn default() -> Self {
        Self {
            create_colliders_from_collision_shapes: true,
            create_colliders_from_visual_shapes: false,
            apply_imported_mass_props: true,
            enable_joint_collisions: false,
            make_roots_fixed: false,
            trimesh_flags: TriMeshFlags::all(),
            mesh_converter: None,
            shift: Pose::IDENTITY,
            scale: 1.0,
            collider_blueprint: ColliderBuilder::default().density(0.0),
            rigid_body_blueprint: RigidBodyBuilder::dynamic(),
            squeeze_empty_fixed_links: true,
        }
    }
}

/// An auxiliary triangle-mesh visual representation associated to a [`UrdfCollider`].
///
/// The loader populates this whenever a [`UrdfLoaderOptions::mesh_converter`] other
/// than the default trimesh converter is configured (e.g. [`MeshConverter::Obb`]),
/// so the original mesh data isn't lost when the collider is a cheap proxy shape.
#[derive(Clone, Debug)]
pub struct UrdfVisual {
    /// The triangle-mesh shape that visually represents the collider.
    pub shape: SharedShape,
    /// Pose of the visual mesh in the parent collider's local frame. When the
    /// collider is a proxy whose pose differs from the original mesh's natural
    /// pose (e.g. the rotated cuboid produced by [`MeshConverter::Obb`]), this
    /// offset puts the mesh back at its source location.
    pub local_pose: Pose,
}

/// A collider produced by the URDF loader together with an optional visual override.
#[derive(Clone, Debug)]
pub struct UrdfCollider {
    /// The collider built for this URDF shape.
    pub collider: Collider,
    /// Optional visual triangle-mesh representation. Populated automatically when
    /// the loader had to substitute the original mesh with a proxy shape (e.g. via
    /// [`MeshConverter::Obb`]); otherwise `None`, in which case the collider's own
    /// shape is meant to be rendered.
    pub visual: Option<UrdfVisual>,
}

/// An urdf link loaded as a rapier [`RigidBody`] and its [`UrdfCollider`]s.
#[derive(Clone, Debug)]
pub struct UrdfLink {
    /// The rigid-body created for this link.
    pub body: RigidBody,
    /// All the colliders built from the URDF visual and/or collision shapes (if the
    /// corresponding [`UrdfLoaderOptions`] option is enabled), each paired with an
    /// optional visual override (see [`UrdfCollider::visual`]).
    pub colliders: Vec<UrdfCollider>,
    /// Index, in the original [`Robot::links`], of the URDF link this link was built from.
    ///
    /// This can differ from this link's index in [`UrdfRobot::links`] when empty links were
    /// removed (see [`UrdfLoaderOptions::squeeze_empty_fixed_links`]).
    pub urdf_link_index: usize,
}

/// An urdf joint loaded as a rapier [`GenericJoint`].
#[derive(Clone, Debug)]
pub struct UrdfJoint {
    /// The rapier version for the corresponding urdf joint.
    pub joint: GenericJoint,
    /// Index of the rigid-body (from the [`UrdfRobot`] array) at the first
    /// endpoint of this joint.
    pub link1: LinkId,
    /// Index of the rigid-body (from the [`UrdfRobot`] array) at the second
    /// endpoint of this joint.
    pub link2: LinkId,
    /// Index, in the original [`Robot::joints`], of the URDF joint this joint was built from.
    ///
    /// This is the URDF joint attached to the child link [`Self::link2`]. It can differ from
    /// this joint's index in [`UrdfRobot::joints`] when empty links were removed (see
    /// [`UrdfLoaderOptions::squeeze_empty_fixed_links`]). In that case, the URDF parent link
    /// of the joint might also have been replaced by another one (see [`Self::link1`]).
    pub urdf_joint_index: usize,
    /// Indices, in the original [`Robot::joints`], of the URDF joints that were merged into
    /// this joint when the empty links between them were removed (see
    /// [`UrdfLoaderOptions::squeeze_empty_fixed_links`]).
    ///
    /// They are ordered from the child side to the parent side. The last one (if any) is the
    /// joint this joint inherited its type, axis, limits, dynamics, and mimic from (see
    /// [`Self::source_urdf_joint_index`]). This is empty if no joint was merged into this one.
    ///
    /// A URDF joint is merged into at most one joint, even if the empty link it led to had
    /// several children.
    pub merged_urdf_joint_indices: Vec<usize>,
}

impl UrdfJoint {
    /// Index, in the original [`Robot::joints`], of the URDF joint this joint inherited its
    /// type, axis, limits, dynamics, and mimic from.
    ///
    /// This is [`Self::urdf_joint_index`], unless other joints were merged into this one (see
    /// [`Self::merged_urdf_joint_indices`]).
    pub fn source_urdf_joint_index(&self) -> usize {
        self.merged_urdf_joint_indices
            .last()
            .copied()
            .unwrap_or(self.urdf_joint_index)
    }
}

/// A robot represented as a set of rapier rigid-bodies, colliders, and joints.
#[derive(Clone, Debug)]
pub struct UrdfRobot {
    /// The bodies and colliders loaded from the urdf file.
    ///
    /// This vector follows the order of [`Robot::links`], but some links might be missing if
    /// [`UrdfLoaderOptions::squeeze_empty_fixed_links`] is enabled. Use
    /// [`UrdfLink::urdf_link_index`] to find the URDF link each element was built from.
    pub links: Vec<UrdfLink>,
    /// The joints loaded from the urdf file.
    ///
    /// This vector follows the order of [`Robot::joints`], but some joints might be missing if
    /// [`UrdfLoaderOptions::squeeze_empty_fixed_links`] is enabled. Use
    /// [`UrdfJoint::urdf_joint_index`] to find the URDF joint each element was built from.
    pub joints: Vec<UrdfJoint>,
}

/// Handle of a joint read from the URDF file and inserted into rapier’s `ImpulseJointSet`
/// or a `MultibodyJointSet`.
pub struct UrdfJointHandle<JointHandle> {
    /// The inserted joint handle.
    pub joint: JointHandle,
    /// The handle of the first rigid-body attached by this joint.
    pub link1: RigidBodyHandle,
    /// The handle of the second rigid-body attached by this joint.
    pub link2: RigidBodyHandle,
}

/// A handle to an inserted URDF collider together with its optional visual override.
pub struct UrdfColliderHandle {
    /// Handle of the inserted collider.
    pub handle: ColliderHandle,
    /// Optional visual triangle-mesh representation associated with this collider
    /// (see [`UrdfCollider::visual`]).
    pub visual: Option<UrdfVisual>,
}

/// The handles related to a link read from the URDF file and inserted into Rapier’s
/// `RigidBodySet` and `ColliderSet`.
pub struct UrdfLinkHandle {
    /// Handle of the inserted link’s rigid-body.
    pub body: RigidBodyHandle,
    /// Handle of the colliders attached to [`Self::body`], each paired with an
    /// optional visual override.
    pub colliders: Vec<UrdfColliderHandle>,
}

/// Handles to all the Rapier objects created when inserting this robot into Rapier’s
/// `RigidBodySet`, `ColliderSet`, `ImpulseJointSet`, `MultibodyJointSet`.
pub struct UrdfRobotHandles<JointHandle> {
    /// The handles related to each URDF robot link.
    pub links: Vec<UrdfLinkHandle>,
    /// The handles related to each URDF robot joint.
    pub joints: Vec<UrdfJointHandle<JointHandle>>,
}

impl UrdfRobot {
    /// Parses a URDF file and returns both the rapier objects (`UrdfRobot`) and the original urdf
    /// structures (`Robot`).
    ///
    /// Both structures are arranged in the same order, but some links and joints of the `Robot`
    /// might have no counterpart in the `UrdfRobot` if [`UrdfLoaderOptions::squeeze_empty_fixed_links`]
    /// is enabled. Use [`UrdfLink::urdf_link_index`] and [`UrdfJoint::urdf_joint_index`] to find the
    /// `Robot` element each `UrdfRobot` element was built from.
    ///
    /// If the URDF file references external meshes, they will be loaded automatically if the format
    /// is supported. The format is detected from the file’s extension. All the mesh formats are
    /// disabled by default and can be enabled through cargo features (e.g. the `stl` feature of
    /// this crate enabled loading referenced meshes in stl format).
    ///
    /// # Parameters
    /// - `path`: the path of the URDF file.
    /// - `options`: customize the creation of rapier objects from the URDF description.
    /// - `mesh_dir`: the base directory containing the meshes referenced by the URDF file. When
    ///   a mesh reference is found in the URDF file, this `mesh_dir` is appended
    ///   to the file path. If `mesh_dir` is `None` then the mesh directory is assumed
    ///   to be the same directory as the one containing the URDF file.
    pub fn from_file(
        path: impl AsRef<Path>,
        options: UrdfLoaderOptions,
        mesh_dir: Option<&Path>,
    ) -> anyhow::Result<(Self, Robot)> {
        let path = path.as_ref().canonicalize()?;
        let mesh_dir = mesh_dir
            .or_else(|| path.parent())
            .unwrap_or_else(|| Path::new("./"));
        let raw = std::fs::read_to_string(&path)?;
        let sanitized = sanitize_urdf_str(&raw);
        let robot = urdf_rs::read_from_string(&sanitized)?;
        Ok((Self::from_robot(&robot, options, mesh_dir), robot))
    }

    /// Parses a string in URDF format and returns both the rapier objects (`UrdfRobot`) and the original urdf
    /// structures (`Robot`).
    ///
    /// Both structures are arranged in the same order, but some links and joints of the `Robot`
    /// might have no counterpart in the `UrdfRobot` if [`UrdfLoaderOptions::squeeze_empty_fixed_links`]
    /// is enabled. Use [`UrdfLink::urdf_link_index`] and [`UrdfJoint::urdf_joint_index`] to find the
    /// `Robot` element each `UrdfRobot` element was built from.
    ///
    /// If the URDF file references external meshes, they will be loaded automatically if the format
    /// is supported. The format is detected from the file’s extension. All the mesh formats are
    /// disabled by default and can be enabled through cargo features (e.g. the `stl` feature of
    /// this crate enabled loading referenced meshes in stl format).
    ///
    /// # Parameters
    /// - `str`: the string content of an URDF file.
    /// - `options`: customize the creation of rapier objects from the URDF description.
    /// - `mesh_dir`: the base directory containing the meshes referenced by the URDF file. When
    ///   a mesh reference is found in the URDF file, this `mesh_dir` is appended
    ///   to the file path.
    pub fn from_str(
        str: &str,
        options: UrdfLoaderOptions,
        mesh_dir: &Path,
    ) -> anyhow::Result<(Self, Robot)> {
        let sanitized = sanitize_urdf_str(str);
        let robot = urdf_rs::read_from_string(&sanitized)?;
        Ok((Self::from_robot(&robot, options, mesh_dir), robot))
    }

    /// From an already loaded urdf file as a `Robot`, this creates the matching rapier objects
    /// (`UrdfRobot`).
    ///
    /// Both structures are arranged in the same order, but some links and joints of the `Robot`
    /// might have no counterpart in the `UrdfRobot` if [`UrdfLoaderOptions::squeeze_empty_fixed_links`]
    /// is enabled. Use [`UrdfLink::urdf_link_index`] and [`UrdfJoint::urdf_joint_index`] to find the
    /// `Robot` element each `UrdfRobot` element was built from.
    ///
    /// If the URDF file references external meshes, they will be loaded automatically if the format
    /// is supported. The format is detected mostly from the file’s extension. All the mesh formats are
    /// disabled by default and can be enabled through cargo features (e.g. the `stl` feature of
    /// this crate enabled loading referenced meshes in stl format).
    ///
    /// # Parameters
    /// - `robot`: the robot loaded from an URDF file.
    /// - `options`: customize the creation of rapier objects from the URDF description.
    /// - `mesh_dir`: the base directory containing the meshes referenced by the URDF file. When
    ///   a mesh reference is found in the URDF file, this `mesh_dir` is appended
    ///   to the file path.
    pub fn from_robot(robot: &Robot, options: UrdfLoaderOptions, mesh_dir: &Path) -> Self {
        // Optionally rewrite the URDF graph to remove empty links connected by fixed
        // joints so we don't end up with bodyless rigid-bodies that exist only to
        // serve as named frames.
        let (robot_owned, squeeze): (std::borrow::Cow<Robot>, SqueezeResult) =
            if options.squeeze_empty_fixed_links {
                let mut clone = robot.clone();
                let squeeze = squeeze_empty_fixed_links(&mut clone);
                (std::borrow::Cow::Owned(clone), squeeze)
            } else {
                (
                    std::borrow::Cow::Borrowed(robot),
                    SqueezeResult::identity(robot),
                )
            };
        let robot = robot_owned.as_ref();
        let SqueezeResult {
            force_fixed: force_fixed_links,
            link_ids,
            joint_ids,
            mut merged_joint_ids,
            child_offsets,
        } = squeeze;

        let mut name_to_link_id = HashMap::new();
        let mut link_is_root = vec![true; robot.links.len()];
        let mut links: Vec<_> = robot
            .links
            .iter()
            .enumerate()
            .map(|(id, link)| {
                name_to_link_id.insert(&link.name, id);
                let mut colliders = vec![];
                if options.create_colliders_from_collision_shapes {
                    colliders.extend(link.collision.iter().flat_map(|co| {
                        urdf_to_colliders(&options, mesh_dir, &co.geometry, &co.origin)
                    }))
                }
                if options.create_colliders_from_visual_shapes {
                    colliders.extend(link.visual.iter().flat_map(|vis| {
                        urdf_to_colliders(&options, mesh_dir, &vis.geometry, &vis.origin)
                    }))
                }
                let mut body = urdf_to_rigid_body(&options, &link.inertial);
                let new_pos = options.shift * body.position();
                body.set_position(new_pos, false);
                UrdfLink {
                    body,
                    colliders,
                    urdf_link_index: link_ids[id],
                }
            })
            .collect();
        // A joint places its child link relative to its parent link, so the parent must be
        // placed first.
        let mut joints: Vec<Option<UrdfJoint>> = vec![None; robot.joints.len()];
        for i in joints_from_roots_to_leaves(robot) {
            let joint = &robot.joints[i];
            let link1 = name_to_link_id[&joint.parent.link];
            let link2 = name_to_link_id[&joint.child.link];
            let pose1 = *links[link1].body.position();
            let rb2 = &mut links[link2].body;
            let generic_joint = urdf_to_joint(&options, joint, &child_offsets[i], &pose1, rb2);
            link_is_root[link2] = false;

            joints[i] = Some(UrdfJoint {
                joint: generic_joint,
                link1,
                link2,
                urdf_joint_index: joint_ids[i],
                merged_urdf_joint_indices: std::mem::take(&mut merged_joint_ids[i]),
            });
        }
        let joints = joints.into_iter().flatten().collect();

        if options.make_roots_fixed {
            for (link, is_root) in links.iter_mut().zip(link_is_root.iter().copied()) {
                if is_root {
                    link.body.set_body_type(RigidBodyType::Fixed, false)
                }
            }
        }

        // Force-fix every link the squeeze step orphaned from the world: the empty
        // root that used to anchor them via a fixed joint is gone, but they were
        // meant to be rigidly attached to the world.
        if !force_fixed_links.is_empty() {
            for (id, link) in robot.links.iter().enumerate() {
                if force_fixed_links.contains(&link.name) {
                    links[id].body.set_body_type(RigidBodyType::Fixed, false);
                }
            }
        }

        Self { links, joints }
    }

    /// Inserts all the robots elements to the rapier rigid-body, collider, and impulse joint, sets.
    ///
    /// Joints are represented as impulse joints. This implies that joint constraints are simulated
    /// in full coordinates using impulses. For a reduced-coordinates approach, see
    /// [`UrdfRobot::insert_using_multibody_joints`].
    pub fn insert_using_impulse_joints(
        self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        joint_set: &mut ImpulseJointSet,
    ) -> UrdfRobotHandles<ImpulseJointHandle> {
        let links: Vec<_> = self
            .links
            .into_iter()
            .map(|link| {
                let body = rigid_body_set.insert(link.body);
                let colliders = link
                    .colliders
                    .into_iter()
                    .map(|co| {
                        let handle =
                            collider_set.insert_with_parent(co.collider, body, rigid_body_set);
                        UrdfColliderHandle {
                            handle,
                            visual: co.visual,
                        }
                    })
                    .collect();
                UrdfLinkHandle { body, colliders }
            })
            .collect();
        let joints: Vec<_> = self
            .joints
            .into_iter()
            .map(|joint| {
                let link1 = links[joint.link1].body;
                let link2 = links[joint.link2].body;
                let joint = joint_set.insert(link1, link2, joint.joint, false);
                UrdfJointHandle {
                    joint,
                    link1,
                    link2,
                }
            })
            .collect();

        UrdfRobotHandles { links, joints }
    }

    /// Inserts all the robots elements to the rapier rigid-body, collider, and multibody joint, sets.
    ///
    /// Joints are represented as multibody joints. This implies that the robot as a whole can be
    /// accessed as a single [`Multibody`] from the [`MultibodyJointSet`]. That multibody uses reduced
    /// coordinates for modeling joints, meaning that it will be very close to the way they are usually
    /// represented for robotics applications. Multibodies also support inverse kinematics.
    pub fn insert_using_multibody_joints(
        self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        joint_set: &mut MultibodyJointSet,
        multibody_options: UrdfMultibodyOptions,
    ) -> UrdfRobotHandles<Option<MultibodyJointHandle>> {
        let links: Vec<_> = self
            .links
            .into_iter()
            .map(|link| {
                let body = rigid_body_set.insert(link.body);
                let colliders = link
                    .colliders
                    .into_iter()
                    .map(|co| {
                        let handle =
                            collider_set.insert_with_parent(co.collider, body, rigid_body_set);
                        UrdfColliderHandle {
                            handle,
                            visual: co.visual,
                        }
                    })
                    .collect();
                UrdfLinkHandle { body, colliders }
            })
            .collect();
        let joints: Vec<_> = self
            .joints
            .into_iter()
            .map(|joint| {
                let link1 = links[joint.link1].body;
                let link2 = links[joint.link2].body;
                let joint =
                    if multibody_options.contains(UrdfMultibodyOptions::JOINTS_ARE_KINEMATIC) {
                        joint_set.insert_kinematic(link1, link2, joint.joint, false)
                    } else {
                        joint_set.insert(link1, link2, joint.joint, false)
                    };

                if let Some(joint) = joint {
                    let (multibody, _) = joint_set.get_mut(joint).unwrap_or_else(|| unreachable!());
                    multibody.set_self_contacts_enabled(
                        !multibody_options.contains(UrdfMultibodyOptions::DISABLE_SELF_CONTACTS),
                    );
                }

                UrdfJointHandle {
                    joint,
                    link1,
                    link2,
                }
            })
            .collect();

        UrdfRobotHandles { links, joints }
    }

    /// Appends a transform to all the rigid-bodie of this robot.
    pub fn append_transform(&mut self, transform: &Pose) {
        for link in &mut self.links {
            let new_pos = transform * link.body.position();
            link.body.set_position(new_pos, true);
        }
    }
}

#[rustfmt::skip]
fn urdf_to_rigid_body(options: &UrdfLoaderOptions, inertial: &Inertial) -> RigidBody {
    let mut origin = urdf_to_pose(&inertial.origin);
    origin.translation *= options.scale;
    let mut builder = options.rigid_body_blueprint.clone();
    builder.body_type = RigidBodyType::Dynamic;

    if options.apply_imported_mass_props {
        builder = builder.additional_mass_properties(MassProperties::with_inertia_matrix(
            origin.translation,
            inertial.mass.value as Real,
            // See http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model#Inertia
            na::Matrix3::new(
                inertial.inertia.ixx as Real, inertial.inertia.ixy as Real, inertial.inertia.ixz as Real,
                inertial.inertia.ixy as Real, inertial.inertia.iyy as Real, inertial.inertia.iyz as Real,
                inertial.inertia.ixz as Real, inertial.inertia.iyz as Real,inertial.inertia.izz as Real,
            ).into(),
        ))
    }

    builder.build()
}

fn urdf_to_colliders(
    options: &UrdfLoaderOptions,
    _mesh_dir: &Path, // Unused if none of the meshloader features is enabled.
    geometry: &Geometry,
    origin: &UrdfPose,
) -> Vec<UrdfCollider> {
    let s = options.scale;

    // Per-shape data: the collider shape, the extra local pose to apply on top of
    // the URDF origin (e.g. cylinder Z-up rotation, or the offset returned by
    // `MeshConverter::Obb`), and an optional triangle-mesh visual override.
    let mut shapes: Vec<(SharedShape, Pose, Option<UrdfVisual>)> = Vec::new();

    match &geometry {
        Geometry::Box { size } => {
            // A zero-sized box is the sanitizer's stand-in for an empty `<geometry/>`
            // tag (some URDFs declare `<collision>` blocks without an actual shape, which
            // urdf-rs rejects). Skip it so we don't emit a degenerate collider.
            if size[0] == 0.0 && size[1] == 0.0 && size[2] == 0.0 {
                return Vec::new();
            }
            shapes.push((
                SharedShape::cuboid(
                    size[0] as Real * s / 2.0,
                    size[1] as Real * s / 2.0,
                    size[2] as Real * s / 2.0,
                ),
                Pose::IDENTITY,
                None,
            ));
        }
        Geometry::Cylinder { radius, length } => {
            // This rotation will make the cylinder Z-up as per the URDF spec,
            // instead of rapier’s default Y-up.
            shapes.push((
                SharedShape::cylinder(*length as Real * s / 2.0, *radius as Real * s),
                Pose::rotation(Vector::X * Real::frac_pi_2()),
                None,
            ));
        }
        Geometry::Capsule { radius, length } => {
            shapes.push((
                SharedShape::capsule_z(*length as Real * s / 2.0, *radius as Real * s),
                Pose::IDENTITY,
                None,
            ));
        }
        Geometry::Sphere { radius } => {
            shapes.push((SharedShape::ball(*radius as Real * s), Pose::IDENTITY, None));
        }
        #[cfg(not(feature = "__meshloader_is_enabled"))]
        Geometry::Mesh { .. } => {
            log::error!(
                "Mesh loading is disabled by default. Enable one of the format features (`stl`, `collada`, `wavefront`) of `rapier3d-urdf` for mesh support."
            );
        }
        #[cfg(feature = "__meshloader_is_enabled")]
        Geometry::Mesh { filename, scale } => {
            let full_path = _mesh_dir.join(filename);
            let mesh_scale = scale
                .map(|sc| Vector::new(sc[0] as Real, sc[1] as Real, sc[2] as Real))
                .unwrap_or_else(|| Vector::splat(1.0))
                * s;

            let converter = options
                .mesh_converter
                .unwrap_or(MeshConverter::TriMeshWithFlags(options.trimesh_flags));
            // Whenever the user picks a converter that replaces the mesh with a
            // proxy shape (e.g. Obb), keep the original triangle mesh around as a
            // visual representation so renderers can still draw the source mesh.
            let needs_visual = !matches!(
                converter,
                MeshConverter::TriMesh | MeshConverter::TriMeshWithFlags(_)
            );
            let visual_converter = MeshConverter::TriMeshWithFlags(options.trimesh_flags);
            let Ok(loaded_mesh) =
                rapier3d_meshloader::load_from_path(full_path, &converter, mesh_scale)
            else {
                return Vec::new();
            };
            for loaded in loaded_mesh.into_iter().filter_map(|x| x.ok()) {
                let visual = if needs_visual {
                    rapier3d_meshloader::load_from_raw_mesh(
                        &loaded.raw_mesh,
                        &visual_converter,
                        mesh_scale,
                    )
                    .ok()
                    .map(|(visual_shape, visual_pose)| UrdfVisual {
                        shape: visual_shape,
                        // The collider sits at `urdf_origin * loaded.pose`; the
                        // visual mesh naturally sits at `urdf_origin * visual_pose`.
                        // Local offset relative to the collider:
                        local_pose: loaded.pose.inverse() * visual_pose,
                    })
                } else {
                    None
                };
                shapes.push((loaded.shape, loaded.pose, visual));
            }
        }
    }

    let mut local_pose = urdf_to_pose(origin);
    local_pose.translation *= s;

    shapes
        .into_iter()
        .map(move |(shape, shape_transform, visual)| {
            let mut builder = options.collider_blueprint.clone();
            builder.shape = shape;
            let collider = builder.position(local_pose * shape_transform).build();
            UrdfCollider { collider, visual }
        })
        .collect()
}

fn urdf_to_pose(pose: &UrdfPose) -> Pose {
    Pose::from_parts(
        Vector::new(
            pose.xyz[0] as Real,
            pose.xyz[1] as Real,
            pose.xyz[2] as Real,
        ),
        Rotation::from_euler(
            EulerRot::XYZ,
            pose.rpy[0] as Real,
            pose.rpy[1] as Real,
            pose.rpy[2] as Real,
        ),
    )
}

fn urdf_to_joint(
    options: &UrdfLoaderOptions,
    joint: &Joint,
    child_offset: &Pose,
    pose1: &Pose,
    link2: &mut RigidBody,
) -> GenericJoint {
    let locked_axes = match joint.joint_type {
        urdf_rs::JointType::Fixed => JointAxesMask::LOCKED_FIXED_AXES,
        urdf_rs::JointType::Continuous | urdf_rs::JointType::Revolute => {
            JointAxesMask::LOCKED_REVOLUTE_AXES
        }
        urdf_rs::JointType::Floating => JointAxesMask::empty(),
        urdf_rs::JointType::Planar => JointAxesMask::ANG_AXES | JointAxesMask::LIN_X,
        urdf_rs::JointType::Prismatic => JointAxesMask::LOCKED_PRISMATIC_AXES,
        urdf_rs::JointType::Spherical => JointAxesMask::LOCKED_SPHERICAL_AXES,
    };
    let mut joint_to_parent = urdf_to_pose(&joint.origin);
    joint_to_parent.translation *= options.scale;
    let joint_axis = Vector::new(
        joint.axis.xyz[0] as Real,
        joint.axis.xyz[1] as Real,
        joint.axis.xyz[2] as Real,
    )
    .normalize_or_zero();

    // The pose of the child link in the joint frame, which isn't the identity if the joint was
    // merged with the fixed joints following it.
    let mut child_offset = *child_offset;
    child_offset.translation *= options.scale;
    let joint_to_child = child_offset.inverse();

    link2.set_position(pose1 * joint_to_parent * child_offset, false);

    let mut builder =
        GenericJointBuilder::new(locked_axes).contacts_enabled(options.enable_joint_collisions);

    if joint_axis != Vector::ZERO {
        // Build a single joint frame from the URDF joint axis and reuse it on both
        // sides — picking independent orthonormal bases for `local_axis1` /
        // `local_axis2` would yield mismatched secondary axes, leaving the joint
        // unsatisfied at rest and snapping the bodies on the first step.
        let basis = GenericJoint::complete_ang_frame(joint_axis);
        let basis = Pose::from_rotation(basis);
        builder = builder
            .local_frame1(joint_to_parent * basis)
            .local_frame2(joint_to_child * basis);
    } else {
        builder = builder
            .local_frame1(joint_to_parent)
            .local_frame2(joint_to_child);
    }

    match joint.joint_type {
        urdf_rs::JointType::Prismatic => {
            builder = builder.limits(
                JointAxis::LinX,
                [
                    joint.limit.lower as Real * options.scale,
                    joint.limit.upper as Real * options.scale,
                ],
            )
        }
        urdf_rs::JointType::Revolute => {
            builder = builder.limits(
                JointAxis::AngX,
                [joint.limit.lower as Real, joint.limit.upper as Real],
            )
        }
        _ => {}
    }

    // TODO: the following fields are currently ignored:
    //       - Joint::dynamics
    //       - Joint::limit.effort / limit.velocity
    //       - Joint::mimic
    //       - Joint::safety_controller
    builder.build()
}

/// Replaces empty `<geometry></geometry>` blocks with a zero-sized box placeholder so
/// that strict URDF parsers (e.g. `urdf-rs`) don't reject the file. The downstream
/// collider construction skips these placeholders.
fn sanitize_urdf_str(input: &str) -> String {
    const PLACEHOLDER: &str = "<box size=\"0 0 0\"/>";
    let mut out = String::with_capacity(input.len());
    let mut rem = input;
    while let Some(open_rel) = rem.find("<geometry") {
        out.push_str(&rem[..open_rel]);
        let after_open_tag = &rem[open_rel..];
        // Find the closing `>` of the opening tag to handle attributes / self-closing.
        let Some(open_end_rel) = after_open_tag.find('>') else {
            out.push_str(after_open_tag);
            return out;
        };
        let open_tag = &after_open_tag[..=open_end_rel];
        let body_start = open_end_rel + 1;
        // Self-closing `<geometry .../>` has nothing to sanitize.
        if open_tag.ends_with("/>") {
            out.push_str(open_tag);
            rem = &after_open_tag[body_start..];
            continue;
        }
        let body_and_after = &after_open_tag[body_start..];
        let Some(close_rel) = body_and_after.find("</geometry>") else {
            out.push_str(after_open_tag);
            return out;
        };
        let body = &body_and_after[..close_rel];
        let close_tag = "</geometry>";
        out.push_str(open_tag);
        if body.trim().is_empty() {
            out.push_str(PLACEHOLDER);
        } else {
            out.push_str(body);
        }
        out.push_str(close_tag);
        rem = &body_and_after[close_rel + close_tag.len()..];
    }
    out.push_str(rem);
    out
}

/// Returns `true` when the URDF link carries no `<visual>`, `<collision>`, nor
/// `<inertial>` element with mass or inertia. urdf-rs synthesises a default
/// (zero-mass, zero-inertia) `Inertial` when the tag is missing, so a fully
/// zero-valued inertial is treated as "no inertial".
fn is_link_empty(link: &urdf_rs::Link) -> bool {
    if !link.visual.is_empty() || !link.collision.is_empty() {
        return false;
    }
    let inertia = &link.inertial.inertia;
    link.inertial.mass.value == 0.0
        && inertia.ixx == 0.0
        && inertia.ixy == 0.0
        && inertia.ixz == 0.0
        && inertia.iyy == 0.0
        && inertia.iyz == 0.0
        && inertia.izz == 0.0
}

fn pose_to_urdf_pose(pose: &Pose) -> UrdfPose {
    let (rx, ry, rz) = pose.rotation.to_euler(EulerRot::XYZ);
    UrdfPose {
        xyz: urdf_rs::Vec3([
            pose.translation.x as f64,
            pose.translation.y as f64,
            pose.translation.z as f64,
        ]),
        rpy: urdf_rs::Vec3([rx as f64, ry as f64, rz as f64]),
    }
}

/// The indices of the joints of `robot`, ordered so that the joint attached to a link comes
/// before the joints attached to its children.
fn joints_from_roots_to_leaves(robot: &Robot) -> Vec<usize> {
    let mut child_joints: HashMap<&str, Vec<usize>> = HashMap::new();
    let mut child_links = HashSet::new();
    for (i, joint) in robot.joints.iter().enumerate() {
        child_joints
            .entry(joint.parent.link.as_str())
            .or_default()
            .push(i);
        child_links.insert(joint.child.link.as_str());
    }

    let mut order = Vec::with_capacity(robot.joints.len());
    let mut visited = vec![false; robot.joints.len()];
    let mut stack: Vec<&str> = robot
        .links
        .iter()
        .map(|link| link.name.as_str())
        .filter(|name| !child_links.contains(name))
        .collect();
    while let Some(link) = stack.pop() {
        for &i in child_joints.get(link).into_iter().flatten() {
            if !visited[i] {
                visited[i] = true;
                order.push(i);
                stack.push(robot.joints[i].child.link.as_str());
            }
        }
    }

    // Joints unreachable from a root (in an invalid URDF with a kinematic loop) keep their order.
    order.extend((0..robot.joints.len()).filter(|i| !visited[*i]));
    order
}

/// The outcome of [`squeeze_empty_fixed_links`].
struct SqueezeResult {
    /// Names of the links that must be fixed because the empty root anchoring them was removed.
    force_fixed: HashSet<String>,
    /// For each remaining link, its index in the original robot.
    link_ids: Vec<usize>,
    /// For each remaining joint, its index in the original robot.
    joint_ids: Vec<usize>,
    /// For each remaining joint, the original indices of the joints merged into it.
    merged_joint_ids: Vec<Vec<usize>>,
    /// For each remaining joint, the pose (in URDF units) of its child link in the joint frame.
    ///
    /// This isn't the identity when the joint took over a moving joint leading to an empty link.
    child_offsets: Vec<Pose>,
}

impl SqueezeResult {
    /// The result of squeezing a robot without any empty link.
    fn identity(robot: &Robot) -> Self {
        Self {
            force_fixed: HashSet::new(),
            link_ids: (0..robot.links.len()).collect(),
            joint_ids: (0..robot.joints.len()).collect(),
            merged_joint_ids: vec![vec![]; robot.joints.len()],
            child_offsets: vec![Pose::IDENTITY; robot.joints.len()],
        }
    }

    fn remove_link(&mut self, robot: &mut Robot, name: &str) {
        if let Some(id) = robot.links.iter().position(|l| l.name == name) {
            robot.links.remove(id);
            self.link_ids.remove(id);
        }
    }

    fn remove_joint(&mut self, robot: &mut Robot, id: usize) {
        robot.joints.remove(id);
        self.joint_ids.remove(id);
        self.merged_joint_ids.remove(id);
        self.child_offsets.remove(id);
    }

    /// Records that the joint `parent_joint` was merged into the joint `joint`.
    fn merge_parent_joint(&mut self, joint: usize, parent_joint: usize) {
        let parent_id = self.joint_ids[parent_joint];
        let parent_merged = self.merged_joint_ids[parent_joint].clone();
        let merged = &mut self.merged_joint_ids[joint];
        merged.push(parent_id);
        merged.extend(parent_merged);
    }
}

/// Attaches the joint `joint` of an empty link to `new_parent`, where `new_parent_to_link` is
/// the pose of the empty link in the frame of `new_parent`.
fn reparent_joint(robot: &mut Robot, joint: usize, new_parent: &str, new_parent_to_link: &Pose) {
    let joint = &mut robot.joints[joint];
    joint.parent.link = new_parent.to_string();
    joint.origin = pose_to_urdf_pose(&(new_parent_to_link * urdf_to_pose(&joint.origin)));
}

/// Among the fixed joints attaching children to an empty link, picks the one that takes over
/// the empty link's parent joint.
///
/// Non-empty children are preferred, then empty children that have children themselves.
fn pick_representative(robot: &Robot, fixed_child_joints: &[usize]) -> Option<usize> {
    let is_empty = |name: &str| {
        robot
            .links
            .iter()
            .find(|l| l.name == name)
            .is_none_or(is_link_empty)
    };
    let has_children = |name: &str| robot.joints.iter().any(|j| j.parent.link == name);
    fixed_child_joints.iter().copied().min_by_key(|&i| {
        let child = robot.joints[i].child.link.as_str();
        if !is_empty(child) {
            0
        } else if has_children(child) {
            1
        } else {
            2
        }
    })
}

/// Rewrites `robot` to remove empty links connected by fixed joints, and tracks the
/// original indices of the remaining links and joints.
///
/// See [`UrdfLoaderOptions::squeeze_empty_fixed_links`] for the precise rules.
fn squeeze_empty_fixed_links(robot: &mut Robot) -> SqueezeResult {
    let mut result = SqueezeResult::identity(robot);
    while squeeze_one_empty_link(robot, &mut result) {}
    result
}

/// Removes one empty link (or the fixed joints of an empty root) from `robot`, returning `false`
/// if no empty link can be removed.
fn squeeze_one_empty_link(robot: &mut Robot, result: &mut SqueezeResult) -> bool {
    let empty_names: Vec<String> = robot
        .links
        .iter()
        .filter(|l| is_link_empty(l))
        .map(|l| l.name.clone())
        .collect();

    for empty_name in &empty_names {
        let parent_joint = robot
            .joints
            .iter()
            .position(|j| j.child.link == *empty_name);
        let child_joints: Vec<usize> = robot
            .joints
            .iter()
            .enumerate()
            .filter_map(|(i, j)| (j.parent.link == *empty_name).then_some(i))
            .collect();
        let fixed_child_joints: Vec<usize> = child_joints
            .iter()
            .copied()
            .filter(|i| robot.joints[*i].joint_type == urdf_rs::JointType::Fixed)
            .collect();

        match parent_joint {
            Some(pj) if robot.joints[pj].joint_type == urdf_rs::JointType::Fixed => {
                // The empty link is rigidly attached to its parent: attach its children to its
                // parent directly. One fixed child records the removed joint.
                let parent = robot.joints[pj].parent.link.clone();
                let parent_to_link =
                    urdf_to_pose(&robot.joints[pj].origin) * result.child_offsets[pj];
                for &cj in &child_joints {
                    reparent_joint(robot, cj, &parent, &parent_to_link);
                }
                if let Some(rep) = pick_representative(robot, &fixed_child_joints) {
                    result.merge_parent_joint(rep, pj);
                }
                result.remove_joint(robot, pj);
                result.remove_link(robot, empty_name);
                return true;
            }
            Some(pj) if child_joints.is_empty() => {
                // Empty leaf.
                result.remove_joint(robot, pj);
                result.remove_link(robot, empty_name);
                return true;
            }
            Some(pj) => {
                // The empty link is moved by its parent joint: one of its fixed children (the
                // representative) takes over that joint, and its other children are attached to
                // the representative so they all keep moving together.
                let Some(rep) = pick_representative(robot, &fixed_child_joints) else {
                    // Only moving children: the empty link is needed between the moving joints.
                    continue;
                };
                let rep_link = robot.joints[rep].child.link.clone();
                let link_to_rep =
                    urdf_to_pose(&robot.joints[rep].origin) * result.child_offsets[rep];
                let rep_to_link = link_to_rep.inverse();
                for &cj in &child_joints {
                    if cj != rep {
                        reparent_joint(robot, cj, &rep_link, &rep_to_link);
                    }
                }

                // The joint keeps its origin and axis in the frame of the empty link, which
                // becomes the representative's offset from the joint frame.
                let rep_joint = &robot.joints[rep];
                let mut merged_joint = robot.joints[pj].clone();
                merged_joint.name = rep_joint.name.clone();
                merged_joint.child = rep_joint.child.clone();
                robot.joints[rep] = merged_joint;
                result.child_offsets[rep] = result.child_offsets[pj] * link_to_rep;
                result.merge_parent_joint(rep, pj);

                result.remove_joint(robot, pj);
                result.remove_link(robot, empty_name);
                return true;
            }
            None => {
                // Empty root: its fixed children are anchored to the world instead.
                // Remove them in reverse to keep the indices valid.
                for &cj in fixed_child_joints.iter().rev() {
                    let child = robot.joints[cj].child.link.clone();
                    result.force_fixed.insert(child);
                    result.remove_joint(robot, cj);
                }
                if fixed_child_joints.len() == child_joints.len() {
                    result.remove_link(robot, empty_name);
                    return true;
                } else if !fixed_child_joints.is_empty() {
                    return true;
                }
            }
        }
    }

    false
}
