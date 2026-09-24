//! Spawning robots described by URDF files as `bevy_rapier` entities.
//!
//! Load a robot with [`UrdfModel::from_file`] or [`UrdfModel::from_str`], then spawn it with
//! [`spawn_urdf_robot`]:
//!
//! ```no_run
//! # use bevy::prelude::*;
//! use bevy_rapier3d::loaders::urdf::{spawn_urdf_robot, UrdfLoaderOptions, UrdfModel, UrdfSpawnOptions};
//!
//! fn setup(mut commands: Commands) {
//!     let model = UrdfModel::from_file("robot.urdf", UrdfLoaderOptions::default(), None).unwrap();
//!     let robot = spawn_urdf_robot(&mut commands, &model, &UrdfSpawnOptions::default());
//!     println!("Spawned {} links.", robot.links.len());
//! }
//! ```
//!
//! Each URDF link becomes a rigid-body entity (a child of [`SpawnedUrdfRobot::root`]), each of its
//! shapes a collider entity child of the link, and each URDF joint an [`ImpulseJoint`] or a
//! [`MultibodyJoint`] on the entity of its child link.
//!
//! On top of what `rapier3d-urdf` converts, the joint `<dynamics>` (damping and friction) and
//! `<mimic>` elements are applied to multibody joints (as [`MultibodyJointDamping`],
//! [`MultibodyJointFriction`] and [`MultibodyJointCouplings`]). They are ignored by impulse
//! joints, as are the joint effort and velocity limits and the safety controllers.

use crate::dynamics::{
    GenericJoint, ImpulseJoint, KinematicMultibodyJoint, MultibodyJoint, MultibodyJointCoupling,
    MultibodyJointCouplings, MultibodyJointDamping, MultibodyJointFriction,
    MultibodySelfContactsDisabled,
};
use crate::loaders::common::{
    active_hooks, insert_collider, insert_context_link, insert_rigid_body, joint_axis, LoaderError,
};
use crate::math::Real;
use crate::utils::iso_to_transform;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use rapier::dynamics::{JointAxesMask, JointAxis};
use rapier::geometry::SharedShape;
use rapier::math::SPATIAL_DIM;
use std::path::Path;

pub use rapier3d_urdf::{
    self, urdf_rs, UrdfCollider, UrdfJoint, UrdfLink, UrdfLoaderOptions, UrdfMultibodyOptions,
    UrdfRobot, UrdfVisual,
};

/// A robot loaded from an URDF file, ready to be spawned with [`spawn_urdf_robot`].
///
/// This pairs the Rapier objects built by `rapier3d-urdf` with the original URDF description.
/// Their links and joints can differ since empty links are removed when
/// [`UrdfLoaderOptions::squeeze_empty_fixed_links`] is enabled: see [`UrdfLink::urdf_link_index`]
/// and [`UrdfJoint::urdf_joint_index`] to match them.
#[derive(Clone, Debug)]
pub struct UrdfModel {
    /// The rigid-bodies, colliders and joints built from the URDF file.
    pub robot: UrdfRobot,
    /// The original URDF description.
    pub urdf: urdf_rs::Robot,
}

impl UrdfModel {
    /// Loads a robot from an URDF file.
    ///
    /// Meshes referenced by the file are looked for in `mesh_dir`, or in the directory
    /// containing the URDF file if `mesh_dir` is `None`.
    pub fn from_file(
        path: impl AsRef<Path>,
        options: UrdfLoaderOptions,
        mesh_dir: Option<&Path>,
    ) -> Result<Self, LoaderError> {
        let path = path.as_ref().canonicalize()?;
        let mesh_dir = mesh_dir
            .or_else(|| path.parent())
            .unwrap_or_else(|| Path::new("./"));
        let (robot, urdf) = UrdfRobot::from_file(&path, options, Some(mesh_dir))?;
        Ok(Self { robot, urdf })
    }

    /// Loads a robot from the content of an URDF file.
    ///
    /// Meshes referenced by the file are looked for in `mesh_dir`.
    pub fn from_str(
        urdf: &str,
        options: UrdfLoaderOptions,
        mesh_dir: &Path,
    ) -> Result<Self, LoaderError> {
        let (robot, urdf) = UrdfRobot::from_str(urdf, options, mesh_dir)?;
        Ok(Self { robot, urdf })
    }

    /// Builds a robot from an already parsed URDF description.
    ///
    /// Meshes referenced by the description are looked for in `mesh_dir`.
    pub fn from_urdf(urdf: urdf_rs::Robot, options: UrdfLoaderOptions, mesh_dir: &Path) -> Self {
        let robot = UrdfRobot::from_robot(&urdf, options, mesh_dir);
        Self { robot, urdf }
    }

    /// The URDF description of the `i`-th link of [`Self::robot`].
    pub fn urdf_link(&self, i: usize) -> Option<&urdf_rs::Link> {
        self.urdf
            .links
            .get(self.robot.links.get(i)?.urdf_link_index)
    }

    /// The URDF description of the `i`-th joint of [`Self::robot`], i.e., the URDF joint attached
    /// to its child link (see [`UrdfJoint::urdf_joint_index`]).
    pub fn urdf_joint(&self, i: usize) -> Option<&urdf_rs::Joint> {
        self.urdf
            .joints
            .get(self.robot.joints.get(i)?.urdf_joint_index)
    }

    /// The URDF joint the `i`-th joint of [`Self::robot`] inherited its type, dynamics and mimic
    /// from (see [`UrdfJoint::source_urdf_joint_index`]).
    ///
    /// This differs from [`Self::urdf_joint`] when joints leading to removed empty links were
    /// merged into this one.
    pub fn source_urdf_joint(&self, i: usize) -> Option<&urdf_rs::Joint> {
        let index = self.robot.joints.get(i)?.source_urdf_joint_index();
        self.urdf.joints.get(index)
    }
}

/// The first axis left free by `locked_axes`, if any.
fn first_free_axis(locked_axes: JointAxesMask) -> Option<JointAxis> {
    (0..SPATIAL_DIM)
        .find(|axis| locked_axes.bits() & (1 << axis) == 0)
        .map(joint_axis)
}

/// Options controlling how an [`UrdfModel`] is spawned by [`spawn_urdf_robot`].
#[derive(Clone, Debug)]
pub struct UrdfSpawnOptions {
    /// If `true`, the joints are spawned as [`MultibodyJoint`]s (reduced coordinates). Otherwise,
    /// they are spawned as [`ImpulseJoint`]s (default: `false`).
    pub multibody: bool,
    /// Options applied to the multibody joints, if [`Self::multibody`] is `true`.
    pub multibody_options: UrdfMultibodyOptions,
    /// The transform of the root entity of the robot, which all its links are children of
    /// (default: identity).
    pub root_transform: Transform,
    /// The physics context the robot is added to (default: `None`, the default context).
    pub context: Option<Entity>,
}

impl Default for UrdfSpawnOptions {
    fn default() -> Self {
        Self {
            multibody: false,
            multibody_options: UrdfMultibodyOptions::empty(),
            root_transform: Transform::IDENTITY,
            context: None,
        }
    }
}

/// The entities spawned by [`spawn_urdf_robot`].
///
/// The vectors are indexed like the links and joints of [`UrdfModel::robot`].
#[derive(Clone, Debug, Default)]
pub struct SpawnedUrdfRobot {
    /// The entity every link is a child of. Despawning it despawns the whole robot.
    pub root: Option<Entity>,
    /// The rigid-body entity of each link.
    pub links: Vec<Entity>,
    /// The collider entities of each link (children of the link entity).
    pub colliders: Vec<Vec<Entity>>,
    /// The entity containing each joint (the entity of its child link).
    pub joints: Vec<Entity>,
    /// The link entities, by URDF link name.
    pub links_by_name: HashMap<String, Entity>,
    /// The joint entities, by URDF joint name.
    ///
    /// This includes the URDF joints merged into another joint when empty links were removed
    /// (see [`UrdfJoint::merged_urdf_joint_indices`]).
    pub joints_by_name: HashMap<String, Entity>,
}

/// The index of the URDF link a rigid-body entity was spawned from by [`spawn_urdf_robot`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, Component)]
pub struct UrdfLinkId(pub usize);

/// The visual elements of the URDF link a rigid-body entity was spawned from.
///
/// This is added by [`spawn_urdf_robot`] to the link entities with visual elements, so that
/// rendering meshes can be attached to them. The visual poses are relative to the link entity.
#[derive(Clone, Debug, Component)]
pub struct UrdfLinkVisuals(pub Vec<urdf_rs::Visual>);

/// The original mesh of a collider which was approximated by a simpler shape.
///
/// This is added by [`spawn_urdf_robot`] to the collider entities built from a mesh when a
/// [`UrdfLoaderOptions::mesh_converter`] other than a triangle mesh is used.
#[derive(Clone, Debug, Component)]
pub struct UrdfColliderVisual {
    /// The triangle mesh of the original shape.
    pub shape: SharedShape,
    /// The pose of the triangle mesh relative to the collider entity.
    pub transform: Transform,
}

/// Spawns the links, colliders and joints of an URDF robot.
///
/// See the [module documentation](self) for details.
pub fn spawn_urdf_robot(
    commands: &mut Commands,
    model: &UrdfModel,
    options: &UrdfSpawnOptions,
) -> SpawnedUrdfRobot {
    let robot_name = model.urdf.name.clone();
    let mut root = commands.spawn((options.root_transform, Name::new(robot_name)));
    insert_context_link(&mut root, options.context);
    let root = root.id();

    let mut result = SpawnedUrdfRobot {
        root: Some(root),
        ..default()
    };

    for (i, link) in model.robot.links.iter().enumerate() {
        let urdf_link = model.urdf_link(i);
        let name = urdf_link.map(|l| l.name.clone()).unwrap_or_default();
        let mut entity = commands.spawn((
            ChildOf(root),
            iso_to_transform(link.body.position()),
            Name::new(name.clone()),
            UrdfLinkId(link.urdf_link_index),
        ));
        insert_rigid_body(&mut entity, &link.body);
        insert_context_link(&mut entity, options.context);
        if let Some(visuals) = urdf_link.filter(|l| !l.visual.is_empty()) {
            entity.insert(UrdfLinkVisuals(visuals.visual.clone()));
        }
        let link_entity = entity.id();

        let colliders = link
            .colliders
            .iter()
            .enumerate()
            .map(|(k, co)| {
                let mut entity = commands.spawn((
                    ChildOf(link_entity),
                    Name::new(format!("{name} collider {k}")),
                ));
                insert_collider(&mut entity, &co.collider, active_hooks(&co.collider));
                insert_context_link(&mut entity, options.context);
                if let Some(visual) = &co.visual {
                    entity.insert(UrdfColliderVisual {
                        shape: visual.shape.clone(),
                        transform: iso_to_transform(&visual.local_pose),
                    });
                }
                entity.id()
            })
            .collect();

        result.links.push(link_entity);
        result.colliders.push(colliders);
        result.links_by_name.insert(name, link_entity);
    }

    let multibody_options = options.multibody_options;
    for (i, joint) in model.robot.joints.iter().enumerate() {
        let parent = result.links[joint.link1];
        let entity = result.links[joint.link2];
        let data = GenericJoint { raw: joint.joint };
        let mut entity_commands = commands.entity(entity);
        let source = model.source_urdf_joint(i);

        if options.multibody {
            entity_commands.insert(MultibodyJoint::new(parent, data));
            if multibody_options.contains(UrdfMultibodyOptions::JOINTS_ARE_KINEMATIC) {
                entity_commands.insert(KinematicMultibodyJoint);
            }
            if multibody_options.contains(UrdfMultibodyOptions::DISABLE_SELF_CONTACTS) {
                entity_commands.insert(MultibodySelfContactsDisabled);
            }
            if let (Some(dynamics), Some(axis)) = (
                source.and_then(|j| j.dynamics.as_ref()),
                first_free_axis(joint.joint.locked_axes),
            ) {
                entity_commands.insert((
                    MultibodyJointDamping::default().with(axis, dynamics.damping as Real),
                    MultibodyJointFriction::default().with(axis, dynamics.friction as Real),
                ));
            }
        } else {
            entity_commands.insert(ImpulseJoint::new(parent, data));
        }

        result.joints.push(entity);
        let merged = joint.merged_urdf_joint_indices.iter();
        for k in std::iter::once(&joint.urdf_joint_index).chain(merged) {
            if let Some(urdf_joint) = model.urdf.joints.get(*k) {
                result
                    .joints_by_name
                    .insert(urdf_joint.name.clone(), entity);
            }
        }
    }

    // Mimic joints are coupled to the joint they follow once all the joint entities are known.
    if options.multibody {
        for (i, joint) in model.robot.joints.iter().enumerate() {
            let Some(mimic) = model.source_urdf_joint(i).and_then(|j| j.mimic.as_ref()) else {
                continue;
            };
            let Some(source) = result.joints_by_name.get(&mimic.joint).copied() else {
                log::warn!("URDF mimic joint `{}` not found.", mimic.joint);
                continue;
            };
            let source_axis = result
                .joints
                .iter()
                .position(|e| *e == source)
                .and_then(|k| first_free_axis(model.robot.joints[k].joint.locked_axes));
            let (Some(axis), Some(source_axis)) =
                (first_free_axis(joint.joint.locked_axes), source_axis)
            else {
                continue;
            };
            commands
                .entity(result.joints[i])
                .insert(MultibodyJointCouplings(vec![MultibodyJointCoupling::new(
                    axis,
                    source,
                    source_axis,
                    mimic.multiplier.unwrap_or(1.0) as Real,
                    mimic.offset.unwrap_or(0.0) as Real,
                )]));
        }
    }

    result
}

/// Converts a pose read from an URDF file into a Bevy transform.
///
/// This can be used to place the visual elements of [`UrdfLinkVisuals`] relative to their link.
/// Note that the [`UrdfLoaderOptions::scale`] isn't applied to these poses.
pub fn urdf_pose_to_transform(pose: &urdf_rs::Pose) -> Transform {
    Transform {
        translation: Vec3::new(pose.xyz[0] as f32, pose.xyz[1] as f32, pose.xyz[2] as f32),
        rotation: Quat::from_euler(
            EulerRot::XYZ,
            pose.rpy[0] as f32,
            pose.rpy[1] as f32,
            pose.rpy[2] as f32,
        ),
        ..default()
    }
}
