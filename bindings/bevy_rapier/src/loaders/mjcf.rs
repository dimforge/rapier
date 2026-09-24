//! Spawning models described by MuJoCo MJCF files as `bevy_rapier` entities.
//!
//! Load a model with [`MjcfRobot::from_file`] or [`MjcfRobot::from_str`], then spawn it with
//! [`spawn_mjcf_model`]:
//!
//! ```no_run
//! # use bevy::prelude::*;
//! use bevy_rapier3d::loaders::mjcf::{spawn_mjcf_model, MjcfLoaderOptions, MjcfRobot, MjcfSpawnOptions};
//!
//! fn setup(mut commands: Commands) {
//!     let (robot, _) = MjcfRobot::from_file("model.xml", MjcfLoaderOptions::default()).unwrap();
//!     let model = spawn_mjcf_model(&mut commands, &robot, &MjcfSpawnOptions::default());
//!     println!("Spawned {} actuators.", model.actuators.len());
//! }
//! ```
//!
//! # What is translated
//!
//! - Each body becomes a rigid-body entity (child of [`SpawnedMjcfModel::root`]) with its mass
//!   properties, and `gravcomp` as a [`GravityScale`](crate::dynamics::GravityScale). The implicit
//!   world body is spawned as a fixed rigid-body when something is attached to it.
//! - Each geom becomes a collider entity child of its body, with `contype`/`conaffinity` converted
//!   to [`CollisionGroups`](crate::geometry::CollisionGroups) by `rapier3d-mjcf` (see
//!   [`ContactFilterMode`]).
//! - Joints become [`MultibodyJoint`]s (the default, matching MuJoCo's reduced coordinates) or
//!   [`ImpulseJoint`]s, on the entity of their child body. With multibody joints, `damping`,
//!   `armature`, `frictionloss`, `stiffness`/`springref`, `springdamper` and `<equality><joint>` /
//!   fixed tendon couplings are converted into [`MultibodyJointDamping`],
//!   [`MultibodyJointArmature`], [`MultibodyJointFriction`], [`MultibodyJointSprings`] and
//!   [`MultibodyJointCouplings`]. With impulse joints, they are approximated by joint motors.
//! - `<equality><connect>` and `<equality><weld>` become [`ImpulseJoint`]s on child entities of
//!   their second body (multibodies can't form loops).
//! - Each actuator becomes an entity with an [`MjcfActuator`] component, driving the motor of its
//!   joint (with [`MotorModel::ForceBased`](crate::dynamics::MotorModel::ForceBased)) once
//!   [`MjcfPlugin`] is added.
//! - `<contact><exclude>` and the friction of `<contact><pair>` are registered in the
//!   [`MjcfContactFilters`] resource, applied by the [`MjcfPhysicsHooks`] physics hooks.
//!
//! Keyframes, sensors, `solref`/`solimp`, `<contact><pair>` margins and the actuator types not
//! handled by `rapier3d-mjcf` (e.g. `intvelocity`, muscles) are not translated. Visual-only geoms
//! are exposed through the [`MjcfVisualMeshes`] component so they can be rendered.

use crate::dynamics::{
    free_joint_dofs, GenericJoint, ImpulseJoint, KinematicMultibodyJoint, MultibodyJoint,
    MultibodyJointArmature, MultibodyJointCoupling, MultibodyJointCouplings, MultibodyJointDamping,
    MultibodyJointFriction, MultibodyJointSprings, MultibodySelfContactsDisabled, JOINT_DOFS,
};
use crate::geometry::ActiveHooks;
use crate::loaders::common::{
    active_hooks, insert_collider, insert_context_link, insert_rigid_body, joint_axis,
};
use crate::math::{Real, Vect};
use crate::pipeline::{BevyPhysicsHooks, ContactModificationContextView, PairFilterContextView};
use crate::plugin::PhysicsSet;
use crate::utils::iso_to_transform;
use bevy::ecs::intern::Interned;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::ecs::system::SystemParam;
use bevy::platform::collections::{HashMap, HashSet};
use bevy::prelude::*;
use rapier::dynamics::{
    GenericJoint as RapierGenericJoint, ImpulseJointSet, MultibodyJointHandle, MultibodyJointSet,
    RigidBodyHandle, RigidBodySet,
};
use rapier::geometry::{ColliderSet, SolverFlags};
use rapier3d_mjcf::{MjcfActuatorHandle, MjcfRobotHandles};

pub use rapier3d_mjcf::{
    self, mjcf_rs, ContactFilterMode, MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRenderMaterial,
    MjcfRobot, MjcfVisualMesh,
};

/// Options controlling how an [`MjcfRobot`] is spawned by [`spawn_mjcf_model`].
#[derive(Clone, Debug)]
pub struct MjcfSpawnOptions {
    /// If `true` (default), the joints are spawned as [`MultibodyJoint`]s (reduced coordinates,
    /// like MuJoCo). Otherwise, they are spawned as [`ImpulseJoint`]s.
    pub multibody: bool,
    /// Options applied to the multibody joints, if [`Self::multibody`] is `true`.
    pub multibody_options: MjcfMultibodyOptions,
    /// The transform of the root entity of the model, which all its bodies are children of
    /// (default: identity).
    pub root_transform: Transform,
    /// The physics context the model is added to (default: `None`, the default context).
    pub context: Option<Entity>,
}

impl Default for MjcfSpawnOptions {
    fn default() -> Self {
        Self {
            multibody: true,
            multibody_options: MjcfMultibodyOptions::empty(),
            root_transform: Transform::IDENTITY,
            context: None,
        }
    }
}

/// The entities spawned by [`spawn_mjcf_model`].
///
/// The vectors are indexed like the bodies, joints and actuators of the [`MjcfRobot`].
#[derive(Clone, Debug)]
pub struct SpawnedMjcfModel {
    /// The entity every body is a child of. Despawning it despawns the whole model.
    pub root: Entity,
    /// The rigid-body entity of each body (`None` for the world body if nothing is attached to
    /// it).
    pub bodies: Vec<Option<Entity>>,
    /// The collider entities of each body (children of the body entity).
    pub colliders: Vec<Vec<Entity>>,
    /// The entity containing each joint (the entity of its child body), or `None` if the joint
    /// couldn't be created (e.g. a multibody joint closing a loop).
    pub joints: Vec<Option<Entity>>,
    /// The entity containing the impulse joint of each equality constraint, or `None` if it
    /// wasn't created (see [`MjcfMultibodyOptions::SKIP_LOOP_CLOSURES`]).
    pub equality_joints: Vec<Option<Entity>>,
    /// The entity containing the [`MjcfActuator`] of each actuator.
    pub actuators: Vec<Entity>,
    /// The body entities, by MJCF body name.
    pub bodies_by_name: HashMap<String, Entity>,
    /// The joint entities, by MJCF joint name.
    pub joints_by_name: HashMap<String, Entity>,
    /// The collider entities, by MJCF geom name.
    pub colliders_by_name: HashMap<String, Entity>,
    /// The actuator entities, by MJCF actuator name.
    pub actuators_by_name: HashMap<String, Entity>,
    /// The gravity declared by the model (`<option gravity>`), expressed in the frame of the
    /// parent of [`Self::root`].
    ///
    /// This isn't applied automatically: it can be copied into the
    /// [`RapierConfiguration`](crate::plugin::RapierConfiguration) of the physics context.
    pub gravity: Vect,
}

/// The index of the [`MjcfRobot`] body a rigid-body entity was spawned from by
/// [`spawn_mjcf_model`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, Component)]
pub struct MjcfBodyId(pub usize);

/// The visual-only geoms of the body a rigid-body entity was spawned from.
///
/// This is added by [`spawn_mjcf_model`] to the body entities with visual geoms (when
/// [`MjcfLoaderOptions::create_colliders_from_visual_shapes`] is `false`), so that rendering
/// meshes can be attached to them. The mesh poses are relative to the body entity.
#[derive(Clone, Debug, Component)]
pub struct MjcfVisualMeshes(pub Vec<MjcfVisualMesh>);

/// An MJCF actuator driving the motor of a joint.
///
/// This is spawned by [`spawn_mjcf_model`], and applied to the [`MultibodyJoint`] or
/// [`ImpulseJoint`] of [`Self::joint`] by the [`apply_mjcf_actuators`] system (added by
/// [`MjcfPlugin`]) whenever this component changes. Set [`Self::ctrl`] to control the actuator.
#[derive(Clone, Debug, Component)]
pub struct MjcfActuator {
    /// The actuator description.
    pub actuator: mjcf_rs::extras::Actuator,
    /// The entity containing the joint driven by this actuator, if it was created.
    pub joint: Option<Entity>,
    /// The control input of the actuator (MuJoCo's `ctrl`, default: `0`).
    pub ctrl: Real,
    /// A uniform scale applied to the gains and force limits of the actuator (default: `1`).
    pub gain_scale: Real,
}

impl MjcfActuator {
    /// Configures the motor of the given joint as this actuator does with its current control.
    ///
    /// The motors use the [`MotorModel::ForceBased`](crate::dynamics::MotorModel::ForceBased)
    /// model, and their target and gains depend on the actuator type, following
    /// `rapier3d-mjcf`'s `MjcfRobotHandles::apply_controls`.
    pub fn configure_joint(&self, joint: &mut RapierGenericJoint) {
        // Reuse `rapier3d-mjcf`'s actuator semantics by running them on a single-joint set.
        let mut joints = ImpulseJointSet::new();
        let handle = joints.insert(
            RigidBodyHandle::from_raw_parts(0, 0),
            RigidBodyHandle::from_raw_parts(1, 0),
            *joint,
            false,
        );
        let handles = MjcfRobotHandles {
            bodies: vec![],
            joints: vec![],
            equality_joints: vec![],
            actuators: vec![MjcfActuatorHandle {
                actuator: self.actuator.clone(),
                joint: Some(handle),
            }],
        };
        handles.apply_controls_scaled(&mut joints, &[self.ctrl], self.gain_scale);
        if let Some(configured) = joints.get(handle) {
            *joint = configured.data;
        }
    }
}

/// System applying the [`MjcfActuator`]s that changed to the joints they drive.
pub fn apply_mjcf_actuators(
    actuators: Query<&MjcfActuator, Changed<MjcfActuator>>,
    mut multibody_joints: Query<&mut MultibodyJoint>,
    mut impulse_joints: Query<&mut ImpulseJoint, Without<MultibodyJoint>>,
) {
    for actuator in actuators.iter() {
        let Some(entity) = actuator.joint else {
            continue;
        };
        if let Ok(mut joint) = multibody_joints.get_mut(entity) {
            actuator.configure_joint(&mut joint.data.as_mut().raw);
        } else if let Ok(mut joint) = impulse_joints.get_mut(entity) {
            actuator.configure_joint(&mut joint.data.as_mut().raw);
        }
    }
}

/// Plugin running the [`apply_mjcf_actuators`] system and initializing the
/// [`MjcfContactFilters`] resource.
///
/// Its schedule must match the one of the
/// [`RapierPhysicsPlugin`](crate::plugin::RapierPhysicsPlugin) (`PostUpdate` by default).
pub struct MjcfPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl MjcfPlugin {
    /// Runs the actuator system in the given schedule, before [`PhysicsSet::SyncBackend`].
    pub fn in_schedule(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for MjcfPlugin {
    fn default() -> Self {
        Self::in_schedule(PostUpdate)
    }
}

impl Plugin for MjcfPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MjcfContactFilters>().add_systems(
            self.schedule,
            apply_mjcf_actuators.before(PhysicsSet::SyncBackend),
        );
    }
}

/// The contact filtering rules of MJCF models: `<contact><exclude>` and the friction overrides of
/// `<contact><pair>`, between collider entities.
///
/// [`spawn_mjcf_model`] registers the rules of each model in this resource. They are applied by
/// the [`MjcfPhysicsHooks`] physics hooks, or by custom hooks calling
/// [`Self::filter_contact_pair`] and [`Self::modify_solver_contacts`]. The colliders involved in
/// these rules have their [`ActiveHooks`] set accordingly.
#[derive(Clone, Debug, Default, Resource)]
pub struct MjcfContactFilters {
    /// Pairs of colliders that never generate contacts (both orders are stored).
    pub excluded: HashSet<(Entity, Entity)>,
    /// Friction coefficient overrides of pairs of colliders (both orders are stored).
    pub friction: HashMap<(Entity, Entity), Real>,
}

impl MjcfContactFilters {
    /// Prevents contacts between two colliders.
    pub fn exclude(&mut self, collider1: Entity, collider2: Entity) {
        self.excluded.insert((collider1, collider2));
        self.excluded.insert((collider2, collider1));
    }

    /// Overrides the friction coefficient of the contacts between two colliders.
    pub fn set_friction(&mut self, collider1: Entity, collider2: Entity, friction: Real) {
        self.friction.insert((collider1, collider2), friction);
        self.friction.insert((collider2, collider1), friction);
    }

    /// Removes all the rules involving the given collider.
    pub fn remove(&mut self, collider: Entity) {
        self.excluded
            .retain(|(a, b)| *a != collider && *b != collider);
        self.friction
            .retain(|(a, b), _| *a != collider && *b != collider);
    }

    /// Filters a contact pair according to these rules.
    pub fn filter_contact_pair(&self, context: &PairFilterContextView) -> Option<SolverFlags> {
        let pair = (context.collider1(), context.collider2());
        (!self.excluded.contains(&pair)).then_some(SolverFlags::COMPUTE_RIGID_IMPULSES)
    }

    /// Modifies the solver contacts of a contact pair according to these rules.
    pub fn modify_solver_contacts(&self, context: &mut ContactModificationContextView) {
        let pair = (context.collider1(), context.collider2());
        if let Some(friction) = self.friction.get(&pair) {
            context.set_friction(*friction);
        }
    }
}

/// Physics hooks applying the [`MjcfContactFilters`] resource.
///
/// Use it as the hooks of the physics plugin: `RapierPhysicsPlugin::<MjcfPhysicsHooks>`.
#[derive(SystemParam)]
pub struct MjcfPhysicsHooks<'w> {
    filters: Option<Res<'w, MjcfContactFilters>>,
}

impl BevyPhysicsHooks for MjcfPhysicsHooks<'_> {
    fn filter_contact_pair(&self, context: PairFilterContextView) -> Option<SolverFlags> {
        match &self.filters {
            Some(filters) => filters.filter_contact_pair(&context),
            None => Some(SolverFlags::COMPUTE_RIGID_IMPULSES),
        }
    }

    fn modify_solver_contacts(&self, mut context: ContactModificationContextView) {
        if let Some(filters) = &self.filters {
            filters.modify_solver_contacts(&mut context);
        }
    }
}

/// The per-joint multibody properties computed by `rapier3d-mjcf`, read from a scratch insertion
/// of the model into Rapier.
struct MultibodyJointProperties {
    data: RapierGenericJoint,
    kinematic: bool,
    damping: MultibodyJointDamping,
    armature: MultibodyJointArmature,
    friction: MultibodyJointFriction,
    springs: MultibodyJointSprings,
}

/// The result of inserting the model with `rapier3d-mjcf`'s multibody path, which resolves the
/// joint properties depending on the assembled multibody (e.g. `springdamper`).
struct MultibodyScratch {
    handles: MjcfRobotHandles<Option<MultibodyJointHandle>>,
    joints: MultibodyJointSet,
}

impl MultibodyScratch {
    fn new(robot: &MjcfRobot, options: MjcfMultibodyOptions) -> Self {
        let mut joints = MultibodyJointSet::new();
        let handles = robot.clone().insert_using_multibody_joints(
            &mut RigidBodySet::new(),
            &mut ColliderSet::new(),
            &mut joints,
            &mut ImpulseJointSet::new(),
            options,
        );
        Self { handles, joints }
    }

    fn joint_properties(&self, handle: MultibodyJointHandle) -> Option<MultibodyJointProperties> {
        let (mb, link_id) = self.joints.get(handle)?;
        let link = mb.link(link_id)?;
        let joint = link.joint();
        let offset = link.assembly_id();
        let data = joint.data;
        let mut result = MultibodyJointProperties {
            data,
            kinematic: joint.kinematic,
            damping: default(),
            armature: default(),
            friction: default(),
            springs: default(),
        };
        for (dof, axis) in free_joint_dofs(joint.data.locked_axes) {
            result.damping.0[axis] = mb.damping()[offset + dof];
            result.armature.0[axis] = mb.armature()[offset + dof];
            result.friction.0[axis] = mb.frictions()[offset + dof];
        }
        for axis in 0..JOINT_DOFS {
            let (stiffness, rest) = joint.spring(axis);
            result.springs.stiffness[axis] = stiffness;
            result.springs.rest[axis] = rest;
        }
        Some(result)
    }

    /// The index of the body of `handle` in the model.
    fn body_index(&self, handle: RigidBodyHandle) -> Option<usize> {
        self.handles
            .bodies
            .iter()
            .position(|b| b.as_ref().is_some_and(|b| b.body == handle))
    }
}

/// Spawns the bodies, colliders, joints and actuators of an MJCF model.
///
/// See the [module documentation](self) for details.
pub fn spawn_mjcf_model(
    commands: &mut Commands,
    robot: &MjcfRobot,
    options: &MjcfSpawnOptions,
) -> SpawnedMjcfModel {
    let multibody_options = options.multibody_options;
    let skip_loop_closures =
        options.multibody && multibody_options.contains(MjcfMultibodyOptions::SKIP_LOOP_CLOSURES);
    let scratch = options
        .multibody
        .then(|| MultibodyScratch::new(robot, multibody_options));

    let name = robot
        .name
        .clone()
        .unwrap_or_else(|| "mjcf model".to_string());
    let mut root = commands.spawn((options.root_transform, Name::new(name)));
    insert_context_link(&mut root, options.context);
    let root = root.id();

    // The world body is only needed when something is attached to it.
    let world_needed = match &scratch {
        Some(scratch) => scratch.handles.bodies.first().is_some_and(|b| b.is_some()),
        None => {
            robot.joints.iter().any(|j| j.link1 == 0 || j.link2 == 0)
                || robot
                    .equality_joints
                    .iter()
                    .any(|j| j.link1 == 0 || j.link2 == 0)
                || robot
                    .bodies
                    .first()
                    .is_some_and(|b| !b.colliders.is_empty())
        }
    };

    // Collect the colliders involved in contact rules, which are the only ones needing hooks.
    let body_colliders = |body_name: &str| {
        robot
            .body_name_to_idx
            .get(body_name)
            .map(|b| (0..robot.bodies[*b].colliders.len()).map(move |k| (*b, k)))
            .into_iter()
            .flatten()
    };
    let mut excluded = vec![];
    for exclude in &robot.contact_excludes {
        for c1 in body_colliders(&exclude.body1) {
            for c2 in body_colliders(&exclude.body2) {
                excluded.push((c1, c2));
            }
        }
    }
    let mut friction_overrides = vec![];
    for pair in &robot.contact_pairs {
        let c1 = robot.geom_name_to_collider.get(&pair.geom1);
        let c2 = robot.geom_name_to_collider.get(&pair.geom2);
        if let (Some(c1), Some(c2), Some(friction)) = (c1, c2, pair.friction) {
            friction_overrides.push((*c1, *c2, friction[0] as Real));
        }
    }
    let with_hooks: HashSet<(usize, usize)> = excluded
        .iter()
        .flat_map(|(c1, c2)| [*c1, *c2])
        .chain(friction_overrides.iter().flat_map(|(c1, c2, _)| [*c1, *c2]))
        .collect();
    let geom_names: HashMap<(usize, usize), &String> = robot
        .geom_name_to_collider
        .iter()
        .map(|(name, id)| (*id, name))
        .collect();

    let mut result = SpawnedMjcfModel {
        root,
        bodies: vec![],
        colliders: vec![],
        joints: vec![],
        equality_joints: vec![],
        actuators: vec![],
        bodies_by_name: default(),
        joints_by_name: default(),
        colliders_by_name: default(),
        actuators_by_name: default(),
        gravity: options.root_transform.rotation * (robot.base_shift.rotation * robot.gravity),
    };

    for (i, body) in robot.bodies.iter().enumerate() {
        if i == 0 && !world_needed {
            result.bodies.push(None);
            result.colliders.push(vec![]);
            continue;
        }

        let name = body.name.clone().unwrap_or_else(|| {
            if i == 0 {
                "world".into()
            } else {
                format!("body {i}")
            }
        });
        let mut entity = commands.spawn((
            ChildOf(root),
            iso_to_transform(body.body.position()),
            Name::new(name.clone()),
            MjcfBodyId(i),
        ));
        insert_rigid_body(&mut entity, &body.body);
        insert_context_link(&mut entity, options.context);
        if !body.visual_meshes.is_empty() {
            entity.insert(MjcfVisualMeshes(body.visual_meshes.clone()));
        }
        let body_entity = entity.id();

        let colliders: Vec<_> = body
            .colliders
            .iter()
            .enumerate()
            .map(|(k, co)| {
                let geom_name = geom_names.get(&(i, k));
                let name = geom_name.map_or_else(|| format!("{name} geom {k}"), |n| n.to_string());
                let hooks = if with_hooks.contains(&(i, k)) {
                    active_hooks(co)
                } else {
                    active_hooks(co)
                        - (ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS)
                };
                let mut entity = commands.spawn((ChildOf(body_entity), Name::new(name)));
                insert_collider(&mut entity, co, hooks);
                insert_context_link(&mut entity, options.context);
                if let Some(geom_name) = geom_name {
                    result
                        .colliders_by_name
                        .insert(geom_name.to_string(), entity.id());
                }
                entity.id()
            })
            .collect();

        if let Some(name) = &body.name {
            result.bodies_by_name.insert(name.clone(), body_entity);
        }
        result.bodies.push(Some(body_entity));
        result.colliders.push(colliders);
    }

    // Joints are added to the entity of their child body, or to a new child entity of that body
    // if it already contains a joint.
    let mut has_joint = vec![false; robot.bodies.len()];
    let mut joint_entity = |commands: &mut Commands, link2: usize, body2: Entity| {
        if std::mem::replace(&mut has_joint[link2], true) {
            let mut entity = commands.spawn(ChildOf(body2));
            insert_context_link(&mut entity, options.context);
            entity.id()
        } else {
            body2
        }
    };

    for (i, joint) in robot.joints.iter().enumerate() {
        let bodies = (
            result.bodies.get(joint.link1).copied().flatten(),
            result.bodies.get(joint.link2).copied().flatten(),
        );
        let (Some(body1), Some(body2)) = bodies else {
            result.joints.push(None);
            continue;
        };

        let entity = if let Some(scratch) = &scratch {
            let Some(props) = scratch
                .handles
                .joints
                .get(i)
                .and_then(|h| h.joint)
                .and_then(|h| scratch.joint_properties(h))
            else {
                result.joints.push(None);
                continue;
            };
            let entity = joint_entity(commands, joint.link2, body2);
            let mut entity_commands = commands.entity(entity);
            entity_commands.insert(MultibodyJoint::new(body1, GenericJoint { raw: props.data }));
            if props.kinematic {
                entity_commands.insert(KinematicMultibodyJoint);
            }
            if props.damping != MultibodyJointDamping::default() {
                entity_commands.insert(props.damping);
            }
            if props.armature != MultibodyJointArmature::default() {
                entity_commands.insert(props.armature);
            }
            if props.friction != MultibodyJointFriction::default() {
                entity_commands.insert(props.friction);
            }
            if props.springs != MultibodyJointSprings::default() {
                entity_commands.insert(props.springs);
            }
            entity
        } else {
            let entity = joint_entity(commands, joint.link2, body2);
            commands
                .entity(entity)
                .insert(ImpulseJoint::new(body1, GenericJoint { raw: joint.joint }));
            entity
        };

        if let Some(name) = &joint.name {
            result.joints_by_name.insert(name.clone(), entity);
        }
        result.joints.push(Some(entity));
    }

    if let Some(scratch) = &scratch {
        // DoF couplings, stored on the joint of their second link.
        let mut couplings: HashMap<Entity, Vec<MultibodyJointCoupling>> = default();
        let link_entity = |mb: &rapier::dynamics::Multibody, link: usize| {
            let handle = mb.link(link)?.rigid_body_handle();
            result
                .bodies
                .get(scratch.body_index(handle)?)
                .copied()
                .flatten()
        };
        for mb in scratch.joints.multibodies() {
            for c in mb.couplings() {
                let (Some(source), Some(target)) =
                    (link_entity(mb, c.link1), link_entity(mb, c.link2))
                else {
                    continue;
                };
                couplings
                    .entry(target)
                    .or_default()
                    .push(MultibodyJointCoupling::new(
                        joint_axis(c.axis2),
                        source,
                        joint_axis(c.axis1),
                        c.coeff,
                        c.offset,
                    ));
            }
            if !mb.self_contacts_enabled() {
                if let Some(root) = link_entity(mb, 0) {
                    commands.entity(root).insert(MultibodySelfContactsDisabled);
                }
            }
        }
        for (entity, couplings) in couplings {
            commands
                .entity(entity)
                .insert(MultibodyJointCouplings(couplings));
        }
    }

    for eq in &robot.equality_joints {
        let bodies = (
            result.bodies.get(eq.link1).copied().flatten(),
            result.bodies.get(eq.link2).copied().flatten(),
        );
        let (Some(body1), Some(body2), false) = (bodies.0, bodies.1, skip_loop_closures) else {
            result.equality_joints.push(None);
            continue;
        };
        let mut data = eq.joint;
        data.set_enabled(eq.active);
        let name = eq.name.clone().unwrap_or_else(|| "equality".into());
        let mut entity = commands.spawn((
            ChildOf(body2),
            Name::new(name),
            ImpulseJoint::new(body1, GenericJoint { raw: data }),
        ));
        insert_context_link(&mut entity, options.context);
        result.equality_joints.push(Some(entity.id()));
    }

    for (i, binding) in robot.actuators.iter().enumerate() {
        let name = binding
            .actuator
            .name
            .clone()
            .unwrap_or_else(|| format!("actuator {i}"));
        let joint = binding
            .joint_index
            .and_then(|j| result.joints.get(j).copied().flatten());
        let entity = commands
            .spawn((
                ChildOf(root),
                Name::new(name.clone()),
                MjcfActuator {
                    actuator: binding.actuator.clone(),
                    joint,
                    ctrl: 0.0,
                    gain_scale: 1.0,
                },
            ))
            .id();
        if binding.actuator.name.is_some() {
            result.actuators_by_name.insert(name, entity);
        }
        result.actuators.push(entity);
    }

    // Register the contact rules between the spawned collider entities.
    let collider = |(b, k): (usize, usize)| result.colliders.get(b)?.get(k).copied();
    let excluded: Vec<_> = excluded
        .into_iter()
        .filter_map(|(c1, c2)| Some((collider(c1)?, collider(c2)?)))
        .collect();
    let friction_overrides: Vec<_> = friction_overrides
        .into_iter()
        .filter_map(|(c1, c2, f)| Some((collider(c1)?, collider(c2)?, f)))
        .collect();
    if !excluded.is_empty() || !friction_overrides.is_empty() {
        commands.queue(move |world: &mut World| {
            let mut filters = world.get_resource_or_init::<MjcfContactFilters>();
            for (c1, c2) in excluded {
                filters.exclude(c1, c2);
            }
            for (c1, c2, friction) in friction_overrides {
                filters.set_friction(c1, c2, friction);
            }
        });
    }

    result
}
