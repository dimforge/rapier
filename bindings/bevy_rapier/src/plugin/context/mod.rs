//! These are components used and modified during a simulation frame.

pub mod systemparams;

use bevy::prelude::*;
use rapier::geometry::{ContactData, ContactManifoldData};
use rapier::parry::query::{PersistentQueryDispatcher, QueryDispatcher};
use std::collections::HashMap;
use std::sync::RwLock;

use rapier::prelude::{
    Aabb, CCDSolver, ColliderHandle, ColliderSet, CollisionPipeline, EventHandler, FeatureId,
    ImpulseJointHandle, ImpulseJointSet, IntegrationParameters, IslandManager,
    MultibodyJointHandle, MultibodyJointSet, NarrowPhase, PhysicsHooks, PhysicsPipeline,
    QueryFilter as RapierQueryFilter, QueryPipeline, QueryPipelineMut, Ray, Real, RigidBodyHandle,
    RigidBodySet, RigidBodyType, SoftBodyHandle, SoftBodySet,
};

use crate::geometry::{
    AsShape, NonlinearMotion, PointProjection, RayIntersection, ShapeCastHit, ShapeClosestPoints,
    ShapeContact,
};
use crate::math::{Rot, Vect};
use crate::pipeline::{CollisionEvent, ContactForceEvent, EventHandlerFanOut, EventQueue};
use bevy::prelude::{Entity, GlobalTransform, Query};
use rapier::geometry::Collider as RapierCollider;
use rapier::parry::partitioning::Bvh;
use rapier::parry::query::ClosestPoints;
use rapier::parry::shape::CompositeShapeRef;

use crate::control::{CharacterCollision, MoveShapeOptions, MoveShapeOutput};
use crate::dynamics::{
    ImpulseJointImpulses, InverseKinematicsOption, MultibodyJointState, RapierSoftBody,
    RapierSoftBodyTearEvent, TransformInterpolation,
};
use crate::parry::query::details::ShapeCastOptions;
use crate::pipeline::{PhysicsQuarantineEvent, SoftBodyTearResult};
use crate::plugin::configuration::{BroadPhaseOptimizationStrategy, SimulationMode, TimestepMode};
use crate::prelude::{CollisionGroups, QueryFilter, RapierRigidBodyHandle};
use rapier::control::CharacterAutostep;
use rapier::dynamics::ImpulseJoint as RapierImpulseJoint;
use rapier::dynamics::{Multibody, MultibodyJoint as RapierMultibodyJoint};
use rapier::geometry::DefaultBroadPhase;
use rapier::math::{DVector, Jacobian};
use std::ops::Range;

#[cfg(doc)]
use crate::prelude::{
    systemparams::{RapierContext, ReadRapierContext},
    ImpulseJoint, KinematicMultibodyJoint, MultibodyJoint, RevoluteJoint, TypedJoint,
};

/// Difference between simulation and rendering time
#[derive(Component, Default, Reflect, Clone)]
pub struct SimulationToRenderTime {
    /// Difference between simulation and rendering time
    pub diff: f32,
}

/// Marker component for to access the default [`ReadRapierContext`].
///
/// This is used as the default marker filter for [`systemparams::ReadRapierContext`] and [`systemparams::WriteRapierContext`]
/// to help with getting a reference to the correct RapierContext.
///
/// If you're making a library, you might be interested in [`RapierContextEntityLink`]
/// and leverage a [`Query`] to have precise access to relevant components (for example [`RapierContextSimulation`]).
///
/// See the list of full components in [`RapierContext`]
#[derive(Component, Reflect, Debug, Clone, Copy)]
pub struct DefaultRapierContext;

/// This is a component applied to any entity containing a rapier handle component.
/// The inner Entity referred to has the component [`RapierContextSimulation`]
/// and others from [`crate::plugin::context`], responsible for handling
/// its rapier data.
#[derive(Component, Reflect, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RapierContextEntityLink(pub Entity);

/// The set of colliders part of the simulation.
///
/// This should be attached on an entity with a [`RapierContextSimulation`]
///
/// With the `serde-serialize` feature, the entity-to-handle map isn't serialized: it is rebuilt
/// from the colliders’ user-data on deserialization (see [`Self::rebuild_entity_maps`]).
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "serde-serialize",
    serde(from = "serde_shadows::RapierContextCollidersData")
)]
#[derive(Component, Default, Debug, Clone)]
pub struct RapierContextColliders {
    /// The set of colliders part of the simulation.
    pub colliders: ColliderSet,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2collider: HashMap<Entity, ColliderHandle>,
}

impl RapierContextColliders {
    /// Rebuilds the map from entities to collider handles from the user-data of the colliders,
    /// which contain the bits of their entity.
    ///
    /// This is called automatically when deserializing this component. Call it manually after
    /// replacing [`Self::colliders`] with a set restored in another way.
    pub fn rebuild_entity_maps(&mut self) {
        self.entity2collider = self
            .colliders
            .iter()
            .filter_map(|(handle, co)| Some((Entity::try_from_bits(co.user_data as u64)?, handle)))
            .collect();
    }

    /// If the collider attached to `entity` is attached to a rigid-body, this
    /// returns the `Entity` containing that rigid-body.
    pub fn collider_parent(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
    ) -> Option<Entity> {
        self.entity2collider
            .get(&entity)
            .and_then(|h| self.colliders.get(*h))
            .and_then(|co| co.parent())
            .and_then(|h| rigidbody_set.rigid_body_entity(h))
    }

    /// If entity is a rigid-body, this returns the collider `Entity`s attached
    /// to that rigid-body.
    pub fn rigid_body_colliders<'a, 'b: 'a>(
        &'a self,
        entity: Entity,
        rigidbody_set: &'b RapierRigidBodySet,
    ) -> impl Iterator<Item = Entity> + 'a {
        rigidbody_set
            .entity2body()
            .get(&entity)
            .and_then(|handle| rigidbody_set.bodies.get(*handle))
            .map(|body| {
                body.colliders()
                    .iter()
                    .filter_map(|handle| self.collider_entity(*handle))
            })
            .into_iter()
            .flatten()
    }

    /// Retrieve the Bevy entity the given Rapier collider (identified by its handle) is attached to.
    pub fn collider_entity(&self, handle: ColliderHandle) -> Option<Entity> {
        RapierContextColliders::collider_entity_with_set(&self.colliders, handle)
    }

    // Mostly used to avoid borrowing self completely.
    pub(crate) fn collider_entity_with_set(
        colliders: &ColliderSet,
        handle: ColliderHandle,
    ) -> Option<Entity> {
        colliders.get(handle).map(Self::entity_from_collider)
    }

    /// Retrieve the Bevy entity the given Rapier collider is attached to.
    pub fn entity_from_collider(collider: &rapier::prelude::Collider) -> Entity {
        Entity::from_bits(collider.user_data as u64)
    }

    /// The map from entities to collider handles.
    pub fn entity2collider(&self) -> &HashMap<Entity, ColliderHandle> {
        &self.entity2collider
    }

    /// The Rapier collider attached to the given `entity`, if any.
    fn collider(&self, entity: Entity) -> Option<&rapier::geometry::Collider> {
        self.colliders.get(*self.entity2collider.get(&entity)?)
    }

    /// Computes the world-space axis-aligned bounding box of the collider of `entity`.
    ///
    /// This doesn't include the contact skin. Returns `None` if `entity` has no collider.
    #[cfg(feature = "dim2")]
    pub fn compute_aabb(&self, entity: Entity) -> Option<bevy::shape::Aabb2d> {
        Some(aabb_to_bevy(self.collider(entity)?.compute_aabb()))
    }

    /// Computes the world-space axis-aligned bounding box of the collider of `entity`.
    ///
    /// This doesn't include the contact skin. Returns `None` if `entity` has no collider.
    #[cfg(feature = "dim3")]
    pub fn compute_aabb(&self, entity: Entity) -> Option<bevy::shape::Aabb3d> {
        Some(aabb_to_bevy(self.collider(entity)?.compute_aabb()))
    }

    /// Computes the world-space axis-aligned bounding box swept by the collider of `entity` when
    /// moving from its current position to `next_position`.
    ///
    /// The scale of `next_position` is ignored. Returns `None` if `entity` has no collider.
    #[cfg(feature = "dim2")]
    pub fn compute_swept_aabb(
        &self,
        entity: Entity,
        next_position: Transform,
    ) -> Option<bevy::shape::Aabb2d> {
        let next_position = crate::utils::transform_to_iso(&next_position);
        Some(aabb_to_bevy(
            self.collider(entity)?.compute_swept_aabb(&next_position),
        ))
    }

    /// Computes the world-space axis-aligned bounding box swept by the collider of `entity` when
    /// moving from its current position to `next_position`.
    ///
    /// The scale of `next_position` is ignored. Returns `None` if `entity` has no collider.
    #[cfg(feature = "dim3")]
    pub fn compute_swept_aabb(
        &self,
        entity: Entity,
        next_position: Transform,
    ) -> Option<bevy::shape::Aabb3d> {
        let next_position = crate::utils::transform_to_iso(&next_position);
        Some(aabb_to_bevy(
            self.collider(entity)?.compute_swept_aabb(&next_position),
        ))
    }

    /// The volume (in 3D) or area (in 2D) of the collider of `entity`.
    ///
    /// Returns `None` if `entity` has no collider.
    pub fn volume(&self, entity: Entity) -> Option<Real> {
        Some(self.collider(entity)?.volume())
    }

    /// The mass of the collider of `entity`.
    ///
    /// Returns `None` if `entity` has no collider.
    pub fn mass(&self, entity: Entity) -> Option<Real> {
        Some(self.collider(entity)?.mass())
    }

    /// The density of the collider of `entity`.
    ///
    /// If the collider's mass was set explicitly, this is the mass divided by the volume. Returns
    /// `None` if `entity` has no collider.
    pub fn density(&self, entity: Entity) -> Option<Real> {
        Some(self.collider(entity)?.density())
    }
}

/// Converts a Rapier AABB to a Bevy AABB.
#[cfg(feature = "dim2")]
fn aabb_to_bevy(aabb: Aabb) -> bevy::shape::Aabb2d {
    bevy::shape::Aabb2d {
        min: aabb.mins,
        max: aabb.maxs,
    }
}

/// Converts a Rapier AABB to a Bevy AABB.
#[cfg(feature = "dim3")]
fn aabb_to_bevy(aabb: Aabb) -> bevy::shape::Aabb3d {
    bevy::shape::Aabb3d {
        min: aabb.mins.into(),
        max: aabb.maxs.into(),
    }
}

/// The sets of joints part of the simulation.
///
/// This should be attached on an entity with a [`RapierContextSimulation`]
///
/// With the `serde-serialize` feature, the entity-to-handle maps aren't serialized: they are
/// rebuilt from the joints’ user-data on deserialization (see [`Self::rebuild_entity_maps`]).
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "serde-serialize",
    serde(from = "serde_shadows::RapierContextJointsData")
)]
#[derive(Component, Default, Debug, Clone)]
pub struct RapierContextJoints {
    /// The set of impulse joints part of the simulation.
    pub impulse_joints: ImpulseJointSet,
    /// The set of multibody joints part of the simulation.
    pub multibody_joints: MultibodyJointSet,

    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2impulse_joint: HashMap<Entity, ImpulseJointHandle>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2multibody_joint: HashMap<Entity, MultibodyJointHandle>,
}

impl RapierContextJoints {
    /// Rebuilds the maps from entities to impulse and multibody joint handles from the
    /// user-data of the joints, which contain the bits of their entity.
    ///
    /// This is called automatically when deserializing this component. Call it manually after
    /// replacing the joint sets with sets restored in another way.
    pub fn rebuild_entity_maps(&mut self) {
        self.entity2impulse_joint = self
            .impulse_joints
            .iter()
            .filter_map(|(handle, joint)| {
                Some((Entity::try_from_bits(joint.data.user_data as u64)?, handle))
            })
            .collect();
        self.entity2multibody_joint = self
            .multibody_joints
            .iter()
            .filter_map(|(handle, _, _, link)| {
                Some((
                    Entity::try_from_bits(link.joint().data.user_data as u64)?,
                    handle,
                ))
            })
            .collect();
    }

    /// The map from entities to impulse joint handles.
    pub fn entity2impulse_joint(&self) -> &HashMap<Entity, ImpulseJointHandle> {
        &self.entity2impulse_joint
    }

    /// The map from entities to multibody joint handles.
    pub fn entity2multibody_joint(&self) -> &HashMap<Entity, MultibodyJointHandle> {
        &self.entity2multibody_joint
    }

    /// Retrieve the entity containing the [`ImpulseJoint`] component the given Rapier impulse
    /// joint was created from.
    pub fn entity_from_impulse_joint(joint: &RapierImpulseJoint) -> Entity {
        Entity::from_bits(joint.data.user_data as u64)
    }

    /// Retrieve the entity containing the [`ImpulseJoint`] component of the Rapier impulse joint
    /// identified by the given handle.
    pub fn impulse_joint_entity(&self, handle: ImpulseJointHandle) -> Option<Entity> {
        self.impulse_joints
            .get(handle)
            .map(Self::entity_from_impulse_joint)
    }

    /// The impulses applied during the last simulation step by the [`ImpulseJoint`] of `entity`.
    ///
    /// This is the value written back into the [`ImpulseJointImpulses`] component after each
    /// step. Returns `None` if `entity` has no impulse joint.
    pub fn impulse_joint_impulses(&self, entity: Entity) -> Option<ImpulseJointImpulses> {
        let joint = self
            .impulse_joints
            .get(*self.entity2impulse_joint.get(&entity)?)?;
        Some(ImpulseJointImpulses::from_rapier(joint))
    }

    /// Iterates through the entities of all the impulse joints attaching the rigid-bodies of
    /// `body1` and `body2` (in any order).
    pub fn impulse_joints_between<'a>(
        &'a self,
        rigidbody_set: &RapierRigidBodySet,
        body1: Entity,
        body2: Entity,
    ) -> impl Iterator<Item = Entity> + 'a {
        let handles = rigidbody_set
            .entity2body
            .get(&body1)
            .copied()
            .zip(rigidbody_set.entity2body.get(&body2).copied());
        handles.into_iter().flat_map(move |(handle1, handle2)| {
            self.impulse_joints
                .joints_between(handle1, handle2)
                .map(|(_, joint)| Self::entity_from_impulse_joint(joint))
        })
    }

    /// Iterates through the entities of all the impulse joints attached to the rigid-body of
    /// `body`, including disabled joints.
    pub fn attached_impulse_joints<'a>(
        &'a self,
        rigidbody_set: &RapierRigidBodySet,
        body: Entity,
    ) -> impl Iterator<Item = Entity> + 'a {
        let handle = rigidbody_set.entity2body.get(&body).copied();
        handle.into_iter().flat_map(move |handle| {
            self.impulse_joints
                .attached_joints(handle)
                .map(|(_, _, _, joint)| Self::entity_from_impulse_joint(joint))
        })
    }

    /// Iterates through the entities of the enabled impulse joints attached to the rigid-body
    /// of `body`.
    pub fn attached_enabled_impulse_joints<'a>(
        &'a self,
        rigidbody_set: &RapierRigidBodySet,
        body: Entity,
    ) -> impl Iterator<Item = Entity> + 'a {
        let handle = rigidbody_set.entity2body.get(&body).copied();
        handle.into_iter().flat_map(move |handle| {
            self.impulse_joints
                .attached_enabled_joints(handle)
                .map(|(_, _, _, joint)| Self::entity_from_impulse_joint(joint))
        })
    }

    /// Retrieve the entity containing the [`MultibodyJoint`] component the given Rapier
    /// multibody joint was created from.
    ///
    /// This is also the entity of the rigid-body of the link containing this joint.
    pub fn entity_from_multibody_joint(joint: &RapierMultibodyJoint) -> Entity {
        Entity::from_bits(joint.data.user_data as u64)
    }

    /// The multibody containing the rigid-body of `entity`, and the index of the link of that
    /// rigid-body in the multibody.
    ///
    /// The root rigid-body of a multibody is its link `0`. Returns `None` if the rigid-body is not
    /// attached to any [`MultibodyJoint`].
    pub fn multibody(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
    ) -> Option<(&Multibody, usize)> {
        let body = rigidbody_set.entity2body.get(&entity)?;
        let link = self.multibody_joints.rigid_body_link(*body)?;
        Some((
            self.multibody_joints.get_multibody(link.multibody)?,
            link.id,
        ))
    }

    /// The multibody containing the rigid-body of `entity`, and the index of the link of that
    /// rigid-body in the multibody, for modification.
    ///
    /// Note that Rapier doesn’t wake up the multibody automatically after a modification.
    pub fn multibody_mut(
        &mut self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
    ) -> Option<(&mut Multibody, usize)> {
        let body = rigidbody_set.entity2body.get(&entity)?;
        let link = *self.multibody_joints.rigid_body_link(*body)?;
        Some((
            self.multibody_joints.get_multibody_mut(link.multibody)?,
            link.id,
        ))
    }

    /// The entity of the root rigid-body of the multibody containing the rigid-body of `entity`.
    pub fn multibody_root(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
    ) -> Option<Entity> {
        let (mb, _) = self.multibody(rigidbody_set, entity)?;
        rigidbody_set.rigid_body_entity(mb.root().rigid_body_handle())
    }

    /// Iterates through the entities of the rigid-bodies of all the links of the multibody
    /// containing the rigid-body of `entity`.
    ///
    /// The root comes first, and each link is yielded before its descendants. All the entities
    /// except the root contain the [`MultibodyJoint`] attaching them to their parent link: to
    /// remove all the joints of a multibody (like Rapier’s
    /// `MultibodyJointSet::remove_multibody_articulations`), remove the [`MultibodyJoint`]
    /// component from these entities (for example with `.skip(1)`).
    pub fn multibody_links<'a>(
        &'a self,
        rigidbody_set: &'a RapierRigidBodySet,
        entity: Entity,
    ) -> impl Iterator<Item = Entity> + 'a {
        self.multibody(rigidbody_set, entity)
            .into_iter()
            .flat_map(move |(mb, _)| {
                mb.links()
                    .filter_map(|link| rigidbody_set.rigid_body_entity(link.rigid_body_handle()))
            })
    }

    /// The entity of the rigid-body of the parent link of the rigid-body of `entity` in its
    /// multibody.
    ///
    /// Returns `None` if `entity` is the root of its multibody, or isn’t part of any multibody.
    pub fn multibody_parent_link(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
    ) -> Option<Entity> {
        let (mb, link_id) = self.multibody(rigidbody_set, entity)?;
        let parent = mb.link(mb.link(link_id)?.parent_id()?)?;
        rigidbody_set.rigid_body_entity(parent.rigid_body_handle())
    }

    /// The entity of the [`MultibodyJoint`] attaching the rigid-bodies of `body1` and `body2`
    /// (in any order), if any.
    pub fn multibody_joint_between(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        body1: Entity,
        body2: Entity,
    ) -> Option<Entity> {
        let handle1 = rigidbody_set.entity2body.get(&body1)?;
        let handle2 = rigidbody_set.entity2body.get(&body2)?;
        let (_, _, link) = self.multibody_joints.joint_between(*handle1, *handle2)?;
        rigidbody_set.rigid_body_entity(link.rigid_body_handle())
    }

    /// Iterates through the entities of all the multibody joints attached to the rigid-body of
    /// `body`: the joint attaching it to its parent link (if any), and the joints of its child
    /// links.
    pub fn attached_multibody_joints<'a>(
        &'a self,
        rigidbody_set: &'a RapierRigidBodySet,
        body: Entity,
    ) -> impl Iterator<Item = Entity> + 'a {
        let handle = rigidbody_set.entity2body.get(&body).copied();
        handle.into_iter().flat_map(move |handle| {
            self.multibody_joints
                .attached_joints(handle)
                .filter_map(|(_, body2, _)| rigidbody_set.rigid_body_entity(body2))
        })
    }

    /// The range of the degrees of freedom of the link of `entity` in the generalized
    /// coordinates, velocities and displacements of its multibody.
    ///
    /// The degrees of freedom of a joint are its free axes, in [`JointAxis`] order.
    ///
    /// [`JointAxis`]: crate::dynamics::JointAxis
    pub fn multibody_link_dofs(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
    ) -> Option<Range<usize>> {
        let (mb, link_id) = self.multibody(rigidbody_set, entity)?;
        let link = mb.link(link_id)?;
        let start = link.assembly_id();
        Some(start..start + link.joint().ndofs())
    }

    /// The generalized velocities of the multibody containing the rigid-body of `entity`.
    ///
    /// See [`Self::multibody_link_dofs`] for the range of each link in this slice.
    pub fn multibody_generalized_velocity(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
    ) -> Option<&[Real]> {
        let (mb, _) = self.multibody(rigidbody_set, entity)?;
        Some(mb.generalized_velocity().data.into_slice())
    }

    /// The generalized velocities of the multibody containing the rigid-body of `entity`, for
    /// modification.
    ///
    /// This wakes up the rigid-bodies of the multibody.
    pub fn multibody_generalized_velocity_mut(
        &mut self,
        rigidbody_set: &mut RapierRigidBodySet,
        entity: Entity,
    ) -> Option<&mut [Real]> {
        let (mb, _) = self.multibody_mut(rigidbody_set, entity)?;
        crate::plugin::systems::wake_up_multibody(mb, &mut rigidbody_set.bodies);
        Some(mb.generalized_velocity_mut().data.into_slice_mut())
    }

    /// The generalized velocities of the [`MultibodyJoint`] of `entity`, one per free axis in
    /// [`JointAxis`] order, for modification.
    ///
    /// This is typically used to drive a [`KinematicMultibodyJoint`]. This wakes up the
    /// rigid-bodies of the multibody.
    ///
    /// [`JointAxis`]: crate::dynamics::JointAxis
    pub fn multibody_joint_velocity_mut(
        &mut self,
        rigidbody_set: &mut RapierRigidBodySet,
        entity: Entity,
    ) -> Option<&mut [Real]> {
        let handle = *self.entity2multibody_joint.get(&entity)?;
        let (mb, link_id) = self.multibody_joints.get_mut(handle)?;
        crate::plugin::systems::wake_up_multibody(mb, &mut rigidbody_set.bodies);
        let link = mb.link(link_id)?;
        let range = link.assembly_id()..link.assembly_id() + link.joint().ndofs();
        Some(&mut mb.generalized_velocity_mut().data.into_slice_mut()[range])
    }

    /// The state of the [`MultibodyJoint`] of `entity`.
    ///
    /// This is the value written back into the [`MultibodyJointState`] component after each
    /// step.
    pub fn multibody_joint_state(&self, entity: Entity) -> Option<MultibodyJointState> {
        self.multibody_joint_state_from_handle(*self.entity2multibody_joint.get(&entity)?)
    }

    /// The state of the multibody joint with the given handle.
    pub(crate) fn multibody_joint_state_from_handle(
        &self,
        handle: MultibodyJointHandle,
    ) -> Option<MultibodyJointState> {
        let (mb, link_id) = self.multibody_joints.get(handle)?;
        let link = mb.link(link_id)?;
        Some(MultibodyJointState::from_rapier(
            link.joint(),
            mb.joint_velocity(link).as_slice(),
        ))
    }

    /// The jacobian of the link of `entity`, mapping the generalized velocities of its multibody
    /// to the link’s world-space linear and angular velocities.
    ///
    /// This is computed by the last forward kinematics (see
    /// [`Self::multibody_forward_kinematics`]), which runs at each simulation step.
    pub fn multibody_body_jacobian(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
    ) -> Option<&Jacobian<Real>> {
        let (mb, link_id) = self.multibody(rigidbody_set, entity)?;
        Some(mb.body_jacobian(link_id))
    }

    /// Applies displacements, in generalized coordinates, to the multibody containing the
    /// rigid-body of `entity`.
    ///
    /// `displacements` must contain one value per degree of freedom of the multibody. This only
    /// modifies the joint coordinates: call [`Self::multibody_forward_kinematics`] to update the
    /// rigid-bodies. Returns `false` if `entity` isn’t part of a multibody or the number of
    /// displacements doesn’t match.
    pub fn multibody_apply_displacements(
        &mut self,
        rigidbody_set: &RapierRigidBodySet,
        entity: Entity,
        displacements: &[Real],
    ) -> bool {
        let Some((mb, _)) = self.multibody_mut(rigidbody_set, entity) else {
            return false;
        };
        if displacements.len() != mb.ndofs() {
            return false;
        }
        mb.apply_displacements(displacements);
        true
    }

    /// Runs forward kinematics on the multibody containing the rigid-body of `entity`, and moves
    /// its rigid-bodies to the resulting poses.
    ///
    /// Set `read_root_pose_from_rigid_body` if the root rigid-body was moved and the multibody
    /// should follow. The `Transform` of the rigid-bodies are updated during the next writeback
    /// (after the next simulation step). Returns `false` if `entity` isn’t part of a multibody.
    pub fn multibody_forward_kinematics(
        &mut self,
        rigidbody_set: &mut RapierRigidBodySet,
        entity: Entity,
        read_root_pose_from_rigid_body: bool,
    ) -> bool {
        let Some((mb, _)) = self.multibody_mut(rigidbody_set, entity) else {
            return false;
        };
        mb.forward_kinematics(&rigidbody_set.bodies, read_root_pose_from_rigid_body);
        mb.update_rigid_bodies(&mut rigidbody_set.bodies, true);
        true
    }

    /// Computes the displacements, in generalized coordinates, moving the link of `link` as close
    /// as possible to the world-space `target` pose, with a jacobian-based inverse kinematics
    /// solver.
    ///
    /// `displacements` is an output buffer: it is cleared and resized to the number of degrees of
    /// freedom of the multibody (see [`Self::multibody_link_dofs`]), so the solver always starts
    /// from the current configuration, then filled with the result, which can then be applied with
    /// [`Self::multibody_apply_displacements`]. The same buffer can be reused across calls without
    /// resetting it. Only the joints for which `joint_can_move`
    /// returns `true` are moved; it is called with the entity of each link between the root
    /// (included) and `link`. Returns `false` if `link` isn’t part of a multibody.
    pub fn multibody_inverse_kinematics(
        &self,
        rigidbody_set: &RapierRigidBodySet,
        link: Entity,
        target: Transform,
        options: &InverseKinematicsOption,
        joint_can_move: impl Fn(Entity) -> bool,
        displacements: &mut Vec<Real>,
    ) -> bool {
        let Some((mb, link_id)) = self.multibody(rigidbody_set, link) else {
            return false;
        };
        displacements.clear();
        displacements.resize(mb.ndofs(), 0.0);
        let mut result = DVector::from_vec(std::mem::take(displacements));
        mb.inverse_kinematics(
            &rigidbody_set.bodies,
            link_id,
            options,
            &crate::utils::transform_to_iso(&target),
            |link| {
                rigidbody_set
                    .rigid_body_entity(link.rigid_body_handle())
                    .is_some_and(&joint_can_move)
            },
            &mut result,
        );
        *displacements = result.data.into();
        true
    }
}

/// Wrapper around [QueryPipeline] to provide bevy friendly methods.
///
/// This wrapper is designed to be short lived, made whenever necessary.
///
/// See [RapierQueryPipeline::new_scoped] to create one.
#[derive(Copy, Clone)]
pub struct RapierQueryPipeline<'a> {
    /// The query pipeline, which performs scene queries (ray-casting, point projection, etc.)
    pub query_pipeline: QueryPipeline<'a>,
}

/// Wrapper around [QueryPipelineMut] to provide bevy friendly methods.
///
/// This wrapper is designed to be short lived, made whenever necessary.
///
/// See [RapierQueryPipelineMut::new_scoped] to create one.
pub struct RapierQueryPipelineMut<'a> {
    /// The query pipeline, which performs scene queries (ray-casting, point projection, etc.)
    pub query_pipeline: QueryPipelineMut<'a>,
}
impl<'a> RapierQueryPipelineMut<'a> {
    /// Creates a temporary [RapierQueryPipelineMut] and passes it as a parameter to `scoped_fn`.
    pub fn new_scoped<T>(
        broad_phase: &DefaultBroadPhase,
        colliders: &mut RapierContextColliders,
        rigid_bodies: &mut RapierRigidBodySet,
        filter: &QueryFilter<'_>,
        dispatcher: &dyn QueryDispatcher,
        scoped_fn: impl FnOnce(RapierQueryPipelineMut<'_>) -> T,
    ) -> T {
        let predicate = to_rapier_query_filter_combined_predicate(filter);
        let rapier_filter =
            to_rapier_query_filter(filter, colliders, rigid_bodies, predicate.as_ref());
        let query_pipeline = broad_phase.as_query_pipeline_mut(
            dispatcher,
            &mut rigid_bodies.bodies,
            &mut colliders.colliders,
            rapier_filter,
        );
        scoped_fn(RapierQueryPipelineMut { query_pipeline })
    }

    /// Downgrades the mutable reference to an immutable reference.
    pub fn as_ref(&self) -> RapierQueryPipeline<'_> {
        RapierQueryPipeline {
            query_pipeline: self.query_pipeline.as_ref(),
        }
    }
}

/// Wraps a [bevy query filter predicate](QueryFilter::predicate) into a [rapier query filter predicate](RapierQueryFilter::predicate).
pub fn to_rapier_query_filter_predicate(
    predicate: &dyn Fn(Entity, &RapierCollider) -> bool,
) -> impl Fn(ColliderHandle, &RapierCollider) -> bool + use<'_> {
    |_: ColliderHandle, collider: &RapierCollider| -> bool {
        let entity = RapierContextColliders::entity_from_collider(collider);
        (predicate)(entity, collider)
    }
}

/// Wraps the [`QueryFilter::predicate`] into a Rapier predicate, or returns `None` if it isn’t set.
fn to_rapier_query_filter_combined_predicate<'b>(
    filter: &QueryFilter<'b>,
) -> Option<impl Fn(ColliderHandle, &RapierCollider) -> bool + 'b> {
    filter.predicate.map(to_rapier_query_filter_predicate)
}

/// Converts a bevy [`QueryFilter`] into a Rapier query filter using the given Rapier predicate.
fn to_rapier_query_filter<'b>(
    filter: &QueryFilter<'_>,
    colliders: &RapierContextColliders,
    rigid_bodies: &RapierRigidBodySet,
    predicate: Option<&'b (impl Fn(ColliderHandle, &RapierCollider) -> bool + 'b)>,
) -> RapierQueryFilter<'b> {
    RapierQueryFilter {
        flags: filter.flags,
        groups: filter.groups.map(CollisionGroups::into),
        exclude_collider: filter
            .exclude_collider
            .and_then(|c| colliders.entity2collider.get(&c).copied()),
        exclude_rigid_body: filter
            .exclude_rigid_body
            .and_then(|b| rigid_bodies.entity2body.get(&b).copied()),
        predicate: predicate.map(|p| p as &dyn Fn(ColliderHandle, &RapierCollider) -> bool),
    }
}

impl<'a> RapierQueryPipeline<'a> {
    /// Creates a temporary [RapierQueryPipeline] and passes it as a parameter to `scoped_fn`.
    pub fn new_scoped<T>(
        broad_phase: &DefaultBroadPhase,
        colliders: &RapierContextColliders,
        rigid_bodies: &RapierRigidBodySet,
        filter: &QueryFilter<'_>,
        dispatcher: &dyn QueryDispatcher,
        scoped_fn: impl FnOnce(RapierQueryPipeline<'_>) -> T,
    ) -> T {
        let predicate = to_rapier_query_filter_combined_predicate(filter);
        let rapier_filter =
            to_rapier_query_filter(filter, colliders, rigid_bodies, predicate.as_ref());
        let query_pipeline = broad_phase.as_query_pipeline(
            dispatcher,
            &rigid_bodies.bodies,
            &colliders.colliders,
            rapier_filter,
        );
        scoped_fn(RapierQueryPipeline { query_pipeline })
    }

    /// Retrieves the Entity for a given collider handle.
    pub fn collider_entity(&self, collider_handle: ColliderHandle) -> Entity {
        RapierContextColliders::collider_entity_with_set(
            self.query_pipeline.colliders,
            collider_handle,
        )
        .unwrap()
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// * `ray_origin`: the starting point of the ray to cast.
    /// * `ray_dir`: the direction of the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///   it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///   even if its starts inside of it.
    pub fn cast_ray(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
    ) -> Option<(Entity, Real)> {
        let ray = Ray::new(ray_origin, ray_dir);

        let (h, toi) = self.query_pipeline.cast_ray(&ray, max_toi, solid)?;

        Some((self.collider_entity(h), toi))
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// * `ray_origin`: the starting point of the ray to cast.
    /// * `ray_dir`: the direction of the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///   it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///   even if its starts inside of it.
    pub fn cast_ray_and_get_normal(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
    ) -> Option<(Entity, RayIntersection)> {
        let ray = Ray::new(ray_origin, ray_dir);

        let (h, result) = self
            .query_pipeline
            .cast_ray_and_get_normal(&ray, max_toi, solid)?;

        Some((
            self.collider_entity(h),
            RayIntersection::from_rapier(result, ray_origin, ray_dir),
        ))
    }

    /// Iterates through all the colliders intersecting a given ray, alongside their Rapier collider.
    ///
    /// # Parameters
    /// * `ray_origin`: the starting point of the ray to cast.
    /// * `ray_dir`: the direction of the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///   it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///   even if its starts inside of it.
    pub fn intersect_ray(
        &'a self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_toi: Real,
        solid: bool,
    ) -> impl Iterator<Item = (Entity, &'a RapierCollider, RayIntersection)> + 'a {
        let ray = Ray::new(ray_origin, ray_dir);

        self.query_pipeline.intersect_ray(ray, max_toi, solid).map(
            move |(_, collider, intersection)| {
                (
                    RapierContextColliders::entity_from_collider(collider),
                    collider,
                    RayIntersection::from_rapier(intersection, ray_origin, ray_dir),
                )
            },
        )
    }

    /// Retrieve all the colliders intersecting the given shape, alongside their Rapier collider.
    ///
    /// # Parameters
    /// * `shape_pos` - The position of the shape used for the intersection test.
    /// * `shape_rot` - The orientation of the shape used for the intersection test.
    /// * `shape` - The shape used for the intersection test.
    pub fn intersect_shape(
        &'a self,
        shape_pos: Vect,
        shape_rot: Rot,
        shape: &'a (impl AsShape + ?Sized + 'a),
    ) -> impl Iterator<Item = (Entity, &'a RapierCollider)> + 'a {
        let shape = shape.as_shape();
        let scaled_transform = crate::utils::pose_from(shape_pos, shape_rot);

        self.query_pipeline
            .intersect_shape(scaled_transform, shape)
            .map(|(_, collider)| {
                (
                    RapierContextColliders::entity_from_collider(collider),
                    collider,
                )
            })
    }

    /// Find the projection of a point on the closest collider.
    ///
    /// # Parameters
    /// * `point` - The point to project.
    /// * `max_dist` - The maximum distance between the point and its projection. Colliders
    ///   further than this distance are ignored.
    /// * `solid` - If this is set to `true` then the collider shapes are considered to
    ///   be plain (if the point is located inside of a plain shape, its projection is the point
    ///   itself). If it is set to `false` the collider shapes are considered to be hollow
    ///   (if the point is located inside of an hollow shape, it is projected on the shape's
    ///   boundary).
    pub fn project_point(
        &self,
        point: Vect,
        max_dist: Real,
        solid: bool,
    ) -> Option<(Entity, PointProjection)> {
        let (h, result) = self.query_pipeline.project_point(point, max_dist, solid)?;

        Some((
            self.collider_entity(h),
            PointProjection::from_rapier(result),
        ))
    }

    /// Find all the colliders containing the given point, alongside their Rapier collider.
    ///
    /// # Parameters
    /// * `point` - The point used for the containment test.
    pub fn intersect_point(
        &'a self,
        point: Vect,
    ) -> impl Iterator<Item = (Entity, &'a RapierCollider)> + 'a {
        self.query_pipeline
            .intersect_point(point)
            .map(|(_, collider)| {
                (
                    RapierContextColliders::entity_from_collider(collider),
                    collider,
                )
            })
    }

    /// Find the projection of a point on the boundary of the closest collider.
    ///
    /// The results include the ID of the feature hit by the point.
    ///
    /// # Parameters
    /// * `point` - The point to project.
    /// * `max_dist` - The maximum distance between the point and its projection. Colliders
    ///   further than this distance are ignored. Use `Real::MAX` for an unbounded search.
    pub fn project_point_and_get_feature(
        &self,
        point: Vect,
        max_dist: Real,
    ) -> Option<(Entity, PointProjection, FeatureId)> {
        let (h, proj, fid) = self
            .query_pipeline
            .project_point_and_get_feature(point, max_dist)?;

        Some((
            self.collider_entity(h),
            PointProjection::from_rapier(proj),
            fid,
        ))
    }

    /// Finds all the colliders with an [`Aabb`] intersecting the given [`Aabb`], alongside their
    /// Rapier collider.
    ///
    /// Note that the collider AABB taken into account is the one currently stored in the query
    /// pipeline’s BVH. It doesn’t recompute the latest collider AABB.
    pub fn intersect_aabb_conservative(
        &'a self,
        #[cfg(feature = "dim2")] aabb: bevy::shape::Aabb2d,
        #[cfg(feature = "dim3")] aabb: bevy::shape::Aabb3d,
    ) -> impl Iterator<Item = (Entity, &'a RapierCollider)> + 'a {
        #[cfg(feature = "dim2")]
        let scaled_aabb = Aabb {
            mins: aabb.min,
            maxs: aabb.max,
        };
        #[cfg(feature = "dim3")]
        let scaled_aabb = Aabb {
            mins: aabb.min.into(),
            maxs: aabb.max.into(),
        };
        self.query_pipeline
            .intersect_aabb_conservative(scaled_aabb)
            .map(|(_, collider)| {
                (
                    RapierContextColliders::entity_from_collider(collider),
                    collider,
                )
            })
    }

    /// Casts a shape at a constant linear velocity and retrieve the first collider it hits.
    ///
    /// This is similar to ray-casting except that we are casting a whole shape instead of just a
    /// point (the ray origin). In the resulting `ShapeCastHit`, witness and normal 1 refer to the
    /// collider hit and are in world-space, while witness and normal 2 refer to the cast shape and
    /// are in its local-space (see [`ShapeCastHitDetails`](crate::geometry::ShapeCastHitDetails)).
    ///
    /// # Parameters
    /// * `shape_pos` - The initial translation of the shape to cast.
    /// * `shape_rot` - The rotation of the shape to cast.
    /// * `shape_vel` - The constant velocity of the shape to cast (i.e. the cast direction).
    /// * `shape` - The shape to cast.
    /// * `max_toi` - The maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the distance traveled by the shape to `shape_vel.norm() * maxToi`.
    /// * `stop_at_penetration` - If the casted shape starts in a penetration state with any
    ///   collider, two results are possible. If `stop_at_penetration` is `true` then, the
    ///   result will have a `toi` equal to `start_time`. If `stop_at_penetration` is `false`
    ///   then the nonlinear shape-casting will see if further motion wrt. the penetration normal
    ///   would result in tunnelling. If it does not (i.e. we have a separating velocity along
    ///   that normal) then the nonlinear shape-casting will attempt to find another impact,
    ///   at a time `> start_time` that could result in tunnelling.
    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape(
        &'a self,
        shape_pos: Vect,
        shape_rot: Rot,
        shape_vel: Vect,
        shape: &(impl AsShape + ?Sized),
        options: ShapeCastOptions,
    ) -> Option<(Entity, ShapeCastHit)> {
        let shape = shape.as_shape();
        let scaled_transform = crate::utils::pose_from(shape_pos, shape_rot);

        let (h, result) =
            self.query_pipeline
                .cast_shape(&scaled_transform, shape_vel, shape, options)?;

        Some((
            self.collider_entity(h),
            ShapeCastHit::from_rapier(result, options.compute_impact_geometry_on_penetration),
        ))
    }

    /// Casts a shape with an arbitrary continuous motion and retrieve the first collider it hits.
    ///
    /// In the resulting `ShapeCastHit`, witness and normal 1 refer to the collider hit and are in
    /// world-space, while witness and normal 2 refer to the cast shape and are in its local-space.
    ///
    /// # Parameters
    /// * `shape_motion` - The motion of the shape.
    /// * `shape` - The shape to cast.
    /// * `start_time` - The starting time of the interval where the motion takes place.
    /// * `end_time` - The end time of the interval where the motion takes place.
    /// * `stop_at_penetration` - If the casted shape starts in a penetration state with any
    ///   collider, two results are possible. If `stop_at_penetration` is `true` then, the
    ///   result will have a `toi` equal to `start_time`. If `stop_at_penetration` is `false`
    ///   then the nonlinear shape-casting will see if further motion wrt. the penetration normal
    ///   would result in tunnelling. If it does not (i.e. we have a separating velocity along
    ///   that normal) then the nonlinear shape-casting will attempt to find another impact,
    ///   at a time `> start_time` that could result in tunnelling.
    pub fn cast_shape_nonlinear(
        &self,
        shape_motion: &NonlinearMotion,
        shape: &(impl AsShape + ?Sized),
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
    ) -> Option<(Entity, ShapeCastHit)> {
        let shape = shape.as_shape();
        let (h, result) = self.query_pipeline.cast_shape_nonlinear(
            &shape_motion.into_rapier(),
            shape,
            start_time,
            end_time,
            stop_at_penetration,
        )?;

        Some((
            self.collider_entity(h),
            ShapeCastHit::from_rapier(result, false),
        ))
    }

    /// Computes the smallest distance between the given shape and the closest collider.
    ///
    /// Returns the entity of the closest collider alongside that distance, which is `0.0` if they
    /// are intersecting. Returns `None` if no collider passes the query filter.
    ///
    /// # Parameters
    /// * `shape_pos` - The translation of the shape.
    /// * `shape_rot` - The rotation of the shape.
    /// * `shape` - The shape to compute the distance from.
    pub fn distance_to_shape(
        &self,
        shape_pos: Vect,
        shape_rot: Rot,
        shape: &(impl AsShape + ?Sized),
    ) -> Option<(Entity, Real)> {
        let shape = shape.as_shape();
        let pose = crate::utils::pose_from(shape_pos, shape_rot);
        let (id, dist) = CompositeShapeRef(&self.query_pipeline).distance_to_shape(
            self.query_pipeline.dispatcher,
            &pose,
            shape,
        )?;
        Some((self.collider_entity_from_id(id)?, dist.distance))
    }

    /// Computes the closest points between the given shape and the closest collider.
    ///
    /// Returns the entity of the closest collider alongside the closest points. In
    /// [`ShapeClosestPoints::WithinMargin`], the first point is on the collider and the second
    /// point is on `shape`, both in world-space. Returns `None` if no collider lies within
    /// `max_dist` of `shape`.
    ///
    /// # Parameters
    /// * `shape_pos` - The translation of the shape.
    /// * `shape_rot` - The rotation of the shape.
    /// * `shape` - The shape to compute the closest points from.
    /// * `max_dist` - Colliders further than this distance from `shape` are ignored.
    pub fn closest_points_to_shape(
        &self,
        shape_pos: Vect,
        shape_rot: Rot,
        shape: &(impl AsShape + ?Sized),
        max_dist: Real,
    ) -> Option<(Entity, ShapeClosestPoints)> {
        let shape = shape.as_shape();
        let pose = crate::utils::pose_from(shape_pos, shape_rot);
        let (id, mut pts) = CompositeShapeRef(&self.query_pipeline).closest_points_to_shape(
            self.query_pipeline.dispatcher,
            &pose,
            shape,
            max_dist,
        )?;
        match &mut pts {
            ClosestPoints::Disjoint => return None,
            ClosestPoints::Intersecting => {}
            // The first point is already in world-space, the second one is local to `shape`.
            ClosestPoints::WithinMargin(_, p2) => *p2 = pose * *p2,
        }
        Some((
            self.collider_entity_from_id(id)?,
            ShapeClosestPoints::from_rapier(pts),
        ))
    }

    /// Computes the contact between the given shape and the closest (or deepest penetrating)
    /// collider.
    ///
    /// Returns the entity of that collider alongside the contact, where the first shape is the
    /// collider and the second shape is `shape`; contact points and normals are in world-space.
    /// Returns `None` if no collider lies within `prediction` of `shape`.
    ///
    /// # Parameters
    /// * `shape_pos` - The translation of the shape.
    /// * `shape_rot` - The rotation of the shape.
    /// * `shape` - The shape to compute the contact with.
    /// * `prediction` - Colliders further than this distance from `shape` are ignored.
    pub fn contact_with_shape(
        &self,
        shape_pos: Vect,
        shape_rot: Rot,
        shape: &(impl AsShape + ?Sized),
        prediction: Real,
    ) -> Option<(Entity, ShapeContact)> {
        let shape = shape.as_shape();
        let pose = crate::utils::pose_from(shape_pos, shape_rot);
        let (id, mut contact) = CompositeShapeRef(&self.query_pipeline).contact_with_shape(
            self.query_pipeline.dispatcher,
            &pose,
            shape,
            prediction,
        )?;
        // The first point and normal are already in world-space, the second ones are local to `shape`.
        contact.point2 = pose * contact.point2;
        contact.normal2 = pose.rotation * contact.normal2;
        Some((
            self.collider_entity_from_id(id)?,
            ShapeContact::from_rapier(contact),
        ))
    }

    /// The bounding volume hierarchy containing the world-space AABB of every collider.
    ///
    /// This can be used for custom scene traversals. Each leaf index identifies a collider and
    /// can be resolved with [`Self::bvh_leaf_collider`]. Note that the BVH itself ignores the
    /// query filter.
    pub fn bvh(&self) -> &'a Bvh {
        self.query_pipeline.bvh
    }

    /// Retrieves the entity and Rapier collider identified by a leaf index of [`Self::bvh`].
    ///
    /// Returns `None` if the collider doesn't exist or is rejected by the query filter.
    pub fn bvh_leaf_collider(&self, leaf: u32) -> Option<(Entity, &'a RapierCollider)> {
        let (collider, handle) = self.query_pipeline.colliders.get_unknown_gen(leaf)?;
        self.query_pipeline
            .filter
            .test(self.query_pipeline.bodies, handle, collider)
            .then(|| {
                (
                    RapierContextColliders::entity_from_collider(collider),
                    collider,
                )
            })
    }

    /// Retrieves the Entity of the collider identified by a sub-shape id of the query pipeline.
    fn collider_entity_from_id(&self, id: u32) -> Option<Entity> {
        self.query_pipeline
            .colliders
            .get_unknown_gen(id)
            .map(|(collider, _)| RapierContextColliders::entity_from_collider(collider))
    }
}

/// The set of rigid-bodies part of the simulation.
///
/// This should be attached on an entity with a [`RapierContextSimulation`]
///
/// With the `serde-serialize` feature, the entity-to-handle map isn't serialized: it is rebuilt
/// from the rigid-bodies’ user-data on deserialization (see [`Self::rebuild_entity_maps`]).
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "serde-serialize",
    serde(from = "serde_shadows::RapierRigidBodySetData")
)]
#[derive(Component, Default, Clone)]
pub struct RapierRigidBodySet {
    /// The set of rigid-bodies part of the simulation.
    pub bodies: RigidBodySet,
    /// The set of soft-bodies part of the simulation.
    pub soft_bodies: SoftBodySet,
    /// NOTE: this map is needed to handle despawning.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2body: HashMap<Entity, RigidBodyHandle>,
    /// NOTE: this map is needed to handle despawning.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) entity2soft_body: HashMap<Entity, SoftBodyHandle>,

    /// For transform change detection.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) last_body_transform_set: HashMap<RigidBodyHandle, GlobalTransform>,
}

impl RapierRigidBodySet {
    /// Rebuilds the map from entities to rigid-body handles from the user-data of the
    /// rigid-bodies, which contain the bits of their entity.
    ///
    /// This is called automatically when deserializing this component. Call it manually after
    /// replacing [`Self::bodies`] with a set restored in another way. The transform change
    /// detection state is reset, so the next [`GlobalTransform`] change of a rigid-body entity
    /// is applied to its rigid-body.
    ///
    /// The soft-body map is rebuilt from the soft bodies' user-data too. The hidden proxy
    /// rigid-bodies of the soft bodies are not mapped, except the proxies of the clusters that
    /// have their own entity (see [`SoftBodyCluster`](crate::dynamics::SoftBodyCluster)) and the
    /// whole-body proxy of each soft body (see [`Self::soft_body_whole_proxy`]).
    pub fn rebuild_entity_maps(&mut self) {
        let soft_bodies = &self.soft_bodies;
        self.entity2body = self
            .bodies
            .iter()
            .filter(|(_, rb)| {
                // A proxy with the user-data of its soft body stands for the soft body entity.
                rb.soft_body()
                    .and_then(|h| soft_bodies.get(h))
                    .is_none_or(|sb| sb.user_data != rb.user_data)
            })
            .filter_map(|(handle, rb)| Some((Entity::try_from_bits(rb.user_data as u64)?, handle)))
            .collect();
        self.entity2soft_body = self
            .soft_bodies
            .iter()
            .filter_map(|(handle, sb)| Some((Entity::try_from_bits(sb.user_data as u64)?, handle)))
            .collect();
        let soft_bodies: Vec<_> = self
            .entity2soft_body
            .iter()
            .map(|(entity, handle)| (*entity, *handle))
            .collect();
        for (entity, handle) in soft_bodies {
            self.map_soft_body_whole_proxy(entity, handle);
        }
        self.last_body_transform_set.clear();
    }

    /// Maps `entity` to the whole-body proxy of its soft body `handle` (or unmaps it if the soft
    /// body has none).
    pub(crate) fn map_soft_body_whole_proxy(&mut self, entity: Entity, handle: SoftBodyHandle) {
        match self.whole_proxy(entity, handle) {
            Some(proxy) => {
                self.entity2body.insert(entity, proxy);
            }
            None => {
                self.entity2body.remove(&entity);
            }
        }
    }

    /// The whole-body proxy of the soft body `handle` standing for `entity`: its root body, or
    /// else the proxy of a cluster covering all its particles.
    fn whole_proxy(&self, entity: Entity, handle: SoftBodyHandle) -> Option<RigidBodyHandle> {
        let sb = self.soft_bodies.get(handle)?;
        let bits = entity.to_bits() as u128;
        let stands_for_entity = |proxy: RigidBodyHandle| {
            self.bodies
                .get(proxy)
                .is_some_and(|rb| rb.user_data == bits)
        };
        let root = sb.root_body();
        if stands_for_entity(root) {
            return Some(root);
        }
        sb.live_clusters()
            .map(|(_, cluster)| cluster)
            .find(|cluster| {
                cluster.particles().len() == sb.num_particles()
                    && stands_for_entity(cluster.proxy())
            })
            .map(|cluster| cluster.proxy())
    }

    /// Moves the Rapier objects of the new soft body `handle` (split off another one by a tear) to
    /// `entity`: the user-data of the soft body and of its proxies and colliders standing for the
    /// soft body it was split from.
    pub(crate) fn map_soft_body_piece(
        &mut self,
        colliders: &mut RapierContextColliders,
        handle: SoftBodyHandle,
        entity: Entity,
    ) {
        let new_bits = entity.to_bits() as u128;
        let Some(sb) = self.soft_bodies.get_mut(handle) else {
            return;
        };
        let old_bits = sb.user_data;
        sb.user_data = new_bits;
        for (_, cluster) in sb.live_clusters() {
            let Some(rb) = self.bodies.get_mut(cluster.proxy()) else {
                continue;
            };
            if rb.user_data == old_bits {
                rb.user_data = new_bits;
            }
            for co_handle in rb.colliders() {
                if let Some(co) = colliders.colliders.get_mut(*co_handle) {
                    if co.user_data == old_bits {
                        co.user_data = new_bits;
                    }
                }
            }
        }
        self.entity2soft_body.insert(entity, handle);
    }

    /// Maps the soft body entity to the collider of the soft body's collision mesh.
    pub(crate) fn map_soft_body_collision_mesh(
        &self,
        colliders: &mut RapierContextColliders,
        handle: SoftBodyHandle,
        entity: Entity,
    ) {
        match self
            .soft_bodies
            .get(handle)
            .and_then(|sb| sb.collision_mesh())
        {
            Some(mesh) => {
                colliders.entity2collider.insert(entity, mesh.collider());
            }
            None => {
                colliders.entity2collider.remove(&entity);
            }
        }
    }

    /// The colliders standing for the soft body of `entity`: the colliders of its collision
    /// meshes (or of its particles), attached to its proxies and reporting `entity` as their
    /// entity. The colliders attached by the user to its proxies (e.g. [`DeformableCollider`]s or
    /// children colliders) are not included.
    ///
    /// Returns `None` if `entity` has no soft body.
    ///
    /// [`DeformableCollider`]: crate::dynamics::DeformableCollider
    pub fn soft_body_colliders(
        &self,
        colliders: &ColliderSet,
        entity: Entity,
    ) -> Option<Vec<ColliderHandle>> {
        let sb = self.soft_body(entity)?;
        let bits = entity.to_bits() as u128;
        let mut result = vec![];
        for (_, cluster) in sb.live_clusters() {
            let Some(rb) = self.bodies.get(cluster.proxy()) else {
                continue;
            };
            result.extend(
                rb.colliders()
                    .iter()
                    .copied()
                    .filter(|h| colliders.get(*h).is_some_and(|co| co.user_data == bits)),
            );
        }
        Some(result)
    }

    /// The map from entities to soft-body handles.
    pub fn entity2soft_body(&self) -> &HashMap<Entity, SoftBodyHandle> {
        &self.entity2soft_body
    }

    /// Retrieve the Bevy entity of the given Rapier soft body (identified by its handle).
    pub fn soft_body_entity(&self, handle: SoftBodyHandle) -> Option<Entity> {
        let entity = Entity::try_from_bits(self.soft_bodies.get(handle)?.user_data as u64)?;
        (self.entity2soft_body.get(&entity) == Some(&handle)).then_some(entity)
    }

    /// The Rapier soft body attached to the given `entity`, if any.
    pub fn soft_body(&self, entity: Entity) -> Option<&RapierSoftBody> {
        self.soft_bodies.get(*self.entity2soft_body.get(&entity)?)
    }

    /// The Rapier soft body attached to the given `entity`, if any.
    ///
    /// This gives access to every setter of the soft body (particle positions, velocities and
    /// kinematic targets, pinning, forces, impulses, clusters...). Prefer the soft-body components
    /// (e.g. [`SoftBodyMaterial`](crate::dynamics::SoftBodyMaterial)) for the properties they
    /// cover, since they are applied again whenever they change.
    pub fn soft_body_mut(&mut self, entity: Entity) -> Option<&mut RapierSoftBody> {
        self.soft_bodies
            .get_mut(*self.entity2soft_body.get(&entity)?)
    }

    /// The world-space positions of the particles of the soft body of `entity`.
    ///
    /// Returns `None` if `entity` has no soft body.
    pub fn soft_body_particle_positions(
        &self,
        entity: Entity,
    ) -> Option<impl ExactSizeIterator<Item = Vect> + '_> {
        Some(self.soft_body(entity)?.particle_positions())
    }

    /// The world-space velocities of the particles of the soft body of `entity`.
    ///
    /// Returns `None` if `entity` has no soft body.
    pub fn soft_body_particle_velocities(
        &self,
        entity: Entity,
    ) -> Option<impl ExactSizeIterator<Item = Vect> + '_> {
        Some(self.soft_body(entity)?.particle_velocities())
    }

    /// The area (2D) or volume (3D) currently enclosed by the soft body of `entity` (`0.0` for a
    /// soft body without closed boundary).
    ///
    /// Returns `None` if `entity` has no soft body.
    pub fn soft_body_volume(&self, entity: Entity) -> Option<Real> {
        Some(self.soft_body(entity)?.volume())
    }

    /// The total mass of the particles of the soft body of `entity`.
    ///
    /// Returns `None` if `entity` has no soft body.
    pub fn soft_body_mass(&self, entity: Entity) -> Option<Real> {
        Some(self.soft_body(entity)?.mass())
    }

    /// The world-space, mass-weighted, center of the particles of the soft body of `entity`.
    ///
    /// Returns `None` if `entity` has no soft body.
    pub fn soft_body_center_of_mass(&self, entity: Entity) -> Option<Vect> {
        Some(self.soft_body(entity)?.center_of_mass())
    }

    /// Whether the soft body of `entity` is sleeping.
    ///
    /// Returns `None` if `entity` has no soft body.
    pub fn is_soft_body_sleeping(&self, entity: Entity) -> Option<bool> {
        Some(self.soft_body(entity)?.is_sleeping())
    }

    /// Wakes up the soft body of `entity` (effective at the start of the next step, together
    /// with the island of the bodies it touches).
    ///
    /// Returns `false` if `entity` has no soft body.
    pub fn wake_up_soft_body(&mut self, entity: Entity, strong: bool) -> bool {
        let Some(handle) = self.entity2soft_body.get(&entity).copied() else {
            return false;
        };
        self.soft_bodies.wake_up(handle, &mut self.bodies, strong);
        true
    }

    /// The proxy rigid-body of the cluster `cluster` of the soft body of `entity`.
    ///
    /// The cluster `0` is the whole-body cluster created with the soft body. Returns `None` if
    /// `entity` has no soft body or if that cluster doesn't exist.
    pub fn soft_body_cluster_proxy(&self, entity: Entity, cluster: u32) -> Option<RigidBodyHandle> {
        self.soft_body(entity)?.cluster_proxy(cluster)
    }

    /// The whole-body proxy rigid-body of the soft body of `entity`: the proxy of its cluster
    /// covering all its particles, which `entity` is mapped to in [`Self::entity2body`].
    ///
    /// This lets the soft body entity be used like a rigid-body entity by impulse joints (as
    /// their `parent`, or on the soft body entity itself). Returns `None` if `entity` has no soft
    /// body.
    pub fn soft_body_whole_proxy(&self, entity: Entity) -> Option<RigidBodyHandle> {
        self.whole_proxy(entity, *self.entity2soft_body.get(&entity)?)
    }

    /// The soft body and the index of the cluster standing for `entity`: a
    /// [`SoftBodyCluster`](crate::dynamics::SoftBodyCluster) entity, or a soft body entity (for
    /// its whole-body cluster).
    ///
    /// The index can be given to the cluster methods of [`RapierSoftBody`] (e.g.
    /// [`RapierSoftBody::set_cluster_pinned`]). It may change when the soft body tears, so it
    /// should not be stored. Returns `None` if `entity` doesn't stand for a live cluster.
    pub fn soft_body_cluster_index(&self, entity: Entity) -> Option<(SoftBodyHandle, u32)> {
        let proxy = *self.entity2body.get(&entity)?;
        let handle = self.bodies.get(proxy)?.soft_body()?;
        let sb = self.soft_bodies.get(handle)?;
        let index = self
            .bodies
            .get(proxy)?
            .soft_cluster()
            .filter(|i| sb.cluster_proxy(*i) == Some(proxy))
            .or_else(|| {
                sb.live_clusters()
                    .find(|(_, cluster)| cluster.proxy() == proxy)
                    .map(|(i, _)| i)
            })?;
        Some((handle, index))
    }

    /// The map from entities to rigid-body handles.
    pub fn entity2body(&self) -> &HashMap<Entity, RigidBodyHandle> {
        &self.entity2body
    }

    /// Retrieve the Bevy entity the given Rapier rigid-body (identified by its handle) is attached.
    pub fn rigid_body_entity(&self, handle: RigidBodyHandle) -> Option<Entity> {
        self.bodies
            .get(handle)
            .map(|c| Entity::from_bits(c.user_data as u64))
    }

    /// This method makes sure that the rigid-body positions have been propagated to
    /// their attached colliders, without having to perform a simulation step.
    pub fn propagate_modified_body_positions_to_colliders(
        &self,
        colliders: &mut RapierContextColliders,
    ) {
        self.bodies
            .propagate_modified_body_positions_to_colliders(&mut colliders.colliders);
    }

    /// Computes the angle between the two bodies attached by the [`RevoluteJoint`] component (if any) referenced by the given `entity`.
    ///
    /// The angle is computed along the revolute joint’s principal axis.
    ///
    /// Parameter `entity` should have a [`ImpulseJoint`] component with a [`TypedJoint::RevoluteJoint`] variant as `data`.
    pub fn impulse_revolute_joint_angle(
        &self,
        joints: &RapierContextJoints,
        entity: Entity,
    ) -> Option<f32> {
        let joint_handle = joints.entity2impulse_joint().get(&entity)?;
        let impulse_joint = joints.impulse_joints.get(*joint_handle)?;
        let revolute_joint = impulse_joint.data.as_revolute()?;

        let rb1 = &self.bodies[impulse_joint.body1()];
        let rb2 = &self.bodies[impulse_joint.body2()];
        Some(revolute_joint.angle(rb1.rotation(), rb2.rotation()))
    }

    /// The Rapier rigid-body attached to the given `entity`, if any.
    fn body(&self, entity: Entity) -> Option<&rapier::dynamics::RigidBody> {
        self.bodies.get(*self.entity2body.get(&entity)?)
    }

    /// The linear velocity of the given world-space `point`, assuming it is rigidly attached to the
    /// rigid-body of `entity`.
    ///
    /// Unlike [`crate::dynamics::Velocity::linear_velocity_at_point`], this uses the center of mass
    /// computed by Rapier. Returns `None` if `entity` has no rigid-body.
    pub fn velocity_at_point(&self, entity: Entity, point: Vect) -> Option<Vect> {
        Some(self.body(entity)?.velocity_at_point(point))
    }

    /// Returns `true` if the rigid-body of `entity` has a non-zero linear or angular velocity.
    ///
    /// Returns `None` if `entity` has no rigid-body.
    pub fn is_moving(&self, entity: Entity) -> Option<bool> {
        Some(self.body(entity)?.is_moving())
    }

    /// Returns `true` if CCD is active for the rigid-body of `entity`, i.e., if it moved fast
    /// enough during the last step to justify a CCD run.
    ///
    /// Returns `None` if `entity` has no rigid-body.
    pub fn is_ccd_active(&self, entity: Entity) -> Option<bool> {
        Some(self.body(entity)?.is_ccd_active())
    }

    /// The next position of the rigid-body of `entity`.
    ///
    /// For kinematic rigid-bodies, this is the position set by the user for the next step. For
    /// other rigid-bodies, this value is currently unspecified. The returned transform has a unit
    /// scale. Returns `None` if `entity` has no rigid-body.
    pub fn next_position(&self, entity: Entity) -> Option<Transform> {
        Some(crate::utils::iso_to_transform(
            self.body(entity)?.next_position(),
        ))
    }

    /// Predicts the position of the rigid-body of `entity` after `dt` seconds, by integrating its
    /// current velocity and forces (including gravity).
    ///
    /// The returned transform has a unit scale. Returns `None` if `entity` has no rigid-body.
    pub fn predict_position_using_velocity_and_forces(
        &self,
        entity: Entity,
        dt: Real,
    ) -> Option<Transform> {
        Some(crate::utils::iso_to_transform(
            &self
                .body(entity)?
                .predict_position_using_velocity_and_forces(dt),
        ))
    }

    /// Predicts the position of the rigid-body of `entity` after `dt` seconds, by integrating its
    /// current velocity only (forces are ignored).
    ///
    /// The returned transform has a unit scale. Returns `None` if `entity` has no rigid-body.
    pub fn predict_position_using_velocity(&self, entity: Entity, dt: Real) -> Option<Transform> {
        Some(crate::utils::iso_to_transform(
            &self.body(entity)?.predict_position_using_velocity(dt),
        ))
    }

    /// The kinetic energy of the rigid-body of `entity`.
    ///
    /// Returns `None` if `entity` has no rigid-body.
    pub fn kinetic_energy(&self, entity: Entity) -> Option<Real> {
        Some(self.body(entity)?.kinetic_energy())
    }

    /// The gravitational potential energy of the rigid-body of `entity`.
    ///
    /// The position is projected back by half a timestep `dt` so the result is synchronized with
    /// [`Self::kinetic_energy`]. Returns `None` if `entity` has no rigid-body.
    pub fn gravitational_potential_energy(
        &self,
        entity: Entity,
        dt: Real,
        gravity: Vect,
    ) -> Option<Real> {
        Some(
            self.body(entity)?
                .gravitational_potential_energy(dt, gravity),
        )
    }

    /// The total user force applied to the rigid-body of `entity` (e.g. from its
    /// [`crate::dynamics::ExternalForce`] component).
    ///
    /// This is zero for non-dynamic rigid-bodies. Returns `None` if `entity` has no rigid-body.
    pub fn user_force(&self, entity: Entity) -> Option<Vect> {
        Some(self.body(entity)?.user_force())
    }

    /// The total user torque applied to the rigid-body of `entity` (e.g. from its
    /// [`crate::dynamics::ExternalForce`] component).
    ///
    /// This is zero for non-dynamic rigid-bodies. Returns `None` if `entity` has no rigid-body.
    #[cfg(feature = "dim2")]
    pub fn user_torque(&self, entity: Entity) -> Option<Real> {
        Some(self.body(entity)?.user_torque())
    }

    /// The total user torque applied to the rigid-body of `entity` (e.g. from its
    /// [`crate::dynamics::ExternalForce`] component).
    ///
    /// This is zero for non-dynamic rigid-bodies. Returns `None` if `entity` has no rigid-body.
    #[cfg(feature = "dim3")]
    pub fn user_torque(&self, entity: Entity) -> Option<Vect> {
        Some(self.body(entity)?.user_torque())
    }

    /// The dominance group actually used by the solver for the rigid-body of `entity`.
    ///
    /// This is `i8::MAX + 1` for non-dynamic rigid-bodies. Returns `None` if `entity` has no
    /// rigid-body.
    pub fn effective_dominance_group(&self, entity: Entity) -> Option<i16> {
        Some(self.body(entity)?.effective_dominance_group())
    }

    /// The world-space center of mass of the rigid-body of `entity`.
    ///
    /// Returns `None` if `entity` has no rigid-body.
    pub fn center_of_mass(&self, entity: Entity) -> Option<Vect> {
        Some(self.body(entity)?.center_of_mass())
    }

    /// The mass of the rigid-body of `entity` (including the contributions of its colliders).
    ///
    /// This is zero for rigid-bodies without mass. Returns `None` if `entity` has no rigid-body.
    pub fn mass(&self, entity: Entity) -> Option<Real> {
        Some(self.body(entity)?.mass())
    }
}

/// Teleports the kinematic position-based rigid-bodies to their next kinematic position.
///
/// This emulates, in collision-only mode, the motion the physics pipeline would apply.
fn apply_kinematic_targets(bodies: &mut RigidBodySet) {
    let targets: Vec<_> = bodies
        .iter()
        .filter(|(_, rb)| {
            rb.body_type() == RigidBodyType::KinematicPositionBased
                && rb.position() != rb.next_position()
        })
        .map(|(handle, rb)| (handle, *rb.next_position()))
        .collect();
    for (handle, pose) in targets {
        if let Some(rb) = bodies.get_mut(handle) {
            rb.set_position(pose, false);
        }
    }
}

/// A soft-body tear waiting for the `handle_soft_body_tears` system.
pub(crate) struct PendingSoftBodyTear {
    /// The Rapier tear event.
    pub raw: RapierSoftBodyTearEvent,
    /// The entities already spawned for the pieces split off the torn body (the pieces of `raw`
    /// but the first), or empty if they must be spawned by the system.
    pub piece_entities: Vec<Entity>,
}

/// Statistics about the simulation steps executed by the last call to
/// [`RapierContextSimulation::step_simulation`], i.e., during the last physics update.
///
/// The step count and time are measured by `bevy_rapier` and always available. The other
/// statistics are read from the [`PhysicsPipeline::counters`] after each step, and are only
/// measured while these counters are enabled (e.g. by the
/// [`RapierDiagnosticsPlugin`](crate::plugin::diagnostics::RapierDiagnosticsPlugin)). The other
/// timings also require the `profiler` feature. In [`SimulationMode::CollisionOnly`], only the
/// contact pairs are counted besides the steps.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct SimulationStepStats {
    /// The number of simulation steps (including substeps) executed.
    pub num_steps: usize,
    /// The total wall-clock time spent running the simulation steps, in milliseconds.
    pub step_time_ms: f64,
    /// Time spent on collision detection (broad-phase and narrow-phase), in milliseconds.
    pub collision_detection_time_ms: f64,
    /// Time spent in the broad-phase, in milliseconds.
    pub broad_phase_time_ms: f64,
    /// Time spent in the narrow-phase, in milliseconds.
    pub narrow_phase_time_ms: f64,
    /// Time spent building the simulation islands, in milliseconds.
    pub island_construction_time_ms: f64,
    /// Time spent in the constraints solver, in milliseconds.
    pub solver_time_ms: f64,
    /// Time spent on Continuous Collision Detection, in milliseconds.
    pub ccd_time_ms: f64,
    /// Time spent updating the rigid-bodies’ positions and velocities, in milliseconds.
    pub update_time_ms: f64,
    /// The number of CCD substeps executed (zero on the steps where the CCD didn't need to act).
    pub ccd_substeps: usize,
    /// The number of contact pairs tracked by the narrow-phase (touching or not) after the last
    /// step.
    pub num_contact_pairs: usize,
    /// The number of contact manifolds and impulse joints handed to the constraints solver,
    /// summed over the steps. The contacts and joints of sleeping bodies aren't counted.
    pub num_solver_constraints: usize,
    /// The number of contact points handed to the constraints solver, soft-body contacts
    /// included, summed over the steps.
    pub num_solver_contacts: usize,
}

impl SimulationStepStats {
    /// Adds the counters of the last step of `pipeline` to these statistics.
    fn accumulate_counters(&mut self, pipeline: &PhysicsPipeline) {
        let counters = &pipeline.counters;
        self.collision_detection_time_ms += counters.collision_detection_time_ms();
        self.broad_phase_time_ms += counters.broad_phase_time_ms();
        self.narrow_phase_time_ms += counters.narrow_phase_time_ms();
        self.island_construction_time_ms += counters.island_construction_time_ms();
        self.solver_time_ms += counters.solver_time_ms();
        self.ccd_time_ms += counters.ccd_time_ms();
        self.update_time_ms += counters.update_time_ms();
        self.ccd_substeps += counters.ccd.num_substeps;
        self.num_contact_pairs = counters.cd.ncontact_pairs;
        self.num_solver_constraints += counters.solver.nconstraints;
        self.num_solver_contacts += counters.solver.ncontacts;
    }
}

/// The Rapier context, containing parts of the state of the physics engine, specific to the simulation step.
///
/// This is the main driver for a rapier context, which will create other required components if needed.
///
/// Additionally to its required components, this component is also always paired with a [`RapierConfiguration`][crate::prelude::RapierConfiguration] component.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Component)]
#[require(
    RapierContextColliders,
    RapierRigidBodySet,
    RapierContextJoints,
    SimulationToRenderTime
)]
pub struct RapierContextSimulation {
    /// The island manager, which detects what object is sleeping
    /// (not moving much) to reduce computations.
    pub islands: IslandManager,
    /// The broad-phase, which detects potential contact pairs.
    pub broad_phase: DefaultBroadPhase,
    /// The narrow-phase, which computes contact points, tests intersections,
    /// and maintain the contact and intersection graphs.
    pub narrow_phase: NarrowPhase,
    /// The solver, which handles Continuous Collision Detection (CCD).
    pub ccd_solver: CCDSolver,
    /// The physics pipeline, which advance the simulation step by step.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub pipeline: PhysicsPipeline,
    /// The collision pipeline, which only runs collision detection when the context is in
    /// [`SimulationMode::CollisionOnly`].
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub collision_pipeline: CollisionPipeline,
    /// The integration parameters, controlling various low-level coefficient of the simulation.
    pub integration_parameters: IntegrationParameters,
    /// The user-provided event handler, see [`Self::set_event_handler`].
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) event_handler: Option<Box<dyn EventHandler + Send + Sync>>,
    // This maps the handles of colliders that have been deleted since the last
    // physics update, to the entity they was attached to.
    /// NOTE: this map is needed to handle despawning.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) deleted_colliders: HashMap<ColliderHandle, Entity>,

    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) collision_events_to_send: Vec<CollisionEvent>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) contact_force_events_to_send: Vec<ContactForceEvent>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) character_collisions_collector: Vec<rapier::control::CharacterCollision>,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) step_stats: SimulationStepStats,
    /// Rigid-body entities quarantined since the last quarantine event was sent.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) quarantined_bodies: Vec<Entity>,
    /// Collider entities quarantined since the last quarantine event was sent.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) quarantined_colliders: Vec<Entity>,
    /// Soft-body entities quarantined since the last quarantine event was sent.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) quarantined_soft_bodies: Vec<Entity>,
    /// The number of threads last requested through [`Self::set_num_threads`].
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) requested_num_threads: Option<usize>,
    /// Soft-body tears not yet turned into [`SoftBodyTearEvent`] messages (their piece entities
    /// are spawned by the `handle_soft_body_tears` system).
    ///
    /// [`SoftBodyTearEvent`]: crate::pipeline::SoftBodyTearEvent
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) pending_soft_body_tears: Vec<PendingSoftBodyTear>,
}

impl Default for RapierContextSimulation {
    fn default() -> Self {
        Self {
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            ccd_solver: CCDSolver::new(),
            pipeline: PhysicsPipeline::new(),
            collision_pipeline: CollisionPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            event_handler: None,
            deleted_colliders: HashMap::default(),
            collision_events_to_send: Vec::new(),
            contact_force_events_to_send: Vec::new(),
            character_collisions_collector: Vec::new(),
            step_stats: SimulationStepStats::default(),
            quarantined_bodies: Vec::new(),
            quarantined_colliders: Vec::new(),
            quarantined_soft_bodies: Vec::new(),
            requested_num_threads: None,
            pending_soft_body_tears: Vec::new(),
        }
    }
}

impl RapierContextSimulation {
    /// Creates a simulation context whose broad-phase uses the given BVH optimization strategy.
    ///
    /// This is what [`RapierContextInitialization::InitializeDefaultRapierContext`] uses for the
    /// default context.
    ///
    /// [`RapierContextInitialization::InitializeDefaultRapierContext`]: crate::plugin::RapierContextInitialization::InitializeDefaultRapierContext
    pub fn with_broad_phase_optimization_strategy(
        strategy: BroadPhaseOptimizationStrategy,
    ) -> Self {
        Self {
            broad_phase: DefaultBroadPhase::with_optimization_strategy(strategy.into()),
            ..Self::default()
        }
    }

    /// Statistics about the simulation steps executed during the last physics update.
    pub fn step_stats(&self) -> &SimulationStepStats {
        &self.step_stats
    }

    /// Sets the number of threads of the thread pool dedicated to this context’s simulation, or
    /// removes that pool if `num_threads` is `None`.
    ///
    /// The pool is only rebuilt if `num_threads` differs from the value of the previous call. This
    /// is called automatically with [`RapierConfiguration::num_threads`] before each simulation
    /// step, and does nothing unless the `parallel` feature is enabled (and `unsync-callbacks`
    /// is not).
    ///
    /// [`RapierConfiguration::num_threads`]: crate::plugin::RapierConfiguration::num_threads
    pub fn set_num_threads(&mut self, num_threads: Option<usize>) {
        if self.requested_num_threads == num_threads {
            return;
        }
        self.requested_num_threads = num_threads;

        #[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
        match num_threads {
            Some(n) => {
                if let Err(err) = self.pipeline.configure_thread_pool(n) {
                    log::error!("Failed to create a Rapier thread pool with {n} threads: {err}");
                }
            }
            None => self.pipeline.clear_thread_pool(),
        }
    }

    /// Creates a simulation context whose narrow-phase uses a custom query dispatcher.
    ///
    /// The query dispatcher computes the contact manifolds and intersections between pairs of
    /// shapes, which makes it possible to support custom shapes or override the contact
    /// generation of existing ones. Spawn it with
    /// [`RapierContextInitialization::NoAutomaticRapierContext`](crate::plugin::RapierContextInitialization::NoAutomaticRapierContext),
    /// or see [`Self::set_query_dispatcher`] for the default context.
    pub fn with_query_dispatcher<D>(dispatcher: D) -> Self
    where
        D: 'static + PersistentQueryDispatcher<ContactManifoldData, ContactData>,
    {
        Self {
            narrow_phase: NarrowPhase::with_query_dispatcher(dispatcher),
            ..Self::default()
        }
    }

    /// Replaces the narrow-phase with an empty one using the given custom query dispatcher.
    ///
    /// This discards every contact and intersection pair of the current narrow-phase, so it
    /// must be called before any collider is added to this context (for example in a
    /// `Startup` system, since colliders are inserted during [`PhysicsSet::SyncBackend`]).
    ///
    /// [`PhysicsSet::SyncBackend`]: crate::plugin::PhysicsSet::SyncBackend
    pub fn set_query_dispatcher<D>(&mut self, dispatcher: D)
    where
        D: 'static + PersistentQueryDispatcher<ContactManifoldData, ContactData>,
    {
        self.narrow_phase = NarrowPhase::with_query_dispatcher(dispatcher);
    }

    /// The query dispatcher used by the narrow-phase to compute contacts and intersections.
    pub fn query_dispatcher(
        &self,
    ) -> &dyn PersistentQueryDispatcher<ContactManifoldData, ContactData> {
        self.narrow_phase.query_dispatcher()
    }

    /// Installs a Rapier [`EventHandler`] called by the physics pipeline for every collision,
    /// contact force and soft-body tear event.
    ///
    /// It is called in addition to (not instead of) the forwarding of events as
    /// [`CollisionEvent`] and [`ContactForceEvent`] Bevy messages, and replaces the previously
    /// installed handler, if any. The handler receives raw Rapier handles: the entity of a
    /// collider can be retrieved with [`RapierContextColliders::entity_from_collider`]. Note that
    /// the colliders of an event flagged with `CollisionEventFlags::REMOVED` may no longer be
    /// part of the collider set.
    pub fn set_event_handler(&mut self, handler: impl EventHandler + Send + Sync + 'static) {
        self.event_handler = Some(Box::new(handler));
    }

    /// Removes the event handler installed with [`Self::set_event_handler`] and returns it.
    pub fn remove_event_handler(&mut self) -> Option<Box<dyn EventHandler + Send + Sync>> {
        self.event_handler.take()
    }

    /// The event handler installed with [`Self::set_event_handler`], if any.
    pub fn event_handler(&self) -> Option<&(dyn EventHandler + Send + Sync)> {
        self.event_handler.as_deref()
    }

    /// Removes the soft body `handle` (with its colliders, cluster proxies and the joints
    /// attached to them), and unmaps the entities standing for them.
    pub(crate) fn remove_soft_body(
        &mut self,
        colliders: &mut RapierContextColliders,
        joints: &mut RapierContextJoints,
        rigidbody_set: &mut RapierRigidBodySet,
        handle: SoftBodyHandle,
    ) -> Option<RapierSoftBody> {
        let sb = rigidbody_set.soft_bodies.get(handle)?;
        for (_, cluster) in sb.live_clusters() {
            let proxy = cluster.proxy();
            let Some(rb) = rigidbody_set.bodies.get(proxy) else {
                continue;
            };
            if let Some(entity) = Entity::try_from_bits(rb.user_data as u64) {
                if rigidbody_set.entity2body.get(&entity) == Some(&proxy) {
                    rigidbody_set.entity2body.remove(&entity);
                    rigidbody_set.last_body_transform_set.remove(&proxy);
                }
            }
            for co_handle in rb.colliders() {
                let Some(co) = colliders.colliders.get(*co_handle) else {
                    continue;
                };
                let Some(entity) = Entity::try_from_bits(co.user_data as u64) else {
                    continue;
                };
                // Removed colliders may still be reported by the next step's events.
                self.deleted_colliders.insert(*co_handle, entity);
                if colliders.entity2collider.get(&entity) == Some(co_handle) {
                    colliders.entity2collider.remove(&entity);
                }
            }
            for (_, _, joint_handle, joint) in joints.impulse_joints.attached_joints(proxy) {
                let entity = RapierContextJoints::entity_from_impulse_joint(joint);
                if joints.entity2impulse_joint.get(&entity) == Some(&joint_handle) {
                    joints.entity2impulse_joint.remove(&entity);
                }
            }
        }
        rigidbody_set.entity2soft_body.retain(|_, h| *h != handle);
        rigidbody_set.soft_bodies.remove(
            handle,
            &mut self.islands,
            &mut rigidbody_set.bodies,
            &mut colliders.colliders,
            &mut joints.impulse_joints,
            &mut joints.multibody_joints,
        )
    }

    /// Tears the soft body of `entity` right away along the given edges and through the given
    /// cells (see [`SoftBodySet::tear`]).
    ///
    /// The entities of the pieces split off the soft body are spawned with `commands` and
    /// returned right away; they get their components and a
    /// [`SoftBodyTearEvent`](crate::pipeline::SoftBodyTearEvent) message is sent during the next
    /// [`PhysicsSet::Writeback`](crate::plugin::PhysicsSet::Writeback) (see
    /// [`SoftBodyTearResult`]). Returns `None` if `entity` has no soft body or if nothing changed.
    #[allow(clippy::too_many_arguments)]
    pub fn tear_soft_body(
        &mut self,
        colliders: &mut RapierContextColliders,
        joints: &mut RapierContextJoints,
        rigidbody_set: &mut RapierRigidBodySet,
        commands: &mut Commands,
        entity: Entity,
        edges: &[u32],
        cells: &[u32],
    ) -> Option<SoftBodyTearResult> {
        let handle = *rigidbody_set.entity2soft_body.get(&entity)?;
        let event = rigidbody_set.soft_bodies.tear(
            handle,
            edges,
            cells,
            &mut self.islands,
            &mut rigidbody_set.bodies,
            &mut colliders.colliders,
            &mut joints.impulse_joints,
            &mut joints.multibody_joints,
        )?;
        Some(self.map_manual_tear(colliders, rigidbody_set, commands, entity, event))
    }

    /// Cuts the soft body of `entity` right away along a blade: a world-space segment in 2D, a
    /// world-space triangle in 3D (see [`SoftBodySet::cut`]).
    ///
    /// The entities of the pieces split off the soft body are spawned with `commands` and
    /// returned right away; they get their components and a
    /// [`SoftBodyTearEvent`](crate::pipeline::SoftBodyTearEvent) message is sent during the next
    /// [`PhysicsSet::Writeback`](crate::plugin::PhysicsSet::Writeback) (see
    /// [`SoftBodyTearResult`]). Returns `None` if `entity` has no soft body or if nothing changed.
    pub fn cut_soft_body(
        &mut self,
        colliders: &mut RapierContextColliders,
        joints: &mut RapierContextJoints,
        rigidbody_set: &mut RapierRigidBodySet,
        commands: &mut Commands,
        entity: Entity,
        blade: &[Vect; rapier::math::DIM],
    ) -> Option<SoftBodyTearResult> {
        let handle = *rigidbody_set.entity2soft_body.get(&entity)?;
        let event = rigidbody_set.soft_bodies.cut(
            handle,
            blade,
            &mut self.islands,
            &mut rigidbody_set.bodies,
            &mut colliders.colliders,
            &mut joints.impulse_joints,
            &mut joints.multibody_joints,
        )?;
        Some(self.map_manual_tear(colliders, rigidbody_set, commands, entity, event))
    }

    /// Spawns the entities of the pieces of a tear of the soft body of `entity`, maps them to their
    /// soft bodies, and queues the tear for the `handle_soft_body_tears` system.
    fn map_manual_tear(
        &mut self,
        colliders: &mut RapierContextColliders,
        rigidbody_set: &mut RapierRigidBodySet,
        commands: &mut Commands,
        entity: Entity,
        event: RapierSoftBodyTearEvent,
    ) -> SoftBodyTearResult {
        let mut pieces = Vec::with_capacity(event.pieces.len());
        for (k, piece) in event.pieces.iter().enumerate() {
            let piece_entity = if k == 0 {
                entity
            } else {
                let piece_entity = commands.spawn_empty().id();
                rigidbody_set.map_soft_body_piece(colliders, piece.soft_body, piece_entity);
                piece_entity
            };
            rigidbody_set.map_soft_body_collision_mesh(colliders, piece.soft_body, piece_entity);
            rigidbody_set.map_soft_body_whole_proxy(piece_entity, piece.soft_body);
            pieces.push(piece_entity);
        }
        if event.pieces.is_empty() {
            rigidbody_set.map_soft_body_collision_mesh(colliders, event.soft_body, entity);
            rigidbody_set.map_soft_body_whole_proxy(entity, event.soft_body);
        }
        self.pending_soft_body_tears.push(PendingSoftBodyTear {
            raw: event.clone(),
            piece_entities: pieces.get(1..).unwrap_or_default().to_vec(),
        });
        SoftBodyTearResult { pieces, raw: event }
    }

    /// Advance the simulation, based on the given timestep mode.
    ///
    /// With [`SimulationMode::CollisionOnly`], only collision detection runs at each step (see
    /// [`SimulationMode`]).
    ///
    /// Steps with a zero (or negative) dt, like the first frame with [`TimestepMode::Variable`],
    /// are skipped entirely.
    #[allow(clippy::too_many_arguments)]
    pub fn step_simulation(
        &mut self,
        colliders: &mut RapierContextColliders,
        joints: &mut RapierContextJoints,
        rigidbody_set: &mut RapierRigidBodySet,
        gravity: Vect,
        simulation_mode: SimulationMode,
        timestep_mode: TimestepMode,
        events: Option<(
            &MessageWriter<CollisionEvent>,
            &MessageWriter<ContactForceEvent>,
        )>,
        hooks: &dyn PhysicsHooks,
        time: &Time,
        sim_to_render_time: &mut SimulationToRenderTime,
        mut interpolation_query: Option<
            &mut Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
        >,
    ) {
        self.step_stats = SimulationStepStats::default();

        // Temporarily move the event handling state out of `self` so it can be borrowed while
        // `self` is borrowed mutably by each step.
        let user_event_handler = self.event_handler.take();
        let deleted_colliders = std::mem::take(&mut self.deleted_colliders);

        let event_queue = if events.is_some() {
            Some(EventQueue {
                deleted_colliders: &deleted_colliders,
                collision_events: RwLock::new(Vec::new()),
                contact_force_events: RwLock::new(Vec::new()),
            })
        } else {
            None
        };

        let soft_body_tears = RwLock::new(Vec::new());
        let fan_out = EventHandlerFanOut {
            queue: event_queue.as_ref(),
            user: user_event_handler.as_deref(),
            soft_body_tears: &soft_body_tears,
        };
        let event_handler = &fan_out as &dyn EventHandler;

        let mut executed_steps = 0;
        match timestep_mode {
            TimestepMode::Interpolated {
                dt,
                time_scale,
                substeps,
            } => {
                self.integration_parameters.dt = dt;

                sim_to_render_time.diff += time.delta_secs();

                while sim_to_render_time.diff > 0.0 {
                    // NOTE: in this comparison we do the same computations we
                    // will do for the next `while` iteration test, to make sure we
                    // don't get bit by potential float inaccuracy.
                    if sim_to_render_time.diff - dt <= 0.0 {
                        if let Some(interpolation_query) = interpolation_query.as_mut() {
                            // This is the last simulation step to be executed in the loop
                            // Update the previous state transforms
                            for (handle, mut interpolation) in interpolation_query.iter_mut() {
                                if let Some(body) = rigidbody_set.bodies.get(handle.0) {
                                    interpolation.start = Some(*body.position());
                                    interpolation.end = None;
                                }
                            }
                        }
                    }

                    let mut substep_integration_parameters = self.integration_parameters;
                    substep_integration_parameters.dt = dt / (substeps as Real) * time_scale;

                    // A zero `time_scale` would run zero-length steps; skip them.
                    let num_substeps = if substep_integration_parameters.dt > 0.0 {
                        substeps
                    } else {
                        0
                    };

                    for _ in 0..num_substeps {
                        self.step_once(
                            simulation_mode,
                            gravity,
                            &substep_integration_parameters,
                            colliders,
                            joints,
                            rigidbody_set,
                            hooks,
                            event_handler,
                        );
                        executed_steps += 1;
                    }

                    sim_to_render_time.diff -= dt;
                }
            }
            TimestepMode::Variable {
                max_dt,
                time_scale,
                substeps,
            } => {
                self.integration_parameters.dt = (time.delta_secs() * time_scale).min(max_dt);

                let mut substep_integration_parameters = self.integration_parameters;
                substep_integration_parameters.dt /= substeps as Real;

                // The first frame (or a zero `time_scale`) has a zero dt: skip the step, which
                // would only run a useless collision detection pass.
                let num_substeps = if substep_integration_parameters.dt > 0.0 {
                    substeps
                } else {
                    0
                };

                for _ in 0..num_substeps {
                    self.step_once(
                        simulation_mode,
                        gravity,
                        &substep_integration_parameters,
                        colliders,
                        joints,
                        rigidbody_set,
                        hooks,
                        event_handler,
                    );
                    executed_steps += 1;
                }
            }
            TimestepMode::Fixed { dt, substeps } => {
                self.integration_parameters.dt = dt;

                let mut substep_integration_parameters = self.integration_parameters;
                substep_integration_parameters.dt = dt / (substeps as Real);

                let num_substeps = if substep_integration_parameters.dt > 0.0 {
                    substeps
                } else {
                    0
                };

                for _ in 0..num_substeps {
                    self.step_once(
                        simulation_mode,
                        gravity,
                        &substep_integration_parameters,
                        colliders,
                        joints,
                        rigidbody_set,
                        hooks,
                        event_handler,
                    );
                    executed_steps += 1;
                }
            }
        }
        if let Some(mut event_queue) = event_queue {
            // NOTE: event_queue and its inner locks are only accessed from
            // within the steps executed above, so we can unwrap here safely.
            self.collision_events_to_send =
                std::mem::take(event_queue.collision_events.get_mut().unwrap());
            self.contact_force_events_to_send =
                std::mem::take(event_queue.contact_force_events.get_mut().unwrap());
        }

        // NOTE: the lock is only accessed from within the steps executed above.
        self.pending_soft_body_tears.extend(
            soft_body_tears
                .into_inner()
                .unwrap_or_default()
                .into_iter()
                .map(|raw| PendingSoftBodyTear {
                    raw,
                    piece_entities: vec![],
                }),
        );

        self.event_handler = user_event_handler;
        if executed_steps == 0 {
            self.deleted_colliders = deleted_colliders;
            // No step ran (e.g. zero dt): still move colliders attached to bodies the user moved,
            // like when the physics pipeline is inactive.
            rigidbody_set.propagate_modified_body_positions_to_colliders(colliders);
        }
    }

    /// Executes a single simulation step (or collision-detection step) and updates the step
    /// statistics and quarantine reports.
    #[allow(clippy::too_many_arguments)]
    fn step_once(
        &mut self,
        simulation_mode: SimulationMode,
        gravity: Vect,
        integration_parameters: &IntegrationParameters,
        colliders: &mut RapierContextColliders,
        joints: &mut RapierContextJoints,
        rigidbody_set: &mut RapierRigidBodySet,
        hooks: &dyn PhysicsHooks,
        event_handler: &dyn EventHandler,
    ) {
        let start = bevy::platform::time::Instant::now();

        match simulation_mode {
            SimulationMode::Full => {
                self.pipeline.step(
                    gravity,
                    integration_parameters,
                    &mut self.islands,
                    &mut self.broad_phase,
                    &mut self.narrow_phase,
                    &mut rigidbody_set.bodies,
                    &mut colliders.colliders,
                    &mut joints.impulse_joints,
                    &mut joints.multibody_joints,
                    &mut rigidbody_set.soft_bodies,
                    &mut self.ccd_solver,
                    hooks,
                    event_handler,
                );
            }
            SimulationMode::CollisionOnly => {
                apply_kinematic_targets(&mut rigidbody_set.bodies);
                self.collision_pipeline.step(
                    integration_parameters.prediction_distance(),
                    &mut self.islands,
                    &mut self.broad_phase,
                    &mut self.narrow_phase,
                    &mut rigidbody_set.bodies,
                    &mut colliders.colliders,
                    hooks,
                    event_handler,
                );
            }
        }

        self.step_stats.num_steps += 1;
        self.step_stats.step_time_ms += start.elapsed().as_secs_f64() * 1000.0;

        // Rapier doesn't reset its counters while they are disabled, so they would be stale.
        if self.pipeline.counters.enabled() {
            match simulation_mode {
                SimulationMode::Full => self.step_stats.accumulate_counters(&self.pipeline),
                // The collision pipeline has no counters.
                SimulationMode::CollisionOnly => {
                    self.step_stats.num_contact_pairs = self.narrow_phase.contact_pairs().count();
                }
            }
        }

        if simulation_mode == SimulationMode::Full {
            let quarantine = self.pipeline.quarantine();
            if !quarantine.is_empty() {
                self.quarantined_bodies.extend(
                    quarantine
                        .bodies()
                        .iter()
                        .filter_map(|h| rigidbody_set.rigid_body_entity(*h)),
                );
                self.quarantined_colliders.extend(
                    quarantine
                        .colliders()
                        .iter()
                        .filter_map(|h| colliders.collider_entity(*h)),
                );
                self.quarantined_soft_bodies.extend(
                    quarantine
                        .soft_bodies()
                        .iter()
                        .filter_map(|h| rigidbody_set.soft_body_entity(*h)),
                );
            }
        }
    }

    /// Sends a [`PhysicsQuarantineEvent`] (and logs a warning) if some rigid-bodies, colliders or
    /// soft bodies were quarantined by the simulation steps executed since the last call.
    ///
    /// `context` is the entity of this context. This is called automatically by the
    /// [`step_simulation`](crate::plugin::systems::step_simulation) system.
    pub fn send_quarantine_events(
        &mut self,
        context: Entity,
        quarantine_event_writer: &mut MessageWriter<PhysicsQuarantineEvent>,
    ) {
        if self.quarantined_bodies.is_empty()
            && self.quarantined_colliders.is_empty()
            && self.quarantined_soft_bodies.is_empty()
        {
            return;
        }

        let event = PhysicsQuarantineEvent {
            context,
            bodies: std::mem::take(&mut self.quarantined_bodies),
            colliders: std::mem::take(&mut self.quarantined_colliders),
            soft_bodies: std::mem::take(&mut self.quarantined_soft_bodies),
        };
        log::warn!(
            "Rapier detected non-finite (NaN or infinite) state in context {context}; the \
            following rigid-bodies were rolled back and disabled: {:?}; the following colliders \
            were disabled: {:?}; the following soft bodies were disabled: {:?}.",
            event.bodies,
            event.colliders,
            event.soft_bodies
        );
        quarantine_event_writer.write(event);
    }

    /// Generates bevy events for any physics interactions that have happened
    /// that are stored in the events list
    pub fn send_bevy_events(
        &mut self,
        collision_event_writer: &mut MessageWriter<CollisionEvent>,
        contact_force_event_writer: &mut MessageWriter<ContactForceEvent>,
    ) {
        for collision_event in self.collision_events_to_send.drain(..) {
            collision_event_writer.write(collision_event);
        }
        for contact_force_event in self.contact_force_events_to_send.drain(..) {
            contact_force_event_writer.write(contact_force_event);
        }
    }

    /// Attempts to move shape, optionally sliding or climbing obstacles.
    ///
    /// The obstacles are the colliders of this context matching `filter`, detected with this
    /// context's [query dispatcher](Self::query_dispatcher).
    ///
    /// # Parameters
    /// * `colliders`, `rigidbody_set`: the collider and rigid-body sets of this context.
    /// * `movement`: the translational movement to apply.
    /// * `shape`: the shape to move.
    /// * `shape_translation`: the initial position of the shape.
    /// * `shape_rotation`: the rotation of the shape.
    /// * `shape_mass`: the mass of the shape to be considered by the impulse calculation if
    ///   `MoveShapeOptions::apply_impulse_to_dynamic_bodies` is set to true.
    /// * `options`: configures the behavior of the automatic sliding and climbing.
    /// * `filter`: selects the colliders the shape can collide with (exclude the collider or
    ///   rigid-body of the moved shape itself).
    /// * `events`: callback run on each obstacle hit by the shape on its path, once the
    ///   movement is computed.
    #[allow(clippy::too_many_arguments)]
    pub fn move_shape(
        &mut self,
        colliders: &mut RapierContextColliders,
        rigidbody_set: &mut RapierRigidBodySet,
        movement: Vect,
        shape: &(impl AsShape + ?Sized),
        shape_translation: Vect,
        shape_rotation: Rot,
        shape_mass: Real,
        options: &MoveShapeOptions,
        filter: QueryFilter,
        mut events: impl FnMut(CharacterCollision),
    ) -> MoveShapeOutput {
        let shape = shape.as_shape();
        assert!(
            options.up.length_squared() > 0.0,
            "The up vector must be non-zero."
        );
        let up = options.up.normalize();
        let autostep = options.autostep.map(|autostep| CharacterAutostep {
            max_height: autostep.max_height,
            min_width: autostep.min_width,
            include_dynamic_bodies: autostep.include_dynamic_bodies,
        });
        let controller = rapier::control::KinematicCharacterController {
            up,
            offset: options.offset,
            slide: options.slide,
            autostep,
            max_slope_climb_angle: options.max_slope_climb_angle,
            min_slope_slide_angle: options.min_slope_slide_angle,
            snap_to_ground: options.snap_to_ground,
            normal_nudge_factor: options.normal_nudge_factor,
        };

        let dt = self.integration_parameters.dt;
        let collisions = &mut self.character_collisions_collector;
        collisions.clear();

        let result = RapierQueryPipelineMut::new_scoped(
            &self.broad_phase,
            colliders,
            rigidbody_set,
            &filter,
            self.narrow_phase.query_dispatcher(),
            |mut query_pipeline| {
                let result = controller.move_shape(
                    dt,
                    &query_pipeline.query_pipeline.as_ref(),
                    shape,
                    &crate::utils::pose_from(shape_translation, shape_rotation),
                    movement,
                    |c| collisions.push(c),
                );

                if options.apply_impulse_to_dynamic_bodies {
                    controller.solve_character_collision_impulses(
                        dt,
                        &mut query_pipeline.query_pipeline,
                        shape,
                        shape_mass,
                        collisions.iter(),
                    );
                }

                result
            },
        );

        for collision in collisions.iter() {
            if let Some(collision) =
                CharacterCollision::from_raw_with_set(&colliders.colliders, collision, true)
            {
                events(collision);
            }
        }

        MoveShapeOutput {
            effective_translation: result.translation,
            grounded: result.grounded,
            is_sliding_down_slope: result.is_sliding_down_slope,
        }
    }
}

/// Deserialization helpers rebuilding the entity maps skipped during serialization.
#[cfg(feature = "serde-serialize")]
mod serde_shadows {
    use super::*;
    use rapier::prelude::{ImpulseJointSet, MultibodyJointSet};

    #[derive(Deserialize)]
    pub(super) struct RapierContextCollidersData {
        colliders: ColliderSet,
    }

    impl From<RapierContextCollidersData> for RapierContextColliders {
        fn from(data: RapierContextCollidersData) -> Self {
            let mut result = Self {
                colliders: data.colliders,
                entity2collider: HashMap::default(),
            };
            result.rebuild_entity_maps();
            result
        }
    }

    #[derive(Deserialize)]
    pub(super) struct RapierContextJointsData {
        impulse_joints: ImpulseJointSet,
        multibody_joints: MultibodyJointSet,
    }

    impl From<RapierContextJointsData> for RapierContextJoints {
        fn from(data: RapierContextJointsData) -> Self {
            let mut result = Self {
                impulse_joints: data.impulse_joints,
                multibody_joints: data.multibody_joints,
                entity2impulse_joint: HashMap::default(),
                entity2multibody_joint: HashMap::default(),
            };
            result.rebuild_entity_maps();
            result
        }
    }

    #[derive(Deserialize)]
    pub(super) struct RapierRigidBodySetData {
        bodies: RigidBodySet,
        soft_bodies: SoftBodySet,
    }

    impl From<RapierRigidBodySetData> for RapierRigidBodySet {
        fn from(data: RapierRigidBodySetData) -> Self {
            let mut result = Self {
                bodies: data.bodies,
                soft_bodies: data.soft_bodies,
                entity2body: HashMap::default(),
                entity2soft_body: HashMap::default(),
                last_body_transform_set: HashMap::default(),
            };
            result.rebuild_entity_maps();
            result
        }
    }
}
