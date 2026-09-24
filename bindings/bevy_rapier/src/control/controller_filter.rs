use bevy::ecs::entity::EntityHashSet;
use bevy::prelude::{Component, Entity, Reflect, ReflectComponent, ReflectDefault};
use rapier::dynamics::RigidBodyHandle;
use rapier::geometry::Collider as RapierCollider;
use std::fmt;
use std::sync::Arc;

use crate::plugin::context::{RapierContextColliders, RapierRigidBodySet};

/// Marker component excluding a collider from the environment seen by every controller (the
/// [`KinematicCharacterController`]s, and the wheels of the `RayCastVehicleController`s in 3D).
///
/// Insert it on a collider entity to ignore that collider, or on a rigid-body entity to ignore
/// all its colliders. Unlike [`ControllerFilterPredicate`], it can be inserted and removed by
/// systems reading any ECS data (e.g. to let characters walk through the doors they own).
///
/// [`KinematicCharacterController`]: crate::control::KinematicCharacterController
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct ControllerIgnored;

/// A user-defined predicate deciding which colliders are seen by a controller.
///
/// The predicate is given the entity and Rapier collider of each candidate collider and must
/// return `false` to exclude it. Since components cannot hold borrowed closures, the closure is
/// reference-counted, and it can't read ECS data: see [`ControllerIgnored`] for exclusions
/// computed by systems.
#[derive(Clone)]
#[allow(clippy::type_complexity)]
pub struct ControllerFilterPredicate(
    pub Arc<dyn Fn(Entity, &RapierCollider) -> bool + Send + Sync>,
);

impl ControllerFilterPredicate {
    /// Wraps the given closure into a predicate usable by the controllers.
    pub fn new(
        predicate: impl Fn(Entity, &RapierCollider) -> bool + Send + Sync + 'static,
    ) -> Self {
        Self(Arc::new(predicate))
    }

    /// Evaluates the predicate for the given collider attached to `entity`.
    pub fn test(&self, entity: Entity, collider: &RapierCollider) -> bool {
        (self.0)(entity, collider)
    }
}

impl fmt::Debug for ControllerFilterPredicate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("ControllerFilterPredicate(..)")
    }
}

/// Entity-based collider exclusions of a controller, resolved against the Rapier sets.
pub(crate) struct ControllerExclusions<'a> {
    colliders: &'a EntityHashSet,
    /// The entities with a [`ControllerIgnored`] component.
    ignored: &'a EntityHashSet,
    rigid_bodies: Vec<RigidBodyHandle>,
    predicate: Option<&'a ControllerFilterPredicate>,
}

impl<'a> ControllerExclusions<'a> {
    /// Resolves the excluded rigid-body entities (and the rigid-bodies among the `ignored`
    /// entities) into handles; entities without a rigid-body are ignored.
    pub fn new(
        colliders: &'a EntityHashSet,
        rigid_bodies: &EntityHashSet,
        ignored: &'a EntityHashSet,
        predicate: Option<&'a ControllerFilterPredicate>,
        rigidbody_set: &RapierRigidBodySet,
    ) -> Self {
        let rigid_bodies = rigid_bodies
            .iter()
            .chain(ignored.iter())
            .filter_map(|e| rigidbody_set.entity2body.get(e).copied())
            .collect();
        Self {
            colliders,
            ignored,
            rigid_bodies,
            predicate,
        }
    }

    /// Returns `true` if no collider can be excluded by `self`.
    pub fn is_empty(&self) -> bool {
        self.colliders.is_empty()
            && self.ignored.is_empty()
            && self.rigid_bodies.is_empty()
            && self.predicate.is_none()
    }

    /// Returns `true` if the collider must be seen by the controller.
    pub fn test(&self, collider: &RapierCollider) -> bool {
        let entity = RapierContextColliders::entity_from_collider(collider);
        !self.colliders.contains(&entity)
            && !self.ignored.contains(&entity)
            && collider
                .parent()
                .is_none_or(|parent| !self.rigid_bodies.contains(&parent))
            && self.predicate.is_none_or(|p| p.test(entity, collider))
    }
}
