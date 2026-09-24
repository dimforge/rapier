use bevy::prelude::Entity;

pub use rapier::pipeline::QueryFilterFlags;

use crate::geometry::CollisionGroups;
use rapier::geometry::Collider as RapierCollider;

/// A filter that describes what collider should be included or excluded from a scene query.
#[derive(Copy, Clone, Default)]
pub struct QueryFilter<'a> {
    /// Flags indicating what particular type of colliders should be excluded.
    pub flags: QueryFilterFlags,
    /// If set, only colliders with collision groups compatible with this one will
    /// be included in the scene query.
    ///
    /// The [`CollisionGroups::test_mode`] of both groups is taken into account.
    pub groups: Option<CollisionGroups>,
    /// If set, the collider attached to that entity will be excluded from the query.
    pub exclude_collider: Option<Entity>,
    /// If set, any collider attached to the rigid-body attached to that entity
    /// will be excluded from the query.
    pub exclude_rigid_body: Option<Entity>,
    /// If set, any collider for which this closure returns false will be excluded from the query.
    ///
    /// The closure is given the collider’s entity and its Rapier collider, which gives access to
    /// its shape, position, parent, etc.
    #[allow(clippy::type_complexity)]
    pub predicate: Option<&'a dyn Fn(Entity, &RapierCollider) -> bool>,
}

impl From<QueryFilterFlags> for QueryFilter<'_> {
    fn from(flags: QueryFilterFlags) -> Self {
        Self {
            flags,
            ..QueryFilter::default()
        }
    }
}

impl From<CollisionGroups> for QueryFilter<'_> {
    fn from(groups: CollisionGroups) -> Self {
        Self {
            groups: Some(groups),
            ..QueryFilter::default()
        }
    }
}

impl<'a> QueryFilter<'a> {
    /// A query filter that doesn’t exclude any collider.
    pub fn new() -> Self {
        Self::default()
    }

    /// Exclude from the query any collider attached to a fixed rigid-body and colliders with no rigid-body attached.
    pub fn exclude_fixed() -> Self {
        QueryFilterFlags::EXCLUDE_FIXED.into()
    }

    /// Exclude from the query any collider attached to a kinematic rigid-body.
    pub fn exclude_kinematic() -> Self {
        QueryFilterFlags::EXCLUDE_KINEMATIC.into()
    }

    /// Exclude from the query any collider attached to a dynamic rigid-body.
    pub fn exclude_dynamic() -> Self {
        QueryFilterFlags::EXCLUDE_DYNAMIC.into()
    }

    /// Excludes all colliders not attached to a dynamic rigid-body.
    pub fn only_dynamic() -> Self {
        QueryFilterFlags::ONLY_DYNAMIC.into()
    }

    /// Excludes all colliders not attached to a kinematic rigid-body.
    pub fn only_kinematic() -> Self {
        QueryFilterFlags::ONLY_KINEMATIC.into()
    }

    /// Exclude all colliders attached to a non-fixed rigid-body
    /// (this will not exclude colliders not attached to any rigid-body).
    pub fn only_fixed() -> Self {
        QueryFilterFlags::ONLY_FIXED.into()
    }

    /// Exclude from the query any collider that is a sensor.
    pub fn exclude_sensors(mut self) -> Self {
        self.flags |= QueryFilterFlags::EXCLUDE_SENSORS;
        self
    }

    /// Exclude from the query any collider that is not a sensor.
    pub fn exclude_solids(mut self) -> Self {
        self.flags |= QueryFilterFlags::EXCLUDE_SOLIDS;
        self
    }

    /// Only colliders with collision groups compatible with this one will
    /// be included in the scene query.
    ///
    /// The [`CollisionGroups::test_mode`] of both groups is taken into account.
    pub fn groups(mut self, groups: CollisionGroups) -> Self {
        self.groups = Some(groups);
        self
    }

    /// Set the collider that will be excluded from the scene query.
    pub fn exclude_collider(mut self, collider: Entity) -> Self {
        self.exclude_collider = Some(collider);
        self
    }

    /// Set the rigid-body that will be excluded from the scene query.
    pub fn exclude_rigid_body(mut self, rigid_body: Entity) -> Self {
        self.exclude_rigid_body = Some(rigid_body);
        self
    }

    /// Set the predicate to apply a custom collider filtering during the scene query.
    ///
    /// The predicate is given the collider’s entity and its Rapier collider.
    pub fn predicate(mut self, predicate: &'a impl Fn(Entity, &RapierCollider) -> bool) -> Self {
        self.predicate = Some(predicate);
        self
    }
}
