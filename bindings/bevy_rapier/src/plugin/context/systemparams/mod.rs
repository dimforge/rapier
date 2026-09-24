//! Helper types to ease bevy_rapier API usage, grouping often used together components
//! through [`bevy::ecs::query::QueryData`] or [`bevy::ecs::system::SystemParam`] to
//! avoid writing too much boilerplate: shorter parameters for your systems,
//! and pass less parameters to bey_rapier lower level functions.

mod rapier_context_systemparam;

use bevy::{ecs::query::QueryData, prelude::Entity};
pub use rapier_context_systemparam::*;

use super::RapierContextEntityLink;

/// Information needed to access the rapier data from an entity managed by rapier.
#[derive(QueryData)]
pub struct RapierEntity {
    /// This bevy [`Entity`].
    pub entity: Entity,
    /// Link to another bevy [`Entity`], which owns the context of this [`RapierEntity`]:
    /// i.e. the relevant components from [`crate::plugin::context`].
    pub rapier_context_link: &'static RapierContextEntityLink,
}
