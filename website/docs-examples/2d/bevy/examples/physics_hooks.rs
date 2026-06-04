//! Mostly taken from https://github.com/dimforge/bevy_rapier/blob/0110ee9a054664aa154ab3ffe27c348b07ffba57/bevy_rapier2d/examples/contact_filter2.rs#L5

use bevy::{ecs::system::SystemParam, prelude::*};
use bevy_rapier2d::prelude::*;

// DOCUSAURUS: PhysicsHooks start
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // Make sure the Rapier plugin is parametrized by our custom user-data type.
        .add_plugins(RapierPhysicsPlugin::<SameUserDataFilter>::default())
        .add_systems(Startup, setup_physics)
        .run();
}

#[derive(Component, PartialEq, Eq, Clone, Copy)]
enum CustomFilterTag {
    GroupA,
    GroupB,
}

// A custom filter that allows contacts/intersections only between rigid-bodies
// with the same CustomFilterTag component value.
// Note that using collision groups would be a more efficient way of doing
// this, but we use custom filters instead for demonstration purpose.
#[derive(SystemParam)]
struct SameUserDataFilter<'w, 's> {
    tags: Query<'w, 's, &'static CustomFilterTag>,
}
impl BevyPhysicsHooks for SameUserDataFilter<'_, '_> {
    fn filter_contact_pair(&self, context: PairFilterContextView) -> Option<SolverFlags> {
        if self.tags.get(context.collider1()).ok().copied()
            == self.tags.get(context.collider2()).ok().copied()
        {
            Some(SolverFlags::COMPUTE_IMPULSES)
        } else {
            None
        }
    }

    fn filter_intersection_pair(&self, context: PairFilterContextView) -> bool {
        self.tags.get(context.collider1()).ok().copied()
            == self.tags.get(context.collider2()).ok().copied()
    }
}

fn setup_physics(mut commands: Commands) {
    // Add colliders with a `CustomFilterTag` component. Only colliders
    // with the same `CustomFilterTag` variant will collider thanks to
    // our custom physics hooks:
    commands.spawn((
        Collider::ball(0.5),
        ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::FILTER_INTERSECTION_PAIR,
        CustomFilterTag::GroupA,
    ));

    // TODO: add other colliders in a similar way.
}
// DOCUSAURUS: PhysicsHooks stop
