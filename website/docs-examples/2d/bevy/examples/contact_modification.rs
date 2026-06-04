use bevy::{ecs::system::SystemParam, prelude::*};
use bevy_rapier2d::prelude::*;

// DOCUSAURUS: ContactModification start
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<MyPhysicsHooks>::default())
        .add_systems(Startup, setup_physics)
        .run();
}

#[derive(SystemParam)]
struct MyPhysicsHooks;

impl BevyPhysicsHooks for MyPhysicsHooks {
    fn modify_solver_contacts(&self, context: ContactModificationContextView) {
        // This is a silly example of contact modifier that does silly things
        // for illustration purpose:
        // - Flip all the contact normals.
        // - Delete the first contact.
        // - Set the friction coefficients to 0.3
        // - Set the restitution coefficients to 0.4
        // - Set the tangent velocities to X * 10.0
        *context.raw.normal = -*context.raw.normal;

        if !context.raw.solver_contacts.is_empty() {
            context.raw.solver_contacts.swap_remove(0);
        }

        for solver_contact in &mut *context.raw.solver_contacts {
            solver_contact.friction = 0.3;
            solver_contact.restitution = 0.4;
            solver_contact.tangent_velocity.x = 10.0;
        }

        // Use the persistent user-data to count the number of times
        // contact modification was called for this contact manifold
        // since its creation.
        *context.raw.user_data += 1;
        println!(
            "Contact manifold has been modified {} times since its creation.",
            *context.raw.user_data
        );
    }
}

fn setup_physics(mut commands: Commands) {
    // Add colliders
    commands.spawn((Collider::ball(0.5), ActiveHooks::MODIFY_SOLVER_CONTACTS));

    // TODO: add other colliders in a similar way.
}
// DOCUSAURUS: ContactModification stop
