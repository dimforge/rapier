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
    fn modify_solver_contacts(&self, mut context: ContactModificationContextView) {
        // This is a silly example of contact modifier that does silly things
        // for illustration purpose:
        // - Flip all the contact normals.
        // - Delete the first contact.
        // - Set the friction coefficient to 0.3
        // - Set the restitution coefficient to 0.4
        // - Set the tangent velocities to X * 10.0
        // The contacts of two soft surfaces are candidates rather than a manifold:
        // only the manifolds of rigid pairs are modified here.
        let Some(normal) = context.normal() else {
            return;
        };
        context.set_normal(-normal);

        if let Some(solver_contacts) = context.solver_contacts_mut() {
            if !solver_contacts.is_empty() {
                solver_contacts.swap_remove(0);
            }

            for solver_contact in solver_contacts.iter_mut() {
                solver_contact.tangent_velocity.x = 10.0;
            }
        }

        // Friction and restitution are combined once per manifold, so they are set
        // for the whole manifold rather than per solver contact.
        context.set_friction(0.3);
        context.set_restitution(0.4);

        // Use the persistent user-data to count the number of times
        // contact modification was called for this contact manifold
        // since its creation.
        let num_calls = context.user_data().unwrap_or(0) + 1;
        context.set_user_data(num_calls);
        println!(
            "Contact manifold has been modified {} times since its creation.",
            num_calls
        );
    }
}

fn setup_physics(mut commands: Commands) {
    // Add colliders
    commands.spawn((Collider::ball(0.5), ActiveHooks::MODIFY_SOLVER_CONTACTS));

    // TODO: add other colliders in a similar way.
}
// DOCUSAURUS: ContactModification stop
