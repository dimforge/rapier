use rapier3d::prelude::*;

// DOCUSAURUS: OneWayPlatform start
struct OneWayPlatform {
    platform: ColliderHandle,
}

impl PhysicsHooks for OneWayPlatform {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        // Keep only the contacts pushing along the local +y axis of the platform; the other
        // ones (the character arriving from below) are discarded. The normal is expressed in
        // the frame of the first collider of the pair, hence the flip.
        let allowed_local_n1 = if context.collider1 == self.platform {
            Vector::Y
        } else {
            -Vector::Y
        };
        context.update_as_oneway_platform(allowed_local_n1, 0.1);
    }
}
// DOCUSAURUS: OneWayPlatform stop

// DOCUSAURUS: ConveyorBelt start
struct ConveyorBelt;

impl PhysicsHooks for ConveyorBelt {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        if let Some(rigid) = context.rigid_mut() {
            for contact in rigid.solver_contacts.iter_mut() {
                // The belt drags the objects along the world-space z axis at 12 m/s.
                contact.tangent_velocity.z = 12.0;
            }
        }
    }
}
// DOCUSAURUS: ConveyorBelt stop

fn main() {
    let mut world = PhysicsWorld::new();

    // DOCUSAURUS: MovingPlatform start
    let (platform_handle, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 1.0, 0.0)),
        ColliderBuilder::cuboid(2.0, 0.1, 2.0),
    );

    for step in 0..200 {
        // Setting the next position of the platform, once per timestep.
        let time = step as f32 * world.integration_parameters.dt;
        let platform = &mut world.bodies[platform_handle];
        platform.set_next_kinematic_translation(Vector::new(time.sin() * 2.0, 1.0, 0.0));
        world.step();
    }
    // DOCUSAURUS: MovingPlatform stop

    // DOCUSAURUS: Hooks start
    // The hooks are only called for the colliders asking for them.
    let platform_collider = world.bodies[platform_handle].colliders()[0];
    world.colliders[platform_collider].set_active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS);

    let hooks = OneWayPlatform {
        platform: platform_collider,
    };
    world.step_with_events(&hooks, &());
    // DOCUSAURUS: Hooks stop

    world.step_with_events(&ConveyorBelt, &());
}
