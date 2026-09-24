use rapier3d::{control::KinematicCharacterController, prelude::*};

fn main() {
    let mut world = PhysicsWorld::new();

    /* Create the ground. */
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1, 100.0), None);

    /* Create the body to control. */
    let collider = ColliderBuilder::ball(0.5).restitution(0.7).build();
    let (rigid_body_handle, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0, 0.0)),
        collider.clone(),
    );

    let character_shape = collider.shape();
    let dt = world.integration_parameters.dt;
    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..200 {
        {
            let character_pos = world.bodies[rigid_body_handle].position();
            // DOCUSAURUS: Setup start
            // The translation we would like to apply if there were no obstacles.
            let desired_translation = Vector::new(1.0, -2.0, 3.0);
            // Create the character controller, here with the default configuration.
            let character_controller = KinematicCharacterController::default();
            // Init the query pipeline.
            let filter = QueryFilter::default()
                // Make sure the character we are trying to move isn’t considered an obstacle.
                .exclude_rigid_body(rigid_body_handle);
            let query_pipeline = world.query_pipeline_with_filter(filter);
            // Calculate the possible movement.
            let corrected_movement = character_controller.move_shape(
                dt,              // The timestep length (can be set to SimulationSettings::dt).
                &query_pipeline, // The query pipeline.
                character_shape, // The character’s shape.
                character_pos,   // The character’s initial position.
                desired_translation,
                |_| {}, // We don’t care about events in this example.
            );

            // TODO: apply the `corrected_movement.translation` to the rigid-body or collider based on the rules described below.
            // DOCUSAURUS: Setup stop
        }

        world.step();

        let character_body = &world.bodies[rigid_body_handle];
        println!("Character body position: {}", character_body.translation());
    }
}
