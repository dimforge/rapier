use rapier3d::{
    control::{CharacterAutostep, CharacterLength, KinematicCharacterController},
    prelude::*,
};

fn main() {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
    colliders.insert(collider);

    /* Create the body to control. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 10.0, 0.0])
        .build();
    let collider = ColliderBuilder::ball(0.5).restitution(0.7).build();
    let rigid_body_handle = bodies.insert(rigid_body);
    colliders.insert_with_parent(collider.clone(), rigid_body_handle, &mut bodies);

    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, -9.81, 0.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();
    let character_shape = collider.shape();
    let dt = integration_parameters.dt;
    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..200 {
        {
            let character_pos = bodies[rigid_body_handle].position();
            // DOCUSAURUS: Setup start
            // The translation we would like to apply if there were no obstacles.
            let desired_translation = vector![1.0, -2.0, 3.0];
            // Create the character controller, here with the default configuration.
            let character_controller = KinematicCharacterController::default();
            // Init the query pipeline.
            let filter = QueryFilter::default()
                // Make sure the character we are trying to move isn’t considered an obstacle.
                .exclude_rigid_body(rigid_body_handle);
            let query_pipeline = broad_phase.as_query_pipeline(
                narrow_phase.query_dispatcher(),
                &bodies,
                &colliders,
                filter,
            );
            // Calculate the possible movement.
            let corrected_movement = character_controller.move_shape(
                dt,              // The timestep length (can be set to SimulationSettings::dt).
                &query_pipeline, // The query pipelien.
                character_shape, // The character’s shape.
                character_pos,   // The character’s initial position.
                desired_translation,
                |_| {}, // We don’t care about events in this example.
            );

            // TODO: apply the `corrected_movement.translation` to the rigid-body or collider based on the rules described bellow.
            // DOCUSAURUS: Setup stop
        }

        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            &physics_hooks,
            &event_handler,
        );

        let character_body = &bodies[rigid_body_handle];
        println!("Character body position: {}", character_body.translation());
    }
}
