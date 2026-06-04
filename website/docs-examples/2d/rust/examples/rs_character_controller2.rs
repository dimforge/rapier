use rapier2d::{
    control::{CharacterAutostep, CharacterLength, KinematicCharacterController},
    prelude::*,
};

fn main() {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(100.0, 0.1).build();
    colliders.insert(collider);

    /* Create the body to control. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 10.0])
        .build();
    let collider = ColliderBuilder::ball(0.5).restitution(0.7).build();
    let rigid_body_handle = bodies.insert(rigid_body);
    colliders.insert_with_parent(collider.clone(), rigid_body_handle, &mut bodies);

    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, -9.81];
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
            let desired_translation = vector![1.0, -2.0];
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
                &query_pipeline, // The query pipeline.
                character_shape, // The character’s shape.
                character_pos,   // The character’s initial position.
                desired_translation,
                |_| {}, // We don’t care about events in this example.
            );
            // TODO: apply the `corrected_movement.translation` to the rigid-body or collider based on the rules described below.
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
        let filter = QueryFilter::default()
            // Make sure the character we are trying to move isn’t considered an obstacle.
            .exclude_rigid_body(rigid_body_handle);
        let query_pipeline = broad_phase.as_query_pipeline(
            narrow_phase.query_dispatcher(),
            &bodies,
            &colliders,
            filter,
        );

        {
            let character_pos = bodies[rigid_body_handle].position();
            let desired_translation = vector![1.0, -2.0];
            // DOCUSAURUS: Collisions1 start
            let character_controller = KinematicCharacterController::default();
            // Use a closure to handle or collect the collisions while
            // the character is being moved.
            character_controller.move_shape(
                dt,
                &query_pipeline,
                character_shape,
                character_pos,
                desired_translation,
                |collision| { /* Handle or collect the collision in this closure. */ },
            );
            // DOCUSAURUS: Collisions1 stop

            let character_controller = KinematicCharacterController::default();
            let character_mass = 2f32;

            // DOCUSAURUS: Collisions2 start
            // First, collect all the collisions.
            let mut collisions = vec![];
            character_controller.move_shape(
                dt,
                &query_pipeline,
                character_shape,
                &character_pos,
                desired_translation,
                |collision| collisions.push(collision),
            );
            // Then, let the character controller solve (and apply) the collision impulses
            // to the dynamic rigid-bodies hit along its path.
            // Note that we need to init a QueryPipelineMut here (because the impulse
            // application will modify rigid-bodies.
            let mut query_pipeline_mut = broad_phase.as_query_pipeline_mut(
                narrow_phase.query_dispatcher(),
                &mut bodies,
                &mut colliders,
                filter,
            );
            character_controller.solve_character_collision_impulses(
                dt,
                &mut query_pipeline_mut,
                character_shape,
                character_mass,
                &collisions,
            );
            // DOCUSAURUS: Collisions2 stop
        }
        let character_body = &bodies[rigid_body_handle];
        println!("Character body position: {}", character_body.translation());
    }

    let mut character_controller = KinematicCharacterController::default();
    // DOCUSAURUS: Offset start
    // The character offset is set to 0.01.
    character_controller.offset = CharacterLength::Absolute(0.01);
    // The character offset is set to 0.01 multiplied by the shape’s height.
    character_controller.offset = CharacterLength::Relative(0.01);
    // DOCUSAURUS: Offset stop

    // DOCUSAURUS: UpVector start
    // Set the up-vector to the positive X axis.
    character_controller.up = Vector::x_axis();
    // DOCUSAURUS: UpVector stop

    // DOCUSAURUS: Slopes start
    // Don’t allow climbing slopes larger than 45 degrees.
    character_controller.max_slope_climb_angle = 45_f32.to_radians();
    // Automatically slide down on slopes smaller than 30 degrees.
    character_controller.min_slope_slide_angle = 30_f32.to_radians();
    // DOCUSAURUS: Slopes stop

    // DOCUSAURUS: Stairs start
    // Set autostep to None to disable it.
    character_controller.autostep = None;
    // Autostep if the step height is smaller than 0.5, and its width larger than 0.2.
    character_controller.autostep = Some(CharacterAutostep {
        max_height: CharacterLength::Absolute(0.5),
        min_width: CharacterLength::Absolute(0.2),
        include_dynamic_bodies: true,
    });
    // Autostep if the step height is smaller than 0.5 multiplied by the character’s height,
    // and its width larger than 0.5 multiplied by the character’s width (i.e. half the character’s
    // width).
    character_controller.autostep = Some(CharacterAutostep {
        max_height: CharacterLength::Relative(0.3),
        min_width: CharacterLength::Relative(0.5),
        include_dynamic_bodies: true,
    });
    // DOCUSAURUS: Stairs stop

    // DOCUSAURUS: Snap start
    // Set snap-to-ground to None to disable it.
    character_controller.snap_to_ground = None;
    // Snap to the ground if the vertical distance to the ground is smaller than 0.5.
    character_controller.snap_to_ground = Some(CharacterLength::Absolute(0.5));
    // Snap to the ground if the vertical distance to the ground is smaller than 0.2 times the character’s height.
    character_controller.snap_to_ground = Some(CharacterLength::Relative(0.2));
    // DOCUSAURUS: Snap stop
}
