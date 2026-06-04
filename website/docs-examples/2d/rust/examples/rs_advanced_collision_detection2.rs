use rapier2d::prelude::*;

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(100.0, 0.1).build();
    let collider_handle1 = collider_set.insert(collider);

    /* Create the bouncing ball. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 10.0])
        .build();
    let collider = ColliderBuilder::ball(0.5).restitution(0.7).build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    let collider_handle2 =
        collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

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

    // DOCUSAURUS: Events start
    // Initialize the event collector.
    let (collision_send, collision_recv) = std::sync::mpsc::channel();
    let (contact_force_send, contact_force_recv) = std::sync::mpsc::channel();
    let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);

    physics_pipeline.step(
        &gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        &physics_hooks,
        &event_handler,
    );

    while let Ok(collision_event) = collision_recv.try_recv() {
        // Handle the collision event.
        println!("Received collision event: {:?}", collision_event);
    }

    while let Ok(contact_force_event) = contact_force_recv.try_recv() {
        // Handle the contact force event.
        println!("Received contact force event: {:?}", contact_force_event);
    }
    // DOCUSAURUS: Events stop

    // DOCUSAURUS: ContactGraph1 start
    /* Find the contact pair, if it exists, between two colliders. */
    if let Some(contact_pair) = narrow_phase.contact_pair(collider_handle1, collider_handle2) {
        // The contact pair exists meaning that the broad-phase identified a potential contact.
        if contact_pair.has_any_active_contact {
            // The contact pair has active contacts, meaning that it
            // contains contacts for which contact forces were computed.
        }

        // We may also read the contact manifolds to access the contact geometry.
        for manifold in &contact_pair.manifolds {
            println!("Local-space contact normal: {}", manifold.local_n1);
            println!("Local-space contact normal: {}", manifold.local_n2);
            println!("World-space contact normal: {}", manifold.data.normal);

            // Read the geometric contacts.
            for contact_point in &manifold.points {
                // Keep in mind that all the geometric contact data are expressed in the local-space of the colliders.
                println!("Found local contact point 1: {:?}", contact_point.local_p1);
                println!("Found contact distance: {:?}", contact_point.dist); // Negative if there is a penetration.
                println!("Found contact impulse: {}", contact_point.data.impulse);
                println!(
                    "Found friction impulse: {}",
                    contact_point.data.tangent_impulse
                );
            }

            // Read the solver contacts.
            for solver_contact in &manifold.data.solver_contacts {
                // Keep in mind that all the solver contact data are expressed in world-space.
                println!("Found solver contact point: {:?}", solver_contact.point);
                // The solver contact distance is negative if there is a penetration.
                println!("Found solver contact distance: {:?}", solver_contact.dist);
            }
        }
    }
    // DOCUSAURUS: ContactGraph1 stop

    // DOCUSAURUS: ContactGraph2 start
    /* Iterate through all the contact pairs involving a specific collider. */
    for contact_pair in narrow_phase.contact_pairs_with(collider_handle1) {
        let other_collider = if contact_pair.collider1 == collider_handle1 {
            contact_pair.collider2
        } else {
            contact_pair.collider1
        };

        // Process the contact pair in a way similar to what we did in
        // the previous example.
    }
    // DOCUSAURUS: ContactGraph2 stop

    // DOCUSAURUS: IntersectionGraph1 start
    /* Find the intersection pair, if it exists, between two colliders. */
    if narrow_phase.intersection_pair(collider_handle1, collider_handle2) == Some(true) {
        println!(
            "The colliders {:?} and {:?} are intersecting!",
            collider_handle1, collider_handle2
        );
    }
    // DOCUSAURUS: IntersectionGraph1 stop

    // DOCUSAURUS: IntersectionGraph2 start
    /* Iterate through all the intersection pairs involving a specific collider. */
    for (collider1, collider2, intersecting) in
        narrow_phase.intersection_pairs_with(collider_handle1)
    {
        if intersecting {
            println!(
                "The colliders {:?} and {:?} are intersecting!",
                collider1, collider2
            );
        }
    }
    // DOCUSAURUS: IntersectionGraph2 stop

    {
        // DOCUSAURUS: PhysicsHooks start
        struct MyPhysicsHooks;

        impl PhysicsHooks for MyPhysicsHooks {
            fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
                // This is a silly example of contact pair filter that:
                // - Enables contact and force computation if both colliders have even user-data.
                // - Enables contact computation but not force computation if both colliders have equal user-data.
                // - Disables contact computation otherwise.
                let user_data1 = context.colliders[context.collider1].user_data;
                let user_data2 = context.colliders[context.collider2].user_data;

                if user_data1 % 2 == 0 && user_data2 % 2 == 0 {
                    Some(SolverFlags::COMPUTE_IMPULSES)
                } else if user_data1 == user_data2 {
                    Some(SolverFlags::empty())
                } else {
                    None
                }
            }

            fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
                // This is a silly example of intersection pair filter that
                // enables the intersection test if both colliders have odd
                // user-data.
                let user_data1 = context.colliders[context.collider1].user_data;
                let user_data2 = context.colliders[context.collider2].user_data;

                user_data1 % 2 == 1 && user_data2 % 2 == 1
            }
        }
        // DOCUSAURUS: PhysicsHooks stop
    }

    // DOCUSAURUS: ContactModification start
    struct MyPhysicsHooks;

    impl PhysicsHooks for MyPhysicsHooks {
        fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
            // This is a silly example of contact modifier that does silly things
            // for illustration purpose:
            // - Flip all the contact normals.
            // - Delete the first contact.
            // - Set the friction coefficients to 0.3
            // - Set the restitution coefficients to 0.4
            // - Set the tangent velocities to X * 10.0
            *context.normal = -*context.normal;

            if !context.solver_contacts.is_empty() {
                context.solver_contacts.swap_remove(0);
            }

            for solver_contact in &mut *context.solver_contacts {
                solver_contact.friction = 0.3;
                solver_contact.restitution = 0.4;
                solver_contact.tangent_velocity.x = 10.0;
            }

            // Use the persistent user-data to count the number of times
            // contact modification was called for this contact manifold
            // since its creation.
            *context.user_data += 1;
            println!(
                "Contact manifold has been modified {} times since its creation.",
                *context.user_data
            );
        }
    }
    // DOCUSAURUS: ContactModification stop
}
