use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

struct OneWayPlatformHook {
    platform1: ColliderHandle,
    platform2: ColliderHandle,
}

impl PhysicsHooks for OneWayPlatformHook {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        // The allowed normal for the first platform is its local +y axis, and the
        // allowed normal for the second platform is its local -y axis.
        //
        // Now we have to be careful because the `manifold.local_n1` normal points
        // toward the outside of the shape of `context.co1`. So we need to flip the
        // allowed normal direction if the platform is in `context.collider_handle2`.
        //
        // Therefore:
        // - If context.collider_handle1 == self.platform1 then the allowed normal is +y.
        // - If context.collider_handle2 == self.platform1 then the allowed normal is -y.
        // - If context.collider_handle1 == self.platform2 then its allowed normal +y needs to be flipped to -y.
        // - If context.collider_handle2 == self.platform2 then the allowed normal -y needs to be flipped to +y.
        let mut allowed_local_n1 = Vector::zeros();

        if context.collider1 == self.platform1 {
            allowed_local_n1 = Vector::y();
        } else if context.collider2 == self.platform1 {
            // Flip the allowed direction.
            allowed_local_n1 = -Vector::y();
        }

        if context.collider1 == self.platform2 {
            allowed_local_n1 = -Vector::y();
        } else if context.collider2 == self.platform2 {
            // Flip the allowed direction.
            allowed_local_n1 = Vector::y();
        }

        // Call the helper function that simulates one-way platforms.
        context.update_as_oneway_platform(&allowed_local_n1, 0.1);

        // Set the surface velocity of the accepted contacts.
        let tangent_velocity =
            if context.collider1 == self.platform1 || context.collider2 == self.platform2 {
                -12.0
            } else {
                12.0
            };

        for contact in context.solver_contacts.iter_mut() {
            contact.tangent_velocity.x = tangent_velocity;
        }
    }
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::fixed();
    let handle = bodies.insert(rigid_body);

    let collider = ColliderBuilder::cuboid(25.0, 0.5)
        .translation(vector![30.0, 2.0])
        .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS);
    let platform1 = colliders.insert_with_parent(collider, handle, &mut bodies);
    let collider = ColliderBuilder::cuboid(25.0, 0.5)
        .translation(vector![-30.0, -2.0])
        .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS);
    let platform2 = colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Setup the one-way platform hook.
     */
    let physics_hooks = OneWayPlatformHook {
        platform1,
        platform2,
    };

    /*
     * Spawn cubes at regular intervals and apply a custom gravity
     * depending on their position.
     */
    testbed.add_callback(move |graphics, physics, _, run_state| {
        if run_state.timestep_id % 200 == 0 && physics.bodies.len() <= 7 {
            // Spawn a new cube.
            let collider = ColliderBuilder::cuboid(1.5, 2.0);
            let body = RigidBodyBuilder::dynamic().translation(vector![20.0, 10.0]);
            let handle = physics.bodies.insert(body);
            physics
                .colliders
                .insert_with_parent(collider, handle, &mut physics.bodies);

            if let Some(graphics) = graphics {
                graphics.add_body(handle, &physics.bodies, &physics.colliders);
            }
        }

        for handle in physics.islands.active_dynamic_bodies() {
            let body = &mut physics.bodies[*handle];
            if body.position().translation.y > 1.0 {
                body.set_gravity_scale(1.0, false);
            } else if body.position().translation.y < -1.0 {
                body.set_gravity_scale(-1.0, false);
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world_with_params(
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        vector![0.0, -9.81],
        physics_hooks,
    );
    testbed.look_at(point![0.0, 0.0], 20.0);
}
