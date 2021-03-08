use na::{Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderHandle, ColliderSet};
use rapier3d::pipeline::{ContactModificationContext, PhysicsHooks, PhysicsHooksFlags};
use rapier_testbed3d::Testbed;

struct OneWayPlatformHook {
    platform1: ColliderHandle,
    platform2: ColliderHandle,
}

impl PhysicsHooks for OneWayPlatformHook {
    fn active_hooks(&self) -> PhysicsHooksFlags {
        PhysicsHooksFlags::MODIFY_SOLVER_CONTACTS
    }

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
        let mut allowed_local_n1 = Vector3::zeros();

        if context.collider_handle1 == self.platform1 {
            allowed_local_n1 = Vector3::y();
        } else if context.collider_handle2 == self.platform1 {
            // Flip the allowed direction.
            allowed_local_n1 = -Vector3::y();
        }

        if context.collider_handle1 == self.platform2 {
            allowed_local_n1 = -Vector3::y();
        } else if context.collider_handle2 == self.platform2 {
            // Flip the allowed direction.
            allowed_local_n1 = Vector3::y();
        }

        // Call the helper function that simulates one-way platforms.
        context.update_as_oneway_platform(&allowed_local_n1, 0.1);

        // Set the surface velocity of the accepted contacts.
        let tangent_velocity = if context.collider_handle1 == self.platform1
            || context.collider_handle2 == self.platform2
        {
            -12.0
        } else {
            12.0
        };

        for contact in context.solver_contacts.iter_mut() {
            contact.tangent_velocity.z = tangent_velocity;
        }
    }
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::new_static().build();
    let handle = bodies.insert(rigid_body);

    let collider = ColliderBuilder::cuboid(9.0, 0.5, 25.0)
        .translation(0.0, 2.0, 30.0)
        .modify_solver_contacts(true)
        .build();
    let platform1 = colliders.insert(collider, handle, &mut bodies);
    let collider = ColliderBuilder::cuboid(9.0, 0.5, 25.0)
        .translation(0.0, -2.0, -30.0)
        .modify_solver_contacts(true)
        .build();
    let platform2 = colliders.insert(collider, handle, &mut bodies);

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
    testbed.add_callback(move |mut window, mut graphics, physics, _, run_state| {
        if run_state.timestep_id % 50 == 0 && physics.bodies.len() <= 7 {
            // Spawn a new cube.
            let collider = ColliderBuilder::cuboid(1.0, 2.0, 1.5).build();
            let body = RigidBodyBuilder::new_dynamic()
                .translation(0.0, 6.0, 20.0)
                .build();
            let handle = physics.bodies.insert(body);
            physics
                .colliders
                .insert(collider, handle, &mut physics.bodies);

            if let (Some(graphics), Some(window)) = (&mut graphics, &mut window) {
                graphics.add(window, handle, &physics.bodies, &physics.colliders);
            }
        }

        physics.bodies.foreach_active_dynamic_body_mut(|_, body| {
            if body.position().translation.y > 1.0 {
                body.set_gravity_scale(1.0, false);
            } else if body.position().translation.y < -1.0 {
                body.set_gravity_scale(-1.0, false);
            }
        });
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world_with_params(
        bodies,
        colliders,
        joints,
        Vector3::new(0.0, -9.81, 0.0),
        physics_hooks,
    );
    testbed.look_at(Point3::new(-100.0, 0.0, 0.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
