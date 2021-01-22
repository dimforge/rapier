use na::Point2;
use rapier2d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier2d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Ground.
     */
    let ground_size = 10.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Create the boxes
     */
    let num = 6;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;
    let centery = shift / 2.0 + 3.04;

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y).build();
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad).build();
            colliders.insert(collider, handle, &mut bodies);
        }
    }

    /*
     * Setup a kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::new_kinematic()
        .translation(-10.0 * rad, 1.5 + 0.8)
        .build();
    let platform_handle = bodies.insert(platform_body);
    let collider = ColliderBuilder::cuboid(rad * 10.0, rad).build();
    colliders.insert(collider, platform_handle, &mut bodies);

    /*
     * Setup a callback to control the platform.
     */
    testbed.add_callback(move |_, _, physics, _, run_state| {
        let platform = physics.bodies.get_mut(platform_handle).unwrap();
        let mut next_pos = *platform.position();

        let dt = 0.016;
        next_pos.translation.vector.y += (run_state.time * 5.0).sin() * dt;
        next_pos.translation.vector.x += run_state.time.sin() * 5.0 * dt;

        if next_pos.translation.vector.x >= rad * 10.0 {
            next_pos.translation.vector.x -= dt;
        }
        if next_pos.translation.vector.x <= -rad * 10.0 {
            next_pos.translation.vector.x += dt;
        }

        platform.set_next_kinematic_position(next_pos);
    });

    /*
     * Run the simulation.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point2::new(0.0, 1.0), 40.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Kinematic body", init_world)]);
    testbed.run()
}
