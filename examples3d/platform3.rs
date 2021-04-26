use na::{Isometry3, Point3, UnitQuaternion, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

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
    let ground_size = 20.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Create the boxes
     */
    let num = 1;
    let rad = 0.2;

    let shift = rad * 2.0;
    let centerx = shift * num as f32 / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * num as f32 / 2.0;

    for i in 0usize..20 {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * (shift + rad / 4.0) - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new_dynamic()
                    .translation(x, y + rad, z)
                    .ccd_enabled(true)
                    .build();
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad, rad * 2.0, rad).build();
                colliders.insert(collider, handle, &mut bodies);
            }
        }
    }

    /*
     * Setup a kinematic rigid body.
     */
    let platform_body = RigidBodyBuilder::new_kinematic()
        .translation(0.2, 0.4, -40.0 * rad)
        .build();
    let platform_handle = bodies.insert(platform_body);
    let collider1 = ColliderBuilder::cuboid(rad * 5.0, rad * 2.0, rad * 10.0).build();
    let collider2 = ColliderBuilder::cuboid(rad * 5.0, rad * 2.0, rad * 10.0)
        .position_wrt_parent(Isometry3::translation(0.0, rad * 2.1, 0.0))
        .build();
    colliders.insert(collider1, platform_handle, &mut bodies);
    colliders.insert(collider2, platform_handle, &mut bodies);
    testbed.set_body_color(platform_handle, Point3::new(1.0, 1.0, 0.0));

    /*
     * Setup a callback to control the platform.
     */
    let mut count = 0;
    testbed.add_callback(move |_, _, physics, _, run_state| {
        count += 1;
        // if count % 100 > 50 {
        //     return;
        // }

        if let Some(platform) = physics.bodies.get_mut(platform_handle) {
            let mut next_pos = *platform.position();

            let dt = 0.016;
            // next_pos.translation.vector.y += (run_state.time * 5.0).sin() * dt;
            // next_pos.translation.vector.z += run_state.time.sin() * 5.0 * dt;

            let drot = UnitQuaternion::new(Vector3::y() * 0.01);
            next_pos.rotation = drot * next_pos.rotation;
            next_pos.translation.vector += next_pos.rotation * Vector3::z() * 0.1;

            // if next_pos.translation.vector.z >= rad * 10.0 {
            //     next_pos.translation.vector.z -= dt;
            // }
            // if next_pos.translation.vector.z <= -rad * 10.0 {
            //     next_pos.translation.vector.z += dt;
            // }

            platform.set_next_kinematic_position(next_pos);
        }
    });

    /*
     * Run the simulation.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(-10.0, 5.0, -10.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Kinematic body", init_world)]);
    testbed.run()
}
