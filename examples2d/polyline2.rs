use rapier2d::na::ComplexField;
use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

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
    let ground_size = 50.0;
    let nsubdivs = 2000;
    let step_size = ground_size / (nsubdivs as f32);
    let mut points = Vec::new();

    points.push(point![-ground_size / 2.0, 40.0]);
    for i in 1..nsubdivs - 1 {
        let x = -ground_size / 2.0 + i as f32 * step_size;
        let y = ComplexField::cos(i as f32 * step_size) * 2.0;
        points.push(point![x, y]);
    }
    points.push(point![ground_size / 2.0, 40.0]);

    let rigid_body = RigidBodyBuilder::new_static().build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::polyline(points, None).build();
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the cubes
     */
    let num = 20;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 3.0;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::new_dynamic()
                .translation(vector![x, y])
                .build();
            let handle = bodies.insert(rigid_body);

            if j % 2 == 0 {
                let collider = ColliderBuilder::cuboid(rad, rad).build();
                colliders.insert_with_parent(collider, handle, &mut bodies);
            } else {
                let collider = ColliderBuilder::ball(rad).build();
                colliders.insert_with_parent(collider, handle, &mut bodies);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(point![0.0, 0.0], 10.0);
}
