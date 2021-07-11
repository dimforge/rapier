use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

    let rad = 0.4;
    let num = 5;
    let shift = 1.0;

    for m in 0..8 {
        let z = m as f32 * shift * (num as f32 + 2.0);

        for l in 0..8 {
            let y = l as f32 * shift * (num as f32) * 2.0;

            for j in 0..50 {
                let x = j as f32 * shift * 4.0;

                let ground = RigidBodyBuilder::new_static()
                    .translation(vector![x, y, z])
                    .build();
                let mut curr_parent = bodies.insert(ground);
                let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
                colliders.insert_with_parent(collider, curr_parent, &mut bodies);

                for i in 0..num {
                    let z = z + (i + 1) as f32 * shift;
                    let density = 1.0;
                    let rigid_body = RigidBodyBuilder::new_dynamic()
                        .translation(vector![x, y, z])
                        .build();
                    let curr_child = bodies.insert(rigid_body);
                    let collider = ColliderBuilder::cuboid(rad, rad, rad)
                        .density(density)
                        .build();
                    colliders.insert_with_parent(collider, curr_child, &mut bodies);

                    let axis = if i % 2 == 0 {
                        UnitVector::new_normalize(vector![1.0, 1.0, 0.0])
                    } else {
                        UnitVector::new_normalize(vector![-1.0, 1.0, 0.0])
                    };

                    let z = Vector::z();
                    let mut prism = PrismaticJoint::new(
                        Point::origin(),
                        axis,
                        z,
                        point![0.0, 0.0, -shift],
                        axis,
                        z,
                    );
                    prism.limits_enabled = true;
                    prism.limits[0] = -2.0;
                    prism.limits[1] = 2.0;
                    joints.insert(curr_parent, curr_child, prism);

                    curr_parent = curr_child;
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(point![262.0, 63.0, 124.0], point![101.0, 4.0, -3.0]);
}
