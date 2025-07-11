use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Create the balls
     */
    // Build the rigid body.
    let rad = 0.4;
    let num = 10;
    let shift = 1.0;

    for l in 0..25 {
        let y = l as f32 * shift * (num as f32 + 2.0) * 2.0;

        for j in 0..50 {
            let x = j as f32 * shift * 4.0;

            let ground = RigidBodyBuilder::fixed().translation(vector![x, y]);
            let mut curr_parent = bodies.insert(ground);
            let collider = ColliderBuilder::cuboid(rad, rad);
            colliders.insert_with_parent(collider, curr_parent, &mut bodies);

            for i in 0..num {
                let y = y - (i + 1) as f32 * shift;
                let density = 1.0;
                let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y]);
                let curr_child = bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad, rad).density(density);
                colliders.insert_with_parent(collider, curr_child, &mut bodies);

                let axis = if i % 2 == 0 {
                    UnitVector::new_normalize(vector![1.0, 1.0])
                } else {
                    UnitVector::new_normalize(vector![-1.0, 1.0])
                };

                let prism = PrismaticJointBuilder::new(axis)
                    .local_anchor2(point![0.0, shift])
                    .limits([-1.5, 1.5]);
                impulse_joints.insert(curr_parent, curr_child, prism, true);

                curr_parent = curr_child;
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![80.0, 80.0], 15.0);
}
