use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let rad = 0.4;
    let num = 10;
    let shift = 2.0;

    for l in 0..4 {
        let y = l as f32 * shift * (num as f32) * 3.0;

        for j in 0..50 {
            let x = j as f32 * shift * 4.0;

            let ground = RigidBodyBuilder::fixed().translation(vector![x, y, 0.0]);
            let mut curr_parent = bodies.insert(ground);
            let collider = ColliderBuilder::cuboid(rad, rad, rad);
            colliders.insert_with_parent(collider, curr_parent, &mut bodies);

            for i in 0..num {
                // Create four bodies.
                let z = i as f32 * shift * 2.0 + shift;
                let positions = [
                    Isometry::translation(x, y, z),
                    Isometry::translation(x + shift, y, z),
                    Isometry::translation(x + shift, y, z + shift),
                    Isometry::translation(x, y, z + shift),
                ];

                let mut handles = [curr_parent; 4];
                for k in 0..4 {
                    let density = 1.0;
                    let rigid_body = RigidBodyBuilder::dynamic().position(positions[k]);
                    handles[k] = bodies.insert(rigid_body);
                    let collider = ColliderBuilder::cuboid(rad, rad, rad).density(density);
                    colliders.insert_with_parent(collider, handles[k], &mut bodies);
                }

                // Setup four impulse_joints.
                let x = Vector::x_axis();
                let z = Vector::z_axis();

                let revs = [
                    RevoluteJointBuilder::new(z).local_anchor2(point![0.0, 0.0, -shift]),
                    RevoluteJointBuilder::new(x).local_anchor2(point![-shift, 0.0, 0.0]),
                    RevoluteJointBuilder::new(z).local_anchor2(point![0.0, 0.0, -shift]),
                    RevoluteJointBuilder::new(x).local_anchor2(point![shift, 0.0, 0.0]),
                ];

                impulse_joints.insert(curr_parent, handles[0], revs[0], true);
                impulse_joints.insert(handles[0], handles[1], revs[1], true);
                impulse_joints.insert(handles[1], handles[2], revs[2], true);
                impulse_joints.insert(handles[2], handles[3], revs[3], true);

                curr_parent = handles[3];
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![478.0, 83.0, 228.0], point![134.0, 83.0, -116.0]);
}
