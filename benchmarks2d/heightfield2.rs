use rapier_testbed2d::Testbed;
use rapier2d::na::DVector;
use rapier2d::prelude::*;

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
    let ground_size = Vector::new(50.0, 1.0);
    let nsubdivs = 2000;

    let heights = DVector::from_fn(nsubdivs + 1, |i, _| {
        if i == 0 || i == nsubdivs {
            80.0
        } else {
            (i as f32 * ground_size.x / (nsubdivs as f32)).cos() * 2.0
        }
    });

    let rigid_body = RigidBodyBuilder::fixed();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::heightfield(heights, ground_size);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the cubes
     */
    let num = 26;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 3.0;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y]);
            let handle = bodies.insert(rigid_body);

            if j % 2 == 0 {
                let collider = ColliderBuilder::cuboid(rad, rad);
                colliders.insert_with_parent(collider, handle, &mut bodies);
            } else {
                let collider = ColliderBuilder::ball(rad);
                colliders.insert_with_parent(collider, handle, &mut bodies);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 50.0], 10.0);
}
