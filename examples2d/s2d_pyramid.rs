use rapier_testbed2d::Testbed;
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
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -1.0]);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(100.0, 1.0).friction(0.6);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * Create the cubes
     */
    const BASE_COUNT_SETTING: &str = "# of basis cubes";
    let settings = testbed.example_settings_mut();
    let base_count = settings.get_or_set_u32(BASE_COUNT_SETTING, 100, 2..=200);

    let h = 0.5;
    let shift = 1.0 * h;

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * shift;

        for j in i..base_count {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j as f32 - i as f32) * shift
                - h * base_count as f32;
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y]);
            let ground_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(h, h).friction(0.6);
            colliders.insert_with_parent(collider, ground_handle, &mut bodies);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 20.0);
}
