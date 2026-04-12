use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let rad = 0.4;
    let num = 10;
    let shift = 2.0;

    for l in 0..4 {
        let y = l as f32 * shift * (num as f32) * 3.0;

        for j in 0..50 {
            let x = j as f32 * shift * 4.0;

            let ground = RigidBodyBuilder::fixed().translation(Vec3::new(x, y, 0.0));
            let collider = ColliderBuilder::cuboid(rad, rad, rad);
            let (mut curr_parent, _) = world.insert(ground, collider);

            for i in 0..num {
                // Create four bodies.
                let z = i as f32 * shift * 2.0 + shift;
                let positions = [
                    Pose3::translation(x, y, z),
                    Pose3::translation(x + shift, y, z),
                    Pose3::translation(x + shift, y, z + shift),
                    Pose3::translation(x, y, z + shift),
                ];

                let mut handles = [curr_parent; 4];
                for k in 0..4 {
                    let density = 1.0;
                    let rigid_body = RigidBodyBuilder::dynamic().pose(positions[k]);
                    let collider = ColliderBuilder::cuboid(rad, rad, rad).density(density);
                    let (h, _) = world.insert(rigid_body, collider);
                    handles[k] = h;
                }

                // Setup four impulse_joints.
                let x = Vec3::X;
                let z = Vec3::Z;

                let revs = [
                    RevoluteJointBuilder::new(z).local_anchor2(Vec3::new(0.0, 0.0, -shift)),
                    RevoluteJointBuilder::new(x).local_anchor2(Vec3::new(-shift, 0.0, 0.0)),
                    RevoluteJointBuilder::new(z).local_anchor2(Vec3::new(0.0, 0.0, -shift)),
                    RevoluteJointBuilder::new(x).local_anchor2(Vec3::new(shift, 0.0, 0.0)),
                ];

                world.insert_impulse_joint(curr_parent, handles[0], revs[0]);
                world.insert_impulse_joint(handles[0], handles[1], revs[1]);
                world.insert_impulse_joint(handles[1], handles[2], revs[2]);
                world.insert_impulse_joint(handles[2], handles[3], revs[3]);

                curr_parent = handles[3];
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(
        Vec3::new(478.0, 83.0, 228.0),
        Vec3::new(134.0, 83.0, -116.0),
    );
}
