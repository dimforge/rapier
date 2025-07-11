use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

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
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    /*
     * Setup groups
     */
    const GREEN_GROUP: InteractionGroups = InteractionGroups::new(Group::GROUP_1, Group::GROUP_1);
    const BLUE_GROUP: InteractionGroups = InteractionGroups::new(Group::GROUP_2, Group::GROUP_2);

    /*
     * A green floor that will collide with the GREEN group only.
     */
    let green_floor = ColliderBuilder::cuboid(1.0, 0.1, 1.0)
        .translation(vector![0.0, 1.0, 0.0])
        .collision_groups(GREEN_GROUP);
    let green_collider_handle =
        colliders.insert_with_parent(green_floor, floor_handle, &mut bodies);

    testbed.set_initial_collider_color(green_collider_handle, [0.0, 1.0, 0.0]);

    /*
     * A blue floor that will collide with the BLUE group only.
     */
    let blue_floor = ColliderBuilder::cuboid(1.0, 0.1, 1.0)
        .translation(vector![0.0, 2.0, 0.0])
        .collision_groups(BLUE_GROUP);
    let blue_collider_handle = colliders.insert_with_parent(blue_floor, floor_handle, &mut bodies);

    testbed.set_initial_collider_color(blue_collider_handle, [0.0, 0.0, 1.0]);

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 0.1;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = 2.5;
    let centerz = shift * (num / 2) as f32;

    for j in 0usize..4 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                // Alternate between the green and blue groups.
                let (group, color) = if k % 2 == 0 {
                    (GREEN_GROUP, [0.0, 1.0, 0.0])
                } else {
                    (BLUE_GROUP, [0.0, 0.0, 1.0])
                };

                let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y, z]);
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad, rad, rad).collision_groups(group);
                colliders.insert_with_parent(collider, handle, &mut bodies);

                testbed.set_initial_body_color(handle, color);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point!(10.0, 10.0, 10.0), Point::origin());
}
