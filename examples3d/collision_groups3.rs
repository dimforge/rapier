use na::Point3;
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet, InteractionGroups};
use rapier_testbed3d::Testbed;

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
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, floor_handle, &mut bodies);

    /*
     * Setup groups
     */
    const GREEN_GROUP: InteractionGroups = InteractionGroups::new(0b01, 0b01);
    const BLUE_GROUP: InteractionGroups = InteractionGroups::new(0b10, 0b10);

    /*
     * A green floor that will collide with the GREEN group only.
     */
    let green_floor = ColliderBuilder::cuboid(1.0, 0.1, 1.0)
        .translation(0.0, 1.0, 0.0)
        .collision_groups(GREEN_GROUP)
        .build();
    let green_collider_handle = colliders.insert(green_floor, floor_handle, &mut bodies);

    testbed.set_collider_initial_color(green_collider_handle, Point3::new(0.0, 1.0, 0.0));

    /*
     * A blue floor that will collide with the BLUE group only.
     */
    let blue_floor = ColliderBuilder::cuboid(1.0, 0.1, 1.0)
        .translation(0.0, 2.0, 0.0)
        .collision_groups(BLUE_GROUP)
        .build();
    let blue_collider_handle = colliders.insert(blue_floor, floor_handle, &mut bodies);

    testbed.set_collider_initial_color(blue_collider_handle, Point3::new(0.0, 0.0, 1.0));

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
                    (GREEN_GROUP, Point3::new(0.0, 1.0, 0.0))
                } else {
                    (BLUE_GROUP, Point3::new(0.0, 0.0, 1.0))
                };

                let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad, rad, rad)
                    .collision_groups(group)
                    .build();
                colliders.insert(collider, handle, &mut bodies);

                testbed.set_body_color(handle, color);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(10.0, 10.0, 10.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
