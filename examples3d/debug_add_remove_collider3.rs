use na::Point3;
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
    let ground_size = 3.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, 0.4).build();
    let mut ground_collider_handle = colliders.insert(collider, ground_handle, &mut bodies);

    /*
     * Rolling ball
     */
    let ball_rad = 0.1;
    let rb = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 0.2, 0.0)
        .build();
    let ball_handle = bodies.insert(rb);
    let collider = ColliderBuilder::ball(ball_rad).density(100.0).build();
    colliders.insert(collider, ball_handle, &mut bodies);

    testbed.add_callback(move |mut window, mut graphics, physics, _, _| {
        // Remove then re-add the ground collider.
        let coll = physics
            .colliders
            .remove(ground_collider_handle, &mut physics.bodies, true)
            .unwrap();
        ground_collider_handle = physics
            .colliders
            .insert(coll, ground_handle, &mut physics.bodies);

        if let (Some(graphics), Some(window)) = (&mut graphics, &mut window) {
            graphics.add_collider(window, ground_collider_handle, &physics.colliders);
        }
    });

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
