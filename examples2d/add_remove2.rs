use na::Point2;
use rapier2d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier2d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    let bodies = RigidBodySet::new();
    let colliders = ColliderSet::new();
    let joints = JointSet::new();
    let rad = 0.5;

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |mut window, mut graphics, physics, _, _| {
        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(0.0, 10.0)
            .build();
        let handle = physics.bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad).build();
        physics
            .colliders
            .insert(collider, handle, &mut physics.bodies);

        if let (Some(graphics), Some(window)) = (&mut graphics, &mut window) {
            graphics.add(*window, handle, &physics.bodies, &physics.colliders);
        }

        let to_remove: Vec<_> = physics
            .bodies
            .iter()
            .filter(|(_, b)| b.position().translation.vector.y < -10.0)
            .map(|e| e.0)
            .collect();
        for handle in to_remove {
            physics
                .bodies
                .remove(handle, &mut physics.colliders, &mut physics.joints);

            if let (Some(graphics), Some(window)) = (&mut graphics, &mut window) {
                graphics.remove_body_nodes(*window, handle);
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point2::new(0.0, 0.0), 20.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Add-remove", init_world)]);
    testbed.run()
}
