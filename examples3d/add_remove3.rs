use na::Point3;
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    let bodies = RigidBodySet::new();
    let colliders = ColliderSet::new();
    let joints = JointSet::new();
    let rad = 0.5;

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |window, physics, _, graphics, _| {
        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(0.0, 10.0, 0.0)
            .build();
        let handle = physics.bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad, rad).density(1.0).build();
        physics
            .colliders
            .insert(collider, handle, &mut physics.bodies);
        graphics.add(window, handle, &physics.bodies, &physics.colliders);

        let to_remove: Vec<_> = physics
            .bodies
            .iter()
            .filter(|(_, b)| b.position.translation.vector.y < -10.0)
            .map(|e| e.0)
            .collect();
        for handle in to_remove {
            physics.pipeline.remove_rigid_body(
                handle,
                &mut physics.broad_phase,
                &mut physics.narrow_phase,
                &mut physics.bodies,
                &mut physics.colliders,
                &mut physics.joints,
            );
            graphics.remove_body_nodes(window, handle);
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(-30.0, -4.0, -30.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Add-remove", init_world)]);
    testbed.run()
}
