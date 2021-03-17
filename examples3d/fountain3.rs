use na::Point3;
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

const MAX_NUMBER_OF_BODIES: usize = 400;

pub fn init_world(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();
    let rad = 0.5;

    /*
     * Ground
     */
    let ground_size = 100.1;
    let ground_height = 2.1; // 16.0;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, handle, &mut bodies);

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |mut window, mut graphics, physics, _, run_state| {
        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(0.0, 10.0, 0.0)
            .build();
        let handle = physics.bodies.insert(rigid_body);
        let collider = match run_state.timestep_id % 3 {
            0 => ColliderBuilder::round_cylinder(rad, rad, rad / 10.0).build(),
            1 => ColliderBuilder::cone(rad, rad).build(),
            _ => ColliderBuilder::cuboid(rad, rad, rad).build(),
        };

        physics
            .colliders
            .insert(collider, handle, &mut physics.bodies);

        if let (Some(graphics), Some(window)) = (&mut graphics, &mut window) {
            graphics.add(window, handle, &physics.bodies, &physics.colliders);
        }

        if physics.bodies.len() > MAX_NUMBER_OF_BODIES {
            let mut to_remove: Vec<_> = physics
                .bodies
                .iter()
                .filter(|e| e.1.is_dynamic())
                .map(|e| (e.0, e.1.position().translation.vector))
                .collect();

            to_remove.sort_by(|a, b| {
                (a.1.x.abs() + a.1.z.abs())
                    .partial_cmp(&(b.1.x.abs() + b.1.z.abs()))
                    .unwrap()
                    .reverse()
            });

            let num_to_remove = to_remove.len() - MAX_NUMBER_OF_BODIES;
            for (handle, _) in &to_remove[..num_to_remove] {
                physics
                    .bodies
                    .remove(*handle, &mut physics.colliders, &mut physics.joints);

                if let (Some(graphics), Some(window)) = (&mut graphics, &mut window) {
                    graphics.remove_body_nodes(window, *handle);
                }
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    // testbed
    //     .physics_state_mut()
    //     .integration_parameters
    //     .velocity_based_erp = 0.2;
    testbed.look_at(Point3::new(-30.0, 4.0, -30.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Add-remove", init_world)]);
    testbed.run()
}
