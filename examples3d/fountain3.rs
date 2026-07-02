use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

const MAX_NUMBER_OF_BODIES: usize = 400;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    let rad = 0.5;

    /*
     * Ground
     */
    let ground_size = 40.0;
    let ground_height = 2.1; // 16.0;

    for k in 0..3 {
        let rigid_body =
            RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height - k as f32, 0.0));
        let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
        let (_handle, _) = world.insert(rigid_body, collider);
    }

    /*
     * Set up the viewer.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(30.0, 4.0, 30.0), Vec3::new(0.0, 1.0, 0.0));

    /*
     * Run the simulation, spawning bodies from the fountain each step and
     * pruning the oldest ones once the cap is reached (was an `add_callback`).
     */
    let mut step_id = 0usize;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
            step_id += 1;

            let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0, 0.0));
            let handle = world.bodies.insert(rigid_body);
            let collider = match step_id % 3 {
                0 => ColliderBuilder::round_cylinder(rad, rad, rad / 10.0),
                1 => ColliderBuilder::cone(rad, rad),
                _ => ColliderBuilder::cuboid(rad, rad, rad),
            };

            world
                .colliders
                .insert_with_parent(collider, handle, &mut world.bodies);
            viewer.add_body(handle, &world);

            if world.bodies.len() > MAX_NUMBER_OF_BODIES {
                let mut to_remove: Vec<(RigidBodyHandle, Vector)> = world
                    .bodies
                    .iter()
                    .filter(|e| e.1.is_dynamic())
                    .map(|e| (e.0, e.1.translation()))
                    .collect();

                to_remove.sort_by(|a, b| {
                    (a.1.x.abs() + a.1.z.abs())
                        .partial_cmp(&(b.1.x.abs() + b.1.z.abs()))
                        .unwrap()
                        .reverse()
                });

                let num_to_remove = to_remove.len().saturating_sub(MAX_NUMBER_OF_BODIES);
                for (handle, _) in &to_remove[..num_to_remove] {
                    world.bodies.remove(
                        *handle,
                        &mut world.islands,
                        &mut world.colliders,
                        &mut world.impulse_joints,
                        &mut world.multibody_joints,
                        true,
                    );
                    viewer.remove_body(*handle);
                }
            }
        }
    }

    Ok(())
}
