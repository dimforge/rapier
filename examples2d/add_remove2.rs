use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    let rad = 0.5;

    let positions = [Vector::new(5.0, -1.0), Vector::new(-5.0, -1.0)];

    let platform_handles = positions
        .into_iter()
        .map(|pos| {
            let rigid_body = RigidBodyBuilder::kinematic_position_based().translation(pos);
            let collider = ColliderBuilder::cuboid(rad * 10.0, rad);
            let (handle, _) = world.insert(rigid_body, collider);
            handle
        })
        .collect::<Vec<_>>();

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::ZERO, 20.0);

    let mut step_id = 0usize;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
            step_id += 1;

            let rot = -(step_id as f32 * world.integration_parameters.dt as f32);
            for rb_handle in &platform_handles {
                let rb = world.bodies.get_mut(*rb_handle).unwrap();
                rb.set_next_kinematic_rotation(Rotation::new(rot));
            }

            if step_id % 10 == 0 {
                let x = rand::random::<f32>() * 10.0 - 5.0;
                let y = rand::random::<f32>() * 10.0 + 10.0;
                let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y));
                let handle = world.bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad, rad);
                world
                    .colliders
                    .insert_with_parent(collider, handle, &mut world.bodies);

                viewer.add_body(handle, &world);
            }

            let to_remove: Vec<_> = world
                .bodies
                .iter()
                .filter(|(_, b)| b.position().translation.y < -10.0)
                .map(|e| e.0)
                .collect();
            for handle in to_remove {
                world.bodies.remove(
                    handle,
                    &mut world.islands,
                    &mut world.colliders,
                    &mut world.impulse_joints,
                    &mut world.multibody_joints,
                    true,
                );

                viewer.remove_body(handle);
            }
        }
    }
    Ok(())
}
