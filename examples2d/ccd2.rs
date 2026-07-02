use kiss3d::color::Color;
use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 25.0;
    let ground_thickness = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().ccd_enabled(true);
    let collider = ColliderBuilder::cuboid(ground_size, ground_thickness);
    let (ground_handle, _) = world.insert(rigid_body, collider);

    let collider =
        ColliderBuilder::cuboid(ground_thickness, ground_size).translation(Vector::new(-3.0, 0.0));
    world.insert_collider(collider, Some(ground_handle));

    let collider =
        ColliderBuilder::cuboid(ground_thickness, ground_size).translation(Vector::new(6.0, 0.0));
    world.insert_collider(collider, Some(ground_handle));

    let collider = ColliderBuilder::cuboid(ground_thickness, ground_size)
        .translation(Vector::new(2.5, 0.0))
        .sensor(true)
        .active_events(ActiveEvents::COLLISION_EVENTS);
    let sensor_handle = world.insert_collider(collider, Some(ground_handle));

    /*
     * Create the shapes
     */
    let radx = 0.4;
    let rady = 0.05;

    let delta1 = Pose::from_translation(Vector::new(0.0, radx - rady));
    let delta2 = Pose::from_translation(Vector::new(-radx + rady, 0.0));
    let delta3 = Pose::from_translation(Vector::new(radx - rady, 0.0));

    let mut compound_parts = Vec::new();
    let vertical = SharedShape::cuboid(rady, radx);
    let horizontal = SharedShape::cuboid(radx, rady);
    compound_parts.push((delta1, horizontal));
    compound_parts.push((delta2, vertical.clone()));
    compound_parts.push((delta3, vertical));

    let compound_shape = SharedShape::compound(compound_parts);

    let num = 6;
    let shift = (radx + 0.01) * 2.0;
    let centerx = shift * num as f32 / 2.0 - 0.5;
    let centery = shift / 2.0 + 4.0;

    for i in 0usize..num {
        for j in 0..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, y))
                .linvel(Vector::new(100.0, -10.0))
                .ccd_enabled(true);

            // for part in &compound_parts {
            //     let collider = ColliderBuilder::new(part.1.clone())
            //         .position_wrt_parent(part.0)
            //         ;
            //     colliders.insert_with_parent(collider, handle, &mut bodies);
            // }

            let collider = ColliderBuilder::new(compound_shape.clone());
            // let collider = ColliderBuilder::cuboid(radx, rady);
            let _ = world.insert(rigid_body, collider);
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 2.5), 20.0);

    let (collision_send, collision_recv) = std::sync::mpsc::channel();
    let (contact_force_send, _contact_force_recv) = std::sync::mpsc::channel();
    let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step_with_events(&(), &event_handler);

            // Handle proximities.
            while let Ok(prox) = collision_recv.try_recv() {
                let color = if prox.started() {
                    Color::new(1.0, 1.0, 0.0, 1.0)
                } else {
                    Color::new(0.5, 0.5, 1.0, 1.0)
                };

                let parent_handle1 = world
                    .colliders
                    .get(prox.collider1())
                    .unwrap()
                    .parent()
                    .unwrap();
                let parent_handle2 = world
                    .colliders
                    .get(prox.collider2())
                    .unwrap()
                    .parent()
                    .unwrap();
                if parent_handle1 != ground_handle && prox.collider1() != sensor_handle {
                    viewer.set_body_color(parent_handle1, color, false);
                }

                if parent_handle2 != ground_handle && prox.collider2() != sensor_handle {
                    viewer.set_body_color(parent_handle2, color, false);
                }
            }
        }
    }
    Ok(())
}
