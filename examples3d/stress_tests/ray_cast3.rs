use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vec3::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    world.insert(rigid_body, collider);

    /*
     * Create the cubes
     */
    let num = 10;
    let rad = 1.0;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0) * 0.5;

    for j in 0usize..num {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz + offset;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(Vec3::new(x, y, z));
                let collider = ColliderBuilder::cuboid(rad, rad, rad);
                world.insert(rigid_body, collider);
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(100.0, 100.0, 100.0), Vec3::ZERO);

    /*
     * Cast rays at each frame.
     */
    let ray_ball_radius = 100.0;
    let ray_ball = Ball::new(ray_ball_radius);
    let (ray_origins, _) = ray_ball.to_trimesh(100, 100);
    let rays: Vec<_> = ray_origins
        .into_iter()
        .map(|pt| Ray::new(pt, -pt.normalize()))
        .collect();
    let mut centered_rays = rays.clone();

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();

            // Re-center the ray relative to the current position of all objects.
            // This ensures demos with falling objects don’t end up with a boring situation
            // where all the rays point into the void.
            let mut center = Vec3::ZERO;
            for (_, b) in world.bodies.iter() {
                center += b.translation();
            }
            center /= world.bodies.len() as Real;

            for (centered, ray) in centered_rays.iter_mut().zip(rays.iter()) {
                centered.origin = center + ray.origin;
            }

            // Cast the rays.
            let t1 = std::time::Instant::now();
            let max_toi = ray_ball_radius - 1.0;

            let query_pipeline = world.broad_phase.as_query_pipeline(
                world.narrow_phase.query_dispatcher(),
                &world.bodies,
                &world.colliders,
                Default::default(),
            );

            let mut hits = 0;
            for ray in &centered_rays {
                if query_pipeline.cast_ray(ray, max_toi, true).is_some() {
                    hits += 1;
                }
            }
            let main_check_time = t1.elapsed().as_secs_f32();

            viewer
                .example_settings_mut()
                .set_label("Ray count:", format!("{}", rays.len()));
            viewer
                .example_settings_mut()
                .set_label("Ray hits:", format!("{}", hits));
            viewer.example_settings_mut().set_label(
                "Ray-cast time",
                format!("{:.2}ms", main_check_time * 1000.0,),
            );
        }
    }
    Ok(())
}
