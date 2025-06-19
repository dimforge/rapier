use rapier3d::prelude::*;
use rapier_testbed3d::{Color, Testbed};

pub fn init_world(testbed: &mut Testbed) {
    let settings = testbed.example_settings_mut();

    // NOTE: this demo is a bit special. It takes as a setting the builder of another demo,
    //       builds it, and add a ton of rays into it. This gives us an easy way to check
    //       ray-casting in a wide variety of situations.
    let demos: Vec<_> = crate::demo_builders()
        .into_iter()
        .filter(|(_, builder)| {
            !std::ptr::fn_addr_eq(*builder, self::init_world as fn(&mut Testbed))
        })
        .collect();
    let demo_names: Vec<_> = demos.iter().map(|(name, _)| name.to_string()).collect();
    let selected = settings.get_or_set_string("Scene", 0, demo_names);
    demos[selected].1(testbed);

    /*
     * Cast rays at each frame.
     */
    let ray_ball_radius = 100.0;
    let ray_ball = Ball::new(ray_ball_radius);
    let (ray_origins, _) = ray_ball.to_trimesh(100, 100);
    let rays: Vec<_> = ray_origins
        .into_iter()
        .map(|pt| Ray::new(pt, -pt.coords.normalize()))
        .collect();
    let mut centered_rays = rays.clone();

    let mut workspace = SahWorkspace::default();

    testbed.add_callback(move |graphics, physics, _, _| {
        let Some(graphics) = graphics else {
            return;
        };

        // Re-center the ray relative to the current position of all objects.
        // This ensures demos with falling objects don’t end up with a boring situation
        // where all the rays point into the void.
        let mut center = Point::origin();
        for (_, b) in physics.bodies.iter() {
            center += b.translation();
        }
        center /= physics.bodies.len() as Real;

        for (centered, ray) in centered_rays.iter_mut().zip(rays.iter()) {
            centered.origin = center + ray.origin.coords;
        }

        let mut query_pipeline_for_comparison = QueryPipeline::new();
        query_pipeline_for_comparison.update(&physics.colliders);

        // Cast the rays.
        let t1 = std::time::Instant::now();
        let max_toi = ray_ball_radius - 1.0;

        for ray in &centered_rays {
            let result = if let Some(sah_bf) = physics.broad_phase.downcast_ref::<BroadPhaseSah>() {
                sah_bf.cast_ray(ray, max_toi, &physics.colliders)
            } else {
                physics.query_pipeline.cast_ray(
                    &physics.bodies,
                    &physics.colliders,
                    ray,
                    max_toi,
                    true,
                    QueryFilter::default(),
                )
            };

            if let Some((_, toi)) = result {
                let a = ray.origin;
                let b = ray.point_at(toi);
                graphics
                    .gizmos
                    .line(a.into(), b.into(), Color::rgba(0.0, 1.0f32, 0.0, 0.1));
            } else {
                let a = ray.origin;
                let b = ray.point_at(max_toi);
                graphics
                    .gizmos
                    .line(a.into(), b.into(), Color::rgba(1.0f32, 0.0, 0.0, 0.1));
            }
        }
        let main_check_time = t1.elapsed().as_secs_f32();
        let t1 = std::time::Instant::now();
        for ray in &centered_rays {
            query_pipeline_for_comparison.cast_ray(
                &physics.bodies,
                &physics.colliders,
                ray,
                max_toi,
                true,
                QueryFilter::default(),
            );
        }
        let comparison_check_time = t1.elapsed().as_secs_f32();

        if let Some(settings) = &mut graphics.settings {
            settings.set_label("Ray count:", format!("{}", rays.len()));
            let speedup = if comparison_check_time < main_check_time {
                (1.0 - main_check_time / comparison_check_time) * 100.0
            } else {
                (comparison_check_time / main_check_time - 1.0) * 100.0
            };
            settings.set_label(
                "Ray-cast time",
                format!(
                    "{:.2}ms (vs. {:.2}ms). Speedup: {:.2}%",
                    main_check_time * 1000.0,
                    comparison_check_time * 1000.0,
                    speedup
                ),
            );
        }
    });
}
