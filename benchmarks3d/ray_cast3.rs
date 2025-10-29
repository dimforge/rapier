use rapier_testbed3d::{Color, Testbed};
use rapier3d::prelude::*;

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

        // Cast the rays.
        let t1 = std::time::Instant::now();
        let max_toi = ray_ball_radius - 1.0;

        let query_pipeline = physics.broad_phase.as_query_pipeline(
            physics.narrow_phase.query_dispatcher(),
            &physics.bodies,
            &physics.colliders,
            Default::default(),
        );

        for ray in &centered_rays {
            if let Some((_, toi)) = query_pipeline.cast_ray(ray, max_toi, true) {
                let a = ray.origin;
                let b = ray.point_at(toi);
                graphics
                    .gizmos
                    .line(a.into(), b.into(), Color::srgba(0.0, 1.0f32, 0.0, 0.1));
            } else {
                let a = ray.origin;
                let b = ray.point_at(max_toi);
                graphics
                    .gizmos
                    .line(a.into(), b.into(), Color::srgba(1.0f32, 0.0, 0.0, 0.1));
            }
        }
        let main_check_time = t1.elapsed().as_secs_f32();

        if let Some(settings) = &mut graphics.settings {
            settings.set_label("Ray count:", format!("{}", rays.len()));
            settings.set_label(
                "Ray-cast time",
                format!("{:.2}ms", main_check_time * 1000.0,),
            );
        }
    });
}
