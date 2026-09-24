use rapier3d::{parry::query::ShapeCastOptions, prelude::*};

fn main() {
    let mut world = PhysicsWorld::new();

    /* Create the ground. */
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1, 100.0), None);

    /* Create the bouncing ball. */
    let (ball_body_handle, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0, 0.0)),
        ColliderBuilder::ball(0.5).restitution(0.7),
    );

    let handle1 = world.insert_collider(ColliderBuilder::cuboid(1.0, 1.0, 1.0).translation(Vector::new(0.0, 10.0, 0.0)), None);
    let handle_to_remove =
        world.insert_collider(ColliderBuilder::cuboid(1.0, 1.0, 1.0).translation(Vector::new(0.0, 15.0, 0.0)), None);
    world.remove_collider(handle_to_remove);
    world.colliders[handle1].set_translation(Vector::new(0.0, 12.0, 0.0));

    // DOCUSAURUS: QueryPipeline start
    // Game loop.
    for _ in 0..10 {
        // Stepping the simulation updates the broad-phase the scene queries rely on.
        world.step();

        // The scene queries take into account the positions of the colliders at the end of
        // the last timestep.
        let query_pipeline = world.query_pipeline();
        // Run the scene queries with `query_pipeline` here.
    }
    // DOCUSAURUS: QueryPipeline stop

    raycast_section(&world);
    shapecast_section(&world);
    point_projection_section(&world);
    intersection_section(&world);
}

#[rustfmt::skip]
fn raycast_section(
    world: &PhysicsWorld,
) {
    // DOCUSAURUS: Raycast start
    let ray = Ray::new(Vector::new(1.0, 2.0, 3.0), Vector::new(0.0, 1.0, 0.0));
    let max_toi = 4.0;
    let solid = true;
    let filter = QueryFilter::default();

    let query_pipeline = world.query_pipeline_with_filter(filter);

    if let Some((handle, toi)) = query_pipeline.cast_ray(
        &ray, max_toi, solid
    ) {
        // The first collider hit has the handle `handle` and it hit after
        // the ray travelled a distance equal to `ray.dir * toi`.
        let hit_point = ray.point_at(toi); // Same as: `ray.origin + ray.dir * toi`
        println!("Collider {:?} hit at point {}", handle, hit_point);
    }


    if let Some((handle, intersection)) = query_pipeline.cast_ray_and_get_normal(
        &ray, max_toi, solid
    ) {
        // This is similar to `QueryPipeline::cast_ray` illustrated above except
        // that it also returns the normal of the collider shape at the hit point.
        let hit_point = ray.point_at(intersection.time_of_impact);
        let hit_normal = intersection.normal;
        println!("Collider {:?} hit at point {} with normal {}", handle, hit_point, hit_normal);
    }

    for (handle, _, intersection) in query_pipeline.intersect_ray(ray, max_toi, solid) {
        // Callback called on each collider hit by the ray.
        let hit_point = ray.point_at(intersection.time_of_impact);
        let hit_normal = intersection.normal;
        println!("Collider {:?} hit at point {} with normal {}", handle, hit_point, hit_normal);
    }
    // DOCUSAURUS: Raycast stop
}

#[rustfmt::skip]
fn shapecast_section(
    world: &PhysicsWorld,
) {
    // DOCUSAURUS: Shapecast start
    let shape = Cuboid::new(Vector::new(1.0, 2.0, 3.0));
    let shape_pos = Pose::new(Vector::new(0.0, 1.0, 0.0), Vector::new(0.2, 0.7, 0.1));
    let shape_vel = Vector::new(0.1, 0.4, 0.2);
    let max_toi = 4.0;
    let filter = QueryFilter::default();
    let options = ShapeCastOptions {
        max_time_of_impact: 4.0,
        target_distance: 0.0,
        stop_at_penetration: false,
        compute_impact_geometry_on_penetration: false,
    };

    let query_pipeline = world.query_pipeline_with_filter(filter);

    if let Some((handle, hit)) = query_pipeline.cast_shape(
        &shape_pos, shape_vel, &shape, options
    ) {
        // The first collider hit has the handle `handle`. The `hit` is a
        // structure containing details about the hit configuration.
        println!("Hit the collider {:?} with the configuration: {:?}", handle, hit);
    }
    // DOCUSAURUS: Shapecast stop
}

#[rustfmt::skip]
fn point_projection_section(
    world: &PhysicsWorld,
) {
    // DOCUSAURUS: PointProjection start
    let point = Vector::new(1.0, 2.0, 3.0);
    let solid = true;
    let max_dist = 12.0;
    let filter = QueryFilter::default();

    let query_pipeline = world.query_pipeline_with_filter(filter);
    
    if let Some((handle, projection)) = query_pipeline.project_point(
        point, max_dist, solid
    ) {
        // The collider closest to the point has this `handle`.
        println!("Projected point on collider {:?}. Point projection: {}", handle, projection.point);
        println!("Point was inside of the collider shape: {}", projection.is_inside);
    }
    
    for (handle, _) in query_pipeline.intersect_point(point) {
        // Callback called on each collider with a shape containing the point.
        println!("The collider {:?} contains the point.", handle);
    }
    // DOCUSAURUS: PointProjection stop
}

#[rustfmt::skip]
fn intersection_section(
    world: &PhysicsWorld,
) {
    // DOCUSAURUS: IntersectionTest start
    let shape = Cuboid::new(Vector::new(1.0, 2.0, 3.0));
    let shape_pos = Pose::new(Vector::new(0.0, 1.0, 0.0), Vector::new(0.2, 0.7, 0.1));
    let filter = QueryFilter::default();

    let query_pipeline = world.query_pipeline_with_filter(filter);

    for (handle, _) in query_pipeline.intersect_shape(shape_pos, &shape) {
        println!("The collider {:?} intersects our shape.", handle);
    }

    let aabb = Aabb::new(Vector::new(-1.0, -2.0, -3.0), Vector::new(1.0, 2.0, 3.0));
    for (handle, _) in query_pipeline.intersect_aabb_conservative(aabb) {
        println!("The collider {:?} has an AABB intersecting our test AABB", handle);
    }
    // DOCUSAURUS: IntersectionTest stop
}
