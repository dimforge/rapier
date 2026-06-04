use bevy::math::bounding::Aabb3d;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(FixedUpdate, cast_ray)
        .add_systems(FixedUpdate, cast_shape)
        .add_systems(FixedUpdate, project_point)
        .add_systems(FixedUpdate, test_intersections)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    /* Create the ground. */
    commands
        .spawn(Collider::cuboid(100.0, 0.1, 100.0))
        .insert(Transform::from_xyz(0.0, -2.0, 0.0));

    /* Create the bouncing ball. */
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::ball(0.5))
        .insert(Restitution::coefficient(0.7))
        .insert(Transform::from_xyz(0.0, 4.0, 0.0));
}

// DOCUSAURUS: Raycast start
/* Cast a ray inside of a system. */
fn cast_ray(rapier_context: ReadRapierContext) {
    let rapier_context = rapier_context.single().unwrap();
    let ray_pos = Vec3::new(1.0, 2.0, 3.0);
    let ray_dir = Vec3::new(0.0, 1.0, 0.0);
    let max_toi = 4.0;
    let solid = true;
    let filter = QueryFilter::default();

    if let Some((entity, toi)) = rapier_context.cast_ray(ray_pos, ray_dir, max_toi, solid, filter) {
        // The first collider hit has the entity `entity` and it hit after
        // the ray travelled a distance equal to `ray_dir * toi`.
        let hit_point = ray_pos + ray_dir * toi;
        println!("Entity {:?} hit at point {}", entity, hit_point);
    }

    if let Some((entity, intersection)) =
        rapier_context.cast_ray_and_get_normal(ray_pos, ray_dir, max_toi, solid, filter)
    {
        // This is similar to `QueryPipeline::cast_ray` illustrated above except
        // that it also returns the normal of the collider shape at the hit point.
        let hit_point = intersection.point;
        let hit_normal = intersection.normal;
        println!(
            "Entity {:?} hit at point {} with normal {}",
            entity, hit_point, hit_normal
        );
    }

    rapier_context.intersections_with_ray(
        ray_pos,
        ray_dir,
        max_toi,
        solid,
        filter,
        |entity, intersection| {
            // Callback called on each collider hit by the ray.
            let hit_point = intersection.point;
            let hit_normal = intersection.normal;
            println!(
                "Entity {:?} hit at point {} with normal {}",
                entity, hit_point, hit_normal
            );
            true // Return `false` instead if we want to stop searching for other hits.
        },
    );
}
// DOCUSAURUS: Raycast stop

// DOCUSAURUS: Shapecast start
/* Cast a shape inside of a system. */
fn cast_shape(rapier_context: ReadRapierContext) {
    let rapier_context = rapier_context.single().unwrap();
    let shape = Collider::cuboid(1.0, 2.0, 3.0);
    let shape_pos = Vec3::new(1.0, 2.0, 3.0);
    let shape_rot = Quat::from_rotation_z(0.8);
    let shape_vel = Vec3::new(0.1, 0.4, 0.2);
    let filter = QueryFilter::default();
    let options = ShapeCastOptions {
        max_time_of_impact: 4.0,
        target_distance: 0.0,
        stop_at_penetration: false,
        compute_impact_geometry_on_penetration: false,
    };

    if let Some((entity, hit)) =
        rapier_context.cast_shape(shape_pos, shape_rot, shape_vel, &shape, options, filter)
    {
        // The first collider hit has the entity `entity`. The `hit` is a
        // structure containing details about the hit configuration.
        println!(
            "Hit the entity {:?} with the configuration: {:?}",
            entity, hit
        );
    }
}
// DOCUSAURUS: Shapecast stop

// DOCUSAURUS: PointProjection start
/* Project a point inside of a system. */
fn project_point(rapier_context: ReadRapierContext) {
    let rapier_context = rapier_context.single().unwrap();
    let point = Vec3::new(1.0, 2.0, 3.0);
    let solid = true;
    let filter = QueryFilter::default();

    if let Some((entity, projection)) = rapier_context.project_point(point, solid, filter) {
        // The collider closest to the point has this `handle`.
        println!(
            "Projected point on entity {:?}. Point projection: {}",
            entity, projection.point
        );
        println!(
            "Point was inside of the collider shape: {}",
            projection.is_inside
        );
    }

    rapier_context.intersections_with_point(point, filter, |entity| {
        // Callback called on each collider with a shape containing the point.
        println!("The entity {:?} contains the point.", entity);
        // Return `false` instead if we want to stop searching for other colliders containing this point.
        true
    });
}
// DOCUSAURUS: PointProjection stop

// DOCUSAURUS: IntersectionTest start
/* Test intersections inside of a system. */
fn test_intersections(rapier_context: ReadRapierContext) {
    let rapier_context = rapier_context.single().unwrap();
    let shape = Collider::cuboid(1.0, 2.0, 3.0);
    let shape_pos = Vec3::new(0.0, 1.0, 2.0);
    let shape_rot = Quat::from_rotation_z(0.8);
    let filter = QueryFilter::default();

    rapier_context.intersections_with_shape(shape_pos, shape_rot, &shape, filter, |entity| {
        println!("The entity {:?} intersects our shape.", entity);
        true // Return `false` instead if we want to stop searching for other colliders that contain this point.
    });

    let aabb = Aabb3d::new(Vec3::new(-1.0, -2.0, -3.0), Vec3::new(1.0, 2.0, 3.0));
    rapier_context.colliders_with_aabb_intersecting_aabb(aabb, |entity| {
        println!(
            "The entity {:?} has an AABB intersecting our test AABB",
            entity
        );
        true // Return `false` instead if we want to stop searching for other colliders that contain this point.
    });
}
// DOCUSAURUS: IntersectionTest stop
