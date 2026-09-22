//! The query pipeline's hits keep the hit collider's own sub-shape (a trimesh's triangle), and
//! the query filter still applies.

use rapier3d::parry::query::{NonlinearRigidMotion, ShapeCastOptions};
use rapier3d::prelude::*;

/// A ground trimesh of two triangles (`z <= x` is triangle 0, `z >= x` triangle 1) at
/// `x = 10`, facing +y, over a cuboid at `y = -5`.
fn scene() -> (PhysicsWorld, ColliderHandle, ColliderHandle) {
    let mut world = PhysicsWorld::new();
    let vertices = vec![
        Vector::new(-1.0, 0.0, -1.0),
        Vector::new(1.0, 0.0, -1.0),
        Vector::new(1.0, 0.0, 1.0),
        Vector::new(-1.0, 0.0, 1.0),
    ];
    let (_, mesh) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(10.0, 0.0, 0.0)),
        ColliderBuilder::trimesh(vertices, vec![[0, 2, 1], [0, 3, 2]]).unwrap(),
    );
    let (_, floor) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(10.0, -5.0, 0.0)),
        ColliderBuilder::cuboid(5.0, 0.5, 5.0),
    );
    // The broad phase indexes the colliders on the first step.
    world.step();
    (world, mesh, floor)
}

#[test]
fn ray_hits_report_the_triangle() {
    let (world, mesh, floor) = scene();
    let pipeline = world.query_pipeline();
    let down = Vector::new(0.0, -1.0, 0.0);
    for (x, z, triangle) in [(0.5, -0.5, 0), (-0.5, 0.5, 1)] {
        let ray = Ray::new(Vector::new(10.0 + x, 5.0, z), down);
        let (handle, hit) = pipeline
            .cast_ray_and_get_normal(&ray, Real::MAX, true)
            .expect("the ray hits the mesh");
        assert_eq!(handle, mesh);
        assert_eq!(hit.subshape, triangle, "wrong triangle");
        assert!((hit.time_of_impact - 5.0).abs() < 1.0e-4);
        let trimesh = world.colliders[mesh].shape().as_trimesh().unwrap();
        assert!(!trimesh.is_backface(hit.feature));
    }
    // From below, the same triangle is hit on its back face.
    let ray = Ray::new(Vector::new(10.5, -2.0, -0.5), -down);
    let (handle, hit) = pipeline
        .cast_ray_and_get_normal(&ray, Real::MAX, true)
        .unwrap();
    assert_eq!(handle, mesh);
    assert_eq!(hit.subshape, 0);
    let trimesh = world.colliders[mesh].shape().as_trimesh().unwrap();
    assert!(trimesh.is_backface(hit.feature));
    // The filter still excludes colliders: the ray passes on to the floor.
    let filter = QueryFilter::default().exclude_collider(mesh);
    let ray = Ray::new(Vector::new(10.5, 5.0, -0.5), down);
    let (handle, hit) = world
        .query_pipeline_with_filter(filter)
        .cast_ray_and_get_normal(&ray, Real::MAX, true)
        .unwrap();
    assert_eq!(handle, floor);
    assert_eq!(hit.subshape, 0);
    assert!((hit.time_of_impact - 9.5).abs() < 1.0e-4);
}

#[test]
fn point_projections_report_the_triangle() {
    let (world, mesh, _) = scene();
    let pipeline = world.query_pipeline();
    for (x, z, triangle) in [(0.5, -0.5, 0), (-0.5, 0.5, 1)] {
        let point = Vector::new(10.0 + x, 0.3, z);
        let (handle, proj) = pipeline
            .project_point(point, Real::MAX, true)
            .expect("the mesh is the closest collider");
        assert_eq!(handle, mesh);
        assert_eq!(proj.subshape, triangle, "wrong triangle");
        assert!((proj.point - Vector::new(10.0 + x, 0.0, z)).length() < 1.0e-4);
        let (handle, proj, feature) = pipeline
            .project_point_and_get_feature(point, Real::MAX)
            .unwrap();
        assert_eq!(handle, mesh);
        assert_eq!(proj.subshape, triangle);
        assert_eq!(feature, FeatureId::Face(0), "the triangle's own feature");
    }
}

#[test]
fn shape_casts_report_the_triangle() {
    let (world, mesh, _) = scene();
    let pipeline = world.query_pipeline();
    let ball = Ball::new(0.25);
    for (x, z, triangle) in [(0.5, -0.5, 0), (-0.5, 0.5, 1)] {
        let start = Pose::from_translation(Vector::new(10.0 + x, 5.0, z));
        let (handle, hit) = pipeline
            .cast_shape(
                &start,
                Vector::new(0.0, -1.0, 0.0),
                &ball,
                ShapeCastOptions::default(),
            )
            .expect("the ball hits the mesh");
        assert_eq!(handle, mesh);
        assert_eq!((hit.subshape1, hit.subshape2), (triangle, 0));
        assert!((hit.time_of_impact - 4.75).abs() < 1.0e-3);

        let motion = NonlinearRigidMotion::new(
            start,
            Vector::ZERO,
            Vector::new(0.0, -1.0, 0.0),
            Vector::ZERO,
        );
        let (handle, hit) = pipeline
            .cast_shape_nonlinear(&motion, &ball, 0.0, 10.0, true)
            .expect("the ball hits the mesh");
        assert_eq!(handle, mesh);
        assert_eq!((hit.subshape1, hit.subshape2), (triangle, 0));
        assert!((hit.time_of_impact - 4.75).abs() < 1.0e-2);
    }
}
