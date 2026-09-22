//! Jelly (2D): triangulated squares of increasing stiffness stacked in a pyramid, a bridge of
//! soft slabs pinned at both ends supporting rigid boxes, and a shape-matched blob driven along a
//! path.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(20.0, 0.5),
    );

    /*
     * A pyramid of jelly squares, softer toward the top.
     */
    let levels = 4;
    for level in 0..levels {
        let young = 2.0e4 / (1.0 + level as Real * 1.5);
        for i in 0..(levels - level) {
            let x = -8.0 + (i as Real - (levels - level) as Real * 0.5 + 0.5) * 1.6;
            let y = 0.75 + level as Real * 1.5;
            let square = SoftBodyBuilder::grid(Vector::new(x, y), Vector::splat(0.6), 5, 5)
                .cell_model(SoftBodyCellModel::Corotational)
                .material(SoftBodyMaterial {
                    young_modulus: young,
                    poisson_ratio: 0.4,
                    elastic_damping_ratio: 0.5,
                    ..Default::default()
                })
                .particle_mass(0.1);
            world.insert_soft_body(square);
        }
    }

    /*
     * A soft bridge pinned at both ends, with rigid boxes dropped on it.
     */
    let bridge = SoftBodyBuilder::grid(Vector::new(3.0, 3.0), Vector::new(4.0, 0.2), 41, 3)
        .cell_model(SoftBodyCellModel::Volume)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .pinned_particles([0, 1, 2, 120, 121, 122])
        .particle_mass(0.05);
    world.insert_soft_body(bridge);
    for i in 0..6 {
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(0.5 + i as Real, 5.0 + i as Real)),
            ColliderBuilder::cuboid(0.25, 0.25),
        );
    }

    /*
     * A shape-matched blob driven along a circle: a squishy kinematic body pushing rigid boxes.
     */
    let driven = SoftBodyBuilder::disk(Vector::new(8.0, 4.0), 0.8, 24)
        .shape_matching(true)
        .softness(SpringCoefficients::new(15.0, 1.0))
        .volume_preservation(false)
        .gravity_scale(0.0)
        .particle_mass(0.2)
        .can_sleep(false);
    let driven = world.insert_soft_body(driven);
    for i in 0..8 {
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(6.5 + 0.5 * i as Real, 0.25)),
            ColliderBuilder::cuboid(0.2, 0.2),
        );
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 3.0), 30.0);

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            t += world.integration_parameters.dt;
            let center = Vector::new(8.0 + 2.5 * t.cos(), 1.0 + 1.5 * (2.0 * t).sin().abs());
            world.soft_bodies[driven]
                .cluster_mut(0)
                .unwrap()
                .set_shape_matching_target(Some(Pose::from_parts(center, Rotation::new(t))));
            world.step();
        }
    }
    Ok(())
}
