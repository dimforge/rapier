//! Jelly: a stack of tetrahedral cubes of increasing stiffness, balloons piled in a box, a
//! shape-matched cube animated toward a moving pose, and tetrahedralized (ball, capsule) jelly
//! bodies.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground and a container for the balloons.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(12.0, 0.1, 12.0),
    );
    for (dx, dz, hx, hz) in [
        (-2.5, 0.0, 0.1, 2.5),
        (2.5, 0.0, 0.1, 2.5),
        (0.0, -2.5, 2.5, 0.1),
        (0.0, 2.5, 2.5, 0.1),
    ] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(dx + 5.0, 1.0, dz)),
            ColliderBuilder::cuboid(hx, 1.0, hz),
        );
    }

    /*
     * A stack of elastic cubes, softer at the top.
     */
    for (i, young) in [1.0e4, 4.0e3, 1.5e3].iter().enumerate() {
        let cube = SoftBodyBuilder::cuboid(
            Vector::new(-4.0, 0.7 + 1.5 * i as Real, 0.0),
            Vector::splat(0.6),
            5,
            5,
            5,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: *young,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.1);
        world.insert_soft_body(cube);
    }

    /*
     * Balloons dropped in the container.
     */
    for i in 0..6 {
        let x = 5.0 + ((i % 3) as Real - 1.0) * 1.2;
        let z = ((i / 3) as Real - 0.5) * 1.2;
        let balloon = SoftBodyBuilder::sphere(Vector::new(x, 2.0 + i as Real * 1.5, z), 0.6, 2)
            .softness(SpringCoefficients::new(15.0, 1.0))
            .volume_factor(1.1)
            .particle_mass(0.03);
        world.insert_soft_body(balloon);
    }

    /*
     * A shape-matched particle cube driven toward an animated pose: it behaves like a squishy
     * kinematic body.
     */
    let driven = SoftBodyBuilder::cuboid(Vector::splat(0.45), Vector::splat(0.45), 4, 4, 4)
        .shape_matching(true)
        .softness(SpringCoefficients::new(15.0, 1.0))
        .particle_radius(0.1)
        .particle_mass(0.2)
        .gravity_scale(0.0)
        .can_sleep(false);
    let driven = world.insert_soft_body(driven);

    /*
     * Volumetric jelly: a ball and a capsule filled with tetrahedral cells from their boundary
     * meshes (`SoftBodyBuilder::volumetric`), dropped behind the stack.
     */
    let jelly = |builder: SoftBodyBuilder, young: Real| {
        builder
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: young,
                poisson_ratio: 0.35,
                elastic_damping_ratio: 0.5,
                ..Default::default()
            })
            .particle_mass(0.05)
            .surface_collider(ColliderBuilder::ball(0.1).friction(0.7))
    };
    let (vertices, indices) = Ball::new(0.7).to_trimesh(16, 16);
    let vertices: Vec<Vector> = vertices
        .iter()
        .map(|p| *p + Vector::new(-4.0, 3.0, -4.0))
        .collect();
    if let Some(ball) = SoftBodyBuilder::volumetric(&vertices, &indices, 0.2) {
        world.insert_soft_body(jelly(ball, 1.0e4));
    }
    let (vertices, indices) = Capsule::new_y(0.6, 0.45).to_trimesh(12, 12);
    let vertices: Vec<Vector> = vertices
        .iter()
        .map(|p| Rotation::from_axis_angle(Vector::Z, 1.2) * *p + Vector::new(-4.0, 6.0, -4.0))
        .collect();
    if let Some(capsule) = SoftBodyBuilder::volumetric(&vertices, &indices, 0.2) {
        world.insert_soft_body(jelly(capsule, 3.0e4));
    }

    /*
     * Rigid boxes to be shoved around by the driven cube.
     */
    for i in 0..8 {
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(
                -0.5 + (i % 4) as Real * 0.5,
                0.25,
                4.0 + (i / 4) as Real * 0.5,
            )),
            ColliderBuilder::cuboid(0.2, 0.2, 0.2),
        );
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(8.0, 7.0, 14.0), Vec3::new(0.0, 1.5, 1.0));

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            t += world.integration_parameters.dt;
            // Circle around the rigid boxes at height 0.6, spinning.
            let center = Vector::new(2.0 * t.cos(), 0.6, 4.0 + 2.0 * t.sin());
            let rotation = Rotation::from_axis_angle(Vector::Y, 2.0 * t);
            world.soft_bodies[driven]
                .cluster_mut(0)
                .unwrap()
                .set_shape_matching_target(Some(Pose::from_parts(center, rotation)));
            world.step();
        }
    }
    Ok(())
}
