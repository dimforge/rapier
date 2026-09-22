//! Plasticity: elastic cells whose rest shape flows once their strain exceeds a yield
//! (`SoftBodyMaterial::plastic_yield` / `plastic_creep`): blocks of increasing plasticity hit by
//! heavy balls, a stamped clay slab, a creeping column beside an elastic one, clay balls on a wall.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground and a wall for the clay balls.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(14.0, 0.1, 14.0),
    );
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(9.0, 2.0, 5.0)),
        ColliderBuilder::cuboid(0.2, 2.0, 3.0).friction(0.8),
    );

    let clay = |young: Real, plastic_yield: Real, plastic_creep: Real| SoftBodyMaterial {
        young_modulus: young,
        poisson_ratio: 0.35,
        elastic_damping_ratio: 1.0,
        plastic_yield,
        plastic_creep,
        deformation_damping: 4.0,
        ..Default::default()
    };

    /*
     * The yield ladder: identical blocks, from purely elastic (left) to very plastic (right),
     * each hit by the same heavy ball.
     */
    for (i, plastic_yield) in [0.0, 0.2, 0.08, 0.02].into_iter().enumerate() {
        let x = -7.0 + i as Real * 2.6;
        let block = SoftBodyBuilder::cuboid(Vector::new(x, 0.6, -4.0), Vector::splat(0.6), 5, 5, 5)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(clay(1.0e4, plastic_yield, 20.0))
            .particle_mass(0.1)
            .surface_collider(ColliderBuilder::ball(0.12).friction(0.8));
        world.insert_soft_body(block);
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(x, 5.0, -4.0)),
            ColliderBuilder::ball(0.4).density(5.0),
        );
    }

    /*
     * A clay slab stamped by a kinematic press: the imprints stay after the press lifts.
     */
    let slab = SoftBodyBuilder::cuboid(
        Vector::new(0.0, 0.4, 1.5),
        Vector::new(2.4, 0.4, 1.4),
        13,
        3,
        8,
    )
    .cell_model(SoftBodyCellModel::Corotational)
    .material(clay(3.0e4, 0.02, 50.0))
    .particle_mass(0.1)
    .surface_collider(ColliderBuilder::ball(0.15).friction(0.8));
    world.insert_soft_body(slab);
    let press_rest = Vector::new(-1.5, 2.2, 1.5);
    let (press, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(press_rest),
        ColliderBuilder::cuboid(0.35, 0.35, 0.35)
            .rotation(Vector::new(0.0, 0.0, core::f32::consts::FRAC_PI_4))
            .friction(0.5),
    );

    /*
     * Two soft columns under their own weight: the elastic one stands, the plastic one creeps,
     * leans and collapses into a heap (a body whose cells keep flowing is kept awake).
     */
    for (i, plastic_yield) in [0.0, 0.05].into_iter().enumerate() {
        let x = -6.0 + i as Real * 2.0;
        let column = SoftBodyBuilder::cuboid(
            Vector::new(x, 1.2, 5.5),
            Vector::new(0.3, 1.2, 0.3),
            3,
            12,
            3,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e3,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 1.0,
            plastic_yield,
            plastic_creep: 0.5,
            deformation_damping: 4.0,
            ..Default::default()
        })
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.1).friction(1.0));
        world.insert_soft_body(column);
    }

    /*
     * Clay balls thrown at the wall.
     */
    let (vertices, indices) = Ball::new(0.5).to_trimesh(12, 12);
    for i in 0..3 {
        let center = Vector::new(
            4.0 - i as Real * 1.5,
            2.0 + i as Real * 0.3,
            3.5 + i as Real * 1.5,
        );
        let vertices: Vec<Vector> = vertices.iter().map(|p| *p + center).collect();
        if let Some(ball) = SoftBodyBuilder::volumetric(&vertices, &indices, 0.2) {
            let ball = ball
                .cell_model(SoftBodyCellModel::Corotational)
                .material(clay(1.0e4, 0.03, 60.0))
                .particle_mass(0.05)
                .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
            let handle = world.insert_soft_body(ball);
            let sb = &mut world.soft_bodies[handle];
            for k in 0..sb.num_particles() {
                sb.set_particle_velocity(k, Vector::new(10.0, 1.0, 0.0));
            }
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(4.0, 9.0, 16.0), Vec3::new(0.0, 0.5, 0.0));

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            t += world.integration_parameters.dt;
            // The press stamps a new spot every 3 seconds: down for a second, up for a second,
            // then slides to the next spot.
            let period = 3.0;
            let cycle = (t / period).floor();
            let phase = t - cycle * period;
            let x = press_rest.x + (cycle % 4.0) * 1.0;
            let depth = 1.0;
            let y = press_rest.y
                - if phase < 1.0 {
                    depth * phase
                } else if phase < 2.0 {
                    depth * (2.0 - phase)
                } else {
                    0.0
                };
            world.bodies[press].set_next_kinematic_translation(Vector::new(x, y, press_rest.z));
            world.step();
        }
    }
    Ok(())
}
