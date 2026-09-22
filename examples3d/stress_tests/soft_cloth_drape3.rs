//! Stress test: one large sheet of cloth (ten thousand particles, self-contacts on) dropped over a field of rigid obstacles, then loaded with a shower of heavy balls.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Ground and obstacles.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(12.0, 0.1, 12.0),
    );
    for i in 0..4 {
        for j in 0..4 {
            let x = -4.5 + i as Real * 3.0;
            let z = -4.5 + j as Real * 3.0;
            let body = RigidBodyBuilder::fixed().translation(Vector::new(x, 1.0, z));
            match (i + j) % 3 {
                0 => world.insert(body, ColliderBuilder::ball(0.8)),
                1 => world.insert(body, ColliderBuilder::cuboid(0.6, 1.0, 0.6)),
                _ => world.insert(body, ColliderBuilder::capsule_y(0.6, 0.4)),
            };
        }
    }

    /*
     * The sheet.
     */
    let n = 101;
    let sheet = SoftBodyBuilder::cloth(
        Vector::new(-6.0, 3.5, -6.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    .softness(SpringCoefficients::new(30.0, 1.0))
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
    })
    .self_contacts(true)
    .particle_mass(0.02)
    .particle_radius(0.05)
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
    world.insert_soft_body(sheet);

    /*
     * Heavy balls dropped on the draped sheet.
     */
    for i in 0..5 {
        for j in 0..5 {
            let x = -4.0 + i as Real * 2.0 + (j % 2) as Real * 0.5;
            let z = -4.0 + j as Real * 2.0;
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(
                    x,
                    6.0 + (i + j) as Real * 0.5,
                    z,
                )),
                ColliderBuilder::ball(0.4).density(2.0),
            );
        }
    }

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(14.0, 10.0, 14.0), Vec3::new(0.0, 1.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
