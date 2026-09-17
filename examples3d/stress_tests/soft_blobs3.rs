//! Stress test: hundreds of pressurized balloons poured into a tall bin, and a long floppy cloth
//! strip with self-contacts dropped onto the pile. Soft-vs-soft contacts through deformable
//! surfaces, under the weight of the pile.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    /*
     * Bin.
     */
    let half = 7.0;
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(half + 0.5, 0.5, half + 0.5),
    );
    // Half-height of the walls: enough for the settled pile, not for the drop column.
    let wall_height = 4.0;
    for (dx, dz) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
        let (hx, hz) = if dx != 0.0 {
            (0.25, half + 0.5)
        } else {
            (half + 0.5, 0.25)
        };
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(
                dx * (half + 0.25),
                wall_height,
                dz * (half + 0.25),
            )),
            ColliderBuilder::cuboid(hx, wall_height, hz),
        );
    }

    /*
     * Balloons of three sizes.
     */
    for layer in 0..8 {
        for i in 0..7 {
            for j in 0..7 {
                let radius = 0.45 + 0.1 * ((i + j + layer) % 3) as Real;
                let x = -5.7 + i as Real * 1.9 + (layer % 2) as Real * 0.4;
                let z = -5.7 + j as Real * 1.9 + (layer % 3) as Real * 0.3;
                let y = 2.0 + layer as Real * 2.0;
                let balloon = SoftBodyBuilder::sphere(Vector::new(x, y, z), radius, 1)
                    .softness(SpringCoefficients::new(20.0, 1.0))
                    .volume_factor(1.1)
                    .particle_mass(0.03)
                    .particle_radius(0.08)
                    .surface_collider(ColliderBuilder::ball(0.08).friction(0.6));
                world.insert_soft_body(balloon);
            }
        }
    }

    /*
     * A long floppy strip with self-contacts, dropped on the pile.
     */
    let strip = SoftBodyBuilder::cloth(
        Vector::new(-4.0, 28.0, -0.6),
        Vector::new(0.15, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.15),
        54,
        9,
    )
    .softness(SpringCoefficients::new(30.0, 1.0))
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
    })
    .self_contacts(true)
    .particle_mass(0.02)
    .particle_radius(0.05)
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.5));
    world.insert_soft_body(strip);

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    // Translucent bin, to see the pile inside.
    let bin: Vec<_> = world
        .bodies
        .iter()
        .filter(|(_, body)| body.is_fixed())
        .map(|(handle, _)| handle)
        .collect();
    for handle in bin {
        viewer.set_body_color(handle, [0.6, 0.7, 1.0, 0.3].into(), false);
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(20.0, 18.0, 20.0), Vec3::new(0.0, 6.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
