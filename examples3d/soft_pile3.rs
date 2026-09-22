//! Stress test: a big pile of jelly cubes, pressurized balloons and ropes poured into a bin with
//! a few rigid crates, mostly soft-vs-soft contacts under load. Cloth is left out: crumpled
//! cloth pinched in a pile does not settle yet.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Bin.
     */
    let half = 5.0;
    let (h, _) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(half + 0.5, 0.5, half + 0.5),
    );
    viewer.set_body_color(h, [0.6, 0.7, 1.0, 0.3].into(), false);
    for (dx, dz) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
        let (hx, hz) = if dx != 0.0 {
            (0.25, half + 0.5)
        } else {
            (half + 0.5, 0.25)
        };
        let (h, _) = world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(
                dx * (half + 0.25),
                4.0,
                dz * (half + 0.25),
            )),
            ColliderBuilder::cuboid(hx, 4.0, hz),
        );
        viewer.set_body_color(h, [0.6, 0.7, 1.0, 0.3].into(), false);
    }

    /*
     * Layers of soft objects, dropped from increasing heights.
     */
    let mut k = 0usize;
    for layer in 0..40 {
        for i in 0..3 {
            for j in 0..3 {
                let x = -3.0 + i as Real * 3.0 + (layer % 2) as Real * 0.7;
                let z = -3.0 + j as Real * 3.0 + (layer % 3) as Real * 0.5;
                let y = 3.0 + layer as Real * 2.5;
                match k % 5 {
                    0 | 2 => {
                        // Pressurized balloons.
                        let balloon = SoftBodyBuilder::sphere(Vector::new(x, y, z), 0.6, 1)
                            .softness(SpringCoefficients::new(15.0, 1.0))
                            .volume_factor(1.15)
                            .particle_mass(0.03)
                            .particle_radius(0.08)
                            .surface_collider(ColliderBuilder::ball(0.08).friction(0.6));
                        world.insert_soft_body(balloon);
                    }
                    1 | 3 => {
                        // Smaller, softer balloons.
                        let balloon = SoftBodyBuilder::sphere(Vector::new(x, y, z), 0.45, 1)
                            .softness(SpringCoefficients::new(10.0, 1.0))
                            .volume_factor(1.3)
                            .particle_mass(0.03)
                            .particle_radius(0.07)
                            .surface_collider(ColliderBuilder::ball(0.07).friction(0.6));
                        world.insert_soft_body(balloon);
                    }
                    _ => {
                        // Rigid crates in the mix.
                        world.insert(
                            RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z)),
                            ColliderBuilder::cuboid(0.4, 0.4, 0.4).density(0.5),
                        );
                    }
                }
                k += 1;
            }
        }
    }

    /*
     * A couple of ropes thrown on top.
     */
    for i in 0..3 {
        let z = -2.0 + i as Real * 2.0;
        let rope = SoftBodyBuilder::rope(
            Vector::new(-3.5, 24.0 + i as Real, z),
            Vector::new(3.5, 24.0 + i as Real, z + 0.5),
            30,
        )
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.03)
        .surface_collider(ColliderBuilder::ball(0.08).friction(0.6));
        world.insert_soft_body(rope);
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(14.0, 12.0, 14.0), Vec3::new(0.0, 4.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
