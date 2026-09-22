//! Stress test: a Keva tower of seven hundred planks with a square of cloth sandwiched between
//! its chunks (the runs of rows sharing a footprint) and a bigger one dropped flat on top, all
//! with self-contacts on. The tower drapes, slides and topples through the cloths.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// The scene alone, so it can be stepped without the testbed.
pub fn build_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(50.0, 0.1, 50.0),
    );

    // A square of cloth of the given side centered on the tower axis at height `y`, coarse
    // enough (`spacing`) to stay cheap.
    let radius = 0.15;
    let cloth = |side: Real, spacing: Real, y: Real| {
        let n = (side / spacing).round() as usize + 1;
        let width = (n - 1) as Real * spacing;
        SoftBodyBuilder::cloth(
            Vector::new(-width * 0.5, y, -width * 0.5),
            Vector::new(spacing, 0.0, 0.0),
            Vector::new(0.0, 0.0, spacing),
            n,
            n,
        )
        .softness(SpringCoefficients::new(30.0, 1.0))
        .material(SoftBodyMaterial {
            bend_softness: SpringCoefficients::new(3.0, 1.0),
            ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
        })
        .self_contacts(true)
        .particle_mass(0.05)
        .particle_radius(radius)
        .surface_collider(ColliderBuilder::ball(radius).friction(0.6))
    };

    /*
     * The tower: the three lower chunks of the Keva demo, a cloth sandwiched between chunks.
     */
    let half_extents = Vector::new(0.02, 0.1, 0.4) / 2.0 * 10.0;
    let numy = [0, 9, 13, 17];
    let mut tower_height = 0.0;
    for i in (1..=3).rev() {
        let numx = i;
        let numy = numy[i];
        let numz = numx * 3 + 1;
        let block_width = numx as f32 * half_extents.z * 2.0;
        crate::keva3::build_block(
            &mut world,
            half_extents,
            Vector::new(-block_width / 2.0, tower_height, -block_width / 2.0),
            (numx, numy, numz),
        );
        tower_height += numy as f32 * half_extents.y * 2.0 + half_extents.x * 2.0;

        if i > 1 {
            // Overhangs the chunk below; the chunk above rests on it.
            world.insert_soft_body(cloth(block_width + 4.0, 0.5, tower_height + radius));
            tower_height += 2.0 * radius;
        }
    }

    /*
     * The big cloth dropped on top.
     */
    world.insert_soft_body(cloth(18.0, 0.75, tower_height + 6.0));

    world
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = build_world();

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(50.0, 50.0, 50.0), Vec3::new(0.0, 15.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
