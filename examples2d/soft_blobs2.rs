//! Many pressurized blobs poured into a container, with self-contacts enabled on one of them
//! so it can fold onto itself.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Container.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(6.0, 0.5),
    );
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(-6.0, 6.0)),
        ColliderBuilder::cuboid(0.5, 6.0),
    );
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(6.0, 6.0)),
        ColliderBuilder::cuboid(0.5, 6.0),
    );

    /*
     * Blobs.
     */
    let mut k = 0;
    for j in 0..15 {
        for i in 0..5 {
            let radius = 0.45 + 0.1 * ((i + j) % 3) as Real;
            let x = -4.0 + i as Real * 2.0 + (j % 2) as Real * 0.5;
            let y = 3.0 + j as Real * 2.0;
            // Self-contacts keep a hard squeeze from pushing the loop through itself (a
            // crossed loop is a degenerate pressure vessel that stays crushed under the pile).
            let blob = SoftBodyBuilder::disk(Vector::new(x, y), radius, 20)
                .softness(SpringCoefficients::new(20.0, 1.0))
                .volume_factor(1.05)
                .self_contacts(true)
                .particle_mass(0.05);
            world.insert_soft_body(blob);
            k += 1;
        }
    }
    let _ = k;

    /*
     * A long floppy strip with self-contacts, dropped on the pile.
     */
    let strip = SoftBodyBuilder::grid(Vector::new(0.0, 16.0), Vector::new(3.0, 0.15), 40, 3)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .self_contacts(true)
        .particle_mass(0.02);
    world.insert_soft_body(strip);

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 6.0), 30.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
