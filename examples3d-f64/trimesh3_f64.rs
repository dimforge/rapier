use rapier_testbed3d::TestbedViewer;
use rapier3d::glamx::{DVec3, Vec3};
use rapier3d::na::ComplexField;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = DVec3::new(200.0, 1.0, 200.0);
    let nsubdivs = 20;

    let heights = Array2::from_fn(nsubdivs + 1, nsubdivs + 1, |i, j| {
        if i == 0 || i == nsubdivs || j == 0 || j == nsubdivs {
            10.0
        } else {
            let x = i as f64 * ground_size.x / (nsubdivs as f64);
            let z = j as f64 * ground_size.z / (nsubdivs as f64);

            // NOTE: make sure we use the sin/cos from simba to ensure
            // cross-platform determinism of the example when the
            // enhanced_determinism feature is enabled.
            <f64 as ComplexField>::sin(x) + <f64 as ComplexField>::cos(z)
        }
    });

    // Here we will build our trimesh from the mesh representation of an
    // heightfield.
    let heightfield = HeightField::new(heights, ground_size);
    let (vertices, indices) = heightfield.to_trimesh();

    let rigid_body = RigidBodyBuilder::fixed();
    let handle = world.bodies.insert(rigid_body);
    let collider = ColliderBuilder::trimesh(vertices, indices).unwrap();
    world
        .colliders
        .insert_with_parent(collider, handle, &mut world.bodies);

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f64;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f64;

    for j in 0usize..47 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f64 * shift - centerx;
                let y = j as f64 * shift + centery + 3.0;
                let z = k as f64 * shift - centerz;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(DVec3::new(x, y, z));
                let handle = world.bodies.insert(rigid_body);

                if j % 2 == 0 {
                    let collider = ColliderBuilder::cuboid(rad, rad, rad);
                    world
                        .colliders
                        .insert_with_parent(collider, handle, &mut world.bodies);
                } else {
                    let collider = ColliderBuilder::ball(rad);
                    world
                        .colliders
                        .insert_with_parent(collider, handle, &mut world.bodies);
                }
            }
        }
    }

    /*
     * Set up the viewer.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(100.0, 100.0, 100.0), Vec3::ZERO);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }

    Ok(())
}
