//! Port of box3d's `joint_grid` benchmark (`CreateJointGrid`,
//! `box3d/shared/benchmarks.c`). Release settings: a 100x100 grid of spheres
//! wired together with spherical joints; the `i == 0` column is static.
//! Sleeping disabled.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0, 0.0);

    let n = 100usize;
    let mut bodies = vec![RigidBodyHandle::invalid(); n * n];
    let mut index = 0usize;

    for k in 0..n {
        for i in 0..n {
            let fk = k as f32;
            let fi = i as f32;

            let body = if i == 0 {
                RigidBodyBuilder::fixed()
            } else {
                RigidBodyBuilder::dynamic().can_sleep(false)
            }
            .translation(Vector::new(fk, -fi, 0.0));
            let handle = world.insert_body(body);
            world.insert_collider(ColliderBuilder::ball(0.4), Some(handle));

            if i > 0 {
                // Spherical joint to the body above (previous i).
                let joint = SphericalJointBuilder::new()
                    .local_anchor1(Vector::new(0.0, -0.5, 0.0))
                    .local_anchor2(Vector::new(0.0, 0.5, 0.0));
                world.insert_impulse_joint(bodies[index - 1], handle, joint);
            }
            if k > 0 {
                // Spherical joint to the body in the previous column.
                let joint = SphericalJointBuilder::new()
                    .local_anchor1(Vector::new(0.5, 0.0, 0.0))
                    .local_anchor2(Vector::new(-0.5, 0.0, 0.0));
                world.insert_impulse_joint(bodies[index - n], handle, joint);
            }

            bodies[index] = handle;
            index += 1;
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(50.0, -25.0, 90.0), Vec3::new(50.0, -50.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
