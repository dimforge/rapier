use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * An 8x8 grid of hanging ropes, 60 capsule segments each (3840 dynamic
     * bodies, one spherical joint per segment), swinging from an initial
     * sideways kick. Nearly contact-free: a joint-solver stress test.
     */
    let segments = 60;
    let seg_len = 1.0;

    for i in 0..8 {
        for k in 0..8 {
            let top = Vec3::new(i as f32 * 4.0, 0.0, k as f32 * 4.0);
            let mut parent = world
                .bodies
                .insert(RigidBodyBuilder::fixed().translation(top));

            for s in 0..segments {
                let center = top + Vec3::new(0.0, -(s as f32 + 0.5) * seg_len, 0.0);
                let body = RigidBodyBuilder::dynamic()
                    .translation(center)
                    .linvel(Vec3::new(2.0, 0.0, 0.0));
                let handle = world.bodies.insert(body);
                world.colliders.insert_with_parent(
                    ColliderBuilder::capsule_y(0.35, 0.1),
                    handle,
                    &mut world.bodies,
                );

                let anchor1 = if s == 0 {
                    Vec3::ZERO
                } else {
                    Vec3::new(0.0, -seg_len / 2.0, 0.0)
                };
                let joint = SphericalJointBuilder::new()
                    .local_anchor1(anchor1)
                    .local_anchor2(Vec3::new(0.0, seg_len / 2.0, 0.0))
                    .contacts_enabled(false);
                world.insert_impulse_joint(parent, handle, joint);
                parent = handle;
            }
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(-45.0, -10.0, -45.0), Vec3::new(14.0, -30.0, 14.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
