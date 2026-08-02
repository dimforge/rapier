use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * 64 hanging ropes of 60 capsule segments each (3840 dynamic bodies, one
     * revolute joint per segment), swinging from an initial sideways kick.
     * Nearly contact-free: a joint-solver stress test.
     */
    let segments = 60;
    let seg_len = 1.0;

    for i in 0..64 {
        let top = Vec2::new(i as f32 * 4.0, 0.0);
        let mut parent = world
            .bodies
            .insert(RigidBodyBuilder::fixed().translation(top));

        for s in 0..segments {
            let center = top + Vec2::new(0.0, -(s as f32 + 0.5) * seg_len);
            let body = RigidBodyBuilder::dynamic()
                .translation(center)
                .linvel(Vec2::new(2.0, 0.0));
            let handle = world.bodies.insert(body);
            world.colliders.insert_with_parent(
                ColliderBuilder::capsule_y(0.35, 0.1),
                handle,
                &mut world.bodies,
            );

            let anchor1 = if s == 0 {
                Vec2::ZERO
            } else {
                Vec2::new(0.0, -seg_len / 2.0)
            };
            let joint = RevoluteJointBuilder::new()
                .local_anchor1(anchor1)
                .local_anchor2(Vec2::new(0.0, seg_len / 2.0))
                .contacts_enabled(false);
            world.insert_impulse_joint(parent, handle, joint);
            parent = handle;
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(128.0, -30.0), 4.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
