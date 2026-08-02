//! Port of box2d's `joint_grid` benchmark (`CreateJointGrid`,
//! `box2d/shared/benchmarks.c`). Release: a 100x100 grid of circles wired with
//! revolute joints; a band of the top row is static. Circles don't collide with
//! each other (box2d category/mask filter). Sleeping disabled.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);

    let n = 100usize;
    // box2d: circles are category 2, mask ~2 -> they collide with everything
    // except other circles.
    let groups = InteractionGroups::new(
        Group::GROUP_2,
        Group::ALL ^ Group::GROUP_2,
        InteractionTestMode::And,
    );

    let mut bodies = vec![RigidBodyHandle::invalid(); n * n];
    let mut index = 0usize;

    for k in 0..n {
        for i in 0..n {
            let is_static = k >= n / 2 - 3 && k <= n / 2 + 3 && i == 0;
            let body = if is_static {
                RigidBodyBuilder::fixed()
            } else {
                RigidBodyBuilder::dynamic().can_sleep(false)
            }
            .translation(Vector::new(k as f32, -(i as f32)));
            let handle = world.insert_body(body);
            world.insert_collider(
                ColliderBuilder::ball(0.4)
                    .density(1.0)
                    .collision_groups(groups),
                Some(handle),
            );

            if i > 0 {
                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(Vector::new(0.0, -0.5))
                    .local_anchor2(Vector::new(0.0, 0.5))
                    .contacts_enabled(false);
                world.insert_impulse_joint(bodies[index - 1], handle, joint);
            }
            if k > 0 {
                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(Vector::new(0.5, 0.0))
                    .local_anchor2(Vector::new(-0.5, 0.0))
                    .contacts_enabled(false);
                world.insert_impulse_joint(bodies[index - n], handle, joint);
            }

            bodies[index] = handle;
            index += 1;
        }
    }

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(50.0, -50.0), 4.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
