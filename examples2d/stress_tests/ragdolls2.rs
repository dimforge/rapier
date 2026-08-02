use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// A 10-body articulated ragdoll (torso, head, 2×2 arm links, 2×2 leg links)
/// whose torso center is at `origin`. 9 revolute joints, all with limits.
fn ragdoll(world: &mut PhysicsWorld, origin: Vec2) {
    let part = |world: &mut PhysicsWorld, offset: Vec2, collider: ColliderBuilder| {
        let body = RigidBodyBuilder::dynamic().translation(origin + offset);
        let (handle, _) = world.insert(body, collider);
        (handle, offset)
    };

    let revolute = |parent: (RigidBodyHandle, Vec2),
                    child: (RigidBodyHandle, Vec2),
                    anchor: Vec2,
                    limits: [f32; 2]| {
        RevoluteJointBuilder::new()
            .local_anchor1(anchor - parent.1)
            .local_anchor2(anchor - child.1)
            .limits(limits)
            .contacts_enabled(false)
    };

    let torso = part(world, Vec2::ZERO, ColliderBuilder::capsule_y(0.3, 0.15));
    let head = part(world, Vec2::new(0.0, 0.55), ColliderBuilder::ball(0.15));
    let neck = revolute(torso, head, Vec2::new(0.0, 0.42), [-0.5, 0.5]);
    world.insert_impulse_joint(torso.0, head.0, neck);

    for side in [-1.0f32, 1.0] {
        let upper_arm = part(
            world,
            Vec2::new(side * 0.36, 0.25),
            ColliderBuilder::capsule_x(0.14, 0.06),
        );
        let forearm = part(
            world,
            Vec2::new(side * 0.70, 0.25),
            ColliderBuilder::capsule_x(0.14, 0.06),
        );
        let thigh = part(
            world,
            Vec2::new(side * 0.09, -0.52),
            ColliderBuilder::capsule_y(0.16, 0.07),
        );
        let shin = part(
            world,
            Vec2::new(side * 0.09, -0.92),
            ColliderBuilder::capsule_y(0.16, 0.07),
        );

        let shoulder = revolute(torso, upper_arm, Vec2::new(side * 0.19, 0.25), [-1.2, 1.2]);
        world.insert_impulse_joint(torso.0, upper_arm.0, shoulder);
        let elbow = revolute(upper_arm, forearm, Vec2::new(side * 0.53, 0.25), [0.0, 2.5]);
        world.insert_impulse_joint(upper_arm.0, forearm.0, elbow);
        let hip = revolute(torso, thigh, Vec2::new(side * 0.09, -0.33), [-1.0, 1.0]);
        world.insert_impulse_joint(torso.0, thigh.0, hip);
        let knee = revolute(thigh, shin, Vec2::new(side * 0.09, -0.72), [0.0, 2.3]);
        world.insert_impulse_joint(thigh.0, shin.0, knee);
    }
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vec2::new(0.0, -1.0)),
        ColliderBuilder::cuboid(1000.0, 1.0),
    );

    /*
     * Ragdolls dropped into a pile: 20 columns, 10 layers (200 ragdolls,
     * 2000 dynamic bodies, 1800 limit joints).
     */
    for layer in 0..10 {
        for col in 0..20 {
            ragdoll(
                &mut world,
                Vec2::new(col as f32 * 2.2, 1.5 + layer as f32 * 2.6),
            );
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(22.0, 6.0), 15.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
