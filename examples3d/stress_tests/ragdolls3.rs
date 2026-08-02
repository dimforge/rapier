use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

/// A 10-body articulated ragdoll (torso, head, 2×2 arm links, 2×2 leg links)
/// whose torso center is at `origin`. 9 joints, all with limits: spherical
/// neck, shoulders and hips; revolute elbows and knees.
fn ragdoll(world: &mut PhysicsWorld, origin: Vec3) {
    let part = |world: &mut PhysicsWorld, offset: Vec3, collider: ColliderBuilder| {
        let body = RigidBodyBuilder::dynamic().translation(origin + offset);
        let (handle, _) = world.insert(body, collider);
        (handle, offset)
    };

    let spherical = |parent: (RigidBodyHandle, Vec3),
                     child: (RigidBodyHandle, Vec3),
                     anchor: Vec3,
                     limit: f32| {
        SphericalJointBuilder::new()
            .local_anchor1(anchor - parent.1)
            .local_anchor2(anchor - child.1)
            .limits(JointAxis::AngX, [-limit, limit])
            .limits(JointAxis::AngY, [-limit, limit])
            .limits(JointAxis::AngZ, [-limit, limit])
            .contacts_enabled(false)
    };
    let revolute = |parent: (RigidBodyHandle, Vec3),
                    child: (RigidBodyHandle, Vec3),
                    anchor: Vec3,
                    limits: [f32; 2]| {
        RevoluteJointBuilder::new(Vec3::Z)
            .local_anchor1(anchor - parent.1)
            .local_anchor2(anchor - child.1)
            .limits(limits)
            .contacts_enabled(false)
    };

    let torso = part(world, Vec3::ZERO, ColliderBuilder::capsule_y(0.3, 0.15));
    let head = part(
        world,
        Vec3::new(0.0, 0.55, 0.0),
        ColliderBuilder::ball(0.15),
    );
    let neck = spherical(torso, head, Vec3::new(0.0, 0.42, 0.0), 0.5);
    world.insert_impulse_joint(torso.0, head.0, neck);

    for side in [-1.0f32, 1.0] {
        let upper_arm = part(
            world,
            Vec3::new(side * 0.36, 0.25, 0.0),
            ColliderBuilder::capsule_x(0.14, 0.06),
        );
        let forearm = part(
            world,
            Vec3::new(side * 0.70, 0.25, 0.0),
            ColliderBuilder::capsule_x(0.14, 0.06),
        );
        let thigh = part(
            world,
            Vec3::new(side * 0.09, -0.52, 0.0),
            ColliderBuilder::capsule_y(0.16, 0.07),
        );
        let shin = part(
            world,
            Vec3::new(side * 0.09, -0.92, 0.0),
            ColliderBuilder::capsule_y(0.16, 0.07),
        );

        let shoulder = spherical(torso, upper_arm, Vec3::new(side * 0.19, 0.25, 0.0), 1.2);
        world.insert_impulse_joint(torso.0, upper_arm.0, shoulder);
        let elbow = revolute(
            upper_arm,
            forearm,
            Vec3::new(side * 0.53, 0.25, 0.0),
            [0.0, 2.5],
        );
        world.insert_impulse_joint(upper_arm.0, forearm.0, elbow);
        let hip = spherical(torso, thigh, Vec3::new(side * 0.09, -0.33, 0.0), 1.0);
        world.insert_impulse_joint(torso.0, thigh.0, hip);
        let knee = revolute(thigh, shin, Vec3::new(side * 0.09, -0.72, 0.0), [0.0, 2.3]);
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
    let (ground, _) = world.insert(
        RigidBodyBuilder::fixed().translation(Vec3::new(0.0, -1.0, 0.0)),
        ColliderBuilder::cuboid(100.0, 1.0, 100.0),
    );
    let _ = ground;

    /*
     * Ragdolls dropped into a pile: a 5x5 grid, 5 layers (125 ragdolls,
     * 1250 dynamic bodies, 1125 limit joints).
     */
    for layer in 0..5 {
        for row in 0..5 {
            for col in 0..5 {
                let origin =
                    Vec3::new(col as f32 * 2.2, 1.5 + layer as f32 * 2.6, row as f32 * 2.2);
                ragdoll(&mut world, origin);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(-12.0, 10.0, -12.0), Vec3::new(4.5, 1.0, 4.5));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
