//! Regression test for #752: `update_as_oneway_platform` takes the allowed
//! local normal (and angle) so the platform's one-way direction can be chosen,
//! and works whether the platform is the first or second collider of the pair.

use rapier2d::prelude::*;

struct OneWayPlatformHook {
    platform: ColliderHandle,
}

impl PhysicsHooks for OneWayPlatformHook {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        // `manifold.local_n1` points towards the outside of `collider1`, so the
        // allowed normal must be flipped if the platform is the second collider.
        let allowed_local_n1 = if context.collider1 == self.platform {
            Vector::Y
        } else {
            -Vector::Y
        };
        context.update_as_oneway_platform(allowed_local_n1, 0.1);
    }
}

#[test]
fn oneway_platform_with_custom_normal() {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ccd = CCDSolver::new();
    let params = IntegrationParameters::default();
    let gravity = Vector::new(0.0, -9.81);

    let ground = bodies.insert(RigidBodyBuilder::fixed());
    let platform = colliders.insert_with_parent(
        ColliderBuilder::cuboid(2.0, 0.1).active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS),
        ground,
        &mut bodies,
    );

    // A ball starting below the platform, moving up fast enough to reach above it.
    let ball = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, -3.0))
            .linvel(Vector::new(0.0, 12.0)),
    );
    colliders.insert_with_parent(ColliderBuilder::ball(0.25), ball, &mut bodies);

    let hooks = OneWayPlatformHook { platform };

    let mut max_y: Real = -3.0;
    for _ in 0..300 {
        pipeline.step(
            gravity,
            &params,
            &mut islands,
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &hooks,
            &(),
        );
        max_y = max_y.max(bodies[ball].translation().y);
    }

    // The ball must have passed through the platform from below...
    assert!(
        max_y > 1.0,
        "ball did not pass through the platform from below (max_y = {max_y})"
    );
    // ...and then landed (and stayed) on top of it.
    let final_y = bodies[ball].translation().y;
    assert!(
        (0.2..0.6).contains(&final_y),
        "ball did not land on top of the platform (final_y = {final_y})"
    );
}
