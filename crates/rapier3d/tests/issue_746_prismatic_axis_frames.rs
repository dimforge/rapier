//! Regression test for https://github.com/dimforge/rapier/issues/746
//!
//! `set_local_axis1`/`set_local_axis2` each completed an ARBITRARY orthonormal frame around
//! the axis, so frames set from two relatively-rotated axes could disagree by a twist that
//! fights a prismatic joint's angular locks (a relative pi rotation diverged past 1e7 in one
//! step). They are now completed with the minimal rotation taking +X to the axis.

use rapier3d::prelude::*;

#[test]
fn prismatic_joint_stays_bounded_for_all_axis_rotations() {
    for i in 0..8 {
        let mut world = PhysicsWorld::new();
        world.integration_parameters.dt = 0.016;

        let angle = std::f32::consts::PI / 2.0 * i as f32;
        let rotation = AngVector::new(0.0, angle, 0.0);
        let local_axis2 = Rotation::from_scaled_axis(rotation).inverse() * Vector::X;
        assert!((Rotation::from_scaled_axis(rotation) * local_axis2).dot(Vector::X) > 0.999);

        let body1 = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(0.0, 0.0, 0.0))
                .gravity_scale(0.0),
        );
        let body2 = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(1.0, 0.0, 0.0))
                .rotation(rotation)
                .gravity_scale(0.0),
        );

        world.colliders.insert_with_parent(
            ColliderBuilder::cuboid(1.0, 1.0, 1.0),
            body1,
            &mut world.bodies,
        );
        world.colliders.insert_with_parent(
            ColliderBuilder::cuboid(1.0, 1.0, 1.0),
            body2,
            &mut world.bodies,
        );

        let joint = PrismaticJointBuilder::new(Vector::X)
            .local_anchor1(Vector::new(0.0, 0.0, 0.0))
            .local_anchor2(Vector::new(0.0, 0.0, 0.0))
            .local_axis1(Vector::X)
            .local_axis2(local_axis2)
            .contacts_enabled(false);
        world.impulse_joints.insert(body1, body2, joint, true);

        for _ in 0..60 {
            world.step();
        }

        for (name, handle) in [("body1", body1), ("body2", body2)] {
            let pos = world.bodies[handle].translation();
            assert!(
                pos.length() < 5.0,
                "case {i} (angle {angle}): {name} diverged to {pos:?}"
            );
        }
    }
}
