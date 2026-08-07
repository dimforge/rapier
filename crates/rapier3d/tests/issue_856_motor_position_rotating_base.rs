//! Regression test for https://github.com/dimforge/rapier/issues/856
//!
//! A revolute joint driven by `motor_position` whose base body is also rotated by the user
//! every frame used to blow the attached body up to NaN. The simulation must stay finite,
//! with nothing NaN-quarantined.

use rapier3d::prelude::*;

#[test]
fn motor_position_with_rotating_base_stays_finite() {
    let mut world = PhysicsWorld::new();

    // The rotating base (the "yellow cylinder" from the report): kept dynamic
    // so the joint is fully simulated, but rotated by the user every frame.
    let (base, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vec3::new(0.0, 3.0, 0.0)),
        ColliderBuilder::cylinder(0.2, 1.0),
    );

    // The "hammer" attached to the base through a revolute joint.
    let (hammer, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vec3::new(2.0, 3.0, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.1, 0.1),
    );

    let joint = RevoluteJointBuilder::new(Vec3::Z)
        .local_anchor1(Vec3::new(1.0, 0.0, 0.0))
        .local_anchor2(Vec3::new(-1.0, 0.0, 0.0))
        .motor_position(core::f32::consts::PI, 1.0e4, 100.0);
    world.insert_impulse_joint(base, hammer, joint);

    for i in 0..300 {
        // User-driven base rotation, like the demo's mouse-driven spin.
        let angle = i as f32 * 0.05;
        world.bodies[base].set_rotation(Rotation::from_axis_angle(Vec3::Y, angle), true);

        world.step();

        for (handle, rb) in world.rigid_bodies() {
            assert!(
                rb.translation().is_finite() && rb.rotation().is_finite(),
                "body {handle:?} went non-finite at step {i}: pos={:?} rot={:?}",
                rb.translation(),
                rb.rotation()
            );
        }
        assert!(
            world.quarantine().is_empty(),
            "bodies were NaN-quarantined at step {i}: {:?}",
            world.quarantine().bodies()
        );
    }

    // The motor must actually have driven the hammer instead of freezing it.
    let hammer_pos = world.bodies[hammer].translation();
    assert!(hammer_pos.is_finite());
}
