//! Showcases angular joint limits sitting anywhere on the circle (issue #499): each "dial"
//! is a motor-driven arm on a revolute joint whose limit range is drawn from a different
//! family. The top row is driven counter-clockwise and parks at the range's max; the bottom
//! row is driven clockwise and parks at the min. The last column's range is wider than a
//! full turn, which is indistinguishable from "no limit" for a wrapped angle: those arms
//! spin forever.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// The showcased `[min, max]` limit ranges, in degrees, with the family each illustrates
/// (shown in the UI, one entry per dial column from left to right).
const LIMITS_DEG: [([f32; 2], &str); 5] = [
    // Already worked before the fix.
    ([-45.0, 45.0], "within half a turn"),
    // Used to fold back (stopped at 90° instead of 270°).
    ([0.0, 270.0], "past half a turn"),
    ([135.0, 225.0], "straddles the ±180° seam"),
    ([-350.0, 0.0], "nearly a full turn"),
    // The limit row is disabled, the arm spins freely.
    ([-200.0, 200.0], "wider than a turn: free"),
];

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World: no gravity so the motors alone decide where each arm settles.
     */
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    let settings = viewer.example_settings_mut();
    let use_multibody = settings.get_or_set_bool("Multibody joints", false);

    // The dial ranges, shown in the Example Settings window (left to right).
    const COLUMN_KEYS: [&str; 5] = ["Column 1", "Column 2", "Column 3", "Column 4", "Column 5"];
    for (key, (limits, family)) in COLUMN_KEYS.iter().zip(LIMITS_DEG.iter()) {
        settings.set_label(key, format!("[{}°, {}°] — {family}", limits[0], limits[1]));
    }
    settings.set_label("Rows", "top: driven to the max — bottom: to the min");

    let spacing = 4.0;

    for (i, (limits_deg, _)) in LIMITS_DEG.iter().enumerate() {
        // dir = 1: driven counter-clockwise toward the max; dir = -1: toward the min.
        for dir in [1.0f32, -1.0] {
            let center = Vector::new(i as f32 * spacing, if dir > 0.0 { 0.0 } else { -spacing });

            // A small fixed disc marking the dial's center.
            let anchor = world
                .bodies
                .insert(RigidBodyBuilder::fixed().translation(center));
            world.insert_collider(ColliderBuilder::ball(0.2), Some(anchor));

            // The arm. Angular damping caps the speed it reaches the limit with, so it
            // settles right at the bound instead of oscillating around it.
            let arm = world.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(center + Vector::new(1.0, 0.0))
                    .angular_damping(3.0)
                    .can_sleep(false),
            );
            world.insert_collider(ColliderBuilder::cuboid(0.5, 0.1), Some(arm));

            let joint = RevoluteJointBuilder::new()
                .local_anchor1(Vector::ZERO)
                .local_anchor2(Vector::new(-1.0, 0.0))
                .limits([limits_deg[0].to_radians(), limits_deg[1].to_radians()])
                .motor_velocity(dir * 5.0, 20.0);

            if use_multibody {
                world
                    .multibody_joints
                    .insert(anchor, arm, joint, true)
                    .unwrap();
            } else {
                world.impulse_joints.insert(anchor, arm, joint, true);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(2.0 * spacing, -0.5 * spacing), 40.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
