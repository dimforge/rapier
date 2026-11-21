//! `RigidBody::sleep` on a body that touches awake bodies. Islands sleep as a whole, so the
//! call puts the body's whole island to sleep where it is, or does nothing when a body of the
//! island cannot sleep. It used to set the flag only: the body kept falling with the body
//! under it while `is_sleeping` reported `true`, then froze in the air once they separated.

use rapier3d::prelude::*;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0),
    );
    world
}

fn insert_box(world: &mut PhysicsWorld, y: Real) -> RigidBodyHandle {
    world
        .insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(0.0, y, 0.0)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        )
        .0
}

// No body may be flagged asleep while it is still simulated.
fn assert_no_sleeper_is_simulated(world: &PhysicsWorld) {
    for (handle, body) in world.active_bodies() {
        assert!(!body.is_sleeping(), "{handle:?} is asleep and simulated");
    }
}

#[test]
fn sleep_puts_the_whole_island_to_sleep() {
    let mut world = world_with_ground();
    // A box resting on another box, both falling, plus a ball elsewhere that stays awake.
    let top = insert_box(&mut world, 5.0);
    let bottom = insert_box(&mut world, 3.999);
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(10.0, 20.0, 0.0)),
        ColliderBuilder::ball(0.5),
    );
    world.step();

    world.bodies[top].sleep();
    let (top_y, bottom_y) = (
        world.bodies[top].translation().y,
        world.bodies[bottom].translation().y,
    );
    for _ in 0..60 {
        world.step();
        for (h, y) in [(top, top_y), (bottom, bottom_y)] {
            let body = &world.bodies[h];
            assert!(body.is_sleeping(), "the island should be asleep");
            assert_eq!(body.translation().y, y, "a sleeping body moved");
        }
        assert_no_sleeper_is_simulated(&world);
    }
    assert!(
        !world.bodies[ball].is_sleeping(),
        "the ball isn't touching them and keeps falling"
    );

    // Waking either box wakes both, and they fall to the ground.
    world.bodies[top].wake_up(true);
    for _ in 0..240 {
        world.step();
    }
    assert!(world.bodies[bottom].translation().y < 0.6);
    assert!(world.bodies[top].translation().y < 1.6);
}

#[test]
fn sleep_leaves_a_moving_kinematic_body_alone() {
    let mut world = world_with_ground();
    // A box riding a lift.
    let (lift, _) = world.insert(
        RigidBodyBuilder::kinematic_velocity_based()
            .translation(Vector::new(0.0, 1.0, 0.0))
            .linvel(Vector::new(0.0, 0.5, 0.0)),
        ColliderBuilder::cuboid(2.0, 0.25, 2.0),
    );
    let rider = insert_box(&mut world, 1.75);
    for _ in 0..30 {
        world.step();
    }

    world.bodies[rider].sleep();
    let lift_y = world.bodies[lift].translation().y;
    for _ in 0..60 {
        world.step();
        assert_no_sleeper_is_simulated(&world);
    }
    // The lift kept rising at 0.5 m/s, and the box with it.
    assert!(!world.bodies[lift].is_sleeping());
    assert_eq!(world.bodies[lift].linvel().y, 0.5);
    assert!((world.bodies[lift].translation().y - lift_y - 0.5).abs() < 1.0e-3);
    assert!(!world.bodies[rider].is_sleeping());
    let gap = world.bodies[rider].translation().y - world.bodies[lift].translation().y;
    assert!((gap - 0.75).abs() < 0.01, "the box left the lift: {gap}");
}

#[test]
fn sleep_leaves_a_body_that_never_sleeps_awake() {
    let mut world = world_with_ground();
    let (restless, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5, 0.0))
            .can_sleep(false),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    let top = insert_box(&mut world, 1.5);
    for _ in 0..30 {
        world.step();
    }

    world.bodies[top].sleep();
    for _ in 0..60 {
        world.step();
        assert_no_sleeper_is_simulated(&world);
        assert!(!world.bodies[restless].is_sleeping());
        assert!(!world.bodies[top].is_sleeping());
    }
}

#[test]
fn a_wake_up_in_the_same_step_wins() {
    for sleep_first in [true, false] {
        let mut world = world_with_ground();
        let bottom = insert_box(&mut world, 0.5);
        let top = insert_box(&mut world, 1.5);
        for _ in 0..30 {
            world.step();
        }

        // A push up on the top box, and `sleep` on the one under it, in either order.
        let push = Vector::new(0.0, 5.0, 0.0);
        if sleep_first {
            world.bodies[bottom].sleep();
            world.bodies[top].apply_impulse(push, true);
        } else {
            world.bodies[top].apply_impulse(push, true);
            world.bodies[bottom].sleep();
        }
        let top_y = world.bodies[top].translation().y;
        let mut highest = top_y;
        for _ in 0..30 {
            world.step();
            assert_no_sleeper_is_simulated(&world);
            highest = highest.max(world.bodies[top].translation().y);
        }
        assert!(highest - top_y > 0.5, "the push was lost");
    }
}

#[test]
fn sleep_only_takes_the_bodies_still_connected() {
    let mut world = world_with_ground();
    let bottom = insert_box(&mut world, 0.5);
    let top = insert_box(&mut world, 1.5);
    for _ in 0..30 {
        world.step();
    }
    // The top box is thrown up while the bottom one slides away: they stop touching.
    world.bodies[top].set_linvel(Vector::new(0.0, 4.0, 0.0), true);
    world.bodies[bottom].set_linvel(Vector::new(2.0, 0.0, 0.0), true);
    for _ in 0..10 {
        world.step();
    }
    let top_y = world.bodies[top].translation().y;
    assert!(top_y > 2.0);

    world.bodies[bottom].sleep();
    for _ in 0..240 {
        world.step();
        assert_no_sleeper_is_simulated(&world);
    }
    // The top box came back down: it did not go to sleep in the air with the other one.
    assert!(world.bodies[bottom].is_sleeping());
    assert!(world.bodies[top].translation().y < 1.6);
}

#[test]
fn sleep_with_another_change_in_the_same_step() {
    let mut world = world_with_ground();
    let boxes = [0.5, 1.5, 2.5].map(|y| insert_box(&mut world, y));
    for _ in 0..30 {
        world.step();
    }

    // `sleep` on the middle box, and a change that wakes the bottom one in that same step.
    world.bodies[boxes[1]].sleep();
    let bottom_collider = world.bodies[boxes[0]].colliders()[0];
    let groups = InteractionGroups::new(Group::ALL, Group::ALL, InteractionTestMode::Or);
    world.colliders[bottom_collider].set_collision_groups(groups);
    for _ in 0..120 {
        world.step();
        assert_no_sleeper_is_simulated(&world);
    }
    // The stack stayed where it was.
    for (handle, y) in boxes.iter().zip([0.5, 1.5, 2.5]) {
        assert!((world.bodies[*handle].translation().y - y).abs() < 0.01);
    }
}
