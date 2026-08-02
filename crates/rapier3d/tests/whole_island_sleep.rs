//! Tests for whole-island sleep (the only sleep behavior): an
//! island only sleeps once *all* of its bodies are sleep-eligible, and it
//! sleeps — and wakes — as a single unit.

use rapier3d::pipeline::PhysicsWorld;
use rapier3d::prelude::*;

/// Inserts a vertical stack of `num` unit cubes resting on the ground, at
/// horizontal offset `x`. Returns the body handles, bottom first.
fn insert_stack(world: &mut PhysicsWorld, x: Real, num: usize) -> Vec<RigidBodyHandle> {
    (0..num)
        .map(|i| {
            let body =
                RigidBodyBuilder::dynamic().translation(Vector::new(x, 0.5 + i as Real * 1.0, 0.0));
            world.insert(body, ColliderBuilder::cuboid(0.5, 0.5, 0.5)).0
        })
        .collect()
}

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let ground = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0));
    world.insert(ground, ColliderBuilder::cuboid(100.0, 0.5, 100.0));
    world
}

fn num_sleeping(world: &PhysicsWorld) -> usize {
    world
        .rigid_bodies()
        .filter(|(_, rb)| rb.is_dynamic() && rb.is_sleeping())
        .count()
}

/// More than enough steps for a pre-settled stack to become sleep-eligible
/// (default `time_until_sleep` is 1s = 60 steps).
const SETTLE_STEPS: usize = 240;

/// A `can_sleep(false)` body atop a stack must keep the *whole* stack awake.
/// Once removed, the stack must sleep even though no body newly becomes eligible
/// at that point (the re-queue path); waking one body wakes the whole island.
#[test]
fn whole_island_blocks_partial_sleep() {
    let mut world = world_with_ground();

    let stack = insert_stack(&mut world, 0.0, 6);
    let restless = world
        .insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(0.0, 6.5, 0.0))
                .can_sleep(false),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        )
        .0;

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(
        num_sleeping(&world),
        0,
        "no body of an island containing a non-sleeping body may sleep"
    );

    // The non-sleeper leaves the island: the stack must now fall asleep even
    // though none of its bodies newly becomes eligible (they already were).
    world.remove_body(restless);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(
        num_sleeping(&world),
        stack.len(),
        "the whole stack must sleep"
    );

    // Waking any body wakes the island as a unit.
    world.wake_up(stack[0], true);
    assert_eq!(num_sleeping(&world), 0, "the island must wake as a unit");
}

/// The same scene under the *default* configuration: whole-island sleep is
/// the only behavior now (partial-island/Region sleep was removed), so no
/// body of the stack may sleep while the non-sleeping body touches it.
#[test]
fn default_never_sleeps_partial_islands() {
    let mut world = world_with_ground();

    insert_stack(&mut world, 0.0, 6);
    let restless = world
        .insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(0.0, 6.5, 0.0))
                .can_sleep(false),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        )
        .0;

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(
        num_sleeping(&world),
        0,
        "no body of an island containing a non-sleeping body may sleep"
    );
    assert!(!world.bodies[restless].is_sleeping());
}

/// Sleep re-attempts are event-driven: a kinematic body grazing the stack (no
/// motion transfer) taints the island but never becomes eligible itself, so when
/// its contact stops, the contact-stop re-queue is the only path back to sleep.
#[test]
fn whole_island_sleeps_after_mover_departs() {
    let mut world = world_with_ground();

    let stack = insert_stack(&mut world, 0.0, 4);
    // A kinematic cube grazing the side of the stack's bottom cube (1mm
    // overlap: keeps the contact active without meaningfully pushing the
    // stack), sliding along that face fast enough to never be sleep-eligible.
    let mover = world
        .insert(
            RigidBodyBuilder::kinematic_velocity_based()
                .translation(Vector::new(0.999, 0.5, -0.9))
                .linvel(Vector::new(0.0, 0.0, 0.8)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        )
        .0;

    // The stack is eligible after ~60 steps; the mover stays in contact until
    // it slid past the face (|z| > 1), around step 142.
    for _ in 0..90 {
        world.step();
    }
    assert_eq!(
        num_sleeping(&world),
        0,
        "the sliding kinematic body in contact with the island must keep it awake"
    );

    // Keep sliding: the contact stops while the kinematic body keeps moving
    // forever (no eligibility transition ever fires, for anything).
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(
        num_sleeping(&world),
        stack.len(),
        "the stack must sleep once the kinematic mover slid out of contact"
    );
    assert!(!world.bodies[mover].is_sleeping());
}

/// Inserts a grounded cube jointed to a "taint anchor": a never-sleeping,
/// collider-less, gravity-free floating body — eternally non-eligible yet
/// motionless, so only the explicit joint-edit re-queues can ever heal the cube.
fn insert_cube_jointed_to_floating_taint(
    world: &mut PhysicsWorld,
) -> (RigidBodyHandle, RigidBodyHandle, ImpulseJointHandle) {
    let cube = insert_stack(world, 0.0, 1)[0];
    let restless = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(5.0, 3.0, 0.0))
            .additional_mass(1.0)
            .gravity_scale(0.0)
            .can_sleep(false),
    );
    // Anchors coincide at the floating body's position: the joint starts (and
    // stays) at zero error, so it transmits no force and nothing ever moves.
    let joint = world.impulse_joints.insert(
        cube,
        restless,
        SphericalJointBuilder::new().local_anchor1(Vector::new(5.0, 2.5, 0.0)),
        true,
    );
    (cube, restless, joint)
}

/// A joint removed with `wake_up = false` silently deletes the edge that kept
/// a never-sleeping body attached to the island: the leftover body must still
/// fall asleep (the joint-removal re-queue), without anything being woken.
#[test]
fn whole_island_sleeps_after_joint_removed_without_wake() {
    let mut world = world_with_ground();

    let (cube, restless, joint) = insert_cube_jointed_to_floating_taint(&mut world);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(
        num_sleeping(&world),
        0,
        "the joint to a never-sleeping body must keep the cube awake"
    );

    // Remove the joint WITHOUT waking: no eligibility transition ever fires
    // for the cube, only the removal re-queue can put it to sleep.
    world.impulse_joints.remove(joint, false);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(
        world.bodies[cube].is_sleeping(),
        "the cube must sleep once the joint to the never-sleeping body is removed"
    );
    assert!(!world.bodies[restless].is_sleeping());
}

/// Turning a collider-less joint partner into a fixed body removes its taint
/// without touching any contact or removing the joint: only the type-change
/// re-queue of joint partners can let the cube sleep.
#[test]
fn whole_island_sleeps_after_joint_partner_turned_fixed() {
    let mut world = world_with_ground();

    let (cube, restless, _joint) = insert_cube_jointed_to_floating_taint(&mut world);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(num_sleeping(&world), 0);

    // Freeze the floating body: fixed bodies neither sleep nor taint, and a
    // collider-less body has no contacts through which anything gets woken.
    world.bodies[restless].set_body_type(RigidBodyType::Fixed, false);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(
        world.bodies[cube].is_sleeping(),
        "the cube must sleep once its never-sleeping joint partner is frozen to fixed"
    );
}

/// Turning the island's mover into a fixed body removes its taint without any
/// eligibility transition or edge removal: the type-change re-queue must let
/// the rest of the island sleep.
#[test]
fn whole_island_sleeps_after_mover_turned_fixed() {
    let mut world = world_with_ground();

    let stack = insert_stack(&mut world, 0.0, 4);
    // A never-sleeping cube on top of the stack.
    let restless = world
        .insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(0.0, 4.5, 0.0))
                .can_sleep(false),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        )
        .0;

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(num_sleeping(&world), 0);

    // Freeze the restless body: fixed bodies neither sleep nor taint.
    world.bodies[restless].set_body_type(RigidBodyType::Fixed, false);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(
        num_sleeping(&world),
        stack.len(),
        "the stack must sleep once the restless body is frozen to fixed"
    );
}

/// Disabling a body detaches its joints without waking the partners: the
/// disable re-queue must let the leftover body sleep.
#[test]
fn whole_island_sleeps_after_jointed_body_disabled() {
    let mut world = world_with_ground();

    let (cube, restless, _joint) = insert_cube_jointed_to_floating_taint(&mut world);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(num_sleeping(&world), 0);

    world.bodies[restless].set_enabled(false);
    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert!(
        world.bodies[cube].is_sleeping(),
        "the cube must sleep once its never-sleeping joint partner is disabled"
    );
}

/// Two disjoint stacks are independent islands: each sleeps on its own, and
/// waking one island leaves the other asleep.
#[test]
fn whole_island_components_are_independent() {
    let mut world = world_with_ground();

    let stack_a = insert_stack(&mut world, 0.0, 4);
    let stack_b = insert_stack(&mut world, 20.0, 4);

    for _ in 0..SETTLE_STEPS {
        world.step();
    }
    assert_eq!(num_sleeping(&world), 8, "both stacks must sleep");

    world.wake_up(stack_a[0], true);
    assert!(
        stack_a.iter().all(|h| !world.bodies[*h].is_sleeping()),
        "waking one body must wake its whole island"
    );
    assert!(
        stack_b.iter().all(|h| world.bodies[*h].is_sleeping()),
        "the other island must stay asleep"
    );
}
