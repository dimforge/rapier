//! Zero-length steps (`IntegrationParameters::dt == 0`, e.g. the first frame of a variable
//! timestep) must leave the simulation state untouched instead of corrupting it.

use rapier2d::prelude::*;

/// A plate welded to the top-edge cluster of a jelly: a zero-length step used to corrupt the
/// cluster's state, making the plate jump (up to NaN colliders).
#[test]
fn zero_dt_step_does_not_disturb_a_welded_cluster() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(10.0, 0.5),
    );
    let jelly = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::splat(0.6), 5, 5)
        .cell_model(SoftBodyCellModel::Corotational)
        .particle_mass(0.08)
        .particle_radius(0.05)
        .can_sleep(false);
    let top_y = jelly
        .particle_positions()
        .iter()
        .map(|p| p.y)
        .fold(Real::MIN, Real::max);
    let top_edge: Vec<u32> = (0..jelly.particle_positions().len() as u32)
        .filter(|i| jelly.particle_positions()[*i as usize].y > top_y - 1.0e-3)
        .collect();
    let h = world.insert_soft_body(jelly);
    let cluster = world.add_soft_body_cluster(h, &top_edge).unwrap();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    let (plate, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, top_y + 0.2)),
        ColliderBuilder::cuboid(0.7, 0.06),
    );
    world.insert_impulse_joint(
        plate,
        proxy,
        FixedJointBuilder::new().local_anchor1(Vector::new(0.0, -0.2)),
    );

    let particles_before: Vec<Vector> = world.soft_bodies[h].particle_positions().collect();
    let plate_before = *world.bodies[plate].position();

    // The zero-length step changes nothing and injects no velocity.
    world.integration_parameters.dt = 0.0;
    world.step();
    assert!(
        world.soft_bodies[h]
            .particle_positions()
            .eq(particles_before.iter().copied())
    );
    assert_eq!(*world.bodies[plate].position(), plate_before);
    assert_eq!(world.bodies[plate].linvel(), Vector::ZERO);
    assert_eq!(world.bodies[plate].angvel(), 0.0);
    assert!(
        world.soft_bodies[h]
            .particle_velocities()
            .all(|v| v == Vector::ZERO)
    );

    // The regular steps that follow stay calm and finite.
    world.integration_parameters.dt = 1.0 / 60.0;
    world.step();
    let max_particle_speed = world.soft_bodies[h]
        .particle_velocities()
        .map(|v| v.length())
        .fold(0.0, Real::max);
    let plate_speed = world.bodies[plate].linvel().length();
    assert!(
        max_particle_speed < 0.5 && plate_speed < 0.5,
        "{max_particle_speed} {plate_speed}"
    );

    for _ in 0..60 {
        world.step();
    }
    assert!(
        world.soft_bodies[h]
            .particle_positions()
            .all(|p| p.is_finite())
    );
    let plate_pos = world.bodies[plate].position();
    assert!(plate_pos.translation.is_finite());
    // The plate stays (roughly) level on top of the jelly.
    assert!(
        plate_pos.rotation.angle().abs() < 0.3,
        "{}",
        plate_pos.rotation.angle()
    );
    for (_, co) in world.colliders.iter() {
        assert!(co.position().translation.is_finite());
    }
}

/// A zero-length step still applies the user changes and runs collision detection, so contacts
/// and scene queries reflect the latest edits, but moves nothing.
#[test]
fn zero_dt_step_detects_collisions_without_moving_bodies() {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = 0.0;
    let (_, ground_co) = world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(10.0, 0.5),
    );
    let (ball, ball_co) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.9))
            .linvel(Vector::new(1.0, 2.0)),
        ColliderBuilder::ball(0.5),
    );
    let (kinematic, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(5.0, 3.0)),
        ColliderBuilder::ball(0.5),
    );
    world.bodies[kinematic].set_next_kinematic_translation(Vector::new(6.0, 3.0));

    for _ in 0..3 {
        world.step();
    }

    assert_eq!(world.bodies[ball].translation(), Vector::new(0.0, 0.9));
    assert_eq!(world.bodies[ball].linvel(), Vector::new(1.0, 2.0));
    assert_eq!(world.bodies[kinematic].translation(), Vector::new(5.0, 3.0));

    assert!(
        world
            .narrow_phase
            .contact_pair(ground_co, ball_co)
            .is_some_and(|pair| pair.has_any_active_contact())
    );

    // The next regular step simulates normally (and moves the kinematic body).
    world.integration_parameters.dt = 1.0 / 60.0;
    world.step();
    assert_ne!(world.bodies[ball].translation(), Vector::new(0.0, 0.9));
    assert_eq!(world.bodies[kinematic].translation(), Vector::new(6.0, 3.0));
}
