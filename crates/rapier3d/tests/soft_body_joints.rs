//! Impulse joints attached to soft-body clusters through their proxy rigid bodies: soft-rigid,
//! soft-soft and soft-multibody coupling, angular DOFs on full-rank clusters, the angular strip
//! on rank-deficient ones, momentum conservation of the gather/scatter, and wake-up.

use rapier3d::prelude::*;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0).translation(Vector::new(0.0, -0.5, 0.0)),
    );
    world
}

fn cloth_at(world: &mut PhysicsWorld, origin: Vector) -> SoftBodyHandle {
    world.insert_soft_body(SoftBodyBuilder::cloth(
        origin,
        Vector::X * 0.2,
        Vector::Z * 0.2,
        6,
        6,
    ))
}

fn jelly_at(world: &mut PhysicsWorld, center: Vector) -> SoftBodyHandle {
    world.insert_soft_body(
        SoftBodyBuilder::cuboid(center, Vector::splat(0.4), 3, 3, 3)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e4,
                poisson_ratio: 0.3,
                elastic_damping_ratio: 1.0,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05)),
    )
}

/// A one-particle cluster jointed (spherically) to a kinematic mover: the cloth corner follows
/// it, two-way like an attachment but through the ordinary joint pipeline.
#[test]
fn spherical_joint_holds_a_cloth_corner_on_a_moving_body() {
    let mut world = world_with_ground();
    let h = cloth_at(&mut world, Vector::new(0.0, 2.0, 0.0));
    let corner_pos = world.soft_bodies[h].particle_position(0);
    let cluster = world
        .soft_bodies
        .add_cluster(h, &[0], &mut world.bodies, &mut world.colliders)
        .unwrap();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    // The proxy's frame sits at the cluster's centroid: the corner particle.
    assert!((world.bodies[proxy].position().translation - corner_pos).length() < 1.0e-6);

    let mover = world.insert_body(
        RigidBodyBuilder::kinematic_velocity_based()
            .translation(corner_pos)
            .linvel(Vector::new(0.5, 0.0, 0.0)),
    );
    let joint = SphericalJointBuilder::new()
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::ZERO);
    world.insert_impulse_joint(mover, proxy, joint);

    for _ in 0..120 {
        world.step();
    }
    let anchor = world.bodies[mover].position().translation;
    let corner = world.soft_bodies[h].particle_position(0);
    assert!(
        (corner - anchor).length() < 0.05,
        "corner {corner:?} strayed from its anchor {anchor:?}"
    );
    // The mover travelled: the corner really followed a moving target.
    assert!(anchor.x > 0.9);
    // And the rest of the cloth hangs from it rather than falling.
    assert!(world.soft_bodies[h].particle_position(35).y > 0.0);
}

/// A rope joint on the whole-body cluster limits how far a jelly can fall.
#[test]
fn rope_joint_limits_a_jelly_fall() {
    let mut world = PhysicsWorld::new();
    let h = jelly_at(&mut world, Vector::new(0.0, 0.0, 0.0));
    let root = world.soft_bodies[h].root_body();
    let anchor =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 2.0, 0.0)));
    let joint = RopeJointBuilder::new(3.0)
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::ZERO);
    world.insert_impulse_joint(anchor, root, joint);
    for _ in 0..300 {
        world.step();
    }
    let com = world.soft_bodies[h].center_of_mass();
    let dist = (com - Vector::new(0.0, 2.0, 0.0)).length();
    assert!(
        dist < 3.3,
        "the jelly fell past its rope limit (dist {dist})"
    );
    assert!(dist > 2.0, "the jelly did not fall at all (dist {dist})");
}

/// An angular motor on a full-rank cluster spins the jelly about the joint axis: the frame's
/// angular velocity is scattered into a real rigid rotation of the particles.
#[test]
fn revolute_motor_spins_a_jelly() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let h = jelly_at(&mut world, Vector::new(0.0, 0.0, 0.0));
    let root = world.soft_bodies[h].root_body();
    let com = world.soft_bodies[h].center_of_mass();
    let anchor = world.insert_body(RigidBodyBuilder::fixed().translation(com));
    let joint = RevoluteJointBuilder::new(Vector::Y)
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::ZERO)
        .motor_velocity(2.0, 50.0);
    world.insert_impulse_joint(anchor, root, joint);

    for _ in 0..120 {
        world.step();
    }
    // The gathered frame angular velocity approaches the motor target about Y.
    let w = world.bodies[root].angvel();
    assert!(
        w.y > 1.0,
        "the motor failed to spin the jelly (angvel {w:?})"
    );
    // And it is a real particle motion, not proxy bookkeeping: a border particle moves.
    let speed: Real = world.soft_bodies[h]
        .particle_velocities()
        .map(|v| v.length())
        .fold(0.0, |a: Real, b| a.max(b));
    assert!(speed > 0.4, "particles do not move (max speed {speed})");
    // The com stays pinned by the revolute joint.
    let com2 = world.soft_bodies[h].center_of_mass();
    assert!((com2 - com).length() < 0.1);
}

/// A position motor drives the jelly's frame to a target angle and holds it there.
#[test]
fn revolute_position_motor_reaches_its_target() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let h = jelly_at(&mut world, Vector::new(0.0, 0.0, 0.0));
    let root = world.soft_bodies[h].root_body();
    let com = world.soft_bodies[h].center_of_mass();
    let anchor = world.insert_body(RigidBodyBuilder::fixed().translation(com));
    let target = 1.0;
    let joint = RevoluteJointBuilder::new(Vector::Y)
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::ZERO)
        .motor_position(target, 100.0, 20.0);
    world.insert_impulse_joint(anchor, root, joint);
    for _ in 0..400 {
        world.step();
    }
    // The frame's rotation about Y approaches the target.
    let rot = world.bodies[root].position().rotation;
    let (axis, angle) = rot.to_axis_angle();
    let signed = angle * axis.y.signum();
    assert!(
        (signed - target).abs() < 0.25,
        "target {target}, reached {signed} (axis {axis:?}, angle {angle})"
    );
}

/// A soft-soft spherical joint conserves the linear momentum of the pair (the gather/scatter is
/// momentum-exact and the joint only exchanges impulses between the two clusters).
#[test]
fn soft_soft_joint_conserves_momentum() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let a = jelly_at(&mut world, Vector::new(0.0, 0.0, 0.0));
    let b = jelly_at(&mut world, Vector::new(1.2, 0.0, 0.0));
    let root_a = world.soft_bodies[a].root_body();
    let root_b = world.soft_bodies[b].root_body();
    let joint = SphericalJointBuilder::new()
        .local_anchor1(Vector::new(0.6, 0.0, 0.0))
        .local_anchor2(Vector::new(-0.6, 0.0, 0.0));
    world.insert_impulse_joint(root_a, root_b, joint);

    world.soft_bodies[a].apply_impulse(Vector::new(1.5, 0.0, 0.0), true);
    let momentum = |world: &PhysicsWorld, h: SoftBodyHandle| -> Vector {
        world.soft_bodies[h]
            .particles()
            .iter()
            .map(|p| p.velocity() * p.mass())
            .fold(Vector::ZERO, |a, b| a + b)
    };
    let p0 = momentum(&world, a) + momentum(&world, b);
    for _ in 0..120 {
        world.step();
    }
    let p1 = momentum(&world, a) + momentum(&world, b);
    assert!(
        (p1 - p0).length() < 0.05 * p0.length().max(1.0),
        "momentum drifted: {p0:?} -> {p1:?}"
    );
    // And the joint dragged the second jelly along.
    assert!(momentum(&world, b).x > 0.2);
}

/// The angular axes of a joint on a rank-deficient (single-particle) cluster are stripped: a
/// fixed joint behaves as a point weld, and the other body keeps spinning freely.
#[test]
fn angular_axes_are_stripped_on_a_point_cluster() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let h = cloth_at(&mut world, Vector::new(0.0, 2.0, 0.0));
    let cluster = world
        .soft_bodies
        .add_cluster(h, &[0], &mut world.bodies, &mut world.colliders)
        .unwrap();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    let corner = world.soft_bodies[h].particle_position(0);
    // No collider: the spinner would otherwise be expelled by the cloth's own surface
    // contacts, which is contact behavior, not the joint's.
    let spinner = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(corner)
            .angvel(Vector::new(0.0, 3.0, 0.0))
            .additional_mass_properties(MassProperties::from_ball(2.0, 0.05)),
    );
    let joint = FixedJointBuilder::new()
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::ZERO);
    world.insert_impulse_joint(spinner, proxy, joint);
    for _ in 0..60 {
        world.step();
    }
    // The spinner's rotation must remain free: a zero-inertia particle cannot define an
    // orientation, and the strip keeps the joint from freezing the spinner instead.
    let w = world.bodies[spinner].angvel();
    assert!(
        w.y > 2.5,
        "the fixed joint froze the spinner's rotation (angvel {w:?})"
    );
    // While its position stays welded to the corner.
    let d = (world.bodies[spinner].position().translation
        - world.soft_bodies[h].particle_position(0))
    .length();
    assert!(d < 0.08, "the point weld drifted ({d})");
}

/// Inserting a joint to a sleeping soft body wakes it, and the joint couples their islands.
#[test]
fn joint_insertion_wakes_the_soft_body() {
    let mut world = world_with_ground();
    let h = cloth_at(&mut world, Vector::new(0.0, 0.05, 0.0));
    for _ in 0..2000 {
        world.step();
        if world.soft_bodies[h].is_sleeping() {
            break;
        }
    }
    assert!(world.soft_bodies[h].is_sleeping(), "cloth never slept");
    let root = world.soft_bodies[h].root_body();
    let (body, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(3.0, 3.0, 0.0)),
        ColliderBuilder::ball(0.2),
    );
    let joint = RopeJointBuilder::new(4.0);
    world.insert_impulse_joint(body, root, joint);
    world.step();
    assert!(!world.soft_bodies[h].is_sleeping());
    assert!(!world.bodies[body].is_sleeping());
}

/// A soft cluster jointed to a multibody link: the joint goes through the generic (jacobian)
/// path and still holds.
#[test]
fn soft_cluster_jointed_to_a_multibody() {
    let mut world = PhysicsWorld::new();
    // A two-link multibody arm hanging from a fixed root.
    let root = world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 3.0, 0.0)));
    let (link1, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 3.0, 0.0)),
        ColliderBuilder::cuboid(0.4, 0.1, 0.1).density(1.0),
    );
    let j1 = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::new(-1.0, 0.0, 0.0));
    world
        .multibody_joints
        .insert(root, link1, j1, true)
        .unwrap();

    let h = jelly_at(&mut world, Vector::new(2.0, 3.0, 0.0));
    let sb_root = world.soft_bodies[h].root_body();
    let joint = SphericalJointBuilder::new()
        .local_anchor1(Vector::new(0.5, 0.0, 0.0))
        .local_anchor2(Vector::new(-0.5, 0.0, 0.0));
    world.insert_impulse_joint(link1, sb_root, joint);

    for _ in 0..240 {
        world.step();
    }
    // The jelly hangs from the arm instead of free-falling.
    let com = world.soft_bodies[h].center_of_mass();
    assert!(com.y > 0.5, "the jelly fell as if unjointed (com {com:?})");
    assert!(com.is_finite());
}

/// The joint pipeline works on a FEM-solved body too: `SoftFemSystem::propagate` answers the
/// scattered joint impulse with the whole body's elasticity.
#[cfg(feature = "fem")]
#[test]
fn joint_on_a_fem_body_holds() {
    let mut world = PhysicsWorld::new();
    let h = world.insert_soft_body(
        SoftBodyBuilder::cuboid(Vector::new(0.0, 0.0, 0.0), Vector::splat(0.4), 3, 3, 3)
            .cell_model(SoftBodyCellModel::Corotational)
            .solver(SoftBodySolver::Fem)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e4,
                poisson_ratio: 0.3,
                elastic_damping_ratio: 1.0,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05)),
    );
    let root = world.soft_bodies[h].root_body();
    let anchor =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 2.0, 0.0)));
    let joint = RopeJointBuilder::new(3.0)
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::ZERO);
    world.insert_impulse_joint(anchor, root, joint);
    for _ in 0..300 {
        world.step();
    }
    let com = world.soft_bodies[h].center_of_mass();
    assert!(com.is_finite());
    let dist = (com - Vector::new(0.0, 2.0, 0.0)).length();
    assert!(dist < 3.3, "the FEM jelly fell past its rope (dist {dist})");
}

/// A NaN injected into a jointed soft body lands in the quarantine: the body is disabled, the
/// joint partner survives, the world keeps stepping.
#[test]
fn quarantine_contains_a_nan_on_a_jointed_cluster() {
    let mut world = world_with_ground();
    let h = cloth_at(&mut world, Vector::new(0.0, 1.0, 0.0));
    let root = world.soft_bodies[h].root_body();
    let (partner, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 1.0, 0.0)),
        ColliderBuilder::ball(0.1),
    );
    world.insert_impulse_joint(partner, root, RopeJointBuilder::new(2.0));
    for _ in 0..10 {
        world.step();
    }
    world.soft_bodies[h].set_particle_velocity(0, Vector::NAN);
    for _ in 0..10 {
        world.step();
    }
    assert!(
        !world.soft_bodies[h].is_enabled(),
        "the NaN body must be quarantined"
    );
    assert!(world.bodies[partner].position().translation.is_finite());
}

/// Tearing a jointed cloth keeps the joint anchored to its (possibly renumbered) cluster: the
/// duplicated particles join their source's clusters and the proxy stays live.
#[test]
fn tearing_keeps_cluster_joints_alive() {
    let mut world = world_with_ground();
    let h = world.insert_soft_body(
        SoftBodyBuilder::cloth(
            Vector::new(0.0, 2.0, 0.0),
            Vector::X * 0.2,
            Vector::Z * 0.2,
            6,
            6,
        )
        .tear_strain(0.2),
    );
    let cluster = world
        .soft_bodies
        .add_cluster(h, &[0, 1, 6, 7], &mut world.bodies, &mut world.colliders)
        .unwrap();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    let anchor = world.insert_body(
        RigidBodyBuilder::fixed().translation(world.bodies[proxy].position().translation),
    );
    world.insert_impulse_joint(anchor, proxy, SphericalJointBuilder::new());
    world.step();
    // Tear a couple of middle edges.
    let torn = world.tear_soft_body(h, &[28, 30], &[]);
    assert!(torn.is_some());
    world.soft_bodies[h].validate_topology().unwrap();
    for _ in 0..60 {
        world.step();
    }
    // The jointed corner still hangs near its anchor while the cloth drapes.
    let d = (world.bodies[proxy].position().translation
        - world.bodies[anchor].position().translation)
        .length();
    assert!(d < 0.3, "the jointed cluster strayed after the tear ({d})");
    for p in world.soft_bodies[h].particles() {
        assert!(p.position().is_finite());
    }
}

/// A violently dragged mouse-joint grab of a one-particle cluster keeps bounded particle speeds
/// (the cluster's roundoff inertia is pseudo-inverted with an absolute noise floor).
#[test]
fn violently_dragged_one_particle_cluster_stays_bounded() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(18.0, 0.1, 18.0),
    );
    let h = world.insert_soft_body(
        SoftBodyBuilder::cuboid(Vector::new(0.0, 0.61, 0.0), Vector::splat(0.5), 4, 4, 4)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e3,
                poisson_ratio: 0.35,
                elastic_damping_ratio: 0.8,
                ..Default::default()
            })
            .particle_mass(0.08)
            .particle_radius(0.06)
            .surface_collider(ColliderBuilder::ball(0.06)),
    );
    // Grab the top corner particle, like the testbed's mouse grab does.
    let corner = Vector::new(0.5, 1.1, 0.5);
    let nearest = (0..world.soft_bodies[h].num_particles())
        .min_by(|&a, &b| {
            let body = &world.soft_bodies[h];
            let da = (body.particle_position(a) - corner).length_squared();
            let db = (body.particle_position(b) - corner).length_squared();
            da.partial_cmp(&db).unwrap()
        })
        .unwrap() as u32;
    let anchor = world.soft_bodies[h].particle_position(nearest as usize);
    let cluster = world.add_soft_body_cluster(h, &[nearest]).unwrap();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    let mouse_body =
        world.insert_body(RigidBodyBuilder::kinematic_position_based().translation(anchor));
    let joint: GenericJoint = GenericJointBuilder::new(JointAxesMask::empty())
        .motor_position(JointAxis::LinX, 0.0, 1000.0, 50.0)
        .motor_position(JointAxis::LinY, 0.0, 1000.0, 50.0)
        .motor_position(JointAxis::LinZ, 0.0, 1000.0, 50.0)
        .into();
    world.insert_impulse_joint(mouse_body, proxy, joint);

    // A violent wide circular wave (~45 length-units/s of hand speed).
    let mut t: Real = 0.0;
    let mut peak: Real = 0.0;
    for _ in 0..150 {
        t += world.integration_parameters.dt;
        let target =
            anchor + Vector::new(3.0 * (15.0 * t).cos(), 1.5 + 1.5 * (15.0 * t).sin(), 0.0);
        world.bodies[mouse_body].set_next_kinematic_translation(target);
        world.bodies[proxy].wake_up(true);
        world.step();
        let body = &world.soft_bodies[h];
        let v = (0..body.num_particles())
            .map(|i| body.particle_velocity(i).length())
            .fold(0.0, Real::max);
        peak = peak.max(v);
    }
    assert!(world.quarantine().is_empty());
    // The drag itself moves at ~45; anything far past the joint's following speed is the
    // roundoff-inertia divergence (it reached 1e13 before the fix).
    assert!(peak < 500.0, "particle speeds diverged: peak {peak}");
}
