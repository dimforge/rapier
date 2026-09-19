//! Mouse-grab replay on a torn soft body (2D): the testbed grab (a one-particle cluster whose
//! proxy is pulled by a motor-only joint from a kinematic body) drags the broken bridge of the
//! `soft_tearing2` demo around; the simulation stays finite and the grab cluster in one piece.

use rapier2d::prelude::*;

/// Particle speed bound. The mouse target circles at 75 m/s and the grabbed piece, a soft body
/// of its own since the tear, follows it with some overshoot (healthy runs peak near 107 m/s);
/// the divergent grab exceeded 300,000 m/s.
const MAX_PARTICLE_SPEED: Real = 130.0;
/// Gains of the testbed mouse joint (`src_testbed/grab.rs`).
const GRAB_STIFFNESS: Real = 1000.0;
const GRAB_DAMPING: Real = 50.0;
/// The mouse lifts by this height over one second, then circles with this radius and frequency.
const GRAB_LIFT: Real = 3.0;
const GRAB_RADIUS: Real = 4.0;
const GRAB_FREQUENCY: Real = 3.0;

#[derive(Default, Debug)]
struct Outcome {
    peak_speed: Real,
    peak_time: Real,
    non_finite: Option<Real>,
    /// First (time, speed) past `MAX_PARTICLE_SPEED`.
    too_fast: Option<(Real, Real)>,
    /// First (time, piece count) with the grab cluster spanning several pieces.
    split_cluster: Option<(Real, usize)>,
    end_time: Real,
}

/// The bridge of `soft_tearing2` (pinned at both ends) with the heavy disk that breaks it.
fn bridge_scene() -> (PhysicsWorld, SoftBodyHandle) {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(30.0, 0.5),
    );
    let (nx, ny) = (31usize, 6usize);
    let idx = move |i: usize, j: usize| (i * ny + j) as u32;
    let bridge = SoftBodyBuilder::grid(Vector::new(-8.0, 4.0), Vector::new(3.0, 0.5), nx, ny)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e6,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 1.0,
            tear_strain: Some(0.35),
            ..Default::default()
        })
        .pinned_particles((0..ny).flat_map(move |j| [idx(0, j), idx(nx - 1, j)]))
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    let bridge = world.insert_soft_body(bridge);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(-8.0, 9.0)),
        ColliderBuilder::ball(0.6).density(20.0),
    );
    (world, bridge)
}

/// The free particle of the biggest piece nearest to that piece's free-particle centroid.
fn grab_particle(sb: &SoftBody) -> u32 {
    let pieces = sb.connected_pieces();
    let biggest = pieces.iter().max_by_key(|p| p.len()).unwrap();
    let free: Vec<u32> = biggest
        .iter()
        .copied()
        .filter(|&v| !sb.particles()[v as usize].is_pinned())
        .collect();
    let centroid = free
        .iter()
        .map(|&v| sb.particle_position(v as usize))
        .sum::<Vector>()
        / free.len() as Real;
    *free
        .iter()
        .min_by(|a, b| {
            let da = (sb.particle_position(**a as usize) - centroid).length();
            let db = (sb.particle_position(**b as usize) - centroid).length();
            da.total_cmp(&db)
        })
        .unwrap()
}

/// Grabs the bridge at `grab_time` and drags it until `end_time`, recording the first violation
/// of each check.
fn simulate(grab_time: Real, end_time: Real) -> Outcome {
    let (mut world, bridge) = bridge_scene();
    let dt = world.integration_parameters.dt;
    let mut outcome = Outcome::default();
    let mut grab: Option<(RigidBodyHandle, RigidBodyHandle, Vector)> = None;
    let mut t: Real = 0.0;

    while t < end_time {
        if grab.is_none() && t >= grab_time {
            let particle = grab_particle(&world.soft_bodies[bridge]);
            let anchor = world.soft_bodies[bridge].particle_position(particle as usize);
            let cluster = world
                .add_soft_body_cluster(bridge, &[particle])
                .expect("grab cluster refused");
            let proxy = world.soft_bodies[bridge].cluster_proxy(cluster).unwrap();
            let mouse =
                world.insert_body(RigidBodyBuilder::kinematic_position_based().translation(anchor));
            let joint = GenericJointBuilder::new(JointAxesMask::empty())
                .motor_position(JointAxis::LinX, 0.0, GRAB_STIFFNESS, GRAB_DAMPING)
                .motor_position(JointAxis::LinY, 0.0, GRAB_STIFFNESS, GRAB_DAMPING);
            world.insert_impulse_joint(mouse, proxy, joint);
            grab = Some((mouse, proxy, anchor));
        }
        if let Some((mouse, _, anchor)) = grab {
            let tg = t - grab_time;
            let lift = tg.min(1.0) * GRAB_LIFT;
            let phase = (tg - 1.0).max(0.0) * GRAB_FREQUENCY * std::f64::consts::TAU as Real;
            let circle = Vector::new(phase.sin(), 1.0 - phase.cos()) * GRAB_RADIUS;
            world.bodies[mouse].set_next_kinematic_translation(anchor + Vector::Y * lift + circle);
        }
        world.step();
        t += dt;
        outcome.end_time = t;

        let mut finite = world
            .bodies
            .iter()
            .all(|(_, rb)| rb.translation().is_finite() && rb.linvel().is_finite());
        let mut speed: Real = 0.0;
        for (_, sb) in world.soft_bodies.iter() {
            for p in sb.particles() {
                finite &= p.position().is_finite() && p.velocity().is_finite();
                speed = speed.max(p.velocity().length());
            }
        }
        if speed > outcome.peak_speed {
            outcome.peak_speed = speed;
            outcome.peak_time = t;
        }
        if speed > MAX_PARTICLE_SPEED && outcome.too_fast.is_none() {
            outcome.too_fast = Some((t, speed));
        }
        if let Some((_, proxy, _)) = grab {
            // The cluster follows the tears: its proxy says which body holds it now, and the
            // cluster must lie within one connected piece of that body.
            let rb = world.bodies.get(proxy).expect("grab proxy removed");
            let body = rb.soft_body().expect("grab proxy lost its soft body");
            let cluster = rb.soft_cluster().expect("grab proxy lost its cluster");
            let sb = &world.soft_bodies[body];
            let mut piece_of = vec![0usize; sb.num_particles()];
            for (k, piece) in sb.connected_pieces().iter().enumerate() {
                for &v in piece {
                    piece_of[v as usize] = k;
                }
            }
            let cl = sb.cluster(cluster).expect("grab cluster removed");
            let mut pieces: Vec<usize> = cl
                .particles()
                .iter()
                .map(|&v| piece_of[v as usize])
                .collect();
            pieces.sort_unstable();
            pieces.dedup();
            if pieces.len() > 1 && outcome.split_cluster.is_none() {
                outcome.split_cluster = Some((t, pieces.len()));
            }
        }
        if !finite {
            outcome.non_finite = Some(t);
            break;
        }
        // Past this speed the run has diverged; the rest of it is noise.
        if speed > 1.0e3 {
            break;
        }
    }
    outcome
}

/// Checks that grabbing the broken bridge at t = 3 s, lifting it 3 m and circling it (4 m radius,
/// 3 Hz) stays bounded.
#[test]
fn grabbing_the_broken_bridge_stays_bounded() {
    let outcome = simulate(3.0, 6.0);
    let mut failures = Vec::new();
    if let Some(t) = outcome.non_finite {
        failures.push(format!("non-finite state at t = {t:.3} s"));
    }
    if let Some((t, speed)) = outcome.too_fast {
        failures.push(format!(
            "particle speed {speed:.1} m/s past {MAX_PARTICLE_SPEED} m/s at t = {t:.3} s"
        ));
    }
    if let Some((t, pieces)) = outcome.split_cluster {
        failures.push(format!(
            "grab cluster spans {pieces} pieces at t = {t:.3} s"
        ));
    }
    assert!(
        failures.is_empty(),
        "{} (peak {:.1} m/s at t = {:.3} s, stopped at t = {:.3} s)",
        failures.join("; "),
        outcome.peak_speed,
        outcome.peak_time,
        outcome.end_time,
    );
}
