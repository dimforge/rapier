//! Mouse-grab replays on tearable soft bodies (3D): the testbed grab (a one-particle cluster whose
//! proxy a motor-only joint pulls from a kinematic body) drags the `soft_tearing3` cloths and bar.
//! The simulation must stay finite and bounded, and the grab cluster within one connected piece.

use rapier3d::prelude::*;

/// Collects the tear events of every step.
#[derive(Default)]
struct TearLog(std::sync::Mutex<Vec<SoftBodyTearEvent>>);

impl TearLog {
    fn drain(&self) -> Vec<SoftBodyTearEvent> {
        std::mem::take(&mut *self.0.lock().unwrap())
    }
}

impl EventHandler for TearLog {
    fn handle_collision_event(
        &self,
        _: &RigidBodySet,
        _: &ColliderSet,
        _: CollisionEvent,
        _: Option<&ContactPair>,
    ) {
    }
    fn handle_contact_force_event(
        &self,
        _: Real,
        _: &RigidBodySet,
        _: &ColliderSet,
        _: &ContactPair,
        _: Real,
    ) {
    }
    fn handle_soft_body_tear_event(&self, _: &SoftBodySet, event: &SoftBodyTearEvent) {
        self.0.lock().unwrap().push(event.clone());
    }
}

/// Particle speed bound. Healthy runs of this scene peak under 37 m/s (the ball shot through the
/// curtain) and the mouse target moves at under 10 m/s; the divergent grabs exceed 400 m/s.
const MAX_PARTICLE_SPEED: Real = 80.0;
/// Gains of the testbed mouse joint (`src_testbed/grab.rs`).
const GRAB_STIFFNESS: Real = 1000.0;
const GRAB_DAMPING: Real = 50.0;
/// The mouse lifts by this height over one second, then circles with this radius and frequency.
const GRAB_LIFT: Real = 1.0;
const GRAB_RADIUS: Real = 1.5;
const GRAB_FREQUENCY: Real = 1.0;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum Target {
    Sheet,
    Curtain,
    Bar,
}

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

struct Scene {
    world: PhysicsWorld,
    soft_bodies: Vec<(Target, SoftBodyHandle)>,
    /// Right-end bar particles (holding body, index there) and their rest positions pulled from
    /// t = 1 s; they follow the bar's pieces through the tears.
    bar_pull: Vec<(SoftBodyHandle, u32, Vector)>,
}

fn cloth_material() -> SoftBodyMaterial {
    SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        tear_strain: Some(0.4),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(100.0, 1.0))
    }
}

/// The `soft_tearing3` scene: a pinned sheet hit by a falling ball, a curtain shot through by a
/// fast ball, and a bar pulled apart by its kinematic ends.
fn scene() -> Scene {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(30.0, 0.1, 30.0),
    );

    let n = 40usize;
    let border = (0..n * n).filter_map(move |k| {
        let (i, j) = (k / n, k % n);
        (i == 0 || j == 0 || i == n - 1 || j == n - 1).then_some(k as u32)
    });
    let sheet = SoftBodyBuilder::cloth(
        Vector::new(-6.0, 3.0, -2.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    .pinned_particles(border)
    .material(cloth_material())
    .particle_mass(0.02)
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.8));
    let sheet = world.insert_soft_body(sheet);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(-4.0, 6.0, 0.0)),
        ColliderBuilder::ball(0.6).density(15.0),
    );

    let curtain = SoftBodyBuilder::cloth(
        Vector::new(0.0, 4.5, 4.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, -0.1, 0.0),
        50,
        40,
    )
    .pinned_particles((0..50).map(|i| (i * 40) as u32))
    .material(cloth_material())
    .particle_mass(0.02);
    let curtain = world.insert_soft_body(curtain);
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(2.5, 2.5, 12.0))
            .linvel(Vector::new(0.0, 0.0, -25.0)),
        ColliderBuilder::ball(0.5).density(10.0),
    );

    let bar = SoftBodyBuilder::cuboid(
        Vector::new(3.0, 1.0, -4.0),
        Vector::new(2.0, 0.4, 0.4),
        21,
        5,
        5,
    )
    .cell_model(SoftBodyCellModel::Corotational)
    .material(SoftBodyMaterial {
        young_modulus: 5.0e4,
        poisson_ratio: 0.3,
        elastic_damping_ratio: 1.0,
        tear_strain: Some(0.4),
        ..Default::default()
    })
    .particle_mass(0.05)
    .surface_collider(ColliderBuilder::ball(0.1).friction(0.8));
    let bar = world.insert_soft_body(bar);
    let sb = &mut world.soft_bodies[bar];
    let mut bar_pull = Vec::new();
    for i in 0..sb.num_particles() {
        let p = sb.particle_position(i);
        if p.x < 1.01 || p.x > 4.99 {
            sb.set_particle_pinned(i, true);
        }
        if p.x > 4.99 {
            bar_pull.push((bar, i as u32, p));
        }
    }

    Scene {
        world,
        soft_bodies: vec![
            (Target::Sheet, sheet),
            (Target::Curtain, curtain),
            (Target::Bar, bar),
        ],
        bar_pull,
    }
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

/// Grabs `target` at `grab_time` and drags it until `end_time`, recording the first violation of
/// each check.
fn simulate(target: Target, grab_time: Real, end_time: Real) -> Outcome {
    let Scene {
        mut world,
        soft_bodies,
        mut bar_pull,
    } = scene();
    let grabbed = soft_bodies.iter().find(|(t, _)| *t == target).unwrap().1;
    let log = TearLog::default();
    let dt = world.integration_parameters.dt;
    let mut outcome = Outcome::default();
    let mut grab: Option<(RigidBodyHandle, RigidBodyHandle, Vector)> = None;
    let mut t: Real = 0.0;

    while t < end_time {
        if grab.is_none() && t >= grab_time {
            let particle = grab_particle(&world.soft_bodies[grabbed]);
            let anchor = world.soft_bodies[grabbed].particle_position(particle as usize);
            let cluster = world
                .add_soft_body_cluster(grabbed, &[particle])
                .expect("grab cluster refused");
            let proxy = world.soft_bodies[grabbed].cluster_proxy(cluster).unwrap();
            let mouse =
                world.insert_body(RigidBodyBuilder::kinematic_position_based().translation(anchor));
            let joint = GenericJointBuilder::new(JointAxesMask::empty())
                .motor_position(JointAxis::LinX, 0.0, GRAB_STIFFNESS, GRAB_DAMPING)
                .motor_position(JointAxis::LinY, 0.0, GRAB_STIFFNESS, GRAB_DAMPING)
                .motor_position(JointAxis::LinZ, 0.0, GRAB_STIFFNESS, GRAB_DAMPING);
            world.insert_impulse_joint(mouse, proxy, joint);
            grab = Some((mouse, proxy, anchor));
        }
        if let Some((mouse, _, anchor)) = grab {
            let tg = t - grab_time;
            let lift = tg.min(1.0) * GRAB_LIFT;
            let phase = (tg - 1.0).max(0.0) * GRAB_FREQUENCY * std::f64::consts::TAU as Real;
            let circle = Vector::new(phase.sin(), 1.0 - phase.cos(), 0.0) * GRAB_RADIUS;
            world.bodies[mouse].set_next_kinematic_translation(anchor + Vector::Y * lift + circle);
        }
        let shift = ((t - 1.0).max(0.0) * 0.5).min(4.0);
        for &(body, i, rest) in &bar_pull {
            world.soft_bodies[body]
                .set_particle_kinematic_target(i as usize, rest + Vector::new(shift, 0.0, 0.0));
        }
        world.step_with_events(&(), &log);
        for event in log.drain() {
            for pull in &mut bar_pull {
                if event.soft_body == pull.0 {
                    if let Some((body, i)) = event.particle_destination(pull.1) {
                        (pull.0, pull.1) = (body, i);
                    }
                }
            }
        }
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

fn check(target: Target, grab_time: Real, end_time: Real) {
    let outcome = simulate(target, grab_time, end_time);
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
        "{target:?} grabbed at t = {grab_time} s: {} (peak {:.1} m/s at t = {:.3} s, stopped at t = {:.3} s)",
        failures.join("; "),
        outcome.peak_speed,
        outcome.peak_time,
        outcome.end_time,
    );
}

/// Checks the sheet stays bounded when grabbed before the ball lands.
#[test]
fn grabbing_the_sheet_stays_bounded() {
    check(Target::Sheet, 0.5, 2.0);
}

/// Checks the torn curtain stays bounded when grabbed after the ball went through.
#[test]
fn grabbing_the_torn_curtain_stays_bounded() {
    check(Target::Curtain, 4.0, 8.0);
}

/// Checks the stretched bar stays bounded when grabbed while its ends are pulled apart.
#[test]
fn grabbing_the_stretched_bar_stays_bounded() {
    check(Target::Bar, 4.0, 6.0);
}
