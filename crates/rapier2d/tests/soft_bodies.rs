//! Soft-body regression tests (2D): substep invariance, ropes, blobs (area preservation),
//! triangulated bodies, shape matching, two-way coupling, sleeping and removal.

use rapier2d::prelude::*;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(50.0, 0.5).translation(Vector::new(0.0, -0.5)),
    );
    world
}

fn assert_finite(world: &PhysicsWorld, handle: SoftBodyHandle) {
    for p in world.soft_bodies[handle].particles() {
        assert!(p.position().is_finite(), "non-finite particle position");
    }
}

/// A soft body and every body split off it by tears, recursively (see `SoftBody::pieces`).
fn family(world: &PhysicsWorld, handle: SoftBodyHandle) -> Vec<SoftBodyHandle> {
    let mut bodies = vec![handle];
    let mut i = 0;
    while i < bodies.len() {
        bodies.extend(world.soft_bodies[bodies[i]].pieces().iter().copied());
        i += 1;
    }
    bodies
}

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

/// Where a particle driven by index is after the tears of a step: the body and index the events
/// moved it to.
fn follow(
    events: &[SoftBodyTearEvent],
    mut body: SoftBodyHandle,
    mut i: u32,
) -> (SoftBodyHandle, u32) {
    for event in events {
        if event.soft_body == body {
            if let Some(destination) = event.particle_destination(i) {
                (body, i) = destination;
            }
        }
    }
    (body, i)
}

/// The period of a mass-spring pair set by its natural frequency does not depend on the
/// substep count.
#[test]
fn edge_period_is_substep_invariant() {
    let natural_frequency = 2.0;
    let mut periods = Vec::new();
    for substeps in [1usize, 4, 16] {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.num_solver_iterations = substeps;
        world.integration_parameters.dt = 1.0 / 240.0;
        let builder = SoftBodyBuilder::new(vec![Vector::ZERO, Vector::new(1.0, 0.0)])
            .edges(vec![[0, 1]])
            .pinned_particles([0])
            .softness(SpringCoefficients::new(natural_frequency, 0.0))
            .no_surface_collider()
            .can_sleep(false);
        let handle = world.insert_soft_body(builder);
        world.soft_bodies[handle].set_particle_position(1, Vector::new(1.2, 0.0));
        let mut crossings = Vec::new();
        let mut prev_len = 1.2;
        for step in 0..2000 {
            world.step();
            let len = world.soft_bodies[handle].particle_position(1).x;
            if prev_len > 1.0 && len <= 1.0 {
                crossings.push(step as Real * world.integration_parameters.dt);
                if crossings.len() == 2 {
                    break;
                }
            }
            prev_len = len;
        }
        assert_eq!(
            crossings.len(),
            2,
            "no oscillation with {substeps} substeps"
        );
        periods.push(crossings[1] - crossings[0]);
    }
    let expected = 1.0 / natural_frequency;
    for period in &periods {
        assert!(
            (period - expected).abs() < 0.08 * expected,
            "period {period} vs {expected}"
        );
    }
}

/// A rope pinned at one end hangs under gravity.
#[test]
fn rope_hangs_from_anchor() {
    let mut world = PhysicsWorld::new();
    let rope = SoftBodyBuilder::rope(Vector::ZERO, Vector::new(2.0, 0.0), 21)
        .pinned_particles([0])
        .softness(SpringCoefficients::new(60.0, 1.0))
        .linear_damping(1.0);
    let handle = world.insert_soft_body(rope);
    for _ in 0..600 {
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    let end = sb.particle_position(20);
    assert!(end.y < -1.9, "rope end did not fall: {end:?}");
    assert!(
        end.x.abs() < 0.2,
        "rope end did not settle under the anchor: {end:?}"
    );
}

/// A disk with area preservation dropped on the ground keeps its area; inflating it grows it.
#[test]
fn disk_keeps_its_area() {
    let mut world = world_with_ground();
    let disk = SoftBodyBuilder::disk(Vector::new(0.0, 1.5), 1.0, 32)
        .softness(SpringCoefficients::new(20.0, 1.0));
    let handle = world.insert_soft_body(disk);
    let rest = world.soft_bodies[handle].rest_volume();
    assert!(rest > 3.0, "disk area too small: {rest}");
    for _ in 0..300 {
        world.step();
    }
    assert_finite(&world, handle);
    let area = world.soft_bodies[handle].volume();
    assert!(
        (area - rest).abs() < 0.15 * rest,
        "area drifted: {area} vs {rest}"
    );
    let min_y = world.soft_bodies[handle]
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MAX, Real::min);
    assert!(min_y > -0.05 && min_y < 0.3, "disk sank or floats: {min_y}");

    world.soft_bodies[handle].set_volume_factor(1.5);
    for _ in 0..300 {
        world.step();
    }
    let inflated = world.soft_bodies[handle].volume();
    assert!(
        inflated > 1.25 * rest,
        "disk did not inflate: {inflated} vs {rest}"
    );
}

/// A triangulated square (both cell models) dropped on the ground settles without inverting
/// any cell.
#[test]
fn grid_settles() {
    for model in [SoftBodyCellModel::Volume, SoftBodyCellModel::Corotational] {
        let mut world = world_with_ground();
        let grid = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::splat(0.5), 5, 5)
            .cell_model(model)
            .softness(SpringCoefficients::new(30.0, 1.0));
        let handle = world.insert_soft_body(grid);
        let rest = world.soft_bodies[handle].rest_volume();
        assert!((rest - 1.0).abs() < 1.0e-3, "square rest area {rest}");
        for _ in 0..400 {
            world.step();
        }
        assert_finite(&world, handle);
        let sb = &world.soft_bodies[handle];
        let area = sb.volume();
        assert!(
            (area - rest).abs() < 0.2 * rest,
            "{model:?}: area drifted: {area} vs {rest}"
        );
        for c in sb.cells() {
            let x: [Vector; 3] =
                core::array::from_fn(|k| sb.particle_position(c.vertices[k] as usize));
            let a = (x[1] - x[0]).perp_dot(x[2] - x[0]) * 0.5;
            assert!(a > 0.0, "{model:?}: inverted cell (area {a})");
        }
    }
}

/// A very stiff corotational square (E = 1e8, 0.1 kg particles), squeezed by 20% and dropped
/// spinning, settles and sleeps; a free-floating one keeps its angular momentum.
#[test]
fn stiff_elastic_square_is_stable() {
    let square = |young: Real| {
        SoftBodyBuilder::grid(Vector::new(0.0, 3.0), Vector::splat(0.75), 5, 5)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: young,
                poisson_ratio: 0.4,
                elastic_damping_ratio: 0.5,
                ..Default::default()
            })
            .particle_mass(0.1)
    };
    let spin = |world: &mut PhysicsWorld, handle: SoftBodyHandle, squeeze: Real| {
        let sb = &mut world.soft_bodies[handle];
        for i in 0..sb.num_particles() {
            let mut t = sb.particle_position(i);
            t.y = 3.0 + (t.y - 3.0) * squeeze;
            sb.set_particle_position(i, t);
            let arm = t - Vector::new(0.0, 3.0);
            sb.set_particle_velocity(i, Vector::new(-arm.y, arm.x) * 5.0);
        }
    };
    for young in [3.0e4, 3.0e6, 1.0e8] {
        let mut world = world_with_ground();
        let handle = world.insert_soft_body(square(young));
        spin(&mut world, handle, 0.8);
        for _ in 0..500 {
            world.step();
        }
        assert_finite(&world, handle);
        let sb = &world.soft_bodies[handle];
        let max_vel = sb
            .particle_velocities()
            .map(|v| v.length())
            .fold(0.0, Real::max);
        assert!(
            max_vel < 0.05,
            "E = {young}: square still moving at {max_vel} m/s"
        );
        assert!(sb.is_sleeping(), "E = {young}: square did not fall asleep");
        let area = sb.volume();
        assert!(
            (area - sb.rest_volume()).abs() < 0.05 * sb.rest_volume(),
            "E = {young}: area {area} vs rest {}",
            sb.rest_volume()
        );
    }

    let angular_momentum = |world: &PhysicsWorld, handle: SoftBodyHandle| {
        let sb = &world.soft_bodies[handle];
        let com = sb.center_of_mass();
        sb.particles()
            .iter()
            .map(|p| (p.position() - com).perp_dot(p.velocity() * p.mass()))
            .sum::<Real>()
    };
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let handle = world.insert_soft_body(square(3.0e6));
    spin(&mut world, handle, 1.0);
    world.step();
    let l0 = angular_momentum(&world, handle);
    for _ in 0..600 {
        world.step();
    }
    let l1 = angular_momentum(&world, handle);
    assert!(
        (l1 - l0).abs() < 0.01 * l0.abs(),
        "angular momentum drifted: {l0} -> {l1}"
    );
}

/// The same squeezed, spinning Neo-Hookean square dropped at E = 1e4, 1e6 and 1e8 settles,
/// sleeps and keeps its area.
#[test]
fn stiff_neo_hookean_square_is_stable() {
    for young in [1.0e4, 1.0e6, 1.0e8] {
        let mut world = world_with_ground();
        let square = SoftBodyBuilder::grid(Vector::new(0.0, 3.0), Vector::splat(0.75), 5, 5)
            .cell_model(SoftBodyCellModel::NeoHookean)
            .material(SoftBodyMaterial {
                young_modulus: young,
                poisson_ratio: 0.4,
                elastic_damping_ratio: 0.5,
                ..Default::default()
            })
            .particle_mass(0.1);
        let handle = world.insert_soft_body(square);
        {
            let sb = &mut world.soft_bodies[handle];
            for i in 0..sb.num_particles() {
                let mut t = sb.particle_position(i);
                t.y = 3.0 + (t.y - 3.0) * 0.8;
                sb.set_particle_position(i, t);
                let arm = t - Vector::new(0.0, 3.0);
                sb.set_particle_velocity(i, Vector::new(-arm.y, arm.x) * 5.0);
            }
        }
        for _ in 0..500 {
            world.step();
        }
        assert_finite(&world, handle);
        let sb = &world.soft_bodies[handle];
        let max_vel = sb
            .particle_velocities()
            .map(|v| v.length())
            .fold(0.0, Real::max);
        assert!(
            max_vel < 0.05,
            "E = {young}: square still moving at {max_vel} m/s"
        );
        assert!(sb.is_sleeping(), "E = {young}: square did not fall asleep");
        let area = sb.volume();
        assert!(
            (area - sb.rest_volume()).abs() < 0.05 * sb.rest_volume(),
            "E = {young}: area {area} vs rest {}",
            sb.rest_volume()
        );
    }
}

/// A soft Neo-Hookean square squashed to 30% of its height by a kinematic plate keeps its cells
/// positively oriented (up to a couple of transient flips under the plate while held) and
/// recovers its area within 5% once the plate lifts, with no inverted cell left.
#[test]
fn neo_hookean_square_survives_large_compression() {
    let mut world = world_with_ground();
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 0.5), Vector::splat(0.5), 5, 5)
        .cell_model(SoftBodyCellModel::NeoHookean)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e4,
            poisson_ratio: 0.45,
            elastic_damping_ratio: 1.0,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.03)
        .surface_collider(ColliderBuilder::ball(0.03).friction(0.5));
    let handle = world.insert_soft_body(square);
    // Plate bottom at 1.2 when lifted, 0.3 when down.
    let plate_rest = Vector::new(0.0, 1.7);
    let (plate, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(plate_rest),
        ColliderBuilder::cuboid(2.0, 0.5),
    );
    let inverted = |world: &PhysicsWorld| {
        let sb = &world.soft_bodies[handle];
        sb.cells()
            .iter()
            .filter(|c| {
                let x: [Vector; 3] =
                    core::array::from_fn(|k| sb.particle_position(c.vertices[k] as usize));
                (x[1] - x[0]).perp_dot(x[2] - x[0]) < 0.0
            })
            .count()
    };
    let dt = world.integration_parameters.dt;
    let mut t: Real = 0.0;
    let mut min_height = Real::MAX;
    for _ in 0..(7.0 / dt) as usize {
        t += dt;
        let depth = if t < 2.0 {
            t / 2.0
        } else if t < 3.0 {
            1.0
        } else if t < 5.0 {
            (5.0 - t) / 2.0
        } else {
            0.0
        };
        world.bodies[plate]
            .set_next_kinematic_translation(plate_rest - Vector::new(0.0, 0.9 * depth));
        world.step();
        assert_finite(&world, handle);
        if (2.0..3.0).contains(&t) {
            let sb = &world.soft_bodies[handle];
            let top = sb.particle_positions().map(|p| p.y).fold(0.0, Real::max);
            min_height = min_height.min(top);
            let inverted = inverted(&world);
            assert!(
                inverted <= 2,
                "{inverted} inverted cells while squashed (t = {t})"
            );
        }
    }
    assert!(
        min_height < 0.35,
        "the plate did not squash the square: {min_height}"
    );
    assert_eq!(inverted(&world), 0, "inverted cells after release");
    let sb = &world.soft_bodies[handle];
    let area = sb.volume();
    assert!(
        (area - sb.rest_volume()).abs() < 0.05 * sb.rest_volume(),
        "area {area} vs rest {}",
        sb.rest_volume()
    );
}

/// The Neo-Hookean material must not depend on the substep count either: the equilibrium
/// compression of a soft column standing on its pinned base under its own weight, and its
/// small-oscillation period, are the same at 1, 4 and 16 substeps.
#[test]
fn neo_hookean_response_is_substep_invariant() {
    let mut compressions = Vec::new();
    let mut periods = Vec::new();
    for substeps in [1usize, 4, 16] {
        let mut world = PhysicsWorld::new();
        world.integration_parameters.num_solver_iterations = substeps;
        let mut column = SoftBodyBuilder::grid(Vector::new(0.0, 0.5), Vector::splat(0.5), 3, 3)
            .cell_model(SoftBodyCellModel::NeoHookean)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e2,
                poisson_ratio: 0.3,
                elastic_damping_ratio: 0.05,
                ..Default::default()
            })
            .particle_mass(1.0)
            .can_sleep(false)
            .no_surface_collider();
        let base: Vec<u32> = (0..column.positions.len() as u32)
            .filter(|i| column.positions[*i as usize].y < 1.0e-3)
            .collect();
        column = column.pinned_particles(base);
        let handle = world.insert_soft_body(column);
        let top: Vec<usize> = world.soft_bodies[handle]
            .particle_positions()
            .enumerate()
            .filter(|(_, p)| p.y > 0.999)
            .map(|(i, _)| i)
            .collect();
        let height = |world: &PhysicsWorld| {
            let sb = &world.soft_bodies[handle];
            top.iter().map(|i| sb.particle_position(*i).y).sum::<Real>() / top.len() as Real
        };
        for _ in 0..600 {
            world.step();
        }
        let equilibrium = height(&world);
        compressions.push(1.0 - equilibrium);
        for &i in &top {
            world.soft_bodies[handle].set_particle_velocity(i, Vector::new(0.0, 0.3));
        }
        let mut crossings = Vec::new();
        let mut prev = height(&world);
        for step in 0..1000 {
            world.step();
            let y = height(&world);
            if prev > equilibrium && y <= equilibrium {
                crossings.push(step as Real * world.integration_parameters.dt);
                if crossings.len() == 3 {
                    break;
                }
            }
            prev = y;
        }
        assert_eq!(
            crossings.len(),
            3,
            "no oscillation with {substeps} substeps"
        );
        periods.push((crossings[2] - crossings[0]) / 2.0);
    }
    let spread = |v: &[Real]| {
        let (min, max) = v
            .iter()
            .fold((Real::MAX, Real::MIN), |(a, b), x| (a.min(*x), b.max(*x)));
        (max - min) / max
    };
    assert!(
        compressions.iter().all(|c| *c > 0.05),
        "the column barely compresses: {compressions:?}"
    );
    assert!(
        spread(&compressions) < 0.08,
        "the equilibrium compression depends on the substep count: {compressions:?}"
    );
    assert!(
        spread(&periods) < 0.08,
        "the oscillation period depends on the substep count: {periods:?}"
    );
}

/// A closed surface expels bodies that cross it: a small rigid ball spawned straddling a soft
/// square's edge from inside (and a particle of a second soft body likewise) come out instead
/// of being trapped, and the square itself stays whole.
#[test]
fn closed_surface_expels_intruders() {
    let mut world = world_with_ground();
    world.gravity = Vector::ZERO;
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 3.0), Vector::splat(0.75), 5, 5)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e4,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.05);
    let square = world.insert_soft_body(square);
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.1, 3.68)),
        ColliderBuilder::ball(0.1),
    );
    let intruder = SoftBodyBuilder::new(vec![
        Vector::new(0.72, 3.1),
        Vector::new(1.2, 3.0),
        Vector::new(1.0, 3.5),
    ])
    .cells(vec![[0, 1, 2]])
    .particle_radius(0.05)
    .particle_mass(0.1);
    let intruder = world.insert_soft_body(intruder);
    for _ in 0..300 {
        world.step();
    }
    assert_finite(&world, square);
    let sb = &world.soft_bodies[square];
    let inside = |p: Vector| {
        let com = sb.center_of_mass();
        (p - com).abs().max_element() < 0.75 - 0.05
    };
    let ball_pos = world.bodies[ball].translation();
    assert!(
        !inside(ball_pos),
        "rigid ball still inside the square: {ball_pos:?}"
    );
    for p in world.soft_bodies[intruder].particles() {
        assert!(
            !inside(p.position()),
            "intruder particle still inside: {:?}",
            p.position()
        );
    }
    let area = sb.volume();
    assert!(
        (area - sb.rest_volume()).abs() < 0.1 * sb.rest_volume(),
        "square area {area} vs rest {}",
        sb.rest_volume()
    );
}

/// A shape-matched cloud returns to its rest shape after being scrambled.
#[test]
fn shape_matching_recovers_rest_shape() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let mut positions = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            positions.push(Vector::new(i as Real, j as Real) * 0.5);
        }
    }
    let builder = SoftBodyBuilder::new(positions.clone())
        .shape_matching(true)
        .softness(SpringCoefficients::new(10.0, 1.0))
        .no_surface_collider()
        .can_sleep(false);
    let handle = world.insert_soft_body(builder);
    {
        let sb = &mut world.soft_bodies[handle];
        for (i, p) in positions.iter().enumerate() {
            let x = ((i * 7919) % 100) as Real / 100.0 - 0.5;
            let y = ((i * 104729) % 100) as Real / 100.0 - 0.5;
            sb.set_particle_position(i, *p + Vector::new(x, y) * 0.6);
        }
    }
    for _ in 0..600 {
        world.step();
    }
    let sb = &world.soft_bodies[handle];
    let mut max_err: Real = 0.0;
    for a in 0..positions.len() {
        for b in a + 1..positions.len() {
            let rest = (positions[a] - positions[b]).length();
            let cur = (sb.particle_position(a) - sb.particle_position(b)).length();
            max_err = max_err.max((rest - cur).abs());
        }
    }
    assert!(
        max_err < 0.05,
        "shape not recovered, max distance error {max_err}"
    );
}

/// Two-way coupling: a light box (0.36 kg) dropped just above a heavy damped ring blob (40 kg)
/// rests on its contact skin, not inside it, and squashes it.
#[test]
fn rigid_box_rests_on_blob() {
    let mut world = world_with_ground();
    let blob = SoftBodyBuilder::disk(Vector::new(0.0, 1.0), 1.0, 40)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .linear_damping(0.5);
    let handle = world.insert_soft_body(blob);
    let skin = world.soft_bodies[handle].particle_radius();
    let (rb, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.4)),
        ColliderBuilder::cuboid(0.3, 0.3),
    );
    for _ in 0..600 {
        world.step();
    }
    assert_finite(&world, handle);
    let box_y = world.bodies[rb].translation().y;
    let sb = &world.soft_bodies[handle];
    let max_y = sb
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MIN, Real::max);
    assert!(max_y < 2.0, "blob was not squashed: {max_y}");
    let box_bottom = box_y - 0.3;
    assert!(
        (box_bottom - (max_y + skin)).abs() < 0.02,
        "box bottom {box_bottom} is not resting on the blob's skin ({max_y} + {skin})"
    );
    let speed = world.bodies[rb].linvel().length();
    assert!(speed < 0.01, "box did not settle: {speed} m/s");
}

/// A soft body sleeps as a unit, wakes when hit, and is removed cleanly.
#[test]
fn soft_body_sleeps_wakes_and_is_removed() {
    let mut world = world_with_ground();
    let blob = SoftBodyBuilder::grid(Vector::new(0.0, 0.5), Vector::splat(0.5), 4, 4)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .linear_damping(0.5);
    let handle = world.insert_soft_body(blob);
    let num_bodies_before = world.bodies.len();
    for _ in 0..1200 {
        world.step();
    }
    assert!(
        world.soft_bodies[handle].is_sleeping(),
        "soft body did not fall asleep"
    );
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 3.0)),
        ColliderBuilder::ball(0.2),
    );
    let mut woke = false;
    for _ in 0..200 {
        world.step();
        if !world.soft_bodies[handle].is_sleeping() {
            woke = true;
            break;
        }
    }
    assert!(woke, "soft body was not woken by the ball");
    let sb = world.remove_soft_body(handle).unwrap();
    assert!(world.bodies.get(sb.root_body()).is_none());
    // One root body was removed by `remove_soft_body`, and one ball body was inserted since
    // `num_bodies_before` was captured: the counts cancel out.
    assert_eq!(world.bodies.len(), num_bodies_before);
    for _ in 0..10 {
        world.step();
    }
}

/// A snapshot of a world with soft bodies round-trips through serde.
#[cfg(feature = "serde-serialize")]
#[test]
fn soft_bodies_serialize() {
    let mut world = world_with_ground();
    let disk = SoftBodyBuilder::disk(Vector::new(0.0, 1.5), 0.5, 16);
    let handle = world.insert_soft_body(disk);
    for _ in 0..50 {
        world.step();
    }
    let bytes = bincode::serialize(&world).unwrap();
    let mut restored: PhysicsWorld = bincode::deserialize(&bytes).unwrap();
    for _ in 0..50 {
        world.step();
        restored.step();
    }
    let a = world.soft_bodies[handle].particle_position(3);
    let b = restored.soft_bodies[handle].particle_position(3);
    assert!(
        (a - b).length() < 1.0e-3,
        "restored world diverged: {a:?} vs {b:?}"
    );
}

/// Surface (polyline) collisions: a box resting on a triangulated soft square neither sinks in
/// nor slides, and nothing creeps at rest.
#[test]
fn box_rests_on_surface_and_nothing_creeps() {
    let mut world = world_with_ground();
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 0.6), Vector::splat(0.6), 6, 6)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 5.0e3,
            poisson_ratio: 0.35,
            ..Default::default()
        })
        .particle_mass(0.2);
    let handle = world.insert_soft_body(square);
    assert!(world.soft_bodies[handle].collision_mesh().is_some());
    let (rb, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.1, 2.0)),
        ColliderBuilder::cuboid(0.2, 0.2).density(2.0),
    );
    for _ in 0..600 {
        world.step();
    }
    assert_finite(&world, handle);
    let box_pos = world.bodies[rb].translation();
    assert!(
        box_pos.y > 1.2,
        "box sank into the soft square: {box_pos:?}"
    );
    assert!(
        box_pos.y < 1.7,
        "box floats above the soft square: {box_pos:?}"
    );
    assert!(
        (box_pos.x - 0.1).abs() < 0.05,
        "box slid on the soft square: {box_pos:?}"
    );
    let com = world.soft_bodies[handle].center_of_mass();
    assert!(com.x.abs() < 0.02, "soft square crept: {com:?}");
    for v in world.soft_bodies[handle].particle_velocities() {
        assert!(v.length() < 0.01);
    }
}

/// Soft-vs-soft: a blob dropped on a soft square rests on top of it.
#[test]
fn blob_rests_on_blob() {
    let mut world = world_with_ground();
    let bottom = SoftBodyBuilder::grid(Vector::new(0.0, 0.8), Vector::splat(0.8), 6, 6)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.1);
    let bottom = world.insert_soft_body(bottom);
    let top = SoftBodyBuilder::disk(Vector::new(0.1, 2.6), 0.6, 24)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.05);
    let top = world.insert_soft_body(top);
    for _ in 0..400 {
        world.step();
    }
    assert_finite(&world, top);
    let top_min = world.soft_bodies[top]
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MAX, Real::min);
    let bottom_max = world.soft_bodies[bottom]
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MIN, Real::max);
    assert!(
        top_min > 0.9,
        "top blob fell through the bottom one: {top_min}"
    );
    assert!(
        top_min < bottom_max + 0.3,
        "top blob floats: {top_min} vs {bottom_max}"
    );
}

/// A strip pinched between two jelly squares, all colliding through their polylines, settles
/// and sleeps: the polyline-vs-particle constraints of both sides agree and see live deformation.
#[test]
fn strip_pinched_between_soft_squares_settles() {
    let mut world = world_with_ground();
    let jelly = |center: Vector| {
        SoftBodyBuilder::grid(center, Vector::splat(0.5), 4, 4)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 1.0e4,
                poisson_ratio: 0.4,
                elastic_damping_ratio: 0.5,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(0.08)
            .surface_collider(ColliderBuilder::ball(0.08).friction(0.7))
    };
    let strip = SoftBodyBuilder::grid(Vector::new(0.0, 1.4), Vector::new(0.9, 0.06), 16, 2)
        .cell_model(SoftBodyCellModel::Volume)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.03)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
    let handles = [
        world.insert_soft_body(jelly(Vector::new(0.0, 0.6))),
        world.insert_soft_body(strip),
        world.insert_soft_body(jelly(Vector::new(0.0, 2.2))),
    ];
    for _ in 0..900 {
        world.step();
    }
    for h in handles {
        assert_finite(&world, h);
        assert!(
            world.soft_bodies[h].is_sleeping(),
            "the sandwich did not settle within 15 s"
        );
    }
    let com = |h| world.soft_bodies[h].center_of_mass();
    assert!(com(handles[2]).y > com(handles[1]).y && com(handles[1]).y > com(handles[0]).y);
    assert!(
        com(handles[2]).y > 1.6,
        "top square sank: {:?}",
        com(handles[2])
    );
}

/// Edge-vs-edge (segment-vs-segment) contacts: a taut horizontal rope pushed sideways into a
/// taut vertical rope crosses it between the vertices of both (no vertex ever comes within the
/// vertex contact reach); the crossing segments must hold each other instead of passing through.
#[test]
fn crossing_ropes_hold_through_segment_contacts() {
    let mut world = PhysicsWorld::new();
    let rope = |start: Vector, end: Vector| {
        SoftBodyBuilder::rope(start, end, 5)
            .softness(SpringCoefficients::new(60.0, 1.0))
            .particle_mass(0.05)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05))
            .pinned_particles([0, 4])
    };
    // Vertical rope at x = 1 (vertices at y = 0.5, 1, 1.5, 2, 2.5).
    let vertical = world.insert_soft_body(rope(Vector::new(1.0, 0.5), Vector::new(1.0, 2.5)));
    // Horizontal rope at y = 1.25 (vertices at x = -1.5, -1, ..., 0.5); its pinned ends move
    // 0.75 right over 3 s so its last segment crosses the vertical rope's second segment
    // 0.25 away from every vertex.
    let horizontal = world.insert_soft_body(rope(Vector::new(-1.5, 1.25), Vector::new(0.5, 1.25)));
    let pins: Vec<(usize, Vector)> = [0usize, 4]
        .iter()
        .map(|&i| (i, world.soft_bodies[horizontal].particle_position(i)))
        .collect();
    for step in 0..600 {
        let t = (step + 1) as Real / 60.0;
        let shift = 0.75 * (t / 3.0).min(1.0);
        for &(i, rest) in &pins {
            world.soft_bodies[horizontal]
                .set_particle_kinematic_target(i, rest + Vector::new(shift, 0.0));
        }
        world.step();
    }
    assert_finite(&world, vertical);
    let v = &world.soft_bodies[vertical];
    let h = &world.soft_bodies[horizontal];
    // The vertical rope is bowed to the right by the crossing segment.
    let bow = v.particle_position(2).x;
    assert!(bow > 1.15, "the ropes passed through each other: bow {bow}");
    // The crossing segments stay two particle radii apart.
    let (a0, a1) = (v.particle_position(1), v.particle_position(2));
    let (b0, b1) = (h.particle_position(3), h.particle_position(4));
    let (la, lb) = parry2d::query::details::closest_points_segment_segment_with_locations_nD(
        (&a0, &a1),
        (&b0, &b1),
    );
    let pa = a0 * la.barycentric_coordinates()[0] + a1 * la.barycentric_coordinates()[1];
    let pb = b0 * lb.barycentric_coordinates()[0] + b1 * lb.barycentric_coordinates()[1];
    let gap = (pb - pa).length();
    assert!(gap > 0.08, "crossing segments too close: {gap}");
}

/// `SoftBodyBuilder::volumetric` fills a closed polygon with conforming triangular cells: a disk
/// filled with cells a fifth of its radius has about the disk's area, no inverted cell, and rests
/// on the ground.
#[test]
fn volumetric_disk_is_a_valid_soft_body() {
    let n = 32;
    let vertices: Vec<Vector> = (0..n)
        .map(|i| {
            let a = i as Real / n as Real * core::f32::consts::TAU;
            Vector::new(a.cos(), 1.5 + a.sin())
        })
        .collect();
    let indices: Vec<[u32; 2]> = (0..n).map(|i| [i as u32, ((i + 1) % n) as u32]).collect();
    let builder = SoftBodyBuilder::volumetric(&vertices, &indices, 0.2)
        .expect("the disk is filled with cells")
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 5.0e4,
            poisson_ratio: 0.3,
            ..Default::default()
        })
        .particle_mass(0.05);
    let mut world = world_with_ground();
    let handle = world.insert_soft_body(builder);
    let sb = &world.soft_bodies[handle];
    let area = sb.rest_volume();
    let disk_area = core::f32::consts::PI;
    assert!(
        (area - disk_area).abs() < 0.3 * disk_area,
        "volumetric area {area} vs disk {disk_area}"
    );
    for c in sb.cells() {
        let x: [Vector; 3] = core::array::from_fn(|k| sb.particle_position(c.vertices[k] as usize));
        let a = (x[1] - x[0]).perp_dot(x[2] - x[0]) * 0.5;
        assert!(a > 0.0, "inverted cell (area {a})");
    }
    for _ in 0..300 {
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    let min_y = sb
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MAX, Real::min);
    assert!(
        min_y > -0.05 && min_y < 0.3,
        "the disk sank or floats: {min_y}"
    );
}

/// A cell torn far past its rest shape recovers without snapping back at hundreds of m/s: the
/// strain error fed to the elastic bias is clamped, continuously through the inversion.
#[test]
fn torn_cell_recovers_without_snapping() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let strip = SoftBodyBuilder::grid(Vector::new(0.0, 5.0), Vector::new(1.2, 0.12), 13, 2)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e4,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.03)
        .particle_radius(0.06)
        .no_surface_collider();
    let handle = world.insert_soft_body(strip);
    // Drag a middle particle 8 m away, through the strip (its cells end up inverted, with a
    // 40x stretch), and let go.
    let sb = &world.soft_bodies[handle];
    let torn = (0..sb.num_particles())
        .min_by(|&a, &b| {
            let d = |i: usize| (sb.particle_position(i) - Vector::new(0.0, 5.12)).length();
            d(a).partial_cmp(&d(b)).unwrap()
        })
        .unwrap();
    world.soft_bodies[handle].set_particle_position(torn, Vector::new(0.0, -3.0));
    let mut max_speed: Real = 0.0;
    for _ in 0..600 {
        world.step();
        for v in world.soft_bodies[handle].particle_velocities() {
            max_speed = max_speed.max(v.length());
        }
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    let mut inverted = 0;
    let mut worst_ratio: Real = 0.0;
    for c in sb.cells() {
        let x: [Vector; 3] = core::array::from_fn(|k| sb.particle_position(c.vertices[k] as usize));
        let area = (x[1] - x[0]).perp_dot(x[2] - x[0]) * 0.5;
        if area < 0.0 {
            inverted += 1;
        }
        worst_ratio = worst_ratio.max((area / c.rest_volume - 1.0).abs());
    }
    assert_eq!(inverted, 0, "cells still inverted");
    assert!(
        worst_ratio < 0.2,
        "a cell is still off its rest area by {worst_ratio}"
    );
    // Uncapped, the recovery through the inversion peaked above 150 m/s.
    assert!(max_speed < 60.0, "recovery snapped at {max_speed} m/s");
}

/// A closed surface only expels intruders through elements whose winding can be trusted: an
/// element belonging to an inverted cell has a mirrored winding, and a ball resting on it must
/// not be "expelled" through the body.
#[test]
fn inverted_cell_elements_do_not_expel() {
    let mut world = world_with_ground();
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 0.75), Vector::splat(0.75), 4, 4)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e4,
            poisson_ratio: 0.4,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.05);
    let handle = world.insert_soft_body(square);
    // Pin the top edge, its two middle particles swapped and raised: the segment between them
    // runs backwards (its cell is inverted), so its winding normal points into the square, and
    // it stands alone above the rest of the top edge.
    let sb = &world.soft_bodies[handle];
    let n = sb.num_particles();
    let mut top: Vec<usize> = (0..n)
        .filter(|&i| sb.particle_position(i).y > 1.49)
        .collect();
    top.sort_by(|&a, &b| {
        sb.particle_position(a)
            .x
            .partial_cmp(&sb.particle_position(b).x)
            .unwrap()
    });
    let (a, b) = (top[1], top[2]);
    for &i in &top {
        world.soft_bodies[handle].set_particle_pinned(i, true);
    }
    for (i, x) in [(a, 0.6), (b, -0.6)] {
        world.soft_bodies[handle].set_particle_position(i, Vector::new(x, 1.9));
    }
    // A ball resting on that segment only.
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.9 + 0.05 + 0.15)),
        ColliderBuilder::ball(0.15),
    );
    for _ in 0..120 {
        world.step();
    }
    assert_finite(&world, handle);
    // Resting on the segment: 1.9 + skin 0.05 + radius 0.15 (a mirrored-winding expulsion
    // pushed it down to 1.93).
    let y = world.bodies[ball].translation().y;
    assert!(
        y > 2.05,
        "the ball was pushed into the square through a mirrored element: y = {y}"
    );
}

/// A pressurized body turned inside out is accepted as mirrored: its area target follows its
/// orientation and its outward normals flip with it, so it stays a blob and supports a box.
#[test]
fn inverted_blob_is_accepted_as_mirrored() {
    let mut world = world_with_ground();
    let blob = SoftBodyBuilder::disk(Vector::new(0.0, 0.7), 0.6, 20)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .volume_factor(1.1)
        .particle_mass(0.05)
        .particle_radius(0.06);
    let handle = world.insert_soft_body(blob);
    // Mirror it (same shape, reversed winding).
    {
        let sb = &mut world.soft_bodies[handle];
        for i in 0..sb.num_particles() {
            let p = sb.particle_position(i);
            sb.set_particle_position(i, Vector::new(-p.x, p.y));
        }
    }
    let (block, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.8)),
        ColliderBuilder::cuboid(0.3, 0.2).density(0.3),
    );
    for _ in 0..240 {
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    let area = sb.volume();
    assert!(
        (area.abs() - 1.1 * sb.rest_volume()).abs() < 0.15 * sb.rest_volume(),
        "mirrored blob area {area} vs target {}",
        1.1 * sb.rest_volume()
    );
    let y = world.bodies[block].translation().y;
    assert!(
        y > 1.0,
        "the block did not rest on the mirrored blob: y = {y}"
    );
}

/// A fixed thin pin inside a blob pressed on the ground by a plate cannot be expelled: the
/// expulsion is demoted to a quiet two-sided contact and the blob rests and sleeps.
#[test]
fn stuck_expulsion_settles() {
    let mut world = world_with_ground();
    let blob = SoftBodyBuilder::disk(Vector::new(0.0, 0.6), 0.6, 24)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .volume_factor(1.1)
        .particle_mass(0.05)
        .particle_radius(0.06)
        .surface_collider(ColliderBuilder::ball(0.06).friction(0.6));
    let handle = world.insert_soft_body(blob);
    // The pin, just above the blob's bottom, fully inside.
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, 0.18)),
        ColliderBuilder::capsule_x(0.3, 0.04),
    );
    // A plate lowered onto the blob during the first second, then held.
    let (plate, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 1.5)),
        ColliderBuilder::cuboid(1.5, 0.2),
    );
    let dt = world.integration_parameters.dt;
    let mut slept_at = None;
    let mut max_speed_after_3s: Real = 0.0;
    for step in 0..(10.0 / dt) as usize {
        let t = step as Real * dt;
        let y = 1.5 - 0.5 * t.min(1.0);
        world.bodies[plate].set_next_kinematic_translation(Vector::new(0.0, y));
        world.step();
        let sb = &world.soft_bodies[handle];
        if t > 3.0 {
            for v in sb.particle_velocities() {
                max_speed_after_3s = max_speed_after_3s.max(v.length());
            }
        }
        if sb.is_sleeping() {
            slept_at = Some(t);
            break;
        }
    }
    assert_finite(&world, handle);
    assert!(
        max_speed_after_3s < 0.5,
        "the blob keeps fighting the pin: {max_speed_after_3s} m/s"
    );
    assert!(
        slept_at.is_some_and(|t| t < 10.0),
        "the blob never fell asleep (slept at {slept_at:?})"
    );
}

/// A fully pinned rope (every particle kinematic) still collides through its polyline: a box
/// dropped on it rests on it instead of falling through (a body without a free particle used to
/// be left out of the solver's awake list, so its surface had no contact constraint).
#[test]
fn fully_pinned_rope_holds_a_box() {
    let mut world = world_with_ground();
    let n = 11;
    let rope = SoftBodyBuilder::rope(Vector::new(-0.5, 1.0), Vector::new(0.5, 1.0), n)
        .pinned_particles(0..n as u32)
        .surface_collider(ColliderBuilder::ball(0.03));
    let handle = world.insert_soft_body(rope);
    let (block, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.0)),
        ColliderBuilder::cuboid(0.15, 0.15),
    );
    for _ in 0..240 {
        world.step();
    }
    assert_finite(&world, handle);
    // Resting on the polyline: 1.0 + half-height 0.15 + the surface's skin (the particle radius).
    let y = world.bodies[block].translation().y;
    assert!(y > 1.14, "the box fell through the pinned rope: y = {y}");
    assert!(y < 1.3, "the box did not settle on the rope: y = {y}");
    let sb = &world.soft_bodies[handle];
    for i in 0..sb.num_particles() {
        assert!(
            (sb.particle_position(i).y - 1.0).abs() < 1.0e-6,
            "a pinned particle moved: {:?}",
            sb.particle_position(i)
        );
    }
}

/// The pinned particles moved kinematically (the whole rope rising) lift the resting box up
/// with them.
#[test]
fn kinematic_pinned_rope_pushes_a_box() {
    let mut world = world_with_ground();
    let n = 11;
    let rope = SoftBodyBuilder::rope(Vector::new(-0.5, 1.0), Vector::new(0.5, 1.0), n)
        .pinned_particles(0..n as u32)
        .surface_collider(ColliderBuilder::ball(0.03));
    let handle = world.insert_soft_body(rope);
    let (block, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.5)),
        ColliderBuilder::cuboid(0.15, 0.15),
    );
    for _ in 0..120 {
        world.step();
    }
    let rested = world.bodies[block].translation().y;
    assert!(
        rested > 1.14,
        "the box fell through the pinned rope: y = {rested}"
    );
    // Raise the rope at 0.5 m/s for two seconds.
    let dt = world.integration_parameters.dt;
    let num_particles = world.soft_bodies[handle].num_particles();
    let mut rope_y = 1.0;
    for _ in 0..120 {
        rope_y += 0.5 * dt;
        for i in 0..num_particles {
            let mut pos = world.soft_bodies[handle].particle_position(i);
            pos.y = rope_y;
            world.soft_bodies[handle].set_particle_kinematic_target(i, pos);
        }
        world.step();
        let y = world.bodies[block].translation().y;
        assert!(
            y > rope_y + 0.1,
            "the rising rope passed through the box: box y = {y}, rope y = {rope_y}"
        );
    }
    assert_finite(&world, handle);
    let y = world.bodies[block].translation().y;
    assert!(
        y > rope_y + 0.14 && y < rope_y + 0.35,
        "the box does not ride the rope: box y = {y}, rope y = {rope_y}"
    );
}

/// A triangulated strip pinned at both ends and pulled past its tear strain rips under the volume
/// model (edges tear) and the corotational one (cells tear on their principal strain); cracks
/// split particles without removing cells and the pieces keep consistent tables.
#[test]
fn strip_tears_when_pulled_apart() {
    for model in [SoftBodyCellModel::Volume, SoftBodyCellModel::Corotational] {
        let mut world = PhysicsWorld::new();
        let (nx, ny) = (13usize, 3usize);
        let strip = SoftBodyBuilder::grid(Vector::new(0.0, 2.0), Vector::new(1.2, 0.2), nx, ny)
            .cell_model(model)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e3,
                tear_strain: Some(0.4),
                ..Default::default()
            })
            .particle_mass(0.05);
        let handle = world.insert_soft_body(strip);
        let sb = &world.soft_bodies[handle];
        let (num_particles, num_cells, measure) =
            (sb.num_particles(), sb.cells().len(), sb.rest_measure());
        let left: Vec<usize> = (0..sb.num_particles())
            .filter(|&i| sb.particle_position(i).x < -1.19)
            .collect();
        let right: Vec<usize> = (0..sb.num_particles())
            .filter(|&i| sb.particle_position(i).x > 1.19)
            .collect();
        assert_eq!(left.len(), ny);
        for &i in left.iter().chain(right.iter()) {
            world.soft_bodies[handle].set_particle_pinned(i, true);
        }
        let rest: Vec<Vector> = right
            .iter()
            .map(|&i| world.soft_bodies[handle].particle_position(i))
            .collect();
        // The driven particles follow the tears: the piece they are in becomes its own body.
        let mut right: Vec<(SoftBodyHandle, u32)> =
            right.iter().map(|&i| (handle, i as u32)).collect();
        let log = TearLog::default();
        for k in 0..240 {
            let shift = 2.4 * (k as Real / 180.0).min(1.0);
            for (&(body, i), p) in right.iter().zip(rest.iter()) {
                world.soft_bodies[body]
                    .set_particle_kinematic_target(i as usize, *p + Vector::new(shift, 0.0));
            }
            world.step_with_events(&(), &log);
            let events = log.drain();
            for r in &mut right {
                *r = follow(&events, r.0, r.1);
            }
            for h in family(&world, handle) {
                assert_finite(&world, h);
                world.soft_bodies[h].validate_topology().unwrap();
            }
        }
        let bodies = family(&world, handle);
        assert!(bodies.len() >= 2, "{model:?}: the strip is still in one piece");
        let sum = |f: &dyn Fn(&SoftBody) -> Real| -> Real {
            bodies.iter().map(|&h| f(&world.soft_bodies[h])).sum()
        };
        assert_eq!(
            sum(&|sb| sb.cells().len() as Real),
            num_cells as Real,
            "{model:?}: a tear removed cells"
        );
        assert!(
            (sum(&|sb| sb.rest_measure()) - measure).abs() < 1.0e-4 * measure,
            "{model:?}: the rest area changed"
        );
        assert!(
            sum(&|sb| sb.num_particles() as Real) > num_particles as Real,
            "{model:?}: no particle split"
        );
        // Torn apart: no remaining cell edge is stretched past twice the tear strain.
        let mut max_stretch: Real = 0.0;
        for &h in &bodies {
            let sb = &world.soft_bodies[h];
            assert!(!sb.boundary().is_empty());
            for c in sb.cells() {
                for a in 0..3 {
                    for b in a + 1..3 {
                        let (pa, pb) = (
                            sb.particle_position(c.vertices[a] as usize),
                            sb.particle_position(c.vertices[b] as usize),
                        );
                        let rest = (sb.particles()[c.vertices[a] as usize].rest_position()
                            - sb.particles()[c.vertices[b] as usize].rest_position())
                        .length();
                        max_stretch = max_stretch.max((pa - pb).length() / rest);
                    }
                }
            }
        }
        assert!(
            max_stretch < 1.8,
            "{model:?}: a cell is still stretched by {max_stretch}: the strip did not split"
        );
    }
}

/// A particle two pieces share alone is duplicated: tearing a bottom edge of a one-cell-thick
/// strip opens a crack at its first endpoint, and the top particle left joining the halves alone
/// splits too. No cell is removed; mass and rest area are conserved.
#[test]
fn tearing_splits_the_particle_two_fans_share() {
    let mut world = world_with_ground();
    let ny = 2u32;
    let grid = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::new(1.0, 0.25), 5, 2)
        .particle_mass(0.5);
    let handle = world.insert_soft_body(grid);
    world.step();
    let sb = &world.soft_bodies[handle];
    let idx = |i: u32, j: u32| i * ny + j;
    let (bottom, top) = (idx(2, 0), idx(2, 1));
    let torn = [bottom, idx(3, 0)];
    let edge = sb
        .edges()
        .iter()
        .position(|e| e.vertices == torn || e.vertices == [torn[1], torn[0]])
        .expect("the bottom edge exists") as u32;
    let num_particles = sb.num_particles() as u32;
    let (num_cells, mass, measure) = (sb.cells().len(), sb.mass(), sb.rest_measure());
    let event = world.tear_soft_body(handle, &[edge], &[])
        .expect("nothing tore");
    world.soft_bodies[handle].validate_topology().unwrap();
    assert_eq!(event.soft_body, handle);
    assert_eq!(event.torn_edges.len(), 1);
    assert!(event.torn_edges[0] == torn || event.torn_edges[0] == [torn[1], torn[0]]);
    assert!(event.torn_cells.is_empty());
    assert_eq!(
        event.split_particles,
        vec![(num_particles, bottom), (num_particles + 1, top)]
    );
    // The originals stay with the left half, the copies go with the right one; the halves are
    // equal, so the left one (the smaller particle) keeps the handle and the right one is a new
    // body.
    assert_eq!(event.pieces.len(), 2);
    let left: Vec<u32> = (0..=top).collect();
    let right: Vec<u32> = (top + 1..num_particles + 2).collect();
    assert_eq!(event.pieces[0].soft_body, handle);
    assert_eq!(event.pieces[0].particles, left);
    assert_eq!(event.pieces[1].particles, right);
    let bodies: Vec<SoftBodyHandle> = event.bodies().collect();
    let sum = |f: &dyn Fn(&SoftBody) -> Real| -> Real {
        bodies.iter().map(|&h| f(&world.soft_bodies[h])).sum()
    };
    assert_eq!(sum(&|sb| sb.cells().len() as Real), num_cells as Real);
    assert!((sum(&|sb| sb.mass()) - mass).abs() < 1.0e-5);
    assert!((sum(&|sb| sb.rest_measure()) - measure).abs() < 1.0e-5);
    for _ in 0..120 {
        world.step();
        for &h in &bodies {
            assert_finite(&world, h);
        }
    }
    for &h in &bodies {
        world.soft_bodies[h].validate_topology().unwrap();
    }
}

/// `SoftBodyBuilder::trimesh` (2D): the triangles' vertices become the particles, their edges
/// springs, the boundary the surface (counter-clockwise, holes clockwise), and shape matching
/// keeps the shape once dropped.
#[test]
fn trimesh_soft_body_keeps_its_shape() {
    let mut world = world_with_ground();
    // A square with a square hole: 8 vertices, 8 triangles (given clockwise on purpose).
    let outer = [
        Vector::new(-1.0, -1.0),
        Vector::new(1.0, -1.0),
        Vector::new(1.0, 1.0),
        Vector::new(-1.0, 1.0),
    ];
    let inner = [
        Vector::new(-0.4, -0.4),
        Vector::new(0.4, -0.4),
        Vector::new(0.4, 0.4),
        Vector::new(-0.4, 0.4),
    ];
    let vertices: Vec<Vector> = outer
        .iter()
        .chain(inner.iter())
        .map(|v| *v + Vector::new(0.0, 2.0))
        .collect();
    let mut triangles = Vec::new();
    for k in 0..4u32 {
        let (a, b) = (k, (k + 1) % 4);
        triangles.push([a, 4 + a, b]);
        triangles.push([b, 4 + a, 4 + b]);
    }
    let square = SoftBodyBuilder::trimesh(vertices, triangles)
        .unwrap()
        .material(SoftBodyMaterial {
            shape_matching_softness: SpringCoefficients::new(30.0, 1.0),
            ..Default::default()
        })
        .particle_mass(0.1);
    let handle = world.insert_soft_body(square);
    let sb = &world.soft_bodies[handle];
    assert!(sb.cluster(0).unwrap().shape_matching_enabled());
    assert_eq!(sb.boundary().len(), 8, "outer and inner boundary segments");
    assert_eq!(sb.edges().len(), 16);
    assert!(sb.cells().is_empty());
    // Outer loop counter-clockwise, hole clockwise: a positive enclosed area of 4 - 0.64.
    assert!(
        (sb.rest_volume() - 3.36).abs() < 1.0e-4,
        "{}",
        sb.rest_volume()
    );
    assert!(SoftBodyBuilder::trimesh(Vec::new(), Vec::new()).is_none());
    for _ in 0..300 {
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    let pose = world.bodies[sb.cluster_proxy(0).unwrap()].position();
    let max_dev = sb
        .particles()
        .iter()
        .map(|p| (pose * p.rest_position() - p.position()).length())
        .fold(0.0, Real::max);
    assert!(max_dev < 0.05, "the square lost its shape: {max_dev}");
    let com = sb.center_of_mass();
    assert!(
        (com.y - 1.0 - sb.particle_radius()).abs() < 0.1,
        "the square did not rest on the ground: {com:?}"
    );
}

/// `SoftBodyBuilder::polyline` (2D): segments as springs and surface, bending edges at every
/// two-segment vertex, shape matching; a closed counter-clockwise polyline has an inside.
#[test]
fn polyline_soft_body() {
    let mut world = world_with_ground();
    let n = 12u32;
    let ring: Vec<Vector> = (0..n)
        .map(|i| {
            let a = i as Real / n as Real * core::f32::consts::TAU;
            Vector::new(a.cos(), a.sin() + 2.0)
        })
        .collect();
    let segments: Vec<[u32; 2]> = (0..n).map(|i| [i, (i + 1) % n]).collect();
    let ring = SoftBodyBuilder::polyline(ring, Some(segments)).unwrap();
    let handle = world.insert_soft_body(ring.particle_mass(0.05));
    let sb = &world.soft_bodies[handle];
    assert_eq!(sb.boundary().len(), 12);
    assert_eq!(sb.edges().len(), 24, "12 structural + 12 bending edges");
    assert!(sb.rest_volume() > 3.0, "{}", sb.rest_volume());
    // An open strip: consecutive vertices, one bending edge per interior vertex.
    let strip = SoftBodyBuilder::polyline(
        vec![
            Vector::new(3.0, 3.0),
            Vector::new(3.5, 3.0),
            Vector::new(4.0, 3.0),
            Vector::new(4.5, 3.0),
        ],
        None,
    )
    .unwrap();
    let strip = world.insert_soft_body(strip.particle_mass(0.05));
    assert_eq!(world.soft_bodies[strip].edges().len(), 3 + 2);
    assert!(SoftBodyBuilder::polyline(vec![Vector::ZERO], None).is_none());
    for _ in 0..300 {
        world.step();
    }
    assert_finite(&world, handle);
    assert_finite(&world, strip);
    let sb = &world.soft_bodies[handle];
    let com = sb.center_of_mass();
    assert!(
        (com.y - 1.0 - sb.particle_radius()).abs() < 0.15,
        "the ring did not rest on the ground as a ring: {com:?}"
    );
    let strip = &world.soft_bodies[strip];
    let strip_y = strip.center_of_mass().y;
    assert!(
        (strip_y - strip.particle_radius()).abs() < 0.05,
        "the strip did not fall flat: {strip_y}"
    );
}

// ---------------------------------------------------------------------------------------------
// The FEM soft-body solver (`fem` cargo feature, `SoftBodySolver::Fem`).
// ---------------------------------------------------------------------------------------------

/// A FEM soft body with no contacts falls exactly like a point mass, and does not deform.
#[cfg(feature = "fem")]
#[test]
fn fem_free_fall_is_exact() {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.num_solver_iterations = 1;
    let builder = SoftBodyBuilder::grid(Vector::new(0.0, 10.0), Vector::splat(0.5), 4, 4)
        .cell_model(SoftBodyCellModel::Corotational)
        .solver(SoftBodySolver::Fem)
        .no_surface_collider()
        .can_sleep(false);
    let handle = world.insert_soft_body(builder);
    let start = world.soft_bodies[handle].particle_position(0);
    let steps = 60;
    for _ in 0..steps {
        world.step();
    }
    let dt = world.integration_parameters.dt;
    let expected = world.gravity.y * dt * dt * (steps * (steps + 1) / 2) as Real;
    let sb = &world.soft_bodies[handle];
    let drop = sb.particle_position(0).y - start.y;
    assert!(
        (drop - expected).abs() < 1.0e-4 * expected.abs(),
        "free fall {drop} vs expected {expected}"
    );
    for i in 0..sb.num_particles() {
        let v = sb.particle_velocity(i) - sb.particle_velocity(0);
        assert!(v.length() < 1.0e-4, "particle {i} drifted at {v:?}");
    }
}

/// A stiff FEM square dropped on the ground settles, keeps its area and does not explode, at any
/// Young's modulus.
#[cfg(feature = "fem")]
#[test]
fn fem_stiff_square_is_stable() {
    for young in [3.0e4, 3.0e6, 1.0e8] {
        let mut world = world_with_ground();
        let square = SoftBodyBuilder::grid(Vector::new(0.0, 3.0), Vector::splat(0.75), 5, 5)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: young,
                poisson_ratio: 0.4,
                elastic_damping_ratio: 0.5,
                ..Default::default()
            })
            .solver(SoftBodySolver::Fem)
            .particle_mass(0.1);
        let handle = world.insert_soft_body(square);
        for _ in 0..500 {
            world.step();
        }
        assert_finite(&world, handle);
        let sb = &world.soft_bodies[handle];
        let max_vel = sb
            .particles()
            .iter()
            .map(|p| p.velocity().length())
            .fold(0.0, Real::max);
        assert!(
            max_vel < 0.2,
            "E = {young}: square still moving at {max_vel} m/s"
        );
        let area = sb.volume();
        assert!(
            (area - sb.rest_volume()).abs() < 0.1 * sb.rest_volume(),
            "E = {young}: area {area} vs rest {}",
            sb.rest_volume()
        );
    }
}

/// The FEM path's static deflection does not depend on the substep count.
#[cfg(feature = "fem")]
#[test]
fn fem_deflection_is_substep_invariant() {
    let (length, thickness, young) = (2.0, 0.2, 2.0e6);
    let settle = |substeps: usize| -> Real {
        let mut world = PhysicsWorld::new();
        world.integration_parameters.num_solver_iterations = substeps;
        let builder = SoftBodyBuilder::grid(
            Vector::new(length * 0.5, 0.0),
            Vector::new(length * 0.5, thickness * 0.5),
            13,
            3,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: young,
            poisson_ratio: 0.0,
            elastic_damping_ratio: 1.0,
            ..Default::default()
        })
        .solver(SoftBodySolver::Fem)
        .mass(2.0)
        .no_surface_collider()
        .can_sleep(false);
        let pinned: Vec<u32> = builder
            .particle_positions()
            .iter()
            .enumerate()
            .filter(|(_, p)| p.x < 1.0e-4)
            .map(|(i, _)| i as u32)
            .collect();
        let tip: Vec<usize> = builder
            .particle_positions()
            .iter()
            .enumerate()
            .filter(|(_, p)| p.x > length - 1.0e-4)
            .map(|(i, _)| i)
            .collect();
        let handle = world.insert_soft_body(builder.pinned_particles(pinned));
        for _ in 0..1500 {
            world.step();
        }
        let sb = &world.soft_bodies[handle];
        tip.iter().map(|&i| sb.particle_position(i).y).sum::<Real>() / tip.len() as Real
    };
    let deflections: Vec<Real> = [1usize, 2, 8].iter().map(|s| settle(*s)).collect();
    let reference = deflections[2];
    assert!(reference < -1.0e-3, "the beam did not deflect: {reference}");
    for (k, d) in deflections.iter().enumerate() {
        assert!(
            (d - reference).abs() < 0.05 * reference.abs(),
            "FEM deflection depends on the substep count: {deflections:?} (case {k})"
        );
    }
}

/// Contacts against a FEM soft body: a rigid box dropped on one settles on its surface.
#[cfg(feature = "fem")]
#[test]
fn fem_box_rests_on_soft_body() {
    let mut world = world_with_ground();
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 0.6), Vector::splat(0.6), 6, 6)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 5.0e3,
            poisson_ratio: 0.35,
            ..Default::default()
        })
        .solver(SoftBodySolver::Fem)
        .particle_mass(0.2);
    let handle = world.insert_soft_body(square);
    let (rb, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.05, 2.0)),
        ColliderBuilder::cuboid(0.2, 0.2).density(2.0),
    );
    for _ in 0..600 {
        world.step();
    }
    assert_finite(&world, handle);
    let box_pos = world.bodies[rb].translation();
    assert!(
        box_pos.y > 1.1,
        "box sank into the soft square: {box_pos:?}"
    );
    assert!(
        box_pos.y < 1.7,
        "box floats above the soft square: {box_pos:?}"
    );
    assert!(world.bodies[rb].linvel().length() < 0.05);
}

/// A stack of stiff FEM bodies rests on the ground without sinking: contacts see a FEM body
/// through its augmented mass (`J A⁻¹Jᵀ`), not the anchors' lumped masses.
#[cfg(feature = "fem")]
#[test]
fn fem_stiff_stack_rests_without_sinking() {
    let mut world = PhysicsWorld::new();
    let ground_top = 1.2;
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(25.0, ground_top),
    );
    let (half, radius) = (3.0, 0.15);
    let mut handles = Vec::new();
    for row in 0..3 {
        let center = Vector::new(0.0, ground_top + half + 0.5 + row as Real * (2.0 * half + 1.0));
        let square = SoftBodyBuilder::grid(center, Vector::splat(half), 4, 4)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 1.0e6,
                poisson_ratio: 0.4,
                elastic_damping_ratio: 0.5,
                deformation_damping: 25.0,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(radius)
            .self_contacts(true)
            .solver(SoftBodySolver::Fem)
            .can_sleep(false);
        handles.push(world.insert_soft_body(square));
    }
    for _ in 0..500 {
        world.step();
    }
    let mut previous_top = ground_top;
    for &h in &handles {
        let sb = &world.soft_bodies[h];
        assert_finite(&world, h);
        let min_y = sb.particles().iter().map(|p| p.position().y).fold(Real::MAX, Real::min);
        let max_y = sb.particles().iter().map(|p| p.position().y).fold(Real::MIN, Real::max);
        let max_vel = sb.particles().iter().map(|p| p.velocity().length()).fold(0.0, Real::max);
        let penetration = previous_top + radius - min_y;
        assert!(penetration < 0.02, "a body sank by {penetration} into the one below");
        assert!(max_vel < 0.1, "the stack did not settle: {max_vel} m/s");
        previous_top = max_y + radius;
    }
}

/// A `Volume` cell-model body on the FEM path keeps its area: its cells are volume elements
/// of the implicit step (no constraint is left on a FEM body).
#[cfg(feature = "fem")]
#[test]
fn fem_volume_cells_keep_their_area() {
    let mut world = world_with_ground();
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 2.0), Vector::splat(0.75), 5, 5)
        .cell_model(SoftBodyCellModel::Volume)
        .solver(SoftBodySolver::Fem)
        .particle_mass(0.1);
    let handle = world.insert_soft_body(square);
    for _ in 0..500 {
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    let max_vel = sb
        .particles()
        .iter()
        .map(|p| p.velocity().length())
        .fold(0.0, Real::max);
    assert!(max_vel < 0.2, "the square still moves at {max_vel} m/s");
    let area = sb.volume();
    assert!(
        (area - sb.rest_volume()).abs() < 0.1 * sb.rest_volume(),
        "area {area} vs rest {}",
        sb.rest_volume()
    );
}

/// A thin FEM feature (thinner than two skins) with self contacts on stays put: the permanent
/// self contacts across its thickness are excluded (`SoftCollisionMesh::self_contact_excluded`).
#[cfg(feature = "fem")]
#[test]
fn fem_thin_feature_self_contacts_stay_calm() {
    let mut world = world_with_ground();
    let bar = SoftBodyBuilder::grid(Vector::new(0.0, 2.0), Vector::new(2.0, 0.2), 11, 2)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e6,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.3)
        .self_contacts(true)
        .solver(SoftBodySolver::Fem)
        .can_sleep(false);
    let handle = world.insert_soft_body(bar);
    let positions = |world: &PhysicsWorld| -> Vec<Vector> {
        world.soft_bodies[handle]
            .particles()
            .iter()
            .map(|p| p.position())
            .collect()
    };
    for _ in 0..400 {
        world.step();
    }
    let settled = positions(&world);
    for _ in 0..800 {
        world.step();
    }
    assert_finite(&world, handle);
    let drift = positions(&world)
        .iter()
        .zip(&settled)
        .map(|(a, b)| (*a - *b).length())
        .fold(0.0, Real::max);
    let min_y = settled.iter().map(|p| p.y).fold(Real::MAX, Real::min);
    assert!(drift < 1.0e-3, "the bar keeps moving: drift {drift} over 800 steps");
    assert!(min_y > 0.3 - 0.02, "the bar sank into the ground: {min_y}");
}

/// 2D twin of `violently_dragged_one_particle_cluster_stays_bounded` (rapier3d's
/// soft_body_joints.rs): the 2D reduced inertia is a scalar whose plain inverse exploded the
/// same way on a one-particle cluster's roundoff inertia.
#[test]
fn violently_dragged_one_particle_cluster_stays_bounded() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(30.0, 0.5),
    );
    let h = world.insert_soft_body(
        SoftBodyBuilder::grid(Vector::new(0.0, 1.11), Vector::splat(0.5), 5, 5)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e3,
                poisson_ratio: 0.35,
                elastic_damping_ratio: 0.8,
                ..Default::default()
            })
            .particle_mass(0.08),
    );
    let corner = Vector::new(0.5, 1.61);
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
        .into();
    world.insert_impulse_joint(mouse_body, proxy, joint);

    let mut t: Real = 0.0;
    let mut peak: Real = 0.0;
    for _ in 0..150 {
        t += world.integration_parameters.dt;
        let target = anchor + Vector::new(3.0 * (15.0 * t).cos(), 1.5 + 1.5 * (15.0 * t).sin());
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
    assert!(peak < 500.0, "particle speeds diverged: peak {peak}");
}

/// With self-contacts a pressurized blob cannot be squeezed through itself into a crossed
/// figure-8 loop: it survives a grab-crush against the floor and re-inflates once released.
#[test]
fn self_contact_blob_survives_grab_crush() {
    let area_ratio = |world: &PhysicsWorld, sb: SoftBodyHandle| -> Real {
        let body = &world.soft_bodies[sb];
        let mut area = 0.0;
        let mut rest = 0.0;
        for seg in body.boundary() {
            let a = body.particle_position(seg[0] as usize);
            let b = body.particle_position(seg[1] as usize);
            area += 0.5 * (a.x * b.y - a.y * b.x);
            let ra = body.particles()[seg[0] as usize].rest_position();
            let rb = body.particles()[seg[1] as usize].rest_position();
            rest += 0.5 * (ra.x * rb.y - ra.y * rb.x);
        }
        area / rest
    };

    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(4.0, 0.5),
    );
    let mut blobs = Vec::new();
    for j in 0..3 {
        for i in 0..3 {
            let x = -1.2 + i as Real * 1.2 + (j % 2) as Real * 0.3;
            let y = 0.6 + j as Real * 1.1;
            blobs.push(
                world.insert_soft_body(
                    SoftBodyBuilder::disk(Vector::new(x, y), 0.5, 20)
                        .softness(SpringCoefficients::new(20.0, 1.0))
                        .volume_factor(1.05)
                        .self_contacts(true)
                        .particle_mass(0.05),
                ),
            );
        }
    }
    for _ in 0..240 {
        world.step();
    }

    // Grab a bottom-row blob's particle like the testbed mouse does, and park the target far
    // underground: the position-error-scaled motor crushes the blob against the floor.
    let victim = blobs[1];
    let grabbed = 0u32;
    let anchor = world.soft_bodies[victim].particle_position(grabbed as usize);
    let cluster = world.add_soft_body_cluster(victim, &[grabbed]).unwrap();
    let proxy = world.soft_bodies[victim].cluster_proxy(cluster).unwrap();
    let mouse =
        world.insert_body(RigidBodyBuilder::kinematic_position_based().translation(anchor));
    let joint: GenericJoint = GenericJointBuilder::new(JointAxesMask::empty())
        .motor_position(JointAxis::LinX, 0.0, 1000.0, 50.0)
        .motor_position(JointAxis::LinY, 0.0, 1000.0, 50.0)
        .into();
    world.insert_impulse_joint(mouse, proxy, joint);
    for step in 0..900 {
        let tt = step as Real / 60.0;
        world.bodies[mouse].set_next_kinematic_translation(Vector::new(
            anchor.x + 1.5 * (3.0 * tt).sin(),
            -8.0,
        ));
        world.bodies[proxy].wake_up(true);
        world.step();
    }
    // The crush itself must never cross the loop (the signed area keeps its sign).
    assert!(
        area_ratio(&world, victim) > 0.0,
        "the loop crossed despite self-contacts: ratio {}",
        area_ratio(&world, victim)
    );

    world.remove_body(mouse);
    world.remove_soft_body_cluster(victim, cluster);
    for _ in 0..600 {
        world.step();
    }
    assert!(world.quarantine().is_empty());
    // Every blob re-inflates: no stuck figure-8, no permanently crushed neighbor.
    for &b in &blobs {
        let r = area_ratio(&world, b);
        assert!(r > 0.6, "a blob stayed crushed after release: ratio {r}");
    }
}

/// A 1 kg weight hung from a pinned bar by the tested edge `(3, 4)`, which bears the whole
/// weight; a tear splits the edge's pinned end and the free copy falls with the weight. Bar and
/// weight are three segments each, the smallest piece a tear may split off.
fn hanging_weight(tear_force: Real) -> SoftBodyBuilder {
    SoftBodyBuilder::new(vec![
        Vector::new(-3.0, 2.0),
        Vector::new(-2.0, 2.0),
        Vector::new(-1.0, 2.0),
        Vector::new(0.0, 2.0),
        Vector::new(0.0, 1.0),
        Vector::new(0.0, 2.0 / 3.0),
        Vector::new(0.0, 1.0 / 3.0),
        Vector::new(0.0, 0.0),
    ])
    // The pinned bar, the tested edge, and the weight's own edges (reinforced).
    .edges(vec![[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 6], [6, 7]])
    .edge_tear_resistance([(4, 100.0), (5, 100.0), (6, 100.0)])
    .pinned_particles([0, 1, 2, 3])
    .particle_mass(0.25)
    .softness(SpringCoefficients::new(120.0, 1.0))
    .tear_force(tear_force)
    .no_surface_collider()
    .can_sleep(false)
}

/// A stiff edge holding a weight barely stretches, yet it bears the weight: with a tear force
/// below that weight it breaks, with one above it holds, and its `stress` reads the fraction of
/// the threshold it bears.
#[test]
fn stiff_edge_tears_under_force_not_strain() {
    let hang = |tear_force: Real| {
        let mut world = PhysicsWorld::new();
        let handle = world.insert_soft_body(hanging_weight(tear_force));
        for _ in 0..60 {
            world.step();
        }
        // A tear splits the bar off the weight as two bodies: the bar keeps the handle.
        let bodies = family(&world, handle);
        for &h in &bodies {
            world.soft_bodies[h].validate_topology().unwrap();
        }
        let sb = &world.soft_bodies[handle];
        let tested = sb.edges().iter().find(|e| e.vertices == [3, 4]);
        let particles: usize = bodies.iter().map(|&h| world.soft_bodies[h].num_particles()).sum();
        (
            particles,
            bodies.len(),
            tested.map(|e| e.stress()),
            sb.edges().len(),
        )
    };
    // The load is the weight, 9.81 N.
    let (particles, pieces, stress, edges) = hang(30.0);
    assert_eq!((particles, pieces, edges), (8, 1, 7), "the edge tore under a third of its threshold");
    let stress = stress.unwrap();
    assert!(
        (stress - 9.81 / 30.0).abs() < 0.08,
        "stress {stress} does not read the weight over the threshold"
    );
    // The tear keeps every edge and never splits the pinned end: the weight's top particle
    // splits, its copy stays on the bar as a stub and the weight falls as its own body. Edge
    // `[3, 4]` now bears the stub alone (three quarters of the split particle's mass).
    let (particles, pieces, stress, edges) = hang(6.0);
    assert_eq!((particles, pieces, edges), (9, 2, 4), "the edge held a load above its tear force");
    let stress = stress.unwrap();
    assert!(
        (stress - 0.1875 * 9.81 / 6.0).abs() < 0.03,
        "the tested edge does not bear the stub alone: {stress}"
    );
}

/// Tear smoothing: a jerk that would snap an edge at once is shrugged off once the load is
/// averaged over a time constant far longer than the spike.
#[test]
fn tear_smoothing_shrugs_off_a_spike() {
    let jerk = |smoothing: Real| {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        let handle = world.insert_soft_body(hanging_weight(50.0).tear_smoothing(smoothing));
        world.step();
        // A velocity the edge must cancel within a substep: a force spike of hundreds of N.
        for i in 4..8 {
            world.soft_bodies[handle].apply_particle_impulse(i, Vector::new(0.0, -0.75), true);
        }
        for _ in 0..30 {
            world.step();
        }
        // A tear splits a particle (it removes no edge), and the weight falls as its own body.
        family(&world, handle).len()
    };
    assert_eq!(jerk(0.0), 2, "without smoothing the spike snaps the edge");
    assert_eq!(jerk(1.0), 1, "smoothed over a second, the spike is shrugged off");
}

/// A per-edge tear resistance multiplies the edge's threshold: of two edges bearing the same
/// weight past the material's tear force, the reinforced one holds (above the 14 N catch
/// transient of a released chain). Each 1 kg weight is a reinforced chain of three segments.
#[test]
fn tear_resistance_scales_the_threshold() {
    let mut world = PhysicsWorld::new();
    let mut positions = vec![Vector::new(0.0, 2.0), Vector::new(1.0, 2.0)];
    for k in 0..4 {
        let y = 1.0 - k as Real / 3.0;
        positions.extend([Vector::new(0.0, y), Vector::new(1.0, y)]);
    }
    let builder = SoftBodyBuilder::new(positions)
    .edges(vec![[0, 1], [0, 2], [1, 3], [2, 4], [3, 5], [4, 6], [5, 7], [6, 8], [7, 9]])
    .edge_tear_resistance([(2, 3.0), (3, 100.0), (4, 100.0), (5, 100.0), (6, 100.0), (7, 100.0), (8, 100.0)])
    .pinned_particles([0, 1])
    .particle_mass(0.25)
    .softness(SpringCoefficients::new(120.0, 1.0))
    .tear_force(6.0)
    .no_surface_collider()
    .can_sleep(false);
    let handle = world.insert_soft_body(builder);
    for _ in 0..60 {
        world.step();
    }
    // The tear splits one particle and removes no edge: the plain chain's top particle splits,
    // the chain falls as its own body and the copy stays on the anchor as a stub; the rest keeps
    // the handle with particles renumbered. The reinforced edge is still there.
    let bodies = family(&world, handle);
    assert_eq!(bodies.len(), 2, "exactly one edge should have torn");
    let mut edges = 0;
    let mut particles = 0;
    for &h in &bodies {
        let sb = &world.soft_bodies[h];
        sb.validate_topology().unwrap();
        edges += sb.edges().len();
        particles += sb.num_particles();
    }
    assert_eq!((edges, particles), (9, 11));
    let sb = &world.soft_bodies[handle];
    let reinforced = sb.edges().iter().find(|e| e.tear_resistance == 3.0);
    let stress = reinforced.expect("the reinforced edge tore").stress();
    assert!((stress - 9.81 / 18.0).abs() < 0.08, "stress {stress} ignores the resistance");
}

/// An edge whose particles are all interior and undamaged bears its load against
/// `interior_strength` times the threshold, a surface edge against the threshold itself, and a
/// tear through a particle removes the shield. Checked on two worlds differing by that alone.
#[test]
fn interior_strength_shields_undamaged_interior_edges() {
    let n = 5usize;
    let idx = |i: usize, j: usize| (i * n + j) as u32;
    let run = |interior_strength: Real| {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        let grid = SoftBodyBuilder::grid(Vector::ZERO, Vector::splat(1.0), n, n)
            .material(SoftBodyMaterial {
                tear_strain: Some(1.0),
                interior_strength,
                ..Default::default()
            })
            // The crack opens at a free endpoint of the torn edge, chosen by the loads around
            // it, which the shield changes: pinning one endpoint makes both runs open the same
            // crack, so their geometries stay identical and the loads comparable.
            .pinned_particles([idx(1, 1)])
            .no_surface_collider()
            .can_sleep(false);
        let handle = world.insert_soft_body(grid);
        let sb = &mut world.soft_bodies[handle];
        for i in 0..sb.num_particles() {
            let p = sb.particle_position(i);
            sb.set_particle_position(i, p * 1.3);
        }
        world.step();
        let find = |sb: &SoftBody, a: u32, b: u32| {
            sb.edges()
                .iter()
                .position(|e| {
                    (e.vertices == [a, b] || e.vertices == [b, a])
                        && e.kind == SoftBodyEdgeKind::Structural
                })
                .unwrap()
        };
        let sb = &world.soft_bodies[handle];
        assert_eq!(
            sb.particles().iter().filter(|p| p.is_on_surface()).count(),
            4 * (n - 1),
            "the boundary ring is the surface"
        );
        let stress = |sb: &SoftBody, a, b| sb.edges()[find(sb, a, b)].stress();
        let surface = stress(sb, idx(0, 0), idx(0, 1));
        let inner = stress(sb, idx(1, 1), idx(1, 2));
        // Tear the inner edge: its particles (and the cells' around it) become damaged.
        let torn = find(sb, idx(1, 1), idx(1, 2));
        world.soft_bodies[handle].tear_edge(torn);
        // The tear applies at the end of the step; the next step's loads see the damage.
        world.step();
        world.step();
        let sb = &world.soft_bodies[handle];
        assert!(sb.particles()[idx(1, 1) as usize].is_damaged());
        assert!(sb.particles()[idx(1, 2) as usize].is_damaged());
        assert!(!sb.particles()[idx(3, 1) as usize].is_damaged());
        let next_to_damage = stress(sb, idx(1, 2), idx(1, 3));
        let far = stress(sb, idx(3, 1), idx(3, 2));
        (surface, inner, next_to_damage, far)
    };
    let plain = run(1.0);
    let tough = run(4.0);
    assert!(plain.0 > 0.05 && plain.1 > 0.02, "the stretched grid reads a load: {plain:?}");
    let close = |a: Real, b: Real| (a - b).abs() <= 1.0e-5 * a.abs().max(1.0);
    assert!(close(plain.0, tough.0), "a surface edge is not shielded");
    assert!(close(plain.1, 4.0 * tough.1), "an interior edge bears 4x the threshold");
    assert!(close(plain.2, tough.2), "an edge next to damage lost its shield");
    assert!(close(plain.3, 4.0 * tough.3), "an edge far from damage keeps its shield");
}

/// A squeezed edge past its yield takes a permanent set (only the excess strain flows), bounded
/// by the plastic maximum; the flow direction picks squeeze, stretch or both, and the tear
/// strain measures against the original length, so a flowed edge tears at the original stretch.
#[test]
fn edge_plasticity_keeps_a_dent() {
    // The tested edge joins two pinned particles, so its length is exactly what the test sets
    // and it never tears (its tear load is read through its stress). A slack edge to a free
    // particle keeps the body from being frozen; each end hangs on a pinned three-segment chain.
    let squeeze = |material: SoftBodyMaterial| {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        let builder = SoftBodyBuilder::new(vec![
            Vector::new(0.0, 0.0),
            Vector::new(1.0, 0.0),
            Vector::new(5.0, 0.0),
            Vector::new(6.0, 0.0),
            Vector::new(0.0, 1.0),
            Vector::new(1.0, 1.0),
            Vector::new(0.0, 2.0),
            Vector::new(1.0, 2.0),
            Vector::new(0.0, 3.0),
            Vector::new(1.0, 3.0),
        ])
        .edges(vec![[0, 1], [2, 3], [0, 4], [1, 5], [4, 6], [5, 7], [6, 8], [7, 9]])
        .pinned_particles([0, 1, 3, 4, 5, 6, 7, 8, 9])
        .particle_mass(1.0)
        .material(material)
        .no_surface_collider()
        .can_sleep(false);
        let handle = world.insert_soft_body(builder);
        // Squeezed to 0.7: a strain of -0.3.
        world.soft_bodies[handle].set_particle_position(1, Vector::new(0.7, 0.0));
        for _ in 0..30 {
            world.step();
        }
        (world, handle)
    };
    let clay = |flow: SoftEdgePlasticFlow, max: Real| SoftBodyMaterial {
        edge_softness: SpringCoefficients::new(30.0, 1.0),
        edge_plastic_yield: 0.1,
        edge_plastic_creep: Real::INFINITY,
        edge_plastic_max: max,
        edge_plastic_flow: flow,
        tear_strain: Some(0.25),
        ..Default::default()
    };

    // Both directions: the rest length flows to what leaves exactly the yield strain.
    let (mut world, handle) = squeeze(clay(SoftEdgePlasticFlow::Both, 1.0));
    let sb = &world.soft_bodies[handle];
    let e = &sb.edges()[0];
    assert!((e.rest_length - 0.7 / 0.9).abs() < 1.0e-4, "rest length {}", e.rest_length);
    assert!((e.initial_rest_length() - 1.0).abs() < 1.0e-4);
    assert!((e.plastic_strain() - (0.7 / 0.9 - 1.0)).abs() < 1.0e-4);
    // The rest positions followed the flow: the edge is as long at rest as its rest length.
    let rest_distance = |sb: &SoftBody| {
        (sb.particles()[1].rest_position() - sb.particles()[0].rest_position()).length()
    };
    let rest = rest_distance(sb);
    assert!(
        (rest - 0.7 / 0.9).abs() < 5.0e-3,
        "the rest positions did not follow the flow: rest distance {rest}"
    );
    let initial =
        sb.particles()[1].initial_rest_position() - sb.particles()[0].initial_rest_position();
    assert!((initial.length() - 1.0).abs() < 1.0e-6);
    // The reset undoes the set: rest length and rest positions as created.
    world.soft_bodies[handle].reset_plasticity();
    let sb = &world.soft_bodies[handle];
    assert_eq!(sb.edges()[0].rest_length, 1.0);
    assert_eq!(sb.edges()[0].plastic_strain(), 0.0);
    assert_eq!(rest_distance(sb), 1.0);

    // The plastic maximum bounds the set.
    let (world, handle) = squeeze(clay(SoftEdgePlasticFlow::Both, 0.15));
    let e = &world.soft_bodies[handle].edges()[0];
    assert!((e.rest_length - 0.85).abs() < 1.0e-4, "rest length {}", e.rest_length);

    // Tension-only clay does not dent.
    let (world, handle) = squeeze(clay(SoftEdgePlasticFlow::Tension, 1.0));
    let e = &world.soft_bodies[handle].edges()[0];
    assert!((e.rest_length - 1.0).abs() < 1.0e-6, "rest length {}", e.rest_length);

    // Compression-only clay dents, then springs back from a stretch (the rest stays where it
    // flowed to), and the tear strain is measured against the initial length: 20% past it is
    // below the tear strain even though it is 54% past the flowed rest length.
    let (mut world, handle) = squeeze(clay(SoftEdgePlasticFlow::Compression, 1.0));
    world.soft_bodies[handle].set_particle_position(1, Vector::new(1.2, 0.0));
    for _ in 0..30 {
        world.step();
    }
    let sb = &world.soft_bodies[handle];
    let e = &sb.edges()[0];
    assert!((e.rest_length - 0.7 / 0.9).abs() < 1.0e-4, "a stretch flowed: {}", e.rest_length);
    let stress = e.stress();
    assert!(
        (stress - 0.2 / 0.25).abs() < 0.02,
        "the tear load reads the flowed rest length instead of the initial one: {stress}"
    );
    // 30% past the initial length: past the tear strain (the edge would tear, were an end free).
    world.soft_bodies[handle].set_particle_position(1, Vector::new(1.3, 0.0));
    for _ in 0..30 {
        world.step();
    }
    let sb = &world.soft_bodies[handle];
    sb.validate_topology().unwrap();
    assert_eq!(sb.num_particles(), 10, "an edge between two pinned particles tore");
    let stress = sb.edges()[0].stress();
    assert!(
        (stress - 0.3 / 0.25).abs() < 0.02,
        "the overstretched edge does not read past its threshold: {stress}"
    );
}

/// Tears are reported to the event handler and keep every segment: a seven-particle rope whose
/// middle edge snaps reports the torn edge, the split particle and two three-segment pieces.
/// A three-segment piece cannot tear again, and an overloaded bottom edge never sheds a chip.
#[test]
fn tear_events_and_rope_ends() {
    let rope = |num_particles: usize| {
        let positions = (0..num_particles)
            .map(|i| Vector::new(0.0, 3.0 - i as Real))
            .collect();
        let edges = (0..num_particles as u32 - 1).map(|i| [i, i + 1]).collect();
        SoftBodyBuilder::new(positions)
            .edges(edges)
            .pinned_particles([0])
            .particle_mass(1.0)
            .softness(SpringCoefficients::new(120.0, 1.0))
            .no_surface_collider()
            .can_sleep(false)
    };

    // Seven particles: the middle edge bears three masses, past the tear force; the edges above
    // it (four to six masses) are reinforced.
    let mut world = PhysicsWorld::new();
    let handle = world.insert_soft_body(
        rope(7)
            .edge_tear_resistance([(0, 4.0), (1, 4.0), (2, 4.0)])
            .tear_force(24.0),
    );
    let log = TearLog::default();
    for _ in 0..60 {
        world.step_with_events(&(), &log);
    }
    {
        let events = log.0.lock().unwrap();
        assert_eq!(events.len(), 1, "one tear expected, got {events:?}");
        let event = &events[0];
        assert_eq!(event.soft_body, handle);
        assert_eq!(event.torn_edges, vec![[3, 4]]);
        assert!(event.torn_cells.is_empty());
        // Both ends of the torn edge can open: the lower index goes first.
        assert_eq!(event.split_particles, vec![(7, 3)]);
        // Two pieces of equal length: the pinned one (the smaller particle) keeps the handle,
        // the falling one is a new body.
        assert_eq!(event.pieces.len(), 2);
        assert_eq!(event.pieces[0].soft_body, handle);
        assert_eq!(event.pieces[0].particles, vec![0, 1, 2, 3]);
        assert_eq!(event.pieces[1].particles, vec![4, 5, 6, 7]);
        let fallen = &world.soft_bodies[event.pieces[1].soft_body];
        fallen.validate_topology().unwrap();
        assert_eq!(fallen.origin(), Some(handle));
        assert_eq!((fallen.num_particles(), fallen.edges().len()), (4, 3));
        assert_eq!(world.soft_bodies[handle].pieces(), &[event.pieces[1].soft_body]);
    }
    let sb = &world.soft_bodies[handle];
    sb.validate_topology().unwrap();
    assert_eq!(sb.edges().len(), 3);
    assert_eq!(sb.connected_pieces(), vec![vec![0, 1, 2, 3]]);
    // The pinned piece is three segments: any split would leave a side below the minimum.
    let pinned: Vec<u32> = (0..sb.edges().len() as u32).collect();
    assert!(
        world.tear_soft_body(handle, &pinned, &[])
            .is_none(),
        "a piece of three segments tore"
    );
    assert_eq!(world.soft_bodies[handle].num_particles(), 4);

    // The bottom edge is past the tear force. Its end particle cannot open (it has no other
    // segment) and the particle above it would shed a single segment: nothing tears.
    let mut world = PhysicsWorld::new();
    let handle = world.insert_soft_body(
        rope(7)
            .edge_tear_resistance([(0, 100.0), (1, 100.0), (2, 100.0), (3, 100.0), (4, 100.0)])
            .tear_force(6.0),
    );
    let log = TearLog::default();
    for _ in 0..60 {
        world.step_with_events(&(), &log);
    }
    assert!(log.0.lock().unwrap().is_empty(), "the rope shed a chip");
    let sb = &world.soft_bodies[handle];
    sb.validate_topology().unwrap();
    assert_eq!(sb.num_particles(), 7);
    assert_eq!(sb.connected_pieces().len(), 1);
}

/// A builder rope (`SoftBodyBuilder::rope`) torn in its middle separates: the bending edge over
/// the split particle is removed (it would hold the two pieces together), the other bending
/// edges and the surface segments follow their piece, and the loose end falls away.
#[test]
fn torn_builder_rope_separates() {
    let mut world = PhysicsWorld::new();
    let handle = world.insert_soft_body(
        SoftBodyBuilder::rope(Vector::new(0.0, 3.0), Vector::new(4.0, 3.0), 9)
            .pinned_particles([0])
            .can_sleep(false),
    );
    let sb = &world.soft_bodies[handle];
    let before = sb.clone();
    let num_bends = |sb: &SoftBody| {
        sb.edges()
            .iter()
            .filter(|e| e.kind == SoftBodyEdgeKind::Bend)
            .count()
    };
    let torn = sb
        .edges()
        .iter()
        .position(|e| e.vertices == [4, 5] && e.kind == SoftBodyEdgeKind::Structural)
        .unwrap() as u32;
    let event = world.tear_soft_body(handle, &[torn], &[])
        .expect("the rope tore");
    // Particle 4 splits (the lower index), and the bending edge (3, 5) over it goes.
    assert_eq!(event.split_particles, vec![(9, 4)]);
    assert_eq!(event.torn_edges, vec![[4, 5]]);
    assert_eq!(event.removed_edges, vec![[3, 5]]);
    // Two pieces of four segments: the pinned one keeps the handle, the loose one is a new body.
    assert_eq!(event.pieces.len(), 2);
    assert_eq!(event.pieces[0].soft_body, handle);
    assert_eq!(event.pieces[0].particles, vec![0, 1, 2, 3, 4]);
    assert_eq!(event.pieces[1].particles, vec![5, 6, 7, 8, 9]);
    let loose = event.pieces[1].soft_body;

    let (sb, lb) = (&world.soft_bodies[handle], &world.soft_bodies[loose]);
    sb.validate_topology().unwrap();
    lb.validate_topology().unwrap();
    let mass = sb.mass() + lb.mass();
    let measure = sb.rest_measure() + lb.rest_measure();
    assert!((mass - before.mass()).abs() <= 1.0e-5 * before.mass());
    assert!((measure - before.rest_measure()).abs() <= 1.0e-5 * before.rest_measure());
    assert_eq!(sb.edges().len() + lb.edges().len(), before.edges().len() - 1);
    assert_eq!(num_bends(sb) + num_bends(lb), num_bends(&before) - 1);
    assert_eq!(sb.boundary().len() + lb.boundary().len(), before.boundary().len());
    assert_eq!(sb.connected_pieces().len(), 1);
    assert_eq!(lb.connected_pieces().len(), 1);

    // The pinned piece hangs at most 2 m below its pin; the loose one falls freely.
    for _ in 0..60 {
        world.step();
    }
    let (sb, lb) = (&world.soft_bodies[handle], &world.soft_bodies[loose]);
    let gap = (lb.particle_position(4) - sb.particle_position(4)).length();
    assert!(gap > 2.0, "the torn ends stayed together: {gap} m apart");
}

/// A tear keeps every particle in a measure element, whatever it takes around a particle: every
/// edge at a grid's corner, both ring segments of a disk particle, and every edge of a rope.
#[test]
fn tears_keep_every_particle_in_an_element() {
    let mut world = world_with_ground();
    let grid = world.insert_soft_body(
        SoftBodyBuilder::grid(Vector::new(0.0, 2.0), Vector::splat(1.0), 5, 5).particle_mass(0.2),
    );
    let disk = world.insert_soft_body(
        SoftBodyBuilder::disk(Vector::new(4.0, 2.0), 1.6, 40).particle_mass(0.05),
    );
    let rope = world.insert_soft_body(
        SoftBodyBuilder::rope(Vector::new(-4.0, 3.0), Vector::new(-2.0, 3.0), 6)
            .pinned_particles([0]),
    );
    world.step();

    // The edges at the particle closest to `point`, optionally structural ones only.
    let edges_at = |sb: &SoftBody, point: Vector, structural: bool| -> Vec<u32> {
        let v = (0..sb.num_particles())
            .min_by(|&i, &j| {
                (sb.particle_position(i) - point)
                    .length()
                    .total_cmp(&(sb.particle_position(j) - point).length())
            })
            .unwrap() as u32;
        (0..sb.edges().len() as u32)
            .filter(|&i| {
                let e = &sb.edges()[i as usize];
                e.vertices.contains(&v) && (!structural || e.kind == SoftBodyEdgeKind::Structural)
            })
            .collect()
    };
    let corner = edges_at(&world.soft_bodies[grid], Vector::new(-1.0, 1.0), false);
    world.tear_soft_body(grid, &corner, &[]);
    let ring = edges_at(&world.soft_bodies[disk], Vector::new(5.6, 2.0), true);
    assert_eq!(ring.len(), 2);
    world.tear_soft_body(disk, &ring, &[])
        .expect("the ring tore");
    let all_edges: Vec<u32> = (0..world.soft_bodies[rope].edges().len() as u32).collect();
    world.tear_soft_body(rope, &all_edges, &[]);

    for _ in 0..60 {
        world.step();
    }
    for handle in [grid, disk, rope].into_iter().flat_map(|h| family(&world, h)) {
        let sb = &world.soft_bodies[handle];
        sb.validate_topology().unwrap();
        assert!(sb.particles().iter().all(|p| p.position().is_finite()));
    }
}

/// A cut keeps every particle in a measure element: a blade cutting a grid's corner cells off, and
/// a blade crossing a ring on both sides of one of its particles.
#[test]
fn cuts_keep_every_particle_in_an_element() {
    let mut world = world_with_ground();
    let grid = world.insert_soft_body(
        SoftBodyBuilder::grid(Vector::new(0.0, 2.0), Vector::splat(1.0), 5, 5).particle_mass(0.2),
    );
    let disk = world.insert_soft_body(
        SoftBodyBuilder::disk(Vector::new(4.0, 2.0), 1.6, 40).particle_mass(0.05),
    );
    world.step();
    // The line x + y = 0.7 separates the two corner cells (centroids at x + y = 0.5) from the
    // others; the corner particles' neighbors (x + y = 1) lie across it.
    let event = world
        .cut_soft_body(grid, &[Vector::new(-1.3, 2.0), Vector::new(-0.2, 0.9)])
        .expect("the blade crossed the corner");
    assert_eq!(event.pieces.len(), 2);
    // The blade x = 5.59 crosses both ring segments of the particle (5.6, 2).
    let event = world
        .cut_soft_body(disk, &[Vector::new(5.59, 1.0), Vector::new(5.59, 3.0)])
        .expect("the blade crossed the ring");
    assert_eq!(event.inserted_particles.len(), 4);
    for _ in 0..60 {
        world.step();
    }
    for handle in [grid, disk].into_iter().flat_map(|h| family(&world, h)) {
        let sb = &world.soft_bodies[handle];
        sb.validate_topology().unwrap();
        assert!(sb.particles().iter().all(|p| p.position().is_finite()));
    }
}

/// A blade across a grid between two of its columns splits the particles of the edges it meets,
/// each cell going whole to the side of its centroid, leaving two pieces and removing no cell.
/// A blade crossing nothing, and the same blade again, are no-ops.
#[test]
fn cut_splits_a_grid_along_a_segment() {
    let mut world = world_with_ground();
    let n = 5usize;
    let grid = SoftBodyBuilder::grid(Vector::new(0.0, 2.0), Vector::splat(1.0), n, n)
        .particle_mass(0.2);
    let handle = world.insert_soft_body(grid);
    world.step();
    let blade = [Vector::new(0.25, -5.0), Vector::new(0.25, 10.0)];
    let sb = &world.soft_bodies[handle];
    let (edges, cells) = sb.crossing_elements(&blade);
    assert!(!edges.is_empty() && !cells.is_empty());
    let (num_cells, mass, measure) = (sb.cells().len(), sb.mass(), sb.rest_measure());
    assert!(
        world
            .cut_soft_body(handle, &[Vector::new(5.0, -5.0), Vector::new(5.0, 10.0)])
            .is_none(),
        "a blade beside the body cut something"
    );
    let event = world
        .cut_soft_body(handle, &blade)
        .expect("the blade crossed the grid");
    assert_eq!(event.pieces.len(), 2);
    assert_eq!(event.torn_cells.len(), cells.len());
    assert!(!event.split_particles.is_empty() && event.inserted_particles.is_empty());
    // The two pieces are bodies of their own; every cell of a piece has its centroid on one
    // side of the blade.
    let bodies: Vec<SoftBodyHandle> = event.bodies().collect();
    let mut sides = Vec::new();
    let (mut cells_after, mut mass_after, mut measure_after) = (0, 0.0, 0.0);
    for &h in &bodies {
        let sb = &world.soft_bodies[h];
        sb.validate_topology().unwrap();
        assert_eq!(sb.connected_pieces().len(), 1);
        cells_after += sb.cells().len();
        mass_after += sb.mass();
        measure_after += sb.rest_measure();
        let mut side = None;
        for c in sb.cells() {
            let centroid: Vector = c
                .vertices
                .iter()
                .map(|&v| sb.particle_position(v as usize))
                .sum::<Vector>()
                / 3.0;
            assert_eq!(*side.get_or_insert(centroid.x < 0.25), centroid.x < 0.25);
        }
        sides.push(side.unwrap());
    }
    assert_eq!(cells_after, num_cells);
    assert!((mass_after - mass).abs() < 1.0e-5 && (measure_after - measure).abs() < 1.0e-5);
    assert_ne!(sides[0], sides[1]);
    for &h in &bodies {
        assert!(
            world.cut_soft_body(h, &blade).is_none(),
            "the same cut twice changed something"
        );
    }
    for _ in 0..60 {
        world.step();
        for &h in &bodies {
            assert_finite(&world, h);
        }
    }
    for &h in &bodies {
        world.soft_bodies[h].validate_topology().unwrap();
    }
}

/// A cut across a rope segment inserts two particles at the crossing: rest lengths split in
/// proportion, the segment ends share their masses with the inserted particles, the bending
/// edges over it are removed and the rope comes apart. A blade left across the cut is a no-op.
#[test]
fn cut_inserts_particles_into_a_rope() {
    let mut world = world_with_ground();
    // Particles every 0.25 from x = 0 to x = 2; the blade crosses the segment (3, 4) at t = 0.6.
    let rope = SoftBodyBuilder::rope(Vector::new(0.0, 3.0), Vector::new(2.0, 3.0), 9)
        .pinned_particles([0])
        .particle_mass(0.1);
    let handle = world.insert_soft_body(rope);
    let blade = [Vector::new(0.9, 2.0), Vector::new(0.9, 4.0)];
    let event = world
        .cut_soft_body(handle, &blade)
        .expect("the blade crossed the rope");
    assert_eq!(event.inserted_particles, vec![9, 10]);
    assert!(event.split_particles.is_empty());
    assert_eq!(event.torn_edges, vec![[3, 4], [2, 4], [3, 5]]);
    let mut removed = event.removed_edges.clone();
    removed.sort_unstable();
    assert_eq!(removed, vec![[2, 4], [3, 5]]);
    // The longer, free half keeps the handle; the pinned half is a new body. The event maps
    // every particle to its body and index there.
    assert_eq!(event.pieces.len(), 2);
    assert_eq!(event.pieces[0].particles, vec![4, 5, 6, 7, 8, 10]);
    assert_eq!(event.pieces[1].particles, vec![0, 1, 2, 3, 9]);
    let at = |i: u32| event.particle_destination(i).unwrap();
    let (free, pinned) = (&world.soft_bodies[handle], &world.soft_bodies[at(0).0]);
    assert_eq!((at(3), at(9)), ((at(0).0, 3), (at(0).0, 4)));
    assert_eq!((at(10), at(4)), ((handle, 5), (handle, 0)));
    free.validate_topology().unwrap();
    pinned.validate_topology().unwrap();
    let close = |a: Real, b: Real| (a - b).abs() < 1.0e-5;
    let length = |sb: &SoftBody, a: u32, b: u32| {
        sb.edges()
            .iter()
            .find(|e| e.vertices == [a, b])
            .map(|e| e.rest_length)
    };
    assert!(close(length(pinned, 3, 4).unwrap(), 0.15) && close(length(free, 5, 0).unwrap(), 0.1));
    assert!(close(free.rest_measure() + pinned.rest_measure(), 2.0));
    assert!(close(free.mass() + pinned.mass(), 0.9));
    // Each end gives up half its mass (its share of the segment), then the four particles share
    // that mass by the halves' lumped masses: 0.3 and 0.2 of it at each end of the halves.
    let masses = [
        pinned.particles()[3].mass(),
        pinned.particles()[4].mass(),
        free.particles()[5].mass(),
        free.particles()[0].mass(),
    ];
    for (mass, expected) in masses.iter().zip([0.08, 0.03, 0.02, 0.07]) {
        assert!(close(*mass, expected), "masses {masses:?}");
    }
    assert!(pinned.particles()[0].is_pinned() && !pinned.particles()[4].is_pinned());
    for (sb, i) in [(pinned, 4), (free, 5)] {
        assert!((sb.particle_position(i) - Vector::new(0.9, 3.0)).length() < 1.0e-5);
    }
    assert!(pinned.boundary().contains(&[3, 4]) && free.boundary().contains(&[5, 0]));

    let bodies: Vec<SoftBodyHandle> = event.bodies().collect();
    for _ in 0..60 {
        world.step();
        for &h in &bodies {
            let num_particles = world.soft_bodies[h].num_particles();
            assert!(world.cut_soft_body(h, &blade).is_none());
            assert_eq!(world.soft_bodies[h].num_particles(), num_particles);
            assert_finite(&world, h);
        }
    }
    for &h in &bodies {
        world.soft_bodies[h].validate_topology().unwrap();
    }
}

/// Point and radial impulses with a linear falloff: the particle on the point takes the whole
/// impulse over its mass, one halfway to the radius half of it, one at the radius or beyond
/// nothing, and a pinned particle nothing at all; the radial variant pushes away from the center.
#[test]
fn impulses_fall_off_with_the_distance() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let builder = SoftBodyBuilder::new(vec![
        Vector::new(0.0, 0.0),
        Vector::new(1.0, 0.0),
        Vector::new(2.0, 0.0),
        Vector::new(0.0, 1.0),
        Vector::new(-1.0, 0.0),
    ])
    .edges(vec![[0, 1], [1, 2], [0, 3], [0, 4]])
    .pinned_particles([3])
    .masses(vec![1.0, 2.0, 1.0, 1.0, 1.0])
    .no_surface_collider();
    let handle = world.insert_soft_body(builder);
    let sb = &mut world.soft_bodies[handle];
    sb.apply_impulse_at_point(Vector::new(0.0, 4.0), Vector::ZERO, 2.0, true);
    let v = |i: usize| sb.particle_velocity(i);
    assert!((v(0) - Vector::new(0.0, 4.0)).length() < 1.0e-6, "on the point: {:?}", v(0));
    assert!((v(1) - Vector::new(0.0, 1.0)).length() < 1.0e-6, "halfway, twice the mass: {:?}", v(1));
    assert_eq!(v(2), Vector::ZERO, "at the radius");
    assert_eq!(v(3), Vector::ZERO, "pinned");
    assert!((v(4) - Vector::new(0.0, 2.0)).length() < 1.0e-6, "halfway: {:?}", v(4));

    for i in 0..sb.num_particles() {
        sb.set_particle_velocity(i, Vector::ZERO);
    }
    sb.apply_radial_impulse(Vector::ZERO, 4.0, 2.0, true);
    let v = |i: usize| sb.particle_velocity(i);
    assert_eq!(v(0), Vector::ZERO, "on the center: no direction");
    assert!((v(1) - Vector::new(1.0, 0.0)).length() < 1.0e-6, "pushed right: {:?}", v(1));
    assert_eq!(v(2), Vector::ZERO, "at the radius");
    assert_eq!(v(3), Vector::ZERO, "pinned");
    assert!((v(4) - Vector::new(-2.0, 0.0)).length() < 1.0e-6, "pushed left: {:?}", v(4));

    // Without a falloff radius every free particle gets the whole impulse.
    for i in 0..sb.num_particles() {
        sb.set_particle_velocity(i, Vector::ZERO);
    }
    sb.apply_impulse_at_point(Vector::new(0.0, 4.0), Vector::ZERO, 0.0, true);
    assert!((sb.particle_velocity(2) - Vector::new(0.0, 4.0)).length() < 1.0e-6);
}

/// The contact-modification hook sees the contacts between two soft surfaces (their
/// candidates) and the soft-body solver follows its edits: a hook disabling every candidate
/// between the two lets a blob fall through a pinned rope it otherwise rests on.
#[test]
fn modify_solver_contacts_edits_soft_soft_contacts() {
    struct DropSoftSoft;
    impl PhysicsHooks for DropSoftSoft {
        fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
            let soft = |h: ColliderHandle| context.colliders[h].user_data == 7;
            if soft(context.collider1) && soft(context.collider2) {
                // Two soft surfaces: their contacts are candidates, not solver contacts.
                if let Some(candidates) = context.soft_mut() {
                    candidates.disable_all();
                }
            }
        }
    }
    let scene = |hooked: bool| {
        let mut world = world_with_ground();
        let n = 11;
        let template = ColliderBuilder::ball(0.04)
            .user_data(7)
            .active_hooks(if hooked {
                ActiveHooks::MODIFY_SOLVER_CONTACTS
            } else {
                ActiveHooks::empty()
            });
        world.insert_soft_body(
            SoftBodyBuilder::rope(Vector::new(-0.6, 1.0), Vector::new(0.6, 1.0), n)
                .pinned_particles(0..n as u32)
                .particle_mass(0.05)
                .particle_radius(0.04)
                .surface_collider(template.clone()),
        );
        let blob = world.insert_soft_body(
            SoftBodyBuilder::disk(Vector::new(0.0, 1.6), 0.35, 20)
                .softness(SpringCoefficients::new(20.0, 1.0))
                .volume_factor(1.1)
                .particle_mass(0.05)
                .particle_radius(0.06)
                .surface_collider(template),
        );
        for _ in 0..240 {
            world.step_with_events(&DropSoftSoft, &());
        }
        assert_finite(&world, blob);
        let sb = &world.soft_bodies[blob];
        (0..sb.num_particles())
            .map(|i| sb.particle_position(i).y)
            .sum::<Real>()
            / sb.num_particles() as Real
    };
    let resting = scene(false);
    let dropped = scene(true);
    assert!(resting > 1.1, "the blob did not rest on the rope: y = {resting}");
    assert!(dropped < 0.6, "the hook did not drop the soft-soft contacts: y = {dropped}");
}

/// A rope draped over a rigid ball much larger than its segments rests on it: the ball's
/// manifold with each segment holds a single point, so the vertices are held by the pair's
/// predictive vertex contacts (without them they sag into the skin and the rope jitters).
#[test]
fn rope_draped_over_a_large_ball_supports_its_vertices() {
    let mut world = world_with_ground();
    let center = Vector::new(0.0, 1.0);
    world.insert(
        RigidBodyBuilder::fixed().translation(center),
        ColliderBuilder::ball(1.0),
    );
    let skin = 0.1 / 3.0;
    let rope = SoftBodyBuilder::rope(Vector::new(-1.0, 2.1), Vector::new(1.0, 2.1), 21)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.02)
        .particle_radius(skin)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.8));
    let handle = world.insert_soft_body(rope);
    let mut min_dist = Real::MAX;
    let mut max_speed: Real = 0.0;
    let mut sum_speed = 0.0;
    let mut count = 0;
    for step in 0..600 {
        world.step();
        if step < 300 {
            continue;
        }
        for p in world.soft_bodies[handle].particles() {
            let dist = (p.position() - center).length();
            if dist < 1.2 {
                min_dist = min_dist.min(dist);
                max_speed = max_speed.max(p.velocity().length());
                sum_speed += p.velocity().length();
                count += 1;
            }
        }
    }
    assert!(count > 0, "the rope slid off the ball");
    let sag = 1.0 + skin - min_dist;
    let mean_speed = sum_speed / count as Real;
    assert!(sag < 0.05 * skin, "the vertices sag into the ball: {sag}");
    assert!(
        mean_speed < 0.045 && max_speed < 0.3,
        "the rope jitters on the ball: mean speed {mean_speed}, max {max_speed}"
    );
}

/// A tear or a cut never deletes material: the mass and the rest measure (cell areas, or
/// structural edge lengths) are conserved, every particle keeps a measure element and the tables
/// stay consistent. One test per body kind and tear source.
mod tear_and_cut_conserve_mass_and_measure {
    use super::*;

    /// Asserts the tear invariants from the body before a tear or cut to the bodies after it.
    fn assert_tear_invariants(before: &SoftBody, after: &[&SoftBody]) {
        let close = |a: Real, b: Real| (a - b).abs() <= 1.0e-5 * a.abs().max(b.abs());
        let mass: Real = after.iter().map(|sb| sb.mass()).sum();
        let measure: Real = after.iter().map(|sb| sb.rest_measure()).sum();
        assert!(
            close(before.mass(), mass),
            "mass changed: {} before, {} after",
            before.mass(),
            mass
        );
        assert!(
            close(before.rest_measure(), measure),
            "rest measure changed: {} before, {} after",
            before.rest_measure(),
            measure
        );
        for after in after {
            assert_piece_invariants(before, after);
        }
    }

    /// Asserts that every particle of a piece keeps a measure element and its tables hold.
    fn assert_piece_invariants(before: &SoftBody, after: &SoftBody) {
        // The measure elements are chosen by the body kind before the tear.
        let elements: Vec<Vec<u32>> = if !before.cells().is_empty() {
            after.cells().iter().map(|c| c.vertices.to_vec()).collect()
        } else {
            after
                .edges()
                .iter()
                .filter(|e| e.kind == SoftBodyEdgeKind::Structural)
                .map(|e| e.vertices.to_vec())
                .collect()
        };
        let mut supported = vec![false; after.num_particles()];
        for element in &elements {
            for &v in element {
                supported[v as usize] = true;
            }
        }
        if let Some(v) = supported.iter().position(|s| !s) {
            panic!("particle {v} belongs to no measure element");
        }
        after.validate_topology().unwrap();
    }

    /// Inserts the body, tears it with `tear`, checks the invariants, then steps the torn body.
    fn check(
        builder: SoftBodyBuilder,
        tear: impl FnOnce(&mut PhysicsWorld, SoftBodyHandle) -> Option<SoftBodyTearEvent>,
    ) {
        let mut world = PhysicsWorld::new();
        let handle = world.insert_soft_body(builder);
        let before = world.soft_bodies[handle].clone();
        let event = tear(&mut world, handle).expect("nothing tore");
        let bodies: Vec<SoftBodyHandle> = event.bodies().collect();
        let after: Vec<&SoftBody> = bodies.iter().map(|&h| &world.soft_bodies[h]).collect();
        assert_tear_invariants(&before, &after);
        for _ in 0..10 {
            world.step();
            for &h in &bodies {
                assert_finite(&world, h);
            }
        }
        for &h in &bodies {
            world.soft_bodies[h].validate_topology().unwrap();
        }
    }

    /// Tears the structural edge whose midpoint is closest to `point`.
    fn tear_edge_near(
        world: &mut PhysicsWorld,
        handle: SoftBodyHandle,
        point: Vector,
    ) -> Option<SoftBodyTearEvent> {
        let sb = &world.soft_bodies[handle];
        let distance = |e: &SoftBodyEdge| {
            let mid = (sb.particle_position(e.vertices[0] as usize)
                + sb.particle_position(e.vertices[1] as usize))
                * 0.5;
            (mid - point).length()
        };
        let edge = (0..sb.edges().len())
            .filter(|&i| sb.edges()[i].kind == SoftBodyEdgeKind::Structural)
            .min_by(|&i, &j| distance(&sb.edges()[i]).total_cmp(&distance(&sb.edges()[j])))
            .unwrap() as u32;
        world.tear_soft_body(handle, &[edge], &[])
    }

    /// Tears the cell whose centroid is closest to `point`.
    fn tear_cell_near(
        world: &mut PhysicsWorld,
        handle: SoftBodyHandle,
        point: Vector,
    ) -> Option<SoftBodyTearEvent> {
        let sb = &world.soft_bodies[handle];
        let distance = |c: &SoftBodyCell| {
            let sum: Vector = c
                .vertices
                .iter()
                .map(|&v| sb.particle_position(v as usize))
                .sum();
            (sum / c.vertices.len() as Real - point).length()
        };
        let cell = (0..sb.cells().len())
            .min_by(|&i, &j| distance(&sb.cells()[i]).total_cmp(&distance(&sb.cells()[j])))
            .unwrap() as u32;
        world.tear_soft_body(handle, &[], &[cell])
    }

    fn grid() -> SoftBodyBuilder {
        SoftBodyBuilder::grid(Vector::ZERO, Vector::splat(1.0), 5, 5).particle_mass(0.2)
    }

    fn disk() -> SoftBodyBuilder {
        SoftBodyBuilder::disk(Vector::ZERO, 1.0, 24).particle_mass(0.05)
    }

    fn strip() -> SoftBodyBuilder {
        let vertices = (0..8).map(|i| Vector::new(i as Real * 0.25, 0.0)).collect();
        SoftBodyBuilder::polyline(vertices, None).unwrap()
    }

    fn rope() -> SoftBodyBuilder {
        SoftBodyBuilder::rope(Vector::ZERO, Vector::new(2.0, 0.0), 9)
    }

    /// A 3 x 3 particle spring network (no cell).
    fn spring_network() -> SoftBodyBuilder {
        let idx = |i: u32, j: u32| i * 3 + j;
        let mut vertices = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                vertices.push(Vector::new(i as Real * 0.5, j as Real * 0.5));
            }
        }
        let mut indices = Vec::new();
        for i in 0..2 {
            for j in 0..2 {
                indices.push([idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)]);
                indices.push([idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)]);
            }
        }
        SoftBodyBuilder::trimesh(vertices, indices).unwrap()
    }

    fn cut(
        blade: [Vector; 2],
    ) -> impl FnOnce(&mut PhysicsWorld, SoftBodyHandle) -> Option<SoftBodyTearEvent> {
        move |world, handle| world.cut_soft_body(handle, &blade)
    }

    #[test]
    fn grid_edge_tear() {
        check(grid(), |w, h| tear_edge_near(w, h, Vector::new(0.25, 0.0)));
    }

    #[test]
    fn grid_cell_tear() {
        check(grid(), |w, h| tear_cell_near(w, h, Vector::new(0.1, 0.1)));
    }

    #[test]
    fn grid_cut() {
        check(grid(), cut([Vector::new(0.25, -5.0), Vector::new(0.25, 5.0)]));
    }

    #[test]
    fn disk_edge_tear() {
        check(disk(), |w, h| tear_edge_near(w, h, Vector::new(1.0, 0.1)));
    }

    #[test]
    fn disk_cut() {
        check(disk(), cut([Vector::new(0.5, -5.0), Vector::new(0.5, 5.0)]));
    }

    #[test]
    fn polyline_edge_tear() {
        check(strip(), |w, h| tear_edge_near(w, h, Vector::new(0.875, 0.0)));
    }

    #[test]
    fn polyline_cut() {
        check(strip(), cut([Vector::new(0.8, -1.0), Vector::new(0.8, 1.0)]));
    }

    #[test]
    fn rope_edge_tear() {
        check(rope(), |w, h| tear_edge_near(w, h, Vector::new(0.875, 0.0)));
    }

    #[test]
    fn rope_cut() {
        check(rope(), cut([Vector::new(0.9, -1.0), Vector::new(0.9, 1.0)]));
    }

    #[test]
    fn spring_network_edge_tear() {
        check(spring_network(), |w, h| {
            tear_edge_near(w, h, Vector::new(0.75, 0.5))
        });
    }
}

/// Volume preservation holds every piece of material enclosed by its own closed boundary on its
/// own target: of two disks appended into one body, squeezing one does not inflate the other (a
/// single constraint over both boundaries let the free disk take the area the squeezed one lost).
#[test]
fn appended_disks_keep_their_own_areas() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let n = 24;
    let center = Vector::new(-1.5, 0.0);
    let disks = SoftBodyBuilder::disk(center, 1.0, n)
        .append(SoftBodyBuilder::disk(Vector::new(1.5, 0.0), 1.0, n))
        .pinned_particles(0..n as u32)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .can_sleep(false);
    let handle = world.insert_soft_body(disks);
    let sb = &world.soft_bodies[handle];
    assert!(sb.volume_preservation_enabled());
    let pieces = sb.volume_pieces();
    assert_eq!(pieces.len(), 2);
    assert_eq!(pieces[0].particles(), (0..n as u32).collect::<Vec<_>>());
    let rest = pieces[1].rest_volume();
    assert!(rest > 3.0, "disk area too small: {rest}");
    assert!((pieces[0].rest_volume() - rest).abs() < 1.0e-4 * rest);
    assert!((sb.rest_volume() - 2.0 * rest).abs() < 1.0e-4 * rest);

    // The pinned disk is squeezed to half its area.
    let sb = &mut world.soft_bodies[handle];
    let scale = Real::sqrt(0.5);
    for i in 0..n {
        let p = sb.particle_position(i);
        sb.set_particle_position(i, center + (p - center) * scale);
    }
    for _ in 0..300 {
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    let squeezed = sb.volume_pieces()[0].volume(sb);
    let free = sb.volume_pieces()[1].volume(sb);
    assert!(
        (squeezed - 0.5 * rest).abs() < 0.01 * rest,
        "the pinned disk moved: {squeezed} vs {}",
        0.5 * rest
    );
    assert!(
        (free - rest).abs() < 0.02 * rest,
        "the free disk traded area with the squeezed one: {free} vs {rest}"
    );
    assert!((sb.volume() - squeezed - free).abs() < 1.0e-4 * rest);

    // Disks sewn together by an edge are one piece of material.
    let sewn = SoftBodyBuilder::disk(center, 1.0, n)
        .append(SoftBodyBuilder::disk(Vector::new(1.5, 0.0), 1.0, n))
        .add_edges(vec![[0, n as u32]]);
    let handle = world.insert_soft_body(sewn);
    assert_eq!(world.soft_bodies[handle].volume_pieces().len(), 1);
}

/// Whether two segments cross at a point interior to both (touching or collinear segments do
/// not cross).
fn segments_cross(a: [Vector; 2], b: [Vector; 2]) -> bool {
    let orient = |p: Vector, q: Vector, r: Vector| (q - p).perp_dot(r - p);
    let (o1, o2) = (orient(a[0], a[1], b[0]), orient(a[0], a[1], b[1]));
    let (o3, o4) = (orient(b[0], b[1], a[0]), orient(b[0], b[1], a[1]));
    o1 * o2 < 0.0 && o3 * o4 < 0.0
}

/// The number of inverted cells and of crossing pairs of boundary segments of a body.
fn tangles(sb: &SoftBody) -> (usize, usize) {
    let area = |x: [Vector; 3]| (x[1] - x[0]).perp_dot(x[2] - x[0]);
    let inverted = sb
        .cells()
        .iter()
        .filter(|c| {
            let now = area(c.vertices.map(|v| sb.particle_position(v as usize)));
            let rest = area(c.vertices.map(|v| sb.particles()[v as usize].rest_position()));
            now * rest <= 0.0
        })
        .count();
    let boundary = sb.boundary();
    let mut crossings = 0;
    for (i, a) in boundary.iter().enumerate() {
        for b in &boundary[i + 1..] {
            if a.iter().any(|v| b.contains(v)) {
                continue;
            }
            let pos = |s: &[u32; 2]| s.map(|v| sb.particle_position(v as usize));
            if segments_cross(pos(a), pos(b)) {
                crossings += 1;
            }
        }
    }
    (inverted, crossings)
}

/// The faces of a crack collide: a FEM grid with self contacts, cut halfway through, opened and
/// squeezed shut across the cut, keeps its crack faces from passing through each other. The faces
/// rest at distance zero, so `SoftCollisionMesh::self_contact_excluded` must keep their contacts.
#[cfg(feature = "fem")]
#[test]
fn crack_faces_collide_when_squeezed_shut() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let grid = SoftBodyBuilder::grid(Vector::ZERO, Vector::splat(1.0), 9, 9)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e3,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.05)
        .self_contacts(true)
        .solver(SoftBodySolver::Fem)
        .can_sleep(false);
    let handle = world.insert_soft_body(grid);
    world.step();

    // A cut from the bottom edge to the middle, along the particle column at x = 0.
    let blade = [Vector::new(0.05, -2.0), Vector::new(0.05, 0.0)];
    let event = world
        .cut_soft_body(handle, &blade)
        .expect("the blade crossed the grid");
    assert!(event.pieces.is_empty(), "a partial cut separated the grid");
    assert!(!event.split_particles.is_empty());
    let sb = &world.soft_bodies[handle];
    sb.validate_topology().unwrap();
    assert_eq!(tangles(sb), (0, 0));

    // The flaps under the crack tip, told apart by the side of their cells (the crack faces share
    // their rest positions): pulled apart to open the crack, then pushed toward each other.
    let mut side = vec![0.0; sb.num_particles()];
    for c in sb.cells() {
        let x: Real = c
            .vertices
            .iter()
            .map(|&v| sb.particles()[v as usize].rest_position().x)
            .sum();
        for &v in &c.vertices {
            side[v as usize] += x;
        }
    }
    let left: Vec<bool> = side.iter().map(|x| *x < 0.0).collect();
    let push = |world: &mut PhysicsWorld, magnitude: Real| {
        let sb = &mut world.soft_bodies[handle];
        sb.reset_forces(true);
        for i in 0..sb.num_particles() {
            if sb.particles()[i].rest_position().y < -0.2 {
                let dir = if left[i] { 1.0 } else { -1.0 };
                sb.add_particle_force(i, Vector::new(dir * magnitude, 0.0), true);
            }
        }
    };
    // The narrowest opening between a split particle and its copy (negative: passed through).
    let gap = |world: &PhysicsWorld| {
        let sb = &world.soft_bodies[handle];
        event
            .split_particles
            .iter()
            .map(|&(copy, source)| {
                let (l, r) = if left[source as usize] { (source, copy) } else { (copy, source) };
                sb.particle_position(r as usize).x - sb.particle_position(l as usize).x
            })
            .fold(Real::MAX, Real::min)
    };
    push(&mut world, -2.0);
    for _ in 0..60 {
        world.step();
    }
    assert!(gap(&world) > 0.05, "the crack did not open: {}", gap(&world));
    assert_eq!(tangles(&world.soft_bodies[handle]), (0, 0));

    push(&mut world, 4.0);
    let mut worst = (0, 0);
    for _ in 0..240 {
        world.step();
        let (inverted, crossings) = tangles(&world.soft_bodies[handle]);
        worst = (worst.0.max(inverted), worst.1.max(crossings));
    }
    assert_finite(&world, handle);
    assert_eq!(
        worst,
        (0, 0),
        "the crack faces passed through each other (inverted cells, crossings)"
    );
}

/// A body with volume preservation cut in two holds each half on its own area: both halves keep
/// their area under a heavy lid, instead of trading it through one constraint over both.
#[test]
fn cut_halves_keep_their_own_areas() {
    let mut world = world_with_ground();
    let grid = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::splat(1.0), 6, 6)
        .volume_preservation(true)
        .particle_mass(0.1)
        .can_sleep(false);
    let handle = world.insert_soft_body(grid);
    world.step();
    let rest_volume = world.soft_bodies[handle].rest_volume();

    let blade = [Vector::new(0.0, -5.0), Vector::new(0.0, 10.0)];
    let event = world
        .cut_soft_body(handle, &blade)
        .expect("the blade crossed the grid");
    // Each half is a body of its own, on its own volume constraint.
    assert_eq!(event.pieces.len(), 2);
    let halves: Vec<SoftBodyHandle> = event.bodies().collect();
    let mut rest = Vec::new();
    for &h in &halves {
        let sb = &world.soft_bodies[h];
        assert!(sb.volume_preservation_enabled());
        assert_eq!(sb.volume_pieces().len(), 1);
        rest.push(sb.volume_pieces()[0].rest_volume());
    }
    assert!(rest.iter().all(|r| *r > 0.5), "a half is too small: {rest:?}");
    assert!((rest[0] + rest[1] - rest_volume).abs() < 1.0e-4 * rest_volume);

    // A heavy lid presses both halves into the ground.
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.4)),
        ColliderBuilder::cuboid(2.0, 0.2).density(10.0),
    );
    for _ in 0..300 {
        world.step();
    }
    for (&h, rest) in halves.iter().zip(&rest) {
        assert_finite(&world, h);
        let sb = &world.soft_bodies[h];
        let area = sb.volume_pieces()[0].volume(sb);
        assert!(
            (area - rest).abs() < 0.03 * rest,
            "a half lost its area under the lid: {area} vs {rest}"
        );
    }
}

/// Two pieces of one body keep the gaps their features had at rest: the halves of a cut body,
/// interlocked along the cells' facets within the contact skins, rest exactly as cut instead of
/// being pushed apart by the skins; shoved into each other, they collide and come back out.
#[test]
fn cut_halves_rest_as_cut_and_collide_when_shoved() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let grid = SoftBodyBuilder::grid(Vector::ZERO, Vector::splat(1.0), 5, 5)
        .particle_mass(0.1)
        .can_sleep(false);
    let handle = world.insert_soft_body(grid);
    world.step();
    let event = world
        .cut_soft_body(handle, &[Vector::new(0.25, -5.0), Vector::new(0.25, 5.0)])
        .expect("the blade crossed the grid");
    let halves: Vec<SoftBodyHandle> = event.bodies().collect();
    assert_eq!(halves.len(), 2);
    let centroid = |world: &PhysicsWorld, h: SoftBodyHandle| world.soft_bodies[h].center_of_mass();
    let offset = |world: &PhysicsWorld| (centroid(world, halves[1]) - centroid(world, halves[0])).x;
    let rest_offset = offset(&world);
    for _ in 0..60 {
        world.step();
        for &h in &halves {
            let sb = &world.soft_bodies[h];
            let speed = sb.particle_velocities().map(|v| v.length()).fold(0.0, Real::max);
            assert!(speed < 1.0e-3, "the halves moved on their own: {speed} m/s");
        }
    }
    assert!((offset(&world) - rest_offset).abs() < 1.0e-3);

    // Shoved into each other by a fraction of a cell, the halves collide and come back out.
    let sign = rest_offset.signum();
    let shove = 0.15 * sign;
    for i in 0..world.soft_bodies[halves[1]].num_particles() {
        let p = world.soft_bodies[halves[1]].particle_position(i);
        world.soft_bodies[halves[1]].set_particle_position(i, p - Vector::X * shove);
    }
    assert!((offset(&world) - rest_offset) * sign < -0.1);
    for _ in 0..120 {
        world.step();
        for &h in &halves {
            assert_finite(&world, h);
        }
    }
    let pushed_back = (offset(&world) - rest_offset) * sign;
    assert!(pushed_back > -0.05, "the shoved halves stayed interlocked: {pushed_back}");
}
