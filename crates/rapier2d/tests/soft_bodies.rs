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
        for k in 0..240 {
            let shift = 2.4 * (k as Real / 180.0).min(1.0);
            for (&i, p) in right.iter().zip(rest.iter()) {
                world.soft_bodies[handle]
                    .set_particle_kinematic_target(i, *p + Vector::new(shift, 0.0));
            }
            world.step();
            assert_finite(&world, handle);
            world.soft_bodies[handle].validate_topology().unwrap();
        }
        let sb = &world.soft_bodies[handle];
        assert!(
        );
        assert!(sb.num_particles() > num_particles, "{model:?}: no particle split");
        // Torn apart: no remaining cell edge is stretched past twice the tear strain.
        let mut max_stretch: Real = 0.0;
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
    let grid = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::new(1.0, 0.25), 5, 2)
    let handle = world.insert_soft_body(grid);
    world.step();
    let sb = &world.soft_bodies[handle];
    let idx = |i: u32, j: u32| i * ny + j;
        .edges()
        .iter()
    assert!(
        world
            .soft_bodies
            .tear(handle, &[], &torn, &mut world.bodies, &mut world.colliders)
    );
    let sb = &world.soft_bodies[handle];
    sb.validate_topology().unwrap();
    let left: Vec<u32> = (0..=top).collect();
    let right: Vec<u32> = (top + 1..num_particles + 2).collect();
    assert_eq!(sb.connected_pieces(), vec![left, right]);
    assert!((sb.mass() - mass).abs() < 1.0e-5);
    assert!((sb.rest_measure() - measure).abs() < 1.0e-5);
    for _ in 0..120 {
        world.step();
        assert_finite(&world, handle);
    }
    world.soft_bodies[handle].validate_topology().unwrap();
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
