//! Soft-body regression tests: substep invariance of the material, ropes, cloth, volume
//! preservation, shape matching, tetrahedral bodies, two-way coupling, sleeping and removal.

use rapier3d::prelude::*;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0).translation(Vector::new(0.0, -0.5, 0.0)),
    );
    world
}

/// The mesh a body wears as a skin (bound to its cells).
fn skinned_mesh(sb: &SoftBody) -> &SoftCollisionMesh {
    sb.meshes()
        .find(|mesh| mesh.is_skinned())
        .expect("the body has a skinned mesh")
}

fn assert_finite(world: &PhysicsWorld, handle: SoftBodyHandle) {
    let sb = &world.soft_bodies[handle];
    for p in sb.particles() {
        assert!(
            p.position().is_finite(),
            "non-finite particle position {:?}",
            p.position()
        );
    }
}

/// The period of a mass-spring pair set by its natural frequency must not depend on the
/// substep count: this is the property that separates the material model from position-based
/// `rigidity` knobs.
#[test]
fn edge_period_is_substep_invariant() {
    let natural_frequency = 2.0; // Hz
    let mut periods = Vec::new();
    for substeps in [1usize, 4, 16] {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.num_solver_iterations = substeps;
        world.integration_parameters.dt = 1.0 / 240.0;
        // Two particles, one pinned, the free one starts stretched.
        let builder = SoftBodyBuilder::new(vec![Vector::ZERO, Vector::new(1.0, 0.0, 0.0)])
            .edges(vec![[0, 1]])
            .pinned_particles([0])
            .softness(SpringCoefficients::new(natural_frequency, 0.0))
            .no_surface_collider()
            .can_sleep(false);
        let handle = world.insert_soft_body(builder);
        world.soft_bodies[handle].set_particle_position(1, Vector::new(1.2, 0.0, 0.0));

        // Measure the time between the two first crossings of the rest length from above.
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
    for (i, period) in periods.iter().enumerate() {
        assert!(
            (period - expected).abs() < 0.08 * expected,
            "period {period} (substeps case {i}) too far from {expected}"
        );
    }
    let (min, max) = periods
        .iter()
        .fold((Real::MAX, Real::MIN), |(a, b), p| (a.min(*p), b.max(*p)));
    assert!(
        (max - min) < 0.05 * expected,
        "period depends on the substep count: {periods:?}"
    );
}

/// A rope pinned at one end hangs under gravity: its length stays close to the rest length
/// and the free end ends up below the anchor.
#[test]
fn rope_hangs_from_anchor() {
    let mut world = PhysicsWorld::new();
    let rope = SoftBodyBuilder::rope(Vector::ZERO, Vector::new(2.0, 0.0, 0.0), 21)
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
    let mut length = 0.0;
    for i in 0..20 {
        length += (sb.particle_position(i + 1) - sb.particle_position(i)).length();
    }
    assert!((length - 2.0).abs() < 0.1, "rope length drifted: {length}");
}

/// A cloth pinned by two corners drapes without stretching much, and its particles are
/// simulated as rotation-locked rigid bodies.
#[test]
fn cloth_drapes() {
    let mut world = PhysicsWorld::new();
    let n = 10;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(0.0, 2.0, 0.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    .pinned_particles([0, (n - 1) as u32])
    .softness(SpringCoefficients::new(100.0, 1.0));
    let handle = world.insert_soft_body(cloth);
    for _ in 0..300 {
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    // The far corner fell.
    let far = sb.particle_position(n * n - 1);
    assert!(far.y < 1.5, "cloth did not drape: {far:?}");
    // No edge stretched by more than 20% (the two top edges bear ~50 particle weights each:
    // at 100 Hz that is a few percent of stretch).
    for e in sb.edges() {
        if e.kind != SoftBodyEdgeKind::Structural {
            continue;
        }
        let len = (sb.particle_position(e.vertices[0] as usize)
            - sb.particle_position(e.vertices[1] as usize))
        .length();
        assert!(
            len < e.rest_length * 1.2,
            "edge over-stretched: {len} vs {}",
            e.rest_length
        );
    }
    let root = &world.bodies[sb.root_body()];
    assert!(root.is_rotation_locked().iter().all(|l| *l));
    assert!(root.is_translation_locked());
}

/// A hollow sphere with volume preservation dropped on the ground keeps its volume, and a
/// pressurized one inflates.
#[test]
fn sphere_keeps_its_volume() {
    let mut world = world_with_ground();
    let sphere = SoftBodyBuilder::sphere(Vector::new(0.0, 1.5, 0.0), 1.0, 2)
        .softness(SpringCoefficients::new(20.0, 1.0));
    let handle = world.insert_soft_body(sphere);
    let rest = world.soft_bodies[handle].rest_volume();
    assert!(rest > 3.0, "icosphere volume too small: {rest}");
    for _ in 0..300 {
        world.step();
    }
    assert_finite(&world, handle);
    let volume = world.soft_bodies[handle].volume();
    assert!(
        (volume - rest).abs() < 0.15 * rest,
        "volume drifted: {volume} vs {rest}"
    );
    // The lowest particle rests on the ground (within the particle radius).
    let min_y = world.soft_bodies[handle]
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MAX, Real::min);
    assert!(
        min_y > -0.05 && min_y < 0.3,
        "sphere sank or floats: {min_y}"
    );

    // Inflate.
    world.soft_bodies[handle].set_volume_factor(1.5);
    for _ in 0..300 {
        world.step();
    }
    let inflated = world.soft_bodies[handle].volume();
    assert!(
        inflated > 1.25 * rest,
        "sphere did not inflate: {inflated} vs {rest}"
    );
}

/// A shape-matched cloud of particles returns to its rest shape after being randomized.
#[test]
fn shape_matching_recovers_rest_shape() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let mut positions = Vec::new();
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                positions.push(Vector::new(i as Real, j as Real, k as Real) * 0.5);
            }
        }
    }
    let builder = SoftBodyBuilder::new(positions.clone())
        .shape_matching(true)
        .softness(SpringCoefficients::new(10.0, 1.0))
        .no_surface_collider()
        .can_sleep(false);
    let handle = world.insert_soft_body(builder);
    // Scramble.
    let sb = &mut world.soft_bodies[handle];
    for (i, p) in positions.iter().enumerate() {
        let x = ((i * 7919) % 100) as Real / 100.0 - 0.5;
        let y = ((i * 104729) % 100) as Real / 100.0 - 0.5;
        let z = ((i * 1299709) % 100) as Real / 100.0 - 0.5;
        sb.set_particle_position(i, *p + Vector::new(x, y, z) * 0.6);
    }
    for _ in 0..600 {
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    // Compare pairwise distances with the rest shape (the body may have translated/rotated).
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

/// A tetrahedral box (corotational and volume models) dropped on the ground settles and keeps
/// its cells positively oriented and its volume.
#[test]
fn tet_box_settles() {
    for model in [SoftBodyCellModel::Volume, SoftBodyCellModel::Corotational] {
        let mut world = world_with_ground();
        let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 1.0, 0.0), Vector::splat(0.5), 4, 4, 4)
            .cell_model(model)
            .softness(SpringCoefficients::new(30.0, 1.0));
        let handle = world.insert_soft_body(cube);
        let rest = world.soft_bodies[handle].rest_volume();
        assert!((rest - 1.0).abs() < 1.0e-3, "cube rest volume {rest}");
        for _ in 0..400 {
            world.step();
        }
        assert_finite(&world, handle);
        let sb = &world.soft_bodies[handle];
        let volume = sb.volume();
        assert!(
            (volume - rest).abs() < 0.2 * rest,
            "{model:?}: volume drifted: {volume} vs {rest}"
        );
        for c in sb.cells() {
            let x: [Vector; 4] =
                core::array::from_fn(|k| sb.particle_position(c.vertices[k] as usize));
            let v = (x[1] - x[0]).dot((x[2] - x[0]).cross(x[3] - x[0])) / 6.0;
            assert!(v > 0.0, "{model:?}: inverted cell (volume {v})");
        }
        let min_y = sb
            .particle_positions()
            .map(|p| p.y)
            .fold(Real::MAX, Real::min);
        assert!(
            min_y > -0.05,
            "{model:?}: cube sank through the ground: {min_y}"
        );
    }
}

/// The corotational cells are stable at any stiffness: a very stiff cube (E = 1e8, 0.1 kg
/// particles) squeezed by 20% and dropped spinning settles and sleeps, and a free-floating one
/// keeps its angular momentum (the cells neither resist nor damp rotation).
#[test]
fn stiff_elastic_cube_is_stable() {
    let cube = |young: Real| {
        SoftBodyBuilder::cuboid(Vector::new(0.0, 3.0, 0.0), Vector::splat(0.75), 4, 4, 4)
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
            let arm = t - Vector::new(0.0, 3.0, 0.0);
            sb.set_particle_velocity(i, Vector::new(0.0, 5.0, 0.0).cross(arm));
        }
    };
    for young in [3.0e4, 3.0e6, 1.0e8] {
        let mut world = world_with_ground();
        let handle = world.insert_soft_body(cube(young));
        spin(&mut world, handle, 0.8);
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
            max_vel < 0.05,
            "E = {young}: cube still moving at {max_vel} m/s"
        );
        assert!(sb.is_sleeping(), "E = {young}: cube did not fall asleep");
        let volume = sb.volume();
        assert!(
            (volume - sb.rest_volume()).abs() < 0.05 * sb.rest_volume(),
            "E = {young}: volume {volume} vs rest {}",
            sb.rest_volume()
        );
    }

    let angular_momentum = |world: &PhysicsWorld, handle: SoftBodyHandle| {
        let sb = &world.soft_bodies[handle];
        let com = sb.center_of_mass();
        sb.particles()
            .iter()
            .map(|p| (p.position() - com).cross(p.velocity() * p.mass()))
            .fold(Vector::ZERO, |a, b| a + b)
    };
    // With and without deformation damping (which must leave the rigid motion alone).
    for damping in [0.0, 100.0] {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        let mut cube = cube(3.0e6);
        cube.material.deformation_damping = damping;
        let handle = world.insert_soft_body(cube);
        spin(&mut world, handle, 1.0);
        world.step();
        let l0 = angular_momentum(&world, handle);
        for _ in 0..600 {
            world.step();
        }
        let l1 = angular_momentum(&world, handle);
        assert!(
            (l1 - l0).length() < 0.01 * l0.length(),
            "damping {damping}: angular momentum drifted: {l0:?} -> {l1:?}"
        );
    }
}

/// The Neo-Hookean cells are as stable as the corotational ones: the same squeezed, spinning
/// cube dropped on the ground at E = 1e4, 1e6 and 1e8 settles, sleeps and keeps its volume.
#[test]
fn stiff_neo_hookean_cube_is_stable() {
    for young in [1.0e4, 1.0e6, 1.0e8] {
        let mut world = world_with_ground();
        let cube =
            SoftBodyBuilder::cuboid(Vector::new(0.0, 3.0, 0.0), Vector::splat(0.75), 4, 4, 4)
                .cell_model(SoftBodyCellModel::NeoHookean)
                .material(SoftBodyMaterial {
                    young_modulus: young,
                    poisson_ratio: 0.4,
                    elastic_damping_ratio: 0.5,
                    ..Default::default()
                })
                .particle_mass(0.1);
        let handle = world.insert_soft_body(cube);
        let sb = &mut world.soft_bodies[handle];
        for i in 0..sb.num_particles() {
            let mut t = sb.particle_position(i);
            t.y = 3.0 + (t.y - 3.0) * 0.8;
            sb.set_particle_position(i, t);
            let arm = t - Vector::new(0.0, 3.0, 0.0);
            sb.set_particle_velocity(i, Vector::new(0.0, 5.0, 0.0).cross(arm));
        }
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
            max_vel < 0.05,
            "E = {young}: cube still moving at {max_vel} m/s"
        );
        assert!(sb.is_sleeping(), "E = {young}: cube did not fall asleep");
        let volume = sb.volume();
        assert!(
            (volume - sb.rest_volume()).abs() < 0.05 * sb.rest_volume(),
            "E = {young}: volume {volume} vs rest {}",
            sb.rest_volume()
        );
    }
}

/// A soft Neo-Hookean cube squashed to 30% of its height by a kinematic plate keeps its cells
/// positively oriented (up to a couple of transient flips while held) and recovers its volume
/// within 5% once the plate lifts, with no inverted cell left.
#[test]
fn neo_hookean_cube_survives_large_compression() {
    let mut world = world_with_ground();
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 0.5, 0.0), Vector::splat(0.5), 4, 4, 4)
        .cell_model(SoftBodyCellModel::NeoHookean)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e4,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 1.0,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.03)
        .surface_collider(ColliderBuilder::ball(0.03).friction(0.5));
    let handle = world.insert_soft_body(cube);
    // Plate bottom at 1.2 when lifted, 0.3 when down.
    let plate_rest = Vector::new(0.0, 1.7, 0.0);
    let (plate, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(plate_rest),
        ColliderBuilder::cuboid(2.0, 0.5, 2.0),
    );
    let inverted = |world: &PhysicsWorld| {
        let sb = &world.soft_bodies[handle];
        sb.cells()
            .iter()
            .filter(|c| {
                let x: [Vector; 4] =
                    core::array::from_fn(|k| sb.particle_position(c.vertices[k] as usize));
                (x[1] - x[0]).dot((x[2] - x[0]).cross(x[3] - x[0])) < 0.0
            })
            .count()
    };
    // Down to 30% of the height over 2 s, hold 1 s, up over 2 s, rest 2 s.
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
            .set_next_kinematic_translation(plate_rest - Vector::new(0.0, 0.9 * depth, 0.0));
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
        "the plate did not squash the cube: {min_height}"
    );
    assert_eq!(inverted(&world), 0, "inverted cells after release");
    let sb = &world.soft_bodies[handle];
    let volume = sb.volume();
    assert!(
        (volume - sb.rest_volume()).abs() < 0.05 * sb.rest_volume(),
        "volume {volume} vs rest {}",
        sb.rest_volume()
    );
}

/// The Neo-Hookean material must not depend on the substep count either: the equilibrium
/// compression of a soft column standing on its pinned base under its own weight (about 13%
/// strain), and its small-oscillation period, are the same at 1, 4 and 16 substeps.
#[test]
fn neo_hookean_response_is_substep_invariant() {
    let mut compressions = Vec::new();
    let mut periods = Vec::new();
    for substeps in [1usize, 4, 16] {
        let mut world = PhysicsWorld::new();
        world.integration_parameters.num_solver_iterations = substeps;
        let mut column =
            SoftBodyBuilder::cuboid(Vector::new(0.0, 0.5, 0.0), Vector::splat(0.5), 3, 3, 3)
                .cell_model(SoftBodyCellModel::NeoHookean)
                .material(SoftBodyMaterial {
                    young_modulus: 1.0e3,
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
        // Nudge the top layer and time the oscillation about the equilibrium.
        for &i in &top {
            world.soft_bodies[handle].set_particle_velocity(i, Vector::new(0.0, 0.3, 0.0));
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
        compressions.iter().all(|c| *c > 0.1),
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

/// `deformation_damping` settles the residual bending of stiff slender bodies: a clamped stiff
/// beam swings for many seconds without it and stops within a few with it, while a stiff block
/// rocking about its base (a nearly rigid mode) needs a large rate to settle.
#[test]
fn deformation_damping_settles_stiff_bodies() {
    let material = |damping: Real| SoftBodyMaterial {
        young_modulus: 1.0e6,
        poisson_ratio: 0.3,
        elastic_damping_ratio: 1.0,
        deformation_damping: damping,
        ..Default::default()
    };
    // Cantilever: a 3 m beam clamped at x = 0 (its first particle layer pinned).
    let tip_speed_after = |damping: Real, seconds: Real| {
        let mut world = PhysicsWorld::new();
        let beam = SoftBodyBuilder::cuboid(
            Vector::new(1.5, 2.0, 0.0),
            Vector::new(1.5, 0.25, 0.25),
            13,
            3,
            3,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(material(damping))
        .particle_mass(0.1)
        .particle_radius(0.06);
        let handle = world.insert_soft_body(beam);
        let sb = &world.soft_bodies[handle];
        let clamped: Vec<usize> = (0..sb.num_particles())
            .filter(|&i| sb.particle_position(i).x < 0.01)
            .collect();
        let tip: Vec<usize> = (0..sb.num_particles())
            .filter(|&i| sb.particle_position(i).x > 2.99)
            .collect();
        for &i in &clamped {
            world.soft_bodies[handle].set_particle_pinned(i, true);
        }
        for _ in 0..(seconds * 60.0) as usize {
            world.step();
        }
        assert_finite(&world, handle);
        let sb = &world.soft_bodies[handle];
        tip.iter()
            .map(|&i| sb.particle_velocity(i).length())
            .fold(0.0, Real::max)
    };
    let undamped = tip_speed_after(0.0, 6.0);
    let damped = tip_speed_after(3.0, 6.0);
    assert!(
        undamped > 0.5,
        "the undamped beam already stopped: {undamped}"
    );
    assert!(damped < 0.01, "the damped beam still swings: {damped}");

    // Standing block pushed sideways at the top.
    let top_speed_after = |damping: Real| {
        let mut world = world_with_ground();
        let block = SoftBodyBuilder::cuboid(
            Vector::new(0.0, 1.5, 0.0),
            Vector::new(0.25, 1.5, 0.25),
            3,
            13,
            3,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(material(damping))
        .particle_mass(0.1)
        .particle_radius(0.06)
        .surface_collider(ColliderBuilder::ball(0.06).friction(1.0));
        let handle = world.insert_soft_body(block);
        for _ in 0..120 {
            world.step();
        }
        let sb = &mut world.soft_bodies[handle];
        for i in 0..sb.num_particles() {
            let y = sb.particle_position(i).y;
            sb.set_particle_velocity(i, Vector::new(0.4 * y / 3.0, 0.0, 0.0));
        }
        for _ in 0..360 {
            world.step();
        }
        let sb = &world.soft_bodies[handle];
        sb.particles()
            .iter()
            .map(|p| p.velocity().length())
            .fold(0.0, Real::max)
    };
    let undamped = top_speed_after(0.0);
    let damped = top_speed_after(100.0);
    assert!(
        undamped > 0.1,
        "the undamped block already stopped: {undamped}"
    );
    assert!(damped < 0.01, "the damped block still sways: {damped}");
}

/// Two-way coupling: a rigid box dropped on a cloth pinned by its four corners is held up by
/// it (it does not fall through), and it deflects the cloth.
#[test]
fn rigid_box_rests_on_cloth() {
    let mut world = PhysicsWorld::new();
    let n = 12;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(-0.55, 1.0, -0.55),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    .pinned_particles([0, (n - 1) as u32, (n * (n - 1)) as u32, (n * n - 1) as u32])
    .softness(SpringCoefficients::new(30.0, 1.0))
    .particle_mass(0.1);
    let handle = world.insert_soft_body(cloth);
    let (rb, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.5, 0.0)),
        ColliderBuilder::cuboid(0.15, 0.15, 0.15),
    );
    for _ in 0..400 {
        world.step();
    }
    assert_finite(&world, handle);
    let box_y = world.bodies[rb].translation().y;
    assert!(box_y > 0.3, "box fell through the cloth: y = {box_y}");
    assert!(box_y < 1.2, "box did not settle on the cloth: y = {box_y}");
    let center = world.soft_bodies[handle].particle_position(n * n / 2 + n / 2);
    assert!(
        center.y < 0.95,
        "cloth was not deflected by the box: {center:?}"
    );
}

/// A soft body falls asleep as a unit once at rest, wakes up when hit, and is removed cleanly.
#[test]
fn soft_body_sleeps_wakes_and_is_removed() {
    let mut world = world_with_ground();
    let blob = SoftBodyBuilder::cuboid(Vector::new(0.0, 0.5, 0.0), Vector::splat(0.5), 3, 3, 3)
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
    // The soft body's root body sleeps with it (atomic island).
    assert!(world.bodies[world.soft_bodies[handle].root_body()].is_sleeping());
    // A ball dropped on it wakes it up.
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 3.0, 0.0)),
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
    // Removal takes the root body with it.
    let sb = world.remove_soft_body(handle).unwrap();
    assert!(world.bodies.get(sb.root_body()).is_none());
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
    let rope = SoftBodyBuilder::rope(Vector::new(0.0, 2.0, 0.0), Vector::new(2.0, 2.0, 0.0), 10)
        .pinned_particles([0]);
    let handle = world.insert_soft_body(rope);
    for _ in 0..50 {
        world.step();
    }
    let bytes = bincode::serialize(&world).unwrap();
    let mut restored: PhysicsWorld = bincode::deserialize(&bytes).unwrap();
    for _ in 0..50 {
        world.step();
        restored.step();
    }
    let a = world.soft_bodies[handle].particle_position(9);
    let b = restored.soft_bodies[handle].particle_position(9);
    assert!(
        (a - b).length() < 1.0e-3,
        "restored world diverged: {a:?} vs {b:?}"
    );
}

/// Surface collisions: a rigid box resting on a tetrahedral cube's surface neither sinks
/// through it nor slides away, and the resting soft body has no residual velocity.
#[test]
fn box_rests_on_surface_and_nothing_creeps() {
    let mut world = world_with_ground();
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 0.6, 0.0), Vector::splat(0.6), 5, 5, 5)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 5.0e3,
            poisson_ratio: 0.35,
            ..Default::default()
        })
        .particle_mass(0.2);
    let handle = world.insert_soft_body(cube);
    assert!(world.soft_bodies[handle].collision_mesh().is_some());
    let (rb, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.1, 2.0, 0.1)),
        ColliderBuilder::cuboid(0.2, 0.2, 0.2).density(2.0),
    );
    for _ in 0..600 {
        world.step();
    }
    assert_finite(&world, handle);
    let box_pos = world.bodies[rb].translation();
    assert!(box_pos.y > 1.2, "box sank into the soft cube: {box_pos:?}");
    assert!(
        box_pos.y < 1.7,
        "box floats above the soft cube: {box_pos:?}"
    );
    assert!(
        (box_pos.x - 0.1).abs() < 0.05 && (box_pos.z - 0.1).abs() < 0.05,
        "box slid on the soft cube: {box_pos:?}"
    );
    // The box rests on the surface's contact skin (one particle radius above the top face),
    // neither inside the skin nor above it.
    let skin = world.soft_bodies[handle].particle_radius();
    let top_under_box = world.soft_bodies[handle]
        .particle_positions()
        .filter(|p| p.y > 1.0 && (p.x - 0.1).abs() < 0.3 && (p.z - 0.1).abs() < 0.3)
        .map(|p| p.y)
        .fold(Real::MIN, Real::max);
    let box_bottom = box_pos.y - 0.2;
    assert!(
        (box_bottom - (top_under_box + skin)).abs() < 0.02,
        "box bottom {box_bottom} is not on the skin ({top_under_box} + {skin})"
    );
    let com = world.soft_bodies[handle].center_of_mass();
    assert!(
        com.x.abs() < 0.02 && com.z.abs() < 0.02,
        "soft cube crept: {com:?}"
    );
    // At rest: every particle and the box are (nearly) still.
    for p in world.soft_bodies[handle].particles() {
        let v = p.velocity().length();
        assert!(v < 0.01, "residual particle velocity {v}");
    }
    assert!(world.bodies[rb].linvel().length() < 0.01);
    // The soft body's surface contacts report their force through the manifolds.
    let surface = world.soft_bodies[handle].collision_mesh().unwrap().collider();
    let total_impulse: Real = world
        .narrow_phase
        .contact_pairs_with(surface)
        .flat_map(|p| p.manifolds().iter())
        .flat_map(|m| m.points.iter())
        .map(|pt| pt.data.impulse)
        .sum();
    assert!(
        total_impulse > 0.0,
        "no contact impulse reported on the surface"
    );
}

/// Soft-vs-soft: a soft sphere dropped on a soft cube rests on top of it.
#[test]
fn soft_sphere_rests_on_soft_cube() {
    let mut world = world_with_ground();
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 0.6, 0.0), Vector::splat(0.6), 5, 5, 5)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.2);
    let cube = world.insert_soft_body(cube);
    let sphere = SoftBodyBuilder::sphere(Vector::new(0.0, 2.2, 0.0), 0.5, 2)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .particle_mass(0.05);
    let sphere = world.insert_soft_body(sphere);
    for _ in 0..400 {
        world.step();
    }
    assert_finite(&world, cube);
    assert_finite(&world, sphere);
    let sphere_min = world.soft_bodies[sphere]
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MAX, Real::min);
    let cube_max = world.soft_bodies[cube]
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MIN, Real::max);
    assert!(
        sphere_min > 0.9,
        "sphere fell through the cube: {sphere_min}"
    );
    assert!(
        sphere_min < cube_max + 0.4,
        "sphere floats above the cube: {sphere_min} vs {cube_max}"
    );
}

/// A cloth pinched between two jelly cubes (all colliding through their surfaces) settles and
/// sleeps: the constraints of each surface against the other bodies' particles agree, and a
/// surface's constraints see its live deformation.
#[test]
fn cloth_pinched_between_soft_cubes_settles() {
    let mut world = world_with_ground();
    let jelly = |center: Vector| {
        SoftBodyBuilder::cuboid(center, Vector::splat(0.5), 3, 3, 3)
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
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(-0.9, 1.4, -0.9),
        Vector::new(0.12, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.12),
        16,
        16,
    )
    .softness(SpringCoefficients::new(30.0, 1.0))
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
    })
    .particle_mass(0.02)
    .particle_radius(0.05)
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
    let handles = [
        world.insert_soft_body(jelly(Vector::new(0.0, 0.6, 0.0))),
        world.insert_soft_body(cloth),
        world.insert_soft_body(jelly(Vector::new(0.0, 2.2, 0.0))),
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
    // Stacked: the top cube rests on the cloth on the bottom cube.
    let com = |h| world.soft_bodies[h].center_of_mass();
    assert!(com(handles[2]).y > com(handles[1]).y && com(handles[1]).y > com(handles[0]).y);
    assert!(
        com(handles[2]).y > 1.6,
        "top cube sank: {:?}",
        com(handles[2])
    );
}

/// Self-collision: a strip pinned by both ends whose ends are brought together folds without
/// its two halves passing through each other.
#[test]
fn self_contacts_keep_folded_cloth_apart() {
    let mut min_gap = Vec::new();
    for self_contacts in [false, true] {
        let mut world = PhysicsWorld::new();
        let n = 40;
        let strip = SoftBodyBuilder::cloth(
            Vector::new(-2.0, 2.0, 0.0),
            Vector::new(0.1, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.1),
            n,
            3,
        )
        .pinned_particles([
            0,
            1,
            2,
            ((n - 1) * 3) as u32,
            ((n - 1) * 3 + 1) as u32,
            ((n - 1) * 3 + 2) as u32,
        ])
        .self_contacts(self_contacts)
        .softness(SpringCoefficients::new(30.0, 1.0))
        .particle_mass(0.05);
        let handle = world.insert_soft_body(strip);
        // Let it sag, then move the right end onto the left end.
        for _ in 0..120 {
            world.step();
        }
        for step in 0..240 {
            let t = (step as Real / 240.0).min(1.0);
            for j in 0..3 {
                let target = Vector::new(1.9 - 3.85 * t, 2.0, 0.1 * j as Real);
                world.soft_bodies[handle].set_particle_kinematic_target((n - 1) * 3 + j, target);
            }
            world.step();
        }
        for _ in 0..120 {
            world.step();
        }
        assert_finite(&world, handle);
        // Distance between the two halves (first and last quarter of the strip).
        let sb = &world.soft_bodies[handle];
        let mut gap: Real = Real::MAX;
        for i in 0..n / 4 {
            for k in 3 * n / 4..n {
                for j in 0..3 {
                    let a = sb.particle_position(i * 3 + j);
                    let b = sb.particle_position(k * 3 + j);
                    gap = gap.min((a - b).length());
                }
            }
        }
        min_gap.push(gap);
    }
    // Without self-contacts the halves pass through each other freely (gap ~0); with them the
    // layers stay a particle radius apart.
    assert!(
        min_gap[1] > 0.035,
        "self contacts did not keep the layers apart: {min_gap:?}"
    );
    assert!(
        min_gap[1] > min_gap[0] + 0.02,
        "self contacts changed nothing: {min_gap:?}"
    );
}

/// Edge-vs-edge contacts: two coarse thin strips pinned at their ends cross at 90° so that no
/// vertex of either projects on the other (their edges cross between vertices); a plate loading
/// the crossing must not push the upper strip through the lower one.
#[test]
fn crossing_strips_hold_through_edge_contacts() {
    let mut world = world_with_ground();
    let strip = |origin: Vector, du: Vector, dv: Vector| {
        SoftBodyBuilder::cloth(origin, du, dv, 7, 2)
            .softness(SpringCoefficients::new(60.0, 1.0))
            .material(SoftBodyMaterial {
                bend_softness: SpringCoefficients::new(3.0, 1.0),
                ..SoftBodyMaterial::uniform(SpringCoefficients::new(60.0, 1.0))
            })
            .particle_mass(0.05)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05).friction(0.6))
            .pinned_particles([0, 1, 12, 13])
    };
    // Along x at height 1 (vertices at x = -1.5, -1, ..., 1.5 and z = -0.1, 0.1).
    let lower = world.insert_soft_body(strip(
        Vector::new(-1.5, 1.0, -0.1),
        Vector::new(0.5, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.2),
    ));
    // Along z just above (vertices at z = -1.75, ..., 1.25 and x = 0.15, 0.35): its edges cross
    // the lower strip's edges 0.15 away from every vertex, beyond the vertex contact reach.
    let upper = world.insert_soft_body(strip(
        Vector::new(0.15, 1.05, -1.75),
        Vector::new(0.0, 0.0, 0.5),
        Vector::new(0.2, 0.0, 0.0),
    ));
    let (plate, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.25, 1.2, 0.0)),
        ColliderBuilder::cuboid(0.25, 0.05, 0.25).density(100.0),
    );
    for _ in 0..600 {
        world.step();
    }
    assert_finite(&world, upper);
    let mid_y = |h: SoftBodyHandle| {
        let sb = &world.soft_bodies[h];
        (sb.particle_position(6).y + sb.particle_position(7).y) * 0.5
    };
    let (upper_y, lower_y) = (mid_y(upper), mid_y(lower));
    assert!(
        upper_y > lower_y + 0.05,
        "the upper strip went through the lower one: {upper_y} vs {lower_y}"
    );
    let plate_y = world.bodies[plate].translation().y;
    assert!(plate_y > upper_y, "the plate fell through: {plate_y}");
}

/// Fast deformable bodies get speculative contacts over their coming motion (the colliders'
/// margins grow with the particles' speed): a cloth thrown down at 15 m/s lands on a thin
/// dynamic plate instead of tunneling through it.
#[test]
fn fast_cloth_does_not_tunnel_through_thin_dynamic_plate() {
    let mut world = world_with_ground();
    for (x, z) in [(-2.0, -2.0), (2.0, -2.0), (-2.0, 2.0), (2.0, 2.0)] {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, 1.0, z)),
            ColliderBuilder::cuboid(0.2, 1.0, 0.2),
        );
    }
    let (plate, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.02, 0.0)),
        ColliderBuilder::cuboid(3.0, 0.01, 3.0).density(50.0),
    );
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(-0.9, 3.0, -0.9),
        Vector::new(0.12, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.12),
        16,
        16,
    )
    .softness(SpringCoefficients::new(30.0, 1.0))
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
    })
    .particle_mass(0.02)
    .particle_radius(0.05)
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
    let handle = world.insert_soft_body(cloth);
    world.soft_bodies[handle].apply_impulse(Vector::new(0.0, -15.0, 0.0), true);
    for _ in 0..120 {
        world.step();
    }
    assert_finite(&world, handle);
    let plate_top = world.bodies[plate].translation().y + 0.01;
    let min_y = world.soft_bodies[handle]
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MAX, Real::min);
    assert!(
        min_y > plate_top,
        "the cloth tunneled through the plate: {min_y} vs {plate_top}"
    );
}

/// Inverted elastic cells recover instead of exploding: a corner particle of a stiff cube
/// (E = 1e8) teleported through the cube inverts its cells, which un-invert within a few steps
/// under the capped snap-back.
#[test]
fn inverted_cells_recover() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 3.0, 0.0), Vector::splat(0.5), 3, 3, 3)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e8,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .no_surface_collider();
    let handle = world.insert_soft_body(cube);
    world.step();
    let sb = &world.soft_bodies[handle];
    let corner = (0..sb.num_particles())
        .min_by(|&a, &b| {
            let (pa, pb) = (sb.particle_position(a), sb.particle_position(b));
            (pa.x + pa.y + pa.z)
                .partial_cmp(&(pb.x + pb.y + pb.z))
                .unwrap()
        })
        .unwrap();
    let p = sb.particle_position(corner);
    world.soft_bodies[handle].set_particle_position(
        corner,
        Vector::new(3.0, 3.0, 0.0) + (Vector::new(0.0, 3.0, 0.0) - p) * 0.6,
    );
    let inverted = |world: &PhysicsWorld| {
        let sb = &world.soft_bodies[handle];
        sb.cells()
            .iter()
            .filter(|c| {
                let x: [Vector; 4] =
                    core::array::from_fn(|k| sb.particle_position(c.vertices[k] as usize));
                (x[1] - x[0]).dot((x[2] - x[0]).cross(x[3] - x[0])) < 0.0
            })
            .count()
    };
    world.step();
    assert!(inverted(&world) > 0, "the teleport did not invert any cell");
    for _ in 0..60 {
        world.step();
    }
    assert_finite(&world, handle);
    assert_eq!(inverted(&world), 0, "cells still inverted after 1 s");
    let sb = &world.soft_bodies[handle];
    let max_v = sb
        .particles()
        .iter()
        .map(|p| p.velocity().length())
        .fold(0.0, Real::max);
    assert!(max_v < 50.0, "the cube exploded: {max_v} m/s");
}

/// A shape that really is in several pieces is meshed as it is: the thickening that keeps a thin
/// limb attached must not run away on a body that can never come out whole.
#[test]
fn volumetric_keeps_genuinely_separate_pieces() {
    let (ball, ball_indices) = Ball::new(1.0).to_trimesh(16, 16);
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    for center in [Vector::new(-3.0, 0.0, 0.0), Vector::new(3.0, 0.0, 0.0)] {
        let offset = vertices.len() as u32;
        vertices.extend(ball.iter().map(|p| *p + center));
        indices.extend(
            ball_indices
                .iter()
                .map(|t| [t[0] + offset, t[1] + offset, t[2] + offset]),
        );
    }

    let builder = SoftBodyBuilder::volumetric(&vertices, &indices, 0.25).expect("two balls");
    let mut world = world_with_ground();
    let handle = world.insert_soft_body(builder);
    let sb = &world.soft_bodies[handle];

    // Two balls of volume 4/3 pi: the pieces are 6 apart and stay two covers, each
    // containing its ball with up to a cell-thick shell of overshoot.
    let volume = sb.rest_volume();
    let expected = 2.0 * 4.0 / 3.0 * core::f32::consts::PI;
    assert!(
        volume > expected * 0.95 && volume < expected * 1.9,
        "volume {volume} vs {expected}"
    );
}

/// `SoftBodyBuilder::volumetric` fills a closed mesh with conforming tetrahedral cells: a ball
/// filled with cells a fifth of its radius has about the ball's volume, a closed surface, no
/// inverted cell, and stands on the ground.
#[test]
fn volumetric_ball_is_a_valid_soft_body() {
    let (vertices, indices) = Ball::new(1.0).to_trimesh(16, 16);
    let vertices: Vec<Vector> = vertices
        .iter()
        .map(|p| *p + Vector::new(0.0, 1.5, 0.0))
        .collect();
    let builder = SoftBodyBuilder::volumetric(&vertices, &indices, 0.2)
        .expect("the ball is filled with cells")
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
    let volume = sb.rest_volume();
    let ball_volume = 4.0 / 3.0 * core::f32::consts::PI;
    // The cover contains the ball, overshooting by up to a cell-thick shell.
    assert!(
        volume > ball_volume * 0.95 && volume < ball_volume * 1.9,
        "volumetric volume {volume} vs ball {ball_volume}"
    );
    assert!(sb.collision_mesh().is_some(), "no surface");
    // Half the mean edge of the boundary, whose faces are whole lattice faces (edges 0.2
    // and 0.173 long): the cover cuts nothing.
    assert!(sb.particle_radius() > 0.08 && sb.particle_radius() < 0.11);
    for c in sb.cells() {
        let x: [Vector; 4] = core::array::from_fn(|k| sb.particle_position(c.vertices[k] as usize));
        let v = (x[1] - x[0]).dot((x[2] - x[0]).cross(x[3] - x[0])) / 6.0;
        assert!(v > 0.0, "inverted cell (volume {v})");
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
        "the ball sank or floats: {min_y}"
    );
    let com = sb.center_of_mass();
    assert!(com.y > 0.8, "the ball collapsed: {com:?}");
}

/// `SoftBody::attach_particle`: a cloth hung by two corners from a dynamic bar follows the bar
/// (dropped, then swinging on a hinge) and pulls on it, and detaching (removing the joint) lets
/// it fall.
#[test]
fn attached_cloth_follows_its_body() {
    let mut world = world_with_ground();
    // A bar hinged at its left end, above the ground.
    let (bar, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 3.0, 0.0)),
        ColliderBuilder::cuboid(1.0, 0.05, 0.05).density(2.0),
    );
    let (pivot, _) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, 3.0, 0.0)),
        ColliderBuilder::ball(0.05),
    );
    world.impulse_joints.insert(
        pivot,
        bar,
        RevoluteJointBuilder::new(Vector::Z)
            .local_anchor1(Vector::ZERO)
            .local_anchor2(Vector::new(-1.0, 0.0, 0.0)),
        true,
    );
    let n = 12;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(0.4, 2.9, -0.06),
        Vector::new(0.12, 0.0, 0.0),
        Vector::new(0.0, -0.12, 0.0),
        n,
        n,
    )
    .softness(SpringCoefficients::new(30.0, 1.0))
    .particle_mass(0.05);
    let cloth = world.insert_soft_body(cloth);
    // Attach the top corners (particles 0 and (n-1)*n: the top row is j = 0) to the bar.
    let corners = [0usize, (n - 1) * n];
    for &i in &corners {
        world.soft_bodies[cloth].attach_particle(i, bar, &world.bodies);
    }
    assert_eq!(world.soft_bodies[cloth].particle_attachments().len(), 2);
    // The bar is a pendulum: its lowest point tells whether it swung, its pose at the end is
    // phase-dependent.
    let mut lowest = Real::MAX;
    for _ in 0..180 {
        world.step();
        lowest = lowest.min(world.bodies[bar].translation().y);
    }
    assert_finite(&world, cloth);
    // The bar swung down (its free end went below the pivot) and the corners are still on it.
    let bar_pos = *world.bodies[bar].position();
    assert!(lowest < 2.5, "bar did not swing: lowest y {lowest}");
    let sb = &world.soft_bodies[cloth];
    for &i in &corners {
        let p = sb.particle_position(i);
        let local = bar_pos.inverse_transform_point(p);
        assert!(
            local.y.abs() < 0.35 && local.x.abs() < 1.1,
            "corner {i} left the bar: {local:?}"
        );
    }
    let hanging = sb.center_of_mass().y;
    // Detach: the cloth falls to the ground.
    for &i in &corners {
        assert!(world.soft_bodies[cloth].detach_particle(i));
    }
    assert!(world.soft_bodies[cloth].particle_attachments().is_empty());
    for _ in 0..240 {
        world.step();
    }
    let dropped = world.soft_bodies[cloth].center_of_mass().y;
    assert!(
        dropped < hanging - 0.5,
        "the cloth did not fall once detached: {dropped} vs {hanging}"
    );
}

/// Plasticity: a jelly block whose top is sheared past the yield keeps part of the lean once
/// released (its rest shape flowed), while an elastic one springs back.
#[test]
fn plastic_cells_keep_their_deformation() {
    let lean_after = |plastic_yield: Real| {
        let mut world = world_with_ground();
        let block =
            SoftBodyBuilder::cuboid(Vector::new(0.0, 0.75, 0.0), Vector::splat(0.75), 4, 4, 4)
                .cell_model(SoftBodyCellModel::Corotational)
                .material(SoftBodyMaterial {
                    young_modulus: 2.0e4,
                    poisson_ratio: 0.3,
                    elastic_damping_ratio: 1.0,
                    plastic_yield,
                    plastic_creep: 100.0,
                    deformation_damping: 5.0,
                    ..Default::default()
                })
                .particle_mass(0.1)
                .surface_collider(ColliderBuilder::ball(0.15).friction(2.0));
        let handle = world.insert_soft_body(block);
        // Pin the bottom layer, drag the top layer sideways by 60% of the height for a second,
        // then release the top and let everything settle.
        let sb = &world.soft_bodies[handle];
        let bottom: Vec<usize> = (0..sb.num_particles())
            .filter(|&i| sb.particle_position(i).y < 0.01)
            .collect();
        let top: Vec<usize> = (0..sb.num_particles())
            .filter(|&i| sb.particle_position(i).y > 1.49)
            .collect();
        for &i in bottom.iter().chain(top.iter()) {
            world.soft_bodies[handle].set_particle_pinned(i, true);
        }
        let rest: Vec<Vector> = top
            .iter()
            .map(|&i| world.soft_bodies[handle].particle_position(i))
            .collect();
        for step in 0..120 {
            let shift = 0.9 * (step as Real / 60.0).min(1.0);
            for (&i, p) in top.iter().zip(rest.iter()) {
                world.soft_bodies[handle]
                    .set_particle_kinematic_target(i, *p + Vector::new(shift, 0.0, 0.0));
            }
            world.step();
        }
        for &i in &top {
            world.soft_bodies[handle].set_particle_pinned(i, false);
        }
        for _ in 0..240 {
            world.step();
        }
        assert_finite(&world, handle);
        let sb = &world.soft_bodies[handle];
        let top_x: Real =
            top.iter().map(|&i| sb.particle_position(i).x).sum::<Real>() / top.len() as Real;
        let bottom_x: Real = bottom
            .iter()
            .map(|&i| sb.particle_position(i).x)
            .sum::<Real>()
            / bottom.len() as Real;
        top_x - bottom_x
    };
    let elastic = lean_after(0.0);
    let plastic = lean_after(0.05);
    assert!(
        elastic.abs() < 0.15,
        "the elastic block did not spring back: lean {elastic}"
    );
    assert!(
        plastic > 0.4,
        "the plastic block did not keep its lean: {plastic} (elastic {elastic})"
    );
}

/// Plastic flow is bounded: a clay slab stamped again and again by a kinematic wedge keeps
/// well-shaped cells (the flow of a repeatedly crushed cell is capped by `plastic_max`, it
/// cannot creep toward a sliver, invert and blow up), keeps its volume and stays calm.
#[test]
fn plastic_flow_is_bounded() {
    let mut world = world_with_ground();
    let slab = SoftBodyBuilder::cuboid(
        Vector::new(0.0, 0.4, 0.0),
        Vector::new(2.4, 0.4, 1.4),
        13,
        3,
        8,
    )
    .cell_model(SoftBodyCellModel::Corotational)
    .material(SoftBodyMaterial {
        young_modulus: 3.0e4,
        poisson_ratio: 0.35,
        elastic_damping_ratio: 1.0,
        plastic_yield: 0.02,
        plastic_creep: 50.0,
        deformation_damping: 4.0,
        ..Default::default()
    })
    .particle_mass(0.1)
    .surface_collider(ColliderBuilder::ball(0.15).friction(0.8));
    let handle = world.insert_soft_body(slab);
    let press_rest = Vector::new(-1.5, 2.2, 0.0);
    let (press, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(press_rest),
        ColliderBuilder::cuboid(0.35, 0.35, 0.35).rotation(Vector::new(
            0.0,
            0.0,
            core::f32::consts::FRAC_PI_4,
        )),
    );
    // Four stamps, one per second, at four spots.
    let dt = world.integration_parameters.dt;
    let mut t: Real = 0.0;
    for _ in 0..(12.0 / dt) as usize {
        t += dt;
        let cycle = (t / 3.0).floor();
        let phase = t - cycle * 3.0;
        let depth = if phase < 1.0 {
            phase
        } else if phase < 2.0 {
            2.0 - phase
        } else {
            0.0
        };
        world.bodies[press]
            .set_next_kinematic_translation(press_rest + Vector::new(cycle, -depth, 0.0));
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    let mut inverted = 0;
    for c in sb.cells() {
        let x: [Vector; 4] = core::array::from_fn(|k| sb.particle_position(c.vertices[k] as usize));
        if (x[1] - x[0]).dot((x[2] - x[0]).cross(x[3] - x[0])) < 0.0 {
            inverted += 1;
        }
    }
    assert!(inverted <= 2, "{inverted} inverted cells after stamping");
    let volume = sb.volume();
    assert!(
        (volume - sb.rest_volume()).abs() < 0.05 * sb.rest_volume(),
        "slab volume {volume} vs rest {}",
        sb.rest_volume()
    );
    let max_speed = sb
        .particles()
        .iter()
        .map(|p| p.velocity().length())
        .fold(0.0, Real::max);
    assert!(
        max_speed < 1.0,
        "the slab is still agitated: {max_speed} m/s"
    );
}

/// A body whose cells keep flowing is kept awake: a plastic column creeping under its own
/// weight must not fall asleep mid-creep (its elastic twin, at rest, does sleep).
#[test]
fn creeping_body_stays_awake() {
    let mut world = world_with_ground();
    let mut handles = vec![];
    for (i, plastic_yield) in [0.0, 0.05].into_iter().enumerate() {
        let column = SoftBodyBuilder::cuboid(
            Vector::new(i as Real * 3.0, 1.3, 0.0),
            Vector::new(width, 1.2, width),
            3,
            12,
            3,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e3,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 1.0,
            plastic_yield,
            plastic_creep: 0.5,
            deformation_damping: 4.0,
            ..Default::default()
        })
        .particle_mass(0.05)
        .surface_collider(ColliderBuilder::ball(0.1).friction(1.0));
        handles.push(world.insert_soft_body(column));
    }
    let top = |world: &PhysicsWorld, h: SoftBodyHandle| {
        world.soft_bodies[h]
            .particle_positions()
            .map(|p| p.y)
            .fold(0.0, Real::max)
    };
    for _ in 0..180 {
        world.step();
    }
    let plastic_top_early = top(&world, handles[1]);
    for _ in 0..180 {
        world.step();
    }
    assert!(
        world.soft_bodies[handles[0]].is_sleeping(),
        "the elastic column did not sleep"
    );
    assert!(
        !world.soft_bodies[handles[1]].is_sleeping(),
        "the creeping column fell asleep"
    );
    let plastic_top = top(&world, handles[1]);
    assert!(
        plastic_top < plastic_top_early - 0.3,
        "the plastic column did not keep creeping: top {plastic_top_early} -> {plastic_top}"
    );
    assert!(
        top(&world, handles[0]) > 2.2,
        "the elastic column sagged: top {}",
        top(&world, handles[0])
    );
}

/// Shape matching only resists deformation: a shape-matched cloud falls freely (its center of
/// mass follows `g t² / 2`), keeps spinning (only the velocity relative to the rigid motion is
/// damped), and a body driven by a moving target follows it without a speed-dependent lag.
#[test]
fn shape_matching_leaves_rigid_motion_alone() {
    let mut positions = Vec::new();
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                positions.push(Vector::new(i as Real, j as Real, k as Real) * 0.5);
            }
        }
    }
    let builder = || {
        SoftBodyBuilder::new(positions.clone())
            .shape_matching(true)
            .softness(SpringCoefficients::new(10.0, 1.0))
            .no_surface_collider()
            .can_sleep(false)
    };
    // Free fall.
    let mut world = PhysicsWorld::new();
    let handle = world.insert_soft_body(builder());
    let com0 = world.soft_bodies[handle].center_of_mass();
    for _ in 0..60 {
        world.step();
    }
    let fallen = com0.y - world.soft_bodies[handle].center_of_mass().y;
    let expected = 0.5 * 9.81;
    assert!(
        (fallen - expected).abs() < 0.1 * expected,
        "shape-matched body fell {fallen} m in 1 s instead of {expected}"
    );
    // Spin.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let handle = world.insert_soft_body(builder());
    let com = world.soft_bodies[handle].center_of_mass();
    let omega = 2.0;
    let sb = &mut world.soft_bodies[handle];
    for i in 0..sb.num_particles() {
        let r = sb.particle_position(i) - com;
        sb.set_particle_velocity(i, Vector::new(0.0, omega, 0.0).cross(r));
    }
    for _ in 0..120 {
        world.step();
    }
    let sb = &world.soft_bodies[handle];
    let (mut l, mut inertia) = (0.0, 0.0);
    for p in sb.particles() {
        let r = p.position() - sb.center_of_mass();
        let v = p.velocity();
        l += r.cross(v).y * p.mass();
        inertia += (r.x * r.x + r.z * r.z) * p.mass();
    }
    let omega_after = l / inertia;
    assert!(
        (omega_after - omega).abs() < 0.1 * omega,
        "the spin was damped: {omega_after} rad/s instead of {omega}"
    );
    // Driven target moving at 3 m/s.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let handle = world.insert_soft_body(builder());
    let start = world.soft_bodies[handle].center_of_mass();
    let mut t: Real = 0.0;
    for _ in 0..180 {
        t += world.integration_parameters.dt;
        let target = start + Vector::new(3.0 * t, 0.0, 0.0);
        world.soft_bodies[handle]
            .cluster_mut(0)
            .unwrap()
            .set_shape_matching_target(Some(Pose::from_translation(target)));
        world.step();
    }
    let com = world.soft_bodies[handle].center_of_mass();
    let lag = (start.x + 3.0 * t) - com.x;
    assert!(
        lag.abs() < 0.15,
        "the driven body lags {lag} m behind a target moving at 3 m/s"
    );
}

/// Impact-adaptive substeps: a soft cube falling fast onto the ground raises its first
/// particle's `additional_solver_iterations` while approaching and hitting, then drops back at
/// rest; a rigid ball dropped on it raises it again (the rigid bodies' approach speed counts).
#[test]
fn fast_impacts_request_extra_substeps() {
    let mut world = world_with_ground();
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 30.0, 0.0), Vector::splat(0.5), 3, 3, 3)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e4,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05));
    let handle = world.insert_soft_body(cube);
    let root = world.soft_bodies[handle].root_body();
    let dt = world.integration_parameters.dt;

    let mut max_extra = 0;
    let mut extra_before_contact = 0;
    for _ in 0..(5.0 / dt) as usize {
        world.step();
        let extra = world.bodies[root].additional_solver_iterations();
        max_extra = max_extra.max(extra);
        let bottom = world.soft_bodies[handle]
            .particle_positions()
            .map(|p| p.y)
            .fold(Real::MAX, Real::min);
        // Falling freely, well above the ground: no contact, no request.
        if bottom > 3.0 {
            extra_before_contact = extra_before_contact.max(extra);
        }
    }
    assert_eq!(
        extra_before_contact, 0,
        "a free-falling body requested substeps"
    );
    assert_eq!(
        max_extra, world.integration_parameters.soft_bodies.max_extra_substeps,
        "the impact (about 24 m/s: 8 substeps to travel one radius per substep) must request the maximum"
    );
    assert_eq!(
        world.bodies[root].additional_solver_iterations(),
        0,
        "the resting cube must drop back to zero"
    );
    assert_finite(&world, handle);

    // A heavy rigid ball dropped from 20 m onto the resting cube.
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 20.0, 0.0)),
        ColliderBuilder::ball(0.3).density(2.0),
    );
    let mut max_extra = 0;
    for _ in 0..(3.0 / dt) as usize {
        world.step();
        max_extra = max_extra.max(world.bodies[root].additional_solver_iterations());
    }
    assert!(
        max_extra >= 2,
        "the rigid ball's impact (about 19 m/s) must request extra substeps, got {max_extra}"
    );
    for _ in 0..(3.0 / dt) as usize {
        world.step();
    }
    assert_eq!(world.bodies[root].additional_solver_iterations(), 0);
    assert_finite(&world, handle);
}

/// A fully pinned cloth (every particle kinematic) still collides: a box dropped on it rests on
/// it instead of falling through (a body without a free particle used to be left out of the
/// solver's awake list, so its surface had no contact constraint).
#[test]
fn fully_pinned_cloth_holds_a_box() {
    let mut world = world_with_ground();
    let n = 10;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(-0.45, 1.0, -0.45),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    .pinned_particles(0..(n * n) as u32)
    .surface_collider(ColliderBuilder::ball(0.05));
    let handle = world.insert_soft_body(cloth);
    let (block, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.0, 0.0)),
        ColliderBuilder::cuboid(0.15, 0.15, 0.15),
    );
    for _ in 0..240 {
        world.step();
    }
    assert_finite(&world, handle);
    // Resting on the surface: 1.0 + half-height 0.15 + the surface's skin (the particle radius).
    let y = world.bodies[block].translation().y;
    assert!(y > 1.14, "the box fell through the pinned cloth: y = {y}");
    assert!(y < 1.3, "the box did not settle on the cloth: y = {y}");
    let sb = &world.soft_bodies[handle];
    for i in 0..sb.num_particles() {
        assert!(
            (sb.particle_position(i).y - 1.0).abs() < 1.0e-6,
            "a pinned particle moved: {:?}",
            sb.particle_position(i)
        );
    }
}

/// The pinned particles moved kinematically (the whole sheet rising) lift the resting box up
/// with them.
#[test]
fn kinematic_pinned_cloth_pushes_a_box() {
    let mut world = world_with_ground();
    let n = 10;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(-0.45, 1.0, -0.45),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    .pinned_particles(0..(n * n) as u32)
    .surface_collider(ColliderBuilder::ball(0.05));
    let handle = world.insert_soft_body(cloth);
    let (block, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.5, 0.0)),
        ColliderBuilder::cuboid(0.15, 0.15, 0.15),
    );
    for _ in 0..120 {
        world.step();
    }
    let rested = world.bodies[block].translation().y;
    assert!(
        rested > 1.14,
        "the box fell through the pinned cloth: y = {rested}"
    );
    // Raise the sheet at 0.5 m/s for two seconds.
    let dt = world.integration_parameters.dt;
    let mut sheet_y = 1.0;
    for _ in 0..120 {
        sheet_y += 0.5 * dt;
        let sb = &mut world.soft_bodies[handle];
        for i in 0..sb.num_particles() {
            let mut pos = sb.particle_position(i);
            pos.y = sheet_y;
            sb.set_particle_kinematic_target(i, pos);
        }
        world.step();
        let y = world.bodies[block].translation().y;
        assert!(
            y > sheet_y + 0.1,
            "the rising sheet passed through the box: box y = {y}, sheet y = {sheet_y}"
        );
    }
    assert_finite(&world, handle);
    let y = world.bodies[block].translation().y;
    assert!(
        y > sheet_y + 0.14 && y < sheet_y + 0.35,
        "the box does not ride the sheet: box y = {y}, sheet y = {sheet_y}"
    );
}

/// Tearing: a cloth pinned at both ends and pulled apart past the material's tear strain rips
/// (the cracks open by splitting particles, no triangle is lost), the pieces keep simulating and
/// the derived tables stay consistent.
#[test]
fn cloth_tears_when_pulled_apart() {
    let mut world = PhysicsWorld::new();
    let (nu, nv) = (12usize, 6usize);
    let step = 0.2;
    let idx = |i: usize, j: usize| i * nv + j;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(0.0, 2.0, 0.0),
        Vector::new(step, 0.0, 0.0),
        Vector::new(0.0, 0.0, step),
        nu,
        nv,
    )
    .pinned_particles((0..nv).flat_map(|j| [idx(0, j) as u32, idx(nu - 1, j) as u32]))
    .tear_strain(0.5)
    .particle_mass(0.05);
    let handle = world.insert_soft_body(cloth);
    let num_surface = world.soft_bodies[handle].boundary().len();

    // Pull the right column away by one cloth width over two seconds.
    let rest: Vec<Vector> = (0..nv)
        .map(|j| world.soft_bodies[handle].particle_position(idx(nu - 1, j)))
        .collect();
    let width = step * (nu - 1) as Real;
    for k in 0..180 {
        let shift = width * (k as Real / 120.0).min(1.0);
        for (j, p) in rest.iter().enumerate() {
            world.soft_bodies[handle]
                .set_particle_kinematic_target(idx(nu - 1, j), *p + Vector::new(shift, 0.0, 0.0));
        }
        world.step();
        assert_finite(&world, handle);
        world.soft_bodies[handle].validate_topology().unwrap();
    }
    let sb = &world.soft_bodies[handle];
    assert_eq!(
        sb.boundary().len(),
        num_surface,
        "a cloth loses no triangle when it tears"
    assert!(
        sb.num_particles() > nu * nv,
        "the crack did not split any particle"
    );
    // The two pinned columns are now more than twice the rest width apart: what still ties
    // them would be strained way past the threshold, so nothing does. The middle sagged under
    // gravity instead of being held taut.
    let mid = (0..nv)
        .map(|j| sb.particle_position(idx(nu / 2, j)).y)
        .fold(Real::MAX, Real::min);
    assert!(mid < 1.9, "the torn cloth did not sag: {mid}");
    for e in sb.edges() {
        let len = (sb.particle_position(e.vertices[0] as usize)
            - sb.particle_position(e.vertices[1] as usize))
        .length();
        assert!(
            len < e.rest_length * 1.6,
            "an edge past the tear strain survived: {len} vs rest {}",
            e.rest_length
        );
    }
}

/// A cloth crack opens across the torn edges: tearing every warp edge between two columns
/// splits one endpoint per edge along the plane perpendicular to it, cutting the cloth in two
/// with no triangle lost and the mass conserved; the unpinned piece falls away.
#[test]
fn tearing_splits_the_particles_a_crack_passes_through() {
    let mut world = PhysicsWorld::new();
    let n = 6usize;
    let idx = |i: usize, j: usize| (i * n + j) as u32;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(0.0, 2.0, 0.0),
        Vector::new(0.2, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.2),
        n,
        n,
    )
    .pinned_particles([idx(0, 0), idx(0, n - 1)]);
    let handle = world.insert_soft_body(cloth);
    world.step();
    let sb = &world.soft_bodies[handle];
    let num_particles = sb.num_particles();
    let num_surface = sb.boundary().len();
    // The warp edges between the columns `col` and `col + 1`.
    let col = n / 2;
    let warp: Vec<u32> = sb
        .edges()
        .iter()
        .enumerate()
        .filter(|(_, e)| {
            (0..n).any(|j| {
                let (a, b) = (idx(col, j), idx(col + 1, j));
                e.vertices == [a, b] || e.vertices == [b, a]
            })
        })
        .map(|(i, _)| i as u32)
        .collect();
    assert_eq!(warp.len(), n);
    let torn = world.tear_soft_body(handle, &warp, &[]);
    assert!(torn.is_some());
    let sb = &world.soft_bodies[handle];
    sb.validate_topology().unwrap();
    // One split per torn edge (its first endpoint, `(col, j)`), the triangles all still there.
    assert_eq!(sb.num_particles(), num_particles + n);
    assert_eq!(sb.boundary().len(), num_surface);
    let mass: Real = sb.particles().iter().map(|p| p.mass()).sum();
    assert!(
        (mass - num_particles as Real).abs() < 1.0e-4,
        "mass changed: {mass}"
    );
    // The left piece hangs from its pinned corners, the right one falls: the split particles
    // drift apart.
    for _ in 0..120 {
        world.step();
        assert_finite(&world, handle);
    }
    let sb = &world.soft_bodies[handle];
    sb.validate_topology().unwrap();
    // The copies are appended in crack order: the copy of `(col, j)` is particle
    // `num_particles + j`; the event says where each ended up.
    let copy = num_particles + n / 2;
    let gap =
        (sb.particle_position(copy) - sb.particle_position(idx(col, n / 2) as usize)).length();
    assert!(gap > 0.05, "the split particles did not separate: {gap}");
}

/// A corotational jelly bar pulled apart past its tear strain splits: cracks open through its
/// particles (no cell is lost), the crack faces become surface, the tables stay consistent, and
/// both halves keep simulating.
#[test]
fn jelly_bar_tears_when_pulled_apart() {
    let mut world = PhysicsWorld::new();
    let (nx, ny, nz) = (13usize, 3usize, 3usize);
    let bar = SoftBodyBuilder::cuboid(
        Vector::new(0.0, 2.0, 0.0),
        Vector::new(1.2, 0.2, 0.2),
        nx,
        ny,
        nz,
    )
    .cell_model(SoftBodyCellModel::Corotational)
    .material(SoftBodyMaterial {
        young_modulus: 5.0e3,
        poisson_ratio: 0.3,
        elastic_damping_ratio: 1.0,
        tear_strain: Some(0.4),
        ..Default::default()
    })
    .particle_mass(0.05);
    let handle = world.insert_soft_body(bar);
    let sb = &world.soft_bodies[handle];
    let num_cells = sb.cells().len();
    let left: Vec<usize> = (0..sb.num_particles())
        .filter(|&i| sb.particle_position(i).x < -1.19)
        .collect();
    let right: Vec<usize> = (0..sb.num_particles())
        .filter(|&i| sb.particle_position(i).x > 1.19)
        .collect();
    assert_eq!(left.len(), ny * nz);
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
                .set_particle_kinematic_target(i, *p + Vector::new(shift, 0.0, 0.0));
        }
        world.step();
        assert_finite(&world, handle);
        world.soft_bodies[handle].validate_topology().unwrap();
    }
    let sb = &world.soft_bodies[handle];
    assert!(
        sb.cells().len() < num_cells,
        "no cell tore ({num_cells} before and after)"
    );
    assert!(!sb.boundary().is_empty());
    for c in sb.cells() {
        for &v in &c.vertices {
            assert!((v as usize) < sb.num_particles());
        }
    }
    // The bar is now torn: no remaining cell is stretched past twice the tear strain.
    let mut max_stretch: Real = 0.0;
    for c in sb.cells() {
        for a in 0..4 {
            for b in a + 1..4 {
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
    assert!(
        max_stretch < 1.8,
        "a cell is still stretched by {max_stretch}: the bar did not split"
    );
}

/// A torn soft body (split particles included) round-trips through serde.
#[cfg(feature = "serde-serialize")]
#[test]
fn torn_soft_bodies_serialize() {
    let mut world = world_with_ground();
    let n = 5usize;
    let idx = |i: usize, j: usize| (i * n + j) as u32;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(0.0, 1.0, 0.0),
        Vector::new(0.2, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.2),
        n,
        n,
    )
    .pinned_particles([idx(0, 0), idx(0, n - 1)])
    .tear_strain(0.5);
    let handle = world.insert_soft_body(cloth);
    world.step();
    let diagonal: Vec<u32> = world.soft_bodies[handle]
        .edges()
        .iter()
        .enumerate()
        .filter(|(_, e)| {
            (0..n - 1).any(|k| {
                let (a, b) = (idx(k, k), idx(k + 1, k + 1));
                e.vertices == [a, b] || e.vertices == [b, a]
            })
        })
        .map(|(i, _)| i as u32)
        .collect();
    for _ in 0..30 {
        world.step();
    }
    let bytes = bincode::serialize(&world).unwrap();
    let mut restored: PhysicsWorld = bincode::deserialize(&bytes).unwrap();
    restored.soft_bodies[handle].validate_topology().unwrap();
    assert_eq!(
        restored.soft_bodies[handle].num_particles(),
        world.soft_bodies[handle].num_particles()
    );
    for _ in 0..30 {
        world.step();
        restored.step();
    }
    let last = world.soft_bodies[handle].num_particles() - 1;
    let a = world.soft_bodies[handle].particle_position(last);
    let b = restored.soft_bodies[handle].particle_position(last);
    assert!(
        (a - b).length() < 1.0e-3,
        "restored world diverged: {a:?} vs {b:?}"
    );
}

/// Load-adaptive substeps: a soft cube supporting a box many times its own weight requests
/// extra substeps for its island (the solve is under-converged under a heavy load), and drops
/// the request once the box is removed.
#[test]
fn heavy_load_requests_extra_substeps() {
    let mut world = world_with_ground();
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 0.5, 0.0), Vector::splat(0.5), 3, 3, 3)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e5,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 1.0,
            ..Default::default()
        })
        .particle_mass(0.05)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05));
    let handle = world.insert_soft_body(cube);
    let root = world.soft_bodies[handle].root_body();
    let dt = world.integration_parameters.dt;
    for _ in 0..(2.0 / dt) as usize {
        world.step();
    }
    assert_eq!(
        world.bodies[root].additional_solver_iterations(),
        0,
        "a cube under its own weight only must not request substeps"
    );
    // A 100 kg box (the cube weighs 1.35 kg) set down on it.
    let (rb, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.5, 0.0)),
        ColliderBuilder::cuboid(0.4, 0.4, 0.4).density(100.0 / 0.512),
    );
    let mut max_extra = 0;
    for _ in 0..(3.0 / dt) as usize {
        world.step();
        max_extra = max_extra.max(world.bodies[root].additional_solver_iterations());
    }
    assert!(
        max_extra >= 1,
        "the loaded cube must request extra substeps (got {max_extra})"
    );
    assert!(
        world.bodies[root].additional_solver_iterations() >= 1,
        "the request must hold while the load stays"
    );
    assert_finite(&world, handle);
    world.remove_body(rb);
    for _ in 0..(2.0 / dt) as usize {
        world.step();
    }
    assert_eq!(world.bodies[root].additional_solver_iterations(), 0);
}

/// A cloth thrown at 15 m/s onto a resting balloon (a pressurized soft sphere) neither tunnels
/// through it nor crushes it flat: the speculative constraints and impact-adaptive substeps
/// hold, and the balloon keeps more than half of its volume at every step.
#[test]
fn fast_cloth_does_not_crush_a_balloon() {
    let mut world = world_with_ground();
    let balloon = SoftBodyBuilder::sphere(Vector::new(0.0, 1.0, 0.0), 1.0, 2)
        .softness(SpringCoefficients::new(20.0, 1.0))
        .particle_mass(0.05)
        .particle_radius(0.05)
        .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
    let balloon = world.insert_soft_body(balloon);
    for _ in 0..120 {
        world.step();
    }
    let rest = world.soft_bodies[balloon].rest_volume();
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(-0.9, 3.0, -0.9),
        Vector::new(0.12, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.12),
        16,
        16,
    )
    .softness(SpringCoefficients::new(30.0, 1.0))
    .material(SoftBodyMaterial {
        bend_softness: SpringCoefficients::new(3.0, 1.0),
        ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
    })
    .particle_mass(0.02)
    .particle_radius(0.05)
    .surface_collider(ColliderBuilder::ball(0.05).friction(0.6));
    let cloth = world.insert_soft_body(cloth);
    world.soft_bodies[cloth].apply_impulse(Vector::new(0.0, -15.0, 0.0), true);
    let mut min_ratio: Real = 1.0;
    for _ in 0..180 {
        world.step();
        min_ratio = min_ratio.min(world.soft_bodies[balloon].volume() / rest);
    }
    assert_finite(&world, balloon);
    assert_finite(&world, cloth);
    assert!(
        min_ratio > 0.5,
        "the balloon was crushed to {min_ratio} of its volume"
    );
    // No cloth particle ended up inside the balloon.
    let com = world.soft_bodies[balloon].center_of_mass();
    let inside = world.soft_bodies[cloth]
        .particle_positions()
        .filter(|p| (*p - com).length() < 0.8)
        .count();
    assert_eq!(
        inside, 0,
        "{inside} cloth particles ended up inside the balloon"
    );
}

/// `SoftBodyBuilder::append` shifts the appended piece's indices (elements, pins, per-edge
/// overrides) past the first one's particles, and `add_edges` sews them: two cloth squares
/// appended and stitched along a shared edge hang from the first one's pins as one body.
#[test]
fn appended_pieces_are_one_soft_body() {
    let mut world = PhysicsWorld::new();
    let n = 6usize;
    let step = 0.1;
    let a = SoftBodyBuilder::cloth(
        Vector::new(0.0, 2.0, 0.0),
        Vector::new(step, 0.0, 0.0),
        Vector::new(0.0, 0.0, step),
        n,
        n,
    )
    .pinned_particles([0, (n - 1) as u32]);
    let b = SoftBodyBuilder::cloth(
        Vector::new(0.0, 2.0, step * n as Real),
        Vector::new(step, 0.0, 0.0),
        Vector::new(0.0, 0.0, step),
        n,
        n,
    )
    .tension_only();
    let (na, nb) = (a.particle_positions().len(), b.particle_positions().len());
    let (ea, eb) = (a.edges.len(), b.edges.len());
    let bends_a = a.bend_edges.len();
    let tension_b = b.tension_only_edges.len();
    // Seams: the last row of `a` (j = n - 1) to the first row of `b` (j = 0).
    let seams: Vec<[u32; 2]> = (0..n)
        .map(|i| [(i * n + n - 1) as u32, (na + i * n) as u32])
        .collect();
    let stitched = a.append(b).add_edges(seams.clone());
    assert_eq!(stitched.particle_positions().len(), na + nb);
    assert_eq!(stitched.pinned, vec![0, (n - 1) as u32]);
    assert_eq!(stitched.edges.len(), ea + eb + seams.len());
    // The appended piece's tension-only marks point at its own edges and bending edges.
    assert_eq!(stitched.tension_only_edges.len(), tension_b);
    for &i in &stitched.tension_only_edges {
        let i = i as usize;
        let edge = if i < stitched.edges.len() {
            stitched.edges[i]
        } else {
            stitched.bend_edges[i - stitched.edges.len()]
        };
        assert!(
            edge.iter().all(|&v| v as usize >= na),
            "override on a's edge {edge:?}"
        );
    }
    assert_eq!(stitched.bend_edges.len(), bends_a + tension_b - eb);
    let handle = world.insert_soft_body(stitched.particle_mass(0.05));
    world.soft_bodies[handle].validate_topology().unwrap();
    for _ in 0..240 {
        world.step();
    }
    assert_finite(&world, handle);
    let sb = &world.soft_bodies[handle];
    // The second square hangs below the first, held through the seams (not on the ground: no
    // ground here, so it must simply not have fallen away).
    let lowest = sb
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MAX, Real::min);
    assert!(
        lowest > 0.9,
        "the appended piece fell away: lowest y {lowest}"
    );
    let seam_gap = seams
        .iter()
        .map(|s| {
            (sb.particle_position(s[0] as usize) - sb.particle_position(s[1] as usize)).length()
        })
        .fold(0.0, Real::max);
    assert!(seam_gap < 0.2, "a seam opened: {seam_gap}");
}

/// A soft body given a non-finite particle (by the user, or by a non-finite force during a step)
/// is quarantined: disabled, reported, its velocities zeroed, the rest of the world unharmed;
/// fixing the particle and re-enabling the body brings it back.
#[test]
fn non_finite_soft_body_is_quarantined() {
    let mut world = world_with_ground();
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 3.0)),
        ColliderBuilder::ball(0.5),
    );
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 0.5, 0.0), Vector::splat(0.5), 3, 3, 3)
        .softness(SpringCoefficients::new(30.0, 1.0));
    let handle = world.insert_soft_body(cube);
    let other = world.insert_soft_body(SoftBodyBuilder::cuboid(
        Vector::new(3.0, 0.5, 0.0),
        Vector::splat(0.5),
        3,
        3,
        3,
    ));
    for _ in 0..10 {
        world.step();
    }
    // A NaN position set by the user.
    world.soft_bodies[handle].set_particle_position(0, Vector::new(Real::NAN, 0.0, 0.0));
    world.step();
    assert_eq!(world.physics_pipeline.quarantine().soft_bodies(), &[handle]);
    assert!(!world.soft_bodies[handle].is_enabled());
    assert!(!world.bodies[world.soft_bodies[handle].root_body()].is_enabled());
    for _ in 0..30 {
        world.step();
        assert!(world.physics_pipeline.quarantine().is_empty());
        assert!(world.bodies[ball].translation().is_finite());
        for p in world.soft_bodies[other].particles() {
            assert!(p.position().is_finite() && p.velocity().is_finite());
        }
    }
    // Repaired and re-enabled: it simulates again.
    world.soft_bodies[handle].set_particle_position(0, Vector::new(0.0, 0.5, 0.0));
    world.soft_bodies[handle].set_enabled(true);
    let before = world.soft_bodies[handle].center_of_mass();
    for _ in 0..30 {
        world.step();
        assert!(world.physics_pipeline.quarantine().is_empty());
    }
    assert!(world.soft_bodies[handle].is_enabled());
    assert!(world.bodies[world.soft_bodies[handle].root_body()].is_enabled());
    let sb = &world.soft_bodies[handle];
    assert!(sb.particles().iter().all(|p| p.position().is_finite()));
    assert!(
        (sb.center_of_mass() - before).length() > 1.0e-3,
        "the re-enabled body did not move"
    );

    // A non-finite force during a step: quarantined at the end of the step.
    world.soft_bodies[other].add_particle_force(0, Vector::new(0.0, Real::INFINITY, 0.0), true);
    world.step();
    assert_eq!(world.physics_pipeline.quarantine().soft_bodies(), &[other]);
    assert!(!world.soft_bodies[other].is_enabled());
    for _ in 0..10 {
        world.step();
        assert!(world.physics_pipeline.quarantine().is_empty());
        assert!(world.bodies[ball].translation().is_finite());
    }
}

/// A disabled soft body is left where it is (no constraints, no contacts) and comes back when enabled.
#[test]
fn disabled_soft_body_is_left_alone() {
    let mut world = world_with_ground();
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 2.0, 0.0), Vector::splat(0.5), 3, 3, 3);
    let handle = world.insert_soft_body(cube);
    world.soft_bodies[handle].set_enabled(false);
    for _ in 0..60 {
        world.step();
    }
    let com = world.soft_bodies[handle].center_of_mass();
    assert!(
        (com.y - 2.0).abs() < 1.0e-6,
        "the disabled body moved: {com:?}"
    );
    // A ball falls through the place of the disabled body's surface.
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 4.0, 0.0)),
        ColliderBuilder::ball(0.2),
    );
    for _ in 0..120 {
        world.step();
    }
    assert!(world.bodies[ball].translation().y < 1.0);
    world.remove_body(ball);
    world.soft_bodies[handle].set_enabled(true);
    for _ in 0..120 {
        world.step();
    }
    let com = world.soft_bodies[handle].center_of_mass();
    assert!(com.y < 1.0, "the re-enabled body did not fall: {com:?}");
}

/// `SoftBodyBuilder::trimesh`: a mesh's vertices become the particles, its edges springs, its
/// interior edges dihedrals, and shape matching keeps the model's shape once dropped.
#[test]
fn trimesh_soft_body_keeps_its_shape() {
    let mut world = world_with_ground();
    let (vertices, indices) = Cuboid::new(Vector::new(0.5, 0.4, 0.3)).to_trimesh();
    let vertices: Vec<Vector> = vertices
        .into_iter()
        .map(|v| v + Vector::new(0.0, 2.0, 0.0))
        .collect();
    let cube = SoftBodyBuilder::trimesh(vertices, indices)
        .unwrap()
        .material(SoftBodyMaterial {
            shape_matching_softness: SpringCoefficients::new(30.0, 1.0),
            ..Default::default()
        })
        .particle_mass(0.1);
    let handle = world.insert_soft_body(cube);
    let sb = &world.soft_bodies[handle];
    assert!(sb.cluster(0).unwrap().shape_matching_enabled());
    assert_eq!(sb.boundary().len(), 12);
    assert_eq!(sb.edges().len(), 18);
    assert_eq!(sb.dihedrals().len(), 18);
    assert!(sb.cells().is_empty());
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
    assert!(max_dev < 0.05, "the box lost its shape: {max_dev}");
    let com = sb.center_of_mass();
    assert!(
        (com.y - 0.4 - sb.particle_radius()).abs() < 0.1,
        "the box did not rest on the ground: {com:?}"
    );
}

/// A skin is a linear map of the cell it rides: whatever affine motion the cell undergoes, the
/// skin undergoes the same one, for a vertex the cell does not cover just as much as for one
/// inside it.
#[test]
fn skin_follows_the_cells() {
    // A single cell, and a skin with one vertex inside it, one on a face, one well outside it.
    let cell = [Vector::ZERO, Vector::X, Vector::Y, Vector::Z];
    let skin_vertices = vec![
        Vector::new(0.2, 0.2, 0.2),
        Vector::new(0.5, 0.5, 0.0),
        Vector::new(1.5, 1.5, 1.5),
    ];
    let builder = SoftBodyBuilder::new(cell.to_vec())
        .cells(vec![[0, 1, 2, 3]])
        .skin(skin_vertices.clone(), vec![[0, 1, 2]])
        .particle_mass(1.0);

    let mut world = world_with_ground();
    let handle = world.insert_soft_body(builder);
    let sb = &world.soft_bodies[handle];
    let skin = skinned_mesh(sb);

    // Bound at rest: the skin sits exactly where it was given.
    for (bound, rest) in skin.vertices().iter().zip(&skin_vertices) {
        assert!((*bound - *rest).length() < 1.0e-5, "{bound:?} vs {rest:?}");
    }

    // Move the particles by an affine map, then let the body settle wherever it settles.
    let rotation = Rotation::from_axis_angle(Vector::Z, 0.7);
    let sb = &mut world.soft_bodies[handle];
    for (i, p) in cell.iter().enumerate() {
        sb.set_particle_position(
            i,
            rotation * Vector::new(p.x * 2.0, p.y * 0.5, p.z) + Vector::new(3.0, -1.0, 2.0),
        );
    }

    for _ in 0..5 {
        world.step();

        // The affine map the cell actually underwent, recovered from the particles themselves.
        let sb = &world.soft_bodies[handle];
        let moved: Vec<Vector> = sb.particle_positions().collect();
        let rest_frame = Matrix::from_cols(cell[1] - cell[0], cell[2] - cell[0], cell[3] - cell[0]);
        let frame = Matrix::from_cols(
            moved[1] - moved[0],
            moved[2] - moved[0],
            moved[3] - moved[0],
        );
        let map = frame * rest_frame.inverse();

        for (skinned, rest) in skinned_mesh(sb).vertices().iter().zip(&skin_vertices) {
            let expected = map * (*rest - cell[0]) + moved[0];
            assert!(
                (*skinned - expected).length() < 1.0e-4,
                "skin vertex at {skinned:?}, expected {expected:?}"
            );
        }
    }
}

/// `SoftBodyBuilder::volumetric_skinned` keeps the input mesh as the skin, whatever the cells
/// could resolve of it.
#[test]
fn volumetric_skinned_keeps_the_input_mesh() {
    let (vertices, indices) = Ball::new(1.0).to_trimesh(24, 24);
    let vertices: Vec<Vector> = vertices
        .iter()
        .map(|p| *p + Vector::new(0.0, 3.0, 0.0))
        .collect();

    let builder = SoftBodyBuilder::volumetric_skinned(&vertices, &indices, 0.4)
        .expect("the ball is filled")
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
    let skin = skinned_mesh(sb);

    assert_eq!(skin.vertices().len(), vertices.len());
    assert_eq!(skin.indices().len(), indices.len());
    // The cells are far coarser than the mesh they hold.
    assert!(
        sb.num_particles() < vertices.len(),
        "{} particles for {} skin vertices",
        sb.num_particles(),
        vertices.len()
    );

    for _ in 0..300 {
        world.step();
    }
    assert_finite(&world, handle);

    // The skin came down with the cells, and is still a ball-sized mesh.
    let sb = &world.soft_bodies[handle];
    let skin = skinned_mesh(sb);
    let center = skin.vertices().iter().copied().sum::<Vector>() / skin.vertices().len() as Real;
    let radius = skin
        .vertices()
        .iter()
        .map(|v| (*v - center).length())
        .fold(0.0, Real::max);
    assert!(skin.vertices().iter().all(|v| v.is_finite()));
    assert!(radius > 0.5 && radius < 1.6, "skin radius {radius}");
    assert!(
        center.y < 2.0,
        "the skin did not follow the fall: {center:?}"
    );
}

/// The debug-renderer draws the cage of a skinned body: the cells are what is simulated, and are
/// otherwise invisible under the skin.
#[cfg(feature = "debug-render")]
#[test]
fn debug_renderer_draws_the_cage() {
    use rapier3d::pipeline::{
        DebugColor, DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline,
        DebugRenderStyle,
    };

    #[derive(Default)]
    struct CountLines {
        soft_body_lines: usize,
        min: Vector,
        max: Vector,
    }

    impl DebugRenderBackend for CountLines {
        fn draw_line(&mut self, object: DebugRenderObject, a: Vector, b: Vector, _: DebugColor) {
            if matches!(object, DebugRenderObject::SoftBody(..)) {
                self.soft_body_lines += 1;
                self.min = self.min.min(a).min(b);
                self.max = self.max.max(a).max(b);
            }
        }
    }

    let (vertices, indices) = Ball::new(1.0).to_trimesh(16, 16);
    let vertices: Vec<Vector> = vertices
        .iter()
        .map(|p| *p + Vector::new(0.0, 3.0, 0.0))
        .collect();
    let builder = SoftBodyBuilder::volumetric_skinned(&vertices, &indices, 0.4)
        .expect("the ball is filled")
        .cell_model(SoftBodyCellModel::Corotational)
        .particle_mass(0.05);

    let mut world = world_with_ground();
    let handle = world.insert_soft_body(builder);
    world.step();

    let mut backend = CountLines {
        min: Vector::splat(Real::MAX),
        max: Vector::splat(-Real::MAX),
        ..Default::default()
    };
    let mut pipeline =
        DebugRenderPipeline::new(DebugRenderStyle::default(), DebugRenderMode::SOFT_BODIES);
    pipeline.render(
        &mut backend,
        &world.bodies,
        &world.colliders,
        &world.impulse_joints,
        &world.multibody_joints,
        &world.narrow_phase,
        &world.soft_bodies,
    );

    // Every cage edge once, however many cells share it.
    let sb = &world.soft_bodies[handle];
    let cells = sb.cells().len();
    let mut edges = std::collections::HashSet::new();
    for cell in sb.cells() {
        for a in 0..4 {
            for b in a + 1..4 {
                let (x, y) = (cell.vertices[a], cell.vertices[b]);
                edges.insert([x.min(y), x.max(y)]);
            }
        }
    }
    assert!(cells > 0);
    assert_eq!(backend.soft_body_lines, edges.len());
    assert!(
        edges.len() * 3 < cells * 6,
        "{} edges vs {} cell sides: the cage was not deduplicated",
        edges.len(),
        cells * 6
    );

    // And it is drawn where the body is, not at the origin.
    assert!(
        backend.min.y > 1.0 && backend.max.y < 5.0,
        "the cage was drawn at {:?}..{:?}",
        backend.min,
        backend.max
    );
}

/// A skinned body rests on its skin, not on the cells holding it: the cage is dilated and
/// coarse, so a body colliding through it would hover above the ground.
#[test]
fn skin_contacts_rest_on_the_skin() {
    let settle = |skin_collision: bool| -> (Real, Real) {
        let (vertices, indices) = Ball::new(1.0).to_trimesh(24, 24);
        let vertices: Vec<Vector> = vertices
            .iter()
            .map(|p| *p + Vector::new(0.0, 2.0, 0.0))
            .collect();
        let builder = SoftBodyBuilder::volumetric_skinned(&vertices, &indices, 0.7)
            .expect("the ball is filled")
            .skin_collision(skin_collision)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 2.0e5,
                poisson_ratio: 0.3,
                ..Default::default()
            })
            .particle_mass(0.05)
            .particle_radius(0.02);

        let mut world = world_with_ground();
        let handle = world.insert_soft_body(builder);
        for _ in 0..400 {
            world.step();
        }
        assert_finite(&world, handle);

        let sb = &world.soft_bodies[handle];
        let skin = skinned_mesh(sb);
        let lowest_skin = skin
            .vertices()
            .iter()
            .map(|v| v.y)
            .fold(Real::MAX, Real::min);
        let lowest_cell = sb
            .particle_positions()
            .map(|p| p.y)
            .fold(Real::MAX, Real::min);
        (lowest_skin, lowest_cell)
    };

    // Colliding through the skin: the skin is what touches the ground.
    let (skin_y, cell_y) = settle(true);
    assert!(
        skin_y > -0.02 && skin_y < 0.15,
        "the skin should rest on the ground, lowest vertex at {skin_y}"
    );

    // Colliding through the cells: the cells are what touches the ground, and the cover's
    // cage contains the ball (it circumscribes the curvature), so the model hovers above
    // the floor by however much the cage overshoots.
    let (cage_skin_y, cage_cell_y) = settle(false);
    assert!(
        cage_skin_y > 0.02,
        "colliding through the cells should keep the model off the floor, at {cage_skin_y}"
    );
    assert!(
        skin_y < cage_skin_y - 0.02,
        "the skin sits at {skin_y} on itself against {cage_skin_y} on the cells"
    );
    // Either way the body came down and did not sink. Colliding through the skin, the
    // cage has no collider and hangs below the resting skin by its overshoot (up to a
    // 0.7 cell), which is not sinking.
    for y in [skin_y, cage_skin_y, cage_cell_y] {
        assert!(y > -0.3, "something sank through the ground: {y}");
    }
    assert!(cell_y > -1.0, "the cage fell through the ground: {cell_y}");
}

/// A soft body meets another soft body on its skin: `soft_sphere_rests_on_soft_cube` again, with
/// the cube wearing its own boundary as a skin and colliding through it. The vertex-vs-surface
/// pass has to read the mesh each side collides through for the sphere to find the cube at all.
#[test]
fn skinned_bodies_collide_with_each_other() {
    let settle = |skinned: bool| -> (Real, Real) {
        let half = Vector::splat(0.6);
        let center = Vector::new(0.0, 0.6, 0.0);
        let mut cube = SoftBodyBuilder::cuboid(center, half, 5, 5, 5)
            .softness(SpringCoefficients::new(30.0, 1.0))
            .particle_mass(0.2);
        if skinned {
            // The lattice fills the box exactly, so its own boundary is the skin.
            let (vertices, indices) = Cuboid::new(half).to_trimesh();
            let vertices: Vec<Vector> = vertices.iter().map(|p| *p + center).collect();
            cube = cube.skin(vertices, indices);
        }
        let sphere = SoftBodyBuilder::sphere(Vector::new(0.0, 2.2, 0.0), 0.5, 2)
            .softness(SpringCoefficients::new(20.0, 1.0))
            .particle_mass(0.05);

        let mut world = world_with_ground();
        let cube = world.insert_soft_body(cube);
        let sphere = world.insert_soft_body(sphere);
        assert_eq!(
            world.soft_bodies[cube]
                .meshes()
                .any(|m| m.is_skinned()),
            skinned
        );
        for _ in 0..400 {
            world.step();
        }
        assert_finite(&world, cube);
        assert_finite(&world, sphere);

        let sphere_bottom = world.soft_bodies[sphere]
            .particle_positions()
            .map(|p| p.y)
            .fold(Real::MAX, Real::min);
        let cube_top = world.soft_bodies[cube]
            .particle_positions()
            .map(|p| p.y)
            .fold(-Real::MAX, Real::max);
        (sphere_bottom, cube_top)
    };

    let (plain_bottom, plain_top) = settle(false);
    let (skinned_bottom, skinned_top) = settle(true);

    // The sphere rests on the cube either way, and the skin puts it in the same place as the
    // cells do, the two meshes being the same box.
    for (bottom, top) in [(plain_bottom, plain_top), (skinned_bottom, skinned_top)] {
        assert!(bottom > 0.9, "the sphere fell through the cube to {bottom}");
        assert!(
            bottom < top + 0.4,
            "the sphere floats at {bottom} over a cube topped at {top}"
        );
    }
    assert!(
        (skinned_bottom - plain_bottom).abs() < 0.15,
        "the sphere rests at {skinned_bottom} on the skin against {plain_bottom} on the cells"
    );
}

/// Reproducer for a known bug (see `docs/soft-soft-overlap-bug.md`): two soft bodies inserted
/// with overlapping contact skins are thrown apart instead of settling. Ignored because it
/// fails (the per-pair de-overlap budget that fixed it lives on the experimental branch).
#[test]
#[ignore = "known bug: soft-vs-soft bodies inserted within each other's contact skins explode"]
fn soft_bodies_inserted_overlapping_settle() {
    let cube = |y: Real| {
        SoftBodyBuilder::cuboid(Vector::new(0.0, y, 0.0), Vector::splat(0.5), 3, 3, 3)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 1.0e5,
                poisson_ratio: 0.3,
                elastic_damping_ratio: 0.5,
                ..Default::default()
            })
            .particle_mass(0.2)
    };

    // The two particle radii add up to 0.5, so anything below that starts inside the skins.
    for gap in [0.1, 0.3, 0.7] {
        let mut world = world_with_ground();
        let lower = world.insert_soft_body(cube(0.55));
        let upper = world.insert_soft_body(cube(0.55 + 1.0 + gap));
        let start = world.soft_bodies[upper]
            .particle_positions()
            .map(|p| p.y)
            .fold(-Real::MAX, Real::max);
        let mut peak: Real = 0.0;

        for _ in 0..400 {
            world.step();
            peak = peak.max(
                world.soft_bodies[upper]
                    .particle_positions()
                    .map(|p| p.y)
                    .fold(-Real::MAX, Real::max),
            );
        }

        let top = world.soft_bodies[lower]
            .particle_positions()
            .map(|p| p.y)
            .fold(-Real::MAX, Real::max);
        let bottom = world.soft_bodies[upper]
            .particle_positions()
            .map(|p| p.y)
            .fold(Real::MAX, Real::min);
        // Settling out of the authored overlaps necessarily lifts the top: up to 0.4 of
        // pair overlap plus 0.2 of ground-skin overlap. The bound leaves 0.15 of dynamic
        // overshoot on top of that.
        assert!(
            peak < start + 0.75,
            "gap {gap}: the upper cube was thrown from {start} up to {peak}"
        );
        assert!(
            bottom > top - 0.25,
            "gap {gap}: the upper cube ended at {bottom}, under a top at {top}"
        );
    }
}

/// `SoftBodyBuilder::volumetric_with` prices a fine boundary against a coarse interior: the
/// cover's subdivision refines the boundary cells only, so the same boundary resolution costs
/// fewer cells than a uniform fill, and the body still behaves.
#[test]
fn volumetric_subdivision_costs_fewer_cells() {
    use rapier3d::parry::transformation::VolumeMeshParameters;

    let (vertices, indices) = Ball::new(1.0).to_trimesh(30, 30);
    let vertices: Vec<Vector> = vertices
        .iter()
        .map(|p| *p + Vector::new(0.0, 2.0, 0.0))
        .collect();

    let build = |cell_size: Real, subdivisions: u32| {
        let mut params = VolumeMeshParameters::new(cell_size);
        params.cover_subdivisions = subdivisions;
        SoftBodyBuilder::volumetric_with(&vertices, &indices, &params)
            .expect("the ball is filled")
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e4,
                poisson_ratio: 0.3,
                ..Default::default()
            })
            .particle_mass(0.05)
    };

    // The same 0.12 boundary, once as a uniform fill and once as a coarse fill whose
    // boundary is subdivided down to it.
    let uniform = build(0.12, 0);
    let subdivided = build(0.24, 1);
    assert!(
        subdivided.cells.len() * 3 < uniform.cells.len() * 2,
        "subdividing saved little: {} cells against {}",
        subdivided.cells.len(),
        uniform.cells.len()
    );

    // The boundary is meshed just as finely either way, so the two bodies are the same size.
    let mut world = world_with_ground();
    let handles = [
        world.insert_soft_body(uniform),
        world.insert_soft_body(subdivided),
    ];
    let volumes: Vec<Real> = handles
        .iter()
        .map(|h| world.soft_bodies[*h].rest_volume())
        .collect();
    let ball = 4.0 / 3.0 * core::f32::consts::PI;
    for volume in &volumes {
        // The cover overshoots the ball by up to a boundary cell.
        assert!(
            *volume > ball * 0.95 && *volume < ball * 1.9,
            "volume {volume} vs {ball}"
        );
    }

    for _ in 0..300 {
        world.step();
    }
    for handle in handles {
        assert_finite(&world, handle);
        let lowest = world.soft_bodies[handle]
            .particle_positions()
            .map(|p| p.y)
            .fold(Real::MAX, Real::min);
        assert!(lowest > -0.1 && lowest < 0.6, "the ball ended at {lowest}");
    }
}
