//! Unit tests of the particle split core on tiny meshes.

use crate::alloc_prelude::*;
use crate::prelude::*;

use super::tearing_particle_split::{CellIndex, MeasureKind, SplitLog};

/// Incidence stays sorted and complete when a particle is copied more than once and the cells
/// around it move to different copies. Compare the lookup and bounded BFS to the scanning path.
#[test]
fn cell_index_matches_scans_after_splits() {
    #[cfg(feature = "dim2")]
    let builder = SoftBodyBuilder::grid(Vector::ZERO, Vector::splat(1.0), 5, 4);
    #[cfg(feature = "dim3")]
    let builder = SoftBodyBuilder::cuboid(Vector::ZERO, Vector::splat(1.0), 3, 3, 3);
    let (mut world, handle) = world_with(builder.particle_mass(0.5));
    let body = world.soft_bodies.get_mut(handle).unwrap();
    let mut scanned = body.clone();
    let mut scanned_log = SplitLog::default();
    let mut log = SplitLog {
        cell_index: CellIndex::new(body),
        ..SplitLog::default()
    };

    fn check(body: &SoftBody, index: &CellIndex) {
        for v in 0..body.num_particles() as u32 + 1 {
            let actual = body.fan(MeasureKind::Cells, v, Some(index));
            let expected = body.fan(MeasureKind::Cells, v, None);
            assert_eq!(actual.elements, expected.elements);
            assert_eq!(actual.vertices, expected.vertices);
            assert_eq!(actual.measures, expected.measures);
            assert_eq!(actual.groups, expected.groups);
            assert_eq!(actual.num_groups, expected.num_groups);
        }
        for (c, cell) in body.cells.iter().enumerate() {
            for &pivot in cell.vertices.iter().chain(core::iter::once(&u32::MAX)) {
                for limit in [
                    0,
                    1,
                    body.min_piece(MeasureKind::Cells),
                    body.cells.len() + 1,
                ] {
                    for seeds in [
                        &[][..],
                        &[c as u32][..],
                        &[c as u32, c as u32][..],
                        &[c as u32, ((c + 1) % body.cells.len()) as u32][..],
                    ] {
                        let actual = body.reachable_elements(
                            MeasureKind::Cells,
                            seeds,
                            pivot,
                            limit,
                            Some(index),
                        );
                        let expected =
                            body.reachable_elements(MeasureKind::Cells, seeds, pivot, limit, None);
                        assert_eq!(actual, expected);
                    }
                }
            }
            for particles in [cell.vertices.as_slice(), &cell.vertices[..2], &[][..]] {
                assert_eq!(
                    body.measure_element_holds_indexed(particles, Some(index)),
                    body.measure_element_holds(particles),
                );
            }
        }
        assert!(!body.measure_element_holds_indexed(&[u32::MAX], Some(index)));
    }

    check(body, log.cell_index.as_ref().unwrap());
    let original_particles = body.num_particles();
    for v in 0..original_particles as u32 {
        let mut fan = body.fan(MeasureKind::Cells, v, log.cell_index.as_ref());
        if fan.elements.len() < 2 {
            continue;
        }
        // Alternating sides also exercises splits producing more than two groups.
        for (i, side) in fan.sides.iter_mut().enumerate() {
            *side = i % 2;
        }
        fan.assign_groups(v);
        assert!(body.split_fan(v, &fan, &mut log));
        assert!(scanned.split_fan(v, &fan, &mut scanned_log));
        assert_eq!(log.split_particles, scanned_log.split_particles);
        assert_eq!(log.removed_edges, scanned_log.removed_edges);
        assert_eq!(
            std::format!(
                "{:?}",
                (&body.particles, &body.cells, &body.edges, &body.boundary)
            ),
            std::format!(
                "{:?}",
                (
                    &scanned.particles,
                    &scanned.cells,
                    &scanned.edges,
                    &scanned.boundary
                )
            ),
        );
        check(body, log.cell_index.as_ref().unwrap());
    }
    assert!(log.split_particles.len() > 2);
    body.remove_straddlers_across_pieces(&mut log);
    body.finish_topology_change(&log);
    body.validate_topology().unwrap();
}

/// The builder accepts cells with repeated vertices. Their facets are multisets, not the
/// simplices the incidence lookup requires, so they must retain the original scanning path.
#[test]
fn repeated_cell_vertices_keep_the_scanning_path() {
    #[cfg(feature = "dim2")]
    let (positions, cells) = (vec![Vector::ZERO, Vector::X], vec![[0, 1, 1]]);
    #[cfg(feature = "dim3")]
    let (positions, cells) = (vec![Vector::ZERO, Vector::X, Vector::Y], vec![[0, 1, 2, 2]]);
    let (world, handle) = world_with(
        SoftBodyBuilder::new(positions)
            .cells(cells)
            .cell_model(SoftBodyCellModel::Corotational)
            .no_surface_collider(),
    );
    let body = &world.soft_bodies[handle];
    assert!(CellIndex::new(body).is_none());
    assert_eq!(body.fan(MeasureKind::Cells, 1, None).elements, vec![0]);
}

/// A tear's scratch incidence must not become serialized state or survive into another tear.
#[cfg(feature = "serde-serialize")]
#[test]
fn cell_tearing_matches_after_snapshot_restore() {
    #[cfg(feature = "dim2")]
    let builder = SoftBodyBuilder::grid(Vector::ZERO, Vector::splat(1.0), 5, 4);
    #[cfg(feature = "dim3")]
    let builder = SoftBodyBuilder::cuboid(Vector::ZERO, Vector::splat(1.0), 3, 3, 3);
    let (mut world, handle) = world_with(builder.particle_mass(0.5));
    let body = world.soft_bodies.get_mut(handle).unwrap();
    let mut event = SoftBodyTearEvent::default();
    assert!(body.tear_topology(&[], &[body.cells.len() as u32 / 2], &mut event));
    let bytes = bincode::serialize(body).unwrap();
    let mut restored: SoftBody = bincode::deserialize(&bytes).unwrap();
    let cells: Vec<u32> = (0..body.cells.len() as u32).collect();
    for _ in 0..3 {
        let mut expected = SoftBodyTearEvent::default();
        let mut actual = SoftBodyTearEvent::default();
        assert_eq!(
            body.tear_topology(&[], &cells, &mut expected),
            restored.tear_topology(&[], &cells, &mut actual),
        );
        assert_eq!(std::format!("{actual:?}"), std::format!("{expected:?}"));
        assert_eq!(
            bincode::serialize(&restored).unwrap(),
            bincode::serialize(body).unwrap()
        );
        restored.validate_topology().unwrap();
    }
}

/// A repeatable public-API benchmark; timing assertions do not belong in the normal test suite.
#[cfg(feature = "std")]
#[test]
#[ignore]
fn cell_tearing_benchmark() {
    use std::format;
    use std::hash::{DefaultHasher, Hash, Hasher};
    use std::time::Instant;

    #[cfg(feature = "dim2")]
    let sizes = [(10, 7, 1), (25, 12, 1), (45, 18, 1)];
    #[cfg(feature = "dim3")]
    let sizes = [(3, 3, 3), (5, 4, 4), (7, 6, 5)];
    for (nx, ny, nz) in sizes {
        #[cfg(feature = "dim2")]
        let builder = SoftBodyBuilder::grid(Vector::ZERO, Vector::new(1.5, 0.6), nx, ny);
        #[cfg(feature = "dim3")]
        let builder = SoftBodyBuilder::cuboid(Vector::ZERO, Vector::splat(1.0), nx, ny, nz);
        let _ = nz;
        let builder = builder
            .particle_mass(0.004)
            .cell_model(SoftBodyCellModel::Corotational)
            .surface_collider(ColliderBuilder::ball(0.05));
        let (template, handle) = world_with(builder.clone());
        let body = &template.soft_bodies[handle];
        let total_cells = body.cells().len();
        for pattern in ["none", "single", "band", "all"] {
            let cells: Vec<u32> = body
                .cells()
                .iter()
                .enumerate()
                .filter(|(i, c)| {
                    let x = c
                        .vertices
                        .iter()
                        .map(|&v| body.particle_position(v as usize).x)
                        .sum::<Real>()
                        / (DIM + 1) as Real;
                    match pattern {
                        "none" => false,
                        "single" => *i == total_cells / 2,
                        "band" => x.abs() < 0.15,
                        _ => true,
                    }
                })
                .map(|(i, _)| i as u32)
                .collect();
            let mut samples = Vec::new();
            let mut fingerprint = None;
            let mut counts = (0, 0);
            for _ in 0..7 {
                let (mut world, handle) = world_with(builder.clone());
                let start = Instant::now();
                let event = world.tear_soft_body(handle, &[], &cells);
                samples.push(start.elapsed().as_secs_f64() * 1000.0);
                let bodies = family(&world, handle);
                counts = (
                    event.as_ref().map_or(0, |e| e.torn_cells.len()),
                    bodies.len(),
                );
                let mut hash = DefaultHasher::new();
                format!("{event:?}").hash(&mut hash);
                let mut mass = 0.0;
                let mut measure = 0.0;
                for &piece in &bodies {
                    let sb = &world.soft_bodies[piece];
                    sb.validate_topology().unwrap();
                    mass += sb.mass();
                    measure += sb.rest_measure();
                    format!("{:?}", (&sb.particles, &sb.cells, &sb.edges, &sb.boundary))
                        .hash(&mut hash);
                }
                let conserved =
                    |a: Real, b: Real| (a - b).abs() <= 1.0e-4 * a.abs().max(b.abs()).max(1.0);
                assert!(
                    conserved(mass, body.mass()),
                    "mass: {mass} vs {}",
                    body.mass()
                );
                assert!(
                    conserved(measure, body.rest_measure()),
                    "measure: {measure} vs {}",
                    body.rest_measure()
                );
                // Exercise the rebuilt colliders and solver after the tear, outside the timing.
                for _ in 0..3 {
                    world.step();
                }
                for &piece in &bodies {
                    let sb = &world.soft_bodies[piece];
                    for p in sb.particles() {
                        assert!(p.position.is_finite() && p.velocity.is_finite());
                    }
                    format!("{:?}", sb.particles()).hash(&mut hash);
                }
                let value = hash.finish();
                if let Some(expected) = fingerprint {
                    assert_eq!(value, expected, "non-deterministic benchmark fixture");
                }
                fingerprint = Some(value);
            }
            samples.sort_by(f64::total_cmp);
            std::println!(
                "TEARING dim={DIM} cells={total_cells} pattern={pattern} requested={} torn={} pieces={} median_ms={:.6} fingerprint={:016x}",
                cells.len(),
                counts.0,
                counts.1,
                samples[3],
                fingerprint.unwrap()
            );
        }
    }
}

/// Every structural edge of a rope, then every edge and cell of a grid (2D) or a box (3D), torn
/// over and over: the bodies come apart, but no tear splits off a piece smaller than the minimum
/// of its measure kind, and the mass and rest measure are kept.
#[test]
fn tears_never_split_off_a_chip() {
    let rope = SoftBodyBuilder::rope(Vector::ZERO, Vector::X * 3.0, 13).particle_mass(0.1);
    #[cfg(feature = "dim2")]
    let block = SoftBodyBuilder::grid(Vector::ZERO, Vector::splat(1.0), 7, 7).particle_mass(0.1);
    #[cfg(feature = "dim3")]
    let block =
        SoftBodyBuilder::cuboid(Vector::ZERO, Vector::splat(1.0), 4, 4, 4).particle_mass(0.1);
    for (builder, kind) in [(rope, MeasureKind::Segments), (block, MeasureKind::Cells)] {
        let (mut world, handle) = world_with(builder);
        let (mass, measure) = {
            let sb = &world.soft_bodies[handle];
            (sb.mass(), sb.rest_measure())
        };
        for _ in 0..4 {
            for body in family(&world, handle) {
                let sb = &world.soft_bodies[body];
                let edges: Vec<u32> = (0..sb.edges().len() as u32).collect();
                let cells: Vec<u32> = (0..sb.cells().len() as u32).collect();
                let _ = world.tear_soft_body(body, &edges, &cells);
            }
        }

        // Every piece is a body of its own now: the original and the ones split off it.
        let bodies = family(&world, handle);
        assert!(bodies.len() >= 2, "{kind:?}: nothing tore");
        let relative = |a: Real, b: Real| (a - b).abs() <= 1.0e-5 * a.abs().max(b.abs());
        let mut total_mass = 0.0;
        let mut total_measure = 0.0;
        for &body in &bodies {
            let sb = &world.soft_bodies[body];
            sb.validate_topology().unwrap();
            assert_eq!(sb.connected_pieces().len(), 1);
            assert_eq!(sb.origin(), (body != handle).then_some(handle));
            total_mass += sb.mass();
            total_measure += sb.rest_measure();
            let mut count = 0;
            sb.for_each_measure_element(kind, |_, _| count += 1);
            assert!(
                count >= sb.min_piece(kind),
                "{kind:?}: a tear split off a chip of {count} elements"
            );
        }
        assert!(relative(total_mass, mass), "{kind:?}: mass changed");
        assert!(
            relative(total_measure, measure),
            "{kind:?}: rest measure changed"
        );
    }
}

/// Checks the seeded components of a torn rope, of a cluster of it, and of a body made of
/// disconnected ropes: measure and mass add up, a never-connected cluster reports separate
/// components, and a seed pair reaches only the ropes it touches.
#[test]
fn seeded_components_follow_the_tear() {
    let rope = SoftBodyBuilder::rope(Vector::ZERO, Vector::X * 8.0, 9).particle_mass(0.5);
    let (mut world, handle) = world_with(rope);
    let (total_measure, total_mass) = {
        let sb = &world.soft_bodies[handle];
        (sb.rest_measure(), sb.mass())
    };
    let event = world
        .tear_soft_body(handle, &[4], &[])
        .expect("nothing tore");
    // The tear split the rope in two bodies of four segments; the event says which is which.
    assert_eq!(event.seeds(), vec![[9, 4]]);
    assert_eq!(event.pieces.len(), 2);
    assert_eq!(event.pieces[0].soft_body, handle);
    assert_eq!(event.pieces[0].particles, vec![0, 1, 2, 3, 4]);
    assert_eq!(event.pieces[1].particles, vec![5, 6, 7, 8, 9]);
    let mut measure = 0.0;
    let mut mass = 0.0;
    for piece in &event.pieces {
        let sb = &world.soft_bodies[piece.soft_body];
        assert_eq!(sb.connected_pieces().len(), 1);
        assert!(sb.seeded_components(None, &[[0, 1]]).is_empty());
        let all: Vec<u32> = (0..sb.num_particles() as u32).collect();
        assert!(close(sb.rest_measure_of(&all), sb.rest_measure()));
        assert!(close(sb.mass_of(&all), sb.mass()));
        measure += sb.rest_measure();
        mass += sb.mass();
    }
    assert!(close(measure, total_measure));
    assert!(close(mass, total_mass));

    let mut positions = Vec::new();
    let mut edges = Vec::new();
    for rope in 0..3u32 {
        for i in 0..3u32 {
            positions.push(Vector::X * (rope * 3 + i) as Real);
        }
        edges.push([rope * 3, rope * 3 + 1]);
        edges.push([rope * 3 + 1, rope * 3 + 2]);
    }
    let ropes = SoftBodyBuilder::new(positions)
        .edges(edges)
        .no_surface_collider();
    let (world, handle) = world_with(ropes);
    let sb = &world.soft_bodies[handle];
    // Restricted graphs: particles 0 and 2 share no element, 0, 1 and 2 chain through 1.
    assert_eq!(
        sb.seeded_components(Some(&[0, 2]), &[[0, 2]]),
        vec![vec![0], vec![2]]
    );
    assert!(sb.seeded_components(Some(&[0, 1, 2]), &[[0, 2]]).is_empty());
    assert_eq!(
        sb.seeded_components(None, &[[2, 3]]),
        vec![vec![0, 1, 2], vec![3, 4, 5]]
    );
    assert!(sb.seeded_components(None, &[[7, 8]]).is_empty());
}

/// `SoftBodyMaterial::min_piece` sets the smallest piece a tear may split off: a rope of five
/// particles refuses to tear at its middle segment (a two-segment piece on each side), and tears
/// there once the material allows two-segment pieces.
#[test]
fn min_piece_sets_the_smallest_piece_a_tear_leaves() {
    let rope = || SoftBodyBuilder::rope(Vector::ZERO, Vector::X * 4.0, 5).particle_mass(0.5);
    let (mut world, handle) = world_with(rope());
    assert!(world.tear_soft_body(handle, &[2], &[]).is_none());
    let (mut world, handle) = world_with(rope().min_piece(2));
    let event = world
        .tear_soft_body(handle, &[2], &[])
        .expect("nothing tore");
    assert_eq!(event.pieces.len(), 2);
    for body in event.bodies() {
        let sb = &world.soft_bodies[body];
        sb.validate_topology().unwrap();
        assert_eq!(sb.min_piece(MeasureKind::Segments), 2);
    }
}

/// Checks that a pinned particle is never split: a rope pinned at both ends tears at the free end
/// of its first segment, a segment between two pinned particles never tears, and a cut next to a
/// pinned end inserts its particles a fifth of the segment in.
#[test]
fn pinned_particles_are_never_split() {
    let rope = SoftBodyBuilder::rope(Vector::ZERO, Vector::X * 8.0, 9)
        .particle_mass(0.5)
        .min_piece(1)
        .pinned_particles([0, 8]);
    let (mut world, handle) = world_with(rope);
    let event = world
        .tear_soft_body(handle, &[0], &[])
        .expect("nothing tore");
    assert_eq!(event.split_particles, vec![(9, 1)]);
    assert_eq!(event.pieces.len(), 2);
    let stub = &world.soft_bodies[event.pieces[1].soft_body];
    assert_eq!(event.pieces[1].particles, vec![0, 9]);
    assert!(stub.particles()[0].is_pinned() && !stub.particles()[1].is_pinned());

    let both = SoftBodyBuilder::rope(Vector::ZERO, Vector::X * 2.0, 3)
        .min_piece(1)
        .pinned_particles([0, 1]);
    let (mut world, handle) = world_with(both);
    assert!(world.tear_soft_body(handle, &[0], &[]).is_none());
    assert_eq!(world.soft_bodies[handle].num_particles(), 3);

    let rope = SoftBodyBuilder::rope(Vector::ZERO, Vector::X * 8.0, 9).pinned_particles([0]);
    let (mut world, handle) = world_with(rope);
    #[cfg(feature = "dim2")]
    let blade = [Vector::new(0.05, -1.0), Vector::new(0.05, 1.0)];
    #[cfg(feature = "dim3")]
    let blade = [
        Vector::new(0.05, -1.0, -1.0),
        Vector::new(0.05, 2.0, -1.0),
        Vector::new(0.05, -1.0, 2.0),
    ];
    let event = world
        .cut_soft_body(handle, &blade)
        .expect("the blade crossed the rope");
    assert!(event.split_particles.is_empty());
    assert_eq!(event.inserted_particles.len(), 2);
    let stub = &world.soft_bodies[event.pieces[1].soft_body];
    assert_eq!(stub.num_particles(), 2);
    assert!(stub.particles()[0].is_pinned());
    assert!((stub.particle_position(1).x - 0.2).abs() < 1.0e-5);
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

/// A soft body inserted in a fresh world.
fn world_with(builder: SoftBodyBuilder) -> (PhysicsWorld, SoftBodyHandle) {
    let mut world = PhysicsWorld::new();
    let handle = world.insert_soft_body(builder);
    (world, handle)
}

/// The particle pairs of the body's edges, each sorted, in sorted order.
fn edge_pairs(sb: &SoftBody) -> Vec<[u32; 2]> {
    let mut pairs: Vec<[u32; 2]> = sb
        .edges()
        .iter()
        .map(|e| {
            [
                e.vertices[0].min(e.vertices[1]),
                e.vertices[0].max(e.vertices[1]),
            ]
        })
        .collect();
    pairs.sort_unstable();
    pairs
}

fn close(a: Real, b: Real) -> bool {
    (a - b).abs() < 1.0e-5
}

/// A chain of five particles with unequal segments, split at its middle particle across the
/// segment towards the longer side: that segment moves to the copy, the mass is shared by length,
/// the bending edge over the particle is removed and the chain comes apart in two pieces.
#[test]
fn split_chain_at_its_middle_particle() {
    let positions: Vec<Vector> = [0.0, 1.0, 2.0, 4.0, 5.0]
        .iter()
        .map(|&x| Vector::X * x)
        .collect();
    let segments = vec![[0, 1], [1, 2], [2, 3], [3, 4]];
    let builder = SoftBodyBuilder::new(positions)
        .edges(segments.clone())
        .bend_edges(vec![[0, 2], [1, 3], [2, 4]])
        .particle_mass(0.9);
    #[cfg(feature = "dim2")]
    let builder = builder.surface(segments);
    #[cfg(feature = "dim3")]
    let builder = builder.wire(segments);
    let (mut world, handle) = world_with(builder);
    let sb = world.soft_bodies.get_mut(handle).unwrap();
    let measure = sb.rest_measure();

    let origin = sb.particles[2].rest_position;
    let normal = sb.particles[3].rest_position - origin;
    let fan = sb.plane_fan(2, origin, normal, None).unwrap();
    assert_eq!(fan.elements, vec![1, 2]);
    assert_eq!(fan.groups, vec![0, 1]);
    let mut log = SplitLog::default();
    assert!(sb.split_fan(2, &fan, &mut log));
    sb.remove_straddlers_across_pieces(&mut log);
    sb.finish_topology_change(&log);

    assert_eq!(log.split_particles, vec![(5, 2)]);
    assert_eq!(log.removed_edges, vec![[1, 3]]);
    assert!(close(sb.particles[2].mass, 0.3) && close(sb.particles[5].mass, 0.6));
    assert_eq!(
        edge_pairs(sb),
        vec![[0, 1], [0, 2], [1, 2], [3, 4], [3, 5], [4, 5]]
    );
    assert_eq!(sb.connected_pieces(), vec![vec![0, 1, 2], vec![3, 4, 5]]);
    assert!(close(sb.rest_measure(), measure));
    #[cfg(feature = "dim2")]
    assert_eq!(sb.boundary(), &[[0, 1], [1, 2], [5, 3], [3, 4]]);
    #[cfg(feature = "dim3")]
    {
        // The wire segment follows its edge to a new vertex bound to the copy.
        let mesh = sb.meshes().next().unwrap();
        assert_eq!(mesh.vertex_count(), 6);
        assert_eq!(mesh.vertex(sb, 5), sb.particles[5].position);
        assert_eq!(mesh.indices()[2][..2], [5, 3]);
    }
    sb.validate_topology().unwrap();
}

/// A kite of two triangles split at a vertex of their shared diagonal: the diagonal opens (a
/// crack face per cell, oriented outward), the edge along it is duplicated and the mass is
/// shared by area.
#[cfg(feature = "dim2")]
#[test]
fn split_kite_across_its_diagonal() {
    let positions = vec![
        Vector::new(0.0, 0.0),
        Vector::new(0.0, 1.0),
        Vector::new(2.0, 0.0),
        Vector::new(1.0, 1.0),
    ];
    let builder = SoftBodyBuilder::new(positions)
        .cells(vec![[0, 2, 3], [0, 3, 1]])
        .particle_mass(0.9);
    let (mut world, handle) = world_with(builder);
    let sb = world.soft_bodies.get_mut(handle).unwrap();
    assert_eq!(sb.boundary(), &[[0, 2], [2, 3], [3, 1], [1, 0]]);
    let measure = sb.rest_measure();

    let origin = sb.particles[0].rest_position;
    let fan = sb
        .plane_fan(0, origin, Vector::new(1.0, -1.0), None)
        .unwrap();
    assert_eq!(fan.elements, vec![0, 1]);
    assert_eq!(fan.sides, vec![1, 0]);
    assert_eq!(fan.groups, vec![1, 0]);
    let mut log = SplitLog::default();
    assert!(sb.split_fan(0, &fan, &mut log));
    sb.remove_straddlers_across_pieces(&mut log);
    sb.finish_topology_change(&log);

    assert_eq!(log.split_particles, vec![(4, 0)]);
    assert!(log.removed_edges.is_empty());
    assert_eq!(sb.cells[0].vertices, [4, 2, 3]);
    assert_eq!(sb.cells[1].vertices, [0, 3, 1]);
    // The copy's triangle has twice the area of the original's.
    assert!(close(sb.particles[0].mass, 0.3) && close(sb.particles[4].mass, 0.6));
    assert_eq!(
        edge_pairs(sb),
        vec![[0, 1], [0, 3], [1, 3], [2, 3], [2, 4], [3, 4]]
    );
    assert_eq!(
        sb.boundary(),
        &[[4, 2], [2, 3], [3, 1], [1, 0], [3, 4], [0, 3]]
    );
    let rest = |i: u32| sb.particles[i as usize].rest_position;
    for (face, cell) in [([3, 4], 0), ([0, 3], 1)] {
        let centroid: Vector = sb.cells[cell]
            .vertices
            .iter()
            .map(|&v| rest(v))
            .sum::<Vector>()
            / 3.0;
        let (a, b) = (rest(face[0]), rest(face[1]));
        assert!(
            (b - a).perp_dot(centroid - a) > 0.0,
            "crack face {face:?} is not outward"
        );
    }
    assert_eq!(sb.connected_pieces().len(), 1);
    assert!(close(sb.rest_measure(), measure));
    sb.validate_topology().unwrap();
}

/// Two tetrahedra sharing a face, split at a vertex of that face along its plane: the face opens
/// (a crack face per cell, oriented outward), the edges along it are duplicated and the mass is
/// shared by volume.
#[cfg(feature = "dim3")]
#[test]
fn split_tetrahedra_across_their_shared_face() {
    let positions = vec![
        Vector::new(0.0, 0.0, 0.0),
        Vector::new(1.0, 0.0, 0.0),
        Vector::new(0.0, 1.0, 0.0),
        Vector::new(0.25, 0.25, 1.0),
        Vector::new(0.25, 0.25, -2.0),
    ];
    let builder = SoftBodyBuilder::new(positions)
        .cells(vec![[0, 1, 2, 3], [0, 1, 2, 4]])
        .particle_mass(0.9);
    let (mut world, handle) = world_with(builder);
    let sb = world.soft_bodies.get_mut(handle).unwrap();
    assert_eq!(sb.cells[1].vertices, [0, 2, 1, 4]);
    assert_eq!(sb.boundary().len(), 6);
    let measure = sb.rest_measure();

    let origin = sb.particles[0].rest_position;
    let fan = sb.plane_fan(0, origin, Vector::Z, None).unwrap();
    assert_eq!(fan.sides, vec![1, 0]);
    assert_eq!(fan.groups, vec![1, 0]);
    let mut log = SplitLog::default();
    assert!(sb.split_fan(0, &fan, &mut log));
    sb.remove_straddlers_across_pieces(&mut log);
    sb.finish_topology_change(&log);

    assert_eq!(log.split_particles, vec![(5, 0)]);
    assert!(log.removed_edges.is_empty());
    assert_eq!(sb.cells[0].vertices, [5, 1, 2, 3]);
    assert_eq!(sb.cells[1].vertices, [0, 2, 1, 4]);
    // The original keeps the lower tetrahedron, twice the volume of the upper one.
    assert!(close(sb.particles[0].mass, 0.6) && close(sb.particles[5].mass, 0.3));
    assert_eq!(
        edge_pairs(sb),
        vec![
            [0, 1],
            [0, 2],
            [0, 4],
            [1, 2],
            [1, 3],
            [1, 4],
            [1, 5],
            [2, 3],
            [2, 4],
            [2, 5],
            [3, 5]
        ]
    );
    assert_eq!(sb.boundary().len(), 8);
    let rest = |i: u32| sb.particles[i as usize].rest_position;
    for (k, cell) in [(6, 0), (7, 1)] {
        let face = sb.boundary()[k];
        let mut sorted = face;
        sorted.sort_unstable();
        assert_eq!(sorted, if cell == 0 { [1, 2, 5] } else { [0, 1, 2] });
        let centroid: Vector = sb.cells[cell]
            .vertices
            .iter()
            .map(|&v| rest(v))
            .sum::<Vector>()
            / 4.0;
        let (a, b, c) = (rest(face[0]), rest(face[1]), rest(face[2]));
        assert!(
            (b - a).cross(c - a).dot(centroid - a) < 0.0,
            "crack face {face:?} is not outward"
        );
    }
    assert_eq!(sb.connected_pieces().len(), 1);
    assert!(close(sb.rest_measure(), measure));
    sb.validate_topology().unwrap();
}

/// A cloth quad split at a corner of its diagonal: the diagonal opens between the two triangles,
/// the spring along it is duplicated, the other diagonal (now across an opened side) is removed
/// and the mass is shared by area. The no-confetti rule refuses the same split.
#[cfg(feature = "dim3")]
#[test]
fn split_cloth_quad_across_its_diagonal() {
    let builder =
        SoftBodyBuilder::cloth(Vector::ZERO, Vector::X, Vector::Z, 2, 2).particle_mass(0.8);
    let (mut world, handle) = world_with(builder);
    let sb = world.soft_bodies.get_mut(handle).unwrap();
    assert_eq!(sb.boundary(), &[[0, 2, 3], [0, 3, 1]]);
    assert_eq!(
        edge_pairs(sb),
        vec![[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]]
    );
    let measure = sb.rest_measure();

    let origin = sb.particles[0].rest_position;
    let normal = Vector::new(1.0, 0.0, -1.0);
    let fan = sb.plane_fan(0, origin, normal, None).unwrap();
    assert!(!sb.opens_without_confetti(0, &fan, None));
    assert_eq!(fan.sides, vec![1, 0]);
    assert_eq!(fan.groups, vec![1, 0]);
    let mut log = SplitLog::default();
    assert!(sb.split_fan(0, &fan, &mut log));
    sb.remove_straddlers_across_pieces(&mut log);
    sb.finish_topology_change(&log);

    assert_eq!(log.split_particles, vec![(4, 0)]);
    assert_eq!(log.removed_edges, vec![[2, 1]]);
    assert_eq!(sb.boundary(), &[[4, 2, 3], [0, 3, 1]]);
    assert!(close(sb.particles[0].mass, 0.4) && close(sb.particles[4].mass, 0.4));
    assert_eq!(
        edge_pairs(sb),
        vec![[0, 1], [0, 3], [1, 3], [2, 3], [2, 4], [3, 4]]
    );
    assert_eq!(sb.connected_pieces().len(), 1);
    assert!(close(sb.rest_measure(), measure));
    sb.validate_topology().unwrap();
}
