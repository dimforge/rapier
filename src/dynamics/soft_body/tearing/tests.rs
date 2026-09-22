//! Unit tests of the particle split core on tiny meshes.

use crate::alloc_prelude::*;
use crate::prelude::*;

use super::tearing_particle_split::{MeasureKind, SplitLog};

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
    let fan = sb.plane_fan(2, origin, normal).unwrap();
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
    let fan = sb.plane_fan(0, origin, Vector::new(1.0, -1.0)).unwrap();
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
    let fan = sb.plane_fan(0, origin, Vector::Z).unwrap();
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
    let fan = sb.plane_fan(0, origin, normal).unwrap();
    assert!(!sb.opens_without_confetti(0, &fan));
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
