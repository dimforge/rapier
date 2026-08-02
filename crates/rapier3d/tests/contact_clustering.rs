//! Contact clustering (`IntegrationParameters::contact_clustering`): manifolds of a
//! same pair with (nearly) parallel normals are merged into a single cluster manifold
//! before constraint generation. These tests pin the defining properties: a box on a
//! flat trimesh gets a single cluster (instead of one manifold per triangle), rests
//! as stably as without clustering, and the cluster impulses are warm-started across
//! steps.

use rapier3d::prelude::*;

struct World {
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd: CCDSolver,
    params: IntegrationParameters,
}

impl World {
    fn step(&mut self) {
        self.pipeline.step(
            Vector::new(0.0, -9.81, 0.0),
            &self.params,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            &(),
            &(),
        );
    }
}

/// A flat 4x4-quad trimesh floor in the xz plane plus one dynamic box dropped on it.
fn box_on_trimesh_floor(contact_clustering: bool) -> (World, RigidBodyHandle, ColliderHandle) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();

    const N: usize = 4;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for i in 0..=N {
        for j in 0..=N {
            vertices.push(Vector::new(
                i as Real - N as Real / 2.0,
                0.0,
                j as Real - N as Real / 2.0,
            ));
        }
    }
    for i in 0..N as u32 {
        for j in 0..N as u32 {
            let a = i * (N as u32 + 1) + j;
            let b = a + 1;
            let c = a + (N as u32 + 1);
            let d = c + 1;
            indices.push([a, b, c]);
            indices.push([b, d, c]);
        }
    }

    let floor = colliders.insert(
        ColliderBuilder::trimesh_with_flags(vertices, indices, TriMeshFlags::FIX_INTERNAL_EDGES)
            .unwrap(),
    );

    // A wide flat box: rests across several triangles, so the pair has multiple manifolds.
    // Sleeping is disabled so the solver equilibrium (contact impulses balancing
    // gravity) can be asserted at any step.
    let box_body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.3, 0.0))
            .can_sleep(false),
    );
    let box_co = colliders.insert_with_parent(
        ColliderBuilder::cuboid(1.2, 0.25, 1.2),
        box_body,
        &mut bodies,
    );

    let world = World {
        pipeline: PhysicsPipeline::new(),
        islands: IslandManager::new(),
        broad_phase: DefaultBroadPhase::new(),
        narrow_phase: NarrowPhase::new(),
        bodies,
        colliders,
        impulse_joints: ImpulseJointSet::new(),
        multibody_joints: MultibodyJointSet::new(),
        ccd: CCDSolver::new(),
        params: IntegrationParameters {
            contact_clustering,
            ..Default::default()
        },
    };

    let _ = floor;
    (world, box_body, box_co)
}

#[test]
fn box_rests_stably_on_trimesh_with_clustering() {
    let (mut world, box_body, box_co) = box_on_trimesh_floor(true);

    for _ in 0..200 {
        world.step();
    }

    let rb = &world.bodies[box_body];
    let pos = rb.translation();
    let allowed_error = 2.0 * IntegrationParameters::default().allowed_linear_error();

    // The box must rest on the floor (half-height 0.25), not sink or bounce away.
    assert!(
        (pos.y - 0.25).abs() < allowed_error + 0.01,
        "box rest height drifted: y = {}",
        pos.y
    );
    assert!(
        rb.linvel().length() < 1.0e-2,
        "box still moving at rest: |v| = {}",
        rb.linvel().length()
    );

    // Clustering must actually have kicked in: several per-triangle manifolds, but a
    // single flat contact plane, hence exactly one solver cluster with at most 4
    // solver contacts.
    let pair = world
        .narrow_phase
        .contact_pair(box_co, world.colliders.iter().next().unwrap().0)
        .expect("no contact pair between the box and the floor");
    assert!(
        pair.manifolds.len() > 1,
        "test setup must yield multiple per-triangle manifolds, got {}",
        pair.manifolds.len()
    );
    assert_eq!(pair.solver_clusters.len(), 1);
    let cluster = &pair.solver_clusters[0];
    assert!(!cluster.data.solver_contacts.is_empty());
    assert!(cluster.data.solver_contacts.len() <= 4);

    // The cluster is what the solver saw: its contacts hold the impulses that support
    // the box against gravity, and warm-starting must have carried them across steps.
    // Only the points selected as solver contacts hold the impulses of the last solve
    // (unselected points may keep stale values, like with parry's contact matching).
    let total_impulse: Real = cluster
        .data
        .solver_contacts
        .iter()
        .map(|sc| {
            cluster.points[sc.contact_indices()[0] as usize]
                .data
                .impulse
        })
        .sum();
    // The sum includes the soft-constraint bias share, so it is somewhat above the
    // pure weight support (an unclustered run yields the exact same total).
    let weight_dt = 9.81 * world.bodies[box_body].mass() * world.params.dt;
    assert!(
        total_impulse >= weight_dt * 0.9 && total_impulse <= weight_dt * 2.0,
        "cluster impulses don't support the box: {total_impulse} vs {weight_dt}"
    );
    assert!(
        cluster
            .points
            .iter()
            .any(|pt| pt.data.warmstart_impulse != 0.0)
    );

    // The plain manifolds are still exposed for queries, but hold no solver contacts.
    assert!(
        pair.manifolds
            .iter()
            .all(|m| m.data.solver_contacts.is_empty())
    );
    assert!(pair.manifolds.iter().any(|m| !m.points.is_empty()));
}

#[test]
fn clustering_matches_unclustered_rest_behavior() {
    let (mut with, box_a, _) = box_on_trimesh_floor(true);
    let (mut without, box_b, _) = box_on_trimesh_floor(false);

    for _ in 0..200 {
        with.step();
        without.step();
    }

    let pa = with.bodies[box_a].translation();
    let pb = without.bodies[box_b].translation();

    // Not bit-identical (different constraint sets), but both must settle at the same
    // place on the flat floor.
    assert!(
        (pa - pb).length() < 1.0e-2,
        "clustered and unclustered rest positions diverged: {pa:?} vs {pb:?}"
    );
}
