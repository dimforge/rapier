//! Soft-body clusters (2D): how they come apart when a tear runs through them.

use rapier2d::prelude::*;

/// Checks that a chain torn at a one-particle cluster splits the cluster with it: the copy keeps
/// the proxy, the joint follows the source at the same world anchor, and each chain piece gets
/// its own body, mesh and collider.
#[test]
fn torn_cluster_splits_and_its_joint_follows_the_source() {
    let xs = [0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 7.0, 8.0, 9.0];
    let positions: Vec<Vector> = xs.iter().map(|&x| Vector::X * x).collect();
    let segments: Vec<[u32; 2]> = (0..8).map(|i| [i, i + 1]).collect();
    let builder = SoftBodyBuilder::new(positions)
        .edges(segments.clone())
        .particle_mass(0.3)
        .pinned_particles([0]);
    let builder = builder.surface(segments);
    let mut world = PhysicsWorld::new();
    let h = world.insert_soft_body(builder);
    let cluster = world.add_soft_body_cluster(h, &[4]).unwrap();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    let anchor =
        world.insert_body(RigidBodyBuilder::kinematic_position_based().translation(Vector::X * 4.0));
    let joint = world.insert_impulse_joint(
        anchor,
        proxy,
        GenericJointBuilder::new(JointAxesMask::LIN_AXES).build(),
    );

    // The chain tears at particle 4 (the endpoints tie, the smaller index goes first), then
    // comes apart: the right piece is the longer one and keeps the handle.
    let event = world.tear_soft_body(h, &[4], &[]).expect("nothing tore");
    assert_eq!(event.split_particles, vec![(9, 4)]);
    assert_eq!(event.pieces.len(), 2);
    let right = &event.pieces[0];
    let left = &event.pieces[1];
    assert_eq!(right.soft_body, h);
    assert_eq!(right.particles, vec![5, 6, 7, 8, 9]);
    assert_eq!(left.particles, vec![0, 1, 2, 3, 4]);
    let lh = left.soft_body;
    assert_eq!(world.soft_bodies[lh].origin(), Some(h));
    assert_eq!(world.soft_bodies[h].pieces(), &[lh]);
    assert!(world.soft_bodies[h].origin().is_none());

    // The grab cluster split with the particle: the copy (particle 4 of the right body after
    // compaction) is the heavier and kept the proxy, the source (particle 4 of the left body)
    // got a fresh one.
    let splits: Vec<_> = event
        .clusters
        .iter()
        .filter(|c| c.source_cluster == cluster)
        .collect();
    assert_eq!(splits.len(), 2, "{:?}", event.clusters);
    let kept = splits.iter().find(|c| c.keeps_proxy).unwrap();
    let fresh = splits.iter().find(|c| !c.keeps_proxy).unwrap();
    assert_eq!((kept.soft_body, kept.cluster, kept.proxy), (h, cluster, proxy));
    assert_eq!(fresh.soft_body, lh);
    let (rb, lb) = (&world.soft_bodies[h], &world.soft_bodies[lh]);
    assert_eq!(rb.cluster(kept.cluster).unwrap().particles(), &[4]);
    assert_eq!(lb.cluster(fresh.cluster).unwrap().particles(), &[4]);
    assert_eq!(world.bodies[fresh.proxy].soft_body(), Some(lh));
    assert_eq!(world.bodies[fresh.proxy].soft_cluster(), Some(fresh.cluster));
    assert!(rb.cluster(kept.cluster).unwrap().meshes().next().is_none());
    // The left body took the fresh pieces of both clusters (made in cluster order).
    assert_eq!(left.clusters, vec![[2, 0], [3, 1]]);
    assert_eq!(fresh.cluster, 1);

    // The joint sits on the split particle, at the same rest distance from both pieces: the
    // source's wins the tie, and the anchor stays where the particle is.
    assert_eq!(event.moved_joints.len(), 1);
    let moved = event.moved_joints[0];
    assert_eq!((moved.joint, moved.from, moved.to), (joint, proxy, fresh.proxy));
    let j = world.impulse_joints.get(joint).unwrap();
    assert_eq!((j.body1(), j.body2()), (anchor, fresh.proxy));
    let world_anchor = *world.bodies[fresh.proxy].position() * j.data.local_frame2.translation;
    assert!((world_anchor - lb.particle_position(4)).length() < 1.0e-4);

    // Each body has the whole-body cluster of its piece (the left one's was cluster 2 of the
    // torn body, the crack's fresh piece of cluster 0), with its own collision mesh.
    for (body, piece, source) in [(h, right, 0), (lh, left, 2)] {
        let sb = &world.soft_bodies[body];
        assert_eq!(sb.num_particles(), 5);
        assert_eq!(sb.connected_pieces().len(), 1);
        let (ci, c) = sb.live_clusters().next().unwrap();
        assert_eq!(c.proxy(), sb.root_body());
        assert_eq!(c.particles(), &[0, 1, 2, 3, 4]);
        assert_eq!(world.bodies[c.proxy()].soft_body(), Some(body));
        assert_eq!(world.bodies[c.proxy()].soft_cluster(), Some(ci));
        assert!(piece.clusters.contains(&[source, ci]));
        let mesh = c.meshes().next().expect("the piece lost its mesh");
        assert_eq!(mesh.vertex_count(), 5);
        assert_eq!(world.colliders[mesh.collider()].parent(), Some(c.proxy()));
        assert_eq!(
            world.colliders[mesh.collider()]
                .deformable_mesh_ref()
                .map(|r| (r.body, r.id)),
            Some((body, mesh.id()))
        );
        sb.validate_topology().unwrap();
    }
    assert!(lb.particles()[0].is_pinned() && !rb.particles()[0].is_pinned());

    for _ in 0..60 {
        world.step();
    }
    for body in [h, lh] {
        let sb = &world.soft_bodies[body];
        assert!(sb.particles().iter().all(|p| p.position().is_finite()));
    }
    // The pinned piece hangs from its pin, the joint holds it to the anchor; the right piece
    // fell away.
    let (rb, lb) = (&world.soft_bodies[h], &world.soft_bodies[lh]);
    assert!((lb.particle_position(4) - Vector::X * 4.0).length() < 0.5);
    assert!(rb.particle_position(0).y < -2.0);
}

/// A cluster whose two particles are joined only through particles outside it is one piece as
/// long as those links hold: a tear elsewhere leaves it alone, a tear through a link splits it,
/// the particle the tear never reached staying with the retained piece.
#[test]
fn cluster_joined_outside_itself_splits_only_through_its_link() {
    let mut world = PhysicsWorld::new();
    let rope = SoftBodyBuilder::rope(Vector::ZERO, Vector::X * 12.0, 13)
        .particle_mass(0.1)
        .pinned_particles([0]);
    let h = world.insert_soft_body(rope);
    let cluster = world.add_soft_body_cluster(h, &[3, 7]).unwrap();

    // Far from the cluster: the body splits (the piece past the tear falls away as a new body,
    // the cluster's piece keeps the handle), the cluster does not.
    let event = world.tear_soft_body(h, &[8], &[]).expect("nothing tore");
    assert_eq!(event.split_particles, vec![(13, 8)]);
    assert!(event.clusters.iter().all(|c| c.source_cluster == 0));
    assert_eq!(event.clusters.len(), 2);
    assert!(event.moved_joints.is_empty());
    assert_eq!(event.pieces.len(), 2);
    assert_eq!(event.pieces[0].soft_body, h);
    assert_eq!(event.pieces[0].particles, (0..9).collect::<Vec<u32>>());
    assert_eq!(event.pieces[0].clusters, vec![[0, 0], [cluster, cluster]]);
    assert_eq!(world.soft_bodies[h].num_particles(), 9);
    assert_eq!(
        world.soft_bodies[h].cluster(cluster).unwrap().particles(),
        &[3, 7]
    );

    // Through the link: particle 3 splits, its copy 9 takes the segment towards 4, and the
    // three pinned segments come apart from the rest as a new body.
    let event = world.tear_soft_body(h, &[3], &[]).expect("nothing tore");
    assert_eq!(event.split_particles, vec![(9, 3)]);
    assert_eq!(event.pieces.len(), 2);
    assert_eq!(event.pieces[0].particles, vec![4, 5, 6, 7, 8, 9]);
    assert_eq!(event.pieces[1].particles, vec![0, 1, 2, 3]);
    let tail = event.pieces[1].soft_body;
    let splits: Vec<_> = event
        .clusters
        .iter()
        .filter(|c| c.source_cluster == cluster)
        .collect();
    // The crack split the cluster into the source's piece (with 7, unreached by the tear) and the
    // copy's; the body split then parted 3 from 7, the heavier 7 keeping the proxy: three pieces.
    assert_eq!(splits.len(), 3, "{:?}", event.clusters);
    let kept = splits.iter().find(|c| c.keeps_proxy).unwrap();
    assert_eq!((kept.soft_body, kept.cluster), (h, cluster));
    let sb = &world.soft_bodies[h];
    let tail_body = &world.soft_bodies[tail];
    assert_eq!(sb.cluster(cluster).unwrap().particles(), &[3]);
    for fresh in splits.iter().filter(|c| !c.keeps_proxy) {
        let body = &world.soft_bodies[fresh.soft_body];
        let particles = body.cluster(fresh.cluster).unwrap().particles();
        assert_eq!(particles, if fresh.soft_body == h { &[5] } else { &[3] });
        assert_eq!(world.bodies[fresh.proxy].soft_body(), Some(fresh.soft_body));
    }
    assert_eq!(tail_body.num_live_clusters(), 2);
    sb.validate_topology().unwrap();
    tail_body.validate_topology().unwrap();
    for _ in 0..30 {
        world.step();
    }
    for body in [h, tail] {
        let sb = &world.soft_bodies[body];
        assert!(sb.particles().iter().all(|p| p.position().is_finite()));
    }
}
