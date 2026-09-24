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
    let anchor = world
        .insert_body(RigidBodyBuilder::kinematic_position_based().translation(Vector::X * 4.0));
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
    assert_eq!(
        (kept.soft_body, kept.cluster, kept.proxy),
        (h, cluster, proxy)
    );
    assert_eq!(fresh.soft_body, lh);
    let (rb, lb) = (&world.soft_bodies[h], &world.soft_bodies[lh]);
    assert_eq!(rb.cluster(kept.cluster).unwrap().particles(), &[4]);
    assert_eq!(lb.cluster(fresh.cluster).unwrap().particles(), &[4]);
    assert_eq!(world.bodies[fresh.proxy].soft_body(), Some(lh));
    assert_eq!(
        world.bodies[fresh.proxy].soft_cluster(),
        Some(fresh.cluster)
    );
    assert!(rb.cluster(kept.cluster).unwrap().meshes().next().is_none());
    // The left body took the fresh pieces of both clusters (made in cluster order).
    assert_eq!(left.clusters, vec![[2, 0], [3, 1]]);
    assert_eq!(fresh.cluster, 1);

    // The joint sits on the split particle, at the same rest distance from both pieces: the
    // source's wins the tie, and the anchor stays where the particle is.
    assert_eq!(event.moved_joints.len(), 1);
    let moved = event.moved_joints[0];
    assert_eq!(
        (moved.joint, moved.from, moved.to),
        (joint, proxy, fresh.proxy)
    );
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

/// A soft square spun in place (no gravity, no linear motion) turns its whole-body proxy with it
/// in both directions: the proxy's dead zone must not swallow a clockwise rotation change.
#[test]
fn proxy_follows_a_soft_body_spinning_in_place() {
    for spin in [-1.0, 1.0] {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        let square = SoftBodyBuilder::grid(Vector::ZERO, Vector::splat(0.5), 5, 5)
            .material(SoftBodyMaterial {
                young_modulus: 1.0e6,
                ..Default::default()
            })
            .particle_mass(0.1);
        let h = world.insert_soft_body(square);
        let sb = &mut world.soft_bodies[h];
        let corner = (0..sb.num_particles())
            .max_by(|&a, &b| {
                let (pa, pb) = (sb.particle_position(a), sb.particle_position(b));
                (pa.x + pa.y).total_cmp(&(pb.x + pb.y))
            })
            .unwrap();
        let arm0 = sb.particle_position(corner) - sb.center_of_mass();
        for i in 0..sb.num_particles() {
            let arm = sb.particle_position(i);
            sb.set_particle_velocity(i, Vector::new(-arm.y, arm.x) * spin);
        }
        let proxy = world.soft_bodies[h].root_body();
        let rot0 = *world.bodies[proxy].rotation();

        for _ in 0..10 {
            world.step();
        }

        let sb = &world.soft_bodies[h];
        let arm = sb.particle_position(corner) - sb.center_of_mass();
        let particles_angle = arm0.perp_dot(arm).atan2(arm0.dot(arm));
        let proxy_angle = (rot0.inverse() * *world.bodies[proxy].rotation()).angle();
        assert!(
            particles_angle * spin > 0.1,
            "spin {spin}: the square did not turn: {particles_angle}"
        );
        assert!(
            (proxy_angle - particles_angle).abs() < 0.01,
            "spin {spin}: proxy turned by {proxy_angle}, particles by {particles_angle}"
        );
    }
}

/// Asserts that a collider sits at its parent proxy's pose composed with its local pose, and that
/// the broad phase has a leaf covering it there.
fn assert_rides_its_proxy(world: &PhysicsWorld, co: ColliderHandle, step: usize) {
    let collider = &world.colliders[co];
    let proxy = &world.bodies[collider.parent().unwrap()];
    let expected = proxy.position() * collider.position_wrt_parent().unwrap();
    let actual = collider.position();
    let dt = (actual.translation - expected.translation).length();
    // Compares rotated axes: an `acos`-based angle is too noisy near zero in single precision.
    let dr = [Vector::X, Vector::Y]
        .map(|axis| (actual.rotation * axis - expected.rotation * axis).length())
        .into_iter()
        .fold(0.0, Real::max);
    assert!(
        dt < 1.0e-5 && dr < 1.0e-5,
        "step {step}: collider {co:?} lags its proxy (translation error {dt}, rotation error {dr})"
    );
    assert!(
        world
            .intersect_aabb_conservative(collider.compute_aabb(), QueryFilter::default())
            .any(|(handle, _)| handle == co),
        "step {step}: the broad phase has no leaf covering collider {co:?}"
    );
}

/// Rigid colliders hung with an offset on the whole-body proxy and on a sub-cluster proxy of a
/// falling, spinning soft square are at their proxy's pose at the end of every step, not one step
/// behind.
#[test]
fn rigid_colliders_on_proxies_do_not_lag_their_proxy() {
    let mut world = PhysicsWorld::new();
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::splat(0.5), 5, 5)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e5,
            ..Default::default()
        })
        .particle_mass(0.1);
    let h = world.insert_soft_body(square);
    let top: Vec<u32> = world.soft_bodies[h]
        .particles()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.position().y > 1.2)
        .map(|(i, _)| i as u32)
        .collect();
    assert!(!top.is_empty());
    let cluster = world
        .soft_bodies
        .add_cluster(h, &top, &mut world.bodies, &mut world.colliders)
        .unwrap();
    let sb = &mut world.soft_bodies[h];
    let com = sb.center_of_mass();
    for i in 0..sb.num_particles() {
        let arm = sb.particle_position(i) - com;
        sb.set_particle_velocity(i, Vector::new(1.0, 0.5) + Vector::new(-arm.y, arm.x) * 2.0);
    }
    let root = world.soft_bodies[h].root_body();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    let offset = Pose::from_parts(Vector::new(0.3, -0.2), Rotation::new(0.4));
    let colliders: Vec<ColliderHandle> = [root, proxy]
        .into_iter()
        .map(|parent| {
            world.colliders.insert_with_parent(
                ColliderBuilder::cuboid(0.05, 0.05)
                    .density(0.1)
                    .position(offset),
                parent,
                &mut world.bodies,
            )
        })
        .collect();

    for step in 0..30 {
        world.step();
        for co in &colliders {
            assert_rides_its_proxy(&world, *co, step);
        }
    }
    // The proxies did move, so a lagging collider could not have passed.
    assert!(world.bodies[root].translation().y < 0.5);
}

/// Asserts that the broad-phase leaf of a collider contains the collider's current AABB: every
/// corner of that AABB hits the leaf.
fn assert_leaf_contains_current_aabb(world: &PhysicsWorld, co: ColliderHandle, step: usize) {
    let aabb = world.colliders[co].compute_aabb();
    for i in 0..4 {
        let pick = |bit: usize, min: Real, max: Real| if i & bit == 0 { min } else { max };
        let corner = Vector::new(
            pick(1, aabb.mins.x, aabb.maxs.x),
            pick(2, aabb.mins.y, aabb.maxs.y),
        );
        assert!(
            world
                .intersect_aabb_conservative(Aabb::new(corner, corner), QueryFilter::default())
                .any(|(handle, _)| handle == co),
            "step {step}: the broad-phase leaf of collider {co:?} misses the corner {corner:?} of \
             its current AABB"
        );
    }
}

/// The surface collider of a soft square falling faster at each step (the large timestep makes
/// gravity outrun the speculative margin) has a broad-phase leaf around its end-of-step geometry,
/// so a ray cast between steps hits its current bottom edge.
#[test]
fn deformable_collider_leaves_follow_their_surface_at_the_end_of_each_step() {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = 0.1;
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::splat(0.5), 5, 5)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.01);
    let h = world.insert_soft_body(square);
    let surfaces: Vec<ColliderHandle> = world.soft_bodies[h]
        .meshes()
        .map(|mesh| mesh.collider())
        .collect();
    assert!(!surfaces.is_empty());

    for step in 0..20 {
        world.step();
        for co in &surfaces {
            assert_leaf_contains_current_aabb(&world, *co, step);
            let aabb = world.colliders[*co].compute_aabb();
            // Off the surface's vertices, which a ray may graze.
            let ray = Ray::new(
                Vector::new(aabb.center().x + 0.13, aabb.mins.y - 0.01),
                Vector::Y,
            );
            let hit = world.cast_ray(&ray, 0.02, true, QueryFilter::default());
            assert_eq!(
                hit.map(|(handle, _)| handle),
                Some(*co),
                "step {step}: a ray cast misses the bottom edge of collider {co:?}"
            );
        }
    }
    // The square did fall, so a stale leaf could not have passed.
    let root = world.soft_bodies[h].root_body();
    assert!(world.bodies[root].translation().y < -10.0);
}

/// A stiff soft square, with a sub-cluster on its top rows.
fn square_with_a_top_cluster(world: &mut PhysicsWorld) -> (SoftBodyHandle, u32) {
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::splat(0.5), 5, 5)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e5,
            ..Default::default()
        })
        .particle_mass(0.1);
    let h = world.insert_soft_body(square);
    let top: Vec<u32> = world.soft_bodies[h]
        .particles()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.position().y > 1.2)
        .map(|(i, _)| i as u32)
        .collect();
    assert!(!top.is_empty());
    let cluster = world
        .soft_bodies
        .add_cluster(h, &top, &mut world.bodies, &mut world.colliders)
        .unwrap();
    (h, cluster)
}

/// Whether the broad-phase leaf of a collider reaches the point `p`.
fn leaf_reaches(world: &PhysicsWorld, co: ColliderHandle, p: Vector) -> bool {
    world
        .intersect_aabb_conservative(Aabb::new(p, p), QueryFilter::default())
        .any(|(handle, _)| handle == co)
}

/// The points `dist` past the middle of each edge of a collider's current AABB.
fn points_past_aabb(world: &PhysicsWorld, co: ColliderHandle, dist: Real) -> Vec<Vector> {
    let aabb = world.colliders[co].compute_aabb();
    let (center, half) = (aabb.center(), aabb.half_extents());
    (0..2)
        .flat_map(|i| {
            [-1.0, 1.0].map(|sign| {
                let mut p = center;
                p[i] += sign * (half[i] + dist);
                p
            })
        })
        .collect()
}

/// The soft-body motion margin pads the deformable colliders of a fast soft square only: the
/// rigid colliders on its proxies get the broad-phase AABB of a collider on a dynamic body.
#[test]
fn only_deformable_colliders_are_padded_by_the_soft_motion_margin() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let (h, cluster) = square_with_a_top_cluster(&mut world);
    // A motion margin of about 0.67 per step.
    let velocity = Vector::new(40.0, 0.0);
    let sb = &mut world.soft_bodies[h];
    for i in 0..sb.num_particles() {
        sb.set_particle_velocity(i, velocity);
    }
    let surfaces: Vec<ColliderHandle> = world.soft_bodies[h]
        .meshes()
        .map(|mesh| mesh.collider())
        .collect();
    assert!(!surfaces.is_empty());
    let shape = ColliderBuilder::cuboid(0.05, 0.05)
        .density(0.1)
        .translation(Vector::new(0.3, -0.2));
    let root = world.soft_bodies[h].root_body();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    let mut rigid: Vec<ColliderHandle> = [root, proxy]
        .into_iter()
        .map(|parent| {
            world
                .colliders
                .insert_with_parent(shape.clone(), parent, &mut world.bodies)
        })
        .collect();
    // The same collider on a dynamic body moving like the square, for reference.
    let (_, reference) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 5.0))
            .linvel(velocity),
        shape,
    );
    rigid.push(reference);

    let params = world.integration_parameters;
    for step in 0..10 {
        world.step();
        for co in &rigid {
            let collider = &world.colliders[*co];
            assert_eq!(
                collider.compute_broad_phase_aabb(&params, &world.bodies),
                collider.compute_collision_aabb(params.prediction_distance() / 2.0),
                "step {step}: collider {co:?} has a padded broad-phase AABB"
            );
            for p in points_past_aabb(&world, *co, 0.2) {
                assert!(
                    !leaf_reaches(&world, *co, p),
                    "step {step}: the broad-phase leaf of the rigid collider {co:?} reaches {p:?}"
                );
            }
        }
        for co in &surfaces {
            for p in points_past_aabb(&world, *co, 0.2) {
                assert!(
                    leaf_reaches(&world, *co, p),
                    "step {step}: the broad-phase leaf of the surface {co:?} misses {p:?}"
                );
            }
        }
    }
    // The square kept its speed, so its margin stayed large.
    assert!(world.bodies[root].translation().x > 5.0);
}

/// Shoots a ball at a rigid ball hung on the root proxy of a soft square flying toward it
/// (`on_proxy`), or on a dynamic body moving like that square. Returns whether they touched and
/// whether the shot ended past the target.
fn shoot_at_a_moving_target(on_proxy: bool, speed: Real, ccd: bool) -> (bool, bool) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let velocity = Vector::new(10.0, 0.0);
    let offset = Vector::new(1.2, 0.0);
    let target = ColliderBuilder::ball(0.5).density(0.1).translation(offset);
    let target = if on_proxy {
        let (h, _) = square_with_a_top_cluster(&mut world);
        let sb = &mut world.soft_bodies[h];
        for i in 0..sb.num_particles() {
            sb.set_particle_velocity(i, velocity);
        }
        let root = world.soft_bodies[h].root_body();
        world
            .colliders
            .insert_with_parent(target, root, &mut world.bodies)
    } else {
        let body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 1.0))
            .linvel(velocity)
            .additional_mass(2.5);
        world.insert(body, target).1
    };
    let start = world.colliders[target].translation() + Vector::new(6.0, 0.0);
    let (shot, shot_co) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(start)
            .linvel(Vector::new(-speed, 0.0))
            .ccd_enabled(ccd),
        ColliderBuilder::ball(0.1),
    );

    let mut touched = false;
    for _ in 0..40 {
        world.step();
        touched |= world
            .narrow_phase
            .contact_pair(shot_co, target)
            .is_some_and(|pair| pair.has_any_active_contact());
    }
    let passed = world.bodies[shot].translation().x < world.colliders[target].translation().x;
    (touched, passed)
}

/// A rigid collider on a fast soft body's proxy stops a fast ball like the same collider on a
/// dynamic body does, without the soft-body motion margin: discretely when each step's closing
/// travel is shorter than the colliders, through the ball's CCD otherwise.
#[test]
fn rigid_collider_on_a_fast_proxy_does_not_let_a_ball_through() {
    for (speed, ccd) in [(20.0, false), (200.0, true)] {
        for on_proxy in [false, true] {
            let (touched, passed) = shoot_at_a_moving_target(on_proxy, speed, ccd);
            assert!(
                touched && !passed,
                "speed {speed}, ccd {ccd}, on a proxy {on_proxy}: touched {touched}, passed {passed}"
            );
        }
    }
}

/// The motion margin a collider's broad-phase AABB is padded with, past the prediction distance.
fn motion_margin(world: &PhysicsWorld, co: ColliderHandle) -> Real {
    let params = world.integration_parameters;
    let collider = &world.colliders[co];
    let padded = collider.compute_broad_phase_aabb(&params, &world.bodies);
    let unpadded = collider.compute_collision_aabb(params.prediction_distance() / 2.0);
    padded.half_extents().x - unpadded.half_extents().x
}

/// A step split by the CCD sizes the soft-body motion margin with the full step's `dt`, not with
/// the length of its last CCD pass: the margin covers the coming step's travel.
#[test]
fn soft_motion_margin_spans_the_full_step_when_the_ccd_splits_it() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.max_ccd_substeps = 2;
    let dt = world.integration_parameters.dt;
    let square = SoftBodyBuilder::grid(Vector::new(0.0, 1.0), Vector::splat(0.5), 5, 5)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e5,
            ..Default::default()
        })
        .particle_mass(0.1);
    let h = world.insert_soft_body(square);
    let velocity = Vector::new(100.0, 0.0);
    let sb = &mut world.soft_bodies[h];
    for i in 0..sb.num_particles() {
        sb.set_particle_velocity(i, velocity);
    }
    let surfaces: Vec<ColliderHandle> = world.soft_bodies[h]
        .meshes()
        .map(|mesh| mesh.collider())
        .collect();
    assert!(!surfaces.is_empty());
    // A fast CCD ball hitting a wall late in the second step, far from the square: the first CCD
    // pass stops at the impact and the last one only covers the rest of the step.
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -20.0)),
        ColliderBuilder::cuboid(0.1, 2.0),
    );
    let speed = 200.0;
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-0.2 - 1.8 * speed * dt, -20.0))
            .linvel(Vector::new(speed, 0.0))
            .ccd_enabled(true),
        ColliderBuilder::ball(0.1),
    );

    world.step();
    world.step();
    assert_eq!(world.physics_pipeline.counters.ccd.num_substeps, 2);
    let max_speed = world.soft_bodies[h]
        .particles()
        .iter()
        .map(|p| p.velocity().length())
        .fold(0.0, Real::max);
    let expected = max_speed * dt;
    assert!(expected > 1.5);
    for co in &surfaces {
        let margin = motion_margin(&world, *co);
        assert!(
            (margin - expected).abs() < 1.0e-3,
            "surface {co:?}: margin {margin}, expected {expected}"
        );
        for p in points_past_aabb(&world, *co, 0.9 * expected) {
            assert!(
                leaf_reaches(&world, *co, p),
                "the broad-phase leaf of the surface {co:?} misses {p:?}"
            );
        }
    }
}
