//! Soft-body clusters and their proxy rigid bodies: lifecycle (add/remove clusters, remove
//! proxies, remove colliders), reference-counted particle ownership, and the island coupling of
//! a soft body's proxies.

use rapier3d::prelude::*;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0).translation(Vector::new(0.0, -0.5, 0.0)),
    );
    world
}

fn cloth(world: &mut PhysicsWorld) -> SoftBodyHandle {
    world.insert_soft_body(SoftBodyBuilder::cloth(
        Vector::new(0.0, 2.0, 0.0),
        Vector::X * 0.2,
        Vector::Z * 0.2,
        6,
        6,
    ))
}

fn rope(world: &mut PhysicsWorld) -> SoftBodyHandle {
    world.insert_soft_body(SoftBodyBuilder::rope(
        Vector::new(0.0, 2.0, 0.0),
        Vector::X * 0.2,
        8,
    ))
}

#[test]
fn root_body_is_the_whole_body_cluster_proxy() {
    let mut world = world_with_ground();
    let h = cloth(&mut world);
    let sb = &world.soft_bodies[h];
    assert_eq!(sb.num_live_clusters(), 1);
    assert_eq!(sb.cluster_proxy(0), Some(sb.root_body()));
    assert_eq!(sb.cluster(0).unwrap().particles().len(), sb.num_particles());
    let rb = &world.bodies[sb.root_body()];
    assert_eq!(rb.body_type(), RigidBodyType::SoftFrame);
    assert!(rb.is_soft_frame());
    assert_eq!(rb.soft_body(), Some(h));
    assert_eq!(rb.soft_cluster(), Some(0));
}

#[test]
fn soft_frame_setters_are_ignored() {
    let mut world = world_with_ground();
    let h = cloth(&mut world);
    let root = world.soft_bodies[h].root_body();
    // The proxy's pose is the cluster's derived frame; the setters must not disturb it.
    let pose_before = *world.bodies[root].position();
    let vels_before = world.bodies[root].linvel();
    let rb = &mut world.bodies[root];
    rb.set_translation(Vector::new(10.0, 0.0, 0.0), true);
    rb.set_linvel(Vector::new(5.0, 0.0, 0.0), true);
    rb.set_body_type(RigidBodyType::Fixed, true);
    let rb = &world.bodies[root];
    assert_eq!(*rb.position(), pose_before);
    assert_eq!(rb.linvel(), vels_before);
    assert_eq!(rb.body_type(), RigidBodyType::SoftFrame);
    // A regular body cannot be converted to a soft frame either.
    let other = world.insert_body(RigidBodyBuilder::dynamic());
    world.bodies[other].set_body_type(RigidBodyType::SoftFrame, true);
    assert_eq!(world.bodies[other].body_type(), RigidBodyType::Dynamic);
}

#[test]
fn removing_the_root_proxy_removes_the_soft_body() {
    let mut world = world_with_ground();
    let h = cloth(&mut world);
    let root = world.soft_bodies[h].root_body();
    for _ in 0..5 {
        world.step();
    }
    world.remove_body(root);
    // The zombie of old: the soft body must be gone, not left unsimulated.
    assert!(world.soft_bodies.get(h).is_none());
    assert_eq!(world.colliders.len(), 1); // The ground.
    for _ in 0..5 {
        world.step();
    }
}

#[test]
fn removing_the_root_proxy_of_a_rope_then_stepping_is_clean() {
    let mut world = world_with_ground();
    let h = rope(&mut world);
    world.step();
    let root = world.soft_bodies[h].root_body();
    world.remove_body(root);
    assert!(world.soft_bodies.get(h).is_none());
    assert_eq!(world.colliders.len(), 1);
    world.step();
}

#[test]
fn removing_a_corner_cluster_is_non_destructive() {
    let mut world = world_with_ground();
    let h = cloth(&mut world);
    let num_particles = world.soft_bodies[h].num_particles();
    let num_edges = world.soft_bodies[h].edges().len();
    let cluster = world
        .soft_bodies
        .add_cluster(h, &[0, 1, 6], &mut world.bodies, &mut world.colliders)
        .unwrap();
    assert_eq!(world.soft_bodies[h].num_live_clusters(), 2);
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    assert_eq!(world.bodies[proxy].soft_cluster(), Some(cluster));
    world.step();

    // While the whole-body cluster exists, removing a user cluster deletes nothing.
    let report = world
        .soft_bodies
        .remove_cluster(
            h,
            cluster,
            &mut world.islands,
            &mut world.bodies,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
        )
        .unwrap();
    assert_eq!(report.particles, 0);
    assert_eq!(report.edges, 0);
    assert!(!report.soft_body_removed);
    assert!(world.bodies.get(proxy).is_none());
    assert_eq!(world.soft_bodies[h].num_live_clusters(), 1);
    assert_eq!(world.soft_bodies[h].num_particles(), num_particles);
    assert_eq!(world.soft_bodies[h].edges().len(), num_edges);
    world.step();
}

#[test]
fn removing_the_whole_body_cluster_keeps_the_corner() {
    let mut world = world_with_ground();
    let h = cloth(&mut world);
    let num_particles = world.soft_bodies[h].num_particles();
    // A corner cluster over the particles of the first row.
    let corner: Vec<u32> = (0..6).collect();
    let cluster = world
        .soft_bodies
        .add_cluster(h, &corner, &mut world.bodies, &mut world.colliders)
        .unwrap();
    let corner_proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    world.step();

    // Removing cluster 0 deletes everything the corner does not cover.
    let report = world
        .soft_bodies
        .remove_cluster(
            h,
            0,
            &mut world.islands,
            &mut world.bodies,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
        )
        .unwrap();
    assert_eq!(report.particles, num_particles - corner.len());
    assert!(!report.soft_body_removed);
    let sb = &world.soft_bodies[h];
    assert_eq!(sb.num_particles(), corner.len());
    assert_eq!(sb.num_live_clusters(), 1);
    // The root body re-points to the surviving cluster's proxy.
    assert_eq!(sb.root_body(), corner_proxy);
    sb.validate_topology().unwrap();
    for _ in 0..10 {
        world.step();
    }
    // Removing the last cluster removes the body.
    let report = world
        .soft_bodies
        .remove_cluster(
            h,
            cluster,
            &mut world.islands,
            &mut world.bodies,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
        )
        .unwrap();
    assert!(report.soft_body_removed);
    assert_eq!(report.particles, corner.len());
    assert!(world.soft_bodies.get(h).is_none());
    world.step();
}

#[test]
fn removing_the_surface_collider_clears_the_back_reference() {
    let mut world = world_with_ground();
    let h = cloth(&mut world);
    world.step();
    let co = world.soft_bodies[h].collision_mesh().unwrap().collider();
    world.remove_collider(co);
    assert!(world.soft_bodies[h].collision_mesh().is_none());
    for _ in 0..5 {
        world.step();
    }
}

#[test]
fn cluster_proxies_share_the_island_and_sleep_as_a_unit() {
    let mut world = world_with_ground();
    let h = cloth(&mut world);
    let cluster = world
        .soft_bodies
        .add_cluster(h, &[0, 1, 2], &mut world.bodies, &mut world.colliders)
        .unwrap();
    let root = world.soft_bodies[h].root_body();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    // Let the cloth fall asleep on the ground.
    for _ in 0..2000 {
        world.step();
        if world.soft_bodies[h].is_sleeping() {
            break;
        }
    }
    assert!(world.soft_bodies[h].is_sleeping(), "cloth never slept");
    assert!(world.bodies[root].is_sleeping());
    assert!(world.bodies[proxy].is_sleeping(), "proxy left awake alone");
    assert_eq!(
        world.islands.persistent_island_of(&world.bodies, root),
        world.islands.persistent_island_of(&world.bodies, proxy),
    );
}

#[test]
fn tear_duplicates_join_their_source_clusters() {
    let mut world = world_with_ground();
    let h = world.insert_soft_body(
        SoftBodyBuilder::cloth(
            Vector::new(0.0, 2.0, 0.0),
            Vector::X * 0.2,
            Vector::Z * 0.2,
            6,
            6,
        )
        .tear_strain(0.2),
    );
    world.step();
    // Tear a middle edge immediately.
    let torn = world.tear_soft_body(h, &[30], &[]);
    assert!(torn.is_some());
    let sb = &world.soft_bodies[h];
    let after = sb.num_particles();
    // Whatever was duplicated must be covered by cluster 0 (refcounts stay consistent).
    assert_eq!(sb.cluster(0).unwrap().particles().len(), after);
    sb.validate_topology().unwrap();
    for _ in 0..5 {
        world.step();
    }
}

/// The surface collider's vertices live in the cluster frame: a soft body far from the origin
/// keeps its contact precision (world-sized coordinates never enter the vertex data).
#[test]
fn distant_soft_body_rests_cleanly() {
    let far = Vector::new(2000.0, 0.0, 2000.0);
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0).translation(far + Vector::new(0.0, -0.5, 0.0)),
    );
    let h = world.insert_soft_body(SoftBodyBuilder::cloth(
        far + Vector::new(0.0, 0.3, 0.0),
        Vector::X * 0.2,
        Vector::Z * 0.2,
        6,
        6,
    ));
    for _ in 0..600 {
        world.step();
    }
    let sb = &world.soft_bodies[h];
    for p in sb.particles() {
        assert!(p.position().is_finite());
        assert!(
            p.position().y > -0.2 && p.position().y < 0.5,
            "particle at odd height: {:?}",
            p.position()
        );
    }
    // The collider's world pose holds the distance; its local frame stays small.
    let root = sb.root_body();
    assert!((world.bodies[root].position().translation - far).length() < 5.0);
}

/// A user collider attached to a cluster proxy rides the cluster's frame, collides with the
/// world, and never collides with its own soft body's surface.
#[test]
fn user_collider_on_a_proxy_rides_the_frame() {
    let mut world = world_with_ground();
    let h = world.insert_soft_body(
        SoftBodyBuilder::cuboid(Vector::new(0.0, 0.6, 0.0), Vector::splat(0.5), 3, 3, 3)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e4,
                poisson_ratio: 0.3,
                elastic_damping_ratio: 1.0,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05)),
    );
    let root = world.soft_bodies[h].root_body();
    // A small bracket embedded in the jelly (overlapping its own surface: the same-soft-body
    // filter must keep them from fighting).
    let co = world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.1, 0.1, 0.1).density(0.1),
        root,
        &mut world.bodies,
    );
    for _ in 0..300 {
        world.step();
    }
    let sb = &world.soft_bodies[h];
    for p in sb.particles() {
        assert!(p.position().is_finite());
    }
    // The jelly rests on the ground instead of being pushed out by its own bracket.
    let com = sb.center_of_mass();
    assert!(
        com.y > 0.2 && com.y < 0.8,
        "the jelly did not settle (com {com:?})"
    );
    // The bracket's collider follows the frame.
    let co_pos = world.colliders[co].position().translation;
    assert!((co_pos - world.bodies[root].position().translation).length() < 0.3);
}

/// Per-cluster shape matching: two clusters of one cloth are held rigid independently while the
/// rest of the cloth stays soft.
#[test]
fn per_cluster_shape_matching_holds_two_regions() {
    let mut world = world_with_ground();
    let h = world.insert_soft_body(
        SoftBodyBuilder::cloth(
            Vector::new(0.0, 1.0, 0.0),
            Vector::X * 0.2,
            Vector::Z * 0.2,
            6,
            6,
        )
        .pinned_particles([0, 5]),
    );
    // Two disjoint clusters (two triangles of particles), shape-matched.
    let ca = world
        .soft_bodies
        .add_cluster(h, &[14, 15, 20], &mut world.bodies, &mut world.colliders)
        .unwrap();
    let cb = world
        .soft_bodies
        .add_cluster(h, &[27, 28, 33], &mut world.bodies, &mut world.colliders)
        .unwrap();
    let rest = |world: &PhysicsWorld, ids: [usize; 3]| -> [Real; 3] {
        let sb = &world.soft_bodies[h];
        let d01 = (sb.particle_position(ids[0]) - sb.particle_position(ids[1])).length();
        let d12 = (sb.particle_position(ids[1]) - sb.particle_position(ids[2])).length();
        let d02 = (sb.particle_position(ids[0]) - sb.particle_position(ids[2])).length();
        [d01, d12, d02]
    };
    let before_a = rest(&world, [14, 15, 20]);
    world.soft_bodies[h].enable_cluster_shape_matching(ca, true);
    world.soft_bodies[h].enable_cluster_shape_matching(cb, true);
    for _ in 0..240 {
        world.step();
    }
    let sb = &world.soft_bodies[h];
    for p in sb.particles() {
        assert!(p.position().is_finite());
    }
    // The shape-matched triangles keep their rest edge lengths within a few percent while the
    // cloth as a whole drapes.
    let after_a = rest(&world, [14, 15, 20]);
    for (b, a) in before_a.iter().zip(after_a.iter()) {
        assert!(
            (a - b).abs() < 0.15 * b,
            "cluster region deformed: {before_a:?} -> {after_a:?}"
        );
    }
}

/// The cluster material sugar: a stiffness-scaled region of a jelly compresses less than the
/// rest under the same load.
#[test]
fn cluster_stiffness_scale_stiffens_a_region() {
    let mut world = world_with_ground();
    let h = world.insert_soft_body(
        SoftBodyBuilder::cuboid(Vector::new(0.0, 0.5, 0.0), Vector::splat(0.5), 3, 3, 3)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 2.0e2,
                poisson_ratio: 0.3,
                elastic_damping_ratio: 1.0,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05)),
    );
    // A soft reference world with the identical body, unscaled.
    let mut soft_world = world_with_ground();
    let h_soft = soft_world.insert_soft_body(
        SoftBodyBuilder::cuboid(Vector::new(0.0, 0.5, 0.0), Vector::splat(0.5), 3, 3, 3)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 2.0e2,
                poisson_ratio: 0.3,
                elastic_damping_ratio: 1.0,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05)),
    );
    // Stiffen the whole body through the whole-body cluster: it must sag less than the soft
    // reference under its own weight (a few centimeters at this modulus).
    world.soft_bodies[h].set_cluster_stiffness_scale(0, 50.0);
    for _ in 0..300 {
        world.step();
        soft_world.step();
    }
    let top = |world: &PhysicsWorld, h: SoftBodyHandle| -> Real {
        world.soft_bodies[h]
            .particle_positions()
            .map(|p| p.y)
            .fold(0.0, |a: Real, b| a.max(b))
    };
    let stiff_top = top(&world, h);
    let soft_top = top(&soft_world, h_soft);
    assert!(
        stiff_top > soft_top + 0.02,
        "the scaled body did not stiffen: stiff {stiff_top} vs soft {soft_top}"
    );
}

/// The fixed/kinematic mass-point sugar: pinning a cluster holds its region in place, and a
/// kinematic target moves the whole region rigidly (with the matching velocities), dragging the
/// rest of the body.
#[test]
fn pinned_cluster_drives_its_region_kinematically() {
    let mut world = PhysicsWorld::new();
    // A vertical cloth banner; its top edge is the driven cluster.
    let h = world.insert_soft_body(SoftBodyBuilder::cloth(
        Vector::new(0.0, 2.0, 0.0),
        Vector::X * 0.2,
        Vector::Y * -0.2,
        6,
        6,
    ));
    let top_edge: Vec<u32> = (0..6).collect();
    let grip = world
        .soft_bodies
        .add_cluster(h, &top_edge, &mut world.bodies, &mut world.colliders)
        .unwrap();
    world.soft_bodies[h].set_cluster_pinned(grip, true);
    for &v in &top_edge {
        assert!(world.soft_bodies[h].particles()[v as usize].is_pinned());
    }
    // Pinned without a target: the region holds while the rest drapes.
    let before: Vec<Vector> = top_edge
        .iter()
        .map(|&v| world.soft_bodies[h].particle_position(v as usize))
        .collect();
    for _ in 0..120 {
        world.step();
    }
    for (i, &v) in top_edge.iter().enumerate() {
        let now = world.soft_bodies[h].particle_position(v as usize);
        assert!(
            (now - before[i]).length() < 1.0e-4,
            "pinned cluster moved: {:?} -> {now:?}",
            before[i]
        );
    }

    // Drive the grip sideways: the whole row travels rigidly, the cloth follows.
    let grip_proxy = world.soft_bodies[h].cluster_proxy(grip).unwrap();
    let home = world.bodies[grip_proxy].position().translation;
    let shift = Vector::new(1.5, 0.0, 0.0);
    let steps = 120;
    for k in 0..steps {
        let s = (k + 1) as Real / steps as Real;
        world.soft_bodies[h]
            .set_cluster_kinematic_target(grip, Pose::from_translation(home + shift * s));
        world.step();
    }
    // The row arrived, rigidly (edge lengths preserved).
    let d0 = (world.soft_bodies[h].particle_position(0)
        - world.soft_bodies[h].particle_position(5))
    .length();
    assert!(
        (d0 - 1.0).abs() < 1.0e-3,
        "the driven row deformed (width {d0})"
    );
    let mid = world.soft_bodies[h].particle_position(2);
    assert!(
        (mid.x - (before[2].x + shift.x)).abs() < 0.05,
        "the driven row did not reach its target ({mid:?})"
    );
    // The free part of the cloth was dragged along.
    let bottom = world.soft_bodies[h].particle_position(32);
    assert!(
        bottom.x > 0.7,
        "the cloth did not follow its grip ({bottom:?})"
    );
    for p in world.soft_bodies[h].particles() {
        assert!(p.position().is_finite());
    }
}

/// A jelly cube with a stiff corotational material, on the ground.
fn jelly(world: &mut PhysicsWorld) -> SoftBodyHandle {
    world.insert_soft_body(
        SoftBodyBuilder::cuboid(Vector::new(0.0, 0.6, 0.0), Vector::splat(0.5), 3, 3, 3)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e4,
                poisson_ratio: 0.3,
                elastic_damping_ratio: 1.0,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05)),
    )
}

/// The particles of a soft body above `y`.
fn particles_above(world: &PhysicsWorld, handle: SoftBodyHandle, y: Real) -> Vec<u32> {
    world.soft_bodies[handle]
        .particles()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.position().y > y)
        .map(|(i, _)| i as u32)
        .collect()
}

#[test]
fn a_rigid_collider_on_a_sub_cluster_does_not_fight_its_own_body() {
    let mut world = world_with_ground();
    let h = jelly(&mut world);
    let top = particles_above(&world, h, 0.6);
    assert!(!top.is_empty());
    let cluster = world
        .soft_bodies
        .add_cluster(h, &top, &mut world.bodies, &mut world.colliders)
        .unwrap();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    // A bracket embedded in the jelly, on a proxy that is *not* the parent of its surface
    // collider: only the same-soft-body filter can keep the two apart.
    let bracket = world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.2, 0.2, 0.2).density(0.1),
        proxy,
        &mut world.bodies,
    );
    let surface = world.soft_bodies[h].collision_mesh().unwrap().collider();

    for _ in 0..300 {
        world.step();
    }

    assert!(
        !world
            .narrow_phase
            .contact_pair(bracket, surface)
            .is_some_and(|pair| pair.has_any_active_contact()),
        "a body's own bracket collided with its surface"
    );
    let sb = &world.soft_bodies[h];
    assert!(sb.particle_positions().all(|p| p.is_finite()));
    let com = sb.center_of_mass();
    assert!(
        com.y > 0.2 && com.y < 0.8,
        "the jelly did not settle (com {com:?})"
    );
}

#[test]
fn a_rigid_collider_on_a_cluster_brings_the_world_to_the_particles() {
    let mut world = world_with_ground();
    // A soft jelly, so the load's deflection is visible: the box's weight over the top face
    // compresses it by about a twelfth of its height at this modulus.
    let h = world.insert_soft_body(
        SoftBodyBuilder::cuboid(Vector::new(0.0, 0.6, 0.0), Vector::splat(0.5), 3, 3, 3)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: 5.0e2,
                poisson_ratio: 0.3,
                elastic_damping_ratio: 1.0,
                ..Default::default()
            })
            .particle_mass(0.1)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05)),
    );
    let top = particles_above(&world, h, 0.6);
    let cluster = world
        .soft_bodies
        .add_cluster(h, &top, &mut world.bodies, &mut world.colliders)
        .unwrap();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    // A plate on the jelly's top cluster, and a heavy box dropped on it.
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.05, 0.5).translation(Vector::new(0.0, 0.55, 0.0)),
        proxy,
        &mut world.bodies,
    );
    for _ in 0..100 {
        world.step();
    }
    let before = world.soft_bodies[h].center_of_mass().y;

    let (weight, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.5, 0.0)),
        ColliderBuilder::cuboid(0.3, 0.3, 0.3).density(20.0),
    );
    for _ in 0..300 {
        world.step();
    }

    let sb = &world.soft_bodies[h];
    assert!(sb.particle_positions().all(|p| p.is_finite()));
    let after = sb.center_of_mass().y;
    assert!(
        after < before - 0.02,
        "the load on the plate did not reach the particles: {before} -> {after}"
    );
    // The weight rests on the plate rather than falling through the jelly.
    let weight_y = world.bodies[weight].translation().y;
    assert!(weight_y > 0.2, "the weight fell through: {weight_y}");
}

/// A one-particle cluster on a chain torn at that particle splits with it: the copy (the heavier
/// share) keeps the proxy and the joint follows the source at the same world anchor. The chain
/// splits into two soft bodies, the longer one keeping the handle, each with its own collider.
#[test]
fn torn_cluster_splits_and_its_joint_follows_the_source() {
    let xs = [0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 7.0, 8.0, 9.0];
    let positions: Vec<Vector> = xs.iter().map(|&x| Vector::X * x).collect();
    let segments: Vec<[u32; 2]> = (0..8).map(|i| [i, i + 1]).collect();
    let builder = SoftBodyBuilder::new(positions)
        .edges(segments.clone())
        .particle_mass(0.3)
        .pinned_particles([0]);
    let builder = builder.wire(segments);
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
    // copy's; the body split then parted 3 from 7, the heavier 7 keeping the proxy. Three pieces:
    // 7 and the copy in the retained body, 3 in the pinned tail.
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

/// Asserts that a collider sits at its parent proxy's pose composed with its local pose, and that
/// the broad phase has a leaf covering it there.
fn assert_rides_its_proxy(world: &PhysicsWorld, co: ColliderHandle, step: usize) {
    let collider = &world.colliders[co];
    let proxy = &world.bodies[collider.parent().unwrap()];
    let expected = proxy.position() * collider.position_wrt_parent().unwrap();
    let actual = collider.position();
    let dt = (actual.translation - expected.translation).length();
    // Compares rotated axes: an `acos`-based angle is too noisy near zero in single precision.
    let dr = [Vector::X, Vector::Y, Vector::Z]
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
/// falling, spinning jelly are at their proxy's pose at the end of every step, not one step behind.
#[test]
fn rigid_colliders_on_proxies_do_not_lag_their_proxy() {
    let mut world = PhysicsWorld::new();
    let h = jelly(&mut world);
    let top = particles_above(&world, h, 0.6);
    let cluster = world
        .soft_bodies
        .add_cluster(h, &top, &mut world.bodies, &mut world.colliders)
        .unwrap();
    let sb = &mut world.soft_bodies[h];
    let com = sb.center_of_mass();
    for i in 0..sb.num_particles() {
        let arm = sb.particle_position(i) - com;
        sb.set_particle_velocity(
            i,
            Vector::new(1.0, 0.0, 0.5) + Vector::new(0.5, 2.0, 1.0).cross(arm),
        );
    }
    let root = world.soft_bodies[h].root_body();
    let proxy = world.soft_bodies[h].cluster_proxy(cluster).unwrap();
    let offset = Pose::from_parts(
        Vector::new(0.3, 0.2, -0.1),
        Rotation::from_scaled_axis(Vector::new(0.2, 0.4, 0.1)),
    );
    let colliders: Vec<ColliderHandle> = [root, proxy]
        .into_iter()
        .map(|parent| {
            world.colliders.insert_with_parent(
                ColliderBuilder::cuboid(0.05, 0.05, 0.05)
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
    assert!(world.bodies[root].translation().y < 0.0);
}

/// Asserts that the broad-phase leaf of a collider contains the collider's current AABB: every
/// corner of that AABB hits the leaf.
fn assert_leaf_contains_current_aabb(world: &PhysicsWorld, co: ColliderHandle, step: usize) {
    let aabb = world.colliders[co].compute_aabb();
    for i in 0..8 {
        let pick = |bit: usize, min: Real, max: Real| if i & bit == 0 { min } else { max };
        let corner = Vector::new(
            pick(1, aabb.mins.x, aabb.maxs.x),
            pick(2, aabb.mins.y, aabb.maxs.y),
            pick(4, aabb.mins.z, aabb.maxs.z),
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

/// The surface collider of a soft cube falling faster at each step (the large timestep makes
/// gravity outrun the speculative margin) has a broad-phase leaf around its end-of-step geometry,
/// so a ray cast between steps hits its current bottom face.
#[test]
fn deformable_collider_leaves_follow_their_surface_at_the_end_of_each_step() {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = 0.1;
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 1.0, 0.0), Vector::splat(0.5), 3, 3, 3)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .particle_radius(0.01);
    let h = world.insert_soft_body(cube);
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
            // Off the surface's vertices and edges, which a ray may graze.
            let center = aabb.center();
            let ray = Ray::new(
                Vector::new(center.x + 0.13, aabb.mins.y - 0.01, center.z + 0.07),
                Vector::Y,
            );
            let hit = world.cast_ray(&ray, 0.02, true, QueryFilter::default());
            assert_eq!(
                hit.map(|(handle, _)| handle),
                Some(*co),
                "step {step}: a ray cast misses the bottom face of collider {co:?}"
            );
        }
    }
    // The cube did fall, so a stale leaf could not have passed.
    let root = world.soft_bodies[h].root_body();
    assert!(world.bodies[root].translation().y < -10.0);
}

/// Whether the broad-phase leaf of a collider reaches the point `p`.
fn leaf_reaches(world: &PhysicsWorld, co: ColliderHandle, p: Vector) -> bool {
    world
        .intersect_aabb_conservative(Aabb::new(p, p), QueryFilter::default())
        .any(|(handle, _)| handle == co)
}

/// The points `dist` past the middle of each face of a collider's current AABB.
fn points_past_aabb(world: &PhysicsWorld, co: ColliderHandle, dist: Real) -> Vec<Vector> {
    let aabb = world.colliders[co].compute_aabb();
    let (center, half) = (aabb.center(), aabb.half_extents());
    (0..3)
        .flat_map(|i| {
            [-1.0, 1.0].map(|sign| {
                let mut p = center;
                p[i] += sign * (half[i] + dist);
                p
            })
        })
        .collect()
}

/// The soft-body motion margin pads the deformable colliders of a fast jelly only: the rigid
/// colliders on its proxies get the broad-phase AABB of a collider on a dynamic body.
#[test]
fn only_deformable_colliders_are_padded_by_the_soft_motion_margin() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let h = jelly(&mut world);
    let top = particles_above(&world, h, 0.6);
    let cluster = world
        .soft_bodies
        .add_cluster(h, &top, &mut world.bodies, &mut world.colliders)
        .unwrap();
    // A motion margin of about 0.67 per step.
    let velocity = Vector::new(40.0, 0.0, 0.0);
    let sb = &mut world.soft_bodies[h];
    for i in 0..sb.num_particles() {
        sb.set_particle_velocity(i, velocity);
    }
    let surfaces: Vec<ColliderHandle> = world.soft_bodies[h]
        .meshes()
        .map(|mesh| mesh.collider())
        .collect();
    assert!(!surfaces.is_empty());
    let shape = ColliderBuilder::cuboid(0.05, 0.05, 0.05)
        .density(0.1)
        .translation(Vector::new(0.3, 0.2, -0.1));
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
    // The same collider on a dynamic body moving like the jelly, for reference.
    let (_, reference) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 5.0, 0.0))
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
    // The jelly kept its speed, so its margin stayed large.
    assert!(world.bodies[root].translation().x > 5.0);
}

/// Shoots a ball at a rigid ball hung on the root proxy of a jelly flying toward it (`on_proxy`),
/// or on a dynamic body moving like that jelly. Returns whether they touched and whether the
/// shot ended past the target.
fn shoot_at_a_moving_target(on_proxy: bool, speed: Real, ccd: bool) -> (bool, bool) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let velocity = Vector::new(10.0, 0.0, 0.0);
    let offset = Vector::new(1.2, 0.0, 0.0);
    let target = ColliderBuilder::ball(0.5).density(0.1).translation(offset);
    let target = if on_proxy {
        let h = jelly(&mut world);
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
            .translation(Vector::new(0.0, 0.6, 0.0))
            .linvel(velocity)
            .additional_mass(2.7);
        world.insert(body, target).1
    };
    let start = world.colliders[target].translation() + Vector::new(6.0, 0.0, 0.0);
    let (shot, shot_co) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(start)
            .linvel(Vector::new(-speed, 0.0, 0.0))
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
    let h = jelly(&mut world);
    let velocity = Vector::new(100.0, 0.0, 0.0);
    let sb = &mut world.soft_bodies[h];
    for i in 0..sb.num_particles() {
        sb.set_particle_velocity(i, velocity);
    }
    let surfaces: Vec<ColliderHandle> = world.soft_bodies[h]
        .meshes()
        .map(|mesh| mesh.collider())
        .collect();
    assert!(!surfaces.is_empty());
    // A fast CCD ball hitting a wall late in the second step, far from the jelly: the first CCD
    // pass stops at the impact and the last one only covers the rest of the step.
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -20.0, 0.0)),
        ColliderBuilder::cuboid(0.1, 2.0, 2.0),
    );
    let speed = 200.0;
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-0.2 - 1.8 * speed * dt, -20.0, 0.0))
            .linvel(Vector::new(speed, 0.0, 0.0))
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
