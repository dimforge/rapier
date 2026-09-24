use crate::*;

fn same_recipe(desc: &RprSoftBodyDesc, native: SoftBodyBuilder) {
    let actual = unsafe { desc.raw().unwrap() };
    assert_eq!(format!("{actual:?}"), format!("{native:?}"));
}

unsafe fn rope(world: *mut RprWorld, particles: usize) -> RprSoftBodyHandle {
    let desc = rpr_rope_soft_body_desc(Vector::ZERO.into(), Vector::X.into(), particles);
    let handle = unsafe { rpr_insert_soft_body(world, &desc) };
    assert_eq!(rpr_last_status(), RPR_OK);
    handle
}

#[test]
fn particle_velocity_impulses_and_solver() {
    unsafe {
        let world = rpr_new_world();
        let handle = rope(world, 3);
        let v = rpr_soft_body_particle_velocity(handle, 3);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert_eq!(v.raw().unwrap(), Vector::ZERO);
        assert_eq!(rpr_soft_body_solver(handle), RPR_SOFT_SOLVER_CONSTRAINTS);
        assert_eq!(rpr_last_status(), RPR_OK);

        // Linear falloff from the point: 1, 0.5 and 0 for the particles at x = 0, 0.5 and 1.
        let up = Vector::Y * 2.0;
        assert_eq!(
            rpr_soft_body_apply_impulse_at_point(handle, up.into(), Vector::ZERO.into(), 1.0, 1),
            RPR_OK
        );
        for (i, expected) in [2.0, 1.0, 0.0].into_iter().enumerate() {
            let v = rpr_soft_body_particle_velocity(handle, i);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert!((v.raw().unwrap() - Vector::Y * expected).length() < 1.0e-5);
        }
        for i in 0..3 {
            rpr_soft_body_set_particle_velocity(handle, i, Vector::ZERO.into());
        }
        // Away from the middle particle, which gets nothing; no falloff pushes fully.
        let center = Vector::X * 0.5;
        assert_eq!(
            rpr_soft_body_apply_radial_impulse(handle, center.into(), 3.0, 0.0, 1),
            RPR_OK
        );
        let v: Vec<_> = (0..3)
            .map(|i| rpr_soft_body_particle_velocity(handle, i).raw().unwrap())
            .collect();
        assert!((v[0] + Vector::X * 3.0).length() < 1.0e-5);
        assert_eq!(v[1], Vector::ZERO);
        assert!((v[2] - Vector::X * 3.0).length() < 1.0e-5);
        assert_eq!(
            rpr_soft_body_apply_radial_impulse(handle, center.into(), Real::NAN, 0.0, 1),
            RPR_INVALID_ARGUMENT
        );
        assert_eq!(rpr_soft_body_reset_plasticity(handle), RPR_OK);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn deferred_and_immediate_tears_report_consistent_pieces() {
    unsafe {
        let world = rpr_new_world();
        let handle = rope(world, 8);
        assert_eq!(rpr_soft_body_tear_edge(handle, 100), RPR_INVALID_ARGUMENT);
        assert_eq!(rpr_soft_body_tear_cell(handle, 0), RPR_INVALID_ARGUMENT);
        assert_eq!(rpr_soft_body_tear_edge(handle, 3), RPR_OK);
        assert_eq!(rpr_soft_body_count(world), 1);
        assert_eq!(rpr_step(world, std::ptr::null(), std::ptr::null()), RPR_OK);
        assert_eq!(rpr_soft_body_count(world), 2);

        // An immediate tear splitting a rope in two pieces.
        let handle = rope(world, 8);
        let edge = 3u32;
        let event = rpr_soft_body_tear(handle, &edge, 1, std::ptr::null(), 0);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert!(!event.is_null());
        let count = rpr_soft_body_tear_event_piece_count(event);
        assert_eq!(count, 2);
        assert_eq!(
            rpr_soft_body_tear_event_bodies(event, std::ptr::null_mut(), 0),
            count
        );
        let n = rpr_soft_body_tear_event_piece_particles(event, 1, std::ptr::null_mut(), 0);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert!(n > 0);
        rpr_soft_body_tear_event_piece_particles(event, 2, std::ptr::null_mut(), 0);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert_eq!(rpr_free_soft_body_tear_event(event), RPR_OK);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn tear_splitting_nothing_has_one_piece_with_every_particle() {
    // A synthetic event: nothing was split off a body of 4 particles.
    let body = SoftBodyHandle::invalid();
    let event = RprSoftBodyTearEvent(
        SoftBodyTearEvent {
            soft_body: body,
            ..Default::default()
        },
        std::ptr::null_mut(),
        4,
    );
    unsafe {
        assert_eq!(rpr_soft_body_tear_event_piece_count(&event), 1);
        assert_eq!(
            rpr_soft_body_tear_event_bodies(&event, std::ptr::null_mut(), 0),
            1
        );
        let mut particles = [0u32; 4];
        assert_eq!(
            rpr_soft_body_tear_event_piece_particles(&event, 0, particles.as_mut_ptr(), 4),
            4
        );
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(particles, [0, 1, 2, 3]);
        rpr_soft_body_tear_event_piece_particles(&event, 1, std::ptr::null_mut(), 0);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
    }
}

#[test]
fn clusters_attachments_and_root_removal() {
    unsafe {
        let world = rpr_new_world();
        let handle = rope(world, 3);
        let root = rpr_soft_body_root_body(handle);
        // The root is also a cluster proxy, but removing it would remove the whole body.
        assert_eq!(rpr_remove_rigid_body(root, 1), RPR_INVALID_ARGUMENT);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert_eq!(rpr_soft_body_count(world), 1);
        assert_eq!(rpr_rigid_body_contains(root), 1);

        let particle = 2u32;
        let cluster = rpr_soft_body_add_cluster(handle, &particle, 1);
        assert_eq!(rpr_last_status(), RPR_OK);
        let softness = RprSpringCoefficients {
            natural_frequency: 5.0,
            damping_ratio: 1.0,
        };
        assert_eq!(
            rpr_soft_body_set_cluster_edge_softness(handle, cluster, &softness),
            RPR_OK
        );
        assert_eq!(
            rpr_soft_body_set_cluster_edge_softness(handle, cluster, std::ptr::null()),
            RPR_OK
        );
        assert_eq!(
            rpr_soft_body_set_cluster_edge_softness(handle, 99, &softness),
            RPR_INVALID_ARGUMENT
        );
        // Removing a cluster proxy removes its cluster.
        let proxy = rpr_soft_body_cluster_proxy(handle, cluster);
        assert_eq!(rpr_remove_rigid_body(proxy, 1), RPR_OK);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(rpr_soft_body_count(world), 1);

        let desc = rpr_fixed_rigid_body_desc();
        let anchor = rpr_insert_rigid_body(world, &desc);
        assert_eq!(rpr_soft_body_detach_particle(handle, 0), 0);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(rpr_soft_body_attach_particle(handle, 0, anchor), RPR_OK);
        assert_eq!(rpr_soft_body_detach_particle(handle, 0), 1);
        assert_eq!(rpr_soft_body_detach_particle(handle, 0), 0);
        assert_eq!(rpr_soft_body_detach_particle(handle, 99), 0);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn deformable_colliders_report_their_soft_body() {
    unsafe {
        let world = rpr_new_world();
        #[cfg(feature = "dim2")]
        let desc = rpr_disk_soft_body_desc(Vector::ZERO.into(), 1.0, 12);
        #[cfg(feature = "dim3")]
        let desc = rpr_sphere_soft_body_desc(Vector::ZERO.into(), 1.0, 1);
        let handle = rpr_insert_soft_body(world, &desc);
        assert_eq!(rpr_last_status(), RPR_OK);
        let mut collider = RprColliderHandle::default();
        assert_eq!(rpr_soft_body_mesh_colliders(handle, &mut collider, 1), 1);
        assert_eq!(rpr_collider_soft_body(collider), handle);
        assert_eq!(rpr_last_status(), RPR_OK);

        let rigid = RprColliderDesc::default();
        let rigid = rpr_insert_collider_without_parent(world, &rigid);
        assert_eq!(rpr_collider_soft_body(rigid), RprSoftBodyHandle::default());
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn recovery_and_fem_settings_round_trip() {
    unsafe {
        let world = rpr_new_world();
        let defaults = SoftRecoverySettings::default();
        assert_eq!(
            rpr_recovery_overlap_patience(world),
            defaults.overlap_patience
        );
        assert_eq!(
            rpr_recovery_edge_speculation(world),
            defaults.edge_speculation as RprBool
        );
        assert_eq!(rpr_recovery_recovery_pace(world), defaults.recovery_pace);
        assert_eq!(rpr_recovery_set_overlap_split(world, 5), RPR_OK);
        assert_eq!(rpr_recovery_overlap_split(world), 5);
        assert_eq!(rpr_recovery_set_overlap_patience(world, 7), RPR_OK);
        assert_eq!(rpr_recovery_overlap_patience(world), 7);
        assert_eq!(
            rpr_recovery_set_overlap_patch_constraints(world, RPR_SOFT_PATCH_CONSTRAINTS_KEEP),
            RPR_OK
        );
        assert_eq!(
            rpr_recovery_overlap_patch_constraints(world),
            RPR_SOFT_PATCH_CONSTRAINTS_KEEP
        );
        assert_eq!(
            rpr_recovery_set_overlap_patch_constraints(world, 3),
            RPR_INVALID_ARGUMENT
        );
        assert_eq!(
            rpr_recovery_overlap_patch_constraints(world),
            RPR_SOFT_PATCH_CONSTRAINTS_KEEP
        );
        #[cfg(feature = "fem")]
        {
            assert_eq!(rpr_fem_set_max_dense_dofs(world, 12), RPR_OK);
            assert_eq!(rpr_fem_max_dense_dofs(world), 12);
            assert_eq!(rpr_fem_set_linear_tolerance(world, 0.5), RPR_OK);
            assert_eq!(rpr_fem_linear_tolerance(world), 0.5);
            assert_eq!(
                rpr_fem_max_linear_iterations(world),
                SoftFemParameters::default().max_linear_iterations
            );
        }
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn new_recipes_match_native_builders() {
    #[cfg(feature = "dim2")]
    {
        let points: Vec<RprVector> = [Vector::ZERO, Vector::X, Vector::ONE, Vector::Y]
            .into_iter()
            .map(Into::into)
            .collect();
        let view = RprVectorView {
            data: points.as_ptr(),
            count: 4,
        };
        same_recipe(
            &rpr_polygon_soft_body_desc(view),
            SoftBodyBuilder::polygon(vec![Vector::ZERO, Vector::X, Vector::ONE, Vector::Y]),
        );
        let short = RprVectorView {
            data: points.as_ptr(),
            count: 2,
        };
        assert!(unsafe { rpr_polygon_soft_body_desc(short).raw() }.is_err());

        let triangles = [
            RprTriangle { a: 0, b: 1, c: 2 },
            RprTriangle { a: 0, b: 2, c: 3 },
        ];
        let mut desc = rpr_default_soft_body_desc();
        let status = unsafe {
            rpr_soft_body_desc_set_trimesh(
                &mut desc,
                view,
                RprTriangleView {
                    data: triangles.as_ptr(),
                    count: 2,
                },
            )
        };
        assert_eq!(status, RPR_OK);
        same_recipe(
            &desc,
            SoftBodyBuilder::trimesh(
                vec![Vector::ZERO, Vector::X, Vector::ONE, Vector::Y],
                vec![[0, 1, 2], [0, 2, 3]],
            )
            .unwrap(),
        );
    }
    #[cfg(feature = "dim3")]
    {
        let spring = |f| RprSpringCoefficients {
            natural_frequency: f,
            damping_ratio: 1.0,
        };
        let mut desc = rpr_cloth_anisotropic_soft_body_desc(
            Vector::ZERO.into(),
            Vector::X.into(),
            Vector::Z.into(),
            3,
            4,
            spring(10.0),
            spring(20.0),
            spring(5.0),
        );
        let native = |desc: &RprSoftBodyDesc| {
            SoftBodyBuilder::cloth_anisotropic(
                Vector::ZERO,
                Vector::X,
                Vector::Z,
                3,
                4,
                SpringCoefficients::new(10.0, 1.0),
                SpringCoefficients::new(20.0, 1.0),
                SpringCoefficients::new(5.0, 1.0),
            )
            .oriented(desc.oriented.value != 0)
        };
        desc.oriented = RprOptionalBool {
            enabled: 1,
            value: 0,
        };
        same_recipe(&desc, native(&desc));
        // Per-edge overrides of the description apply after the generator's.
        let extra = [RprSoftEdgeSoftness {
            edge: 0,
            softness: spring(1.0),
        }];
        desc.edgeSoftness = RprSoftEdgeSoftnessView {
            data: extra.as_ptr(),
            count: 1,
        };
        let built = unsafe { desc.raw().unwrap() };
        assert_eq!(
            built.edge_softness.last(),
            Some(&(0, SpringCoefficients::new(1.0, 1.0)))
        );
        assert_eq!(
            built.edge_softness.len(),
            native(&desc).edge_softness.len() + 1
        );
    }
}

#[test]
fn appended_descriptions_are_merged_and_sewn() {
    let a = rpr_rope_soft_body_desc(Vector::ZERO.into(), Vector::X.into(), 3);
    let mut b = rpr_rope_soft_body_desc(Vector::Y.into(), (Vector::X + Vector::Y).into(), 4);
    b.translation = (Vector::Y * 2.0).into();
    let pieces = [b];
    let seams = [RprEdge { a: 0, b: 3 }, RprEdge { a: 2, b: 6 }];
    let mut desc = a;
    unsafe {
        assert_eq!(
            rpr_soft_body_desc_set_appended(
                &mut desc,
                RprSoftBodyDescView {
                    data: pieces.as_ptr(),
                    count: 1,
                },
            ),
            RPR_OK
        );
        assert_eq!(
            rpr_soft_body_desc_set_added_edges(
                &mut desc,
                RprEdgeView {
                    data: seams.as_ptr(),
                    count: 2,
                },
            ),
            RPR_OK
        );
    }
    let native_b =
        SoftBodyBuilder::rope(Vector::Y, Vector::X + Vector::Y, 4).translated(Vector::Y * 2.0);
    #[cfg(feature = "dim3")]
    let wire: Vec<[u32; 2]> = native_b.wire.iter().map(|w| [w[0] + 3, w[1] + 3]).collect();
    #[allow(unused_mut)]
    let mut native = SoftBodyBuilder::rope(Vector::ZERO, Vector::X, 3)
        .append(native_b)
        .add_edges([[0, 3], [2, 6]]);
    #[cfg(feature = "dim3")]
    native.wire.extend(wire);
    same_recipe(&desc, native);

    // A piece with its own particle mass keeps it.
    let mut heavy = pieces;
    heavy[0].particleMass = 3.0;
    desc.appended.data = heavy.as_ptr();
    let built = unsafe { desc.raw().unwrap() };
    assert_eq!(built.masses, vec![1.0, 1.0, 1.0, 3.0, 3.0, 3.0, 3.0]);

    // Seams are validated against the merged particles, and nesting is rejected.
    let bad = [RprEdge { a: 0, b: 7 }];
    let mut invalid = desc;
    invalid.addedEdges = RprEdgeView {
        data: bad.as_ptr(),
        count: 1,
    };
    assert!(unsafe { invalid.raw() }.is_err());
    let nested = [desc];
    let mut outer = a;
    outer.appended = RprSoftBodyDescView {
        data: nested.as_ptr(),
        count: 1,
    };
    assert!(unsafe { outer.raw() }.is_err());
}

#[cfg(feature = "dim3")]
#[test]
fn to_trimesh_ignores_subdivisions_of_shapes_without_them() {
    unsafe {
        let cuboid = rpr_cuboid_shared_shape(Vector::ONE.into());
        let mesh = rpr_shared_shape_to_trimesh(cuboid, 0, 0);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert!(!mesh.is_null());
        assert_eq!(rpr_free_tri_mesh_data(mesh), RPR_OK);
        let ball = rpr_ball_shared_shape(1.0);
        assert!(rpr_shared_shape_to_trimesh(ball, 8, 1).is_null());
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert_eq!(rpr_free_shared_shape(cuboid), RPR_OK);
        assert_eq!(rpr_free_shared_shape(ball), RPR_OK);
    }
}
