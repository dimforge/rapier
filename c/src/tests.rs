use crate::*;
#[test]
fn panic_boundary_and_thread_local_errors() {
    assert_eq!(
        crate::error::ffi(|| panic!("intentional boundary test")),
        RPR_PANIC
    );
    let message = unsafe { std::ffi::CStr::from_ptr(rpr_last_error()) }
        .to_str()
        .unwrap()
        .to_owned();
    assert!(message.contains("intentional boundary test"));
    std::thread::spawn(|| {
        assert_eq!(rpr_last_status(), RPR_OK);
        assert!(unsafe { rpr_ball_shared_shape(-1.0) }.is_null());
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert!(
            unsafe { std::ffi::CStr::from_ptr(rpr_last_error()) }
                .to_str()
                .unwrap()
                .contains("positive")
        );
    })
    .join()
    .unwrap();
    assert_eq!(
        unsafe { std::ffi::CStr::from_ptr(rpr_last_error()) }
            .to_str()
            .unwrap(),
        message
    );
    assert_eq!(unsafe { rpr_free_world(std::ptr::null_mut()) }, RPR_OK);
    assert!(
        unsafe { std::ffi::CStr::from_ptr(rpr_last_error()) }
            .to_bytes()
            .is_empty()
    );
}
#[test]
fn buffers_and_null_arrays() {
    let mut count = 99;
    let mut sentinel = [123u32, 456];
    let code = crate::error::ffi(|| unsafe {
        crate::error::copy_out(&[1, 2, 3], sentinel.as_mut_ptr(), 2, &mut count)
    });
    assert_eq!(code, RPR_BUFFER_TOO_SMALL);
    assert_eq!(count, 3);
    assert_eq!(sentinel, [123, 456]);
    assert_eq!(
        crate::error::ffi(|| unsafe {
            crate::error::input::<u32>(std::ptr::null(), 0)?;
            Ok(())
        }),
        RPR_OK
    );
    assert_eq!(
        crate::error::ffi(|| unsafe {
            crate::error::input::<u32>(std::ptr::null(), 1)?;
            Ok(())
        }),
        RPR_NULL_POINTER
    );
    assert_eq!(
        crate::error::ffi(|| unsafe {
            crate::error::input::<u32>(sentinel.as_ptr(), usize::MAX)?;
            Ok(())
        }),
        RPR_INVALID_ARGUMENT
    );
}
#[test]
fn invalid_inputs_leave_objects_unchanged() {
    unsafe {
        let mut bodies = RprRigidBodySet(RigidBodySet::new());
        let handle = bodies.0.insert(RigidBodyBuilder::dynamic()).into();
        assert_eq!(
            native_rigid_body_set_set_enabled(&mut bodies, handle, 2),
            RPR_INVALID_ARGUMENT
        );
        assert_eq!(
            native_rigid_body_set_set_linear_damping(&mut bodies, handle, Real::NAN),
            RPR_INVALID_ARGUMENT
        );
        let mut enabled = 0;
        assert_eq!(
            native_rigid_body_set_get_is_enabled(&bodies, handle, &mut enabled),
            RPR_OK
        );
        assert_eq!(enabled, 1);
    }
}

#[test]
fn build_profile_is_reported_by_the_library() {
    unsafe {
        let profile = rpr_build_profile();
        assert_eq!(
            std::ffi::CStr::from_ptr(profile).to_str().unwrap(),
            env!("RAPIER_CARGO_PROFILE")
        );
        // A later call must not invalidate the borrowed profile string.
        assert_eq!(rpr_free_world(std::ptr::null_mut()), RPR_OK);
        assert_eq!(
            std::ffi::CStr::from_ptr(profile).to_str().unwrap(),
            env!("RAPIER_CARGO_PROFILE")
        );
    }
}

#[test]
fn description_insertion_copies_values_and_preserves_ownership() {
    unsafe {
        let mut body = rpr_dynamic_rigid_body_desc();
        let world = rpr_new_world();
        assert_eq!(rpr_last_status(), RPR_OK);
        let collider = rpr_ball_collider_desc(0.5);

        let body_handle = rpr_insert_rigid_body(world, &body);
        assert_eq!(rpr_last_status(), RPR_OK);
        let mut collider_handle = rpr_insert_collider(body_handle, &collider);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(
            (&(*(*world).read().unwrap().raw()).0.colliders)[collider_handle.raw()].parent(),
            Some(body_handle.raw())
        );

        // Reusing and changing a description does not change inserted objects.
        body.position.translation = Vector::Y.into();
        let other = rpr_insert_rigid_body(world, &body);
        assert_eq!(rpr_last_status(), RPR_OK);
        rpr_insert_collider(other, &collider);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!((*(*world).read().unwrap().raw()).0.bodies.len(), 2);
        assert_eq!(
            (&(*(*world).read().unwrap().raw()).0.bodies)[body_handle.raw()].translation(),
            Vector::ZERO
        );

        let count = (*(*world).read().unwrap().raw()).0.colliders.len();
        rpr_insert_collider(RprRigidBodyHandle::default(), &collider);
        assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
        assert_eq!((*(*world).read().unwrap().raw()).0.colliders.len(), count);
        collider_handle = rpr_insert_collider_without_parent(world, &collider);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(
            (&(*(*world).read().unwrap().raw()).0.colliders)[collider_handle.raw()].parent(),
            None
        );
        rpr_insert_collider(body_handle, std::ptr::null());
        assert_eq!(rpr_last_status(), RPR_NULL_POINTER);
        assert_eq!((*(*world).read().unwrap().raw()).0.bodies.len(), 2);
        assert_eq!(
            rpr_step(world, std::ptr::null(), std::ptr::null_mut()),
            RPR_OK
        );
        assert_eq!(rpr_free_world(world), RPR_OK);

        // Constructors return descriptions even for invalid geometry. Building validates it.
        let collider = rpr_ball_collider_desc(-1.0);
        assert_eq!(collider.shape.radius, -1.0);
        let shape = rpr_shape_desc_build(&collider.shape);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert!(shape.is_null());
    }
}

#[test]
fn world_soft_body_insertion_and_particle_access() {
    unsafe {
        let world = rpr_new_world();
        assert_eq!(rpr_last_status(), RPR_OK);
        let desc = rpr_rope_soft_body_desc(Vector::ZERO.into(), Vector::X.into(), 3);
        let handle = rpr_insert_soft_body(world, &desc);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!((*(*world).read().unwrap().raw()).0.soft_bodies.len(), 1);
        let mut position: RprVector = rpr_soft_body_particle_position(handle, 2);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(position.raw().unwrap(), Vector::X);
        position = rpr_soft_body_particle_position(handle, 3);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert_eq!(position.raw().unwrap(), Vector::ZERO);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn error_handlers_are_scoped_thread_local_and_reentrant_safe() {
    struct Report {
        calls: usize,
        status: RprStatus,
        message: String,
        nested_status: RprStatus,
    }
    unsafe extern "C" fn report(
        status: RprStatus,
        message: *const std::ffi::c_char,
        user_data: *mut std::ffi::c_void,
    ) {
        let report = unsafe { &mut *user_data.cast::<Report>() };
        report.calls += 1;
        report.status = status;
        // Calling Rapier here must not recursively call the handler or invalidate message.
        report.nested_status =
            unsafe { rpr_set_gravity(std::ptr::null_mut(), RprVector::default()) };
        report.message = unsafe { std::ffi::CStr::from_ptr(message) }
            .to_string_lossy()
            .into_owned();
    }
    unsafe {
        let mut observed = Report {
            calls: 0,
            status: RPR_OK,
            message: String::new(),
            nested_status: RPR_OK,
        };
        let previous = rpr_set_error_handler(RprErrorHandler {
            callback: Some(report),
            user_data: (&mut observed as *mut Report).cast(),
        });
        assert!(previous.callback.is_none());
        let mut shape = rpr_ball_shared_shape(-1.0);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert_eq!(observed.calls, 1);
        assert_eq!(observed.status, RPR_INVALID_ARGUMENT);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert!(shape.is_null());
        assert_eq!(observed.nested_status, RPR_NULL_POINTER);
        assert!(observed.message.contains("positive"));
        assert_eq!(
            std::ffi::CStr::from_ptr(rpr_last_error()).to_string_lossy(),
            observed.message
        );
        assert_eq!(rpr_free_world(std::ptr::null_mut()), RPR_OK);
        assert_eq!(observed.calls, 1);
        std::thread::spawn(|| {
            assert_eq!(
                rpr_set_gravity(std::ptr::null_mut(), RprVector::default()),
                RPR_NULL_POINTER
            );
        })
        .join()
        .unwrap();
        assert_eq!(observed.calls, 1);
        // Description validation reports exactly one error at the build boundary.
        shape = rpr_shape_desc_build(&rpr_ball_collider_desc(-1.0).shape);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert_eq!(observed.calls, 2);
        assert!(shape.is_null());
        let installed = rpr_set_error_handler(previous);
        assert_eq!(installed.user_data, (&mut observed as *mut Report).cast());
        assert_eq!(
            rpr_set_gravity(std::ptr::null_mut(), RprVector::default()),
            RPR_NULL_POINTER
        );
        assert_eq!(observed.calls, 2);
    }
}

#[test]
fn query_predicates_filter_hits_and_misses_are_successful() {
    unsafe extern "C" fn only_selected(
        data: *mut std::ffi::c_void,
        _read: *const RprReadContext,
        handle: RprColliderHandle,
    ) -> RprBool {
        (handle == unsafe { *(data.cast::<RprColliderHandle>()) }) as RprBool
    }
    unsafe {
        let mut world = RprPhysicsWorld(PhysicsWorld::new());
        world.0.insert_collider(
            ColliderBuilder::ball(0.5).translation(Vector::X * 2.0),
            None,
        );
        let far = world.0.insert_collider(
            ColliderBuilder::ball(0.5).translation(Vector::X * 5.0),
            None,
        );
        world.0.step();
        let world = RprWorld::new(world);
        let mut query = rpr_default_query_options();
        let mut selected =
            RprColliderHandle::from(far).with_world((&world as *const RprWorld).cast_mut());
        query.predicate = Some(only_selected);
        query.userData = (&mut selected as *mut RprColliderHandle).cast();

        let value = rpr_cast_ray_toi(
            &world,
            &query,
            Vector::ZERO.into(),
            Vector::X.into(),
            10.0,
            1,
        );

        let mut collider = value.collider;
        let mut toi = value.toi;
        let mut found = value.found;
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(found, 1);
        assert!(collider == selected);
        assert!((toi - 4.5).abs() < 1.0e-5);
        let value = rpr_cast_ray_toi(
            &world,
            &query,
            Vector::ZERO.into(),
            (-Vector::X).into(),
            10.0,
            1,
        );

        collider = value.collider;
        toi = value.toi;
        found = value.found;
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(found, 0);
        assert_eq!(toi, 0.0);
        assert!(collider == RprColliderHandle::default());
        query.predicate = None;
        let value = rpr_cast_ray_toi(
            &world,
            &query,
            Vector::ZERO.into(),
            Vector::X.into(),
            10.0,
            1,
        );

        collider = value.collider;
        toi = value.toi;
        found = value.found;
        assert_eq!(rpr_last_status(), RPR_OK);
        assert!((toi - 1.5).abs() < 1.0e-5);
        assert_eq!(found, 1);
        assert_ne!(collider, RprColliderHandle::default());
    }
}

#[test]
fn pid_axes_preserve_uncontrolled_motion_and_body_state() {
    unsafe {
        let pid = rpr_new_pid_controller();
        assert_eq!(rpr_last_status(), RPR_OK);
        let mut bodies = RprRigidBodySet(RigidBodySet::new());
        let handle = bodies
            .0
            .insert(RigidBodyBuilder::dynamic().linvel(Vector::Y));
        let zero = angular_out(AngVector::default());
        let gains = RprPidGains {
            lin_kp: Vector::splat(10.0).into(),
            lin_ki: Vector::ZERO.into(),
            lin_kd: Vector::ZERO.into(),
            ang_kp: zero,
            ang_ki: zero,
            ang_kd: zero,
        };
        assert_eq!(rpr_pid_controller_set_gains(pid, gains), RPR_OK);
        assert_eq!(rpr_pid_controller_set_axes(pid, 1), RPR_OK);
        let mut linear = RprVector::default();
        let mut angular = zero;
        assert_eq!(
            native_pid_controller_rigid_body_correction(
                pid,
                0.01,
                &bodies,
                handle.into(),
                Pose::from_translation(Vector::X + Vector::Y).into(),
                Vector::ZERO.into(),
                zero,
                &mut linear,
                &mut angular
            ),
            RPR_OK
        );
        assert!(linear.x > 0.0);
        assert_eq!(linear.y, 0.0);
        assert_eq!(bodies.0[handle].translation(), Vector::ZERO);
        assert_eq!(bodies.0[handle].linvel(), Vector::Y);
        assert_eq!(rpr_free_pid_controller(pid), RPR_OK);
    }
}

#[test]
fn soft_proxy_can_be_woken_without_exposing_mutable_rigid_body() {
    unsafe {
        let world = rpr_new_world();
        assert_eq!(rpr_last_status(), RPR_OK);
        let desc = rpr_rope_soft_body_desc(Vector::ZERO.into(), Vector::X.into(), 3);
        let handle = rpr_insert_soft_body(world, &desc);
        assert_eq!(rpr_last_status(), RPR_OK);
        let proxy = rpr_soft_body_root_body(handle);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(
            rpr_rigid_body_set_translation(proxy, Vector::ZERO.into(), 1),
            RPR_INVALID_ARGUMENT
        );
        assert_eq!(rpr_rigid_body_wake_up(proxy, 1), RPR_OK);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn legacy_rigid_snapshot_preserves_state() {
    #[derive(serde::Serialize)]
    struct State<'a> {
        gravity: Vector,
        integration_parameters: &'a IntegrationParameters,
        islands: &'a IslandManager,
        broad_phase: &'a BroadPhaseBvh,
        narrow_phase: &'a NarrowPhase,
        bodies: &'a RigidBodySet,
        colliders: &'a ColliderSet,
        impulse_joints: &'a ImpulseJointSet,
        multibody_joints: &'a MultibodyJointSet,
    }
    let mut original = PhysicsWorld::new();
    original.insert(
        RigidBodyBuilder::dynamic().translation(Vector::Y * 3.0),
        ColliderBuilder::ball(0.5),
    );
    original.step();
    let bytes = bincode::serialize(&State {
        gravity: original.gravity,
        integration_parameters: &original.integration_parameters,
        islands: &original.islands,
        broad_phase: &original.broad_phase,
        narrow_phase: &original.narrow_phase,
        bodies: &original.bodies,
        colliders: &original.colliders,
        impulse_joints: &original.impulse_joints,
        multibody_joints: &original.multibody_joints,
    })
    .unwrap();
    unsafe {
        let restored = rpr_deserialize_rigid_state(bytes.as_ptr(), bytes.len());
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!((*(*restored).read().unwrap().raw()).0.bodies.len(), 1);
        original.step();
        assert_eq!(
            rpr_step(restored, std::ptr::null(), std::ptr::null()),
            RPR_OK
        );
        let a = original.bodies.iter().next().unwrap().1;
        let access = (*restored).read().unwrap();
        let b = (*access.raw()).0.bodies.iter().next().unwrap().1;
        assert_eq!(a.position(), b.position());
        assert_eq!(a.linvel(), b.linvel());
        drop(access);
        assert_eq!(rpr_free_world(restored), RPR_OK);
    }
}

#[cfg(feature = "dim3")]
#[test]
fn render_only_meshes_have_distinct_ids_and_no_collider_cache_key() {
    let mut world = PhysicsWorld::new();
    let (vertices, indices) = rapier::parry::shape::Ball::new(0.75).to_trimesh(8, 8);
    let handle = world.insert_soft_body(
        SoftBodyBuilder::cuboid(Vector::ZERO, Vector::ONE, 3, 3, 3)
            .skin(vertices, indices)
            .skin_collision(false),
    );
    let body = world.soft_bodies.get(handle).unwrap();
    let native: Vec<_> = body.meshes().collect();
    assert!(
        native
            .iter()
            .any(|m| m.is_skinned() && m.collider() == ColliderHandle::invalid())
    );
    let body = (body as *const SoftBody).cast::<RprSoftBody>();
    unsafe {
        let mut count = 0;
        assert_eq!(
            native_soft_body_meshes(body, std::ptr::null_mut(), 0, &mut count),
            RPR_OK
        );
        assert_eq!(count, native.len());
        let mut meshes = Vec::<RprSoftMeshInfo>::with_capacity(count);
        assert_eq!(
            native_soft_body_meshes(body, meshes.as_mut_ptr(), count, &mut count),
            RPR_OK
        );
        meshes.set_len(count);
        for (i, mesh) in meshes.iter().enumerate() {
            assert!(!meshes[..i].iter().any(|other| other.id == mesh.id));
            let mut nv = 0;
            assert_eq!(
                native_soft_body_mesh_vertices_by_id(
                    body,
                    mesh.id,
                    std::ptr::null_mut(),
                    0,
                    &mut nv
                ),
                RPR_OK
            );
            let mut vertices = vec![RprVector::default(); nv];
            assert_eq!(
                native_soft_body_mesh_vertices_by_id(
                    body,
                    mesh.id,
                    vertices.as_mut_ptr(),
                    nv,
                    &mut nv
                ),
                RPR_OK
            );
            assert_eq!(
                vertices
                    .iter()
                    .map(|v| v.raw().unwrap())
                    .collect::<Vec<_>>(),
                native[i]
                    .vertex_positions(&world.soft_bodies[handle])
                    .collect::<Vec<_>>()
            );
            let mut ni = 0;
            assert_eq!(
                native_soft_body_mesh_indices_by_id(
                    body,
                    mesh.id,
                    std::ptr::null_mut(),
                    0,
                    &mut ni
                ),
                RPR_OK
            );
            let mut indices = vec![0; ni];
            assert_eq!(
                native_soft_body_mesh_indices_by_id(
                    body,
                    mesh.id,
                    indices.as_mut_ptr(),
                    ni,
                    &mut ni
                ),
                RPR_OK
            );
            assert_eq!(indices, native[i].indices().as_flattened());
            assert!(indices.iter().all(|&v| (v as usize) < nv));
        }
        assert_eq!(
            native_soft_body_mesh_vertices(
                body,
                RprColliderHandle::default(),
                std::ptr::null_mut(),
                0,
                &mut count
            ),
            RPR_INVALID_ARGUMENT
        );
        assert_eq!(
            native_soft_body_mesh_colliders(body, std::ptr::null_mut(), 0, &mut count),
            RPR_OK
        );
        let mut handles = vec![RprColliderHandle::default(); count];
        assert_eq!(
            native_soft_body_mesh_colliders(body, handles.as_mut_ptr(), count, &mut count),
            RPR_OK
        );
        assert!(handles.iter().all(|&h| h != RprColliderHandle::default()));
        assert_eq!(
            native_soft_body_mesh_vertices_by_id(
                body,
                RprSoftMeshId {
                    cluster: u32::MAX,
                    mesh: 0
                },
                std::ptr::null_mut(),
                0,
                &mut count
            ),
            RPR_INVALID_HANDLE
        );
    }
}

#[test]
fn value_returns_preserve_status_until_the_next_fallible_call() {
    unsafe {
        assert!(rpr_ball_shared_shape(-1.0).is_null());
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        let message = std::ffi::CStr::from_ptr(rpr_last_error()).to_owned();
        let _ = rpr_build_info();
        let _ = rpr_dynamic_rigid_body_desc();
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        assert_eq!(
            std::ffi::CStr::from_ptr(rpr_last_error()),
            message.as_c_str()
        );
        let world = rpr_new_world();
        assert!(!world.is_null());
        assert_eq!(rpr_last_status(), RPR_OK);
        assert!(
            std::ffi::CStr::from_ptr(rpr_last_error())
                .to_bytes()
                .is_empty()
        );
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn value_returns_discard_partial_values_after_a_panic() {
    let value: RprVector = ffi_value(|out| {
        ffi(|| {
            unsafe {
                output(out, Vector::X.into())?;
            }
            panic!("value return boundary test");
        })
    });
    assert_eq!(value.raw().unwrap(), Vector::ZERO);
    assert_eq!(rpr_last_status(), RPR_PANIC);
}
