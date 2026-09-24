use crate::*;
use std::{
    ffi::c_void,
    ptr,
    sync::atomic::{AtomicUsize, Ordering},
};

struct CallbackState {
    world: *mut RprWorld,
    calls: AtomicUsize,
}
unsafe fn check_hook_access(
    data: *mut c_void,
    read: *const RprReadContext,
    collider: RprColliderHandle,
) {
    unsafe {
        let state = &*data.cast::<CallbackState>();
        let _position = rpr_read_collider_translation(read, collider);
        assert_eq!(rpr_last_status(), RPR_OK);
        let _count = rpr_collider_count(state.world);
        assert_eq!(rpr_last_status(), RPR_WORLD_BUSY);
        assert_eq!(
            rpr_set_gravity(state.world, Vector::ZERO.into()),
            RPR_WORLD_BUSY
        );
        assert_eq!(
            rpr_step(state.world, ptr::null(), ptr::null()),
            RPR_WORLD_BUSY
        );
        assert_eq!(rpr_free_world(state.world), RPR_WORLD_BUSY);
        state.calls.fetch_add(1, Ordering::Relaxed);
    }
}
unsafe extern "C" fn filter_pair(
    data: *mut c_void,
    read: *const RprReadContext,
    a: RprColliderHandle,
    _b: RprColliderHandle,
    _ba: RprRigidBodyHandle,
    _bb: RprRigidBodyHandle,
) -> i32 {
    unsafe { check_hook_access(data, read, a) };
    1
}
unsafe extern "C" fn modify_contacts(
    data: *mut c_void,
    read: *const RprReadContext,
    a: RprColliderHandle,
    _b: RprColliderHandle,
    context: *mut RprContactModificationContext,
) {
    unsafe {
        check_hook_access(data, read, a);
        assert_eq!(
            rpr_contact_modification_context_set_tangent_velocity(context, Vector::ZERO.into()),
            RPR_OK
        );
    }
}

#[test]
fn hooks_read_scoped_state_and_reject_world_reentry() {
    unsafe {
        let world = rpr_new_world();
        assert_eq!(rpr_last_status(), RPR_OK);
        let mut collider = rpr_ball_collider_desc(1.0);
        collider.activeHooks = 1 | 4;
        let fixed = rpr_fixed_rigid_body_desc();
        let mut dynamic = rpr_dynamic_rigid_body_desc();
        dynamic.position.translation = Vector::Y.into();
        let fixed_handle = rpr_insert_rigid_body(world, &fixed);
        assert_eq!(rpr_last_status(), RPR_OK);
        rpr_insert_collider(fixed_handle, &collider);
        assert_eq!(rpr_last_status(), RPR_OK);
        let dynamic_handle = rpr_insert_rigid_body(world, &dynamic);
        assert_eq!(rpr_last_status(), RPR_OK);
        rpr_insert_collider(dynamic_handle, &collider);
        assert_eq!(rpr_last_status(), RPR_OK);
        let mut state = CallbackState {
            world,
            calls: AtomicUsize::new(0),
        };
        let hooks = RprPhysicsHooks {
            user_data: (&mut state as *mut CallbackState).cast(),
            filter_contact_pair: Some(filter_pair),
            modify_solver_contacts_context: Some(modify_contacts),
            ..Default::default()
        };
        assert_eq!(rpr_step(world, &hooks, ptr::null()), RPR_OK);
        assert!(state.calls.load(Ordering::Relaxed) >= 2);
        // The step released its exclusive access, so the deferred change now succeeds.
        assert_eq!(rpr_set_gravity(world, Vector::ZERO.into()), RPR_OK);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

unsafe extern "C" fn query_predicate(
    data: *mut c_void,
    read: *const RprReadContext,
    collider: RprColliderHandle,
) -> RprBool {
    unsafe {
        let state = &*data.cast::<CallbackState>();
        let _position = rpr_read_collider_translation(read, collider);
        assert_eq!(rpr_last_status(), RPR_OK);
        let count = rpr_collider_count(state.world);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(count, 1);
        assert_eq!(
            rpr_set_gravity(state.world, Vector::ZERO.into()),
            RPR_WORLD_BUSY
        );
        assert_eq!(rpr_free_world(state.world), RPR_WORLD_BUSY);

        // Read-only queries can reenter, provided the nested query doesn't recurse indefinitely.
        let hit = rpr_cast_ray(
            state.world,
            ptr::null(),
            Vector::ZERO.into(),
            Vector::X.into(),
            10.0,
            1,
        );
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(hit.collider, collider);
        state.calls.fetch_add(1, Ordering::Relaxed);
        1
    }
}

#[test]
fn query_options_are_owner_independent_and_nested_reads_are_allowed() {
    unsafe {
        let world = rpr_new_world();
        assert_eq!(rpr_last_status(), RPR_OK);
        let mut desc = rpr_ball_collider_desc(0.5);
        desc.position.translation = (Vector::X * 2.0).into();
        rpr_insert_collider_without_parent(world, &desc);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(
            rpr_detect_collisions(world, ptr::null(), ptr::null()),
            RPR_OK
        );
        let mut state = CallbackState {
            world,
            calls: AtomicUsize::new(0),
        };
        let mut options = rpr_default_query_options();
        options.predicate = Some(query_predicate);
        options.userData = (&mut state as *mut CallbackState).cast();
        let _hit = rpr_cast_ray(
            world,
            &options,
            Vector::ZERO.into(),
            Vector::X.into(),
            10.0,
            1,
        );
        assert_eq!(rpr_last_status(), RPR_OK);
        assert!(state.calls.load(Ordering::Relaxed) > 0);
        assert_eq!(rpr_set_gravity(world, Vector::ZERO.into()), RPR_OK);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn world_gate_checks_conflicts_across_threads_and_releases_after_errors() {
    fn thread_safe<T: Send + Sync>() {}
    thread_safe::<PhysicsWorld>();
    let world = RprWorld::new(RprPhysicsWorld(PhysicsWorld::new()));
    let read = world.read().unwrap();
    std::thread::scope(|scope| {
        scope
            .spawn(|| {
                let count = unsafe { rpr_rigid_body_count(&world) };
                assert_eq!(rpr_last_status(), RPR_OK);
                assert_eq!(count, 0);
                assert!(world.write().is_err());
            })
            .join()
            .unwrap();
    });
    drop(read);
    let write = world.write().unwrap();
    assert!(world.read().is_err());
    assert!(world.write().is_err());
    drop(write);
    assert_eq!(
        ffi(|| {
            let _write = world.write()?;
            panic!("release the guard");
        }),
        RPR_PANIC
    );
    assert!(world.write().is_ok());
}

#[test]
fn removing_a_body_can_preserve_its_colliders() {
    unsafe {
        let world = rpr_new_world();
        assert_eq!(rpr_last_status(), RPR_OK);

        let desc = rpr_dynamic_rigid_body_desc();
        let shape = rpr_ball_collider_desc(0.5);
        let body = rpr_insert_rigid_body(world, &desc);
        assert_eq!(rpr_last_status(), RPR_OK);
        let collider = rpr_insert_collider(body, &shape);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(rpr_remove_rigid_body(body, 0), RPR_OK);
        let parent = rpr_collider_parent(collider);
        assert_eq!(rpr_last_status(), RPR_OK);
        assert_eq!(parent, RprRigidBodyHandle::default());
        assert_eq!(rpr_rigid_body_validate_handle(body), RPR_INVALID_HANDLE);
        assert_eq!(rpr_remove_rigid_body(body, 1), RPR_INVALID_HANDLE);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}
