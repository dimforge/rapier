use crate::*;
use std::ptr;

fn ok() {
    assert_eq!(rpr_last_status(), RPR_OK);
}

#[test]
fn collider_insertion_checks_parent_and_preserves_world_on_failure() {
    unsafe {
        let world = rpr_new_world();
        ok();
        let parent = rpr_insert_rigid_body(world, &rpr_dynamic_rigid_body_desc());
        ok();
        let desc = rpr_ball_collider_desc(0.5);
        let attached = rpr_insert_collider(parent, &desc);
        ok();
        assert_eq!(rpr_collider_parent(attached), parent);
        ok();
        let standalone = rpr_insert_collider_without_parent(world, &desc);
        ok();
        assert_eq!(standalone.world, world);
        assert_eq!(
            rpr_collider_parent(standalone),
            RprRigidBodyHandle::default()
        );
        ok();
        assert_eq!(rpr_remove_rigid_body(parent, 1), 1);
        ok();
        assert_eq!(rpr_collider_count(world), 1);
        ok();
        let failed = rpr_insert_collider(parent, &desc);
        assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
        assert_eq!(failed, RprColliderHandle::default());
        assert_eq!(rpr_collider_count(world), 1);
        ok();
        let failed = rpr_insert_collider(RprRigidBodyHandle::default(), &desc);
        assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
        assert_eq!(failed, RprColliderHandle::default());
        let failed = rpr_insert_collider_without_parent(ptr::null_mut(), &desc);
        assert_eq!(rpr_last_status(), RPR_NULL_POINTER);
        assert_eq!(failed, RprColliderHandle::default());
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn equal_native_indices_in_different_worlds_remain_distinct() {
    unsafe {
        let a = rpr_new_world();
        let b = rpr_new_world();
        let mut desc = rpr_dynamic_rigid_body_desc();
        desc.position.translation.x = 1.0;
        let first_body = rpr_insert_rigid_body(a, &desc);
        ok();
        let first_collider = rpr_insert_collider(first_body, &rpr_ball_collider_desc(0.5));
        ok();
        desc.position.translation.x = 7.0;
        let other_body = rpr_insert_rigid_body(b, &desc);
        ok();
        let other_collider = rpr_insert_collider(other_body, &rpr_ball_collider_desc(0.5));
        ok();
        assert_eq!(first_body.world, a);
        assert_eq!(first_collider.world, a);
        assert_eq!(other_body.world, b);
        assert_eq!(first_body.index, other_body.index);
        assert_eq!(first_body.generation, other_body.generation);
        assert_ne!(first_body, other_body);
        assert_eq!(rpr_rigid_body_translation(first_body).x, 1.0);
        ok();
        assert_eq!(rpr_rigid_body_translation(other_body).x, 7.0);
        ok();
        assert_eq!(rpr_collider_parent(first_collider), first_body);
        ok();
        let failed = rpr_insert_impulse_joint(first_body, other_body, &rpr_fixed_joint_desc());
        assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
        assert_eq!(failed, RprImpulseJointHandle::default());
        assert_eq!(rpr_impulse_joint_handles(a, ptr::null_mut(), 0), 0);
        ok();
        let attached = rpr_insert_collider(other_body, &rpr_ball_collider_desc(0.5));
        ok();
        assert_eq!(attached.world, b);
        assert_eq!(rpr_collider_parent(attached), other_body);
        ok();
        assert_eq!(rpr_collider_count(b), 2);
        ok();
        assert_eq!(rpr_collider_count(a), 1);
        ok();
        let mut options = rpr_default_query_options();
        options.filter.exclude_collider = other_collider;
        rpr_cast_ray(a, &options, RprVector::default(), Vector::X.into(), 10.0, 1);
        assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
        assert_eq!(rpr_free_world(a), RPR_OK);
        assert_eq!(rpr_free_world(b), RPR_OK);
    }
}

#[test]
fn returned_arrays_joints_and_query_hits_keep_the_owner() {
    unsafe {
        let world = rpr_new_world();
        let fixed = rpr_insert_rigid_body(world, &rpr_fixed_rigid_body_desc());
        ok();
        let mut desc = rpr_dynamic_rigid_body_desc();
        desc.position.translation.x = 3.0;
        let item_body = rpr_insert_rigid_body(world, &desc);
        ok();
        let item_collider = rpr_insert_collider(item_body, &rpr_ball_collider_desc(0.5));
        ok();
        let joint = rpr_insert_impulse_joint(fixed, item_body, &rpr_fixed_joint_desc());
        ok();
        assert_eq!(joint.world, world);
        let ends = rpr_impulse_joint_bodies(joint);
        ok();
        assert_eq!(ends.body1, fixed);
        assert_eq!(ends.body2, item_body);
        let mut list = [RprRigidBodyHandle::default(); 2];
        assert_eq!(
            rpr_rigid_body_handles(world, list.as_mut_ptr(), list.len()),
            2
        );
        ok();
        assert!(list.iter().all(|h| h.world == world));
        let sentinel = list[0];
        assert_eq!(rpr_rigid_body_handles(world, list.as_mut_ptr(), 1), 2);
        assert_eq!(rpr_last_status(), RPR_BUFFER_TOO_SMALL);
        assert_eq!(list[0], sentinel);
        let mut colliders = [RprColliderHandle::default(); 1];
        assert_eq!(
            rpr_rigid_body_colliders(item_body, colliders.as_mut_ptr(), 1),
            1
        );
        ok();
        assert_eq!(colliders[0], item_collider);
        assert_eq!(
            rpr_detect_collisions(world, ptr::null(), ptr::null()),
            RPR_OK
        );
        let hit = rpr_cast_ray(
            world,
            ptr::null(),
            Vector::ZERO.into(),
            Vector::X.into(),
            10.0,
            1,
        );
        ok();
        assert_eq!(hit.collider, item_collider);
        assert_eq!(rpr_remove_rigid_body(item_body, 1), 1);
        ok();
        rpr_rigid_body_translation(item_body);
        assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
        assert_eq!(rpr_free_world(world), RPR_OK);
    }
}

#[test]
fn restored_world_returns_fresh_owners_and_survives_source_destruction() {
    unsafe {
        let original = rpr_new_world();
        let mut desc = rpr_dynamic_rigid_body_desc();
        desc.position.translation.x = 5.0;
        let old = rpr_insert_rigid_body(original, &desc);
        ok();
        let snapshot = rpr_serialize_world(original);
        ok();
        let bytes = rpr_bytes_data(snapshot);
        ok();
        let restored = rpr_deserialize_world(bytes.data, bytes.count);
        ok();
        let mut fresh = RprRigidBodyHandle::default();
        assert_eq!(rpr_rigid_body_handles(restored, &mut fresh, 1), 1);
        ok();
        assert_eq!(fresh.world, restored);
        assert_eq!(fresh.index, old.index);
        assert_eq!(fresh.generation, old.generation);
        assert_ne!(fresh, old);
        assert_eq!(rpr_free_world(original), RPR_OK);
        assert_eq!(rpr_rigid_body_translation(fresh).x, 5.0);
        ok();
        assert_eq!(rpr_free_bytes(snapshot), RPR_OK);
        assert_eq!(rpr_free_world(restored), RPR_OK);
    }
}

#[test]
fn callback_and_accumulated_event_handles_keep_their_original_worlds() {
    unsafe extern "C" fn filter(
        data: *mut std::ffi::c_void,
        read: *const RprReadContext,
        a: RprColliderHandle,
        b: RprColliderHandle,
        body_a: RprRigidBodyHandle,
        body_b: RprRigidBodyHandle,
    ) -> i32 {
        let world = data.cast::<RprWorld>();
        assert_eq!(a.world, world);
        assert_eq!(b.world, world);
        assert_eq!(body_a.world, world);
        assert_eq!(body_b.world, world);
        unsafe {
            rpr_read_collider_translation(read, a);
        }
        ok();
        unsafe {
            rpr_collider_translation(a);
        }
        assert_eq!(rpr_last_status(), RPR_WORLD_BUSY);
        1
    }
    unsafe {
        let events = rpr_new_event_collector();
        ok();
        let worlds = [rpr_new_world(), rpr_new_world()];
        for &world in &worlds {
            let mut shape = rpr_ball_collider_desc(1.0);
            shape.activeEvents = RPR_COLLISION_EVENTS;
            shape.activeHooks = RPR_FILTER_CONTACT_PAIRS;
            let fixed = rpr_insert_rigid_body(world, &rpr_fixed_rigid_body_desc());
            ok();
            rpr_insert_collider(fixed, &shape);
            ok();
            let dynamic = rpr_insert_rigid_body(world, &rpr_dynamic_rigid_body_desc());
            ok();
            rpr_insert_collider(dynamic, &shape);
            ok();
            let hooks = RprPhysicsHooks {
                user_data: world.cast(),
                filter_contact_pair: Some(filter),
                ..Default::default()
            };
            assert_eq!(rpr_step(world, &hooks, events), RPR_OK);
        }
        let mut copied = [RprCollisionEvent::default(); 2];
        assert_eq!(
            rpr_event_collector_collision_events(events, copied.as_mut_ptr(), 2),
            2
        );
        ok();
        for (e, &world) in copied.iter().zip(&worlds) {
            assert_eq!(e.collider1.world, world);
            assert_eq!(e.collider2.world, world);
            assert_eq!(rpr_collider_contains(e.collider1), 1);
            ok();
        }
        assert_eq!(rpr_free_event_collector(events), RPR_OK);
        for world in worlds {
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }
}

#[test]
fn invalid_event_alignment_is_reported_before_reading_owner() {
    unsafe {
        let event = ptr::without_provenance::<RprSoftBodyTearEvent>(1);
        assert_eq!(
            rpr_soft_body_tear_event_soft_body(event),
            RprSoftBodyHandle::default()
        );
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
    }
}
