use crate::*;
// Internal transparent adapter for native PhysicsWorld operations.
#[repr(transparent)]
pub(crate) struct RprPhysicsWorld(pub(crate) PhysicsWorld);

// Internal transparent adapter for native PhysicsPipeline operations.
#[repr(transparent)]
pub(crate) struct RprPhysicsPipeline(pub(crate) PhysicsPipeline);

// Internal transparent adapter for native IslandManager operations.
#[repr(transparent)]
pub(crate) struct RprIslandManager(pub(crate) IslandManager);

// Internal transparent adapter for native BroadPhaseBvh operations.
#[repr(transparent)]
pub(crate) struct RprBroadPhaseBvh(pub(crate) BroadPhaseBvh);

// Internal transparent adapter for native NarrowPhase operations.
#[repr(transparent)]
pub(crate) struct RprNarrowPhase(pub(crate) NarrowPhase);

// Internal transparent adapter for native RigidBodySet operations.
#[repr(transparent)]
pub(crate) struct RprRigidBodySet(pub(crate) RigidBodySet);

// Internal transparent adapter for native ColliderSet operations.
#[repr(transparent)]
pub(crate) struct RprColliderSet(pub(crate) ColliderSet);

// Internal transparent adapter for native ImpulseJointSet operations.
#[repr(transparent)]
pub(crate) struct RprImpulseJointSet(pub(crate) ImpulseJointSet);

// Internal transparent adapter for native MultibodyJointSet operations.
#[repr(transparent)]
pub(crate) struct RprMultibodyJointSet(pub(crate) MultibodyJointSet);

// Internal transparent adapter for native SoftBodySet operations.
#[repr(transparent)]
pub(crate) struct RprSoftBodySet(pub(crate) SoftBodySet);

// Internal transparent adapter for native IntegrationParameters operations.
#[repr(transparent)]
pub(crate) struct NativeIntegrationParameters(pub(crate) IntegrationParameters);

// Internal transparent adapter for native RigidBody operations.
#[repr(transparent)]
pub(crate) struct RprRigidBody(pub(crate) RigidBody);

// Internal transparent adapter for native Collider operations.
#[repr(transparent)]
pub(crate) struct RprCollider(pub(crate) Collider);

// Internal transparent adapter for native GenericJoint operations.
#[repr(transparent)]
pub(crate) struct RprGenericJoint(pub(crate) GenericJoint);

/// Opaque SharedShape. See the ownership and borrowing contract in README.md.
/// @ingroup shapes
#[repr(transparent)]
pub struct RprSharedShape(pub(crate) SharedShape);
/// Frees an owned object; NULL is allowed. Never free a borrowed pointer.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_free_shared_shape(object: *mut RprSharedShape) -> RprStatus {
    ffi(|| unsafe {
        if !object.is_null() {
            get(object)?;
            drop(Box::from_raw(object));
        }
        Ok(())
    })
}

/// Create an owned wrapper sharing the same immutable geometry. Release it with
/// rpr_free_shared_shape.
/// @ingroup shapes
#[rapier_export(shared_shape)]
pub unsafe extern "C" fn rpr_shared_shape_clone(
    object: *const RprSharedShape,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let value = get(object)?.0.clone();
            output(out, Box::into_raw(Box::new(RprSharedShape(value))))
        })
    })
}

// Internal transparent adapter for native SoftBody operations.
#[repr(transparent)]
pub(crate) struct RprSoftBody(pub(crate) SoftBody);

/// Return the number of rigid body objects in the world.
/// @ingroup rigid_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_rigid_body_count(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_len(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_len(
    object: *const RprRigidBodySet,
    out: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.len())
    })
}

/// Copy entity handles.
/// @see @ref output_buffers
/// @ingroup rigid_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_rigid_body_handles(
    world: *const RprWorld,
    buffer: *mut RprRigidBodyHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();

                crate::handle_access::forward(native_rigid_body_set_handles(
                    std::ptr::addr_of!((*raw).0.bodies).cast(),
                    buffer,
                    capacity,
                    count,
                ))
            })
        })
    }
}

pub(crate) unsafe fn native_rigid_body_set_handles(
    set: *const RprRigidBodySet,
    buffer: *mut RprRigidBodyHandle,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let values: Vec<_> = get(set)?.0.iter().map(|(h, _)| h.into()).collect();
        copy_out(&values, buffer, capacity, count)
    })
}

/// Test whether the live world contains this rigid body handle. A removed/stale handle returns
/// false.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_contains(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_contains(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_contains(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe { output(out, get(set)?.0.get(handle.raw()).is_some() as u32) })
}

/// Return the number of collider objects in the world.
/// @ingroup colliders
#[rapier_export]
pub unsafe extern "C" fn rpr_collider_count(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_len(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_len(
    object: *const RprColliderSet,
    out: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.len())
    })
}

/// Copy entity handles.
/// @see @ref output_buffers
/// @ingroup colliders
#[rapier_export]
pub unsafe extern "C" fn rpr_collider_handles(
    world: *const RprWorld,
    buffer: *mut RprColliderHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();

                crate::handle_access::forward(native_collider_set_handles(
                    std::ptr::addr_of!((*raw).0.colliders).cast(),
                    buffer,
                    capacity,
                    count,
                ))
            })
        })
    }
}

pub(crate) unsafe fn native_collider_set_handles(
    set: *const RprColliderSet,
    buffer: *mut RprColliderHandle,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let values: Vec<_> = get(set)?.0.iter().map(|(h, _)| h.into()).collect();
        copy_out(&values, buffer, capacity, count)
    })
}

/// Test whether the live world contains this collider handle. A removed/stale handle returns false.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_contains(handle: RprColliderHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_contains(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_contains(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe { output(out, get(set)?.0.get(handle.raw()).is_some() as u32) })
}

/// Return the number of soft body objects in the world.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_soft_body_count(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let object = get(object)?;
            output(out, object.0.len())
        })
    })
}

/// Copy entity handles.
/// @see @ref output_buffers
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_soft_body_handles(
    world: *const RprWorld,
    buffer: *mut RprSoftBodyHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();

                let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

                let values: Vec<_> = get(set)?.0.iter().map(|(h, _)| h.into()).collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    }
}

/// Test whether the live world contains this soft body handle. A removed/stale handle returns
/// false.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_contains(handle: RprSoftBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();
            output(out, get(set)?.0.get(handle.raw()).is_some() as u32)
        })
    })
}

/// Remove a body and its joints, optionally keeping colliders as standalone objects.
/// Returns whether a body was removed; a stale handle returns false without error.
/// @ingroup rigid_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_remove_rigid_body(
    handle: RprRigidBodyHandle,
    remove_attached_colliders: RprBool,
) -> RprBool {
    let world = handle.world;
    ffi_value(|removed: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let remove = boolean(remove_attached_colliders)?;
            if !removed.is_null() {
                out_ptr(removed)?;
            }
            let access = get(world)?.write()?;
            let world = &mut (*access.raw()).0;
            if let Some(body) = world.bodies.get(handle.raw()) {
                ensure(
                    body.soft_body().is_none() || body.is_soft_frame(),
                    "remove a soft-body root through RemoveSoftBody",
                )?;
            }
            let did_remove = world
                .remove_body_with_colliders(handle.raw(), remove)
                .is_some();
            if !removed.is_null() {
                output(removed, did_remove as RprBool)?;
            }
            Ok(())
        })
    })
}
