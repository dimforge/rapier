//! Short-lived element access through generational handles. No element pointer escapes.
//! Handles remain scoped to the original set; they do not keep the set/world alive.
use crate::*;

// Delegate to the existing accessors so their validation and change semantics stay identical.
// ffi suppresses nested error callbacks; the outermost call reports each error once.
pub(crate) unsafe fn forward(status: RprStatus) -> Result {
    if status == RPR_OK {
        Ok(())
    } else {
        Err((
            status,
            unsafe { std::ffi::CStr::from_ptr(rpr_last_error()) }
                .to_string_lossy()
                .into_owned(),
        ))
    }
}

/// Return the rigid body world-space pose.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_position(handle: RprRigidBodyHandle) -> RprPose {
    let world = handle.world;
    ffi_value(|out: *mut RprPose| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_position(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_position(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_position(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}

/// Return the rigid body world-space translation.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_translation(handle: RprRigidBodyHandle) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_translation(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_translation(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_translation(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}

/// Return the rigid body world-space linear velocity.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_linvel(handle: RprRigidBodyHandle) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_linvel(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_linvel(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_linvel(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}

/// Return the rigid body world-space angular velocity (radians per second).
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_angvel(handle: RprRigidBodyHandle) -> RprAngVector {
    let world = handle.world;
    ffi_value(|out: *mut RprAngVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_angvel(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_angvel(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprAngVector,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_angvel(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}

/// Return whether the rigid body is sleeping.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_is_sleeping(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_is_sleeping(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_is_sleeping(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_is_sleeping(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}

/// Return whether the rigid body is enabled.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_is_enabled(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_is_enabled(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_is_enabled(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_is_enabled(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}

/// Return the rigid body application-owned 128-bit user value.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_user_data(handle: RprRigidBodyHandle) -> RprUserData {
    let world = handle.world;
    ffi_value(|out: *mut RprUserData| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_user_data(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_user_data(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprUserData,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_user_data(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}

/// Set the rigid body world-space pose.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_position(
    handle: RprRigidBodyHandle,
    value: RprPose,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_position(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Set the rigid body world-space translation.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_translation(
    handle: RprRigidBodyHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_translation(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Set the rigid body world-space linear velocity.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_linvel(
    handle: RprRigidBodyHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_linvel(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Set the rigid body world-space angular velocity (radians per second).
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_angvel(
    handle: RprRigidBodyHandle,
    value: RprAngVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_angvel(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Set the rigid body next kinematic world-space pose.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_next_kinematic_position(
    handle: RprRigidBodyHandle,
    value: RprPose,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_next_kinematic_position(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Set the rigid body next kinematic world-space translation.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_next_kinematic_translation(
    handle: RprRigidBodyHandle,
    value: RprVector,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_next_kinematic_translation(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Set the rigid body gravity multiplier.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_gravity_scale(
    handle: RprRigidBodyHandle,
    value: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_gravity_scale(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Set the rigid body linear damping coefficient.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_linear_damping(
    handle: RprRigidBodyHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        crate::handle_access::forward(native_rigid_body_set_set_linear_damping(
            std::ptr::addr_of_mut!((*raw).0.bodies).cast(),
            handle,
            value,
        ))
    })
}

pub(crate) unsafe fn native_rigid_body_set_set_linear_damping(
    set: *mut RprRigidBodySet,
    handle: RprRigidBodyHandle,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_linear_damping(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Set the rigid body angular damping coefficient.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_angular_damping(
    handle: RprRigidBodyHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_angular_damping(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Enable or disable the rigid body.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_enabled(
    handle: RprRigidBodyHandle,
    value: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        crate::handle_access::forward(native_rigid_body_set_set_enabled(
            std::ptr::addr_of_mut!((*raw).0.bodies).cast(),
            handle,
            value,
        ))
    })
}

pub(crate) unsafe fn native_rigid_body_set_set_enabled(
    set: *mut RprRigidBodySet,
    handle: RprRigidBodyHandle,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_enabled(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Set the rigid body application-owned 128-bit user value.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_user_data(
    handle: RprRigidBodyHandle,
    value: RprUserData,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_user_data(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Apply a world-space linear impulse.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_apply_impulse(
    handle: RprRigidBodyHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_apply_impulse(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Apply a world-space impulse at a world-space point.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_apply_impulse_at_point(
    handle: RprRigidBodyHandle,
    value: RprVector,
    point: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_apply_impulse_at_point(
            (element as *mut RigidBody).cast(),
            value,
            point,
            wake_up,
        ))
    })
}

/// Accumulate a world-space force; it persists until reset.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_add_force(
    handle: RprRigidBodyHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_add_force(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Clear accumulated user forces.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_reset_forces(
    handle: RprRigidBodyHandle,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_reset_forces(
            (element as *mut RigidBody).cast(),
            wake_up,
        ))
    })
}

/// Put the body to sleep.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_sleep(handle: RprRigidBodyHandle) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_sleep((element as *mut RigidBody).cast()))
    })
}

/// Return the collider world-space pose.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_position(handle: RprColliderHandle) -> RprPose {
    let world = handle.world;
    ffi_value(|out: *mut RprPose| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_position(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_position(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_position(
            (element as *const Collider).cast(),
            out,
        ))
    })
}

/// Return the collider world-space translation.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_translation(handle: RprColliderHandle) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_translation(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_translation(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_translation(
            (element as *const Collider).cast(),
            out,
        ))
    })
}

/// Return the collider friction coefficient.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_friction(handle: RprColliderHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_friction(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_friction(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_friction(
            (element as *const Collider).cast(),
            out,
        ))
    })
}

/// Return the collider restitution coefficient.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_restitution(handle: RprColliderHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_restitution(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_restitution(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_restitution(
            (element as *const Collider).cast(),
            out,
        ))
    })
}

/// Return whether the collider is a sensor (detects overlaps without contact forces).
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_is_sensor(handle: RprColliderHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_is_sensor(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_is_sensor(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_is_sensor(
            (element as *const Collider).cast(),
            out,
        ))
    })
}

/// Return the parent body handle, or an invalid handle with OK status for a standalone collider.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_parent(handle: RprColliderHandle) -> RprRigidBodyHandle {
    let world = handle.world;
    ffi_world_value(world, |out: *mut RprRigidBodyHandle| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_parent(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_parent(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprRigidBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_parent(
            (element as *const Collider).cast(),
            out,
        ))
    })
}

/// Set the collider world-space pose.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_position(
    handle: RprColliderHandle,
    value: RprPose,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_position(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Set the collider world-space translation.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_translation(
    handle: RprColliderHandle,
    value: RprVector,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_translation(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Set the collider friction coefficient.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_friction(
    handle: RprColliderHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_friction(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Set the collider restitution coefficient.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_restitution(
    handle: RprColliderHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_restitution(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Enable or disable a sensor (detects overlaps without contact forces) for the collider.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_sensor(
    handle: RprColliderHandle,
    value: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_sensor(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Set the collider collision filtering groups.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_collision_groups(
    handle: RprColliderHandle,
    value: RprInteractionGroups,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_collision_groups(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Set the collider application-owned 128-bit user value.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_user_data(
    handle: RprColliderHandle,
    value: RprUserData,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_user_data(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Return the world-space position of the indexed particle.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_particle_position(
    handle: RprSoftBodyHandle,
    index: usize,
) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_particle_position(
                (element as *const SoftBody).cast(),
                index,
                out,
            ))
        })
    })
}

/// Copy world-space particle positions.
/// @see @ref output_buffers
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_particle_positions(
    handle: RprSoftBodyHandle,
    buffer: *mut RprVector,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_particle_positions(
                (element as *const SoftBody).cast(),
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Return a copy of the soft body material parameters.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_material(handle: RprSoftBodyHandle) -> RprSoftBodyMaterial {
    let world = handle.world;
    ffi_value(|out: *mut RprSoftBodyMaterial| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_read_material(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Set the world-space position of the indexed particle.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_particle_position(
    handle: RprSoftBodyHandle,
    index: usize,
    value: RprVector,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_particle_position(
            (element as *mut SoftBody).cast(),
            index,
            value,
        ))
    })
}

/// Copy material parameters into the soft body.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_material(
    handle: RprSoftBodyHandle,
    data: *const RprSoftBodyMaterial,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_material_data(
            (element as *mut SoftBody).cast(),
            data,
        ))
    })
}

/// Add a world-space force to the indexed particle.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_add_particle_force(
    handle: RprSoftBodyHandle,
    index: usize,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_add_particle_force(
            (element as *mut SoftBody).cast(),
            index,
            value,
            wake_up,
        ))
    })
}

/// A copied state snapshot, with no pointers or ownership obligations.
/// @ingroup rigid_bodies
#[repr(C)]
#[derive(Clone, Copy)]
#[allow(non_snake_case)]
pub struct RprRigidBodyState {
    /// World-space pose.
    pub position: RprPose,
    /// World-space linear velocity.
    pub linvel: RprVector,
    /// World-space angular velocity in radians per second.
    pub angvel: RprAngVector,
    /// Whether the body starts/is asleep.
    pub sleeping: RprBool,
    /// Whether this setting/object is enabled (0 or 1).
    pub enabled: RprBool,
    /// Application data; Rapier does not own pointers encoded in it.
    pub userData: RprUserData,
}
impl From<&RigidBody> for RprRigidBodyState {
    fn from(body: &RigidBody) -> Self {
        Self {
            position: (*body.position()).into(),
            linvel: body.linvel().into(),
            angvel: angular_out(body.angvel()),
            sleeping: body.is_sleeping() as _,
            enabled: body.is_enabled() as _,
            userData: body.user_data.into(),
        }
    }
}

/// Copies states in the same order as handles, without allocating temporary storage.
/// All handles are validated before writing. On INVALID_HANDLE outputs are unchanged.
/// NULL/0 is a size query. BUFFER_TOO_SMALL returns the required count and leaves states untouched.
/// @ingroup rigid_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_rigid_body_read_states(
    world: *const RprWorld,
    handles: *const RprRigidBodyHandle,
    handle_count: usize,
    states: *mut RprRigidBodyState,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            for value in input(handles, handle_count)? {
                value.check_world(world)?;
            }
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_read_states(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handles,
                handle_count,
                states,
                capacity,
                count,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_read_states(
    set: *const RprRigidBodySet,
    handles: *const RprRigidBodyHandle,
    handle_count: usize,
    states: *mut RprRigidBodyState,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let bodies = &get(set)?.0;
        let handles = input(handles, handle_count)?;
        out_ptr(count)?;
        for handle in handles {
            bodies.get(handle.raw()).ok_or_else(missing)?;
        }
        if states.is_null() && capacity == 0 {
            return output(count, handles.len());
        }
        out_ptr(states)?;
        ensure(
            capacity <= isize::MAX as usize / size_of::<RprRigidBodyState>(),
            "buffer is too large",
        )?;
        if capacity < handles.len() {
            output(count, handles.len())?;
            return Err((RPR_BUFFER_TOO_SMALL, "output buffer is too small".into()));
        }
        for (i, handle) in handles.iter().enumerate() {
            states
                .add(i)
                .write(RprRigidBodyState::from(&bodies[handle.raw()]));
        }
        output(count, handles.len())
    })
}

/// Copies joint configuration without returning a borrowed joint pointer.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_desc(handle: RprImpulseJointHandle) -> RprJointDesc {
    let world = handle.world;
    ffi_value(|out: *mut RprJointDesc| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprImpulseJointSet = std::ptr::addr_of!((*raw).0.impulse_joints).cast();

            output(
                out,
                get(set)?
                    .0
                    .get(handle.raw())
                    .ok_or_else(missing)?
                    .data
                    .into(),
            )
        })
    })
}

/// Replaces configuration after validation, resetting cached limit/motor impulses.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_desc(
    handle: RprImpulseJointHandle,
    desc: *const RprJointDesc,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let desc = get(desc)?.raw()?;
        let wake_up = boolean(wake_up)?;
        get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?
            .data = desc;
        Ok(())
    })
}
