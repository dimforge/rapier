use crate::*;

pub(crate) unsafe fn native_rigid_body_position(
    object: *const RprRigidBody,
    out: *mut RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, (*object.0.position()).into())
    })
}

pub(crate) unsafe fn native_rigid_body_next_position(
    object: *const RprRigidBody,
    out: *mut RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, (*object.0.next_position()).into())
    })
}

pub(crate) unsafe fn native_rigid_body_translation(
    object: *const RprRigidBody,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.translation().into())
    })
}

pub(crate) unsafe fn native_rigid_body_rotation(
    object: *const RprRigidBody,
    out: *mut RprRotation,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, (*object.0.rotation()).into())
    })
}

pub(crate) unsafe fn native_rigid_body_linvel(
    object: *const RprRigidBody,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.linvel().into())
    })
}

pub(crate) unsafe fn native_rigid_body_angvel(
    object: *const RprRigidBody,
    out: *mut RprAngVector,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, angular_out(object.0.angvel()))
    })
}

pub(crate) unsafe fn native_rigid_body_center_of_mass(
    object: *const RprRigidBody,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.center_of_mass().into())
    })
}

pub(crate) unsafe fn native_rigid_body_local_center_of_mass(
    object: *const RprRigidBody,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.local_center_of_mass().into())
    })
}

pub(crate) unsafe fn native_rigid_body_user_force(
    object: *const RprRigidBody,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.user_force().into())
    })
}

pub(crate) unsafe fn native_rigid_body_user_torque(
    object: *const RprRigidBody,
    out: *mut RprAngVector,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, angular_out(object.0.user_torque()))
    })
}

pub(crate) unsafe fn native_rigid_body_body_type(
    object: *const RprRigidBody,
    out: *mut u32,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.body_type() as u32)
    })
}

pub(crate) unsafe fn native_rigid_body_user_data(
    object: *const RprRigidBody,
    out: *mut RprUserData,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.user_data.into())
    })
}

pub(crate) unsafe fn native_rigid_body_mass(
    object: *const RprRigidBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.mass())
    })
}

pub(crate) unsafe fn native_rigid_body_gravity_scale(
    object: *const RprRigidBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.gravity_scale())
    })
}

pub(crate) unsafe fn native_rigid_body_linear_damping(
    object: *const RprRigidBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.linear_damping())
    })
}

pub(crate) unsafe fn native_rigid_body_angular_damping(
    object: *const RprRigidBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.angular_damping())
    })
}

pub(crate) unsafe fn native_rigid_body_kinetic_energy(
    object: *const RprRigidBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.kinetic_energy())
    })
}

pub(crate) unsafe fn native_rigid_body_soft_ccd_prediction(
    object: *const RprRigidBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.soft_ccd_prediction())
    })
}

pub(crate) unsafe fn native_rigid_body_is_sleeping(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_sleeping() as u32)
    })
}

pub(crate) unsafe fn native_rigid_body_is_enabled(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_enabled() as u32)
    })
}

pub(crate) unsafe fn native_rigid_body_is_ccd_enabled(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_ccd_enabled() as u32)
    })
}

pub(crate) unsafe fn native_rigid_body_is_dynamic(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_dynamic() as u32)
    })
}

/// The soft body owning this proxy, or an invalid handle for an ordinary rigid body.
pub(crate) unsafe fn native_rigid_body_soft_body(
    object: *const RprRigidBody,
    out: *mut RprSoftBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        output(
            out,
            get(object)?
                .0
                .soft_body()
                .map(Into::into)
                .unwrap_or_default(),
        )
    })
}

pub(crate) unsafe fn native_rigid_body_is_soft_frame(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe { output(out, get(object)?.0.is_soft_frame() as RprBool) })
}

pub(crate) unsafe fn native_rigid_body_is_fixed(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_fixed() as u32)
    })
}

pub(crate) unsafe fn native_rigid_body_is_kinematic(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_kinematic() as u32)
    })
}

pub(crate) unsafe fn native_rigid_body_is_moving(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_moving() as u32)
    })
}

pub(crate) unsafe fn native_rigid_body_is_ccd_active(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_ccd_active() as u32)
    })
}

pub(crate) unsafe fn native_rigid_body_set_position(
    object: *mut RprRigidBody,
    value: RprPose,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.set_position(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_translation(
    object: *mut RprRigidBody,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.set_translation(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_rotation(
    object: *mut RprRigidBody,
    value: RprRotation,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.set_rotation(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_linvel(
    object: *mut RprRigidBody,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.set_linvel(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_angvel(
    object: *mut RprRigidBody,
    value: RprAngVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = angular(value)?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.set_angvel(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_body_type(
    object: *mut RprRigidBody,
    value: u32,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = body_type(value)?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.set_body_type(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_next_kinematic_position(
    object: *mut RprRigidBody,
    value: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let object = get_mut(object)?;
        object.0.set_next_kinematic_position(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_next_kinematic_translation(
    object: *mut RprRigidBody,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let object = get_mut(object)?;
        object.0.set_next_kinematic_translation(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_next_kinematic_rotation(
    object: *mut RprRigidBody,
    value: RprRotation,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let object = get_mut(object)?;
        object.0.set_next_kinematic_rotation(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_gravity_scale(
    object: *mut RprRigidBody,
    value: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = finite(value)?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.set_gravity_scale(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_additional_mass(
    object: *mut RprRigidBody,
    value: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.set_additional_mass(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_linear_damping(
    object: *mut RprRigidBody,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let object = get_mut(object)?;
        object.0.set_linear_damping(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_angular_damping(
    object: *mut RprRigidBody,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let object = get_mut(object)?;
        object.0.set_angular_damping(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_soft_ccd_prediction(
    object: *mut RprRigidBody,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let object = get_mut(object)?;
        object.0.set_soft_ccd_prediction(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_enabled(
    object: *mut RprRigidBody,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let object = get_mut(object)?;
        object.0.set_enabled(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_ccd_enabled(
    object: *mut RprRigidBody,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let object = get_mut(object)?;
        object.0.enable_ccd(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_translations_locked(
    object: *mut RprRigidBody,
    value: RprBool,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.lock_translations(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_rotations_locked(
    object: *mut RprRigidBody,
    value: RprBool,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.lock_rotations(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_dominance_group(
    object: *mut RprRigidBody,
    value: i8,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get_mut(object)?;
        object.0.set_dominance_group(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_additional_solver_iterations(
    object: *mut RprRigidBody,
    value: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get_mut(object)?;
        object.0.set_additional_solver_iterations(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_additional_pgs_iterations(
    object: *mut RprRigidBody,
    value: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get_mut(object)?;
        object.0.set_additional_pgs_iterations(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_add_force(
    object: *mut RprRigidBody,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.add_force(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_apply_impulse(
    object: *mut RprRigidBody,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.apply_impulse(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_add_torque(
    object: *mut RprRigidBody,
    value: RprAngVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = angular(value)?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.add_torque(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_apply_torque_impulse(
    object: *mut RprRigidBody,
    value: RprAngVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = angular(value)?;
        let wake_up = boolean(wake_up)?;
        let object = get_mut(object)?;
        object.0.apply_torque_impulse(value, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_set_user_data(
    object: *mut RprRigidBody,
    value: RprUserData,
) -> RprStatus {
    ffi(|| unsafe {
        get_mut(object)?.0.user_data = value.raw();
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_add_force_at_point(
    object: *mut RprRigidBody,
    value: RprVector,
    point: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let point = point.raw()?;
        let wake_up = boolean(wake_up)?;
        get_mut(object)?.0.add_force_at_point(value, point, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_apply_impulse_at_point(
    object: *mut RprRigidBody,
    value: RprVector,
    point: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let point = point.raw()?;
        let wake_up = boolean(wake_up)?;
        get_mut(object)?
            .0
            .apply_impulse_at_point(value, point, wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_reset_forces(
    object: *mut RprRigidBody,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let wake_up = boolean(wake_up)?;
        get_mut(object)?.0.reset_forces(wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_reset_torques(
    object: *mut RprRigidBody,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let wake_up = boolean(wake_up)?;
        get_mut(object)?.0.reset_torques(wake_up);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_sleep(object: *mut RprRigidBody) -> RprStatus {
    ffi(|| unsafe {
        get_mut(object)?.0.sleep();
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_velocity_at_point(
    object: *const RprRigidBody,
    point: RprVector,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe { output(out, get(object)?.0.velocity_at_point(point.raw()?).into()) })
}

pub(crate) unsafe fn native_rigid_body_colliders(
    object: *const RprRigidBody,
    buffer: *mut RprColliderHandle,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let values: Vec<_> = get(object)?
            .0
            .colliders()
            .iter()
            .copied()
            .map(Into::into)
            .collect();
        copy_out(&values, buffer, capacity, count)
    })
}

/// Propagate all modified body poses to attached colliders. Run collision detection or step before
/// querying the broad phase.
/// @ingroup rigid_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_rigid_body_propagate_modified_body_positions_to_colliders(
    world: *mut RprWorld,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *const RprRigidBodySet = std::ptr::addr_of!((*raw).0.bodies).cast();
        let colliders: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        get(set)?
            .0
            .propagate_modified_body_positions_to_colliders(&mut get_mut(colliders)?.0);
        Ok(())
    })
}

#[cfg(feature = "dim3")]
pub(crate) unsafe fn native_rigid_body_gyroscopic_forces_enabled(
    body: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe { output(out, get(body)?.0.gyroscopic_forces_enabled() as RprBool) })
}

#[cfg(feature = "dim3")]
pub(crate) unsafe fn native_rigid_body_set_gyroscopic_forces_enabled(
    body: *mut RprRigidBody,
    enabled: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let enabled = boolean(enabled)?;
        get_mut(body)?.0.enable_gyroscopic_forces(enabled);
        Ok(())
    })
}

/// Copies the island manager's active body handles.
/// @see @ref output_buffers
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_active_rigid_bodies(
    world: *const RprWorld,
    buffer: *mut RprRigidBodyHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();

                let islands: *const RprIslandManager = std::ptr::addr_of!((*raw).0.islands).cast();

                let handles: Vec<RprRigidBodyHandle> =
                    get(islands)?.0.active_bodies().map(Into::into).collect();
                copy_out(&handles, buffer, capacity, count)
            })
        })
    }
}

/// Wake a body by handle, including a soft-body cluster proxy.
/// @ingroup rigid_bodies
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_wake_up(
    handle: RprRigidBodyHandle,
    strong: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let strong = boolean(strong)?;
        get_mut(set)?
            .0
            .get_mut(handle.raw())
            .ok_or_else(missing)?
            .wake_up(strong);
        Ok(())
    })
}

pub(crate) unsafe fn native_rigid_body_dominance_group(
    object: *const RprRigidBody,
    out: *mut i8,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.dominance_group())
    })
}

pub(crate) unsafe fn native_rigid_body_additional_solver_iterations(
    object: *const RprRigidBody,
    out: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.additional_solver_iterations())
    })
}

pub(crate) unsafe fn native_rigid_body_additional_pgs_iterations(
    object: *const RprRigidBody,
    out: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.additional_pgs_iterations())
    })
}

pub(crate) unsafe fn native_rigid_body_is_fast_rotation_allowed(
    object: *const RprRigidBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_fast_rotation_allowed() as RprBool)
    })
}

pub(crate) unsafe fn native_rigid_body_set_allow_fast_rotation(
    object: *mut RprRigidBody,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let object = get_mut(object)?;
        object.0.set_allow_fast_rotation(value);
        Ok(())
    })
}
