//! Scoped read access derived from the borrows Rapier supplies to callbacks.
use crate::*;
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_pid_controller)]
pub unsafe extern "C" fn rpr_read_pid_controller_rigid_body_correction(
    context: *const RprReadContext,
    controller: *mut RprPidController,
    dt: RprReal,
    body: RprRigidBodyHandle,
    target_pose: RprPose,
    target_linvel: RprVector,
    target_angvel: RprAngVector,
) -> RprVelocityCorrection {
    ffi_value(|result: *mut RprVelocityCorrection| {
        let linear = unsafe { std::ptr::addr_of_mut!((*result).linear) };
        let angular_velocity = unsafe { std::ptr::addr_of_mut!((*result).angularVelocity) };

        ffi(|| unsafe {
            body.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_pid_controller_rigid_body_correction(
                controller,
                dt,
                get(context)?.bodies,
                body,
                target_pose,
                target_linvel,
                target_angvel,
                linear,
                angular_velocity,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export]
pub unsafe extern "C" fn rpr_read_rigid_body_count(context: *const RprReadContext) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            crate::handle_access::forward(native_rigid_body_set_len(get(context)?.bodies, out))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export]
pub unsafe extern "C" fn rpr_read_rigid_body_handles(
    context: *const RprReadContext,
    buffer: *mut RprRigidBodyHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(
            read_context_world(context),
            buffer,
            capacity,
            |count: *mut usize| {
                ffi(|| {
                    crate::handle_access::forward(native_rigid_body_set_handles(
                        get(context)?.bodies,
                        buffer,
                        capacity,
                        count,
                    ))
                })
            },
        )
    }
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_contains(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_contains(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export]
pub unsafe extern "C" fn rpr_read_collider_count(context: *const RprReadContext) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            crate::handle_access::forward(native_collider_set_len(get(context)?.colliders, out))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export]
pub unsafe extern "C" fn rpr_read_collider_handles(
    context: *const RprReadContext,
    buffer: *mut RprColliderHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(
            read_context_world(context),
            buffer,
            capacity,
            |count: *mut usize| {
                ffi(|| {
                    crate::handle_access::forward(native_collider_set_handles(
                        get(context)?.colliders,
                        buffer,
                        capacity,
                        count,
                    ))
                })
            },
        )
    }
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_contains(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_contains(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_shape_identity(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_shape_identity(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_mass_properties(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprMassProperties {
    ffi_value(|out: *mut RprMassProperties| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_mass_properties(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_locked_axes(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> u8 {
    ffi_value(|out: *mut u8| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_locked_axes(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_is_voxels(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_is_voxels(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_voxel_at_flat_id(
    context: *const RprReadContext,
    handle: RprColliderHandle,
    id: u32,
) -> RprVoxelQuery {
    ffi_value(|result: *mut RprVoxelQuery| {
        let key = unsafe { std::ptr::addr_of_mut!((*result).key) };
        let center = unsafe { std::ptr::addr_of_mut!((*result).center) };
        let size = unsafe { std::ptr::addr_of_mut!((*result).size) };
        let found = unsafe { std::ptr::addr_of_mut!((*result).found) };

        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_voxel_at_flat_id(
                get(context)?.colliders,
                handle,
                id,
                key,
                center,
                size,
                found,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_next_position(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprPose {
    ffi_value(|out: *mut RprPose| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_next_position(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_rotation(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprRotation {
    ffi_value(|out: *mut RprRotation| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_rotation(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_center_of_mass(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprVector {
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_center_of_mass(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_local_center_of_mass(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprVector {
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_local_center_of_mass(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_user_force(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprVector {
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_user_force(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_user_torque(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprAngVector {
    ffi_value(|out: *mut RprAngVector| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_user_torque(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_body_type(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> u32 {
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_body_type(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_mass(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_mass(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_gravity_scale(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_gravity_scale(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_linear_damping(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_linear_damping(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_angular_damping(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_angular_damping(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_kinetic_energy(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_kinetic_energy(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_soft_ccd_prediction(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_soft_ccd_prediction(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_is_ccd_enabled(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_is_ccd_enabled(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_is_dynamic(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_is_dynamic(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_soft_body(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprSoftBodyHandle {
    ffi_world_value(
        unsafe { read_context_world(context) },
        |out: *mut RprSoftBodyHandle| {
            ffi(|| unsafe {
                handle.check_world(read_context_world(context))?;
                crate::handle_access::forward(native_rigid_body_set_get_soft_body(
                    get(context)?.bodies,
                    handle,
                    out,
                ))
            })
        },
    )
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_is_soft_frame(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_is_soft_frame(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_is_fixed(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_is_fixed(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_is_kinematic(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_is_kinematic(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_is_moving(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_is_moving(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_is_ccd_active(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_is_ccd_active(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_velocity_at_point(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
    point: RprVector,
) -> RprVector {
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_velocity_at_point(
                get(context)?.bodies,
                handle,
                point,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_colliders(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
    buffer: *mut RprColliderHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(
            read_context_world(context),
            buffer,
            capacity,
            |count: *mut usize| {
                ffi(|| {
                    handle.check_world(read_context_world(context))?;
                    crate::handle_access::forward(native_rigid_body_set_get_colliders(
                        get(context)?.bodies,
                        handle,
                        buffer,
                        capacity,
                        count,
                    ))
                })
            },
        )
    }
}
#[cfg(feature = "dim3")]
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_gyroscopic_forces_enabled(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_gyroscopic_forces_enabled(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_rotation(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprRotation {
    ffi_value(|out: *mut RprRotation| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_rotation(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_collision_groups(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprInteractionGroups {
    ffi_value(|out: *mut RprInteractionGroups| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_collision_groups(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_solver_groups(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprInteractionGroups {
    ffi_value(|out: *mut RprInteractionGroups| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_solver_groups(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_user_data(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprUserData {
    ffi_value(|out: *mut RprUserData| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_user_data(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_active_events(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> u32 {
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_active_events(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_mass(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_mass(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_density(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_density(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_volume(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_volume(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_contact_skin(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_contact_skin(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_contact_force_event_threshold(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_contact_force_event_threshold(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_is_enabled(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_is_enabled(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_compute_aabb(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprAabb {
    ffi_value(|out: *mut RprAabb| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_compute_aabb(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
/// Returns an owned shape wrapper sharing the geometry. Release it with FreeSharedShape.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_clone_shape(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_shared_shape(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_validate_handle(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        handle.check_world(read_context_world(context))?;
        crate::handle_access::forward(native_rigid_body_set_validate_handle(
            get(context)?.bodies,
            handle,
        ))
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_validate_handle(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprStatus {
    ffi(|| unsafe {
        handle.check_world(read_context_world(context))?;
        crate::handle_access::forward(native_collider_set_validate_handle(
            get(context)?.colliders,
            handle,
        ))
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_position(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprPose {
    ffi_value(|out: *mut RprPose| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_position(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_translation(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprVector {
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_translation(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_linvel(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprVector {
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_linvel(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_angvel(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprAngVector {
    ffi_value(|out: *mut RprAngVector| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_angvel(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_is_sleeping(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_is_sleeping(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_is_enabled(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_is_enabled(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_rigid_body)]
pub unsafe extern "C" fn rpr_read_rigid_body_user_data(
    context: *const RprReadContext,
    handle: RprRigidBodyHandle,
) -> RprUserData {
    ffi_value(|out: *mut RprUserData| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_rigid_body_set_get_user_data(
                get(context)?.bodies,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_position(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprPose {
    ffi_value(|out: *mut RprPose| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_position(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_translation(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprVector {
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_translation(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_friction(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_friction(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_restitution(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_restitution(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_is_sensor(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(read_context_world(context))?;
            crate::handle_access::forward(native_collider_set_get_is_sensor(
                get(context)?.colliders,
                handle,
                out,
            ))
        })
    })
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_parent(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprRigidBodyHandle {
    ffi_world_value(
        unsafe { read_context_world(context) },
        |out: *mut RprRigidBodyHandle| {
            ffi(|| unsafe {
                handle.check_world(read_context_world(context))?;
                crate::handle_access::forward(native_collider_set_get_parent(
                    get(context)?.colliders,
                    handle,
                    out,
                ))
            })
        },
    )
}
/// Read callback-visible state. The context is valid only until its callback returns.
#[rapier_export]
pub unsafe extern "C" fn rpr_read_rigid_body_read_states(
    context: *const RprReadContext,
    handles: *const RprRigidBodyHandle,
    handle_count: usize,
    states: *mut RprRigidBodyState,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            for value in input(handles, handle_count)? {
                value.check_world(read_context_world(context))?;
            }
            crate::handle_access::forward(native_rigid_body_set_read_states(
                get(context)?.bodies,
                handles,
                handle_count,
                states,
                capacity,
                count,
            ))
        })
    })
}
