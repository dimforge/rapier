use crate::*;
pub(crate) fn axis(value: u32) -> Result<JointAxis> {
    match value {
        0 => Ok(JointAxis::LinX),
        1 => Ok(JointAxis::LinY),
        #[cfg(feature = "dim3")]
        2 => Ok(JointAxis::LinZ),
        #[cfg(feature = "dim3")]
        3 => Ok(JointAxis::AngX),
        #[cfg(feature = "dim3")]
        4 => Ok(JointAxis::AngY),
        #[cfg(feature = "dim3")]
        5 => Ok(JointAxis::AngZ),
        #[cfg(feature = "dim2")]
        2 => Ok(JointAxis::AngX),
        _ => Err(invalid("joint axis out of range")),
    }
}
pub(crate) fn axes(value: u8) -> Result<JointAxesMask> {
    JointAxesMask::from_bits(value).ok_or_else(|| invalid("invalid joint axes"))
}

pub(crate) unsafe fn rpr_generic_joint_set_local_frame1(
    joint: *mut RprGenericJoint,
    value: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        get_mut(joint)?.0.set_local_frame1(value);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_local_frame2(
    joint: *mut RprGenericJoint,
    value: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        get_mut(joint)?.0.set_local_frame2(value);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_local_anchor1(
    joint: *mut RprGenericJoint,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        get_mut(joint)?.0.set_local_anchor1(value);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_local_anchor2(
    joint: *mut RprGenericJoint,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        get_mut(joint)?.0.set_local_anchor2(value);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_contacts_enabled(
    joint: *mut RprGenericJoint,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        get_mut(joint)?.0.set_contacts_enabled(value);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_enabled(
    joint: *mut RprGenericJoint,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        get_mut(joint)?.0.set_enabled(value);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_softness(
    joint: *mut RprGenericJoint,
    value: RprSpringCoefficients,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let _ = get_mut(joint)?.0.set_softness(value);
        Ok(())
    })
}

pub(crate) unsafe fn rpr_generic_joint_set_locked_axes(
    joint: *mut RprGenericJoint,
    value: u8,
) -> RprStatus {
    ffi(|| unsafe {
        let value = axes(value)?;
        get_mut(joint)?.0.locked_axes = value;
        Ok(())
    })
}

pub(crate) unsafe fn rpr_generic_joint_set_limit_axes(
    joint: *mut RprGenericJoint,
    value: u8,
) -> RprStatus {
    ffi(|| unsafe {
        let value = axes(value)?;
        get_mut(joint)?.0.limit_axes = value;
        Ok(())
    })
}

pub(crate) unsafe fn rpr_generic_joint_set_motor_axes(
    joint: *mut RprGenericJoint,
    value: u8,
) -> RprStatus {
    ffi(|| unsafe {
        let value = axes(value)?;
        get_mut(joint)?.0.motor_axes = value;
        Ok(())
    })
}

pub(crate) unsafe fn rpr_generic_joint_set_coupled_axes(
    joint: *mut RprGenericJoint,
    value: u8,
) -> RprStatus {
    ffi(|| unsafe {
        let value = axes(value)?;
        get_mut(joint)?.0.coupled_axes = value;
        Ok(())
    })
}

pub(crate) unsafe fn rpr_generic_joint_set_local_axis1(
    joint: *mut RprGenericJoint,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let v = value.raw()?;
        positive(v.length())?;
        get_mut(joint)?.0.set_local_axis1(v.normalize());
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_local_axis2(
    joint: *mut RprGenericJoint,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let v = value.raw()?;
        positive(v.length())?;
        get_mut(joint)?.0.set_local_axis2(v.normalize());
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_limits(
    joint: *mut RprGenericJoint,
    joint_axis: u32,
    min: RprReal,
    max: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        ensure(
            !min.is_nan() && !max.is_nan() && min <= max,
            "limits must be ordered and not NaN",
        )?;
        let a = axis(joint_axis)?;
        get_mut(joint)?.0.set_limits(a, [min, max]);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_motor(
    joint: *mut RprGenericJoint,
    joint_axis: u32,
    target_position: RprReal,
    target_velocity: RprReal,
    stiffness: RprReal,
    damping: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let a = axis(joint_axis)?;
        finite(target_position)?;
        finite(target_velocity)?;
        nonnegative(stiffness)?;
        nonnegative(damping)?;
        get_mut(joint)?
            .0
            .set_motor(a, target_position, target_velocity, stiffness, damping);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_motor_max_force(
    joint: *mut RprGenericJoint,
    joint_axis: u32,
    max_force: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let a = axis(joint_axis)?;
        nonnegative(max_force)?;
        get_mut(joint)?.0.set_motor_max_force(a, max_force);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_motor_model(
    joint: *mut RprGenericJoint,
    joint_axis: u32,
    model: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let a = axis(joint_axis)?;
        let model = match model {
            0 => MotorModel::AccelerationBased,
            1 => MotorModel::ForceBased,
            _ => return Err(invalid("unknown motor model")),
        };
        get_mut(joint)?.0.set_motor_model(a, model);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_user_data(
    joint: *mut RprGenericJoint,
    value: RprUserData,
) -> RprStatus {
    ffi(|| unsafe {
        get_mut(joint)?.0.user_data = value.raw();
        Ok(())
    })
}

/// Remove an impulse joint. wake_up wakes its connected bodies.
/// @ingroup joints
#[rapier_export]
pub unsafe extern "C" fn rpr_remove_impulse_joint(
    handle: RprImpulseJointHandle,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let set = get_mut(set)?;
        set.0.get(handle.raw()).ok_or_else(missing)?;
        set.0.remove(handle.raw(), wake_up);
        Ok(())
    })
}

/// Copy entity handles.
/// @see @ref output_buffers
/// @ingroup joints
#[rapier_export]
pub unsafe extern "C" fn rpr_impulse_joint_handles(
    world: *const RprWorld,
    buffer: *mut RprImpulseJointHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();

                let set: *const RprImpulseJointSet =
                    std::ptr::addr_of!((*raw).0.impulse_joints).cast();

                let values: Vec<_> = get(set)?.0.iter().map(|(h, _)| h.into()).collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    }
}

/// Remove an articulation joint. wake_up wakes affected bodies.
/// @ingroup joints
#[rapier_export]
pub unsafe extern "C" fn rpr_remove_multibody_joint(
    handle: RprMultibodyJointHandle,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprMultibodyJointSet =
            std::ptr::addr_of_mut!((*raw).0.multibody_joints).cast();

        let wake_up = boolean(wake_up)?;
        let set = get_mut(set)?;
        set.0.get(handle.raw()).ok_or_else(missing)?;
        set.0.remove(handle.raw(), wake_up);
        Ok(())
    })
}

/// Copy entity handles.
/// @see @ref output_buffers
/// @ingroup joints
#[rapier_export]
pub unsafe extern "C" fn rpr_multibody_joint_handles(
    world: *const RprWorld,
    buffer: *mut RprMultibodyJointHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();

                let set: *const RprMultibodyJointSet =
                    std::ptr::addr_of!((*raw).0.multibody_joints).cast();

                let values: Vec<_> = get(set)?.0.iter().map(|(h, _, _, _)| h.into()).collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    }
}

/// Return the two bodies connected by an impulse joint.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_bodies(handle: RprImpulseJointHandle) -> RprJointBodies {
    let world = handle.world;
    ffi_world_value(world, |result: *mut RprJointBodies| {
        let body1 = unsafe { std::ptr::addr_of_mut!((*result).body1) };
        let body2 = unsafe { std::ptr::addr_of_mut!((*result).body2) };

        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprImpulseJointSet = std::ptr::addr_of!((*raw).0.impulse_joints).cast();

            out_ptr(body1)?;
            out_ptr(body2)?;
            let j = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            output(body1, j.body1().into())?;
            output(body2, j.body2().into())
        })
    })
}

/// Damped least-squares inverse-kinematics parameters.
/// @ingroup math
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprInverseKinematicsOptions {
    /// Nonnegative motor damping.
    pub damping: RprReal,
    /// Maximum inverse-kinematics iterations.
    pub max_iters: usize,
    /// Controlled-axis bitmask; translations precede rotations.
    pub constrained_axes: u8,
    /// Linear convergence tolerance.
    pub epsilon_linear: RprReal,
    /// Angular convergence tolerance in radians.
    pub epsilon_angular: RprReal,
}
/// Return native default inverse kinematics options. This POD value owns no resources.
/// @ingroup joints
#[rapier_export]
pub extern "C" fn rpr_default_inverse_kinematics_options() -> RprInverseKinematicsOptions {
    let options = InverseKinematicsOption::default();
    RprInverseKinematicsOptions {
        damping: options.damping,
        max_iters: options.max_iters,
        constrained_axes: options.constrained_axes.bits(),
        epsilon_linear: options.epsilon_linear,
        epsilon_angular: options.epsilon_angular,
    }
}

/// Return the articulation degrees of freedom associated with the joint.
/// @ingroup joints
#[rapier_export(multibody_joint)]
pub unsafe extern "C" fn rpr_multibody_joint_ndofs(handle: RprMultibodyJointHandle) -> usize {
    let world = handle.world;
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprMultibodyJointSet =
                std::ptr::addr_of!((*raw).0.multibody_joints).cast();

            output(
                out,
                get(set)?.0.get(handle.raw()).ok_or_else(missing)?.0.ndofs(),
            )
        })
    })
}

/// Optional per-link filter, called synchronously. Must not reenter or retain physics objects.
/// @ingroup joints
pub type RprIkJointCanMove =
    Option<unsafe extern "C" fn(*mut std::ffi::c_void, RprRigidBodyHandle) -> RprBool>;
/// Read/write displacement buffer must contain exactly ndofs entries; zero it for a fresh solve.
/// @ingroup joints
#[rapier_export(multibody_joint)]
pub unsafe extern "C" fn rpr_multibody_joint_inverse_kinematics(
    handle: RprMultibodyJointHandle,
    options: *const RprInverseKinematicsOptions,
    target: RprPose,
    can_move: RprIkJointCanMove,
    user_data: *mut std::ffi::c_void,
    displacements: *mut RprReal,
    count: usize,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.read()?;
        let raw = access.raw();

        let set: *const RprMultibodyJointSet = std::ptr::addr_of!((*raw).0.multibody_joints).cast();
        let bodies: *const RprRigidBodySet = std::ptr::addr_of!((*raw).0.bodies).cast();

        let (multibody, link_id) = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        ensure(multibody.link(link_id).is_some(), "invalid multibody link")?;
        ensure(
            count == multibody.ndofs(),
            "displacement count must equal articulation dofs",
        )?;
        let values = input(displacements, count)?;
        for &value in values {
            finite(value)?;
        }
        let mut values = rapier::math::DVector::from_column_slice(values);
        let options = get(options)?;
        let options = InverseKinematicsOption {
            damping: nonnegative(options.damping)?,
            max_iters: options.max_iters,
            constrained_axes: axes(options.constrained_axes)?,
            epsilon_linear: nonnegative(options.epsilon_linear)?,
            epsilon_angular: nonnegative(options.epsilon_angular)?,
        };
        let target = target.raw()?;
        multibody.inverse_kinematics(
            &get(bodies)?.0,
            link_id,
            &options,
            &target,
            |link| {
                can_move.is_none_or(|callback| {
                    callback(
                        user_data,
                        RprRigidBodyHandle::from(link.rigid_body_handle()).with_world(world),
                    ) != 0
                })
            },
            &mut values,
        );
        if count != 0 {
            std::ptr::copy_nonoverlapping(values.as_ptr(), displacements, count);
        }
        Ok(())
    })
}

/// Apply generalized articulation displacements in native degree-of-freedom order.
/// @ingroup joints
#[rapier_export(multibody_joint)]
pub unsafe extern "C" fn rpr_multibody_joint_apply_displacements(
    handle: RprMultibodyJointHandle,
    displacements: *const RprReal,
    count: usize,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprMultibodyJointSet =
            std::ptr::addr_of_mut!((*raw).0.multibody_joints).cast();

        let values = input(displacements, count)?;
        for &value in values {
            finite(value)?;
        }
        let (multibody, _) = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            count == multibody.ndofs(),
            "displacement count must equal articulation dofs",
        )?;
        multibody.apply_displacements(values);
        Ok(())
    })
}

pub(crate) unsafe fn rpr_generic_joint_set_motor_position(
    joint: *mut RprGenericJoint,
    joint_axis: u32,
    target_position: RprReal,
    stiffness: RprReal,
    damping: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let a = axis(joint_axis)?;
        finite(target_position)?;
        nonnegative(stiffness)?;
        nonnegative(damping)?;
        get_mut(joint)?
            .0
            .set_motor_position(a, target_position, stiffness, damping);
        Ok(())
    })
}
pub(crate) unsafe fn rpr_generic_joint_set_motor_velocity(
    joint: *mut RprGenericJoint,
    joint_axis: u32,
    target_velocity: RprReal,
    factor: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let a = axis(joint_axis)?;
        finite(target_velocity)?;
        nonnegative(factor)?;
        get_mut(joint)?
            .0
            .set_motor_velocity(a, target_velocity, factor);
        Ok(())
    })
}
