//! Joint configuration setters and handle-scoped live-joint edits.
use crate::handle_access::forward;
use crate::*;
/// Set the joint desc joint frame relative to body 1.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_local_frame1(
    desc: *mut RprJointDesc,
    value: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_local_frame1(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint joint frame relative to body 1.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_local_frame1(
    handle: RprImpulseJointHandle,
    value: RprPose,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_local_frame1(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc joint frame relative to body 2.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_local_frame2(
    desc: *mut RprJointDesc,
    value: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_local_frame2(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint joint frame relative to body 2.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_local_frame2(
    handle: RprImpulseJointHandle,
    value: RprPose,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_local_frame2(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc joint anchor relative to body 1.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_local_anchor1(
    desc: *mut RprJointDesc,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_local_anchor1(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint joint anchor relative to body 1.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_local_anchor1(
    handle: RprImpulseJointHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_local_anchor1(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc joint anchor relative to body 2.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_local_anchor2(
    desc: *mut RprJointDesc,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_local_anchor2(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint joint anchor relative to body 2.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_local_anchor2(
    handle: RprImpulseJointHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_local_anchor2(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Enable or disable allowing contacts between connected bodies for the joint desc.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_contacts_enabled(
    desc: *mut RprJointDesc,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_contacts_enabled(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Enable or disable allowing contacts between connected bodies for the impulse joint.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_contacts_enabled(
    handle: RprImpulseJointHandle,
    value: RprBool,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_contacts_enabled(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Enable or disable the joint desc.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_enabled(
    desc: *mut RprJointDesc,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_enabled(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Enable or disable the impulse joint.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_enabled(
    handle: RprImpulseJointHandle,
    value: RprBool,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_enabled(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc joint spring coefficients.
/// @ingroup soft_bodies
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_softness(
    desc: *mut RprJointDesc,
    value: RprSpringCoefficients,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_softness(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint joint spring coefficients.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup soft_bodies
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_softness(
    handle: RprImpulseJointHandle,
    value: RprSpringCoefficients,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_softness(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc translation/rotation lock bitmask.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_locked_axes(
    desc: *mut RprJointDesc,
    value: u8,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_locked_axes(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint translation/rotation lock bitmask.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_locked_axes(
    handle: RprImpulseJointHandle,
    value: u8,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_locked_axes(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc joint axis mask with limits enabled.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_limit_axes(
    desc: *mut RprJointDesc,
    value: u8,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_limit_axes(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint joint axis mask with limits enabled.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_limit_axes(
    handle: RprImpulseJointHandle,
    value: u8,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_limit_axes(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc joint axis mask with motors enabled.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_motor_axes(
    desc: *mut RprJointDesc,
    value: u8,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_motor_axes(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint joint axis mask with motors enabled.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_motor_axes(
    handle: RprImpulseJointHandle,
    value: u8,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_motor_axes(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc coupled joint axis mask.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_coupled_axes(
    desc: *mut RprJointDesc,
    value: u8,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_coupled_axes(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint coupled joint axis mask.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_coupled_axes(
    handle: RprImpulseJointHandle,
    value: u8,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_coupled_axes(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc joint principal axis in body 1 local coordinates.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_local_axis1(
    desc: *mut RprJointDesc,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_local_axis1(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint joint principal axis in body 1 local coordinates.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_local_axis1(
    handle: RprImpulseJointHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_local_axis1(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc joint principal axis in body 2 local coordinates.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_local_axis2(
    desc: *mut RprJointDesc,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_local_axis2(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint joint principal axis in body 2 local coordinates.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_local_axis2(
    handle: RprImpulseJointHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_local_axis2(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc minimum and maximum limits on an axis (linear distance or angular radians).
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_limits(
    desc: *mut RprJointDesc,
    joint_axis: u32,
    min: RprReal,
    max: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_limits(
            &mut joint, joint_axis, min, max,
        ))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint minimum and maximum limits on an axis (linear distance or angular
/// radians).
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_limits(
    handle: RprImpulseJointHandle,
    joint_axis: u32,
    min: RprReal,
    max: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_limits(
            (&mut joint.data as *mut GenericJoint).cast(),
            joint_axis,
            min,
            max,
        ))
    })
}

/// Set the joint desc motor position/velocity targets and spring coefficients on an axis.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_motor(
    desc: *mut RprJointDesc,
    joint_axis: u32,
    target_position: RprReal,
    target_velocity: RprReal,
    stiffness: RprReal,
    damping: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_motor(
            &mut joint,
            joint_axis,
            target_position,
            target_velocity,
            stiffness,
            damping,
        ))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint motor position/velocity targets and spring coefficients on an axis.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_motor(
    handle: RprImpulseJointHandle,
    joint_axis: u32,
    target_position: RprReal,
    target_velocity: RprReal,
    stiffness: RprReal,
    damping: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_motor(
            (&mut joint.data as *mut GenericJoint).cast(),
            joint_axis,
            target_position,
            target_velocity,
            stiffness,
            damping,
        ))
    })
}

/// Set the joint desc maximum motor force or torque on an axis.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_motor_max_force(
    desc: *mut RprJointDesc,
    joint_axis: u32,
    max_force: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_motor_max_force(
            &mut joint, joint_axis, max_force,
        ))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint maximum motor force or torque on an axis.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_motor_max_force(
    handle: RprImpulseJointHandle,
    joint_axis: u32,
    max_force: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_motor_max_force(
            (&mut joint.data as *mut GenericJoint).cast(),
            joint_axis,
            max_force,
        ))
    })
}

/// Set the joint desc motor model on an axis (0 = acceleration-based, 1 = force-based).
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_motor_model(
    desc: *mut RprJointDesc,
    joint_axis: u32,
    model: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_motor_model(
            &mut joint, joint_axis, model,
        ))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint motor model on an axis (0 = acceleration-based, 1 = force-based).
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_motor_model(
    handle: RprImpulseJointHandle,
    joint_axis: u32,
    model: u32,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_motor_model(
            (&mut joint.data as *mut GenericJoint).cast(),
            joint_axis,
            model,
        ))
    })
}

/// Set the joint desc application-owned 128-bit user value.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_user_data(
    desc: *mut RprJointDesc,
    value: RprUserData,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_user_data(&mut joint, value))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint application-owned 128-bit user value.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_user_data(
    handle: RprImpulseJointHandle,
    value: RprUserData,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_user_data(
            (&mut joint.data as *mut GenericJoint).cast(),
            value,
        ))
    })
}

/// Set the joint desc motor position target and spring coefficients on an axis.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_motor_position(
    desc: *mut RprJointDesc,
    joint_axis: u32,
    target_position: RprReal,
    stiffness: RprReal,
    damping: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_motor_position(
            &mut joint,
            joint_axis,
            target_position,
            stiffness,
            damping,
        ))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint motor position target and spring coefficients on an axis.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_motor_position(
    handle: RprImpulseJointHandle,
    joint_axis: u32,
    target_position: RprReal,
    stiffness: RprReal,
    damping: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_motor_position(
            (&mut joint.data as *mut GenericJoint).cast(),
            joint_axis,
            target_position,
            stiffness,
            damping,
        ))
    })
}

/// Set the joint desc motor velocity target and damping factor on an axis.
/// @ingroup joints
#[rapier_export(joint_desc)]
pub unsafe extern "C" fn rpr_joint_desc_set_motor_velocity(
    desc: *mut RprJointDesc,
    joint_axis: u32,
    target_velocity: RprReal,
    factor: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let mut joint = RprGenericJoint(get(desc)?.raw()?);
        forward(rpr_generic_joint_set_motor_velocity(
            &mut joint,
            joint_axis,
            target_velocity,
            factor,
        ))?;
        output(desc, joint.0.into())
    })
}
/// Set the impulse joint motor velocity target and damping factor on an axis.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_set_motor_velocity(
    handle: RprImpulseJointHandle,
    joint_axis: u32,
    target_velocity: RprReal,
    factor: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprImpulseJointSet = std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();

        let wake_up = boolean(wake_up)?;
        let joint = get_mut(set)?
            .0
            .get_mut(handle.raw(), wake_up)
            .ok_or_else(missing)?;
        forward(rpr_generic_joint_set_motor_velocity(
            (&mut joint.data as *mut GenericJoint).cast(),
            joint_axis,
            target_velocity,
            factor,
        ))
    })
}
