//! Joint configuration setters and handle-scoped live-joint edits.
use crate::handle_access::forward;
use crate::*;
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
