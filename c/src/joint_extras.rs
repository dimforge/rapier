//! Joint impulses, counts, user data and live multibody-joint descriptions.
use crate::*;

/// Impulses applied by an impulse joint during the last step, along the axes of its joint frame.
/// @ingroup joints
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprJointImpulses {
    /// Impulse applied along the locked translational axes.
    pub linear: RprVector,
    /// Angular impulse applied along the locked rotational axes (a scalar in 2D).
    pub angular: RprAngVector,
    /// Impulse applied by the limit of each axis, in translation-then-rotation order.
    pub limits: [RprReal; RPR_JOINT_DOF_COUNT],
    /// Impulse applied by the motor of each axis, in translation-then-rotation order.
    pub motors: [RprReal; RPR_JOINT_DOF_COUNT],
}

impl From<&ImpulseJoint> for RprJointImpulses {
    fn from(joint: &ImpulseJoint) -> Self {
        let i = &joint.impulses;
        #[cfg(feature = "dim2")]
        let (linear, angular) = (Vector::new(i[0], i[1]), i[2]);
        #[cfg(feature = "dim3")]
        let (linear, angular) = (
            Vector::new(i[0], i[1], i[2]),
            AngVector::new(i[3], i[4], i[5]),
        );
        Self {
            linear: linear.into(),
            angular: angular_out(angular),
            limits: joint.data.limits.map(|l| l.impulse),
            motors: joint.data.motors.map(|m| m.impulse),
        }
    }
}

/// Return the impulses applied by the impulse joint during the last step. They are zero before
/// its first step, and their components are expressed along the axes of the joint frame.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_impulses(
    handle: RprImpulseJointHandle,
) -> RprJointImpulses {
    let world = handle.world;
    ffi_value(|out: *mut RprJointImpulses| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprImpulseJointSet = std::ptr::addr_of!((*raw).0.impulse_joints).cast();

            let joint = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            output(out, joint.into())
        })
    })
}

/// Return the impulse joint application-owned 128-bit user value.
/// @ingroup joints
#[rapier_export(impulse_joint)]
pub unsafe extern "C" fn rpr_impulse_joint_user_data(handle: RprImpulseJointHandle) -> RprUserData {
    let world = handle.world;
    ffi_value(|out: *mut RprUserData| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprImpulseJointSet = std::ptr::addr_of!((*raw).0.impulse_joints).cast();

            let joint = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            output(out, joint.data.user_data.into())
        })
    })
}

/// Return the number of impulse joints in the world.
/// @ingroup joints
#[rapier_export]
pub unsafe extern "C" fn rpr_impulse_joint_count(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprImpulseJointSet = std::ptr::addr_of!((*raw).0.impulse_joints).cast();
            output(out, get(set)?.0.len())
        })
    })
}

/// Return the number of multibody joints in the world, which is the number of handles copied by
/// rpr_multibody_joint_handles.
/// @ingroup joints
#[rapier_export]
pub unsafe extern "C" fn rpr_multibody_joint_count(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprMultibodyJointSet =
                std::ptr::addr_of!((*raw).0.multibody_joints).cast();
            output(out, get(set)?.0.iter().count())
        })
    })
}

/// The link attached to its parent by a multibody joint. A handle resolving to a root link is
/// stale: it belonged to a removed joint whose child became the root of a new multibody.
pub(crate) fn multibody_joint_link(
    set: &MultibodyJointSet,
    handle: RprMultibodyJointHandle,
) -> Result<(&Multibody, &MultibodyLink)> {
    let (multibody, id) = set.get(handle.raw()).ok_or_else(missing)?;
    if id == 0 {
        return Err(missing());
    }
    Ok((multibody, multibody.link(id).ok_or_else(missing)?))
}

/// Copies the multibody joint configuration without returning a borrowed joint pointer.
/// @ingroup joints
#[rapier_export(multibody_joint)]
pub unsafe extern "C" fn rpr_multibody_joint_desc(handle: RprMultibodyJointHandle) -> RprJointDesc {
    let world = handle.world;
    ffi_value(|out: *mut RprJointDesc| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprMultibodyJointSet =
                std::ptr::addr_of!((*raw).0.multibody_joints).cast();

            let (_, link) = multibody_joint_link(&get(set)?.0, handle)?;
            output(out, link.joint.data.into())
        })
    })
}

/// Replaces the multibody joint configuration after validation. lockedAxes defines the degrees of
/// freedom of the multibody and cannot change: a different value reports INVALID_ARGUMENT.
/// wake_up = 1 wakes the two connected bodies; 0 preserves their sleep state.
/// @ingroup joints
#[rapier_export(multibody_joint)]
pub unsafe extern "C" fn rpr_multibody_joint_set_desc(
    handle: RprMultibodyJointHandle,
    desc: *const RprJointDesc,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let physics: *mut RprPhysicsWorld = raw;

        let desc = get(desc)?.raw()?;
        let wake_up = boolean(wake_up)?;
        let physics = &mut get_mut(physics)?.0;
        let (multibody, link) = multibody_joint_link(&physics.multibody_joints, handle)?;
        ensure(
            link.joint.data.locked_axes == desc.locked_axes,
            "the locked axes of a multibody joint cannot change",
        )?;
        let body2 = link.rigid_body_handle();
        let body1 = link
            .parent_id()
            .and_then(|id| multibody.link(id))
            .map(|l| l.rigid_body_handle());
        let (multibody, id) = physics
            .multibody_joints
            .get_mut(handle.raw())
            .ok_or_else(missing)?;
        multibody.link_mut(id).ok_or_else(missing)?.joint.data = desc;
        if wake_up {
            for body in body1.into_iter().chain([body2]) {
                if let Some(body) = physics.bodies.get_mut(body) {
                    body.wake_up(true);
                }
            }
        }
        Ok(())
    })
}

/// Return the two bodies connected by a multibody joint: its parent link, then its own link.
/// @ingroup joints
#[rapier_export(multibody_joint)]
pub unsafe extern "C" fn rpr_multibody_joint_bodies(
    handle: RprMultibodyJointHandle,
) -> RprJointBodies {
    let world = handle.world;
    ffi_world_value(world, |result: *mut RprJointBodies| {
        let body1 = unsafe { std::ptr::addr_of_mut!((*result).body1) };
        let body2 = unsafe { std::ptr::addr_of_mut!((*result).body2) };

        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprMultibodyJointSet =
                std::ptr::addr_of!((*raw).0.multibody_joints).cast();

            out_ptr(body1)?;
            out_ptr(body2)?;
            let (multibody, link) = multibody_joint_link(&get(set)?.0, handle)?;
            let parent = link
                .parent_id()
                .and_then(|id| multibody.link(id))
                .ok_or_else(missing)?;
            output(body1, parent.rigid_body_handle().into())?;
            output(body2, link.rigid_body_handle().into())
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ptr;

    unsafe fn body(world: *mut RprWorld, kind: u32, x: Real) -> RprRigidBodyHandle {
        unsafe {
            let mut desc = match kind {
                RPR_FIXED => rpr_fixed_rigid_body_desc(),
                _ => rpr_dynamic_rigid_body_desc(),
            };
            desc.position.translation = (Vector::X * x).into();
            let handle = rpr_insert_rigid_body(world, &desc);
            rpr_insert_collider(handle, &rpr_ball_collider_desc(0.1));
            assert_eq!(rpr_last_status(), RPR_OK);
            handle
        }
    }

    #[test]
    fn impulse_joint_impulses_user_data_and_counts() {
        unsafe {
            let world = rpr_new_world();
            let ground = body(world, RPR_FIXED, 0.0);
            let bob = body(world, RPR_DYNAMIC, 1.0);
            let mut desc = rpr_fixed_joint_desc();
            desc.localFrame1.translation = Vector::X.into();
            desc.userData = RprUserData { low: 7, high: 9 };
            let joint = rpr_insert_impulse_joint(ground, bob, &desc);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!(rpr_impulse_joint_count(world), 1);
            assert_eq!(rpr_multibody_joint_count(world), 0);
            let user_data = rpr_impulse_joint_user_data(joint);
            assert_eq!((user_data.low, user_data.high), (7, 9));

            let impulses = rpr_impulse_joint_impulses(joint);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!(impulses.linear.raw().unwrap(), Vector::ZERO);
            assert_eq!(rpr_step(world, ptr::null(), ptr::null()), RPR_OK);
            // The joint holds the body against gravity.
            let impulses = rpr_impulse_joint_impulses(joint);
            assert!(impulses.linear.raw().unwrap().length() > 0.0);
            assert_eq!(impulses.limits, [0.0; RPR_JOINT_DOF_COUNT]);

            assert_eq!(rpr_remove_impulse_joint(joint, 1), RPR_OK);
            assert_eq!(rpr_impulse_joint_count(world), 0);
            rpr_impulse_joint_impulses(joint);
            assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
            rpr_impulse_joint_user_data(joint);
            assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[test]
    fn multibody_joint_desc_bodies_and_stale_handles() {
        unsafe {
            let world = rpr_new_world();
            let root = body(world, RPR_FIXED, 0.0);
            let link1 = body(world, RPR_DYNAMIC, 1.0);
            let link2 = body(world, RPR_DYNAMIC, 2.0);
            let desc = rpr_prismatic_joint_desc(Vector::X.into());
            let joint1 = rpr_insert_multibody_joint(root, link1, &desc);
            let joint2 = rpr_insert_multibody_joint(link1, link2, &desc);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!(rpr_multibody_joint_count(world), 2);

            let bodies = rpr_multibody_joint_bodies(joint2);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!((bodies.body1, bodies.body2), (link1, link2));

            let mut changed = rpr_multibody_joint_desc(joint2);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!(changed.lockedAxes, desc.lockedAxes);
            assert_eq!(
                rpr_joint_desc_set_motor_velocity(&mut changed, RPR_AXIS_LIN_X, 1.0, 0.5),
                RPR_OK
            );
            assert_eq!(rpr_rigid_body_sleep(link2), RPR_OK);
            assert_eq!(rpr_multibody_joint_set_desc(joint2, &changed, 1), RPR_OK);
            assert_eq!(rpr_rigid_body_is_sleeping(link2), 0);
            assert_eq!(rpr_multibody_joint_desc(joint2).motors[0].targetVel, 1.0);

            // The locked axes define the multibody degrees of freedom.
            let mut locked = changed;
            locked.lockedAxes = rpr_fixed_joint_desc().lockedAxes;
            assert_eq!(
                rpr_multibody_joint_set_desc(joint2, &locked, 1),
                RPR_INVALID_ARGUMENT
            );
            assert_eq!(rpr_multibody_joint_desc(joint2).lockedAxes, desc.lockedAxes);
            // Until the next step, the fixed root still counts as a free root.
            assert!(rpr_multibody_joint_ndofs(joint2) > 2);
            assert_eq!(rpr_step(world, ptr::null(), ptr::null()), RPR_OK);
            assert_eq!(rpr_multibody_joint_ndofs(joint2), 2);

            // Removing the first joint makes link1 the root of a new multibody. Its old handle
            // resolves to that root link and must be rejected.
            assert_eq!(rpr_remove_multibody_joint(joint1, 1), RPR_OK);
            assert_eq!(rpr_multibody_joint_count(world), 1);
            let access = (*world).read().unwrap();
            let native = &(*access.raw()).0.multibody_joints;
            assert_eq!(native.get(joint1.raw()).map(|(_, id)| id), Some(0));
            drop(access);
            rpr_multibody_joint_desc(joint1);
            assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
            rpr_multibody_joint_bodies(joint1);
            assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
            assert_eq!(rpr_remove_multibody_joint(joint1, 1), RPR_INVALID_HANDLE);
            assert_eq!(rpr_multibody_joint_count(world), 1);
            assert_eq!(rpr_multibody_joint_bodies(joint2).body1, link1);
            assert_eq!(rpr_step(world, ptr::null(), ptr::null()), RPR_OK);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }
}
