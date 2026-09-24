//! Plain joint configuration, distinct from a borrowed live joint.
#![allow(non_snake_case)]
use crate::*;
#[cfg(feature = "dim2")]
pub const RPR_JOINT_DOF_COUNT: usize = 3;
#[cfg(feature = "dim3")]
pub const RPR_JOINT_DOF_COUNT: usize = 6;
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprJointLimits {
    pub min: RprReal,
    pub max: RprReal,
}
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprJointMotor {
    pub targetVel: RprReal,
    pub targetPos: RprReal,
    pub stiffness: RprReal,
    pub damping: RprReal,
    pub maxForce: RprReal,
    pub model: u32,
}
/// Copyable joint configuration. Limits/motors take effect when their axis mask is enabled.
/// Solver impulses are deliberately excluded. Applying data resets cached limit and motor impulses.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprJointDesc {
    pub localFrame1: RprPose,
    pub localFrame2: RprPose,
    pub lockedAxes: u8,
    pub limitAxes: u8,
    pub motorAxes: u8,
    pub coupledAxes: u8,
    pub limits: [RprJointLimits; RPR_JOINT_DOF_COUNT],
    pub motors: [RprJointMotor; RPR_JOINT_DOF_COUNT],
    pub softness: RprSpringCoefficients,
    pub contactsEnabled: RprBool,
    pub enabled: RprBool,
    pub userData: RprUserData,
}
impl From<GenericJoint> for RprJointDesc {
    fn from(j: GenericJoint) -> Self {
        Self {
            localFrame1: j.local_frame1.into(),
            localFrame2: j.local_frame2.into(),
            lockedAxes: j.locked_axes.bits(),
            limitAxes: j.limit_axes.bits(),
            motorAxes: j.motor_axes.bits(),
            coupledAxes: j.coupled_axes.bits(),
            limits: j.limits.map(|v| RprJointLimits {
                min: v.min,
                max: v.max,
            }),
            motors: j.motors.map(|v| RprJointMotor {
                targetVel: v.target_vel,
                targetPos: v.target_pos,
                stiffness: v.stiffness,
                damping: v.damping,
                maxForce: v.max_force,
                model: match v.model {
                    MotorModel::AccelerationBased => 0,
                    MotorModel::ForceBased => 1,
                },
            }),
            softness: j.softness.into(),
            contactsEnabled: j.contacts_enabled as _,
            enabled: j.is_enabled() as _,
            userData: j.user_data.into(),
        }
    }
}
impl RprJointDesc {
    pub(crate) fn raw(&self) -> Result<GenericJoint> {
        let mut j = GenericJoint::new(axes(self.lockedAxes)?);
        j.local_frame1 = self.localFrame1.raw()?;
        j.local_frame2 = self.localFrame2.raw()?;
        j.limit_axes = axes(self.limitAxes)?;
        j.motor_axes = axes(self.motorAxes)?;
        j.coupled_axes = axes(self.coupledAxes)?;
        j.softness = self.softness.raw()?;
        j.contacts_enabled = boolean(self.contactsEnabled)?;
        j.set_enabled(boolean(self.enabled)?);
        j.user_data = self.userData.raw();
        for i in 0..RPR_JOINT_DOF_COUNT {
            let l = self.limits[i];
            ensure(
                !l.min.is_nan() && !l.max.is_nan(),
                "joint limits must not be NaN",
            )?;
            ensure(l.min <= l.max, "reversed joint limits")?;
            j.limits[i].min = l.min;
            j.limits[i].max = l.max;
            let m = self.motors[i];
            let dst = &mut j.motors[i];
            dst.target_vel = finite(m.targetVel)?;
            dst.target_pos = finite(m.targetPos)?;
            dst.stiffness = nonnegative(m.stiffness)?;
            dst.damping = nonnegative(m.damping)?;
            dst.max_force = nonnegative(m.maxForce)?;
            dst.model = match m.model {
                0 => MotorModel::AccelerationBased,
                1 => MotorModel::ForceBased,
                _ => return Err(invalid("invalid motor model")),
            };
        }
        Ok(j)
    }
}
#[rapier_export]
pub extern "C" fn rpr_default_joint_desc() -> RprJointDesc {
    GenericJoint::new(JointAxesMask::empty()).into()
}

// Axis normalization cannot fail across the C boundary. Invalid directions produce
// nonfinite frames, which the normal description validation rejects at insertion.
fn joint_desc_with_axis(locked_axes: JointAxesMask, axis: RprVector) -> RprJointDesc {
    let mut desc = RprJointDesc::from(GenericJoint::new(locked_axes));
    let rotation = match axis.raw().ok().and_then(|v| v.try_normalize()) {
        Some(axis) => GenericJoint::complete_ang_frame(axis).into(),
        None => {
            #[cfg(feature = "dim2")]
            {
                RprRotation { angle: Real::NAN }
            }
            #[cfg(feature = "dim3")]
            {
                RprRotation {
                    x: Real::NAN,
                    y: Real::NAN,
                    z: Real::NAN,
                    w: Real::NAN,
                }
            }
        }
    };
    desc.localFrame1.rotation = rotation;
    desc.localFrame2.rotation = rotation;
    desc
}

#[rapier_export]
pub extern "C" fn rpr_fixed_joint_desc() -> RprJointDesc {
    GenericJoint::from(FixedJointBuilder::new().build()).into()
}
#[cfg(feature = "dim2")]
#[rapier_export]
pub extern "C" fn rpr_revolute_joint_desc() -> RprJointDesc {
    GenericJoint::from(RevoluteJointBuilder::new().build()).into()
}
/// Returns a joint description. Invalid axes produce nonfinite frames, rejected on insertion.
#[cfg(feature = "dim3")]
#[rapier_export]
pub extern "C" fn rpr_revolute_joint_desc(axis_vector: RprVector) -> RprJointDesc {
    joint_desc_with_axis(JointAxesMask::LOCKED_REVOLUTE_AXES, axis_vector)
}
/// Returns a joint description. Invalid axes produce nonfinite frames, rejected on insertion.
#[rapier_export]
pub extern "C" fn rpr_prismatic_joint_desc(axis_vector: RprVector) -> RprJointDesc {
    joint_desc_with_axis(JointAxesMask::LOCKED_PRISMATIC_AXES, axis_vector)
}
#[rapier_export]
pub extern "C" fn rpr_rope_joint_desc(length: RprReal) -> RprJointDesc {
    GenericJoint::from(RopeJointBuilder::new(length).build()).into()
}
#[rapier_export]
pub extern "C" fn rpr_spring_joint_desc(
    length: RprReal,
    stiffness: RprReal,
    damping: RprReal,
) -> RprJointDesc {
    GenericJoint::from(SpringJointBuilder::new(length, stiffness, damping).build()).into()
}
#[cfg(feature = "dim3")]
#[rapier_export]
pub extern "C" fn rpr_spherical_joint_desc() -> RprJointDesc {
    GenericJoint::from(SphericalJointBuilder::new().build()).into()
}
#[cfg(feature = "dim2")]
/// Returns a joint description. Invalid axes produce nonfinite frames, rejected on insertion.
#[rapier_export]
pub extern "C" fn rpr_pin_slot_joint_desc(axis_vector: RprVector) -> RprJointDesc {
    joint_desc_with_axis(JointAxesMask::LOCKED_PIN_SLOT_AXES, axis_vector)
}
#[rapier_export]
pub unsafe extern "C" fn rpr_insert_impulse_joint(
    body1: RprRigidBodyHandle,
    body2: RprRigidBodyHandle,
    joint: *const RprJointDesc,
) -> RprImpulseJointHandle {
    let world = body1.world;
    ffi_world_value(world, |out: *mut RprImpulseJointHandle| {
        ffi(|| unsafe {
            body1.check_world(world)?;
            body2.check_world(world)?;
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            if !out.is_null() {
                out_ptr(out)?;
            }
            let joint = get(joint)?.raw()?;
            let world = &mut get_mut(world)?.0;
            world.bodies.get(body1.raw()).ok_or_else(missing)?;
            world.bodies.get(body2.raw()).ok_or_else(missing)?;
            ensure(body1 != body2, "joint endpoints must differ")?;
            let handle = world.insert_impulse_joint(body1.raw(), body2.raw(), joint);
            if !out.is_null() {
                output(out, handle.into())?;
            }
            Ok(())
        })
    })
}

#[rapier_export]
pub unsafe extern "C" fn rpr_insert_multibody_joint(
    body1: RprRigidBodyHandle,
    body2: RprRigidBodyHandle,
    joint: *const RprJointDesc,
) -> RprMultibodyJointHandle {
    let world = body1.world;
    ffi_world_value(world, |out: *mut RprMultibodyJointHandle| {
        ffi(|| unsafe {
            body1.check_world(world)?;
            body2.check_world(world)?;
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            if !out.is_null() {
                out_ptr(out)?;
            }
            let joint = get(joint)?.raw()?;
            let world = &mut get_mut(world)?.0;
            world.bodies.get(body1.raw()).ok_or_else(missing)?;
            world.bodies.get(body2.raw()).ok_or_else(missing)?;
            ensure(body1 != body2, "joint endpoints must differ")?;
            let handle = world
                .insert_multibody_joint(body1.raw(), body2.raw(), joint)
                .ok_or_else(|| invalid("multibody loop or duplicate link"))?;
            if !out.is_null() {
                output(out, handle.into())?;
            }
            Ok(())
        })
    })
}
