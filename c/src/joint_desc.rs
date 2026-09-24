//! Plain joint configuration, distinct from a borrowed live joint.
#![allow(non_snake_case)]
use crate::*;
/// @ingroup joints
/// Number of translational and angular joint axes in this dimension.
#[cfg(feature = "dim2")]
pub const RPR_JOINT_DOF_COUNT: usize = 3;
/// @ingroup joints
/// Number of translational and angular joint axes in this dimension.
#[cfg(feature = "dim3")]
pub const RPR_JOINT_DOF_COUNT: usize = 6;
/// Lower and upper axis limits in length units or radians.
/// @ingroup joints
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprJointLimits {
    /// Minimum allowed axis displacement (length or radians).
    pub min: RprReal,
    /// Maximum allowed axis displacement (length or radians).
    pub max: RprReal,
}
/// Position/velocity motor settings for one joint axis.
/// @ingroup joints
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprJointMotor {
    /// Motor target velocity (length per second or radians per second).
    pub targetVel: RprReal,
    /// Motor target position (length or radians).
    pub targetPos: RprReal,
    /// Nonnegative motor spring stiffness.
    pub stiffness: RprReal,
    /// Nonnegative motor damping.
    pub damping: RprReal,
    /// Nonnegative maximum force or torque.
    pub maxForce: RprReal,
    /// Motor model: 0 acceleration-based, 1 force-based.
    pub model: u32,
}
/// Copyable joint configuration. Limits/motors take effect when their axis mask is enabled.
/// Solver impulses are deliberately excluded. Applying data resets cached limit and motor impulses.
/// @ingroup joints
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprJointDesc {
    /// Joint frame in body 1 local coordinates.
    pub localFrame1: RprPose,
    /// Joint frame in body 2 local coordinates.
    pub localFrame2: RprPose,
    /// Locked joint degrees of freedom; translations precede rotations.
    pub lockedAxes: u8,
    /// Axis mask enabling corresponding limits entries.
    pub limitAxes: u8,
    /// Axis mask enabling corresponding motors entries.
    pub motorAxes: u8,
    /// Axis mask sharing a coupled constraint.
    pub coupledAxes: u8,
    /// Axis limits in translation-then-rotation order.
    pub limits: [RprJointLimits; RPR_JOINT_DOF_COUNT],
    /// Axis motors in translation-then-rotation order.
    pub motors: [RprJointMotor; RPR_JOINT_DOF_COUNT],
    /// Spring coefficients for constraint correction.
    pub softness: RprSpringCoefficients,
    /// Whether connected bodies may collide.
    pub contactsEnabled: RprBool,
    /// Whether this setting/object is enabled (0 or 1).
    pub enabled: RprBool,
    /// Application data; Rapier does not own pointers encoded in it.
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
/// Return native default joint desc. This POD value owns no resources.
/// @ingroup joints
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

/// Return a fixed joint description with native defaults; no allocation.
/// @ingroup joints
#[rapier_export]
pub extern "C" fn rpr_fixed_joint_desc() -> RprJointDesc {
    GenericJoint::from(FixedJointBuilder::new().build()).into()
}
/// Return a revolute joint description with native defaults; no allocation.
/// @ingroup joints
#[cfg(feature = "dim2")]
#[rapier_export]
pub extern "C" fn rpr_revolute_joint_desc() -> RprJointDesc {
    GenericJoint::from(RevoluteJointBuilder::new().build()).into()
}
/// Returns a joint description. Invalid axes produce nonfinite frames, rejected on insertion.
/// @ingroup joints
#[cfg(feature = "dim3")]
#[rapier_export]
pub extern "C" fn rpr_revolute_joint_desc(axis_vector: RprVector) -> RprJointDesc {
    joint_desc_with_axis(JointAxesMask::LOCKED_REVOLUTE_AXES, axis_vector)
}
/// Returns a joint description. Invalid axes produce nonfinite frames, rejected on insertion.
/// @ingroup joints
#[rapier_export]
pub extern "C" fn rpr_prismatic_joint_desc(axis_vector: RprVector) -> RprJointDesc {
    joint_desc_with_axis(JointAxesMask::LOCKED_PRISMATIC_AXES, axis_vector)
}
/// Return a rope joint description with native defaults; no allocation.
/// @ingroup joints
#[rapier_export]
pub extern "C" fn rpr_rope_joint_desc(length: RprReal) -> RprJointDesc {
    GenericJoint::from(RopeJointBuilder::new(length).build()).into()
}
/// Return a spring joint description with native defaults; no allocation.
/// @ingroup joints
#[rapier_export]
pub extern "C" fn rpr_spring_joint_desc(
    length: RprReal,
    stiffness: RprReal,
    damping: RprReal,
) -> RprJointDesc {
    GenericJoint::from(SpringJointBuilder::new(length, stiffness, damping).build()).into()
}
/// Return a spherical joint description with native defaults; no allocation.
/// @ingroup joints
#[cfg(feature = "dim3")]
#[rapier_export]
pub extern "C" fn rpr_spherical_joint_desc() -> RprJointDesc {
    GenericJoint::from(SphericalJointBuilder::new().build()).into()
}
/// Returns a joint description. Invalid axes produce nonfinite frames, rejected on insertion.
/// @ingroup joints
#[cfg(feature = "dim2")]
#[rapier_export]
pub extern "C" fn rpr_pin_slot_joint_desc(axis_vector: RprVector) -> RprJointDesc {
    joint_desc_with_axis(JointAxesMask::LOCKED_PIN_SLOT_AXES, axis_vector)
}
/// Create an impulse joint connecting two bodies in the same world. The world owns the joint;
/// wake_up wakes the connected bodies.
/// @ingroup joints
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

/// Create an articulation joint between bodies in the same world. Returns an invalid handle on
/// failure; check rpr_last_status.
/// @ingroup joints
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
