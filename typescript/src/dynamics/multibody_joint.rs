use crate::dynamics::{RawJointAxis, RawJointType, RawMultibodyJointSet};
use crate::math::{RawRotation, RawVector};
use crate::utils::FlatHandle;
use rapier::dynamics::JointAxis;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl RawMultibodyJointSet {
    /// The type of this joint.
    pub fn jointType(&self, handle: FlatHandle) -> RawJointType {
        self.map(handle, |j| j.data.locked_axes.into())
    }

    // /// The unique integer identifier of the first rigid-body this joint it attached to.
    // pub fn jointBodyHandle1(&self, handle: FlatHandle) -> u32 {
    //     self.map(handle, |j| j.body1.into_raw_parts().0)
    // }
    //
    // /// The unique integer identifier of the second rigid-body this joint is attached to.
    // pub fn jointBodyHandle2(&self, handle: FlatHandle) -> u32 {
    //     self.map(handle, |j| j.body2.into_raw_parts().0)
    // }

    /// The angular part of the joint’s local frame relative to the first rigid-body it is attached to.
    pub fn jointFrameX1(&self, handle: FlatHandle) -> RawRotation {
        self.map(handle, |j| j.data.local_frame1.rotation.into())
    }

    /// The angular part of the joint’s local frame relative to the second rigid-body it is attached to.
    pub fn jointFrameX2(&self, handle: FlatHandle) -> RawRotation {
        self.map(handle, |j| j.data.local_frame2.rotation.into())
    }

    /// The position of the first anchor of this joint.
    ///
    /// The first anchor gives the position of the points application point on the
    /// local frame of the first rigid-body it is attached to.
    pub fn jointAnchor1(&self, handle: FlatHandle) -> RawVector {
        self.map(handle, |j| j.data.local_frame1.translation.vector.into())
    }

    /// The position of the second anchor of this joint.
    ///
    /// The second anchor gives the position of the points application point on the
    /// local frame of the second rigid-body it is attached to.
    pub fn jointAnchor2(&self, handle: FlatHandle) -> RawVector {
        self.map(handle, |j| j.data.local_frame2.translation.vector.into())
    }

    /// Are contacts between the rigid-bodies attached by this joint enabled?
    pub fn jointContactsEnabled(&self, handle: FlatHandle) -> bool {
        self.map(handle, |j| j.data.contacts_enabled)
    }

    /// Sets whether contacts are enabled between the rigid-bodies attached by this joint.
    pub fn jointSetContactsEnabled(&mut self, handle: FlatHandle, enabled: bool) {
        self.map_mut(handle, |j| {
            j.data.contacts_enabled = enabled;
        });
    }

    /// Are the limits for this joint enabled?
    pub fn jointLimitsEnabled(&self, handle: FlatHandle, axis: RawJointAxis) -> bool {
        self.map(handle, |j| {
            j.data.limit_axes.contains(JointAxis::from(axis).into())
        })
    }

    /// Return the lower limit along the given joint axis.
    pub fn jointLimitsMin(&self, handle: FlatHandle, axis: RawJointAxis) -> f32 {
        self.map(handle, |j| j.data.limits[axis as usize].min)
    }

    /// If this is a prismatic joint, returns its upper limit.
    pub fn jointLimitsMax(&self, handle: FlatHandle, axis: RawJointAxis) -> f32 {
        self.map(handle, |j| j.data.limits[axis as usize].max)
    }

    // pub fn jointConfigureMotorModel(
    //     &mut self,
    //     handle: FlatHandle,
    //     axis: RawJointAxis,
    //     model: RawMotorModel,
    // ) {
    //     self.map_mut(handle, |j| {
    //         j.data.motors[axis as usize].model = model.into()
    //     })
    // }

    /*
    #[cfg(feature = "dim3")]
    pub fn jointConfigureBallMotorVelocity(
        &mut self,
        handle: FlatHandle,
        vx: f32,
        vy: f32,
        vz: f32,
        factor: f32,
    ) {
        let targetVel = Vector3::new(vx, vy, vz);

        self.map_mut(handle, |j| match &mut j.params {
            JointData::SphericalJoint(j) => j.configure_motor_velocity(targetVel, factor),
            _ => {}
        })
    }

    #[cfg(feature = "dim3")]
    pub fn jointConfigureBallMotorPosition(
        &mut self,
        handle: FlatHandle,
        qw: f32,
        qx: f32,
        qy: f32,
        qz: f32,
        stiffness: f32,
        damping: f32,
    ) {
        let quat = Quaternion::new(qw, qx, qy, qz);

        self.map_mut(handle, |j| match &mut j.params {
            JointData::SphericalJoint(j) => {
                if let Some(unit_quat) = UnitQuaternion::try_new(quat, 1.0e-5) {
                    j.configure_motor_position(unit_quat, stiffness, damping)
                }
            }
            _ => {}
        })
    }

    #[cfg(feature = "dim3")]
    pub fn jointConfigureBallMotor(
        &mut self,
        handle: FlatHandle,
        qw: f32,
        qx: f32,
        qy: f32,
        qz: f32,
        vx: f32,
        vy: f32,
        vz: f32,
        stiffness: f32,
        damping: f32,
    ) {
        let quat = Quaternion::new(qw, qx, qy, qz);
        let vel = Vector3::new(vx, vy, vz);

        self.map_mut(handle, |j| match &mut j.params {
            JointData::SphericalJoint(j) => {
                if let Some(unit_quat) = UnitQuaternion::try_new(quat, 1.0e-5) {
                    j.configure_motor(unit_quat, vel, stiffness, damping)
                }
            }
            _ => {}
        })
    }
    */

    // pub fn jointConfigureMotorVelocity(
    //     &mut self,
    //     handle: FlatHandle,
    //     axis: RawJointAxis,
    //     targetVel: f32,
    //     factor: f32,
    // ) {
    //     self.jointConfigureMotor(handle, axis, 0.0, targetVel, 0.0, factor)
    // }
    //
    // pub fn jointConfigureMotorPosition(
    //     &mut self,
    //     handle: FlatHandle,
    //     axis: RawJointAxis,
    //     targetPos: f32,
    //     stiffness: f32,
    //     damping: f32,
    // ) {
    //     self.jointConfigureMotor(handle, axis, targetPos, 0.0, stiffness, damping)
    // }

    // pub fn jointConfigureMotor(
    //     &mut self,
    //     handle: FlatHandle,
    //     axis: RawJointAxis,
    //     targetPos: f32,
    //     targetVel: f32,
    //     stiffness: f32,
    //     damping: f32,
    // ) {
    //     self.map_mut(handle, |j| {
    //         j.data.motors[axis as usize].target_pos = targetPos;
    //         j.data.motors[axis as usize].target_vel = targetVel;
    //         j.data.motors[axis as usize].stiffness = stiffness;
    //         j.data.motors[axis as usize].damping = damping;
    //     })
    // }
}
