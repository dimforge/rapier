//! Handles produced by inserting an [`MjcfRobot`](super::MjcfRobot) into
//! rapier. The body / joint / actuator handles are organized parallel to
//! their counterparts in [`MjcfRobot`](super::MjcfRobot) so callers can
//! cross-reference by index.

use mjcf_rs::extras::Actuator as MjcfActuator;

use rapier3d::dynamics::{ImpulseJointHandle, RigidBodyHandle};
use rapier3d::geometry::ColliderHandle;

/// A handle to one inserted collider.
#[derive(Clone, Debug)]
pub struct MjcfColliderHandle {
    /// Rapier collider handle.
    pub handle: ColliderHandle,
}

/// A handle to one inserted rigid-body.
#[derive(Clone, Debug)]
pub struct MjcfBodyHandle {
    /// Rapier rigid-body handle.
    pub body: RigidBodyHandle,
    /// Inserted colliders.
    pub colliders: Vec<MjcfColliderHandle>,
}

/// A handle to one inserted joint (typed by the joint set used).
#[derive(Clone, Debug)]
pub struct MjcfJointHandle<H> {
    /// Joint handle (kind depends on the joint set).
    pub joint: H,
    /// Parent rigid-body.
    pub link1: RigidBodyHandle,
    /// Child rigid-body.
    pub link2: RigidBodyHandle,
}

/// Per-actuator handle resolved against the inserted joint set. Returned
/// alongside [`MjcfRobotHandles::joints`] so callers can drive each
/// actuator without having to walk the actuator → joint-index → handle
/// indirection themselves.
#[derive(Clone, Debug)]
pub struct MjcfActuatorHandle<H> {
    /// Original actuator metadata.
    pub actuator: MjcfActuator,
    /// Joint handle the actuator drives, if any.
    pub joint: Option<H>,
}

/// All handles produced by inserting a [`MjcfRobot`](super::MjcfRobot) into rapier.
#[derive(Clone, Debug)]
pub struct MjcfRobotHandles<H> {
    /// Per-body handles. Aligned with [`MjcfRobot::bodies`](super::MjcfRobot::bodies) (entry 0 is
    /// the world body and is left at `None`).
    pub bodies: Vec<Option<MjcfBodyHandle>>,
    /// Per-joint handles, aligned with [`MjcfRobot::joints`](super::MjcfRobot::joints).
    pub joints: Vec<MjcfJointHandle<H>>,
    /// Per-equality-constraint handles. Always inserted as impulse joints.
    pub equality_joints: Vec<MjcfJointHandle<ImpulseJointHandle>>,
    /// Per-actuator handles, aligned with [`MjcfRobot::actuators`](super::MjcfRobot::actuators).
    pub actuators: Vec<MjcfActuatorHandle<H>>,
}
