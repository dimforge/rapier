use rapier3d::{
    dynamics::{
        GenericJoint, GenericJointBuilder, ImpulseJointHandle, ImpulseJointSet, JointAxesMask,
        JointAxis, MassProperties, MultibodyJointHandle, MultibodyJointSet, RigidBody,
        RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
    },
    geometry::{Collider, ColliderBuilder, ColliderHandle, ColliderSet},
    math::{Isometry, Point, Vector},
    na,
};
use std::collections::HashMap;
use std::path::Path;
use urdf_rs::{Collision, Geometry, Inertial, Joint, JointType, Pose, Robot, UrdfError};

pub type LinkId = usize;

/// An urdf link loaded as a rapier [`RigidBody`] and its [`Collider`]s.
#[derive(Clone)]
pub struct RapierLink {
    pub body: RigidBody,
    pub colliders: Vec<Collider>,
}

/// An urdf joint loaded as a rapier [`GenericJoint`].
#[derive(Clone)]
pub struct RapierJoint {
    /// The rapier version for the corresponding urdf joint.
    pub joint: GenericJoint,
    /// Index of the rigid-body (from the [`RapierRobot`] array) at the first
    /// endpoint of this joint.
    pub link1: LinkId,
    /// Index of the rigid-body (from the [`RapierRobot`] array) at the second
    /// endpoint of this joint.
    pub link2: LinkId,
}

/// A robot represented as a set of rapier rigid-bodies, colliders, and joints.
#[derive(Clone)]
pub struct RapierRobot {
    /// The bodies and colliders loaded from the urdf file.
    ///
    /// This vector matches the order of [`Robot::links`].
    pub links: Vec<RapierLink>,
    /// The joints loaded from the urdf file.
    ///
    /// This vector matches the order of [`Robot::joints`].
    pub joints: Vec<RapierJoint>,
}

pub struct RapierJointHandle<JointHandle> {
    pub joint: JointHandle,
    pub link1: RigidBodyHandle,
    pub link2: RigidBodyHandle,
}

pub struct RapierLinkHandle {
    pub body: RigidBodyHandle,
    pub colliders: Vec<ColliderHandle>,
}

pub struct RapierRobotHandles<JointHandle> {
    pub links: Vec<RapierLinkHandle>,
    pub joints: Vec<RapierJointHandle<JointHandle>>,
}

impl RapierRobot {
    pub fn from_file(path: impl AsRef<Path>) -> urdf_rs::Result<(Self, Robot)> {
        let robot = urdf_rs::read_file(path)?;
        Ok((Self::from_robot(&robot), robot))
    }

    pub fn from_str(str: &str) -> urdf_rs::Result<(Self, Robot)> {
        let robot = urdf_rs::read_from_string(str)?;
        Ok((Self::from_robot(&robot), robot))
    }

    pub fn from_robot(robot: &Robot) -> Self {
        let mut name_to_link_id = HashMap::new();

        let links: Vec<_> = robot
            .links
            .iter()
            .enumerate()
            .map(|(id, link)| {
                name_to_link_id.insert(&link.name, id);
                let colliders: Vec<_> = link
                    .collision
                    .iter()
                    .map(|co| urdf_to_collider(co))
                    .collect();
                let body = urdf_to_rigid_body(&link.inertial);
                RapierLink { body, colliders }
            })
            .collect();
        let joints: Vec<_> = robot
            .joints
            .iter()
            .map(|joint| {
                let link1 = name_to_link_id[&joint.parent.link];
                let link2 = name_to_link_id[&joint.child.link];
                let joint = urdf_to_joint(joint);
                RapierJoint {
                    joint,
                    link1,
                    link2,
                }
            })
            .collect();

        Self { links, joints }
    }

    pub fn insert_using_impulse_joints(
        self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        joint_set: &mut ImpulseJointSet,
    ) -> RapierRobotHandles<ImpulseJointHandle> {
        let links: Vec<_> = self
            .links
            .into_iter()
            .map(|link| {
                let body = rigid_body_set.insert(link.body);
                let colliders = link
                    .colliders
                    .into_iter()
                    .map(|co| collider_set.insert_with_parent(co, body, rigid_body_set))
                    .collect();
                RapierLinkHandle { body, colliders }
            })
            .collect();
        let joints: Vec<_> = self
            .joints
            .into_iter()
            .map(|joint| {
                let link1 = links[joint.link1].body;
                let link2 = links[joint.link2].body;
                let joint = joint_set.insert(link1, link2, joint.joint, true);
                RapierJointHandle {
                    joint,
                    link1,
                    link2,
                }
            })
            .collect();

        RapierRobotHandles { links, joints }
    }
    pub fn insert_using_multibody_joints(
        self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        joint_set: &mut MultibodyJointSet,
    ) -> RapierRobotHandles<Option<MultibodyJointHandle>> {
        let links: Vec<_> = self
            .links
            .into_iter()
            .map(|link| {
                let body = rigid_body_set.insert(link.body);
                let colliders = link
                    .colliders
                    .into_iter()
                    .map(|co| collider_set.insert_with_parent(co, body, rigid_body_set))
                    .collect();
                RapierLinkHandle { body, colliders }
            })
            .collect();
        let joints: Vec<_> = self
            .joints
            .into_iter()
            .map(|joint| {
                let link1 = links[joint.link1].body;
                let link2 = links[joint.link2].body;
                let joint = joint_set.insert(link1, link2, joint.joint, true);
                RapierJointHandle {
                    joint,
                    link1,
                    link2,
                }
            })
            .collect();

        RapierRobotHandles { links, joints }
    }
}

fn urdf_to_rigid_body(inertial: &Inertial) -> RigidBody {
    RigidBodyBuilder::dynamic()
        .position(urdf_to_isometry(&inertial.origin))
        .additional_mass_properties(MassProperties::with_inertia_matrix(
            Point::origin(),
            inertial.mass.value as f32,
            na::Matrix3::new(
                inertial.inertia.ixx as f32,
                inertial.inertia.ixy as f32,
                inertial.inertia.ixz as f32,
                inertial.inertia.ixy as f32,
                inertial.inertia.iyy as f32,
                inertial.inertia.iyz as f32,
                inertial.inertia.ixz as f32,
                inertial.inertia.iyz as f32,
                inertial.inertia.izz as f32,
            ),
        ))
        .build()
}

fn urdf_to_collider(co: &Collision) -> Collider {
    let builder = match &co.geometry {
        Geometry::Box { size } => ColliderBuilder::cuboid(
            size[0] as f32 / 2.0,
            size[1] as f32 / 2.0,
            size[2] as f32 / 2.0,
        ),
        Geometry::Cylinder { radius, length } => {
            ColliderBuilder::cylinder(*length as f32 / 2.0, *radius as f32)
        }
        Geometry::Capsule { radius, length } => {
            ColliderBuilder::capsule_y(*length as f32 / 2.0, *radius as f32)
        }
        Geometry::Sphere { radius } => ColliderBuilder::ball(*radius as f32),
        Geometry::Mesh { filename, scale } => todo!(),
    };

    builder
        .position(urdf_to_isometry(&co.origin))
        .density(0.0)
        .build()
}

fn urdf_to_isometry(pose: &Pose) -> Isometry<f32> {
    Isometry::from_parts(
        Point::new(pose.xyz[0] as f32, pose.xyz[1] as f32, pose.xyz[2] as f32).into(),
        na::UnitQuaternion::from_euler_angles(
            pose.rpy[0] as f32,
            pose.rpy[1] as f32,
            pose.rpy[2] as f32,
        ),
    )
}

fn urdf_to_joint(joint: &Joint) -> GenericJoint {
    let locked_axes = match joint.joint_type {
        JointType::Fixed => JointAxesMask::LOCKED_FIXED_AXES,
        JointType::Continuous | JointType::Revolute => JointAxesMask::LOCKED_REVOLUTE_AXES,
        JointType::Floating => JointAxesMask::empty(),
        JointType::Planar => JointAxesMask::ANG_AXES | JointAxesMask::X,
        JointType::Prismatic => JointAxesMask::LOCKED_PRISMATIC_AXES,
        JointType::Spherical => JointAxesMask::LOCKED_SPHERICAL_AXES,
    };
    let joint_to_parent = urdf_to_isometry(&joint.origin);
    let joint_axis = na::Unit::new_normalize(Vector::new(
        joint.axis.xyz[0] as f32,
        joint.axis.xyz[1] as f32,
        joint.axis.xyz[2] as f32,
    ));

    let mut builder = GenericJointBuilder::new(locked_axes)
        .local_axis1(joint_to_parent * joint_axis)
        .local_axis2(joint_axis)
        .local_anchor1(joint_to_parent.translation.vector.into());

    match joint.joint_type {
        JointType::Revolute | JointType::Prismatic => {
            builder = builder.limits(
                JointAxis::X,
                [joint.limit.lower as f32, joint.limit.upper as f32],
            )
        }
        _ => {}
    }

    // TODO: the following fields are currently ignored:
    //       - Joint::dynamics
    //       - Joint::limit.effort / limit.velocity
    //       - Joint::mimic
    //       - Joint::safety_controller
    builder.build()
}
