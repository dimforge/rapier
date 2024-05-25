use rapier3d::{
    dynamics::{GenericJoint, RigidBody},
    geometry::{Collider, ColliderBuilder, SharedShape},
    math::{Isometry, Point, Vector},
    na,
};
use std::path::Path;
use urdf_rs::{Collision, Geometry, Inertial, Pose, Robot, UrdfError};

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
    /// This vector matches the order of the loaded [`Robot::links`].
    pub links: Vec<RapierLink>,
    /// The joints loaded from the urdf file.
    ///
    /// This vector matches the order of the loaded [`Robot::joints`].
    pub joints: Vec<RapierLink>,
}

impl RapierRobot {
    pub fn from_file(path: impl AsRef<Path>) -> urdf_rs::Result<(Self, Robot)> {
        let robot = urdf_rs::read_from_string(
            path.as_ref()
                .to_str()
                .ok_or_else(|| UrdfError::from("file path contains unsupported characters"))?,
        )?;
        Ok((Self::from_robot(&robot), robot))
    }

    pub fn from_str(str: &str) -> urdf_rs::Result<(Self, Robot)> {
        let robot = urdf_rs::read_from_string(str)?;
        Ok((Self::from_robot(&robot), robot))
    }

    pub fn from_robot(robot: &Robot) -> Self {
        let links: Vec<_> = robot
            .links
            .iter()
            .map(|link| {
                let colliders: Vec<_> = link
                    .collision
                    .iter()
                    .map(|co| urdf_to_collider(co))
                    .collect();
                let body = urdf_to_rigid_body(&link.inertial);
                RapierLink { body, colliders }
            })
            .collect();
        todo!()
    }
}

fn urdf_to_rigid_body(inertial: &Inertial) -> RigidBody {
    RigidBodyBuilder::dynamic().mass_props();
}

fn urdf_to_collider(co: &Collision) -> Collider {
    let mut builder = match &co.geometry {
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
        .position(pose_to_isometry(&co.origin))
        .density(0.0)
        .build()
}

fn pose_to_isometry(pose: &Pose) -> Isometry<f32> {
    Isometry::from_parts(
        Point::new(pose.xyz[0] as f32, pose.xyz[1] as f32, pose.xyz[2] as f32).into(),
        na::UnitQuaternion::from_euler_angles(
            pose.rpy[0] as f32,
            pose.rpy[1] as f32,
            pose.rpy[2] as f32,
        ),
    )
}
