use na::{RealField, UnitQuaternion};
use rapier3d::{
    dynamics::{
        GenericJoint, GenericJointBuilder, ImpulseJointHandle, ImpulseJointSet, JointAxesMask,
        JointAxis, MassProperties, MultibodyJointHandle, MultibodyJointSet, RigidBody,
        RigidBodyBuilder, RigidBodyHandle, RigidBodySet, RigidBodyType,
    },
    geometry::{
        Collider, ColliderBuilder, ColliderHandle, ColliderSet, MeshConverter, SharedShape,
    },
    math::{Isometry, Point, Real, Vector},
    na,
};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use xurdf::{Collision, Geometry, Inertial, Joint, Pose, Robot};

pub type LinkId = usize;

#[derive(Clone, Debug)]
pub struct UrdfLoaderOptions {
    pub create_colliders_from_collision_shapes: bool,
    pub create_colliders_from_visual_shapes: bool,
    pub apply_imported_mass_props: bool,
    pub enable_joint_collisions: bool,
    pub make_roots_fixed: bool,
    pub shift: Isometry<Real>,
    pub collider_blueprint: ColliderBuilder,
    pub rigid_body_blueprint: RigidBodyBuilder,
}

impl Default for UrdfLoaderOptions {
    fn default() -> Self {
        Self {
            create_colliders_from_collision_shapes: true,
            create_colliders_from_visual_shapes: false,
            apply_imported_mass_props: true,
            enable_joint_collisions: false,
            make_roots_fixed: false,
            shift: Isometry::identity(),
            collider_blueprint: ColliderBuilder::ball(0.0).density(0.0),
            rigid_body_blueprint: RigidBodyBuilder::dynamic(),
        }
    }
}

/// An urdf link loaded as a rapier [`RigidBody`] and its [`Collider`]s.
#[derive(Clone, Debug)]
pub struct UrdfLink {
    pub body: RigidBody,
    pub colliders: Vec<Collider>,
}

/// An urdf joint loaded as a rapier [`GenericJoint`].
#[derive(Clone, Debug)]
pub struct UrdfJoint {
    /// The rapier version for the corresponding urdf joint.
    pub joint: GenericJoint,
    /// Index of the rigid-body (from the [`UrdfRobot`] array) at the first
    /// endpoint of this joint.
    pub link1: LinkId,
    /// Index of the rigid-body (from the [`UrdfRobot`] array) at the second
    /// endpoint of this joint.
    pub link2: LinkId,
}

/// A robot represented as a set of rapier rigid-bodies, colliders, and joints.
#[derive(Clone, Debug)]
pub struct UrdfRobot {
    /// The bodies and colliders loaded from the urdf file.
    ///
    /// This vector matches the order of [`Robot::links`].
    pub links: Vec<UrdfLink>,
    /// The joints loaded from the urdf file.
    ///
    /// This vector matches the order of [`Robot::joints`].
    pub joints: Vec<UrdfJoint>,
}

pub struct UrdfJointHandle<JointHandle> {
    pub joint: JointHandle,
    pub link1: RigidBodyHandle,
    pub link2: RigidBodyHandle,
}

pub struct UrdfLinkHandle {
    pub body: RigidBodyHandle,
    pub colliders: Vec<ColliderHandle>,
}

pub struct UrdfRobotHandles<JointHandle> {
    pub links: Vec<UrdfLinkHandle>,
    pub joints: Vec<UrdfJointHandle<JointHandle>>,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
enum JointType {
    #[default]
    Fixed,
    Revolute,
    Continuous,
    Floating,
    Planar,
    Prismatic,
    Spherical,
}

impl JointType {
    fn from_str(str: &str) -> Option<Self> {
        match str.as_ref() {
            "fixed" | "Fixed" => Some(Self::Fixed),
            "continuous" | "Continuous" => Some(Self::Continuous),
            "revolute" | "Revolute" => Some(Self::Revolute),
            "floating" | "Floating" => Some(Self::Floating),
            "planar" | "Planar" => Some(Self::Planar),
            "prismatic" | "Prismatic" => Some(Self::Prismatic),
            "spherical" | "Spherical" => Some(Self::Spherical),
            _ => None,
        }
    }
}

impl UrdfRobot {
    pub fn from_file(
        path: impl AsRef<Path>,
        options: UrdfLoaderOptions,
        mesh_dir: Option<&Path>,
    ) -> anyhow::Result<(Self, Robot)> {
        let path = path.as_ref();
        let mesh_dir = mesh_dir.or_else(|| path.parent());
        let robot = xurdf::parse_urdf_from_file(path)?;
        Ok((Self::from_robot(&robot, options, mesh_dir), robot))
    }

    pub fn from_str(
        str: &str,
        options: UrdfLoaderOptions,
        mesh_dir: Option<&Path>,
    ) -> anyhow::Result<(Self, Robot)> {
        let robot = xurdf::parse_urdf_from_string(str)?;
        Ok((Self::from_robot(&robot, options, mesh_dir), robot))
    }

    pub fn from_robot(robot: &Robot, options: UrdfLoaderOptions, mesh_dir: Option<&Path>) -> Self {
        let mut name_to_link_id = HashMap::new();
        let mut link_is_root = vec![true; robot.links.len()];
        let mut links: Vec<_> = robot
            .links
            .iter()
            .enumerate()
            .map(|(id, link)| {
                name_to_link_id.insert(&link.name, id);
                let mut colliders = vec![];
                if options.create_colliders_from_collision_shapes {
                    colliders.extend(link.collisions.iter().filter_map(|co| {
                        urdf_to_collider(&options, mesh_dir, &co.geometry, &co.origin)
                    }))
                }
                if options.create_colliders_from_visual_shapes {
                    colliders.extend(link.visuals.iter().filter_map(|vis| {
                        urdf_to_collider(&options, mesh_dir, &vis.geometry, &vis.origin)
                    }))
                }
                let mut body = urdf_to_rigid_body(&options, &link.inertial);
                body.set_position(options.shift * body.position(), false);
                UrdfLink { body, colliders }
            })
            .collect();
        let joints: Vec<_> = robot
            .joints
            .iter()
            .map(|joint| {
                let link1 = name_to_link_id[&joint.parent];
                let link2 = name_to_link_id[&joint.child];
                let pose1 = *links[link1].body.position();
                let rb2 = &mut links[link2].body;
                let joint = urdf_to_joint(&options, joint, &pose1, rb2);
                link_is_root[link2] = false;

                UrdfJoint {
                    joint,
                    link1,
                    link2,
                }
            })
            .collect();

        if options.make_roots_fixed {
            for (link, is_root) in links.iter_mut().zip(link_is_root.into_iter()) {
                if is_root {
                    link.body.set_body_type(RigidBodyType::Fixed, false)
                }
            }
        }

        Self { links, joints }
    }

    pub fn insert_using_impulse_joints(
        self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        joint_set: &mut ImpulseJointSet,
    ) -> UrdfRobotHandles<ImpulseJointHandle> {
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
                UrdfLinkHandle { body, colliders }
            })
            .collect();
        let joints: Vec<_> = self
            .joints
            .into_iter()
            .map(|joint| {
                let link1 = links[joint.link1].body;
                let link2 = links[joint.link2].body;
                let joint = joint_set.insert(link1, link2, joint.joint, false);
                UrdfJointHandle {
                    joint,
                    link1,
                    link2,
                }
            })
            .collect();

        UrdfRobotHandles { links, joints }
    }
    pub fn insert_using_multibody_joints(
        self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        joint_set: &mut MultibodyJointSet,
    ) -> UrdfRobotHandles<Option<MultibodyJointHandle>> {
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
                UrdfLinkHandle { body, colliders }
            })
            .collect();
        let joints: Vec<_> = self
            .joints
            .into_iter()
            .map(|joint| {
                let link1 = links[joint.link1].body;
                let link2 = links[joint.link2].body;
                let joint = joint_set.insert(link1, link2, joint.joint, false);
                UrdfJointHandle {
                    joint,
                    link1,
                    link2,
                }
            })
            .collect();

        UrdfRobotHandles { links, joints }
    }
}

fn urdf_to_rigid_body(options: &UrdfLoaderOptions, inertial: &Inertial) -> RigidBody {
    let origin = urdf_to_isometry(&inertial.origin);
    let mut builder = options.rigid_body_blueprint.clone();
    builder.body_type = RigidBodyType::Dynamic;

    if options.apply_imported_mass_props {
        builder = builder.additional_mass_properties(MassProperties::with_inertia_matrix(
            origin.translation.vector.into(),
            inertial.mass as Real,
            na::Matrix3::new(
                inertial.inertia.m11 as Real,
                inertial.inertia.m12 as Real,
                inertial.inertia.m13 as Real,
                inertial.inertia.m21 as Real,
                inertial.inertia.m22 as Real,
                inertial.inertia.m23 as Real,
                inertial.inertia.m31 as Real,
                inertial.inertia.m32 as Real,
                inertial.inertia.m33 as Real,
            ),
        ))
    }

    builder.build()
}

fn urdf_to_collider(
    options: &UrdfLoaderOptions,
    mesh_dir: Option<&Path>,
    geometry: &Geometry,
    origin: &Pose,
) -> Option<Collider> {
    let mut builder = options.collider_blueprint.clone();
    let mut shape_transform = Isometry::identity();
    let shape = match &geometry {
        Geometry::Box { size } => SharedShape::cuboid(
            size[0] as Real / 2.0,
            size[1] as Real / 2.0,
            size[2] as Real / 2.0,
        ),
        Geometry::Cylinder { radius, length } => {
            // This rotation will make the cylinder Z-up as per the URDF spec,
            // instead of rapier’s default Y-up.
            shape_transform = Isometry::rotation(Vector::x() * Real::frac_pi_2());
            SharedShape::cylinder(*length as Real / 2.0, *radius as Real)
        }
        Geometry::Sphere { radius } => SharedShape::ball(*radius as Real),
        Geometry::Mesh { filename, scale } => {
            let path: &Path = filename.as_ref();
            let scale = scale
                .map(|s| Vector::new(s.x as Real, s.y as Real, s.z as Real))
                .unwrap_or_else(|| Vector::<Real>::repeat(1.0));
            match path.extension().and_then(|ext| ext.to_str()) {
                #[cfg(feature = "stl")]
                Some("stl") | Some("STL") => {
                    let full_path = mesh_dir.map(|dir| dir.join(filename)).unwrap_or_default();
                    match rapier_stl::load_from_path(&full_path, MeshConverter::TriMesh, scale) {
                        Ok(stl_shape) => {
                            shape_transform = stl_shape.pose;
                            stl_shape.shape
                        }
                        Err(e) => {
                            log::error!("failed to load STL file {filename}: {e}");
                            return None;
                        }
                    }
                }
                _ => {
                    log::error!("failed to load file with unknown type {filename}");
                    return None;
                }
            }
        }
    };

    builder.shape = shape;
    Some(
        builder
            .position(urdf_to_isometry(origin) * shape_transform)
            .build(),
    )
}

fn urdf_to_isometry(pose: &Pose) -> Isometry<Real> {
    Isometry::from_parts(
        Point::new(
            pose.xyz[0] as Real,
            pose.xyz[1] as Real,
            pose.xyz[2] as Real,
        )
        .into(),
        na::UnitQuaternion::from_euler_angles(
            pose.rpy[0] as Real,
            pose.rpy[1] as Real,
            pose.rpy[2] as Real,
        ),
    )
}

fn urdf_to_joint(
    options: &UrdfLoaderOptions,
    joint: &Joint,
    pose1: &Isometry<Real>,
    link2: &mut RigidBody,
) -> GenericJoint {
    let joint_type = JointType::from_str(&joint.joint_type).unwrap_or_default();
    let locked_axes = match joint_type {
        JointType::Fixed => JointAxesMask::LOCKED_FIXED_AXES,
        JointType::Continuous | JointType::Revolute => JointAxesMask::LOCKED_REVOLUTE_AXES,
        JointType::Floating => JointAxesMask::empty(),
        JointType::Planar => JointAxesMask::ANG_AXES | JointAxesMask::LIN_X,
        JointType::Prismatic => JointAxesMask::LOCKED_PRISMATIC_AXES,
        JointType::Spherical => JointAxesMask::LOCKED_SPHERICAL_AXES,
    };
    let joint_to_parent = urdf_to_isometry(&joint.origin);
    let joint_axis = na::Unit::try_new(
        Vector::new(
            joint.axis.x as Real,
            joint.axis.y as Real,
            joint.axis.z as Real,
        ),
        1.0e-5,
    );

    link2.set_position(pose1 * joint_to_parent, false);

    let mut builder = GenericJointBuilder::new(locked_axes)
        .local_anchor1(joint_to_parent.translation.vector.into())
        .contacts_enabled(options.enable_joint_collisions);

    if let Some(joint_axis) = joint_axis {
        builder = builder
            .local_axis1(joint_to_parent * joint_axis)
            .local_axis2(joint_axis);
    }

    /* TODO: panics the multibody
    match joint_type {
        JointType::Prismatic => {
            builder = builder.limits(
                JointAxis::LinX,
                [joint.limit.lower as Real, joint.limit.upper as Real],
            )
        }
        JointType::Revolute => {
            builder = builder.limits(
                JointAxis::AngX,
                [joint.limit.lower as Real, joint.limit.upper as Real],
            )
        }
        _ => {}
    }
     */

    // TODO: the following fields are currently ignored:
    //       - Joint::dynamics
    //       - Joint::limit.effort / limit.velocity
    //       - Joint::mimic
    //       - Joint::safety_controller
    builder.build()
}
