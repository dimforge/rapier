//! ## URDF loader for the Rapier physics engine
//!
//! Rapier is a set of 2D and 3D physics engines for games, animation, and robotics. The `rapier3d-urdf`
//! crate lets you convert an URDF file into a set of rigid-bodies, colliders, and joints, for usage with the
//! `rapier3d` physics engine.
//!
//! ## Optional cargo features
//!
//! - `stl`: enables loading STL meshes referenced by the URDF file.
//! - `collada`: enables loading Collada (`.dae`) meshes referenced by the URDF file.
//! - `wavefront`: enables loading Wavefront (`.obj`) meshes referenced by the URDF file.
//!
//! ## Limitations
//!
//! Are listed below some known limitations you might want to be aware of before picking this library. Contributions to
//! improve
//! these elements are very welcome!
//!
//! - Mesh file types are limited. Contributions are welcome. You may check the `rapier3d-meshloader`
//!   repository for an example of mesh loader.
//! - When inserting joints as multibody joints, they will be reset to their neutral position (all coordinates = 0).
//! - The following fields are currently ignored:
//!     - `Joint::dynamics`
//!     - `Joint::limit.effort` / `limit.velocity`
//!     - `Joint::mimic`
//!     - `Joint::safety_controller`

#![warn(missing_docs)]

use na::RealField;
use rapier3d::{
    dynamics::{
        GenericJoint, GenericJointBuilder, ImpulseJointHandle, ImpulseJointSet, JointAxesMask,
        JointAxis, MassProperties, MultibodyJointHandle, MultibodyJointSet, RigidBody,
        RigidBodyBuilder, RigidBodyHandle, RigidBodySet, RigidBodyType,
    },
    geometry::{Collider, ColliderBuilder, ColliderHandle, ColliderSet, SharedShape, TriMeshFlags},
    math::{Isometry, Point, Real, Vector},
    na,
};
use std::collections::HashMap;
use std::path::Path;
use urdf_rs::{Geometry, Inertial, Joint, Pose, Robot};

#[cfg(doc)]
use rapier3d::dynamics::Multibody;

bitflags::bitflags! {
    /// Options applied to multibody joints created from the URDF joints.
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
    pub struct UrdfMultibodyOptions: u8 {
        /// If this flag is set, the created multibody joint will be marked as kinematic.
        ///
        /// A kinematic joint is entirely controlled by the user (it is not affected by any force).
        /// This particularly useful if you intend to control the robot through inverse-kinematics.
        const JOINTS_ARE_KINEMATIC = 0b0001;
        /// If enabled, any contact between two links belonging to the same generated multibody robot will
        /// be ignored.
        ///
        /// This is useful if the generated colliders are known to be overlapping (e.g. if creating colliders
        /// from visual meshes was enabled) or that collision detection is not needed a computationally
        /// expensive (e.g. if any of these colliders is a high-quality triangle mesh).
        const DISABLE_SELF_CONTACTS = 0b0010;
    }
}

/// The index of an urdf link.
pub type LinkId = usize;

/// A set of configurable options for loading URDF files.
#[derive(Clone, Debug)]
pub struct UrdfLoaderOptions {
    /// If `true` one collider will be created for each **collision** shape from the urdf file (default: `true`).
    pub create_colliders_from_collision_shapes: bool,
    /// If `true` one collider will be created for each **visual** shape from the urdf file (default: `false`).
    ///
    /// Note that visual shapes are usually significantly higher-resolution than collision shapes.
    /// Most of the time they might also overlap, or generate a lot of contacts due to them being
    /// thin triangle meshes.
    ///
    /// So if this option is set to `true`, it is recommended to also keep
    /// [`UrdfLoaderOptions::enable_joint_collisions`] set to `false`. If the model is then added
    /// to the physics sets using multibody joints, it is recommended to call
    /// [`UrdfRobot::insert_using_multibody_joints`] with the [`UrdfMultibodyOptions::DISABLE_SELF_CONTACTS`]
    /// flag enabled.
    pub create_colliders_from_visual_shapes: bool,
    /// If `true`, the mass properties (center-of-mass, mass, and angular inertia) read from the urdf
    /// file will be added to the corresponding rigid-body (default: `true`).
    ///
    /// Note that by default, all colliders created will be given a density of 0.0, meaning that,
    /// by default, the imported mass properties are the only ones added to the created rigid-bodies.
    /// To give colliders a non-zero density, see [`UrdfLoaderOptions::collider_blueprint`].
    pub apply_imported_mass_props: bool,
    /// If `true`, collisions between two links sharing a joint will be disabled (default: `false`).
    ///
    /// It is strongly recommended to leave this to `false` unless you are certain adjacent links
    /// colliders don’t overlap.
    pub enable_joint_collisions: bool,
    /// If `true`, the rigid-body at the root of the kinematic chains will be initialized as [`RigidBodyType::Fixed`]
    /// (default: `false`).
    pub make_roots_fixed: bool,
    /// This is the set of flags set on all the loaded triangle meshes (default: [`TriMeshFlags::all`]).
    ///
    /// Note that the default enables all the flags. This is operating under the assumption that the provided
    /// mesh are generally well-formed and properly oriented (2-manifolds with outward normals).
    pub trimesh_flags: TriMeshFlags,
    /// The transform appended to every created rigid-bodies (default: [`Isometry::identity`]).
    pub shift: Isometry<Real>,
    /// A description of the collider properties that need to be applied to every collider created
    /// by the loader (default: `ColliderBuilder::default().density(0.0)`).
    ///
    /// This collider builder will be used for initializing every collider created by the loader.
    /// The shape specified by this builder isn’t important and will be replaced by the shape read
    /// from the urdf file.
    ///
    /// Note that by default, the collider is given a density of 0.0 so that it doesn’t contribute
    /// to its parent rigid-body’s mass properties (since they should be already provided by the
    /// urdf file assuming the [`UrdfLoaderOptions::apply_imported_mass_props`] wasn’t set `false`).
    pub collider_blueprint: ColliderBuilder,
    /// A description of the rigid-body properties that need to be applied to every rigid-body
    /// created by the loader (default: `RigidBodyBuilder::dynamic()`).
    ///
    /// This rigid-body builder will be used for initializing every rigid-body created by the loader.
    /// The rigid-body type is not important as it will always be set to [`RigidBodyType::Dynamic`]
    /// for non-root links. Root links will be set to [`RigidBodyType::Fixed`] instead of
    /// [`RigidBodyType::Dynamic`] if the [`UrdfLoaderOptions::make_roots_fixed`] is set to `true`.
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
            trimesh_flags: TriMeshFlags::all(),
            shift: Isometry::identity(),
            collider_blueprint: ColliderBuilder::default().density(0.0),
            rigid_body_blueprint: RigidBodyBuilder::dynamic(),
        }
    }
}

/// An urdf link loaded as a rapier [`RigidBody`] and its [`Collider`]s.
#[derive(Clone, Debug)]
pub struct UrdfLink {
    /// The rigid-body created for this link.
    pub body: RigidBody,
    /// All the colliders build from the URDF visual and/or collision shapes (if the corresponding
    /// [`UrdfLoaderOptions`] option is enabled).
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

/// Handle of a joint read from the URDF file and inserted into rapier’s `ImpulseJointSet`
/// or a `MultibodyJointSet`.
pub struct UrdfJointHandle<JointHandle> {
    /// The inserted joint handle.
    pub joint: JointHandle,
    /// The handle of the first rigid-body attached by this joint.
    pub link1: RigidBodyHandle,
    /// The handle of the second rigid-body attached by this joint.
    pub link2: RigidBodyHandle,
}

/// The handles related to a link read from the URDF file and inserted into Rapier’s
/// `RigidBodySet` and `ColliderSet`.
pub struct UrdfLinkHandle {
    /// Handle of the inserted link’s rigid-body.
    pub body: RigidBodyHandle,
    /// Handle of the colliders attached to [`Self::body`].
    pub colliders: Vec<ColliderHandle>,
}

/// Handles to all the Rapier objects created when inserting this robot into Rapier’s
/// `RigidBodySet`, `ColliderSet`, `ImpulseJointSet`, `MultibodyJointSet`.
pub struct UrdfRobotHandles<JointHandle> {
    /// The handles related to each URDF robot link.
    pub links: Vec<UrdfLinkHandle>,
    /// The handles related to each URDF robot joint.
    pub joints: Vec<UrdfJointHandle<JointHandle>>,
}

impl UrdfRobot {
    /// Parses a URDF file and returns both the rapier objects (`UrdfRobot`) and the original urdf
    /// structures (`Robot`). Both structures are arranged the same way, with matching indices for each part.
    ///
    /// If the URDF file references external meshes, they will be loaded automatically if the format
    /// is supported. The format is detected from the file’s extension. All the mesh formats are
    /// disabled by default and can be enabled through cargo features (e.g. the `stl` feature of
    /// this crate enabled loading referenced meshes in stl format).
    ///
    /// # Parameters
    /// - `path`: the path of the URDF file.
    /// - `options`: customize the creation of rapier objects from the URDF description.
    /// - `mesh_dir`: the base directory containing the meshes referenced by the URDF file. When
    ///               a mesh reference is found in the URDF file, this `mesh_dir` is appended
    ///               to the file path. If `mesh_dir` is `None` then the mesh directory is assumed
    ///               to be the same directory as the one containing the URDF file.
    pub fn from_file(
        path: impl AsRef<Path>,
        options: UrdfLoaderOptions,
        mesh_dir: Option<&Path>,
    ) -> anyhow::Result<(Self, Robot)> {
        let path = path.as_ref().canonicalize()?;
        let mesh_dir = mesh_dir
            .or_else(|| path.parent())
            .unwrap_or_else(|| Path::new("./"));
        let robot = urdf_rs::read_file(&path)?;
        Ok((Self::from_robot(&robot, options, mesh_dir), robot))
    }

    /// Parses a string in URDF format and returns both the rapier objects (`UrdfRobot`) and the original urdf
    /// structures (`Robot`). Both structures are arranged the same way, with matching indices for each part.
    ///
    /// If the URDF file references external meshes, they will be loaded automatically if the format
    /// is supported. The format is detected from the file’s extension. All the mesh formats are
    /// disabled by default and can be enabled through cargo features (e.g. the `stl` feature of
    /// this crate enabled loading referenced meshes in stl format).
    ///
    /// # Parameters
    /// - `str`: the string content of an URDF file.
    /// - `options`: customize the creation of rapier objects from the URDF description.
    /// - `mesh_dir`: the base directory containing the meshes referenced by the URDF file. When
    ///               a mesh reference is found in the URDF file, this `mesh_dir` is appended
    ///               to the file path.
    pub fn from_str(
        str: &str,
        options: UrdfLoaderOptions,
        mesh_dir: &Path,
    ) -> anyhow::Result<(Self, Robot)> {
        let robot = urdf_rs::read_from_string(str)?;
        Ok((Self::from_robot(&robot, options, mesh_dir), robot))
    }

    /// From an already loaded urdf file as a `Robot`, this creates the matching rapier objects
    /// (`UrdfRobot`). Both structures are arranged the same way, with matching indices for each part.
    ///
    /// If the URDF file references external meshes, they will be loaded automatically if the format
    /// is supported. The format is detected mostly from the file’s extension. All the mesh formats are
    /// disabled by default and can be enabled through cargo features (e.g. the `stl` feature of
    /// this crate enabled loading referenced meshes in stl format).
    ///
    /// # Parameters
    /// - `robot`: the robot loaded from an URDF file.
    /// - `options`: customize the creation of rapier objects from the URDF description.
    /// - `mesh_dir`: the base directory containing the meshes referenced by the URDF file. When
    ///               a mesh reference is found in the URDF file, this `mesh_dir` is appended
    ///               to the file path.
    pub fn from_robot(robot: &Robot, options: UrdfLoaderOptions, mesh_dir: &Path) -> Self {
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
                    colliders.extend(link.collision.iter().flat_map(|co| {
                        urdf_to_colliders(&options, mesh_dir, &co.geometry, &co.origin)
                    }))
                }
                if options.create_colliders_from_visual_shapes {
                    colliders.extend(link.visual.iter().flat_map(|vis| {
                        urdf_to_colliders(&options, mesh_dir, &vis.geometry, &vis.origin)
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
                let link1 = name_to_link_id[&joint.parent.link];
                let link2 = name_to_link_id[&joint.child.link];
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

    /// Inserts all the robots elements to the rapier rigid-body, collider, and impulse joint, sets.
    ///
    /// Joints are represented as impulse joints. This implies that joint constraints are simulated
    /// in full coordinates using impulses. For a reduced-coordinates approach, see
    /// [`UrdfRobot::insert_using_multibody_joints`].
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

    /// Inserts all the robots elements to the rapier rigid-body, collider, and multibody joint, sets.
    ///
    /// Joints are represented as multibody joints. This implies that the robot as a whole can be
    /// accessed as a single [`Multibody`] from the [`MultibodyJointSet`]. That multibody uses reduced
    /// coordinates for modeling joints, meaning that it will be very close to the way they are usually
    /// represented for robotics applications. Multibodies also support inverse kinematics.
    pub fn insert_using_multibody_joints(
        self,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        joint_set: &mut MultibodyJointSet,
        multibody_options: UrdfMultibodyOptions,
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
                let joint =
                    if multibody_options.contains(UrdfMultibodyOptions::JOINTS_ARE_KINEMATIC) {
                        joint_set.insert_kinematic(link1, link2, joint.joint, false)
                    } else {
                        joint_set.insert(link1, link2, joint.joint, false)
                    };

                if let Some(joint) = joint {
                    let (multibody, _) = joint_set.get_mut(joint).unwrap_or_else(|| unreachable!());
                    multibody.set_self_contacts_enabled(
                        !multibody_options.contains(UrdfMultibodyOptions::DISABLE_SELF_CONTACTS),
                    );
                }

                UrdfJointHandle {
                    joint,
                    link1,
                    link2,
                }
            })
            .collect();

        UrdfRobotHandles { links, joints }
    }

    /// Appends a transform to all the rigid-bodie of this robot.
    pub fn append_transform(&mut self, transform: &Isometry<Real>) {
        for link in &mut self.links {
            link.body
                .set_position(transform * link.body.position(), true);
        }
    }
}

#[rustfmt::skip]
fn urdf_to_rigid_body(options: &UrdfLoaderOptions, inertial: &Inertial) -> RigidBody {
    let origin = urdf_to_isometry(&inertial.origin);
    let mut builder = options.rigid_body_blueprint.clone();
    builder.body_type = RigidBodyType::Dynamic;

    if options.apply_imported_mass_props {
        builder = builder.additional_mass_properties(MassProperties::with_inertia_matrix(
            origin.translation.vector.into(),
            inertial.mass.value as Real,
            // See http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model#Inertia
            na::Matrix3::new(
                inertial.inertia.ixx as Real, inertial.inertia.ixy as Real, inertial.inertia.ixz as Real,
                inertial.inertia.ixy as Real, inertial.inertia.iyy as Real, inertial.inertia.iyz as Real,
                inertial.inertia.ixz as Real, inertial.inertia.iyz as Real,inertial.inertia.izz as Real,
            ),
        ))
    }

    builder.build()
}

fn urdf_to_colliders(
    options: &UrdfLoaderOptions,
    _mesh_dir: &Path, // Unused if none of the meshloader features is enabled.
    geometry: &Geometry,
    origin: &Pose,
) -> Vec<Collider> {
    let mut shape_transform = Isometry::identity();

    let mut colliders = Vec::new();

    match &geometry {
        Geometry::Box { size } => {
            colliders.push(SharedShape::cuboid(
                size[0] as Real / 2.0,
                size[1] as Real / 2.0,
                size[2] as Real / 2.0,
            ));
        }
        Geometry::Cylinder { radius, length } => {
            // This rotation will make the cylinder Z-up as per the URDF spec,
            // instead of rapier’s default Y-up.
            shape_transform = Isometry::rotation(Vector::x() * Real::frac_pi_2());
            colliders.push(SharedShape::cylinder(
                *length as Real / 2.0,
                *radius as Real,
            ));
        }
        Geometry::Capsule { radius, length } => {
            colliders.push(SharedShape::capsule_z(
                *length as Real / 2.0,
                *radius as Real,
            ));
        }
        Geometry::Sphere { radius } => {
            colliders.push(SharedShape::ball(*radius as Real));
        }
        #[cfg(not(feature = "__meshloader_is_enabled"))]
        Geometry::Mesh { .. } => {
            log::error!("Mesh loading is disabled by default. Enable one of the format features (`stl`, `collada`, `wavefront`) of `rapier3d-urdf` for mesh support.");
        }
        #[cfg(feature = "__meshloader_is_enabled")]
        Geometry::Mesh { filename, scale } => {
            let full_path = _mesh_dir.join(filename);
            let scale = scale
                .map(|s| Vector::new(s[0] as Real, s[1] as Real, s[2] as Real))
                .unwrap_or_else(|| Vector::<Real>::repeat(1.0));

            let Ok(loaded_mesh) = rapier3d_meshloader::load_from_path(
                full_path,
                &rapier3d::prelude::MeshConverter::TriMeshWithFlags(options.trimesh_flags),
                scale,
            ) else {
                return Vec::new();
            };
            colliders.append(
                &mut loaded_mesh
                    .into_iter()
                    .filter_map(|x| x.map(|s| s.shape).ok())
                    .collect(),
            );
        }
    }

    colliders
        .drain(..)
        .map(move |shape| {
            let mut builder = options.collider_blueprint.clone();
            builder.shape = shape;
            builder
                .position(urdf_to_isometry(origin) * shape_transform)
                .build()
        })
        .collect()
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
    let locked_axes = match joint.joint_type {
        urdf_rs::JointType::Fixed => JointAxesMask::LOCKED_FIXED_AXES,
        urdf_rs::JointType::Continuous | urdf_rs::JointType::Revolute => {
            JointAxesMask::LOCKED_REVOLUTE_AXES
        }
        urdf_rs::JointType::Floating => JointAxesMask::empty(),
        urdf_rs::JointType::Planar => JointAxesMask::ANG_AXES | JointAxesMask::LIN_X,
        urdf_rs::JointType::Prismatic => JointAxesMask::LOCKED_PRISMATIC_AXES,
        urdf_rs::JointType::Spherical => JointAxesMask::LOCKED_SPHERICAL_AXES,
    };
    let joint_to_parent = urdf_to_isometry(&joint.origin);
    let joint_axis = na::Unit::try_new(
        Vector::new(
            joint.axis.xyz[0] as Real,
            joint.axis.xyz[1] as Real,
            joint.axis.xyz[2] as Real,
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

    match joint.joint_type {
        urdf_rs::JointType::Prismatic => {
            builder = builder.limits(
                JointAxis::LinX,
                [joint.limit.lower as Real, joint.limit.upper as Real],
            )
        }
        urdf_rs::JointType::Revolute => {
            builder = builder.limits(
                JointAxis::AngX,
                [joint.limit.lower as Real, joint.limit.upper as Real],
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
