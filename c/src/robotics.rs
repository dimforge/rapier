//! Optional native URDF and MJCF importers (3D f32).
#![allow(non_snake_case)]
use crate::*;
use rapier3d_mjcf::MjcfVisualMesh;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot, MjcfRobotHandles};
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot, UrdfRobotHandles};
use std::ffi::{CStr, c_char};

unsafe fn path_string<'a>(path: *const c_char) -> Result<&'a str> {
    ensure(!path.is_null(), "null path")?;
    unsafe { CStr::from_ptr(path) }
        .to_str()
        .map_err(|_| invalid("path must be UTF-8"))
}
/// Loader configuration. Initialize with DefaultUrdfLoaderOptions; no destructor.
/// Blueprint array views and shared shapes are borrowed through the load call.
/// @ingroup robotics
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RprUrdfLoaderOptions {
    /// Whether to build colliders from collision geometry.
    pub createCollidersFromCollisionShapes: RprBool,
    /// Whether to also build colliders from visual geometry.
    pub createCollidersFromVisualShapes: RprBool,
    /// Whether to use imported mass/inertia properties.
    pub applyImportedMassProps: RprBool,
    /// Whether bodies connected by imported joints may collide.
    pub enableJointCollisions: RprBool,
    /// Whether imported root bodies are fixed.
    pub makeRootsFixed: RprBool,
    /// Whether to merge empty fixed URDF links.
    pub squeezeEmptyFixedLinks: RprBool,
    /// Transform applied to the imported model.
    pub shift: RprPose,
    /// Shape scale along each axis.
    pub scale: RprReal,
    /// Default collider description; nested geometry resources are borrowed through loading.
    pub colliderBlueprint: RprColliderDesc,
    /// Default rigid-body description used by the importer.
    pub rigidBodyBlueprint: RprRigidBodyDesc,
}
impl Default for RprUrdfLoaderOptions {
    fn default() -> Self {
        let defaults = UrdfLoaderOptions::default();
        Self {
            createCollidersFromCollisionShapes: defaults.create_colliders_from_collision_shapes
                as RprBool,
            createCollidersFromVisualShapes: defaults.create_colliders_from_visual_shapes
                as RprBool,
            applyImportedMassProps: defaults.apply_imported_mass_props as RprBool,
            enableJointCollisions: defaults.enable_joint_collisions as RprBool,
            makeRootsFixed: defaults.make_roots_fixed as RprBool,
            squeezeEmptyFixedLinks: defaults.squeeze_empty_fixed_links as RprBool,
            shift: defaults.shift.into(),
            scale: defaults.scale,
            colliderBlueprint: RprColliderDesc {
                density: 0.0,
                ..Default::default()
            },
            rigidBodyBlueprint: rpr_dynamic_rigid_body_desc(),
        }
    }
}
impl RprUrdfLoaderOptions {
    unsafe fn raw(&self) -> Result<UrdfLoaderOptions> {
        Ok(UrdfLoaderOptions {
            create_colliders_from_collision_shapes: boolean(
                self.createCollidersFromCollisionShapes,
            )?,
            create_colliders_from_visual_shapes: boolean(self.createCollidersFromVisualShapes)?,
            apply_imported_mass_props: boolean(self.applyImportedMassProps)?,
            enable_joint_collisions: boolean(self.enableJointCollisions)?,
            make_roots_fixed: boolean(self.makeRootsFixed)?,
            squeeze_empty_fixed_links: boolean(self.squeezeEmptyFixedLinks)?,
            shift: self.shift.raw()?,
            scale: positive(self.scale)?,
            collider_blueprint: unsafe { self.colliderBlueprint.raw()? },
            rigid_body_blueprint: self.rigidBodyBlueprint.raw()?,
            ..Default::default()
        })
    }
}
/// Return native default urdf loader options. This POD value owns no resources.
/// @ingroup robotics
#[rapier_export]
pub extern "C" fn rpr_default_urdf_loader_options() -> RprUrdfLoaderOptions {
    RprUrdfLoaderOptions::default()
}
/// Loaded URDF robot; insertion clones its simulation objects. Release with the matching Free
/// function.
/// @ingroup robotics
pub struct RprUrdfRobot(pub(crate) UrdfRobot);
/// Release an owned urdf robot. NULL is allowed. Do not pass borrowed pointers or free the object
/// twice.
/// @ingroup robotics
#[rapier_export]
pub unsafe extern "C" fn rpr_free_urdf_robot(object: *mut RprUrdfRobot) -> RprStatus {
    ffi(|| unsafe {
        if !object.is_null() {
            get(object)?;
            drop(Box::from_raw(object));
        }
        Ok(())
    })
}
/// Load from a UTF-8 path. Validates options before reading the file.
/// Options and their blueprint resources are borrowed through this call; the robot is owned.
/// @ingroup robotics
#[rapier_export]
pub unsafe extern "C" fn rpr_urdf_robot_from_file(
    path: *const c_char,
    options: *const RprUrdfLoaderOptions,
) -> *mut RprUrdfRobot {
    ffi_value(|out: *mut *mut RprUrdfRobot| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let path = path_string(path)?;
            let (robot, _) = UrdfRobot::from_file(path, get(options)?.raw()?, None)
                .map_err(|e| invalid(e.to_string()))?;
            output(out, Box::into_raw(Box::new(RprUrdfRobot(robot))))
        })
    })
}
/// Apply an additional transform to the loaded robot before insertion.
/// @ingroup robotics
#[rapier_export(urdf_robot)]
pub unsafe extern "C" fn rpr_urdf_robot_append_transform(
    robot: *mut RprUrdfRobot,
    transform: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        get_mut(robot)?.0.append_transform(&transform.raw()?);
        Ok(())
    })
}
/// Owned container of borrowed handles to an inserted URDF robot. Release with the matching Free
/// function.
/// @ingroup robotics
pub struct RprUrdfRobotHandles {
    world: *mut RprWorld,
    handles: UrdfHandles,
}
enum UrdfHandles {
    Impulse(UrdfRobotHandles<ImpulseJointHandle>),
    Multibody(UrdfRobotHandles<Option<MultibodyJointHandle>>),
}
/// Release an owned urdf robot handles. NULL is allowed. Do not pass borrowed pointers or free the
/// object twice.
/// @ingroup robotics
#[rapier_export]
pub unsafe extern "C" fn rpr_free_urdf_robot_handles(
    handles: *mut RprUrdfRobotHandles,
) -> RprStatus {
    ffi(|| unsafe {
        if !handles.is_null() {
            get(handles)?;
            drop(Box::from_raw(handles));
        }
        Ok(())
    })
}
/// Inserts a clone; the source robot remains owned by the caller. Returns owned handles.
/// @ingroup robotics
#[rapier_export(urdf_robot)]
pub unsafe extern "C" fn rpr_urdf_robot_insert_using_impulse_joints(
    world: *mut RprWorld,
    robot: *const RprUrdfRobot,
) -> *mut RprUrdfRobotHandles {
    ffi_value(|out: *mut *mut RprUrdfRobotHandles| {
        ffi(|| unsafe {
            let owner = world;
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            out_ptr(out)?;

            let world = &mut get_mut(world)?.0;
            let handles = get(robot)?.0.clone().insert_using_impulse_joints(
                &mut world.bodies,
                &mut world.colliders,
                &mut world.impulse_joints,
            );
            output(
                out,
                Box::into_raw(Box::new(RprUrdfRobotHandles {
                    world: owner,
                    handles: UrdfHandles::Impulse(handles),
                })),
            )
        })
    })
}

/// Inserts a clone; the source robot remains owned by the caller. Returns owned handles.
/// @ingroup robotics
#[rapier_export(urdf_robot)]
pub unsafe extern "C" fn rpr_urdf_robot_insert_using_multibody_joints(
    world: *mut RprWorld,
    robot: *const RprUrdfRobot,
    options: u8,
) -> *mut RprUrdfRobotHandles {
    ffi_value(|out: *mut *mut RprUrdfRobotHandles| {
        ffi(|| unsafe {
            let owner = world;
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            out_ptr(out)?;
            let options = UrdfMultibodyOptions::from_bits(options)
                .ok_or_else(|| invalid("unknown multibody options"))?;
            let world = &mut get_mut(world)?.0;
            let handles = get(robot)?.0.clone().insert_using_multibody_joints(
                &mut world.bodies,
                &mut world.colliders,
                &mut world.multibody_joints,
                options,
            );
            output(
                out,
                Box::into_raw(Box::new(RprUrdfRobotHandles {
                    world: owner,
                    handles: UrdfHandles::Multibody(handles),
                })),
            )
        })
    })
}

/// Body handles in source order; absent MJCF bodies have invalid handles.
/// @see @ref output_buffers
/// @ingroup robotics
#[rapier_export(urdf_robot_handles)]
pub unsafe extern "C" fn rpr_urdf_robot_handles_bodies(
    handles: *const RprUrdfRobotHandles,
    buffer: *mut RprRigidBodyHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(
            get(handles).map_or(std::ptr::null_mut(), |h| h.world),
            buffer,
            capacity,
            |count: *mut usize| {
                ffi(|| {
                    let values: Vec<RprRigidBodyHandle> = match &get(handles)?.handles {
                        UrdfHandles::Impulse(h) => h.links.iter().map(|b| b.body.into()).collect(),
                        UrdfHandles::Multibody(h) => {
                            h.links.iter().map(|b| b.body.into()).collect()
                        }
                    };
                    copy_out(&values, buffer, capacity, count)
                })
            },
        )
    }
}
/// Loader configuration. Initialize with DefaultMjcfLoaderOptions; no destructor.
/// Blueprint array views and shared shapes are borrowed through the load call.
/// @ingroup robotics
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RprMjcfLoaderOptions {
    /// Whether to build colliders from collision geometry.
    pub createCollidersFromCollisionShapes: RprBool,
    /// Whether to also build colliders from visual geometry.
    pub createCollidersFromVisualShapes: RprBool,
    /// Whether to use imported mass/inertia properties.
    pub applyImportedMassProps: RprBool,
    /// Whether bodies connected by imported joints may collide.
    pub enableJointCollisions: RprBool,
    /// Whether imported root bodies are fixed.
    pub makeRootsFixed: RprBool,
    /// Whether to omit MJCF plane geometry.
    pub skipPlaneGeoms: RprBool,
    /// Whether imported joint motors are disabled.
    pub disableJointMotors: RprBool,
    /// Transform applied to the imported model.
    pub shift: RprPose,
    /// Shape scale along each axis.
    pub scale: RprReal,
    /// Default collider description; nested geometry resources are borrowed through loading.
    pub colliderBlueprint: RprColliderDesc,
    /// Default rigid-body description used by the importer.
    pub rigidBodyBlueprint: RprRigidBodyDesc,
}
impl Default for RprMjcfLoaderOptions {
    fn default() -> Self {
        let defaults = MjcfLoaderOptions::default();
        Self {
            createCollidersFromCollisionShapes: defaults.create_colliders_from_collision_shapes
                as RprBool,
            createCollidersFromVisualShapes: defaults.create_colliders_from_visual_shapes
                as RprBool,
            applyImportedMassProps: defaults.apply_imported_mass_props as RprBool,
            enableJointCollisions: defaults.enable_joint_collisions as RprBool,
            makeRootsFixed: defaults.make_roots_fixed as RprBool,
            skipPlaneGeoms: defaults.skip_plane_geoms as RprBool,
            disableJointMotors: defaults.disable_joint_motors as RprBool,
            shift: defaults.shift.into(),
            scale: defaults.scale,
            colliderBlueprint: RprColliderDesc {
                density: 0.0,
                ..Default::default()
            },
            rigidBodyBlueprint: rpr_dynamic_rigid_body_desc(),
        }
    }
}
impl RprMjcfLoaderOptions {
    unsafe fn raw(&self) -> Result<MjcfLoaderOptions> {
        Ok(MjcfLoaderOptions {
            create_colliders_from_collision_shapes: boolean(
                self.createCollidersFromCollisionShapes,
            )?,
            create_colliders_from_visual_shapes: boolean(self.createCollidersFromVisualShapes)?,
            apply_imported_mass_props: boolean(self.applyImportedMassProps)?,
            enable_joint_collisions: boolean(self.enableJointCollisions)?,
            make_roots_fixed: boolean(self.makeRootsFixed)?,
            skip_plane_geoms: boolean(self.skipPlaneGeoms)?,
            disable_joint_motors: boolean(self.disableJointMotors)?,
            shift: self.shift.raw()?,
            scale: positive(self.scale)?,
            collider_blueprint: unsafe { self.colliderBlueprint.raw()? },
            rigid_body_blueprint: self.rigidBodyBlueprint.raw()?,
            ..Default::default()
        })
    }
}
/// Return native default mjcf loader options. This POD value owns no resources.
/// @ingroup robotics
#[rapier_export]
pub extern "C" fn rpr_default_mjcf_loader_options() -> RprMjcfLoaderOptions {
    RprMjcfLoaderOptions::default()
}
/// Loaded MJCF robot and its visual/keyframe data. Release with the matching Free function.
/// @ingroup robotics
pub struct RprMjcfRobot(pub(crate) MjcfRobot);
/// Release an owned mjcf robot. NULL is allowed. Do not pass borrowed pointers or free the object
/// twice.
/// @ingroup robotics
#[rapier_export]
pub unsafe extern "C" fn rpr_free_mjcf_robot(object: *mut RprMjcfRobot) -> RprStatus {
    ffi(|| unsafe {
        if !object.is_null() {
            get(object)?;
            drop(Box::from_raw(object));
        }
        Ok(())
    })
}
/// Load from a UTF-8 path. Validates options before reading the file.
/// Options and their blueprint resources are borrowed through this call; the robot is owned.
/// @ingroup robotics
#[rapier_export]
pub unsafe extern "C" fn rpr_mjcf_robot_from_file(
    path: *const c_char,
    options: *const RprMjcfLoaderOptions,
) -> *mut RprMjcfRobot {
    ffi_value(|out: *mut *mut RprMjcfRobot| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let path = path_string(path)?;
            let (robot, _) = MjcfRobot::from_file(path, get(options)?.raw()?)
                .map_err(|e| invalid(e.to_string()))?;
            output(out, Box::into_raw(Box::new(RprMjcfRobot(robot))))
        })
    })
}
/// Apply an additional transform to the loaded robot before insertion.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_append_transform(
    robot: *mut RprMjcfRobot,
    transform: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        get_mut(robot)?.0.append_transform(&transform.raw()?);
        Ok(())
    })
}
/// Owned container of borrowed handles and actuators of an inserted MJCF robot. Release with the
/// matching Free function.
/// @ingroup robotics
pub struct RprMjcfRobotHandles {
    world: *mut RprWorld,
    handles: MjcfHandles,
}
enum MjcfHandles {
    Impulse(MjcfRobotHandles<ImpulseJointHandle>),
    Multibody(MjcfRobotHandles<Option<MultibodyJointHandle>>),
}
/// Release an owned mjcf robot handles. NULL is allowed. Do not pass borrowed pointers or free the
/// object twice.
/// @ingroup robotics
#[rapier_export]
pub unsafe extern "C" fn rpr_free_mjcf_robot_handles(
    handles: *mut RprMjcfRobotHandles,
) -> RprStatus {
    ffi(|| unsafe {
        if !handles.is_null() {
            get(handles)?;
            drop(Box::from_raw(handles));
        }
        Ok(())
    })
}
/// Inserts a clone; the source robot remains owned by the caller. Returns owned handles.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_insert_using_impulse_joints(
    world: *mut RprWorld,
    robot: *const RprMjcfRobot,
) -> *mut RprMjcfRobotHandles {
    ffi_value(|out: *mut *mut RprMjcfRobotHandles| {
        ffi(|| unsafe {
            let owner = world;
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            out_ptr(out)?;

            let world = &mut get_mut(world)?.0;
            let handles = get(robot)?.0.clone().insert_using_impulse_joints(
                &mut world.bodies,
                &mut world.colliders,
                &mut world.impulse_joints,
            );
            output(
                out,
                Box::into_raw(Box::new(RprMjcfRobotHandles {
                    world: owner,
                    handles: MjcfHandles::Impulse(handles),
                })),
            )
        })
    })
}

/// Inserts a clone; the source robot remains owned by the caller. Returns owned handles.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_insert_using_multibody_joints(
    world: *mut RprWorld,
    robot: *const RprMjcfRobot,
    options: u8,
) -> *mut RprMjcfRobotHandles {
    ffi_value(|out: *mut *mut RprMjcfRobotHandles| {
        ffi(|| unsafe {
            let access = get(world)?.write()?;
            let raw = access.raw();

            crate::handle_access::forward(native_mjcf_robot_insert_using_multibody_joints(
                robot, raw, options, out, world,
            ))
        })
    })
}

pub(crate) unsafe fn native_mjcf_robot_insert_using_multibody_joints(
    robot: *const RprMjcfRobot,
    world: *mut RprPhysicsWorld,
    options: u8,
    out: *mut *mut RprMjcfRobotHandles,
    owner: *mut RprWorld,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let options = MjcfMultibodyOptions::from_bits(options)
            .ok_or_else(|| invalid("unknown multibody options"))?;
        let world = &mut get_mut(world)?.0;
        let handles = get(robot)?.0.clone().insert_using_multibody_joints(
            &mut world.bodies,
            &mut world.colliders,
            &mut world.multibody_joints,
            &mut world.impulse_joints,
            options,
        );
        output(
            out,
            Box::into_raw(Box::new(RprMjcfRobotHandles {
                world: owner,
                handles: MjcfHandles::Multibody(handles),
            })),
        )
    })
}
/// Body handles in source order; absent MJCF bodies have invalid handles.
/// @see @ref output_buffers
/// @ingroup robotics
#[rapier_export(mjcf_robot_handles)]
pub unsafe extern "C" fn rpr_mjcf_robot_handles_bodies(
    handles: *const RprMjcfRobotHandles,
    buffer: *mut RprRigidBodyHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(
            get(handles).map_or(std::ptr::null_mut(), |h| h.world),
            buffer,
            capacity,
            |count: *mut usize| {
                ffi(|| {
                    let values: Vec<RprRigidBodyHandle> = match &get(handles)?.handles {
                        MjcfHandles::Impulse(h) => h
                            .bodies
                            .iter()
                            .map(|b| b.as_ref().map(|b| b.body.into()).unwrap_or_default())
                            .collect(),
                        MjcfHandles::Multibody(h) => h
                            .bodies
                            .iter()
                            .map(|b| b.as_ref().map(|b| b.body.into()).unwrap_or_default())
                            .collect(),
                    };
                    copy_out(&values, buffer, capacity, count)
                })
            },
        )
    }
}
/// Resolved model gravity before the caller chooses a world convention.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_gravity(robot: *const RprMjcfRobot) -> RprVector {
    ffi_value(|out: *mut RprVector| ffi(|| unsafe { output(out, get(robot)?.0.gravity.into()) }))
}
/// Return the number of source MJCF bodies.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_body_count(robot: *const RprMjcfRobot) -> usize {
    ffi_value(|out: *mut usize| ffi(|| unsafe { output(out, get(robot)?.0.bodies.len()) }))
}
/// Return the collider count for a source body index.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_body_collider_count(
    robot: *const RprMjcfRobot,
    body: usize,
) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            output(
                out,
                get(robot)?
                    .0
                    .bodies
                    .get(body)
                    .ok_or_else(|| invalid("body index out of range"))?
                    .colliders
                    .len(),
            )
        })
    })
}
/// Set collision groups on a collider in the loaded robot, before insertion.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_set_body_collider_collision_groups(
    robot: *mut RprMjcfRobot,
    body: usize,
    collider: usize,
    groups: RprInteractionGroups,
) -> RprStatus {
    ffi(|| unsafe {
        let groups = groups.raw()?;
        let collider = get_mut(robot)?
            .0
            .bodies
            .get_mut(body)
            .and_then(|b| b.colliders.get_mut(collider))
            .ok_or_else(|| invalid("collider index out of range"))?;
        collider.set_collision_groups(groups);
        Ok(())
    })
}
/// Return the number of imported keyframes.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_keyframe_count(robot: *const RprMjcfRobot) -> usize {
    ffi_value(|out: *mut usize| ffi(|| unsafe { output(out, get(robot)?.0.keyframes.len()) }))
}
/// Copies a NUL-terminated UTF-8 name. Count includes NUL; unnamed keys return an empty string.
/// @see @ref output_buffers
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_keyframe_name(
    robot: *const RprMjcfRobot,
    key: usize,
    buffer: *mut c_char,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let key = get(robot)?
                .0
                .keyframes
                .get(key)
                .ok_or_else(|| invalid("keyframe index out of range"))?;
            let mut bytes = key.name.as_deref().unwrap_or("").as_bytes().to_vec();
            bytes.push(0);
            copy_out(&bytes, buffer.cast(), capacity, count)
        })
    })
}
/// Append a keyframe from the source MJCF model to the loaded robot.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_append_keyframe(
    robot: *mut RprMjcfRobot,
    source: *const RprMjcfRobot,
    key: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let key = get(source)?
            .0
            .keyframes
            .get(key)
            .ok_or_else(|| invalid("keyframe index out of range"))?
            .clone();
        get_mut(robot)?.0.keyframes.push(key);
        Ok(())
    })
}
/// Copy actuator controls for the selected keyframe.
/// @see @ref output_buffers
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_keyframe_controls(
    robot: *const RprMjcfRobot,
    key: usize,
    buffer: *mut RprReal,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let robot = &get(robot)?.0;
            let key = robot
                .keyframes
                .get(key)
                .ok_or_else(|| invalid("keyframe index out of range"))?;
            copy_out(&robot.keyframe_controls(key), buffer, capacity, count)
        })
    })
}
/// Return the number of imported actuators.
/// @ingroup robotics
#[rapier_export(mjcf_robot_handles)]
pub unsafe extern "C" fn rpr_mjcf_robot_handles_actuator_count(
    handles: *const RprMjcfRobotHandles,
) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            output(
                out,
                match &get(handles)?.handles {
                    MjcfHandles::Impulse(h) => h.actuators.len(),
                    MjcfHandles::Multibody(h) => h.actuators.len(),
                },
            )
        })
    })
}
/// Apply the selected keyframe to the inserted robot.
/// @ingroup robotics
#[rapier_export(mjcf_robot_handles)]
pub unsafe extern "C" fn rpr_mjcf_robot_handles_apply_keyframe(
    handles: *const RprMjcfRobotHandles,
    robot: *const RprMjcfRobot,
    key: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let world = get(handles)?.world;
        let access = get(world)?.write()?;
        let raw = access.raw();

        crate::handle_access::forward(native_mjcf_robot_handles_apply_keyframe(
            handles, raw, robot, key,
        ))
    })
}

pub(crate) unsafe fn native_mjcf_robot_handles_apply_keyframe(
    handles: *const RprMjcfRobotHandles,
    world: *mut RprPhysicsWorld,
    robot: *const RprMjcfRobot,
    key: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let robot = &get(robot)?.0;
        let key = robot
            .keyframes
            .get(key)
            .ok_or_else(|| invalid("keyframe index out of range"))?;
        let world = &mut get_mut(world)?.0;
        match &get(handles)?.handles {
            MjcfHandles::Impulse(h) => h.apply_keyframe(&mut world.bodies, robot, key),
            MjcfHandles::Multibody(h) => {
                h.apply_keyframe(&mut world.bodies, &mut world.multibody_joints, robot, key)
            }
        };
        Ok(())
    })
}
/// Apply actuator controls with per-actuator scaling to the inserted robot.
/// @ingroup robotics
#[rapier_export(mjcf_robot_handles)]
pub unsafe extern "C" fn rpr_mjcf_robot_handles_apply_controls_scaled(
    handles: *const RprMjcfRobotHandles,
    controls: *const RprReal,
    count: usize,
    gain: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let world = get(handles)?.world;
        let access = get(world)?.write()?;
        let raw = access.raw();

        crate::handle_access::forward(native_mjcf_robot_handles_apply_controls_scaled(
            handles, raw, controls, count, gain,
        ))
    })
}

pub(crate) unsafe fn native_mjcf_robot_handles_apply_controls_scaled(
    handles: *const RprMjcfRobotHandles,
    world: *mut RprPhysicsWorld,
    controls: *const RprReal,
    count: usize,
    gain: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let controls = input(controls, count)?;
        for &value in controls {
            finite(value)?;
        }
        let gain = nonnegative(gain)?;
        let world = &mut get_mut(world)?.0;
        match &get(handles)?.handles {
            MjcfHandles::Impulse(h) => {
                h.apply_controls_scaled(&mut world.impulse_joints, controls, gain)
            }
            MjcfHandles::Multibody(h) => h.apply_controls_multibody_scaled(
                &mut world.bodies,
                &mut world.multibody_joints,
                controls,
                gain,
            ),
        };
        Ok(())
    })
}
/// A borrowed visual declaration, valid until its robot is freed or its body storage changes.
/// @ingroup robotics
#[repr(transparent)]
pub struct RprMjcfVisualMesh(pub(crate) MjcfVisualMesh);
/// Imported physically based visual material; no texture ownership.
/// @ingroup robotics
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprRenderMaterial {
    /// Material metallic factor.
    pub metallic: f32,
    /// Material roughness factor.
    pub roughness: f32,
    /// Material reflectance factor.
    pub reflectance: f32,
    /// RGB emissive color.
    pub emissive: [f32; 3],
}
/// Copied metadata for a borrowed MJCF visual mesh.
/// @ingroup robotics
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprMjcfVisualMeshInfo {
    /// Visual pose relative to its source body.
    pub local_pose: RprPose,
    /// RGBA visual color.
    pub rgba: [f32; 4],
    /// Copied render material; meaningful when has_material is 1.
    pub material: RprRenderMaterial,
    /// Whether rgba contains an authored color.
    pub has_color: RprBool,
    /// Whether material contains authored material data.
    pub has_material: RprBool,
    /// Whether the visual geometry is a triangle mesh.
    pub is_trimesh: RprBool,
}
/// Return the number of visual meshes for a source body.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_body_visual_count(
    robot: *const RprMjcfRobot,
    body: usize,
) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            output(
                out,
                get(robot)?
                    .0
                    .bodies
                    .get(body)
                    .ok_or_else(|| invalid("body index out of range"))?
                    .visual_meshes
                    .len(),
            )
        })
    })
}
/// Borrow a visual mesh by body/visual index. Valid until the robot is freed or its storage
/// changes; never free this pointer.
/// @ingroup robotics
#[rapier_export(mjcf_robot)]
pub unsafe extern "C" fn rpr_mjcf_robot_body_visual(
    robot: *const RprMjcfRobot,
    body: usize,
    visual: usize,
) -> *const RprMjcfVisualMesh {
    ffi_value(|out: *mut *const RprMjcfVisualMesh| {
        ffi(|| unsafe {
            let visual = get(robot)?
                .0
                .bodies
                .get(body)
                .and_then(|b| b.visual_meshes.get(visual))
                .ok_or_else(|| invalid("visual index out of range"))?;
            output(
                out,
                visual as *const rapier3d_mjcf::MjcfVisualMesh as *const RprMjcfVisualMesh,
            )
        })
    })
}
/// Return a copy of visual pose, color, material, and geometry-kind flags.
/// @ingroup robotics
#[rapier_export(mjcf_visual_mesh)]
pub unsafe extern "C" fn rpr_mjcf_visual_mesh_info(
    visual: *const RprMjcfVisualMesh,
) -> RprMjcfVisualMeshInfo {
    ffi_value(|out: *mut RprMjcfVisualMeshInfo| {
        ffi(|| unsafe {
            let v = &get(visual)?.0;
            let m = v.material;
            output(
                out,
                RprMjcfVisualMeshInfo {
                    local_pose: v.local_pose.into(),
                    rgba: v.rgba.unwrap_or([0.7, 0.7, 0.75, 1.0]),
                    has_color: v.rgba.is_some() as RprBool,
                    has_material: m.is_some() as RprBool,
                    is_trimesh: v.shape.as_trimesh().is_some() as RprBool,
                    material: RprRenderMaterial {
                        metallic: m.map_or(0.0, |m| m.metallic),
                        roughness: m.map_or(1.0, |m| m.roughness),
                        reflectance: m.map_or(0.5, |m| m.reflectance),
                        emissive: m.map_or([0.0; 3], |m| m.emissive),
                    },
                },
            )
        })
    })
}
/// Returns an owned shared shape reference.
/// Returns an owned shape wrapper sharing the geometry. Release it with FreeSharedShape.
/// @ingroup robotics
#[rapier_export(mjcf_visual_mesh)]
pub unsafe extern "C" fn rpr_mjcf_visual_mesh_clone_shape(
    visual: *const RprMjcfVisualMesh,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            output(
                out,
                Box::into_raw(Box::new(RprSharedShape(get(visual)?.0.shape.clone()))),
            )
        })
    })
}
/// Copies flattened pairs of per-vertex UV coordinates.
/// @see @ref output_buffers
/// @ingroup robotics
#[rapier_export(mjcf_visual_mesh)]
pub unsafe extern "C" fn rpr_mjcf_visual_mesh_uvs(
    visual: *const RprMjcfVisualMesh,
    buffer: *mut f32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            copy_out(
                get(visual)?.0.uvs.as_deref().unwrap_or(&[]).as_flattened(),
                buffer,
                capacity,
                count,
            )
        })
    })
}
/// Copies flattened triples of per-vertex normals.
/// @see @ref output_buffers
/// @ingroup robotics
#[rapier_export(mjcf_visual_mesh)]
pub unsafe extern "C" fn rpr_mjcf_visual_mesh_normals(
    visual: *const RprMjcfVisualMesh,
    buffer: *mut f32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            copy_out(
                get(visual)?
                    .0
                    .normals
                    .as_deref()
                    .unwrap_or(&[])
                    .as_flattened(),
                buffer,
                capacity,
                count,
            )
        })
    })
}
/// Copies a NUL-terminated texture path, or an empty string for untextured meshes.
/// @see @ref output_buffers
/// @ingroup robotics
#[rapier_export(mjcf_visual_mesh)]
pub unsafe extern "C" fn rpr_mjcf_visual_mesh_texture(
    visual: *const RprMjcfVisualMesh,
    buffer: *mut c_char,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let v = &get(visual)?.0;
            let mut bytes = v
                .texture
                .as_ref()
                .map(|p| p.to_string_lossy().into_owned())
                .unwrap_or_default()
                .into_bytes();
            bytes.push(0);
            copy_out(&bytes, buffer.cast(), capacity, count)
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn loader_values_preserve_defaults_and_validate_before_io() {
        unsafe {
            let urdf = rpr_default_urdf_loader_options();
            let mjcf = rpr_default_mjcf_loader_options();
            let native_urdf = urdf.raw().unwrap();
            let native_mjcf = mjcf.raw().unwrap();
            assert_eq!(native_urdf.scale, UrdfLoaderOptions::default().scale);
            assert_eq!(
                native_mjcf.skip_plane_geoms,
                MjcfLoaderOptions::default().skip_plane_geoms
            );
            assert!(native_urdf.squeeze_empty_fixed_links);
            for body in [
                native_urdf.rigid_body_blueprint.build(),
                native_mjcf.rigid_body_blueprint.build(),
            ] {
                assert!(body.is_dynamic() && body.is_enabled());
                assert_eq!(body.gravity_scale(), 1.0);
            }
            assert_eq!(native_urdf.collider_blueprint.build().density(), 0.0);
            assert_eq!(native_mjcf.collider_blueprint.build().density(), 0.0);

            let mut copy = urdf;
            copy.rigidBodyBlueprint.gravityScale = 2.0;
            copy.colliderBlueprint.friction = 0.9;
            let converted = copy.raw().unwrap();
            assert_eq!(converted.rigid_body_blueprint.build().gravity_scale(), 2.0);
            assert_eq!(converted.collider_blueprint.build().friction(), 0.9);
            assert_eq!(urdf.rigidBodyBlueprint.gravityScale, 1.0);

            let path = c""; // Invalid options must fail before attempting to open a file.
            copy.scale = 0.0;
            assert!(rpr_urdf_robot_from_file(path.as_ptr(), &copy).is_null());
            assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
            let mut copy = mjcf;
            copy.skipPlaneGeoms = 2;
            assert!(rpr_mjcf_robot_from_file(path.as_ptr(), &copy).is_null());
            assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
            copy = mjcf;
            copy.rigidBodyBlueprint.bodyType = u32::MAX;
            assert!(copy.raw().is_err());

            let shape = rpr_ball_shared_shape(0.75);
            let mut copy = urdf;
            copy.colliderBlueprint.shape.kind = RPR_SHAPE_DESC_SHARED;
            copy.colliderBlueprint.shape.sharedShape = shape;
            let converted = copy.raw().unwrap();
            assert_eq!(rpr_free_shared_shape(shape), RPR_OK);
            assert_eq!(
                converted
                    .collider_blueprint
                    .build()
                    .shape()
                    .as_ball()
                    .unwrap()
                    .radius,
                0.75
            );
        }
    }

    #[test]
    fn urdf_load_uses_value_blueprints() {
        let path = std::env::temp_dir().join(format!("rapier-c-pod-{}.urdf", std::process::id()));
        std::fs::write(
            &path,
            r#"<robot name="pod"><link name="base">
            <collision><geometry><sphere radius="0.5"/></geometry></collision>
            </link></robot>"#,
        )
        .unwrap();
        let cpath = std::ffi::CString::new(path.to_str().unwrap()).unwrap();
        unsafe {
            let robot = {
                let mut options = rpr_default_urdf_loader_options();
                options.makeRootsFixed = 1;
                options.colliderBlueprint.friction = 0.8;
                rpr_urdf_robot_from_file(cpath.as_ptr(), &options)
            }; // No options allocation or destructor; the loaded robot is independent.
            std::fs::remove_file(path).unwrap();
            assert_eq!(rpr_last_status(), RPR_OK);
            assert!(!robot.is_null());
            let world = rpr_new_world();
            let handles = rpr_urdf_robot_insert_using_impulse_joints(world, robot);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!(rpr_free_urdf_robot(robot), RPR_OK);
            let mut bodies = [RprRigidBodyHandle::default(); 1];
            assert_eq!(
                rpr_urdf_robot_handles_bodies(handles, bodies.as_mut_ptr(), 1),
                1
            );
            assert_eq!(rpr_rigid_body_is_fixed(bodies[0]), 1);
            let mut colliders = [RprColliderHandle::default(); 1];
            assert_eq!(
                rpr_rigid_body_colliders(bodies[0], colliders.as_mut_ptr(), 1),
                1
            );
            assert_eq!(rpr_collider_friction(colliders[0]), 0.8);
            assert_eq!(rpr_free_urdf_robot_handles(handles), RPR_OK);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[test]
    fn imported_keyframes_and_actuators_are_accessible_through_c() {
        let path = std::env::temp_dir().join(format!("rapier-c-robot-{}.xml", std::process::id()));
        std::fs::write(
            &path,
            r#"<mujoco model="binding test">
          <worldbody><body name="link" pos="0 0 1"><joint name="hinge" type="hinge" axis="0 1 0"/>
            <geom type="capsule" size="0.1 0.5" mass="1"/>
            <geom type="sphere" size="0.15" contype="0" conaffinity="0" rgba="1 0 0 1"/>
          </body></worldbody>
          <actuator><position joint="hinge" kp="10"/></actuator>
          <keyframe><key name="home" qpos="0.5" ctrl="0.5"/></keyframe>
        </mujoco>"#,
        )
        .unwrap();
        let path_string = std::ffi::CString::new(path.to_str().unwrap()).unwrap();
        unsafe {
            let options = rpr_default_mjcf_loader_options();
            let robot = rpr_mjcf_robot_from_file(path_string.as_ptr(), &options);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert!(!robot.is_null());
            std::fs::remove_file(path).unwrap();
            let mut count = 0;
            assert_eq!(
                {
                    let value = rpr_mjcf_robot_keyframe_count(robot);
                    let status = rpr_last_status();
                    let destination: *mut usize = &mut count;
                    if !destination.is_null() {
                        destination.write(value);
                    }
                    status
                },
                RPR_OK
            );
            assert_eq!(count, 1);
            let mut name = [0i8; 5];
            assert_eq!(
                {
                    let value =
                        rpr_mjcf_robot_keyframe_name(robot, 0, name.as_mut_ptr(), name.len());
                    let status = rpr_last_status();
                    let destination: *mut usize = &mut count;
                    if !destination.is_null() {
                        destination.write(value);
                    }
                    status
                },
                RPR_OK
            );
            assert_eq!(CStr::from_ptr(name.as_ptr()).to_bytes(), b"home");
            let mut ctrl = [0.0];
            assert_eq!(
                {
                    let value =
                        rpr_mjcf_robot_keyframe_controls(robot, 0, ctrl.as_mut_ptr(), ctrl.len());
                    let status = rpr_last_status();
                    let destination: *mut usize = &mut count;
                    if !destination.is_null() {
                        destination.write(value);
                    }
                    status
                },
                RPR_OK
            );
            assert_eq!(ctrl, [0.5]);
            let mut world = RprPhysicsWorld(PhysicsWorld::new());
            let mut handles = std::ptr::null_mut();
            assert_eq!(
                native_mjcf_robot_insert_using_multibody_joints(
                    robot,
                    &mut world,
                    0,
                    &mut handles,
                    std::ptr::null_mut()
                ),
                RPR_OK
            );
            assert_eq!(
                {
                    let value = rpr_mjcf_robot_handles_actuator_count(handles);
                    let status = rpr_last_status();
                    let destination: *mut usize = &mut count;
                    if !destination.is_null() {
                        destination.write(value);
                    }
                    status
                },
                RPR_OK
            );
            assert_eq!(count, 1);
            assert_eq!(
                native_mjcf_robot_handles_apply_keyframe(handles, &mut world, robot, 0),
                RPR_OK
            );
            assert_eq!(
                native_mjcf_robot_handles_apply_controls_scaled(
                    handles,
                    &mut world,
                    ctrl.as_ptr(),
                    1,
                    0.25
                ),
                RPR_OK
            );
            for _ in 0..30 {
                world.0.step();
            }
            for (_, body) in world.0.bodies.iter() {
                assert!(body.translation().is_finite());
            }
            assert_eq!(
                native_mjcf_robot_handles_apply_keyframe(handles, &mut world, robot, 1),
                RPR_INVALID_ARGUMENT
            );
            assert_eq!(rpr_free_mjcf_robot_handles(handles), RPR_OK);
            assert_eq!(rpr_free_mjcf_robot(robot), RPR_OK);
        }
    }
}
