use crate::*;

/// Floating-point scalar: float for f32, double for f64.
/// @ingroup math
#[cfg(feature = "f32")]
pub type RprReal = f32;
/// Floating-point scalar: float for f32, double for f64.
/// @ingroup math
#[cfg(feature = "f64")]
pub type RprReal = f64;
/// ABI booleans are uint32_t: zero is false, one is true.
/// @ingroup math
pub type RprBool = u32;
/// @ingroup errors
/// C binary ABI revision expected by this header.
pub const RPR_ABI_VERSION: u32 = 1;
/// @ingroup rigid_bodies
/// Dynamic body affected by forces and contacts.
pub const RPR_DYNAMIC: u32 = 0;
/// @ingroup rigid_bodies
/// Immovable body.
pub const RPR_FIXED: u32 = 1;
/// @ingroup rigid_bodies
/// Kinematic body controlled by its next pose.
pub const RPR_KINEMATIC_POSITION_BASED: u32 = 2;
/// @ingroup rigid_bodies
/// Kinematic body controlled by its velocity.
pub const RPR_KINEMATIC_VELOCITY_BASED: u32 = 3;
/// @ingroup colliders
/// Enable collision-start and collision-stop events for this collider.
pub const RPR_COLLISION_EVENTS: u32 = 1;
/// @ingroup colliders
/// Enable contact-force events for this collider, subject to its force threshold.
pub const RPR_CONTACT_FORCE_EVENTS: u32 = 2;
/// @ingroup colliders
/// Combine the two material coefficients by their arithmetic mean.
pub const RPR_COMBINE_AVERAGE: u32 = 0;
/// @ingroup colliders
/// Use the smaller of the two material coefficients.
pub const RPR_COMBINE_MIN: u32 = 1;
/// @ingroup colliders
/// Multiply the two material coefficients.
pub const RPR_COMBINE_MULTIPLY: u32 = 2;
/// @ingroup colliders
/// Use the larger of the two material coefficients.
pub const RPR_COMBINE_MAX: u32 = 3;

/// Cartesian vector with two or three components.
/// @ingroup math
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprVector {
    /// X component.
    pub x: RprReal,
    /// Y component.
    pub y: RprReal,
    #[cfg(feature = "dim3")]
    /// Z component.
    pub z: RprReal,
}
impl RprVector {
    pub(crate) fn raw(self) -> Result<Vector> {
        finite(self.x)?;
        finite(self.y)?;
        #[cfg(feature = "dim2")]
        {
            Ok(Vector::new(self.x, self.y))
        }
        #[cfg(feature = "dim3")]
        {
            finite(self.z)?;
            Ok(Vector::new(self.x, self.y, self.z))
        }
    }
}
impl From<Vector> for RprVector {
    fn from(v: Vector) -> Self {
        Self {
            x: v.x,
            y: v.y,
            #[cfg(feature = "dim3")]
            z: v.z,
        }
    }
}
/// Angular scalar in 2D or vector in 3D, in radians for angular displacement.
/// @ingroup math
#[cfg(feature = "dim2")]
pub type RprAngVector = RprReal;
/// Angular scalar in 2D or vector in 3D, in radians for angular displacement.
/// @ingroup math
#[cfg(feature = "dim3")]
pub type RprAngVector = RprVector;
pub(crate) fn angular(v: RprAngVector) -> Result<AngVector> {
    #[cfg(feature = "dim2")]
    {
        finite(v)
    }
    #[cfg(feature = "dim3")]
    {
        v.raw()
    }
}
/// 2D: angle in radians. 3D: unit quaternion in x,y,z,w order (normalized on input).
/// @ingroup math
#[repr(C)]
#[derive(Copy, Clone)]
pub struct RprRotation {
    #[cfg(feature = "dim2")]
    /// Rotation angle in radians.
    pub angle: RprReal,
    #[cfg(feature = "dim3")]
    /// X component.
    pub x: RprReal,
    #[cfg(feature = "dim3")]
    /// Y component.
    pub y: RprReal,
    #[cfg(feature = "dim3")]
    /// Z component.
    pub z: RprReal,
    #[cfg(feature = "dim3")]
    /// Quaternion scalar component.
    pub w: RprReal,
}
impl Default for RprRotation {
    fn default() -> Self {
        Rotation::IDENTITY.into()
    }
}
impl RprRotation {
    pub(crate) fn raw(self) -> Result<Rotation> {
        #[cfg(feature = "dim2")]
        {
            Ok(Rotation::from_angle(finite(self.angle)?))
        }
        #[cfg(feature = "dim3")]
        {
            finite(self.x)?;
            finite(self.y)?;
            finite(self.z)?;
            finite(self.w)?;
            let q = Rotation::from_xyzw(self.x, self.y, self.z, self.w);
            ensure(
                q.length_squared().is_finite() && q.length_squared() > 1.0e-20,
                "quaternion must be nonzero and finite",
            )?;
            Ok(q.normalize())
        }
    }
}
impl From<Rotation> for RprRotation {
    fn from(q: Rotation) -> Self {
        #[cfg(feature = "dim2")]
        {
            Self { angle: q.angle() }
        }
        #[cfg(feature = "dim3")]
        {
            Self {
                x: q.x,
                y: q.y,
                z: q.z,
                w: q.w,
            }
        }
    }
}
/// Rigid transform combining translation and rotation. Use TranslationPose with a zero vector for
/// identity.
/// @ingroup math
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprPose {
    /// Translation vector.
    pub translation: RprVector,
    /// Rotation value.
    pub rotation: RprRotation,
}
impl RprPose {
    pub(crate) fn raw(self) -> Result<Pose> {
        Ok(Pose::from_parts(
            self.translation.raw()?,
            self.rotation.raw()?,
        ))
    }
}
impl From<Pose> for RprPose {
    fn from(p: Pose) -> Self {
        Self {
            translation: p.translation.into(),
            rotation: p.rotation.into(),
        }
    }
}
/// Collision membership/filter masks and their pairwise combination rule.
/// @ingroup math
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprInteractionGroups {
    /// Groups this object belongs to, as a 32-bit mask.
    pub memberships: u32,
    /// Membership groups accepted by this object, as a 32-bit mask.
    pub filter: u32,
    /// 0 requires both membership/filter tests; 1 accepts either test.
    pub test_mode: u32,
}
impl RprInteractionGroups {
    pub(crate) fn raw(self) -> Result<InteractionGroups> {
        ensure(self.test_mode <= 1, "invalid group test mode")?;
        Ok(InteractionGroups::new(
            Group::from_bits_retain(self.memberships),
            Group::from_bits_retain(self.filter),
            if self.test_mode == 0 {
                InteractionTestMode::And
            } else {
                InteractionTestMode::Or
            },
        ))
    }
}
impl From<InteractionGroups> for RprInteractionGroups {
    fn from(g: InteractionGroups) -> Self {
        Self {
            memberships: g.memberships.bits(),
            filter: g.filter.bits(),
            test_mode: g.test_mode as u32,
        }
    }
}
/// Application-defined 128-bit value. Contains no owned pointers.
/// @ingroup math
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprUserData {
    /// Low 64 bits of the application value.
    pub low: u64,
    /// High 64 bits of the application value.
    pub high: u64,
}
impl RprUserData {
    pub(crate) fn raw(self) -> u128 {
        self.low as u128 | ((self.high as u128) << 64)
    }
}
impl From<u128> for RprUserData {
    fn from(v: u128) -> Self {
        Self {
            low: v as u64,
            high: (v >> 64) as u64,
        }
    }
}
/// Axis-aligned bounding box.
/// @ingroup math
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprAabb {
    /// Minimum corner in each coordinate.
    pub mins: RprVector,
    /// Maximum corner in each coordinate.
    pub maxs: RprVector,
}
/// Spring softness expressed as natural frequency and damping ratio.
/// @ingroup math
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprSpringCoefficients {
    /// Nonnegative spring natural frequency in Hz.
    pub natural_frequency: RprReal,
    /// Nonnegative damping ratio; 1 is critical damping.
    pub damping_ratio: RprReal,
}
impl RprSpringCoefficients {
    pub(crate) fn raw(self) -> Result<SpringCoefficients<Real>> {
        Ok(SpringCoefficients::new(
            nonnegative(self.natural_frequency)?,
            nonnegative(self.damping_ratio)?,
        ))
    }
}
impl From<SpringCoefficients<Real>> for RprSpringCoefficients {
    fn from(v: SpringCoefficients<Real>) -> Self {
        Self {
            natural_frequency: v.natural_frequency,
            damping_ratio: v.damping_ratio,
        }
    }
}
/// Runtime ABI identity and fundamental type sizes.
/// @ingroup errors
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprBuildInfo {
    /// Binary ABI revision; compare with the header ABI_VERSION.
    pub abi_version: u32,
    /// Spatial dimension, 2 or 3.
    pub dimension: u32,
    /// Size of Real in bytes, 4 or 8.
    pub real_size: u32,
    /// Pointer size in bytes.
    pub pointer_size: u32,
}
/// Return ABI version, dimension, scalar size, and pointer size of the linked library.
/// @ingroup errors
#[rapier_export]
pub extern "C" fn rpr_build_info() -> RprBuildInfo {
    RprBuildInfo {
        abi_version: RPR_ABI_VERSION,
        dimension: rapier::math::DIM as u32,
        real_size: std::mem::size_of::<Real>() as u32,
        pointer_size: std::mem::size_of::<usize>() as u32,
    }
}
/// Release version of the loaded C bindings, e.g. "0.35.3+c.2".
/// The suffix identifies the C bindings revision for the Rust crate version.
/// The UTF-8, NUL-terminated string is borrowed for the library's lifetime; do not free it.
/// This release identifier is independent of the ABI compatibility version.
/// @ingroup errors
#[rapier_export]
pub extern "C" fn rpr_version() -> *const std::ffi::c_char {
    static VERSION: &[u8] = concat!(env!("RAPIER_C_VERSION"), "\0").as_bytes();
    VERSION.as_ptr().cast()
}

/// Cargo profile of the loaded physics library: "debug" or "release".
/// Custom profiles report the corresponding inherited Cargo profile category.
/// The UTF-8, NUL-terminated string is borrowed for the library's lifetime; do not free it.
/// This is independent of the consumer's build mode and of per-package optimization overrides.
/// @ingroup errors
#[rapier_export]
pub extern "C" fn rpr_build_profile() -> *const std::ffi::c_char {
    static PROFILE: &[u8] = concat!(env!("RAPIER_CARGO_PROFILE"), "\0").as_bytes();
    PROFILE.as_ptr().cast()
}

/// Features available through the loaded C library, independent of consumer defines.
/// @ingroup errors
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprBuildFeatures {
    /// Whether native profiling timers were compiled in.
    pub profiling: RprBool,
    /// Solver SIMD lane count. Hardware instruction width depends on the target CPU.
    pub simd_lanes: u32,
    /// Whether this library exposes Rapier's parallel execution and thread-pool APIs.
    pub parallel: RprBool,
}
/// Return profiling, SIMD width, and parallelism of the linked library.
/// @ingroup errors
#[rapier_export]
pub extern "C" fn rpr_build_features() -> RprBuildFeatures {
    RprBuildFeatures {
        profiling: cfg!(feature = "profiler") as RprBool,
        simd_lanes: rapier::math::SIMD_WIDTH as u32,
        parallel: cfg!(feature = "parallel") as RprBool,
    }
}

pub(crate) fn body_type(value: u32) -> Result<RigidBodyType> {
    match value {
        0 => Ok(RigidBodyType::Dynamic),
        1 => Ok(RigidBodyType::Fixed),
        2 => Ok(RigidBodyType::KinematicPositionBased),
        3 => Ok(RigidBodyType::KinematicVelocityBased),
        _ => Err(invalid("unknown rigid body type")),
    }
}
pub(crate) fn combine(value: u32) -> Result<CoefficientCombineRule> {
    match value {
        0 => Ok(CoefficientCombineRule::Average),
        1 => Ok(CoefficientCombineRule::Min),
        2 => Ok(CoefficientCombineRule::Multiply),
        3 => Ok(CoefficientCombineRule::Max),
        _ => Err(invalid("unknown combine rule")),
    }
}

/// Copyable non-owning handle: world pointer plus entity index and generation.
/// The world must remain alive throughout every use. Copying does not retain it.
/// UINT32_MAX/UINT32_MAX with a NULL world is invalid.
/// @ingroup rigid_bodies
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RprRigidBodyHandle {
    /// Borrowed owning world. Never use this handle after freeing that world.
    pub world: *mut RprWorld,
    /// Slot index; UINT32_MAX denotes the explicit invalid handle.
    pub index: u32,
    /// Slot generation used to reject stale handles. Do not modify it.
    pub generation: u32,
}
impl Default for RprRigidBodyHandle {
    fn default() -> Self {
        Self {
            world: std::ptr::null_mut(),
            index: u32::MAX,
            generation: u32::MAX,
        }
    }
}
impl RprRigidBodyHandle {
    pub(crate) fn raw(self) -> RigidBodyHandle {
        RigidBodyHandle::from_raw_parts(self.index, self.generation)
    }
}
impl From<RigidBodyHandle> for RprRigidBodyHandle {
    fn from(h: RigidBodyHandle) -> Self {
        let (index, generation) = h.into_raw_parts();
        Self {
            world: std::ptr::null_mut(),
            index,
            generation,
        }
    }
}
/// Copyable non-owning handle: world pointer plus entity index and generation.
/// The world must remain alive throughout every use. Copying does not retain it.
/// UINT32_MAX/UINT32_MAX with a NULL world is invalid.
/// @ingroup colliders
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RprColliderHandle {
    /// Borrowed owning world. Never use this handle after freeing that world.
    pub world: *mut RprWorld,
    /// Slot index; UINT32_MAX denotes the explicit invalid handle.
    pub index: u32,
    /// Slot generation used to reject stale handles. Do not modify it.
    pub generation: u32,
}
impl Default for RprColliderHandle {
    fn default() -> Self {
        Self {
            world: std::ptr::null_mut(),
            index: u32::MAX,
            generation: u32::MAX,
        }
    }
}
impl RprColliderHandle {
    pub(crate) fn raw(self) -> ColliderHandle {
        ColliderHandle::from_raw_parts(self.index, self.generation)
    }
}
impl From<ColliderHandle> for RprColliderHandle {
    fn from(h: ColliderHandle) -> Self {
        let (index, generation) = h.into_raw_parts();
        Self {
            world: std::ptr::null_mut(),
            index,
            generation,
        }
    }
}
/// Copyable non-owning handle: world pointer plus entity index and generation.
/// The world must remain alive throughout every use. Copying does not retain it.
/// UINT32_MAX/UINT32_MAX with a NULL world is invalid.
/// @ingroup joints
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RprImpulseJointHandle {
    /// Borrowed owning world. Never use this handle after freeing that world.
    pub world: *mut RprWorld,
    /// Zero-based element index.
    pub index: u32,
    /// Slot generation used to reject stale handles. Do not modify it.
    pub generation: u32,
}
impl Default for RprImpulseJointHandle {
    fn default() -> Self {
        Self {
            world: std::ptr::null_mut(),
            index: u32::MAX,
            generation: u32::MAX,
        }
    }
}
impl RprImpulseJointHandle {
    pub(crate) fn raw(self) -> ImpulseJointHandle {
        ImpulseJointHandle::from_raw_parts(self.index, self.generation)
    }
}
impl From<ImpulseJointHandle> for RprImpulseJointHandle {
    fn from(h: ImpulseJointHandle) -> Self {
        let (index, generation) = h.into_raw_parts();
        Self {
            world: std::ptr::null_mut(),
            index,
            generation,
        }
    }
}
/// Copyable non-owning handle: world pointer plus entity index and generation.
/// The world must remain alive throughout every use. Copying does not retain it.
/// UINT32_MAX/UINT32_MAX with a NULL world is invalid.
/// @ingroup joints
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RprMultibodyJointHandle {
    /// Borrowed owning world. Never use this handle after freeing that world.
    pub world: *mut RprWorld,
    /// Zero-based element index.
    pub index: u32,
    /// Slot generation used to reject stale handles. Do not modify it.
    pub generation: u32,
}
impl Default for RprMultibodyJointHandle {
    fn default() -> Self {
        Self {
            world: std::ptr::null_mut(),
            index: u32::MAX,
            generation: u32::MAX,
        }
    }
}
impl RprMultibodyJointHandle {
    pub(crate) fn raw(self) -> MultibodyJointHandle {
        MultibodyJointHandle::from_raw_parts(self.index, self.generation)
    }
}
impl From<MultibodyJointHandle> for RprMultibodyJointHandle {
    fn from(h: MultibodyJointHandle) -> Self {
        let (index, generation) = h.into_raw_parts();
        Self {
            world: std::ptr::null_mut(),
            index,
            generation,
        }
    }
}
/// Copyable non-owning handle: world pointer plus entity index and generation.
/// The world must remain alive throughout every use. Copying does not retain it.
/// UINT32_MAX/UINT32_MAX with a NULL world is invalid.
/// @ingroup soft_bodies
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RprSoftBodyHandle {
    /// Borrowed owning world. Never use this handle after freeing that world.
    pub world: *mut RprWorld,
    /// Zero-based element index.
    pub index: u32,
    /// Slot generation used to reject stale handles. Do not modify it.
    pub generation: u32,
}
impl Default for RprSoftBodyHandle {
    fn default() -> Self {
        Self {
            world: std::ptr::null_mut(),
            index: u32::MAX,
            generation: u32::MAX,
        }
    }
}
impl RprSoftBodyHandle {
    pub(crate) fn raw(self) -> SoftBodyHandle {
        SoftBodyHandle::from_raw_parts(self.index, self.generation)
    }
}
impl From<SoftBodyHandle> for RprSoftBodyHandle {
    fn from(h: SoftBodyHandle) -> Self {
        let (index, generation) = h.into_raw_parts();
        Self {
            world: std::ptr::null_mut(),
            index,
            generation,
        }
    }
}

/// @ingroup colliders
/// Invoke the contact-pair filtering hook for this collider.
pub const RPR_FILTER_CONTACT_PAIRS: u32 = 1;
/// @ingroup colliders
/// Invoke the sensor-intersection filtering hook for this collider.
pub const RPR_FILTER_INTERSECTION_PAIR: u32 = 2;
/// @ingroup colliders
/// Invoke the solver-contact modification hook for this collider.
pub const RPR_MODIFY_SOLVER_CONTACTS: u32 = 4;
/// @ingroup queries
/// Query filter: exclude fixed.
pub const RPR_QUERY_EXCLUDE_FIXED: u32 = 1;
/// @ingroup queries
/// Query filter: exclude kinematic.
pub const RPR_QUERY_EXCLUDE_KINEMATIC: u32 = 2;
/// @ingroup queries
/// Query filter: exclude dynamic.
pub const RPR_QUERY_EXCLUDE_DYNAMIC: u32 = 4;
/// @ingroup queries
/// Query filter: exclude sensors.
pub const RPR_QUERY_EXCLUDE_SENSORS: u32 = 8;
/// @ingroup queries
/// Query filter: exclude solids.
pub const RPR_QUERY_EXCLUDE_SOLIDS: u32 = 16;
/// @ingroup queries
/// Query filter: only dynamic.
pub const RPR_QUERY_ONLY_DYNAMIC: u32 = 3;
/// @ingroup queries
/// Query filter: only kinematic.
pub const RPR_QUERY_ONLY_KINEMATIC: u32 = 5;
/// @ingroup queries
/// Query filter: only fixed.
pub const RPR_QUERY_ONLY_FIXED: u32 = 6;
/// @ingroup events
/// Debug-render flag: collider shapes.
pub const RPR_DEBUG_COLLIDER_SHAPES: u32 = 1;
/// @ingroup events
/// Debug-render flag: rigid body axes.
pub const RPR_DEBUG_RIGID_BODY_AXES: u32 = 2;
/// @ingroup events
/// Debug-render flag: multibody joints.
pub const RPR_DEBUG_MULTIBODY_JOINTS: u32 = 4;
/// @ingroup events
/// Debug-render flag: impulse joints.
pub const RPR_DEBUG_IMPULSE_JOINTS: u32 = 8;
/// @ingroup events
/// Debug-render flag: solver contacts.
pub const RPR_DEBUG_SOLVER_CONTACTS: u32 = 16;
/// @ingroup events
/// Debug-render flag: contacts.
pub const RPR_DEBUG_CONTACTS: u32 = 32;
/// @ingroup events
/// Debug-render flag: collider aabbs.
pub const RPR_DEBUG_COLLIDER_AABBS: u32 = 64;
/// @ingroup events
/// Debug-render flag: soft bodies.
pub const RPR_DEBUG_SOFT_BODIES: u32 = 128;
/// @ingroup events
/// Debug-render flag: pseudo normals.
pub const RPR_DEBUG_PSEUDO_NORMALS: u32 = 256;
/// @ingroup events
/// Debug-render flag: soft volume contacts.
pub const RPR_DEBUG_SOFT_VOLUME_CONTACTS: u32 = 512;
/// @ingroup events
/// Debug-render flag: soft body stress.
pub const RPR_DEBUG_SOFT_BODY_STRESS: u32 = 1024;
/// @ingroup colliders
/// Require both membership/filter intersections to be nonempty.
pub const RPR_GROUPS_AND: u32 = 0;
/// @ingroup colliders
/// Accept either membership/filter intersection if both participants select OR; otherwise use AND.
pub const RPR_GROUPS_OR: u32 = 1;
/// @ingroup joints
/// Motor stiffness and damping are acceleration-based, independent of mass.
pub const RPR_MOTOR_ACCELERATION_BASED: u32 = 0;
/// @ingroup joints
/// Motor stiffness and damping are force-based, so response depends on mass.
pub const RPR_MOTOR_FORCE_BASED: u32 = 1;
/// @ingroup soft_bodies
/// Cell model that constrains volume without elastic shear response.
pub const RPR_SOFT_CELL_VOLUME: u32 = 0;
/// @ingroup soft_bodies
/// Corotational elastic cell model.
pub const RPR_SOFT_CELL_COROTATIONAL: u32 = 1;
/// @ingroup soft_bodies
/// Neo-Hookean elastic cell model.
pub const RPR_SOFT_CELL_NEO_HOOKEAN: u32 = 2;
/// @ingroup soft_bodies
/// Use the constraint-based soft-body solver.
pub const RPR_SOFT_SOLVER_CONSTRAINTS: u32 = 0;
/// @ingroup soft_bodies
/// Use the finite-element solver; requires RAPIER_FEM.
pub const RPR_SOFT_SOLVER_FEM: u32 = 1;
/// @ingroup joints
/// Joint axis index for translation along local X.
pub const RPR_AXIS_LIN_X: u32 = 0;
/// @ingroup joints
/// Joint axis index for translation along local Y.
pub const RPR_AXIS_LIN_Y: u32 = 1;
/// @ingroup rigid_bodies
/// Rigid-body lock bit: translation x.
pub const RPR_LOCK_TRANSLATION_X: u32 = 1;
/// @ingroup rigid_bodies
/// Rigid-body lock bit: translation y.
pub const RPR_LOCK_TRANSLATION_Y: u32 = 2;
/// @ingroup rigid_bodies
/// Rigid-body lock bit: translation z.
pub const RPR_LOCK_TRANSLATION_Z: u32 = 4;
/// @ingroup rigid_bodies
/// Rigid-body lock bit: rotation x.
pub const RPR_LOCK_ROTATION_X: u32 = 8;
/// @ingroup rigid_bodies
/// Rigid-body lock bit: rotation y.
pub const RPR_LOCK_ROTATION_Y: u32 = 16;
/// @ingroup rigid_bodies
/// Rigid-body lock bit: rotation z.
pub const RPR_LOCK_ROTATION_Z: u32 = 32;
/// @ingroup joints
/// Joint axis index for the first rotation: Z in 2D, local X in 3D.
#[cfg(feature = "dim2")]
pub const RPR_AXIS_ANG_X: u32 = 2;
/// @ingroup joints
/// Locked-axis mask for a fixed joint: all translations and rotations.
#[cfg(feature = "dim2")]
pub const RPR_JOINT_FIXED_AXES: u32 = 7;
/// @ingroup joints
/// Locked-axis mask for a revolute joint: only the first angular axis is free.
#[cfg(feature = "dim2")]
pub const RPR_JOINT_REVOLUTE_AXES: u32 = 3;
/// @ingroup joints
/// Locked-axis mask for a prismatic joint: only translation along local X is free.
#[cfg(feature = "dim2")]
pub const RPR_JOINT_PRISMATIC_AXES: u32 = 6;
/// @ingroup joints
/// Joint axis index for translation along local Z.
#[cfg(feature = "dim3")]
pub const RPR_AXIS_LIN_Z: u32 = 2;
/// @ingroup joints
/// Joint axis index for the first rotation: Z in 2D, local X in 3D.
#[cfg(feature = "dim3")]
pub const RPR_AXIS_ANG_X: u32 = 3;
/// @ingroup joints
/// Joint axis index for rotation around local Y.
#[cfg(feature = "dim3")]
pub const RPR_AXIS_ANG_Y: u32 = 4;
/// @ingroup joints
/// Joint axis index for rotation around local Z.
#[cfg(feature = "dim3")]
pub const RPR_AXIS_ANG_Z: u32 = 5;
/// @ingroup joints
/// Locked-axis mask for a fixed joint: all translations and rotations.
#[cfg(feature = "dim3")]
pub const RPR_JOINT_FIXED_AXES: u32 = 63;
/// @ingroup joints
/// Locked-axis mask for a revolute joint: only the first angular axis is free.
#[cfg(feature = "dim3")]
pub const RPR_JOINT_REVOLUTE_AXES: u32 = 55;
/// @ingroup joints
/// Locked-axis mask for a prismatic joint: only translation along local X is free.
#[cfg(feature = "dim3")]
pub const RPR_JOINT_PRISMATIC_AXES: u32 = 62;
/// @ingroup joints
/// Locked-axis mask for a spherical joint: translations locked, rotations free.
#[cfg(feature = "dim3")]
pub const RPR_JOINT_SPHERICAL_AXES: u32 = 7;

pub(crate) fn angular_out(v: AngVector) -> RprAngVector {
    #[cfg(feature = "dim2")]
    {
        v
    }
    #[cfg(feature = "dim3")]
    {
        v.into()
    }
}
pub(crate) fn real_pi() -> Real {
    #[cfg(feature = "f32")]
    {
        std::f32::consts::PI
    }
    #[cfg(feature = "f64")]
    {
        std::f64::consts::PI
    }
}

/// @ingroup soft_bodies
/// Read-only body type of soft-body cluster proxies; cannot be used to construct a rigid body.
pub const RPR_SOFT_FRAME: u32 = 4;
/// @ingroup queries
/// Shape-cast iteration limit reached before convergence.
pub const RPR_SHAPE_CAST_OUT_OF_ITERATIONS: u32 = 0;
/// @ingroup queries
/// Shape cast converged to the reported impact.
pub const RPR_SHAPE_CAST_CONVERGED: u32 = 1;
/// @ingroup queries
/// Shape-cast numerical solver failed to converge.
pub const RPR_SHAPE_CAST_FAILED: u32 = 2;
/// @ingroup queries
/// Shapes overlap or are within the target distance at the start of the cast.
pub const RPR_SHAPE_CAST_PENETRATING: u32 = 3;
/// @ingroup queries
/// The hit feature is unspecified.
pub const RPR_FEATURE_UNKNOWN: u32 = 0;
/// @ingroup queries
/// The feature ID denotes a vertex.
pub const RPR_FEATURE_VERTEX: u32 = 1;
/// @ingroup queries
/// The feature ID denotes an edge.
pub const RPR_FEATURE_EDGE: u32 = 2;
/// @ingroup queries
/// The feature ID denotes a face.
pub const RPR_FEATURE_FACE: u32 = 3;

/// @ingroup joints
/// Insert reduced-coordinate joints as kinematic articulations.
pub const RPR_MULTIBODY_JOINTS_ARE_KINEMATIC: u8 = 1;
/// @ingroup joints
/// Disable contacts between colliders of the inserted articulation.
pub const RPR_MULTIBODY_DISABLE_SELF_CONTACTS: u8 = 2;
/// @ingroup joints
/// Skip joints that would close a loop in the articulation.
pub const RPR_MULTIBODY_SKIP_LOOP_CLOSURES: u8 = 4;
/// @ingroup joints
/// Do not import joint motors into the articulation.
pub const RPR_MULTIBODY_SKIP_JOINT_MOTORS: u8 = 8;
/// @ingroup joints
/// Do not import joint limits into the articulation.
pub const RPR_MULTIBODY_SKIP_JOINT_LIMITS: u8 = 16;
/// @ingroup joints
/// Do not import joint springs into the articulation.
pub const RPR_MULTIBODY_SKIP_JOINT_SPRINGS: u8 = 32;

/// @ingroup shapes
/// Merge triangle-mesh vertices with identical positions.
pub const RPR_TRIMESH_MERGE_DUPLICATE_VERTICES: u32 = 16;
/// @ingroup shapes
/// Correct contact normals at internal mesh edges; includes duplicate-vertex merging.
pub const RPR_TRIMESH_FIX_INTERNAL_EDGES: u32 = 144;
/// @ingroup shapes
/// Prepare triangle-mesh acceleration data for deformation.
pub const RPR_TRIMESH_DEFORMABLE: u32 = 256;
/// @ingroup shapes
/// Correct internal-edge contacts on both sides of the triangle mesh.
pub const RPR_TRIMESH_FIX_INTERNAL_EDGES_TWO_SIDED: u32 = 656;
/// @ingroup shapes
/// Correct contact normals at internal heightfield edges.
pub const RPR_HEIGHTFIELD_FIX_INTERNAL_EDGES: u32 = 1;
