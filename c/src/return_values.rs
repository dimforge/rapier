//! Values returned by operations that produce several related outputs.
#![allow(non_snake_case)]
use crate::*;

/// Optional ray collider/time result. A miss is found = 0 with status OK.
/// @ingroup queries
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprRayToi {
    /// World-bound collider handle.
    pub collider: RprColliderHandle,
    /// Ray parameter t at impact: origin + direction * t.
    pub toi: RprReal,
    /// Whether a result exists; other result fields are meaningful only when this is 1.
    pub found: RprBool,
}

/// Optional full ray result. A miss is found = 0 with status OK.
/// @ingroup queries
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalRayHit {
    /// Shape/ray impact details.
    pub hit: RprRayHit,
    /// Whether a result exists; other result fields are meaningful only when this is 1.
    pub found: RprBool,
}

/// Linear and angular velocity correction computed by a controller.
/// @ingroup controllers
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprVelocityCorrection {
    /// World-space linear velocity correction.
    pub linear: RprVector,
    /// World-space angular velocity correction, in radians per second.
    pub angularVelocity: RprAngVector,
}

/// Particle remapping after tearing.
/// @ingroup math
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprParticleDestination {
    /// World-bound rigid/soft-body handle, as selected by the field type.
    pub body: RprSoftBodyHandle,
    /// Zero-based element index.
    pub index: u32,
}

/// Optional particle remapping; check found before reading the destination.
/// @ingroup math
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalParticleDestination {
    /// World-bound rigid/soft-body handle, as selected by the field type.
    pub body: RprSoftBodyHandle,
    /// Zero-based element index.
    pub index: u32,
    /// Whether a result exists; other result fields are meaningful only when this is 1.
    pub found: RprBool,
}

/// Borrowed bytes. Valid while the source Bytes object remains alive; never free data.
/// @ingroup math
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprByteView {
    /// Borrowed pointer to contiguous elements; NULL is allowed when count is zero.
    pub data: *const u8,
    /// Number of elements, not bytes unless the element type is a byte.
    pub count: usize,
}

/// Optional voxel lookup result; check found before reading voxel data.
/// @ingroup queries
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprVoxelQuery {
    /// Integer voxel coordinates.
    pub key: RprVoxelKey,
    /// World-space wheel center.
    pub center: RprVector,
    /// Voxel dimensions along each axis.
    pub size: RprVector,
    /// Whether a result exists; other result fields are meaningful only when this is 1.
    pub found: RprBool,
}

/// World-bound handles of the two connected bodies.
/// @ingroup joints
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprJointBodies {
    /// First connected body.
    pub body1: RprRigidBodyHandle,
    /// Second connected body.
    pub body2: RprRigidBodyHandle,
}
