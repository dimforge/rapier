//! Values returned by operations that produce several related outputs.
#![allow(non_snake_case)]
use crate::*;

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprRayToi {
    pub collider: RprColliderHandle,
    pub toi: RprReal,
    pub found: RprBool,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalRayHit {
    pub hit: RprRayHit,
    pub found: RprBool,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprVelocityCorrection {
    pub linear: RprVector,
    pub angularVelocity: RprAngVector,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprParticleDestination {
    pub body: RprSoftBodyHandle,
    pub index: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalParticleDestination {
    pub body: RprSoftBodyHandle,
    pub index: u32,
    pub found: RprBool,
}

/// Borrowed bytes. Valid while the source Bytes object remains alive; never free data.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprByteView {
    pub data: *const u8,
    pub count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprVoxelQuery {
    pub key: RprVoxelKey,
    pub center: RprVector,
    pub size: RprVector,
    pub found: RprBool,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprJointBodies {
    pub body1: RprRigidBodyHandle,
    pub body2: RprRigidBodyHandle,
}
