//! World ownership on foreign entity handles. Internal native adapters may create
//! unbound handles; attach the owner before returning them across the C boundary.
use crate::*;

pub(crate) trait WorldHandles {
    fn attach_world(&mut self, world: *mut RprWorld);
    fn check_world(&self, world: *const RprWorld) -> Result;
    fn with_world(mut self, world: *mut RprWorld) -> Self
    where
        Self: Sized,
    {
        self.attach_world(world);
        self
    }
}
macro_rules! entity_world {
    ($($ty:ty),* $(,)?) => {$ (
        // Handles carry an address only; dereferencing it requires an unsafe API
        // call with external lifetime synchronization and the world's borrow gate.
        unsafe impl Send for $ty {}
        unsafe impl Sync for $ty {}
        impl WorldHandles for $ty {
            fn attach_world(&mut self, world: *mut RprWorld) {
                if self.index != u32::MAX { self.world = world; }
            }
            fn check_world(&self, world: *const RprWorld) -> Result {
                if self.index != u32::MAX && (self.world.is_null() || !std::ptr::eq(self.world, world)) {
                    return Err((RPR_INVALID_HANDLE, "handle belongs to a different world".into()));
                }
                Ok(())
            }
        }
    )*};
}
entity_world!(
    RprRigidBodyHandle,
    RprColliderHandle,
    RprImpulseJointHandle,
    RprMultibodyJointHandle,
    RprSoftBodyHandle
);
macro_rules! fields_world {
    ($ty:ty, $($field:ident),+ $(,)?) => {
        impl WorldHandles for $ty {
            fn attach_world(&mut self, world: *mut RprWorld) { $(self.$field.attach_world(world);)+ }
            fn check_world(&self, world: *const RprWorld) -> Result {
                $(self.$field.check_world(world)?;)+ Ok(())
            }
        }
    };
}
fields_world!(RprCharacterCollision, collider, hit);
fields_world!(RprCollisionEvent, collider1, collider2);
fields_world!(RprContactForceEvent, collider1, collider2);
fields_world!(RprContactPair, collider1, collider2);
fields_world!(RprIntersectionPair, collider1, collider2);
fields_world!(RprJointBodies, body1, body2);
fields_world!(RprOptionalParticleDestination, body);
fields_world!(RprOptionalRayHit, hit);
fields_world!(RprOptionalShapeCastHit, hit);
fields_world!(RprOptionalPointProjection, projection);
fields_world!(RprOptionalContactPair, pair);
fields_world!(RprOptionalIntersectionPair, pair);
fields_world!(RprParticleDestination, body);
fields_world!(RprPointProjection, collider);
fields_world!(RprQueryFilter, exclude_collider, exclude_rigid_body);
fields_world!(RprQueryOptions, filter);
fields_world!(RprRayHit, collider);
fields_world!(RprRayToi, collider);
fields_world!(RprShapeCastHit, collider);
fields_world!(RprSoftClusterSplit, soft_body, proxy);
fields_world!(RprSoftJointMove, joint, from, to);
fields_world!(RprSoftMeshInfo, collider);
#[cfg(feature = "dim3")]
fields_world!(RprWheelState, ground_object);

pub(crate) fn ffi_world_value<T: Default + WorldHandles>(
    world: *const RprWorld,
    call: impl FnOnce(*mut T) -> RprStatus,
) -> T {
    ffi_value(call).with_world(world.cast_mut())
}
/// copy_out writes no elements on failure, including BUFFER_TOO_SMALL.
pub(crate) unsafe fn ffi_world_array<T: WorldHandles>(
    world: *const RprWorld,
    buffer: *mut T,
    capacity: usize,
    call: impl FnOnce(*mut usize) -> RprStatus,
) -> usize {
    let count = ffi_value(call);
    if rpr_last_status() == RPR_OK && !buffer.is_null() && count <= capacity {
        for value in unsafe { std::slice::from_raw_parts_mut(buffer, count) } {
            value.attach_world(world.cast_mut());
        }
    }
    count
}
pub(crate) unsafe fn read_context_world(context: *const RprReadContext) -> *mut RprWorld {
    unsafe { get(context).map_or(std::ptr::null_mut(), |c| c.world) }
}
