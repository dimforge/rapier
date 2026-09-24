//! Owning C world and dynamic borrow checks at the foreign-language boundary.
use crate::*;
use std::{
    cell::UnsafeCell,
    sync::atomic::{AtomicUsize, Ordering},
};

const WRITER: usize = usize::MAX;
/// Sole owner of simulation state. Handles belong to the world that created them.
/// Ordinary reads may overlap. A mutation or step requires exclusive access.
/// Destruction must be externally synchronized with all users of this pointer.
/// @ingroup worlds
pub struct RprWorld {
    access: AtomicUsize,
    data: UnsafeCell<RprPhysicsWorld>,
}
// Access to simulation data is protected by the atomic shared/exclusive gate.
unsafe impl Send for RprWorld {}
unsafe impl Sync for RprWorld {}
impl RprWorld {
    pub(crate) fn new(data: RprPhysicsWorld) -> Self {
        Self {
            access: AtomicUsize::new(0),
            data: UnsafeCell::new(data),
        }
    }
    pub(crate) fn read(&self) -> Result<WorldRead<'_>> {
        self.access
            .fetch_update(Ordering::Acquire, Ordering::Relaxed, |n| {
                (n < WRITER - 1).then(|| n + 1)
            })
            .map_err(|_| busy())?;
        Ok(WorldRead(self))
    }
    pub(crate) fn write(&self) -> Result<WorldWrite<'_>> {
        self.access
            .compare_exchange(0, WRITER, Ordering::Acquire, Ordering::Relaxed)
            .map_err(|_| busy())?;
        Ok(WorldWrite(self))
    }
}
fn busy() -> (RprStatus, String) {
    (RPR_WORLD_BUSY, "world is already borrowed; use callback read access or defer the mutation until the operation returns".into())
}
pub(crate) struct WorldRead<'a>(&'a RprWorld);
impl WorldRead<'_> {
    pub(crate) fn raw(&self) -> *const RprPhysicsWorld {
        self.0.data.get()
    }
}
impl Drop for WorldRead<'_> {
    fn drop(&mut self) {
        self.0.access.fetch_sub(1, Ordering::Release);
    }
}
pub(crate) struct WorldWrite<'a>(&'a RprWorld);
impl WorldWrite<'_> {
    pub(crate) fn raw(&self) -> *mut RprPhysicsWorld {
        self.0.data.get()
    }
}
impl Drop for WorldWrite<'_> {
    fn drop(&mut self) {
        self.0.access.store(0, Ordering::Release);
    }
}
/// Create an owned world. Release it with FreeWorld.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_new_world() -> *mut RprWorld {
    ffi_value(|out: *mut *mut RprWorld| {
        ffi(|| unsafe {
            out_ptr(out)?;
            output(
                out,
                Box::into_raw(Box::new(RprWorld::new(
                    RprPhysicsWorld(PhysicsWorld::new()),
                ))),
            )
        })
    })
}
/// Free a world. NULL is allowed. Rejects destruction from an active callback.
/// The caller must prevent other threads from starting calls during destruction.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_free_world(world: *mut RprWorld) -> RprStatus {
    ffi(|| unsafe {
        if world.is_null() {
            return Ok(());
        }
        let access = get(world)?.write()?;
        // End the shared reference to the wrapper, keeping the gate closed until destruction.
        std::mem::forget(access);
        drop(Box::from_raw(world));
        Ok(())
    })
}

/// Callback-scoped read access to bodies and colliders. Never retain or free it.
/// Only the Read* functions accept this context; it cannot mutate the world.
/// @ingroup callbacks
pub struct RprReadContext {
    pub(crate) world: *mut RprWorld,
    pub(crate) bodies: *const RprRigidBodySet,
    pub(crate) colliders: *const RprColliderSet,
}
impl RprReadContext {
    pub(crate) fn new(
        world: *mut RprWorld,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) -> Self {
        Self {
            world,
            bodies: (bodies as *const RigidBodySet).cast(),
            colliders: (colliders as *const ColliderSet).cast(),
        }
    }
}
