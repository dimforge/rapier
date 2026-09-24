//! Complete set-and-handle element access. No borrowed element pointer escapes.
use crate::handle_access::forward;
use crate::*;
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_shape_identity(handle: RprColliderHandle) -> usize {
    let world = handle.world;
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_shape_identity(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_shape_identity(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_shape_identity(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_num_particles(handle: RprSoftBodyHandle) -> usize {
    let world = handle.world;
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_num_particles(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_topology_version(handle: RprSoftBodyHandle) -> u32 {
    let world = handle.world;
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_topology_version(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_mass(handle: RprSoftBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_mass(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_volume(handle: RprSoftBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_volume(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_rest_volume(handle: RprSoftBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_rest_volume(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_volume_factor(handle: RprSoftBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_volume_factor(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_center_of_mass(handle: RprSoftBodyHandle) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_center_of_mass(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_root_body(handle: RprSoftBodyHandle) -> RprRigidBodyHandle {
    let world = handle.world;
    ffi_world_value(world, |out: *mut RprRigidBodyHandle| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_root_body(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_is_enabled(handle: RprSoftBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_is_enabled(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_is_sleeping(handle: RprSoftBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_is_sleeping(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_particle_velocities(
    handle: RprSoftBodyHandle,
    buffer: *mut RprVector,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_particle_velocities(
                (element as *const SoftBody).cast(),
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_edges(
    handle: RprSoftBodyHandle,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_edges(
                (element as *const SoftBody).cast(),
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_cells(
    handle: RprSoftBodyHandle,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_cells(
                (element as *const SoftBody).cast(),
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_boundary(
    handle: RprSoftBodyHandle,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_boundary(
                (element as *const SoftBody).cast(),
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_pieces(
    handle: RprSoftBodyHandle,
    buffer: *mut RprSoftBodyHandle,
    capacity: usize,
) -> usize {
    let world = handle.world;
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                handle.check_world(world)?;
                let access = get(world)?.read()?;
                let raw = access.raw();

                let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

                let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
                forward(native_soft_body_pieces(
                    (element as *const SoftBody).cast(),
                    buffer,
                    capacity,
                    count,
                ))
            })
        })
    }
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_particle_velocity(
    handle: RprSoftBodyHandle,
    index: usize,
    value: RprVector,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_particle_velocity(
            (element as *mut SoftBody).cast(),
            index,
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_particle_kinematic_target(
    handle: RprSoftBodyHandle,
    index: usize,
    value: RprVector,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_particle_kinematic_target(
            (element as *mut SoftBody).cast(),
            index,
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_particle_pinned(
    handle: RprSoftBodyHandle,
    index: usize,
    value: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_particle_pinned(
            (element as *mut SoftBody).cast(),
            index,
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_apply_particle_impulse(
    handle: RprSoftBodyHandle,
    index: usize,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_apply_particle_impulse(
            (element as *mut SoftBody).cast(),
            index,
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_add_force(
    handle: RprSoftBodyHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_add_force(
            (element as *mut SoftBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_apply_impulse(
    handle: RprSoftBodyHandle,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_apply_impulse(
            (element as *mut SoftBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_reset_forces(
    handle: RprSoftBodyHandle,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_reset_forces(
            (element as *mut SoftBody).cast(),
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_enabled(
    handle: RprSoftBodyHandle,
    value: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_enabled(
            (element as *mut SoftBody).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_volume_factor(
    handle: RprSoftBodyHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_volume_factor(
            (element as *mut SoftBody).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_attach_particle(
    handle: RprSoftBodyHandle,
    index: usize,
    rigid_body: RprRigidBodyHandle,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        rigid_body.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();
        let bodies: *const RprRigidBodySet = std::ptr::addr_of!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_attach_particle(
            (element as *mut SoftBody).cast(),
            index,
            rigid_body,
            bodies,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_detach_particle(
    handle: RprSoftBodyHandle,
    index: usize,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_detach_particle(
            (element as *mut SoftBody).cast(),
            index,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_clusters(
    handle: RprSoftBodyHandle,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_clusters(
                (element as *const SoftBody).cast(),
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_cluster_proxy(
    handle: RprSoftBodyHandle,
    cluster: u32,
) -> RprRigidBodyHandle {
    let world = handle.world;
    ffi_world_value(world, |out: *mut RprRigidBodyHandle| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_cluster_proxy(
                (element as *const SoftBody).cast(),
                cluster,
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_cluster_particles(
    handle: RprSoftBodyHandle,
    cluster: u32,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_cluster_particles(
                (element as *const SoftBody).cast(),
                cluster,
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_cluster_pinned(
    handle: RprSoftBodyHandle,
    cluster: u32,
    value: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_cluster_pinned(
            (element as *mut SoftBody).cast(),
            cluster,
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_cluster_kinematic_target(
    handle: RprSoftBodyHandle,
    cluster: u32,
    value: RprPose,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_cluster_kinematic_target(
            (element as *mut SoftBody).cast(),
            cluster,
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_cluster_shape_matching_enabled(
    handle: RprSoftBodyHandle,
    cluster: u32,
    value: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_cluster_shape_matching_enabled(
            (element as *mut SoftBody).cast(),
            cluster,
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_cluster_stiffness_scale(
    handle: RprSoftBodyHandle,
    cluster: u32,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_cluster_stiffness_scale(
            (element as *mut SoftBody).cast(),
            cluster,
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_cluster_tear_resistance(
    handle: RprSoftBodyHandle,
    cluster: u32,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_cluster_tear_resistance(
            (element as *mut SoftBody).cast(),
            cluster,
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_meshes(
    handle: RprSoftBodyHandle,
    buffer: *mut RprSoftMeshInfo,
    capacity: usize,
) -> usize {
    let world = handle.world;
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                handle.check_world(world)?;
                let access = get(world)?.read()?;
                let raw = access.raw();

                let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

                let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
                forward(native_soft_body_meshes(
                    (element as *const SoftBody).cast(),
                    buffer,
                    capacity,
                    count,
                ))
            })
        })
    }
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_mesh_vertices_by_id(
    handle: RprSoftBodyHandle,
    id: RprSoftMeshId,
    buffer: *mut RprVector,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_mesh_vertices_by_id(
                (element as *const SoftBody).cast(),
                id,
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_mesh_indices_by_id(
    handle: RprSoftBodyHandle,
    id: RprSoftMeshId,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_mesh_indices_by_id(
                (element as *const SoftBody).cast(),
                id,
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_mesh_colliders(
    handle: RprSoftBodyHandle,
    buffer: *mut RprColliderHandle,
    capacity: usize,
) -> usize {
    let world = handle.world;
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                handle.check_world(world)?;
                let access = get(world)?.read()?;
                let raw = access.raw();

                let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

                let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
                forward(native_soft_body_mesh_colliders(
                    (element as *const SoftBody).cast(),
                    buffer,
                    capacity,
                    count,
                ))
            })
        })
    }
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_mesh_vertices(
    handle: RprSoftBodyHandle,
    collider: RprColliderHandle,
    buffer: *mut RprVector,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            collider.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_mesh_vertices(
                (element as *const SoftBody).cast(),
                collider,
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_mesh_indices(
    handle: RprSoftBodyHandle,
    collider: RprColliderHandle,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            collider.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_mesh_indices(
                (element as *const SoftBody).cast(),
                collider,
                buffer,
                capacity,
                count,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_mesh_arity(
    handle: RprSoftBodyHandle,
    collider: RprColliderHandle,
) -> usize {
    let world = handle.world;
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            collider.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_mesh_arity(
                (element as *const SoftBody).cast(),
                collider,
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_mesh_topology_version(
    handle: RprSoftBodyHandle,
    collider: RprColliderHandle,
) -> u32 {
    let world = handle.world;
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            collider.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_mesh_topology_version(
                (element as *const SoftBody).cast(),
                collider,
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[cfg(feature = "fem")]
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_solver(
    handle: RprSoftBodyHandle,
    solver: u32,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_solver(
            (element as *mut SoftBody).cast(),
            solver,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_cluster_shape_matching_target(
    handle: RprSoftBodyHandle,
    cluster: u32,
    target: *const RprPose,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_cluster_shape_matching_target(
            (element as *mut SoftBody).cast(),
            cluster,
            target,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_edge_tear_resistance(
    handle: RprSoftBodyHandle,
    index: usize,
    resistance: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_edge_tear_resistance(
            (element as *mut SoftBody).cast(),
            index,
            resistance,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_mesh_is_closed(
    handle: RprSoftBodyHandle,
    collider: RprColliderHandle,
) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            collider.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_mesh_is_closed(
                (element as *const SoftBody).cast(),
                collider,
                out,
            ))
        })
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_additional_mass_properties(
    handle: RprRigidBodyHandle,
    properties: RprMassProperties,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_additional_mass_properties(
            (element as *mut RigidBody).cast(),
            properties,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_recompute_mass_properties_from_colliders(
    handle: RprRigidBodyHandle,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();
        let colliders: *const RprColliderSet = std::ptr::addr_of!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_recompute_mass_properties_from_colliders(
            (element as *mut RigidBody).cast(),
            colliders,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_mass_properties(
    handle: RprColliderHandle,
    properties: RprMassProperties,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_mass_properties(
            (element as *mut Collider).cast(),
            properties,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_mass_properties(
    handle: RprColliderHandle,
) -> RprMassProperties {
    let world = handle.world;
    ffi_value(|out: *mut RprMassProperties| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_mass_properties(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_mass_properties(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprMassProperties,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_mass_properties(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_locked_axes(
    handle: RprRigidBodyHandle,
    axes: u8,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_locked_axes(
            (element as *mut RigidBody).cast(),
            axes,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_locked_axes(handle: RprRigidBodyHandle) -> u8 {
    let world = handle.world;
    ffi_value(|out: *mut u8| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_locked_axes(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_locked_axes(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut u8,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_locked_axes(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_is_voxels(handle: RprColliderHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_is_voxels(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_is_voxels(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_is_voxels(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_voxel_at_flat_id(
    handle: RprColliderHandle,
    id: u32,
) -> RprVoxelQuery {
    let world = handle.world;
    ffi_value(|result: *mut RprVoxelQuery| {
        let key = unsafe { std::ptr::addr_of_mut!((*result).key) };
        let center = unsafe { std::ptr::addr_of_mut!((*result).center) };
        let size = unsafe { std::ptr::addr_of_mut!((*result).size) };
        let found = unsafe { std::ptr::addr_of_mut!((*result).found) };

        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_voxel_at_flat_id(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                id,
                key,
                center,
                size,
                found,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_voxel_at_flat_id(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    id: u32,
    key: *mut RprVoxelKey,
    center: *mut RprVector,
    size: *mut RprVector,
    found: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_voxel_at_flat_id(
            (element as *const Collider).cast(),
            id,
            key,
            center,
            size,
            found,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_voxel(
    handle: RprColliderHandle,
    key: RprVoxelKey,
    filled: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_voxel(
            (element as *mut Collider).cast(),
            key,
            filled,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_next_position(handle: RprRigidBodyHandle) -> RprPose {
    let world = handle.world;
    ffi_value(|out: *mut RprPose| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_next_position(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_next_position(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_next_position(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_rotation(handle: RprRigidBodyHandle) -> RprRotation {
    let world = handle.world;
    ffi_value(|out: *mut RprRotation| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_rotation(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_rotation(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprRotation,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_rotation(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_center_of_mass(handle: RprRigidBodyHandle) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_center_of_mass(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_center_of_mass(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_center_of_mass(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_local_center_of_mass(
    handle: RprRigidBodyHandle,
) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_local_center_of_mass(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_local_center_of_mass(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_local_center_of_mass(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_user_force(handle: RprRigidBodyHandle) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_user_force(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_user_force(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_user_force(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_user_torque(handle: RprRigidBodyHandle) -> RprAngVector {
    let world = handle.world;
    ffi_value(|out: *mut RprAngVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_user_torque(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_user_torque(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprAngVector,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_user_torque(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_body_type(handle: RprRigidBodyHandle) -> u32 {
    let world = handle.world;
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_body_type(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_body_type(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut u32,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_body_type(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_mass(handle: RprRigidBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_mass(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_mass(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_mass(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_gravity_scale(handle: RprRigidBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_gravity_scale(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_gravity_scale(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_gravity_scale(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_linear_damping(handle: RprRigidBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_linear_damping(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_linear_damping(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_linear_damping(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_angular_damping(handle: RprRigidBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_angular_damping(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_angular_damping(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_angular_damping(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_kinetic_energy(handle: RprRigidBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_kinetic_energy(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_kinetic_energy(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_kinetic_energy(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_soft_ccd_prediction(handle: RprRigidBodyHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_soft_ccd_prediction(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_soft_ccd_prediction(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_soft_ccd_prediction(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_is_ccd_enabled(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_is_ccd_enabled(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_is_ccd_enabled(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_is_ccd_enabled(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_is_dynamic(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_is_dynamic(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_is_dynamic(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_is_dynamic(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_soft_body(handle: RprRigidBodyHandle) -> RprSoftBodyHandle {
    let world = handle.world;
    ffi_world_value(world, |out: *mut RprSoftBodyHandle| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_soft_body(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_soft_body(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprSoftBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_soft_body(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_is_soft_frame(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_is_soft_frame(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_is_soft_frame(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_is_soft_frame(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_is_fixed(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_is_fixed(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_is_fixed(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_is_fixed(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_is_kinematic(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_is_kinematic(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_is_kinematic(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_is_kinematic(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_is_moving(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_is_moving(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_is_moving(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_is_moving(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_is_ccd_active(handle: RprRigidBodyHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_is_ccd_active(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_is_ccd_active(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_is_ccd_active(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_rotation(
    handle: RprRigidBodyHandle,
    value: RprRotation,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_rotation(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_body_type(
    handle: RprRigidBodyHandle,
    value: u32,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_body_type(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_next_kinematic_rotation(
    handle: RprRigidBodyHandle,
    value: RprRotation,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_next_kinematic_rotation(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_additional_mass(
    handle: RprRigidBodyHandle,
    value: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_additional_mass(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_soft_ccd_prediction(
    handle: RprRigidBodyHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_soft_ccd_prediction(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_ccd_enabled(
    handle: RprRigidBodyHandle,
    value: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_ccd_enabled(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_translations_locked(
    handle: RprRigidBodyHandle,
    value: RprBool,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_translations_locked(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_rotations_locked(
    handle: RprRigidBodyHandle,
    value: RprBool,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_rotations_locked(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_dominance_group(
    handle: RprRigidBodyHandle,
    value: i8,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_dominance_group(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_additional_solver_iterations(
    handle: RprRigidBodyHandle,
    value: usize,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_additional_solver_iterations(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_additional_pgs_iterations(
    handle: RprRigidBodyHandle,
    value: usize,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_additional_pgs_iterations(
            (element as *mut RigidBody).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_add_torque(
    handle: RprRigidBodyHandle,
    value: RprAngVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_add_torque(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_apply_torque_impulse(
    handle: RprRigidBodyHandle,
    value: RprAngVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_apply_torque_impulse(
            (element as *mut RigidBody).cast(),
            value,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_add_force_at_point(
    handle: RprRigidBodyHandle,
    value: RprVector,
    point: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_add_force_at_point(
            (element as *mut RigidBody).cast(),
            value,
            point,
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_reset_torques(
    handle: RprRigidBodyHandle,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_reset_torques(
            (element as *mut RigidBody).cast(),
            wake_up,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_velocity_at_point(
    handle: RprRigidBodyHandle,
    point: RprVector,
) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_velocity_at_point(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                point,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_rigid_body_set_get_velocity_at_point(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    point: RprVector,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_velocity_at_point(
            (element as *const RigidBody).cast(),
            point,
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_colliders(
    handle: RprRigidBodyHandle,
    buffer: *mut RprColliderHandle,
    capacity: usize,
) -> usize {
    let world = handle.world;
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                handle.check_world(world)?;
                let access = get(world)?.read()?;
                let raw = access.raw();

                crate::handle_access::forward(native_rigid_body_set_get_colliders(
                    std::ptr::addr_of!((*raw).0.bodies).cast(),
                    handle,
                    buffer,
                    capacity,
                    count,
                ))
            })
        })
    }
}

pub(crate) unsafe fn native_rigid_body_set_get_colliders(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    buffer: *mut RprColliderHandle,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_colliders(
            (element as *const RigidBody).cast(),
            buffer,
            capacity,
            count,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[cfg(feature = "dim3")]
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_gyroscopic_forces_enabled(
    handle: RprRigidBodyHandle,
) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_rigid_body_set_get_gyroscopic_forces_enabled(
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                handle,
                out,
            ))
        })
    })
}
#[cfg(feature = "dim3")]
pub(crate) unsafe fn native_rigid_body_set_get_gyroscopic_forces_enabled(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_rigid_body_gyroscopic_forces_enabled(
            (element as *const RigidBody).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[cfg(feature = "dim3")]
#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_set_gyroscopic_forces_enabled(
    handle: RprRigidBodyHandle,
    enabled: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            element.soft_body().is_none(),
            "mutate soft-body proxies through the soft-body API",
        )?;
        forward(native_rigid_body_set_gyroscopic_forces_enabled(
            (element as *mut RigidBody).cast(),
            enabled,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_density(
    handle: RprColliderHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_density(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_mass(
    handle: RprColliderHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_mass(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_enabled(
    handle: RprColliderHandle,
    value: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_enabled(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_solver_groups(
    handle: RprColliderHandle,
    value: RprInteractionGroups,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_solver_groups(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_friction_combine_rule(
    handle: RprColliderHandle,
    value: u32,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_friction_combine_rule(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_restitution_combine_rule(
    handle: RprColliderHandle,
    value: u32,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_restitution_combine_rule(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_contact_skin(
    handle: RprColliderHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_contact_skin(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_contact_force_event_threshold(
    handle: RprColliderHandle,
    value: RprReal,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_contact_force_event_threshold(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_active_events(
    handle: RprColliderHandle,
    value: u32,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_active_events(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_active_hooks(
    handle: RprColliderHandle,
    value: u32,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_active_hooks(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_active_collision_types(
    handle: RprColliderHandle,
    value: u16,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_active_collision_types(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_rotation(handle: RprColliderHandle) -> RprRotation {
    let world = handle.world;
    ffi_value(|out: *mut RprRotation| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_rotation(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_rotation(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprRotation,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_rotation(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_collision_groups(
    handle: RprColliderHandle,
) -> RprInteractionGroups {
    let world = handle.world;
    ffi_value(|out: *mut RprInteractionGroups| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_collision_groups(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_collision_groups(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprInteractionGroups,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_collision_groups(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_solver_groups(
    handle: RprColliderHandle,
) -> RprInteractionGroups {
    let world = handle.world;
    ffi_value(|out: *mut RprInteractionGroups| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_solver_groups(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_solver_groups(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprInteractionGroups,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_solver_groups(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_user_data(handle: RprColliderHandle) -> RprUserData {
    let world = handle.world;
    ffi_value(|out: *mut RprUserData| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_user_data(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_user_data(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprUserData,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_user_data(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_active_events(handle: RprColliderHandle) -> u32 {
    let world = handle.world;
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_active_events(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_active_events(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut u32,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_active_events(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_mass(handle: RprColliderHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_mass(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_mass(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_mass(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_density(handle: RprColliderHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_density(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_density(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_density(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_volume(handle: RprColliderHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_volume(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_volume(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_volume(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_contact_skin(handle: RprColliderHandle) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_contact_skin(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_contact_skin(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_contact_skin(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_contact_force_event_threshold(
    handle: RprColliderHandle,
) -> RprReal {
    let world = handle.world;
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_contact_force_event_threshold(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_contact_force_event_threshold(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_contact_force_event_threshold(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_is_enabled(handle: RprColliderHandle) -> RprBool {
    let world = handle.world;
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_is_enabled(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_is_enabled(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_is_enabled(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_compute_aabb(handle: RprColliderHandle) -> RprAabb {
    let world = handle.world;
    ffi_value(|out: *mut RprAabb| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_compute_aabb(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_compute_aabb(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprAabb,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_compute_aabb(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
/// Returns an owned shape wrapper sharing the geometry. Release it with FreeSharedShape.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_clone_shape(
    handle: RprColliderHandle,
) -> *mut RprSharedShape {
    let world = handle.world;
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_collider_set_get_shared_shape(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_collider_set_get_shared_shape(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_shared_shape(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_shape(
    handle: RprColliderHandle,
    shape: *const RprSharedShape,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_shape(
            (element as *mut Collider).cast(),
            shape,
        ))
    })
}

/// Resolves the generational handle for this call; rejects stale handles.
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_set_position_wrt_parent(
    handle: RprColliderHandle,
    value: RprPose,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_set_position_wrt_parent(
            (element as *mut Collider).cast(),
            value,
        ))
    })
}

#[rapier_export(rigid_body)]
pub unsafe extern "C" fn rpr_rigid_body_validate_handle(handle: RprRigidBodyHandle) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.read()?;
        let raw = access.raw();

        crate::handle_access::forward(native_rigid_body_set_validate_handle(
            std::ptr::addr_of!((*raw).0.bodies).cast(),
            handle,
        ))
    })
}

pub(crate) unsafe fn native_rigid_body_set_validate_handle(
    set: *const RprRigidBodySet,
    handle: RprRigidBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        Ok(())
    })
}
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_validate_handle(handle: RprColliderHandle) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.read()?;
        let raw = access.raw();

        crate::handle_access::forward(native_collider_set_validate_handle(
            std::ptr::addr_of!((*raw).0.colliders).cast(),
            handle,
        ))
    })
}

pub(crate) unsafe fn native_collider_set_validate_handle(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
) -> RprStatus {
    ffi(|| unsafe {
        get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        Ok(())
    })
}
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_validate_handle(handle: RprSoftBodyHandle) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.read()?;
        let raw = access.raw();

        let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

        get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        Ok(())
    })
}
