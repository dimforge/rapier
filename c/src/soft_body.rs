use crate::*;

#[rapier_export]
pub unsafe extern "C" fn rpr_remove_soft_body(handle: RprSoftBodyHandle) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();
        let islands: *mut RprIslandManager = std::ptr::addr_of_mut!((*raw).0.islands).cast();
        let bodies: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();
        let colliders: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();
        let impulse_joints: *mut RprImpulseJointSet =
            std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();
        let multibody_joints: *mut RprMultibodyJointSet =
            std::ptr::addr_of_mut!((*raw).0.multibody_joints).cast();

        get_mut(set)?
            .0
            .remove(
                handle.raw(),
                &mut get_mut(islands)?.0,
                &mut get_mut(bodies)?.0,
                &mut get_mut(colliders)?.0,
                &mut get_mut(impulse_joints)?.0,
                &mut get_mut(multibody_joints)?.0,
            )
            .ok_or_else(missing)?;
        Ok(())
    })
}

pub(crate) unsafe fn native_soft_body_num_particles(
    body: *const RprSoftBody,
    out: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.num_particles())
    })
}
pub(crate) unsafe fn native_soft_body_topology_version(
    body: *const RprSoftBody,
    out: *mut u32,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.topology_version())
    })
}
pub(crate) unsafe fn native_soft_body_mass(
    body: *const RprSoftBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.mass())
    })
}
pub(crate) unsafe fn native_soft_body_volume(
    body: *const RprSoftBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.volume())
    })
}
pub(crate) unsafe fn native_soft_body_rest_volume(
    body: *const RprSoftBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.rest_volume())
    })
}
pub(crate) unsafe fn native_soft_body_volume_factor(
    body: *const RprSoftBody,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.volume_factor())
    })
}
pub(crate) unsafe fn native_soft_body_center_of_mass(
    body: *const RprSoftBody,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.center_of_mass().into())
    })
}
pub(crate) unsafe fn native_soft_body_root_body(
    body: *const RprSoftBody,
    out: *mut RprRigidBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.root_body().into())
    })
}
pub(crate) unsafe fn native_soft_body_is_enabled(
    body: *const RprSoftBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.is_enabled() as u32)
    })
}
pub(crate) unsafe fn native_soft_body_is_sleeping(
    body: *const RprSoftBody,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        output(out, b.is_sleeping() as u32)
    })
}
pub(crate) unsafe fn native_soft_body_particle_positions(
    body: *const RprSoftBody,
    buffer: *mut RprVector,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let values: Vec<_> = get(body)?.0.particle_positions().map(Into::into).collect();
        copy_out(&values, buffer, capacity, count)
    })
}
pub(crate) unsafe fn native_soft_body_particle_velocities(
    body: *const RprSoftBody,
    buffer: *mut RprVector,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let values: Vec<_> = get(body)?.0.particle_velocities().map(Into::into).collect();
        copy_out(&values, buffer, capacity, count)
    })
}
/// Flat particle indices; count and capacity are numbers of uint32_t elements.
pub(crate) unsafe fn native_soft_body_edges(
    body: *const RprSoftBody,
    buffer: *mut u32,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        let values: Vec<_> = b.edges().iter().flat_map(|v| v.vertices).collect();
        copy_out(&values, buffer, capacity, count)
    })
}
/// Flat particle indices; count and capacity are numbers of uint32_t elements.
pub(crate) unsafe fn native_soft_body_cells(
    body: *const RprSoftBody,
    buffer: *mut u32,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        let values: Vec<_> = b.cells().iter().flat_map(|v| v.vertices).collect();
        copy_out(&values, buffer, capacity, count)
    })
}
/// Flat particle indices; count and capacity are numbers of uint32_t elements.
pub(crate) unsafe fn native_soft_body_boundary(
    body: *const RprSoftBody,
    buffer: *mut u32,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &get(body)?.0;
        let values: Vec<_> = b.boundary().iter().flatten().copied().collect();
        copy_out(&values, buffer, capacity, count)
    })
}
pub(crate) unsafe fn native_soft_body_pieces(
    body: *const RprSoftBody,
    buffer: *mut RprSoftBodyHandle,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let values: Vec<_> = get(body)?
            .0
            .pieces()
            .iter()
            .copied()
            .map(Into::into)
            .collect();
        copy_out(&values, buffer, capacity, count)
    })
}
pub(crate) unsafe fn native_soft_body_set_particle_position(
    body: *mut RprSoftBody,
    index: usize,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let b = &mut get_mut(body)?.0;
        ensure(index < b.num_particles(), "particle index out of range")?;
        b.set_particle_position(index, value);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_set_particle_velocity(
    body: *mut RprSoftBody,
    index: usize,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let b = &mut get_mut(body)?.0;
        ensure(index < b.num_particles(), "particle index out of range")?;
        b.set_particle_velocity(index, value);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_set_particle_kinematic_target(
    body: *mut RprSoftBody,
    index: usize,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let b = &mut get_mut(body)?.0;
        ensure(index < b.num_particles(), "particle index out of range")?;
        b.set_particle_kinematic_target(index, value);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_set_particle_pinned(
    body: *mut RprSoftBody,
    index: usize,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let b = &mut get_mut(body)?.0;
        ensure(index < b.num_particles(), "particle index out of range")?;
        b.set_particle_pinned(index, value);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_add_particle_force(
    body: *mut RprSoftBody,
    index: usize,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        let b = &mut get_mut(body)?.0;
        ensure(index < b.num_particles(), "particle index out of range")?;
        b.add_particle_force(index, value, wake_up);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_apply_particle_impulse(
    body: *mut RprSoftBody,
    index: usize,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        let b = &mut get_mut(body)?.0;
        ensure(index < b.num_particles(), "particle index out of range")?;
        b.apply_particle_impulse(index, value, wake_up);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_add_force(
    body: *mut RprSoftBody,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        get_mut(body)?.0.add_force(value, wake_up);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_apply_impulse(
    body: *mut RprSoftBody,
    value: RprVector,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let wake_up = boolean(wake_up)?;
        get_mut(body)?.0.apply_impulse(value, wake_up);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_reset_forces(
    body: *mut RprSoftBody,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let wake_up = boolean(wake_up)?;
        get_mut(body)?.0.reset_forces(wake_up);
        Ok(())
    })
}
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_wake_up(handle: RprSoftBodyHandle) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        get_mut(set)?
            .0
            .get_mut(handle.raw())
            .ok_or_else(missing)?
            .wake_up();
        Ok(())
    })
}

pub(crate) unsafe fn native_soft_body_set_enabled(
    body: *mut RprSoftBody,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        get_mut(body)?.0.set_enabled(value);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_set_volume_factor(
    body: *mut RprSoftBody,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        positive(value)?;
        get_mut(body)?.0.set_volume_factor(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_soft_body_attach_particle(
    body: *mut RprSoftBody,
    index: usize,
    rigid_body: RprRigidBodyHandle,
    bodies: *const RprRigidBodySet,
) -> RprStatus {
    ffi(|| unsafe {
        let bodies = get(bodies)?;
        let rb = bodies.0.get(rigid_body.raw()).ok_or_else(missing)?;
        ensure(
            rb.soft_body().is_none(),
            "attachment target must be a rigid body",
        )?;
        let b = &mut get_mut(body)?.0;
        ensure(index < b.num_particles(), "particle index out of range")?;
        b.attach_particle(index, rigid_body.raw(), &bodies.0);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_detach_particle(
    body: *mut RprSoftBody,
    index: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let b = &mut get_mut(body)?.0;
        ensure(index < b.num_particles(), "particle index out of range")?;
        b.detach_particle(index);
        Ok(())
    })
}

#[rapier_export]
pub unsafe extern "C" fn rpr_free_soft_body_tear_event(
    event: *mut RprSoftBodyTearEvent,
) -> RprStatus {
    ffi(|| unsafe {
        if !event.is_null() {
            get(event)?;
            drop(Box::from_raw(event));
        }
        Ok(())
    })
}
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_soft_body(
    event: *const RprSoftBodyTearEvent,
) -> RprSoftBodyHandle {
    ffi_world_value(
        unsafe { get(event).map_or(std::ptr::null_mut(), |e| e.1) },
        |out: *mut RprSoftBodyHandle| {
            ffi(|| unsafe { output(out, get(event)?.0.soft_body.into()) })
        },
    )
}
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_bodies(
    event: *const RprSoftBodyTearEvent,
    buffer: *mut RprSoftBodyHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(
            get(event).map_or(std::ptr::null_mut(), |e| e.1),
            buffer,
            capacity,
            |count: *mut usize| {
                ffi(|| {
                    let v: Vec<_> = get(event)?.0.bodies().map(Into::into).collect();
                    copy_out(&v, buffer, capacity, count)
                })
            },
        )
    }
}
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_particle_destination(
    event: *const RprSoftBodyTearEvent,
    particle: u32,
) -> RprParticleDestination {
    ffi_world_value(
        unsafe { get(event).map_or(std::ptr::null_mut(), |e| e.1) },
        |result: *mut RprParticleDestination| {
            let body = unsafe { std::ptr::addr_of_mut!((*result).body) };
            let index = unsafe { std::ptr::addr_of_mut!((*result).index) };

            ffi(|| unsafe {
                out_ptr(body)?;
                out_ptr(index)?;
                let (b, i) = get(event)?
                    .0
                    .particle_destination(particle)
                    .ok_or((RPR_NOT_FOUND, "particle has no destination".into()))?;
                output(body, b.into())?;
                output(index, i)
            })
        },
    )
}
/// Flat indices; element arity follows the corresponding Rust event field.
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_torn_edges(
    event: *const RprSoftBodyTearEvent,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let e = &get(event)?.0;
            let v: Vec<_> = e.torn_edges.iter().flatten().copied().collect();
            copy_out(&v, buffer, capacity, count)
        })
    })
}
/// Flat indices; element arity follows the corresponding Rust event field.
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_torn_cells(
    event: *const RprSoftBodyTearEvent,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let e = &get(event)?.0;
            let v: Vec<_> = e.torn_cells.iter().flatten().copied().collect();
            copy_out(&v, buffer, capacity, count)
        })
    })
}
/// Flat indices; element arity follows the corresponding Rust event field.
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_removed_edges(
    event: *const RprSoftBodyTearEvent,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let e = &get(event)?.0;
            let v: Vec<_> = e.removed_edges.iter().flatten().copied().collect();
            copy_out(&v, buffer, capacity, count)
        })
    })
}
/// Flat indices; element arity follows the corresponding Rust event field.
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_split_particles(
    event: *const RprSoftBodyTearEvent,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let e = &get(event)?.0;
            let v: Vec<_> = e
                .split_particles
                .iter()
                .flat_map(|&(a, b)| [a, b])
                .collect();
            copy_out(&v, buffer, capacity, count)
        })
    })
}
/// Flat indices; element arity follows the corresponding Rust event field.
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_inserted_particles(
    event: *const RprSoftBodyTearEvent,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let e = &get(event)?.0;
            let v: Vec<_> = e.inserted_particles.clone();
            copy_out(&v, buffer, capacity, count)
        })
    })
}
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_piece_particles(
    event: *const RprSoftBodyTearEvent,
    piece_index: usize,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let p = get(event)?
                .0
                .pieces
                .get(piece_index)
                .ok_or_else(|| invalid("piece index out of range"))?;
            copy_out(&p.particles, buffer, capacity, count)
        })
    })
}
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprSoftClusterSplit {
    pub source_cluster: u32,
    pub soft_body: RprSoftBodyHandle,
    pub cluster: u32,
    pub proxy: RprRigidBodyHandle,
    pub keeps_proxy: RprBool,
}
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprSoftJointMove {
    pub joint: RprImpulseJointHandle,
    pub from: RprRigidBodyHandle,
    pub to: RprRigidBodyHandle,
}
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_clusters(
    event: *const RprSoftBodyTearEvent,
    buffer: *mut RprSoftClusterSplit,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(
            get(event).map_or(std::ptr::null_mut(), |e| e.1),
            buffer,
            capacity,
            |count: *mut usize| {
                ffi(|| {
                    let v: Vec<_> = get(event)?
                        .0
                        .clusters
                        .iter()
                        .map(|e| RprSoftClusterSplit {
                            source_cluster: e.source_cluster,
                            soft_body: e.soft_body.into(),
                            cluster: e.cluster,
                            proxy: e.proxy.into(),
                            keeps_proxy: e.keeps_proxy as u32,
                        })
                        .collect();
                    copy_out(&v, buffer, capacity, count)
                })
            },
        )
    }
}
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_moved_joints(
    event: *const RprSoftBodyTearEvent,
    buffer: *mut RprSoftJointMove,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(
            get(event).map_or(std::ptr::null_mut(), |e| e.1),
            buffer,
            capacity,
            |count: *mut usize| {
                ffi(|| {
                    let v: Vec<_> = get(event)?
                        .0
                        .moved_joints
                        .iter()
                        .map(|e| RprSoftJointMove {
                            joint: e.joint.into(),
                            from: e.from.into(),
                            to: e.to.into(),
                        })
                        .collect();
                    copy_out(&v, buffer, capacity, count)
                })
            },
        )
    }
}

#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_tear(
    handle: RprSoftBodyHandle,
    edges: *const u32,
    edge_count: usize,
    cells: *const u32,
    cell_count: usize,
) -> *mut RprSoftBodyTearEvent {
    let world = handle.world;
    ffi_value(|out: *mut *mut RprSoftBodyTearEvent| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.write()?;
            let raw = access.raw();

            let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();
            let islands: *mut RprIslandManager = std::ptr::addr_of_mut!((*raw).0.islands).cast();
            let bodies: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();
            let colliders: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();
            let impulse_joints: *mut RprImpulseJointSet =
                std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();
            let multibody_joints: *mut RprMultibodyJointSet =
                std::ptr::addr_of_mut!((*raw).0.multibody_joints).cast();

            out_ptr(out)?;
            let edges = input(edges, edge_count)?;
            let cells = input(cells, cell_count)?;
            let set = get_mut(set)?;
            let b = set.0.get(handle.raw()).ok_or_else(missing)?;
            ensure(
                edges.iter().all(|&i| (i as usize) < b.edges().len())
                    && cells.iter().all(|&i| (i as usize) < b.cells().len()),
                "element index out of range",
            )?;
            let e = set.0.tear(
                handle.raw(),
                edges,
                cells,
                &mut get_mut(islands)?.0,
                &mut get_mut(bodies)?.0,
                &mut get_mut(colliders)?.0,
                &mut get_mut(impulse_joints)?.0,
                &mut get_mut(multibody_joints)?.0,
            );
            output(
                out,
                e.map(|e| Box::into_raw(Box::new(RprSoftBodyTearEvent(e, handle.world))))
                    .unwrap_or(std::ptr::null_mut()),
            )
        })
    })
}

#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_add_cluster(
    handle: RprSoftBodyHandle,
    particles: *const u32,
    count: usize,
) -> u32 {
    let world = handle.world;
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.write()?;
            let raw = access.raw();

            let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();
            let bodies: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();
            let colliders: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();

            out_ptr(out)?;
            let p = input(particles, count)?;
            let s = get_mut(set)?;
            let b = s.0.get(handle.raw()).ok_or_else(missing)?;
            ensure(
                !p.is_empty() && p.iter().all(|&i| (i as usize) < b.num_particles()),
                "invalid cluster particles",
            )?;
            let id =
                s.0.add_cluster(
                    handle.raw(),
                    p,
                    &mut get_mut(bodies)?.0,
                    &mut get_mut(colliders)?.0,
                )
                .ok_or_else(|| invalid("cluster could not be created"))?;
            output(out, id)
        })
    })
}

#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_remove_cluster(
    handle: RprSoftBodyHandle,
    cluster: u32,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();
        let islands: *mut RprIslandManager = std::ptr::addr_of_mut!((*raw).0.islands).cast();
        let bodies: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();
        let colliders: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();
        let impulse_joints: *mut RprImpulseJointSet =
            std::ptr::addr_of_mut!((*raw).0.impulse_joints).cast();
        let multibody_joints: *mut RprMultibodyJointSet =
            std::ptr::addr_of_mut!((*raw).0.multibody_joints).cast();

        let s = get_mut(set)?;
        s.0.get(handle.raw()).ok_or_else(missing)?;
        s.0.remove_cluster(
            handle.raw(),
            cluster,
            &mut get_mut(islands)?.0,
            &mut get_mut(bodies)?.0,
            &mut get_mut(colliders)?.0,
            &mut get_mut(impulse_joints)?.0,
            &mut get_mut(multibody_joints)?.0,
        )
        .ok_or_else(|| invalid("invalid cluster"))?;
        Ok(())
    })
}

pub(crate) unsafe fn native_soft_body_clusters(
    body: *const RprSoftBody,
    buffer: *mut u32,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let v: Vec<_> = get(body)?.0.live_clusters().map(|(i, _)| i).collect();
        copy_out(&v, buffer, capacity, count)
    })
}
pub(crate) unsafe fn native_soft_body_cluster_proxy(
    body: *const RprSoftBody,
    cluster: u32,
    out: *mut RprRigidBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        let h = get(body)?
            .0
            .cluster_proxy(cluster)
            .ok_or_else(|| invalid("invalid cluster"))?;
        output(out, h.into())
    })
}
pub(crate) unsafe fn native_soft_body_cluster_particles(
    body: *const RprSoftBody,
    cluster: u32,
    buffer: *mut u32,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let c = get(body)?
            .0
            .cluster(cluster)
            .filter(|c| c.is_live())
            .ok_or_else(|| invalid("invalid cluster"))?;
        copy_out(c.particles(), buffer, capacity, count)
    })
}
pub(crate) unsafe fn native_soft_body_set_cluster_pinned(
    body: *mut RprSoftBody,
    cluster: u32,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let b = &mut get_mut(body)?.0;
        b.cluster(cluster)
            .filter(|c| c.is_live())
            .ok_or_else(|| invalid("invalid cluster"))?;
        b.set_cluster_pinned(cluster, value);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_set_cluster_kinematic_target(
    body: *mut RprSoftBody,
    cluster: u32,
    value: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let b = &mut get_mut(body)?.0;
        b.cluster(cluster)
            .filter(|c| c.is_live())
            .ok_or_else(|| invalid("invalid cluster"))?;
        b.set_cluster_kinematic_target(cluster, value);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_set_cluster_shape_matching_enabled(
    body: *mut RprSoftBody,
    cluster: u32,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let b = &mut get_mut(body)?.0;
        b.cluster(cluster)
            .filter(|c| c.is_live())
            .ok_or_else(|| invalid("invalid cluster"))?;
        b.enable_cluster_shape_matching(cluster, value);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_set_cluster_stiffness_scale(
    body: *mut RprSoftBody,
    cluster: u32,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let b = &mut get_mut(body)?.0;
        b.cluster(cluster)
            .filter(|c| c.is_live())
            .ok_or_else(|| invalid("invalid cluster"))?;
        b.set_cluster_stiffness_scale(cluster, value);
        Ok(())
    })
}
pub(crate) unsafe fn native_soft_body_set_cluster_tear_resistance(
    body: *mut RprSoftBody,
    cluster: u32,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let b = &mut get_mut(body)?.0;
        b.cluster(cluster)
            .filter(|c| c.is_live())
            .ok_or_else(|| invalid("invalid cluster"))?;
        b.set_cluster_tear_resistance(cluster, value);
        Ok(())
    })
}

/// Stable identity of a live mesh within one soft body; matches Rapier's SoftMeshId.
#[repr(C)]
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct RprSoftMeshId {
    pub cluster: u32,
    pub mesh: u32,
}
impl RprSoftMeshId {
    fn raw(self) -> SoftMeshId {
        SoftMeshId {
            cluster: self.cluster,
            mesh: self.mesh,
        }
    }
}
/// Mesh identity and rendering metadata. A render-only mesh has an invalid collider handle.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftMeshInfo {
    pub id: RprSoftMeshId,
    pub collider: RprColliderHandle,
    pub arity: usize,
    pub is_skinned: RprBool,
    pub collision_enabled: RprBool,
}
/// Enumerates all live meshes, including skins without a physics collider.
pub(crate) unsafe fn native_soft_body_meshes(
    body: *const RprSoftBody,
    buffer: *mut RprSoftMeshInfo,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let meshes: Vec<_> = get(body)?
            .0
            .meshes()
            .map(|m| RprSoftMeshInfo {
                id: RprSoftMeshId {
                    cluster: m.id().cluster,
                    mesh: m.id().mesh,
                },
                collider: m.collider().into(),
                arity: m.arity(),
                is_skinned: m.is_skinned() as RprBool,
                collision_enabled: m.collision_enabled() as RprBool,
            })
            .collect();
        copy_out(&meshes, buffer, capacity, count)
    })
}
/// Copies world-space vertices of a mesh, including render-only skins.
pub(crate) unsafe fn native_soft_body_mesh_vertices_by_id(
    body: *const RprSoftBody,
    id: RprSoftMeshId,
    buffer: *mut RprVector,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let body = &get(body)?.0;
        let mesh = body.mesh(id.raw()).ok_or_else(missing)?;
        let vertices: Vec<_> = mesh.vertex_positions(body).map(Into::into).collect();
        copy_out(&vertices, buffer, capacity, count)
    })
}
/// Copies flattened vertex indices. Each element has RprSoftMeshInfo::arity entries.
pub(crate) unsafe fn native_soft_body_mesh_indices_by_id(
    body: *const RprSoftBody,
    id: RprSoftMeshId,
    buffer: *mut u32,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let mesh = get(body)?.0.mesh(id.raw()).ok_or_else(missing)?;
        let indices: Vec<_> = (0..mesh.indices().len())
            .flat_map(|i| mesh.element(i).iter().copied())
            .collect();
        copy_out(&indices, buffer, capacity, count)
    })
}
/// Enumerates actual collider handles only. Use SoftBodyMeshes for render-only meshes too.
pub(crate) unsafe fn native_soft_body_mesh_colliders(
    body: *const RprSoftBody,
    buffer: *mut RprColliderHandle,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        let v: Vec<_> = get(body)?
            .0
            .meshes()
            .map(|m| m.collider())
            .filter(|h| *h != ColliderHandle::invalid())
            .map(Into::into)
            .collect();
        copy_out(&v, buffer, capacity, count)
    })
}
pub(crate) unsafe fn native_soft_body_mesh_vertices(
    body: *const RprSoftBody,
    collider: RprColliderHandle,
    buffer: *mut RprVector,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        ensure(
            collider != RprColliderHandle::default(),
            "mesh has no collider; use its mesh ID",
        )?;
        let b = &get(body)?.0;
        let m = b.mesh_of(collider.raw()).ok_or_else(missing)?;
        let v: Vec<_> = m.vertex_positions(b).map(Into::into).collect();
        copy_out(&v, buffer, capacity, count)
    })
}
/// Indices use mesh vertices, not particles. Arity is two for wire meshes, DIM otherwise.
pub(crate) unsafe fn native_soft_body_mesh_indices(
    body: *const RprSoftBody,
    collider: RprColliderHandle,
    buffer: *mut u32,
    capacity: usize,
    count: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        ensure(
            collider != RprColliderHandle::default(),
            "mesh has no collider; use its mesh ID",
        )?;
        let b = &get(body)?.0;
        let m = b.mesh_of(collider.raw()).ok_or_else(missing)?;
        let v: Vec<_> = (0..m.indices().len())
            .flat_map(|i| m.element(i).iter().copied())
            .collect();
        copy_out(&v, buffer, capacity, count)
    })
}
pub(crate) unsafe fn native_soft_body_mesh_arity(
    body: *const RprSoftBody,
    collider: RprColliderHandle,
    out: *mut usize,
) -> RprStatus {
    ffi(|| unsafe {
        ensure(
            collider != RprColliderHandle::default(),
            "mesh has no collider; use its mesh ID",
        )?;
        let m = get(body)?.0.mesh_of(collider.raw()).ok_or_else(missing)?;
        output(out, m.arity())
    })
}
pub(crate) unsafe fn native_soft_body_mesh_topology_version(
    body: *const RprSoftBody,
    collider: RprColliderHandle,
    out: *mut u32,
) -> RprStatus {
    ffi(|| unsafe {
        ensure(
            collider != RprColliderHandle::default(),
            "mesh has no collider; use its mesh ID",
        )?;
        let m = get(body)?.0.mesh_of(collider.raw()).ok_or_else(missing)?;
        output(out, m.topology_version())
    })
}

#[cfg(feature = "fem")]
pub(crate) unsafe fn native_soft_body_set_solver(body: *mut RprSoftBody, solver: u32) -> RprStatus {
    ffi(|| unsafe {
        let v = match solver {
            0 => SoftBodySolver::Constraints,
            1 => SoftBodySolver::Fem,
            _ => return Err(invalid("unknown soft body solver")),
        };
        get_mut(body)?.0.set_solver(v);
        Ok(())
    })
}

/// Set the optional shape-matching target of a live cluster. A null target clears it.
pub(crate) unsafe fn native_soft_body_set_cluster_shape_matching_target(
    body: *mut RprSoftBody,
    cluster: u32,
    target: *const RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let target = if target.is_null() {
            None
        } else {
            Some(get(target)?.raw()?)
        };
        let b = &mut get_mut(body)?.0;
        let c = b
            .cluster_mut(cluster)
            .filter(|c| c.is_live())
            .ok_or_else(|| invalid("invalid cluster"))?;
        c.set_shape_matching_target(target);
        Ok(())
    })
}

/// Corresponds to `SoftBody::particle_position`. Copies one particle position.
pub(crate) unsafe fn native_soft_body_particle_position(
    body: *const RprSoftBody,
    index: usize,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let body = &get(body)?.0;
        ensure(index < body.num_particles(), "particle index out of bounds")?;
        output(out, body.particle_position(index).into())
    })
}

/// Optional particle destination after a tear. Missing destinations are normal and set
/// found to false; body/index are only written when a destination exists.
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_try_particle_destination(
    event: *const RprSoftBodyTearEvent,
    particle: u32,
) -> RprOptionalParticleDestination {
    ffi_world_value(
        unsafe { get(event).map_or(std::ptr::null_mut(), |e| e.1) },
        |result: *mut RprOptionalParticleDestination| {
            let body = unsafe { std::ptr::addr_of_mut!((*result).body) };
            let index = unsafe { std::ptr::addr_of_mut!((*result).index) };
            let found = unsafe { std::ptr::addr_of_mut!((*result).found) };

            ffi(|| unsafe {
                out_ptr(body)?;
                out_ptr(index)?;
                out_ptr(found)?;
                if let Some((destination, new_index)) = get(event)?.0.particle_destination(particle)
                {
                    output(body, destination.into())?;
                    output(index, new_index)?;
                    output(found, 1)
                } else {
                    output(found, 0)
                }
            })
        },
    )
}

/// Changes the tear resistance multiplier for an individual edge.
pub(crate) unsafe fn native_soft_body_set_edge_tear_resistance(
    body: *mut RprSoftBody,
    index: usize,
    resistance: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let resistance = nonnegative(resistance)?;
        let body = &mut get_mut(body)?.0;
        ensure(index < body.edges().len(), "edge index out of range")?;
        body.set_edge_tear_resistance(index, resistance);
        Ok(())
    })
}

/// Cut using DIM points (a segment in 2D, triangle in 3D). A no-op returns a null event.
/// The optional owned event must be freed with FreeSoftBodyTearEvent.
#[rapier_export]
pub unsafe extern "C" fn rpr_cut_soft_body(
    handle: RprSoftBodyHandle,
    blade: *const RprVector,
) -> *mut RprSoftBodyTearEvent {
    let world = handle.world;
    ffi_value(|out: *mut *mut RprSoftBodyTearEvent| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            out_ptr(out)?;
            let blade = input(blade, rapier::math::DIM)?
                .iter()
                .copied()
                .map(RprVector::raw)
                .collect::<Result<Vec<_>>>()?;
            let blade: [Vector; rapier::math::DIM] = blade.try_into().unwrap();
            let event = get_mut(world)?.0.cut_soft_body(handle.raw(), &blade);
            output(
                out,
                event
                    .map(|e| Box::into_raw(Box::new(RprSoftBodyTearEvent(e, handle.world))))
                    .unwrap_or(std::ptr::null_mut()),
            )
        })
    })
}

/// Parameters for the native volumetric mesher. Enclosure: 0 cover, 1 crust (3D).
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprVolumeMeshParameters {
    pub cell_size: RprReal,
    #[cfg(feature = "dim2")]
    pub min_angle: RprReal,
    #[cfg(feature = "dim3")]
    pub enclosure: u32,
    #[cfg(feature = "dim3")]
    pub cover_smoothing: u32,
    #[cfg(feature = "dim3")]
    pub cover_guard: RprReal,
    #[cfg(feature = "dim3")]
    pub cover_subdivisions: u32,
}
#[rapier_export]
pub extern "C" fn rpr_new_volume_mesh_parameters(cell_size: RprReal) -> RprVolumeMeshParameters {
    let p = rapier::parry::transformation::VolumeMeshParameters::new(cell_size);
    RprVolumeMeshParameters {
        cell_size: p.cell_size,
        #[cfg(feature = "dim2")]
        min_angle: p.min_angle,
        #[cfg(feature = "dim3")]
        enclosure: 0,
        #[cfg(feature = "dim3")]
        cover_smoothing: p.cover_smoothing,
        #[cfg(feature = "dim3")]
        cover_guard: p.cover_guard,
        #[cfg(feature = "dim3")]
        cover_subdivisions: p.cover_subdivisions,
    }
}

/// Whether the collision mesh encloses an interior. Non-mesh colliders return false.
pub(crate) unsafe fn native_soft_body_mesh_is_closed(
    body: *const RprSoftBody,
    collider: RprColliderHandle,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        ensure(
            collider != RprColliderHandle::default(),
            "expected a valid collider handle",
        )?;
        output(
            out,
            get(body)?
                .0
                .mesh_of(collider.raw())
                .is_some_and(|m| m.is_closed()) as RprBool,
        )
    })
}
