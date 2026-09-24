//! Soft-body accessors, deferred tears, impulses and world soft-body settings.
use crate::handle_access::forward;
use crate::*;

pub(crate) unsafe fn native_soft_body_particle_velocity(
    body: *const RprSoftBody,
    index: usize,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let body = &get(body)?.0;
        ensure(index < body.num_particles(), "particle index out of bounds")?;
        output(out, body.particle_velocity(index).into())
    })
}
/// Return the world-space velocity of the indexed particle.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_particle_velocity(
    handle: RprSoftBodyHandle,
    index: usize,
) -> RprVector {
    let world = handle.world;
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_particle_velocity(
                (element as *const SoftBody).cast(),
                index,
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_soft_body_solver(body: *const RprSoftBody, out: *mut u32) -> RprStatus {
    ffi(|| unsafe {
        let _body = &get(body)?.0;
        #[cfg(feature = "fem")]
        let solver = match _body.solver() {
            SoftBodySolver::Constraints => RPR_SOFT_SOLVER_CONSTRAINTS,
            SoftBodySolver::Fem => RPR_SOFT_SOLVER_FEM,
        };
        #[cfg(not(feature = "fem"))]
        let solver = RPR_SOFT_SOLVER_CONSTRAINTS;
        output(out, solver)
    })
}
/// Return the solver simulating the soft body's elasticity (RPR_SOFT_SOLVER_*). Always
/// RPR_SOFT_SOLVER_CONSTRAINTS in a library built without FEM.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_solver(handle: RprSoftBodyHandle) -> u32 {
    let world = handle.world;
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprSoftBodySet = std::ptr::addr_of!((*raw).0.soft_bodies).cast();

            let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            forward(native_soft_body_solver(
                (element as *const SoftBody).cast(),
                out,
            ))
        })
    })
}

pub(crate) unsafe fn native_soft_body_set_cluster_edge_softness(
    body: *mut RprSoftBody,
    cluster: u32,
    softness: *const RprSpringCoefficients,
) -> RprStatus {
    ffi(|| unsafe {
        let softness = if softness.is_null() {
            None
        } else {
            Some(get(softness)?.raw()?)
        };
        let b = &mut get_mut(body)?.0;
        b.cluster(cluster)
            .filter(|c| c.is_live())
            .ok_or_else(|| invalid("invalid cluster"))?;
        b.set_cluster_edge_softness(cluster, softness);
        Ok(())
    })
}
/// Override the softness of every structural or bending edge fully contained in a live cluster:
/// regional stiffness for cloth and ropes. A NULL softness restores the body material's.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_set_cluster_edge_softness(
    handle: RprSoftBodyHandle,
    cluster: u32,
    softness: *const RprSpringCoefficients,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_set_cluster_edge_softness(
            (element as *mut SoftBody).cast(),
            cluster,
            softness,
        ))
    })
}

pub(crate) unsafe fn native_soft_body_apply_impulse_at_point(
    body: *mut RprSoftBody,
    impulse: RprVector,
    point: RprVector,
    falloff_radius: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let impulse = impulse.raw()?;
        let point = point.raw()?;
        let falloff_radius = finite(falloff_radius)?;
        let wake_up = boolean(wake_up)?;
        get_mut(body)?
            .0
            .apply_impulse_at_point(impulse, point, falloff_radius, wake_up);
        Ok(())
    })
}
/// Apply a world-space impulse to every free particle within falloff_radius of the world-space
/// point, scaled linearly from 1 at the point to 0 at that radius and divided by the particle's
/// mass. A falloff_radius of zero or less gives every free particle the whole impulse. Pinned
/// particles ignore it.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_apply_impulse_at_point(
    handle: RprSoftBodyHandle,
    impulse: RprVector,
    point: RprVector,
    falloff_radius: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_apply_impulse_at_point(
            (element as *mut SoftBody).cast(),
            impulse,
            point,
            falloff_radius,
            wake_up,
        ))
    })
}

pub(crate) unsafe fn native_soft_body_apply_radial_impulse(
    body: *mut RprSoftBody,
    center: RprVector,
    magnitude: RprReal,
    falloff_radius: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let center = center.raw()?;
        let magnitude = finite(magnitude)?;
        let falloff_radius = finite(falloff_radius)?;
        let wake_up = boolean(wake_up)?;
        get_mut(body)?
            .0
            .apply_radial_impulse(center, magnitude, falloff_radius, wake_up);
        Ok(())
    })
}
/// Apply an impulse of the given magnitude pointing away from the world-space center to every
/// free particle within falloff_radius, scaled linearly from 1 at the center to 0 at that
/// radius and divided by the particle's mass. A particle on the center gets nothing; a
/// falloff_radius of zero or less pushes every free particle fully. A negative magnitude pulls
/// toward the center. Pinned particles ignore it.
/// wake_up = 1 wakes affected bodies; 0 preserves their sleep state.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_apply_radial_impulse(
    handle: RprSoftBodyHandle,
    center: RprVector,
    magnitude: RprReal,
    falloff_radius: RprReal,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_apply_radial_impulse(
            (element as *mut SoftBody).cast(),
            center,
            magnitude,
            falloff_radius,
            wake_up,
        ))
    })
}

pub(crate) unsafe fn native_soft_body_reset_plasticity(body: *mut RprSoftBody) -> RprStatus {
    ffi(|| unsafe {
        get_mut(body)?.0.reset_plasticity();
        Ok(())
    })
}
/// Undo every permanent (plastic) deformation: edge rest lengths, dihedral rest angles, cell
/// rest shapes and particle rest positions return to their creation state. The particles stay
/// put and spring back elastically.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_reset_plasticity(handle: RprSoftBodyHandle) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let element = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        forward(native_soft_body_reset_plasticity(
            (element as *mut SoftBody).cast(),
        ))
    })
}

pub(crate) unsafe fn native_soft_body_tear_edge(body: *mut RprSoftBody, index: usize) -> RprStatus {
    ffi(|| unsafe {
        let b = &mut get_mut(body)?.0;
        ensure(index < b.edges().len(), "edge index out of range")?;
        b.tear_edge(index);
        Ok(())
    })
}
/// Mark the indexed edge as torn. The tear is applied at the end of the next step and reported
/// by a tear event; use rpr_soft_body_tear to tear immediately.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_tear_edge(
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
        forward(native_soft_body_tear_edge(
            (element as *mut SoftBody).cast(),
            index,
        ))
    })
}

pub(crate) unsafe fn native_soft_body_tear_cell(body: *mut RprSoftBody, index: usize) -> RprStatus {
    ffi(|| unsafe {
        let b = &mut get_mut(body)?.0;
        ensure(index < b.cells().len(), "cell index out of range")?;
        b.tear_cell(index);
        Ok(())
    })
}
/// Mark the indexed cell as torn. The tear is applied at the end of the next step and reported
/// by a tear event: no cell is removed, one of its particles splits along the plane
/// perpendicular to the cell's principal rest stretch.
/// @ingroup soft_bodies
#[rapier_export(soft_body)]
pub unsafe extern "C" fn rpr_soft_body_tear_cell(
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
        forward(native_soft_body_tear_cell(
            (element as *mut SoftBody).cast(),
            index,
        ))
    })
}

pub(crate) unsafe fn native_collider_soft_body(
    object: *const RprCollider,
    out: *mut RprSoftBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        output(
            out,
            get(object)?
                .0
                .deformable_mesh_ref()
                .map(|m| m.body.into())
                .unwrap_or_default(),
        )
    })
}
pub(crate) unsafe fn native_collider_set_get_soft_body(
    set: *const RprColliderSet,
    handle: RprColliderHandle,
    out: *mut RprSoftBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        let element = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
        forward(native_collider_soft_body(
            (element as *const Collider).cast(),
            out,
        ))
    })
}
/// Return the soft body owning this deformable collider (a soft-body collision mesh), or an invalid
/// handle for any other collider.
/// @ingroup colliders
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_soft_body(handle: RprColliderHandle) -> RprSoftBodyHandle {
    let world = handle.world;
    ffi_world_value(world, |out: *mut RprSoftBodyHandle| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            forward(native_collider_set_get_soft_body(
                std::ptr::addr_of!((*raw).0.colliders).cast(),
                handle,
                out,
            ))
        })
    })
}
/// Return the soft body owning this deformable collider, or an invalid handle for any other
/// collider. Uses only the callback-scoped read context; never retain the context.
/// @ingroup callbacks
#[rapier_export(read_collider)]
pub unsafe extern "C" fn rpr_read_collider_soft_body(
    context: *const RprReadContext,
    handle: RprColliderHandle,
) -> RprSoftBodyHandle {
    ffi_world_value(
        unsafe { read_context_world(context) },
        |out: *mut RprSoftBodyHandle| {
            ffi(|| unsafe {
                handle.check_world(read_context_world(context))?;
                forward(native_collider_set_get_soft_body(
                    get(context)?.colliders,
                    handle,
                    out,
                ))
            })
        },
    )
}

/// Return the number of soft bodies the torn body is in after the tear: the length of
/// rpr_soft_body_tear_event_bodies, and the exclusive bound of the piece_index of
/// rpr_soft_body_tear_event_piece_particles. It is 1 when nothing was split off.
/// @ingroup soft_bodies
#[rapier_export(soft_body_tear_event)]
pub unsafe extern "C" fn rpr_soft_body_tear_event_piece_count(
    event: *const RprSoftBodyTearEvent,
) -> usize {
    ffi_value(|out: *mut usize| ffi(|| unsafe { output(out, get(event)?.0.pieces.len().max(1)) }))
}

/// Return the world setting documented by RprSoftRecoverySettings::authoredVelocityMargin.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_authored_velocity_margin(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .authored_velocity_margin;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::edgeSpeculation.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_edge_speculation(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.edge_speculation;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::invertedCellDetection.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_inverted_cell_detection(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .inverted_cell_detection;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::selfCrossingDetection.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_self_crossing_detection(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .self_crossing_detection;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::detectionMotionGating.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_detection_motion_gating(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .detection_motion_gating;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::crossBodyDetection.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_cross_body_detection(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.cross_body_detection;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::selfStandDown.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_self_stand_down(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.self_stand_down;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::crossBodyExpelGate.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_cross_body_expel_gate(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .cross_body_expel_gate;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::edgeStandDown.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_edge_stand_down(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.edge_stand_down;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::crossingRepulsion.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_crossing_repulsion(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.crossing_repulsion;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::crossingRepulsionGuide.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_crossing_repulsion_guide(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .crossing_repulsion_guide;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::crossingRepulsionSelfGuide.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_crossing_repulsion_self_guide(
    world: *const RprWorld,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .crossing_repulsion_self_guide;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::recoveryPace.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_recovery_pace(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.recovery_pace;
            output(out, value)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapConstraints.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_constraints(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.overlap_constraints;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapRigid.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_rigid(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.overlap_rigid;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapSkipSelfTangled.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_skip_self_tangled(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .overlap_skip_self_tangled;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapEdgeStandDown.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_edge_stand_down(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .overlap_edge_stand_down;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapConstraintPace.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_constraint_pace(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .overlap_constraint_pace;
            output(out, value)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapPatchConstraints.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_patch_constraints(world: *const RprWorld) -> u32 {
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .overlap_patch_constraints;
            output(
                out,
                match value {
                    SoftPatchConstraints::Keep => RPR_SOFT_PATCH_CONSTRAINTS_KEEP,
                    SoftPatchConstraints::StandDown => RPR_SOFT_PATCH_CONSTRAINTS_STAND_DOWN,
                    SoftPatchConstraints::AlongNormal => RPR_SOFT_PATCH_CONSTRAINTS_ALONG_NORMAL,
                },
            )
        })
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapPatchConstraints.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_patch_constraints(
    world: *mut RprWorld,
    value: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = match value {
            RPR_SOFT_PATCH_CONSTRAINTS_KEEP => SoftPatchConstraints::Keep,
            RPR_SOFT_PATCH_CONSTRAINTS_STAND_DOWN => SoftPatchConstraints::StandDown,
            RPR_SOFT_PATCH_CONSTRAINTS_ALONG_NORMAL => SoftPatchConstraints::AlongNormal,
            _ => return Err(invalid("invalid overlap patch constraints")),
        };
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_patch_constraints = value;
        Ok(())
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapSkinVolume.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_skin_volume(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.overlap_skin_volume;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapKeptDepth.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_kept_depth(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.overlap_kept_depth;
            output(out, value)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapSelfRegions.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_self_regions(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.overlap_self_regions;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapNormalPush.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_normal_push(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.overlap_normal_push;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapMultiVolume.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_multi_volume(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.overlap_multi_volume;
            output(out, value as RprBool)
        })
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapSplit.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_split(world: *const RprWorld) -> u32 {
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.overlap_split;
            output(out, value)
        })
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapSplit.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_split(
    world: *mut RprWorld,
    value: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        get_mut(parameters)?.0.soft_bodies.recovery.overlap_split = value;
        Ok(())
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapPatience.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_patience(world: *const RprWorld) -> u32 {
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.recovery.overlap_patience;
            output(out, value)
        })
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapPatience.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_patience(
    world: *mut RprWorld,
    value: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        get_mut(parameters)?.0.soft_bodies.recovery.overlap_patience = value;
        Ok(())
    })
}

/// Return the world setting documented by RprSoftRecoverySettings::overlapProgressMargin.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_overlap_progress_margin(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?
                .0
                .soft_bodies
                .recovery
                .overlap_progress_margin;
            output(out, value)
        })
    })
}

/// Return the world setting documented by RprSoftFemParameters::linearTolerance.
/// @ingroup soft_bodies
#[cfg(feature = "fem")]
#[rapier_export]
pub unsafe extern "C" fn rpr_fem_linear_tolerance(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.fem.linear_tolerance;
            output(out, value)
        })
    })
}

/// Return the world setting documented by RprSoftFemParameters::maxLinearIterations.
/// @ingroup soft_bodies
#[cfg(feature = "fem")]
#[rapier_export]
pub unsafe extern "C" fn rpr_fem_max_linear_iterations(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.fem.max_linear_iterations;
            output(out, value)
        })
    })
}

/// Return the world setting documented by RprSoftFemParameters::maxDenseDofs.
/// @ingroup soft_bodies
#[cfg(feature = "fem")]
#[rapier_export]
pub unsafe extern "C" fn rpr_fem_max_dense_dofs(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            let value = get(parameters)?.0.soft_bodies.fem.max_dense_dofs;
            output(out, value)
        })
    })
}
