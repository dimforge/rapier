use crate::*;
/// Return the world setting documented by RprIntegrationParameters::dt.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_time_step(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.dt)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::dt.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_time_step(world: *mut RprWorld, value: RprReal) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        positive(value)?;
        get_mut(object)?.0.dt = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::minCcdDt.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_min_ccd_dt(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.min_ccd_dt)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::minCcdDt.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_min_ccd_dt(world: *mut RprWorld, value: RprReal) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        nonnegative(value)?;
        get_mut(object)?.0.min_ccd_dt = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::lengthUnit.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_length_unit(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.length_unit)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::lengthUnit.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_length_unit(world: *mut RprWorld, value: RprReal) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        positive(value)?;
        get_mut(object)?.0.length_unit = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::warmstartCoefficient.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_warmstart_coefficient(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.warmstart_coefficient)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::warmstartCoefficient.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_warmstart_coefficient(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        nonnegative(value)?;
        ensure(value <= 1.0, "warmstart coefficient must be <= 1")?;
        get_mut(object)?.0.warmstart_coefficient = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::normalizedAllowedLinearError.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_normalized_allowed_linear_error(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.normalized_allowed_linear_error)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::normalizedAllowedLinearError.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_normalized_allowed_linear_error(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        nonnegative(value)?;
        get_mut(object)?.0.normalized_allowed_linear_error = value;
        Ok(())
    })
}

/// Return the world setting documented by
/// RprIntegrationParameters::normalizedMaxCorrectiveVelocity.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_normalized_max_corrective_velocity(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.normalized_max_corrective_velocity)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::normalizedMaxCorrectiveVelocity.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_normalized_max_corrective_velocity(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        nonnegative(value)?;
        get_mut(object)?.0.normalized_max_corrective_velocity = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::normalizedPredictionDistance.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_normalized_prediction_distance(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.normalized_prediction_distance)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::normalizedPredictionDistance.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_normalized_prediction_distance(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        nonnegative(value)?;
        get_mut(object)?.0.normalized_prediction_distance = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::normalizedMaxLinearVelocity.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_normalized_max_linear_velocity(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.normalized_max_linear_velocity)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::normalizedMaxLinearVelocity.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_normalized_max_linear_velocity(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        nonnegative(value)?;
        get_mut(object)?.0.normalized_max_linear_velocity = value;
        Ok(())
    })
}

/// Return the world setting documented by
/// RprIntegrationParameters::normalizedContactRecycleDistance.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_normalized_contact_recycle_distance(
    world: *const RprWorld,
) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.normalized_contact_recycle_distance)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::normalizedContactRecycleDistance.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_normalized_contact_recycle_distance(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        nonnegative(value)?;
        get_mut(object)?.0.normalized_contact_recycle_distance = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::numSolverIterations.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_num_solver_iterations(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.num_solver_iterations)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::numSolverIterations.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_num_solver_iterations(
    world: *mut RprWorld,
    value: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        ensure(value > 0, "iteration count must be positive")?;
        get_mut(object)?.0.num_solver_iterations = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::numInternalPgsIterations.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_num_internal_pgs_iterations(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.num_internal_pgs_iterations)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::numInternalPgsIterations.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_num_internal_pgs_iterations(
    world: *mut RprWorld,
    value: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        ensure(value > 0, "iteration count must be positive")?;
        get_mut(object)?.0.num_internal_pgs_iterations = value;
        Ok(())
    })
}

/// Return the world setting documented by
/// RprIntegrationParameters::numInternalStabilizationIterations.
/// @ingroup errors
#[rapier_export]
pub unsafe extern "C" fn rpr_num_internal_stabilization_iterations(
    world: *const RprWorld,
) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.num_internal_stabilization_iterations)
        })
    })
}

/// Set the world setting documented by
/// RprIntegrationParameters::numInternalStabilizationIterations.
/// @ingroup errors
#[rapier_export]
pub unsafe extern "C" fn rpr_set_num_internal_stabilization_iterations(
    world: *mut RprWorld,
    value: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        get_mut(object)?.0.num_internal_stabilization_iterations = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::maxCcdSubsteps.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_max_ccd_substeps(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.max_ccd_substeps)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::maxCcdSubsteps.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_max_ccd_substeps(world: *mut RprWorld, value: usize) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        get_mut(object)?.0.max_ccd_substeps = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::contactClustering.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_contact_clustering(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.contact_clustering as u32)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::contactClustering.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_contact_clustering(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(object)?.0.contact_clustering = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::contactRecycling.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_contact_recycling(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.contact_recycling as u32)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::contactRecycling.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_contact_recycling(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(object)?.0.contact_recycling = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::frictionInBiasPass.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_friction_in_bias_pass(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.friction_in_bias_pass as u32)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::frictionInBiasPass.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_friction_in_bias_pass(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(object)?.0.friction_in_bias_pass = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::warmstartJoints.
/// @ingroup joints
#[rapier_export]
pub unsafe extern "C" fn rpr_warmstart_joints(world: *const RprWorld) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.warmstart_joints as u32)
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::warmstartJoints.
/// @ingroup joints
#[rapier_export]
pub unsafe extern "C" fn rpr_set_warmstart_joints(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(object)?.0.warmstart_joints = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::contactSoftness.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_contact_softness(world: *const RprWorld) -> RprSpringCoefficients {
    ffi_value(|out: *mut RprSpringCoefficients| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.contact_softness.into())
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::contactSoftness.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_set_contact_softness(
    world: *mut RprWorld,
    value: RprSpringCoefficients,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = value.raw()?;
        get_mut(object)?.0.contact_softness = value;
        Ok(())
    })
}

/// Return the world setting documented by RprIntegrationParameters::staticContactSoftness.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_static_contact_softness(
    world: *const RprWorld,
) -> RprSpringCoefficients {
    ffi_value(|out: *mut RprSpringCoefficients| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();

            let object = get(object)?;
            output(out, object.0.static_contact_softness.into())
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::staticContactSoftness.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_set_static_contact_softness(
    world: *mut RprWorld,
    value: RprSpringCoefficients,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let object: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = value.raw()?;
        get_mut(object)?.0.static_contact_softness = value;
        Ok(())
    })
}

use bincode::Options;
use rapier::geometry::{ContactForceEvent, ContactPair, SolverFlags};
use rapier::pipeline::{ContactModificationContext, PairFilterContext};
use std::{ffi::c_void, sync::Mutex};

/// Collision start/stop flags match Rapier CollisionEventFlags.
/// @ingroup events
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprCollisionEvent {
    /// First collider in the pair.
    pub collider1: RprColliderHandle,
    /// Second collider in the pair.
    pub collider2: RprColliderHandle,
    /// 1 for a starting event, 0 for a stopping event.
    pub started: RprBool,
    /// Event flags: bit 0 sensor pair, bit 1 removed collider.
    pub flags: u32,
}
/// Contact-force event, enabled by flags and the collider force threshold.
/// @ingroup events
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprContactForceEvent {
    /// First collider in the pair.
    pub collider1: RprColliderHandle,
    /// Second collider in the pair.
    pub collider2: RprColliderHandle,
    /// Sum of world-space contact forces.
    pub total_force: RprVector,
    /// Sum of contact force magnitudes.
    pub total_force_magnitude: RprReal,
    /// World-space direction of the strongest contact force.
    pub max_force_direction: RprVector,
    /// Magnitude of the strongest contact force.
    pub max_force_magnitude: RprReal,
    /// 1 for a starting event, 0 for a stopping event.
    pub started: RprBool,
}
/// Events accumulate until clear. Copying events never drains them, allowing two-call buffer
/// sizing.
/// @ingroup events
#[derive(Default)]
pub struct RprEventCollector {
    collisions: Mutex<Vec<RprCollisionEvent>>,
    forces: Mutex<Vec<RprContactForceEvent>>,
    tears: Mutex<Vec<RprSoftBodyTearEvent>>,
}
struct WorldEvents<'a> {
    world: *mut RprWorld,
    events: &'a RprEventCollector,
}
// The address is only copied into handles; the step holds the world write guard.
unsafe impl Sync for WorldEvents<'_> {}
impl EventHandler for WorldEvents<'_> {
    fn handle_collision_event(
        &self,
        _: &RigidBodySet,
        _: &ColliderSet,
        event: CollisionEvent,
        _: Option<&ContactPair>,
    ) {
        let (a, b, started, flags) = match event {
            CollisionEvent::Started(a, b, f) => (a, b, 1, f.bits()),
            CollisionEvent::Stopped(a, b, f) => (a, b, 0, f.bits()),
        };
        self.events
            .collisions
            .lock()
            .unwrap()
            .push(RprCollisionEvent {
                collider1: RprColliderHandle::from(a).with_world(self.world),
                collider2: RprColliderHandle::from(b).with_world(self.world),
                started,
                flags,
            });
    }
    fn handle_contact_force_event(
        &self,
        dt: Real,
        _: &RigidBodySet,
        _: &ColliderSet,
        pair: &ContactPair,
        magnitude: Real,
    ) {
        let e = ContactForceEvent::from_contact_pair(dt, pair, magnitude);
        self.events
            .forces
            .lock()
            .unwrap()
            .push(RprContactForceEvent {
                collider1: RprColliderHandle::from(e.collider1).with_world(self.world),
                collider2: RprColliderHandle::from(e.collider2).with_world(self.world),
                total_force: e.total_force.into(),
                total_force_magnitude: e.total_force_magnitude,
                max_force_direction: e.max_force_direction.into(),
                max_force_magnitude: e.max_force_magnitude,
                started: e.started as u32,
            });
    }
    fn handle_soft_body_tear_event(&self, _: &SoftBodySet, event: &SoftBodyTearEvent) {
        self.events
            .tears
            .lock()
            .unwrap()
            .push(RprSoftBodyTearEvent(event.clone(), self.world));
    }
}
/// Pair callback: -1 rejects a contact pair; 0 detects contacts without impulses; 1 computes
/// impulses.
/// For sensor intersections only, zero rejects and any positive value accepts.
/// @ingroup math
pub type RprPairFilter = Option<
    unsafe extern "C" fn(
        user_data: *mut c_void,
        read: *const RprReadContext,
        collider1: RprColliderHandle,
        collider2: RprColliderHandle,
        body1: RprRigidBodyHandle,
        body2: RprRigidBodyHandle,
    ) -> i32,
>;
/// Mutable per-manifold properties. Set enabled=0 to discard all its solver contacts.
/// @ingroup events
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprContactModification {
    /// World-space contact or surface normal.
    pub normal: RprVector,
    /// Nonnegative friction coefficient.
    pub friction: RprReal,
    /// Nonnegative restitution coefficient.
    pub restitution: RprReal,
    /// Application data; Rapier does not own pointers encoded in it.
    pub user_data: u32,
    /// Whether this setting/object is enabled (0 or 1).
    pub enabled: RprBool,
}
/// Modify aggregate contact properties for one pair; contact is mutable only during this callback.
/// @ingroup callbacks
pub type RprModifyContacts = Option<
    unsafe extern "C" fn(
        user_data: *mut c_void,
        read: *const RprReadContext,
        collider1: RprColliderHandle,
        collider2: RprColliderHandle,
        contact: *mut RprContactModification,
    ),
>;
/// Borrowed native contact context. Valid only during its callback; never retain or free it.
/// @ingroup events
pub struct RprContactModificationContext {
    raw: *mut c_void,
}
/// Modify individual solver contacts through a borrowed context, valid only during the callback.
/// @ingroup callbacks
pub type RprModifyContactContext = Option<
    unsafe extern "C" fn(
        user_data: *mut c_void,
        read: *const RprReadContext,
        collider1: RprColliderHandle,
        collider2: RprColliderHandle,
        context: *mut RprContactModificationContext,
    ),
>;

/// Callbacks must not unwind or retain arguments. Use their ReadContext to inspect bodies and
/// colliders; ordinary access to the stepping world returns WORLD_BUSY. Mutations must be
/// performed after stepping. With parallel builds
/// callbacks and their user_data must be safe for concurrent invocation. NULL callbacks use
/// defaults.
/// @ingroup callbacks
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprPhysicsHooks {
    /// Application data; Rapier does not own pointers encoded in it.
    pub user_data: *mut c_void,
    /// Optional contact-pair filter, called only for colliders enabling the hook.
    pub filter_contact_pair: RprPairFilter,
    /// Optional sensor-pair filter, called only for colliders enabling the hook.
    pub filter_intersection_pair: RprPairFilter,
    /// Optional legacy aggregate contact-edit callback.
    pub modify_solver_contacts: RprModifyContacts,
    /// Runs after the legacy property callback. Context accessors may be called here.
    pub modify_solver_contacts_context: RprModifyContactContext,
}
// SAFETY: The public callback contract requires thread-safe callbacks and user_data in parallel builds.
unsafe impl Sync for RprPhysicsHooks {}
struct WorldHooks {
    world: *mut RprWorld,
    hooks: RprPhysicsHooks,
}
// Callback data follows RprPhysicsHooks' thread-safety contract; the address is metadata.
unsafe impl Sync for WorldHooks {}
impl PhysicsHooks for WorldHooks {
    fn filter_contact_pair(&self, c: &PairFilterContext) -> Option<SolverFlags> {
        let Some(f) = self.hooks.filter_contact_pair else {
            return Some(SolverFlags::COMPUTE_RIGID_IMPULSES);
        };
        let result = unsafe {
            f(
                self.hooks.user_data,
                &RprReadContext::new(self.world, c.bodies, c.colliders),
                RprColliderHandle::from(c.collider1).with_world(self.world),
                RprColliderHandle::from(c.collider2).with_world(self.world),
                c.rigid_body1
                    .map(|h| RprRigidBodyHandle::from(h).with_world(self.world))
                    .unwrap_or_default(),
                c.rigid_body2
                    .map(|h| RprRigidBodyHandle::from(h).with_world(self.world))
                    .unwrap_or_default(),
            )
        };
        match result {
            v if v < 0 => None,
            0 => Some(SolverFlags::empty()),
            _ => Some(SolverFlags::COMPUTE_RIGID_IMPULSES),
        }
    }
    fn filter_intersection_pair(&self, c: &PairFilterContext) -> bool {
        self.hooks.filter_intersection_pair.is_none_or(|f| unsafe {
            f(
                self.hooks.user_data,
                &RprReadContext::new(self.world, c.bodies, c.colliders),
                RprColliderHandle::from(c.collider1).with_world(self.world),
                RprColliderHandle::from(c.collider2).with_world(self.world),
                c.rigid_body1
                    .map(|h| RprRigidBodyHandle::from(h).with_world(self.world))
                    .unwrap_or_default(),
                c.rigid_body2
                    .map(|h| RprRigidBodyHandle::from(h).with_world(self.world))
                    .unwrap_or_default(),
            ) > 0
        })
    }
    fn modify_solver_contacts(&self, c: &mut ContactModificationContext) {
        if let Some(f) = self.hooks.modify_solver_contacts {
            let read = RprReadContext::new(self.world, c.bodies, c.colliders);
            let a = RprColliderHandle::from(c.collider1).with_world(self.world);
            let b = RprColliderHandle::from(c.collider2).with_world(self.world);
            if let Some(m) = c.rigid_mut() {
                let mut value = RprContactModification {
                    normal: (*m.normal).into(),
                    friction: *m.friction,
                    restitution: *m.restitution,
                    user_data: *m.user_data,
                    enabled: 1,
                };
                unsafe { f(self.hooks.user_data, &read, a, b, &mut value) };
                // Invalid callback values leave the original manifold unchanged.
                if let (Ok(n), Ok(fr), Ok(re), Ok(enabled)) = (
                    value.normal.raw(),
                    nonnegative(value.friction),
                    nonnegative(value.restitution),
                    boolean(value.enabled),
                ) {
                    if n.length_squared().is_finite() && n.length_squared() > 1.0e-20 {
                        *m.normal = n.normalize();
                        *m.friction = fr;
                        *m.restitution = re;
                        *m.user_data = value.user_data;
                        if !enabled {
                            m.solver_contacts.clear();
                        }
                    }
                }
            }
        }
        if let Some(callback) = self.hooks.modify_solver_contacts_context {
            let mut context = RprContactModificationContext {
                raw: (c as *mut ContactModificationContext<'_>).cast(),
            };
            unsafe {
                callback(
                    self.hooks.user_data,
                    &RprReadContext::new(self.world, c.bodies, c.colliders),
                    RprColliderHandle::from(c.collider1).with_world(self.world),
                    RprColliderHandle::from(c.collider2).with_world(self.world),
                    &mut context,
                )
            };
        }
    }
}

/// Applies Rapier's persistent one-way platform logic to the borrowed manifold.
/// @ingroup worlds
#[rapier_export(contact_modification_context)]
pub unsafe extern "C" fn rpr_contact_modification_context_update_as_oneway_platform(
    context: *mut RprContactModificationContext,
    allowed_local_n1: RprVector,
    allowed_angle: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let normal = allowed_local_n1.raw()?;
        let angle = nonnegative(allowed_angle)?;
        let raw = get_mut(context)?
            .raw
            .cast::<ContactModificationContext<'_>>();
        (*raw).update_as_oneway_platform(normal, angle);
        Ok(())
    })
}

/// Sets the tangent velocity of every rigid solver contact in this manifold.
/// @ingroup worlds
#[rapier_export(contact_modification_context)]
pub unsafe extern "C" fn rpr_contact_modification_context_set_tangent_velocity(
    context: *mut RprContactModificationContext,
    velocity: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let velocity = velocity.raw()?;
        let raw = get_mut(context)?
            .raw
            .cast::<ContactModificationContext<'_>>();
        if let Some(rigid) = (*raw).rigid_mut() {
            for contact in rigid.solver_contacts.iter_mut() {
                contact.tangent_velocity = velocity;
            }
        }
        Ok(())
    })
}

/// Allocate an empty event collector; release it with rpr_free_event_collector.
/// @ingroup events
#[rapier_export]
pub unsafe extern "C" fn rpr_new_event_collector() -> *mut RprEventCollector {
    ffi_value(|out: *mut *mut RprEventCollector| {
        ffi(|| unsafe {
            out_ptr(out)?;
            output(out, Box::into_raw(Box::new(RprEventCollector::default())))
        })
    })
}
/// Release an owned event collector. NULL is allowed. Do not pass borrowed pointers or free the
/// object twice.
/// @ingroup events
#[rapier_export]
pub unsafe extern "C" fn rpr_free_event_collector(events: *mut RprEventCollector) -> RprStatus {
    ffi(|| unsafe {
        if !events.is_null() {
            get(events)?;
            drop(Box::from_raw(events));
        }
        Ok(())
    })
}
/// Discard all collected events. Does not change the world.
/// @ingroup events
#[rapier_export(event_collector)]
pub unsafe extern "C" fn rpr_event_collector_clear(events: *mut RprEventCollector) -> RprStatus {
    ffi(|| unsafe {
        let e = get_mut(events)?;
        e.collisions.get_mut().unwrap().clear();
        e.forces.get_mut().unwrap().clear();
        e.tears.get_mut().unwrap().clear();
        Ok(())
    })
}
/// Copy the collected collision start/stop events without removing them.
/// @see @ref output_buffers
/// @ingroup events
#[rapier_export(event_collector)]
pub unsafe extern "C" fn rpr_event_collector_collision_events(
    events: *const RprEventCollector,
    buffer: *mut RprCollisionEvent,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            copy_out(
                &get(events)?.collisions.lock().unwrap(),
                buffer,
                capacity,
                count,
            )
        })
    })
}
/// Copy the collected contact-force events without removing them.
/// @see @ref output_buffers
/// @ingroup events
#[rapier_export(event_collector)]
pub unsafe extern "C" fn rpr_event_collector_contact_force_events(
    events: *const RprEventCollector,
    buffer: *mut RprContactForceEvent,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            copy_out(
                &get(events)?.forces.lock().unwrap(),
                buffer,
                capacity,
                count,
            )
        })
    })
}
/// Return the number of queued soft-body tear events.
/// @ingroup events
#[rapier_export(event_collector)]
pub unsafe extern "C" fn rpr_event_collector_tear_event_count(
    events: *const RprEventCollector,
) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe { output(out, get(events)?.tears.lock().unwrap().len()) })
    })
}
/// Owned copy of a tear event. Read particle remapping before rebuilding render meshes.
/// @ingroup events
#[derive(Clone)]
pub struct RprSoftBodyTearEvent(pub(crate) SoftBodyTearEvent, pub(crate) *mut RprWorld);
// Owned event data plus a non-owning world address, never dereferenced by the event.
unsafe impl Send for RprSoftBodyTearEvent {}
unsafe impl Sync for RprSoftBodyTearEvent {}
/// Return an owned copy of a queued tear event; release with rpr_free_soft_body_tear_event. Does
/// not remove the queued event.
/// @ingroup events
#[rapier_export(event_collector)]
pub unsafe extern "C" fn rpr_event_collector_tear_event(
    events: *const RprEventCollector,
    index: usize,
) -> *mut RprSoftBodyTearEvent {
    ffi_value(|out: *mut *mut RprSoftBodyTearEvent| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let e = get(events)?
                .tears
                .lock()
                .unwrap()
                .get(index)
                .ok_or_else(|| invalid("event index out of range"))?
                .clone();
            output(out, Box::into_raw(Box::new(e)))
        })
    })
}
/// Return the world-space gravitational acceleration.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_gravity(world: *const RprWorld) -> RprVector {
    ffi_value(|out: *mut RprVector| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let world: *const RprPhysicsWorld = raw;
            output(out, get(world)?.0.gravity.into())
        })
    })
}

/// Set the world-space gravitational acceleration.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_gravity(world: *mut RprWorld, value: RprVector) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let world: *mut RprPhysicsWorld = raw;

        let value = value.raw()?;
        get_mut(world)?.0.gravity = value;
        Ok(())
    })
}

/// Hooks and events may be NULL. This call invalidates all borrowed set-element pointers.
/// Advance simulation by one timestep. Hooks and events may be NULL.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_step(
    world: *mut RprWorld,
    hooks: *const RprPhysicsHooks,
    events: *const RprEventCollector,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let hooks = if hooks.is_null() {
            RprPhysicsHooks::default()
        } else {
            *get(hooks)?
        };
        let hooks = WorldHooks { world, hooks };
        let events = if events.is_null() {
            None
        } else {
            Some(WorldEvents {
                world,
                events: get(events)?,
            })
        };
        let events: &dyn EventHandler = events.as_ref().map_or(&() as &dyn EventHandler, |e| e);
        (*access.raw()).0.step_with_events(&hooks, events);
        Ok(())
    })
}

/// Refresh collision detection without advancing simulation. Hooks and events may be NULL.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_detect_collisions(
    world: *mut RprWorld,
    hooks: *const RprPhysicsHooks,
    events: *const RprEventCollector,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let hooks = if hooks.is_null() {
            RprPhysicsHooks::default()
        } else {
            *get(hooks)?
        };
        let hooks = WorldHooks { world, hooks };
        let events = if events.is_null() {
            None
        } else {
            Some(WorldEvents {
                world,
                events: get(events)?,
            })
        };
        let events: &dyn EventHandler = events.as_ref().map_or(&() as &dyn EventHandler, |e| e);
        (*access.raw()).0.detect_collisions(&hooks, events);
        Ok(())
    })
}

/// Immutable owned byte buffer. Release with the matching FreeBytes function.
/// @ingroup worlds
pub struct RprBytes(Vec<u8>);
/// Borrow snapshot bytes without copying; valid until rpr_free_bytes. Never free the returned data
/// pointer.
/// @ingroup worlds
#[rapier_export(bytes)]
pub unsafe extern "C" fn rpr_bytes_data(bytes: *const RprBytes) -> RprByteView {
    ffi_value(|result: *mut RprByteView| {
        let data = unsafe { std::ptr::addr_of_mut!((*result).data) };
        let count = unsafe { std::ptr::addr_of_mut!((*result).count) };

        ffi(|| unsafe {
            out_ptr(data)?;
            out_ptr(count)?;
            let b = get(bytes)?;
            output(data, b.0.as_ptr())?;
            output(count, b.0.len())
        })
    })
}
/// Release an owned snapshot byte buffer. NULL is allowed. Do not pass borrowed pointers or free
/// the object twice.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_free_bytes(bytes: *mut RprBytes) -> RprStatus {
    ffi(|| unsafe {
        if !bytes.is_null() {
            get(bytes)?;
            drop(Box::from_raw(bytes));
        }
        Ok(())
    })
}
// Wire format: magic, then little-endian u32 ABI, dimension, and scalar size.
// The distinct magic prevents interpreting the old six-byte header as this format.
const SNAPSHOT_HEADER_LEN: usize = 16;

fn snapshot_header(abi_version: u32) -> [u8; SNAPSHOT_HEADER_LEN] {
    let build = rpr_build_info();
    let mut header = [0; SNAPSHOT_HEADER_LEN];
    header[..4].copy_from_slice(b"RPRS");
    header[4..8].copy_from_slice(&abi_version.to_le_bytes());
    header[8..12].copy_from_slice(&build.dimension.to_le_bytes());
    header[12..16].copy_from_slice(&build.real_size.to_le_bytes());
    header
}

fn snapshot_options() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(256 * 1024 * 1024)
        .reject_trailing_bytes()
}
/// Return owned snapshot bytes; release them with rpr_free_bytes. See @ref snapshots for
/// restoration and handle lifetimes.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_serialize_world(world: *const RprWorld) -> *mut RprBytes {
    ffi_value(|out: *mut *mut RprBytes| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let world: *const RprPhysicsWorld = raw;

            out_ptr(out)?;
            let mut bytes = snapshot_header(RPR_ABI_VERSION).to_vec();
            bytes.extend(
                snapshot_options()
                    .serialize(&get(world)?.0)
                    .map_err(|e| invalid(e.to_string()))?,
            );
            output(out, Box::into_raw(Box::new(RprBytes(bytes))))
        })
    })
}

/// Restore ONLY trusted snapshots produced by the identical Rapier build. Snapshots are not a
/// stable file format.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_deserialize_world(data: *const u8, count: usize) -> *mut RprWorld {
    ffi_value(|out: *mut *mut RprWorld| {
        ffi(|| unsafe {
            out_ptr(out)?;
            ensure(count <= 256 * 1024 * 1024, "snapshot exceeds 256 MiB")?;
            let bytes = input(data, count)?;
            ensure(
                bytes.starts_with(&snapshot_header(RPR_ABI_VERSION)),
                "incompatible snapshot",
            )?;
            let world = snapshot_options()
                .deserialize(&bytes[SNAPSHOT_HEADER_LEN..])
                .map_err(|e| invalid(e.to_string()))?;
            output(
                out,
                Box::into_raw(Box::new(RprWorld::new(RprPhysicsWorld(world)))),
            )
        })
    })
}

/// World-space line segment produced by physics debug rendering.
/// @ingroup events
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprDebugLine {
    /// World-space start point.
    pub a: RprVector,
    /// World-space end point.
    pub b: RprVector,
    /// RGBA color, four floats.
    pub color: [f32; 4],
}
struct Lines(Vec<RprDebugLine>);
impl rapier::pipeline::DebugRenderBackend for Lines {
    fn draw_line(
        &mut self,
        _: rapier::pipeline::DebugRenderObject,
        a: Vector,
        b: Vector,
        color: [f32; 4],
    ) {
        self.0.push(RprDebugLine {
            a: a.into(),
            b: b.into(),
            color,
        });
    }
}
/// Color is HSLA (hue in degrees), matching Rapier DebugColor. mode uses DebugRenderMode bits.
/// @see @ref output_buffers
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_debug_render(
    world: *const RprWorld,
    mode: u32,
    buffer: *mut RprDebugLine,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let world: *const RprPhysicsWorld = raw;

            let mode = rapier::pipeline::DebugRenderMode::from_bits(mode)
                .ok_or_else(|| invalid("unknown debug render flags"))?;
            let mut pipeline = rapier::pipeline::DebugRenderPipeline::new(Default::default(), mode);
            let mut lines = Lines(Vec::new());
            get(world)?.0.debug_render(&mut pipeline, &mut lines);
            copy_out(&lines.0, buffer, capacity, count)
        })
    })
}

/// Set the world setting documented by RprSoftBodiesSettings::resweepStrain.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_soft_bodies_set_resweep_strain(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        nonnegative(value)?;
        get_mut(parameters)?.0.soft_bodies.resweep_strain = value;
        Ok(())
    })
}

/// Return the world setting documented by RprSoftBodiesSettings::resweepStrain.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_soft_bodies_resweep_strain(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            output(out, get(parameters)?.0.soft_bodies.resweep_strain)
        })
    })
}

/// Set the world setting documented by RprSoftBodiesSettings::contactStiffening.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_soft_bodies_set_contact_stiffening(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        nonnegative(value)?;
        get_mut(parameters)?.0.soft_bodies.contact_stiffening = value;
        Ok(())
    })
}

/// Return the world setting documented by RprSoftBodiesSettings::contactStiffening.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_soft_bodies_contact_stiffening(world: *const RprWorld) -> RprReal {
    ffi_value(|out: *mut RprReal| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            output(out, get(parameters)?.0.soft_bodies.contact_stiffening)
        })
    })
}

/// Set the world setting documented by RprSoftBodiesSettings::maxExtraSubsteps.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_soft_bodies_set_max_extra_substeps(
    world: *mut RprWorld,
    value: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        get_mut(parameters)?.0.soft_bodies.max_extra_substeps = value;
        Ok(())
    })
}

/// Return the world setting documented by RprSoftBodiesSettings::maxExtraSubsteps.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_soft_bodies_max_extra_substeps(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let parameters: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            output(out, get(parameters)?.0.soft_bodies.max_extra_substeps)
        })
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::authoredVelocityMargin.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_authored_velocity_margin(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .authored_velocity_margin = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::edgeSpeculation.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_edge_speculation(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?.0.soft_bodies.recovery.edge_speculation = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::invertedCellDetection.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_inverted_cell_detection(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .inverted_cell_detection = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::selfCrossingDetection.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_self_crossing_detection(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .self_crossing_detection = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::detectionMotionGating.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_detection_motion_gating(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .detection_motion_gating = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::crossBodyDetection.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_cross_body_detection(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .cross_body_detection = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::selfStandDown.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_self_stand_down(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?.0.soft_bodies.recovery.self_stand_down = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::crossBodyExpelGate.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_cross_body_expel_gate(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .cross_body_expel_gate = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::edgeStandDown.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_edge_stand_down(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?.0.soft_bodies.recovery.edge_stand_down = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::crossingRepulsion.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_crossing_repulsion(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .crossing_repulsion = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::crossingRepulsionGuide.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_crossing_repulsion_guide(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .crossing_repulsion_guide = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::crossingRepulsionSelfGuide.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_crossing_repulsion_self_guide(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .crossing_repulsion_self_guide = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::recoveryPace.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_recovery_pace(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = nonnegative(value)?;
        get_mut(parameters)?.0.soft_bodies.recovery.recovery_pace = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapConstraints.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_constraints(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_constraints = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapRigid.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_rigid(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?.0.soft_bodies.recovery.overlap_rigid = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapSkipSelfTangled.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_skip_self_tangled(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_skip_self_tangled = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapEdgeStandDown.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_edge_stand_down(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_edge_stand_down = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapConstraintPace.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_constraint_pace(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = nonnegative(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_constraint_pace = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapSkinVolume.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_skin_volume(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_skin_volume = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapKeptDepth.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_kept_depth(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = nonnegative(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_kept_depth = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapSelfRegions.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_self_regions(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_self_regions = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapNormalPush.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_normal_push(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_normal_push = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapMultiVolume.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_multi_volume(
    world: *mut RprWorld,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = boolean(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_multi_volume = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftRecoverySettings::overlapProgressMargin.
/// @ingroup soft_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_recovery_set_overlap_progress_margin(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        let value = nonnegative(value)?;
        get_mut(parameters)?
            .0
            .soft_bodies
            .recovery
            .overlap_progress_margin = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftFemParameters::linearTolerance.
/// @ingroup soft_bodies
#[cfg(feature = "fem")]
#[rapier_export]
pub unsafe extern "C" fn rpr_fem_set_linear_tolerance(
    world: *mut RprWorld,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        positive(value)?;
        get_mut(parameters)?.0.soft_bodies.fem.linear_tolerance = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftFemParameters::maxLinearIterations.
/// @ingroup soft_bodies
#[cfg(feature = "fem")]
#[rapier_export]
pub unsafe extern "C" fn rpr_fem_set_max_linear_iterations(
    world: *mut RprWorld,
    value: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        ensure(value > 0, "iterations must be positive")?;
        get_mut(parameters)?.0.soft_bodies.fem.max_linear_iterations = value;
        Ok(())
    })
}

/// Set the world setting documented by RprSoftFemParameters::maxDenseDofs.
/// @ingroup soft_bodies
#[cfg(feature = "fem")]
#[rapier_export]
pub unsafe extern "C" fn rpr_fem_set_max_dense_dofs(
    world: *mut RprWorld,
    value: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let parameters: *mut NativeIntegrationParameters =
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast();

        get_mut(parameters)?.0.soft_bodies.fem.max_dense_dofs = value;
        Ok(())
    })
}

/// Configures a dedicated pool for this world's parallel work. Zero selects Rayon's default.
/// Takes effect on the next step. Reconfiguration must not race with a step or callback.
/// Returns RPR_UNSUPPORTED in builds without the parallel feature; keeps the previous
/// pool when constructing the new one fails. The pool is not included in snapshots.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_num_threads(
    world: *mut RprWorld,
    num_threads: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let world: *mut RprPhysicsWorld = raw;

        let world = get_mut(world)?;
        #[cfg(feature = "parallel")]
        {
            world
                .0
                .configure_thread_pool(num_threads)
                .map_err(|e| invalid(e.to_string()))
        }
        #[cfg(not(feature = "parallel"))]
        {
            let _ = (world, num_threads);
            Err((
                RPR_UNSUPPORTED,
                "thread pools require a Rapier library built with the parallel feature".into(),
            ))
        }
    })
}

/// Removes the world's dedicated pool. A parallel build then uses the calling
/// context's Rayon pool (normally the global pool), not a single worker.
/// Returns RPR_UNSUPPORTED in a build without the parallel feature.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_clear_thread_pool(world: *mut RprWorld) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let world: *mut RprPhysicsWorld = raw;

        let world = get_mut(world)?;
        #[cfg(feature = "parallel")]
        {
            world.0.clear_thread_pool();
            Ok(())
        }
        #[cfg(not(feature = "parallel"))]
        {
            let _ = world;
            Err((
                RPR_UNSUPPORTED,
                "thread pools require a Rapier library built with the parallel feature".into(),
            ))
        }
    })
}

/// Size of the world's dedicated pool, or zero if a parallel build has no dedicated
/// pool configured. Returns one for a build without the parallel feature.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_num_threads(world: *const RprWorld) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let world: *const RprPhysicsWorld = raw;

            let world = get(world)?;
            #[cfg(feature = "parallel")]
            let count = world.0.num_threads().unwrap_or(0);
            #[cfg(not(feature = "parallel"))]
            let count = {
                let _ = world;
                1
            };
            output(out, count)
        })
    })
}

/// Enable or disable the native pipeline profiling counters. Enabling returns
/// RPR_UNSUPPORTED if the library was built without the profiler feature.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_counters_enabled(
    world: *mut RprWorld,
    enabled: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        let pipeline: *mut RprPhysicsPipeline =
            std::ptr::addr_of_mut!((*raw).0.physics_pipeline).cast();

        let enabled = boolean(enabled)?;
        let counters = &mut get_mut(pipeline)?.0.counters;
        if enabled && !cfg!(feature = "profiler") {
            return Err((
                RPR_UNSUPPORTED,
                "profiling requires the profiler feature".into(),
            ));
        }
        if enabled {
            counters.enable();
        } else {
            counters.disable();
        }
        Ok(())
    })
}

/// Native engine time of the most recent step, in milliseconds, as in the Rust testbed.
/// Enable counters before stepping. Excludes C callbacks outside the step, rendering,
/// and dispatch into a dedicated thread pool; remains unchanged while paused.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_step_time_ms(world: *const RprWorld) -> f64 {
    ffi_value(|out: *mut f64| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let pipeline: *const RprPhysicsPipeline =
                std::ptr::addr_of!((*raw).0.physics_pipeline).cast();
            output(out, get(pipeline)?.0.counters.step_time_ms())
        })
    })
}

/// Read ONLY trusted legacy rigid-world snapshots from debug_deserialize3.rs,
/// produced by the identical Rapier build. This is not a stable interchange format.

/// Import trusted legacy Rust testbed rigid-state bytes into a new owned world. Release with
/// rpr_free_world; see @ref snapshots.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_deserialize_rigid_state(
    data: *const u8,
    count: usize,
) -> *mut RprWorld {
    ffi_value(|out: *mut *mut RprWorld| {
        #[derive(serde::Deserialize)]
        struct PhysicsState {
            gravity: Vector,
            integration_parameters: IntegrationParameters,
            islands: IslandManager,
            broad_phase: BroadPhaseBvh,
            narrow_phase: NarrowPhase,
            bodies: RigidBodySet,
            colliders: ColliderSet,
            impulse_joints: ImpulseJointSet,
            multibody_joints: MultibodyJointSet,
        }
        ffi(|| unsafe {
            out_ptr(out)?;
            ensure(count <= 256 * 1024 * 1024, "snapshot exceeds 256 MiB")?;
            let bytes = input(data, count)?;
            let state: PhysicsState = bincode::DefaultOptions::new()
                .with_fixint_encoding()
                .allow_trailing_bytes()
                .with_limit(256 * 1024 * 1024)
                .deserialize(bytes)
                .map_err(|e| invalid(e.to_string()))?;
            let mut world = PhysicsWorld::new();
            world.gravity = state.gravity;
            world.integration_parameters = state.integration_parameters;
            world.islands = state.islands;
            world.broad_phase = state.broad_phase;
            world.narrow_phase = state.narrow_phase;
            world.bodies = state.bodies;
            world.colliders = state.colliders;
            world.impulse_joints = state.impulse_joints;
            world.multibody_joints = state.multibody_joints;
            output(
                out,
                Box::into_raw(Box::new(RprWorld::new(RprPhysicsWorld(world)))),
            )
        })
    })
}

#[cfg(test)]
mod snapshot_header_tests {
    use super::*;

    #[test]
    fn snapshot_header_preserves_full_abi_version() {
        for version in [1, 255, 256, 257, 0x0102_0304, u32::MAX] {
            let header = snapshot_header(version);
            assert_eq!(&header[..4], b"RPRS");
            assert_eq!(
                u32::from_le_bytes(header[4..8].try_into().unwrap()),
                version
            );
            assert_eq!(
                u32::from_le_bytes(header[8..12].try_into().unwrap()),
                rpr_build_info().dimension
            );
            assert_eq!(
                u32::from_le_bytes(header[12..16].try_into().unwrap()),
                rpr_build_info().real_size
            );
        }
        assert_eq!(&snapshot_header(0x0102_0304)[4..8], &[4, 3, 2, 1]);
        assert_ne!(snapshot_header(1), snapshot_header(257));
    }

    #[test]
    fn snapshots_reject_incompatible_and_truncated_headers() {
        unsafe {
            let world = rpr_new_world();
            let snapshot = rpr_serialize_world(world);
            assert_eq!(rpr_last_status(), RPR_OK);
            let data = &(*snapshot).0;
            let header = snapshot_header(RPR_ABI_VERSION);
            assert!(data.starts_with(&header));

            let restored = rpr_deserialize_world(data.as_ptr(), data.len());
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!(rpr_free_world(restored), RPR_OK);

            // Every header byte is checked, including the high ABI bytes that
            // previously disappeared in the u8 cast (e.g. ABI 1 versus 257).
            for index in 0..SNAPSHOT_HEADER_LEN {
                let mut invalid = data.clone();
                invalid[index] ^= 1;
                assert!(rpr_deserialize_world(invalid.as_ptr(), invalid.len()).is_null());
                assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
            }
            for len in 0..=SNAPSHOT_HEADER_LEN {
                assert!(rpr_deserialize_world(data.as_ptr(), len).is_null());
                assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
            }

            let mut legacy = vec![b'R', b'P', b'R', 1, header[8], header[12]];
            legacy.extend_from_slice(&data[SNAPSHOT_HEADER_LEN..]);
            assert!(rpr_deserialize_world(legacy.as_ptr(), legacy.len()).is_null());
            assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
            assert_eq!(rpr_free_bytes(snapshot), RPR_OK);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }
}
