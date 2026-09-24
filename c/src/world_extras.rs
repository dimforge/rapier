//! World diagnostics and settings, controller axes, ABI feature checks, and deterministic math.
use crate::*;
use rapier::na::{ComplexField, RealField};

/// @ingroup controllers
/// Controller axis bit: translation along X.
pub const RPR_AXES_MASK_LIN_X: u32 = 1;
/// @ingroup controllers
/// Controller axis bit: translation along Y.
pub const RPR_AXES_MASK_LIN_Y: u32 = 2;
/// @ingroup controllers
/// Controller axis bit: translation along Z.
#[cfg(feature = "dim3")]
pub const RPR_AXES_MASK_LIN_Z: u32 = 4;
/// @ingroup controllers
/// Controller axis bit: rotation about X.
#[cfg(feature = "dim3")]
pub const RPR_AXES_MASK_ANG_X: u32 = 8;
/// @ingroup controllers
/// Controller axis bit: rotation about Y.
#[cfg(feature = "dim3")]
pub const RPR_AXES_MASK_ANG_Y: u32 = 16;
/// @ingroup controllers
/// Controller axis bit: rotation about Z (the only rotation axis in 2D).
pub const RPR_AXES_MASK_ANG_Z: u32 = 32;

/// @ingroup errors
/// ABI feature bit: RAPIER_FEM, which changes the layout of RprIntegrationParameters.
pub const RPR_ABI_FEATURE_FEM: u32 = 1;
/// @ingroup errors
/// ABI feature bit: RAPIER_ROBOTICS (3D, f32 only), which declares the URDF/MJCF API.
pub const RPR_ABI_FEATURE_ROBOTICS: u32 = 2;
/// @ingroup errors
/// RPR_ABI_FEATURE_* bits selected by the defines of this header; pass it to CheckAbi.
#[cfg(all(
    feature = "fem",
    feature = "robotics",
    feature = "dim3",
    feature = "f32"
))]
pub const RPR_ABI_FEATURES: u32 = 3;
/// @ingroup errors
/// RPR_ABI_FEATURE_* bits selected by the defines of this header; pass it to CheckAbi.
#[cfg(all(
    feature = "fem",
    not(all(feature = "robotics", feature = "dim3", feature = "f32"))
))]
pub const RPR_ABI_FEATURES: u32 = 1;
/// @ingroup errors
/// RPR_ABI_FEATURE_* bits selected by the defines of this header; pass it to CheckAbi.
#[cfg(all(
    not(feature = "fem"),
    feature = "robotics",
    feature = "dim3",
    feature = "f32"
))]
pub const RPR_ABI_FEATURES: u32 = 2;
/// @ingroup errors
/// RPR_ABI_FEATURE_* bits selected by the defines of this header; pass it to CheckAbi.
#[cfg(all(
    not(feature = "fem"),
    not(all(feature = "robotics", feature = "dim3", feature = "f32"))
))]
pub const RPR_ABI_FEATURES: u32 = 0;

/// Copy the rigid bodies quarantined by the most recent Step because their pose or velocity became
/// non-finite (NaN or infinite). Rapier disabled them, restored their last valid pose when known,
/// and zeroed their velocities and forces; re-enable them with RigidBody_SetEnabled once the cause
/// is fixed. The list is cleared at the start of every Step and may hold handles removed since.
/// @see @ref output_buffers
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_quarantined_rigid_bodies(
    world: *const RprWorld,
    buffer: *mut RprRigidBodyHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();
                let values: Vec<RprRigidBodyHandle> = (*raw)
                    .0
                    .quarantine()
                    .bodies()
                    .iter()
                    .map(|h| (*h).into())
                    .collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    }
}

/// Copy the colliders quarantined by the most recent Step because their own pose or shape became
/// non-finite, independently of their parent. Rapier disabled them; re-enable them with
/// Collider_SetEnabled once fixed. The list is cleared at the start of every Step and may hold
/// handles removed since.
/// @see @ref output_buffers
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_quarantined_colliders(
    world: *const RprWorld,
    buffer: *mut RprColliderHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();
                let values: Vec<RprColliderHandle> = (*raw)
                    .0
                    .quarantine()
                    .colliders()
                    .iter()
                    .map(|h| (*h).into())
                    .collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    }
}

/// Copy the soft bodies quarantined by the most recent Step because a particle position or velocity
/// became non-finite. Rapier disabled them and zeroed their velocities but left the non-finite
/// positions: fix them with SoftBody_SetParticlePosition before SoftBody_SetEnabled. The list is
/// cleared at the start of every Step and may hold handles removed since.
/// @see @ref output_buffers
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_quarantined_soft_bodies(
    world: *const RprWorld,
    buffer: *mut RprSoftBodyHandle,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();
                let values: Vec<RprSoftBodyHandle> = (*raw)
                    .0
                    .quarantine()
                    .soft_bodies()
                    .iter()
                    .map(|h| (*h).into())
                    .collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    }
}

/// Return the world setting documented by RprIntegrationParameters::frictionModel.
/// @ingroup worlds
#[cfg(feature = "dim3")]
#[rapier_export]
pub unsafe extern "C" fn rpr_friction_model(world: *const RprWorld) -> u32 {
    ffi_value(|out: *mut u32| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();
            let model = (*raw).0.integration_parameters.friction_model;
            output(out, friction_model_value(model))
        })
    })
}

/// Set the world setting documented by RprIntegrationParameters::frictionModel.
/// @ingroup worlds
#[cfg(feature = "dim3")]
#[rapier_export]
pub unsafe extern "C" fn rpr_set_friction_model(world: *mut RprWorld, value: u32) -> RprStatus {
    ffi(|| unsafe {
        let model = friction_model(value)?;
        let access = get(world)?.write()?;
        (*access.raw()).0.integration_parameters.friction_model = model;
        Ok(())
    })
}

/// Sine of an angle in radians, computed by Rapier's math backend. With enhanced-determinism
/// (see BuildFeatures), the result is identical on every platform.
/// @ingroup math
#[rapier_export]
pub extern "C" fn rpr_sin(x: RprReal) -> RprReal {
    ComplexField::sin(x)
}
/// Cosine of an angle in radians, computed by Rapier's math backend. See rpr_sin.
/// @ingroup math
#[rapier_export]
pub extern "C" fn rpr_cos(x: RprReal) -> RprReal {
    ComplexField::cos(x)
}
/// Tangent of an angle in radians, computed by Rapier's math backend. See rpr_sin.
/// @ingroup math
#[rapier_export]
pub extern "C" fn rpr_tan(x: RprReal) -> RprReal {
    ComplexField::tan(x)
}
/// Arcsine in radians, computed by Rapier's math backend. See rpr_sin.
/// @ingroup math
#[rapier_export]
pub extern "C" fn rpr_asin(x: RprReal) -> RprReal {
    ComplexField::asin(x)
}
/// Arccosine in radians, computed by Rapier's math backend. See rpr_sin.
/// @ingroup math
#[rapier_export]
pub extern "C" fn rpr_acos(x: RprReal) -> RprReal {
    ComplexField::acos(x)
}
/// Angle in radians of the point (x, y), in [-pi, pi], computed by Rapier's math backend. See
/// rpr_sin.
/// @ingroup math
#[rapier_export]
pub extern "C" fn rpr_atan2(y: RprReal, x: RprReal) -> RprReal {
    RealField::atan2(y, x)
}
/// Exponential e^x, computed by Rapier's math backend. See rpr_sin.
/// @ingroup math
#[rapier_export]
pub extern "C" fn rpr_exp(x: RprReal) -> RprReal {
    ComplexField::exp(x)
}
/// Natural logarithm, computed by Rapier's math backend. See rpr_sin.
/// @ingroup math
#[rapier_export]
pub extern "C" fn rpr_ln(x: RprReal) -> RprReal {
    ComplexField::ln(x)
}
/// base raised to the power exponent, computed by Rapier's math backend. See rpr_sin.
/// @ingroup math
#[rapier_export]
pub extern "C" fn rpr_powf(base: RprReal, exponent: RprReal) -> RprReal {
    ComplexField::powf(base, exponent)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{ffi::c_void, ptr};

    unsafe fn insert_body(
        world: *mut RprWorld,
        desc: RprRigidBodyDesc,
        at: Vector,
    ) -> RprRigidBodyHandle {
        let mut desc = desc;
        desc.position.translation = at.into();
        let handle = unsafe { rpr_insert_rigid_body(world, &desc) };
        assert_eq!(rpr_last_status(), RPR_OK);
        handle
    }

    unsafe fn insert_collider(
        body: RprRigidBodyHandle,
        desc: RprColliderDesc,
    ) -> RprColliderHandle {
        let handle = unsafe { rpr_insert_collider(body, &desc) };
        assert_eq!(rpr_last_status(), RPR_OK);
        handle
    }

    #[test]
    fn quarantine_reports_the_last_step_only() {
        unsafe {
            let world = rpr_new_world();
            let body = insert_body(world, rpr_dynamic_rigid_body_desc(), Vector::ZERO);
            insert_collider(body, rpr_ball_collider_desc(0.5));
            {
                // The C API rejects non-finite velocities, so poison the body natively.
                let access = get(world).unwrap().write().unwrap();
                let bodies = &mut (*access.raw()).0.bodies;
                bodies[body.raw()].set_linvel(Vector::splat(Real::NAN), true);
            }
            assert_eq!(rpr_quarantined_rigid_bodies(world, ptr::null_mut(), 0), 0);
            assert_eq!(rpr_step(world, ptr::null(), ptr::null()), RPR_OK);
            let mut buffer = [RprRigidBodyHandle::default(); 2];
            assert_eq!(
                rpr_quarantined_rigid_bodies(world, buffer.as_mut_ptr(), 2),
                1
            );
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!(buffer[0], body);
            assert_eq!(rpr_quarantined_colliders(world, ptr::null_mut(), 0), 0);
            assert_eq!(rpr_quarantined_soft_bodies(world, ptr::null_mut(), 0), 0);
            assert_eq!(rpr_step(world, ptr::null(), ptr::null()), RPR_OK);
            assert_eq!(rpr_quarantined_rigid_bodies(world, ptr::null_mut(), 0), 0);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[test]
    fn solver_iterations_are_validated_consistently() {
        unsafe {
            let world = rpr_new_world();
            let mut params = rpr_integration_parameters(world);
            params.numSolverIterations = 0;
            assert_eq!(
                rpr_set_integration_parameters(world, &params),
                RPR_INVALID_ARGUMENT
            );
            params.numSolverIterations = 4;
            params.numInternalPgsIterations = 0;
            assert_eq!(
                rpr_set_integration_parameters(world, &params),
                RPR_INVALID_ARGUMENT
            );
            assert_eq!(
                rpr_set_num_solver_iterations(world, 0),
                RPR_INVALID_ARGUMENT
            );
            assert_eq!(
                rpr_set_num_internal_pgs_iterations(world, 0),
                RPR_INVALID_ARGUMENT
            );
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[cfg(feature = "dim3")]
    #[test]
    fn friction_model_round_trips() {
        unsafe {
            let world = rpr_new_world();
            assert_eq!(rpr_friction_model(world), RPR_FRICTION_MODEL_SIMPLIFIED);
            assert_eq!(
                rpr_set_friction_model(world, RPR_FRICTION_MODEL_COULOMB),
                RPR_OK
            );
            assert_eq!(rpr_friction_model(world), RPR_FRICTION_MODEL_COULOMB);
            assert_eq!(
                rpr_integration_parameters(world).frictionModel,
                RPR_FRICTION_MODEL_COULOMB
            );
            assert_eq!(rpr_set_friction_model(world, 2), RPR_INVALID_ARGUMENT);
            assert_eq!(rpr_friction_model(world), RPR_FRICTION_MODEL_COULOMB);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[test]
    fn check_abi_rejects_a_feature_define_mismatch() {
        unsafe {
            let sizes = (
                size_of::<Real>(),
                size_of::<RprVector>(),
                size_of::<RprPose>(),
            );
            let dim = rapier::math::DIM as u32;
            let check =
                |features| rpr_check_abi(RPR_ABI_VERSION, dim, sizes.0, sizes.1, sizes.2, features);
            assert_eq!(check(RPR_ABI_FEATURES), RPR_OK);
            assert_eq!(
                check(RPR_ABI_FEATURES ^ RPR_ABI_FEATURE_FEM),
                RPR_INVALID_ARGUMENT
            );
            assert_eq!(
                check(RPR_ABI_FEATURES ^ RPR_ABI_FEATURE_ROBOTICS),
                RPR_INVALID_ARGUMENT
            );
            assert_eq!(check(RPR_ABI_FEATURES | 64), RPR_INVALID_ARGUMENT);
            assert_eq!(
                RPR_ABI_FEATURES & RPR_ABI_FEATURE_FEM != 0,
                cfg!(feature = "fem")
            );
        }
    }

    #[test]
    fn math_functions_match_the_simulation_backend() {
        let x: Real = 0.7;
        assert_eq!(rpr_sin(x), ComplexField::sin(x));
        assert_eq!(rpr_cos(x), ComplexField::cos(x));
        assert_eq!(rpr_tan(x), ComplexField::tan(x));
        assert_eq!(rpr_asin(x), ComplexField::asin(x));
        assert_eq!(rpr_acos(x), ComplexField::acos(x));
        assert_eq!(rpr_atan2(1.0, -1.0), 3.0 * Real::frac_pi_4());
        assert_eq!(rpr_exp(0.0), 1.0);
        assert_eq!(rpr_ln(1.0), 0.0);
        assert_eq!(rpr_powf(2.0, 10.0), 1024.0);
    }

    #[test]
    fn pd_controller_defaults_and_correction() {
        unsafe {
            let pd = rpr_default_pd_controller();
            assert!(pd.raw().unwrap() == rapier::control::PdController::default());
            let mut bodies = RprRigidBodySet(RigidBodySet::new());
            let handle = bodies.0.insert(RigidBodyBuilder::dynamic());
            let mut only_x = pd;
            only_x.axes = RPR_AXES_MASK_LIN_X;
            let mut linear = RprVector::default();
            let mut angular = RprAngVector::default();
            let target = Pose::from_translation(Vector::X + Vector::Y);
            let zero = angular_out(AngVector::default());
            assert_eq!(
                native_pd_controller_rigid_body_correction(
                    &only_x,
                    &bodies,
                    handle.into(),
                    target.into(),
                    Vector::ZERO.into(),
                    zero,
                    &mut linear,
                    &mut angular,
                ),
                RPR_OK
            );
            assert!(linear.x > 0.0);
            assert_eq!(linear.y, 0.0);
            let mut invalid = pd;
            invalid.axes = 64;
            assert_eq!(
                native_pd_controller_rigid_body_correction(
                    &invalid,
                    &bodies,
                    handle.into(),
                    target.into(),
                    Vector::ZERO.into(),
                    zero,
                    &mut linear,
                    &mut angular,
                ),
                RPR_INVALID_ARGUMENT
            );
        }
    }

    #[test]
    fn pid_axes_and_integrals() {
        unsafe {
            let pid = rpr_new_pid_controller();
            let all = RPR_AXES_MASK_LIN_X | RPR_AXES_MASK_LIN_Y | RPR_AXES_MASK_ANG_Z;
            #[cfg(feature = "dim3")]
            let all = all | RPR_AXES_MASK_LIN_Z | RPR_AXES_MASK_ANG_X | RPR_AXES_MASK_ANG_Y;
            assert_eq!(rpr_pid_controller_axes(pid), all);
            assert_eq!(
                rpr_pid_controller_set_axes(pid, RPR_AXES_MASK_LIN_Y),
                RPR_OK
            );
            assert_eq!(rpr_pid_controller_axes(pid), RPR_AXES_MASK_LIN_Y);
            #[cfg(feature = "dim2")]
            assert_eq!(rpr_pid_controller_set_axes(pid, 4), RPR_INVALID_ARGUMENT);
            (*pid).0.lin_integral = Vector::X;
            assert_eq!(rpr_pid_controller_reset_integrals(pid), RPR_OK);
            assert_eq!((*pid).0.lin_integral, Vector::ZERO);
            assert_eq!(rpr_free_pid_controller(pid), RPR_OK);
        }
    }

    #[test]
    fn character_controller_getters() {
        unsafe {
            let c = rpr_new_kinematic_character_controller();
            let up = rpr_kinematic_character_controller_up(c);
            assert_eq!(up.y, 1.0);
            let offset = rpr_kinematic_character_controller_offset(c);
            assert_eq!((offset.value, offset.relative), (0.01, 1));
            let step = rpr_kinematic_character_controller_autostep(c);
            assert_eq!(step.enabled, 0);
            assert_eq!(step.include_dynamic_bodies, 1);
            let height = RprCharacterLength {
                value: 0.3,
                relative: 0,
            };
            let width = RprCharacterLength {
                value: 0.2,
                relative: 1,
            };
            assert_eq!(
                rpr_kinematic_character_controller_set_autostep(c, 1, height, width, 0),
                RPR_OK
            );
            let step = rpr_kinematic_character_controller_autostep(c);
            assert_eq!((step.enabled, step.include_dynamic_bodies), (1, 0));
            assert_eq!((step.max_height.value, step.max_height.relative), (0.3, 0));
            assert_eq!((step.min_width.value, step.min_width.relative), (0.2, 1));
            assert_eq!(
                rpr_kinematic_character_controller_normal_nudge_factor(c),
                1.0e-4
            );
            assert_eq!(
                rpr_kinematic_character_controller_set_normal_nudge_factor(c, 0.01),
                RPR_OK
            );
            assert_eq!(
                rpr_kinematic_character_controller_normal_nudge_factor(c),
                0.01
            );
            assert_eq!(
                rpr_kinematic_character_controller_set_normal_nudge_factor(c, -1.0),
                RPR_INVALID_ARGUMENT
            );
            assert_eq!(rpr_free_kinematic_character_controller(c), RPR_OK);
        }
    }

    unsafe extern "C" fn reject_all(
        calls: *mut c_void,
        read: *const RprReadContext,
        collider: RprColliderHandle,
    ) -> RprBool {
        unsafe {
            // The world is locked for writing, but its read context stays usable.
            let _ = rpr_read_collider_translation(read, collider);
            assert_eq!(rpr_last_status(), RPR_OK);
            *calls.cast::<usize>() += 1;
        }
        0
    }

    #[test]
    fn character_impulses_honor_the_query_predicate() {
        unsafe {
            let world = rpr_new_world();
            assert_eq!(rpr_set_gravity(world, Vector::ZERO.into()), RPR_OK);
            let box_body = insert_body(world, rpr_dynamic_rigid_body_desc(), Vector::X * 2.0);
            insert_collider(
                box_body,
                rpr_cuboid_collider_desc((Vector::ONE * 0.5).into()),
            );
            assert_eq!(rpr_step(world, ptr::null(), ptr::null()), RPR_OK);

            let shape = rpr_ball_shared_shape(0.5);
            let c = rpr_new_kinematic_character_controller();
            let movement = rpr_kinematic_character_controller_move_shape(
                world,
                ptr::null(),
                c,
                1.0 / 60.0,
                shape,
                Pose::IDENTITY.into(),
                (Vector::X * 2.0).into(),
            );
            assert_eq!(rpr_last_status(), RPR_OK);
            assert!(movement.translation.x < 1.5);

            let mut calls = 0usize;
            let mut options = rpr_default_query_options();
            options.predicate = Some(reject_all);
            options.userData = (&mut calls as *mut usize).cast();
            assert_eq!(
                rpr_kinematic_character_controller_solve_character_collision_impulses(
                    c,
                    shape,
                    1.0 / 60.0,
                    1.0,
                    &options
                ),
                RPR_OK
            );
            assert_eq!(calls, 1);
            assert_eq!(rpr_rigid_body_linvel(box_body).x, 0.0);
            assert_eq!(
                rpr_kinematic_character_controller_solve_character_collision_impulses(
                    c,
                    shape,
                    1.0 / 60.0,
                    1.0,
                    ptr::null()
                ),
                RPR_OK
            );
            assert!(rpr_rigid_body_linvel(box_body).x > 0.0);
            assert_eq!(rpr_free_kinematic_character_controller(c), RPR_OK);
            assert_eq!(rpr_free_shared_shape(shape), RPR_OK);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[cfg(feature = "dim3")]
    unsafe extern "C" fn reject_collider(
        rejected: *mut c_void,
        _read: *const RprReadContext,
        collider: RprColliderHandle,
    ) -> RprBool {
        unsafe { (collider != *rejected.cast::<RprColliderHandle>()) as RprBool }
    }

    #[cfg(feature = "dim3")]
    #[test]
    fn vehicle_keeps_the_user_exclusions_and_the_chassis_exclusion() {
        unsafe {
            let world = rpr_new_world();
            assert_eq!(rpr_set_gravity(world, Vector::ZERO.into()), RPR_OK);
            let ground_body = insert_body(world, rpr_fixed_rigid_body_desc(), Vector::ZERO);
            let ground = insert_collider(
                ground_body,
                rpr_cuboid_collider_desc(Vector::new(10.0, 0.1, 10.0).into()),
            );
            let bump_body = insert_body(world, rpr_fixed_rigid_body_desc(), Vector::Y * 0.3);
            let bump = insert_collider(
                bump_body,
                rpr_cuboid_collider_desc(Vector::new(0.2, 0.05, 0.2).into()),
            );
            let chassis = insert_body(world, rpr_dynamic_rigid_body_desc(), Vector::Y);
            insert_collider(chassis, rpr_ball_collider_desc(0.3));
            assert_eq!(rpr_step(world, ptr::null(), ptr::null()), RPR_OK);

            let vehicle = rpr_new_dynamic_ray_cast_vehicle_controller(chassis);
            let tuning = rpr_default_wheel_tuning();
            rpr_dynamic_ray_cast_vehicle_controller_add_wheel(
                vehicle,
                Vector::ZERO.into(),
                (-Vector::Y).into(),
                Vector::X.into(),
                1.0,
                0.1,
                &tuning,
            );
            assert_eq!(rpr_last_status(), RPR_OK);
            let ground_object = |options: *const RprQueryOptions| {
                assert_eq!(
                    rpr_dynamic_ray_cast_vehicle_controller_update_vehicle(
                        vehicle,
                        1.0 / 60.0,
                        options
                    ),
                    RPR_OK
                );
                let mut wheel = RprWheelState::default();
                assert_eq!(
                    rpr_dynamic_ray_cast_vehicle_controller_wheels(vehicle, &mut wheel, 1),
                    1
                );
                wheel.ground_object
            };
            assert_eq!(ground_object(ptr::null()), bump);
            let mut options = rpr_default_query_options();
            options.filter.exclude_rigid_body = bump_body;
            assert_eq!(ground_object(&options), ground);
            let mut rejected = bump;
            let mut options = rpr_default_query_options();
            options.predicate = Some(reject_collider);
            options.userData = (&mut rejected as *mut RprColliderHandle).cast();
            assert_eq!(ground_object(&options), ground);
            assert_eq!(
                rpr_free_dynamic_ray_cast_vehicle_controller(vehicle),
                RPR_OK
            );
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }
}
