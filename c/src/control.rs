use crate::*;
use rapier::control::{
    CharacterAutostep, CharacterCollision, CharacterLength, KinematicCharacterController,
};
/// Controller plus reusable collision output from the last move_shape call.
/// Kinematic character controller and its last collision list. Release with the matching Free
/// function.
/// @ingroup controllers
pub struct RprKinematicCharacterController {
    world: *mut RprWorld,
    inner: KinematicCharacterController,
    collisions: Vec<CharacterCollision>,
}
/// CharacterLength counterpart: relative=1 scales with character height, relative=0 uses world
/// units.
/// @ingroup controllers
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprCharacterLength {
    /// Value used when enabled is 1.
    pub value: RprReal,
    /// 1 scales value by the character shape size; 0 uses an absolute length.
    pub relative: RprBool,
}
impl RprCharacterLength {
    fn raw(self) -> Result<CharacterLength> {
        nonnegative(self.value)?;
        Ok(if boolean(self.relative)? {
            CharacterLength::Relative(self.value)
        } else {
            CharacterLength::Absolute(self.value)
        })
    }
}
/// Allowed character motion and ground-contact state.
/// @ingroup controllers
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprCharacterMovement {
    /// Allowed world-space displacement; not applied automatically.
    pub translation: RprVector,
    /// Whether the character touches the ground after movement.
    pub grounded: RprBool,
    /// Whether motion includes sliding down a non-climbable slope.
    pub is_sliding_down_slope: RprBool,
}
/// Collision recorded during character movement.
/// @ingroup controllers
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprCharacterCollision {
    /// World-bound collider handle.
    pub collider: RprColliderHandle,
    /// World-space character pose at collision.
    pub character_pos: RprPose,
    /// World-space translation already applied before collision.
    pub translation_applied: RprVector,
    /// World-space translation remaining at collision.
    pub translation_remaining: RprVector,
    /// Shape/ray impact details.
    pub hit: RprShapeCastHit,
}
/// Allocate a character controller with native defaults; release it with
/// rpr_free_kinematic_character_controller.
/// @ingroup controllers
#[rapier_export]
pub unsafe extern "C" fn rpr_new_kinematic_character_controller()
-> *mut RprKinematicCharacterController {
    ffi_value(|out: *mut *mut RprKinematicCharacterController| {
        ffi(|| unsafe {
            out_ptr(out)?;
            output(
                out,
                Box::into_raw(Box::new(RprKinematicCharacterController {
                    world: std::ptr::null_mut(),
                    inner: Default::default(),
                    collisions: Vec::new(),
                })),
            )
        })
    })
}
/// Release an owned kinematic character controller. NULL is allowed. Do not pass borrowed pointers
/// or free the object twice.
/// @ingroup controllers
#[rapier_export]
pub unsafe extern "C" fn rpr_free_kinematic_character_controller(
    controller: *mut RprKinematicCharacterController,
) -> RprStatus {
    ffi(|| unsafe {
        if !controller.is_null() {
            get(controller)?;
            drop(Box::from_raw(controller));
        }
        Ok(())
    })
}
/// Set the up direction; it must be finite and nonzero and is normalized on input.
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_set_up(
    controller: *mut RprKinematicCharacterController,
    up: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let v = up.raw()?;
        positive(v.length())?;
        get_mut(controller)?.inner.up = v.normalize();
        Ok(())
    })
}
/// Set the collision separation margin; use a positive absolute or relative character length.
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_set_offset(
    controller: *mut RprKinematicCharacterController,
    offset: RprCharacterLength,
) -> RprStatus {
    ffi(|| unsafe {
        positive(offset.value)?;
        let offset = offset.raw()?;
        get_mut(controller)?.inner.offset = offset;
        Ok(())
    })
}
/// Enable or disable sliding along obstacles.
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_set_slide(
    controller: *mut RprKinematicCharacterController,
    enabled: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let v = boolean(enabled)?;
        get_mut(controller)?.inner.slide = v;
        Ok(())
    })
}
/// Set the maximum climb angle and minimum slide angle, in radians.
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_set_slopes(
    controller: *mut RprKinematicCharacterController,
    max_climb_angle: RprReal,
    min_slide_angle: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        for a in [max_climb_angle, min_slide_angle] {
            nonnegative(a)?;
            ensure(a <= 2.0 * real_pi(), "angle must be <= 2*pi")?;
        }
        let c = get_mut(controller)?;
        c.inner.max_slope_climb_angle = max_climb_angle;
        c.inner.min_slope_slide_angle = min_slide_angle;
        Ok(())
    })
}
/// Configure automatic stepping over obstacles. enabled = 0 disables it.
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_set_autostep(
    controller: *mut RprKinematicCharacterController,
    enabled: RprBool,
    max_height: RprCharacterLength,
    min_width: RprCharacterLength,
    include_dynamic_bodies: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let step = CharacterAutostep {
            max_height: max_height.raw()?,
            min_width: min_width.raw()?,
            include_dynamic_bodies: boolean(include_dynamic_bodies)?,
        };
        let value = boolean(enabled)?.then_some(step);
        get_mut(controller)?.inner.autostep = value;
        Ok(())
    })
}
/// Configure downward ground snapping. enabled = 0 disables it.
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_set_snap_to_ground(
    controller: *mut RprKinematicCharacterController,
    enabled: RprBool,
    distance: RprCharacterLength,
) -> RprStatus {
    ffi(|| unsafe {
        let d = distance.raw()?;
        let value = boolean(enabled)?.then_some(d);
        get_mut(controller)?.inner.snap_to_ground = value;
        Ok(())
    })
}
/// Computes movement without moving any collider. Use the returned translation to set the character
/// target.
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_move_shape(
    world: *const RprWorld,
    options: *const RprQueryOptions,
    controller: *mut RprKinematicCharacterController,
    dt: RprReal,
    shape: *const RprSharedShape,
    pose: RprPose,
    desired_translation: RprVector,
) -> RprCharacterMovement {
    ffi_value(|out: *mut RprCharacterMovement| {
        ffi(|| unsafe {
            if !options.is_null() {
                get(options)?.check_world(world)?;
            }
            let access = get(world)?.read()?;
            let raw = access.raw();
            let query = QueryAccess::from_world(world as *mut RprWorld, raw, options)?;
            let query: *const QueryAccess = &query;

            out_ptr(out)?;
            positive(dt)?;
            let pose = pose.raw()?;
            let translation = desired_translation.raw()?;
            let shape = &*get(shape)?.0;
            let c = get_mut(controller)?;
            c.world = world as *mut RprWorld;
            c.collisions.clear();
            let result = get(query)?.with_raw(|q| {
                Ok(c.inner
                    .move_shape(dt, &q, shape, &pose, translation, |hit| {
                        c.collisions.push(hit)
                    }))
            })?;
            output(
                out,
                RprCharacterMovement {
                    translation: result.translation.into(),
                    grounded: result.grounded as u32,
                    is_sliding_down_slope: result.is_sliding_down_slope as u32,
                },
            )
        })
    })
}

/// Copy collisions recorded by the most recent MoveShape call.
/// @see @ref output_buffers
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_collisions(
    controller: *const RprKinematicCharacterController,
    buffer: *mut RprCharacterCollision,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(
            get(controller).map_or(std::ptr::null_mut(), |c| c.world),
            buffer,
            capacity,
            |count: *mut usize| {
                ffi(|| {
                    let values: Vec<_> = get(controller)?
                        .collisions
                        .iter()
                        .map(|c| RprCharacterCollision {
                            collider: c.handle.into(),
                            character_pos: c.character_pos.into(),
                            translation_applied: c.translation_applied.into(),
                            translation_remaining: c.translation_remaining.into(),
                            hit: RprShapeCastHit {
                                collider: c.handle.into(),
                                time_of_impact: c.hit.time_of_impact,
                                witness1: c.hit.witness1.into(),
                                witness2: c.hit.witness2.into(),
                                normal1: c.hit.normal1.into(),
                                normal2: c.hit.normal2.into(),
                                status: c.hit.status as u32,
                            },
                        })
                        .collect();
                    copy_out(&values, buffer, capacity, count)
                })
            },
        )
    }
}
/// Applies impulses for the most recent move_shape collisions. Use the same world, shape, dt and
/// filter.
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_solve_character_collision_impulses(
    controller: *const RprKinematicCharacterController,
    shape: *const RprSharedShape,
    dt: RprReal,
    mass: RprReal,
    filter: *const RprQueryFilter,
) -> RprStatus {
    ffi(|| unsafe {
        let world = get(controller)?.world;
        if !filter.is_null() {
            get(filter)?.check_world(world)?;
        }
        let access = get(world)?.write()?;
        let raw = access.raw();

        let world: *mut RprPhysicsWorld = raw;

        positive(dt)?;
        positive(mass)?;
        let f = if filter.is_null() {
            RprQueryFilter::default()
        } else {
            *get(filter)?
        }
        .raw()?;
        let c = get(controller)?;
        let shape = &*get(shape)?.0;
        let w = &mut get_mut(world)?.0;
        let mut q = w.broad_phase.as_query_pipeline_mut(
            w.narrow_phase.query_dispatcher(),
            &mut w.bodies,
            &mut w.colliders,
            f,
        );
        c.inner
            .solve_character_collision_impulses(dt, &mut q, shape, mass, &c.collisions);
        Ok(())
    })
}

#[cfg(feature = "dim3")]
mod vehicle {
    use super::*;
    use rapier::control::{DynamicRayCastVehicleController, WheelTuning};
    /// Vehicle controller borrowing its chassis world. Release with the matching Free function.
    /// @ingroup controllers
    pub struct RprDynamicRayCastVehicleController(DynamicRayCastVehicleController, *mut RprWorld);
    #[repr(C)]
    #[derive(Copy, Clone, Default)]
    /// Wheel suspension/friction parameters. Initialize with rpr_default_wheel_tuning.
    /// @ingroup controllers
    pub struct RprWheelTuning {
        /// Nonnegative suspension spring stiffness.
        pub suspension_stiffness: RprReal,
        /// Nonnegative damping coefficient during suspension compression.
        pub suspension_compression: RprReal,
        /// Nonnegative suspension relaxation damping.
        pub suspension_damping: RprReal,
        /// Maximum suspension travel in length units.
        pub max_suspension_travel: RprReal,
        /// Maximum tire friction/slip coefficient.
        pub friction_slip: RprReal,
        /// Maximum force exerted by suspension.
        pub max_suspension_force: RprReal,
        /// Sideways tire friction stiffness.
        pub side_friction_stiffness: RprReal,
    }
    impl RprWheelTuning {
        fn raw(self) -> Result<WheelTuning> {
            Ok(WheelTuning {
                suspension_stiffness: nonnegative(self.suspension_stiffness)?,
                suspension_compression: nonnegative(self.suspension_compression)?,
                suspension_damping: nonnegative(self.suspension_damping)?,
                max_suspension_travel: nonnegative(self.max_suspension_travel)?,
                friction_slip: nonnegative(self.friction_slip)?,
                max_suspension_force: nonnegative(self.max_suspension_force)?,
                side_friction_stiffness: nonnegative(self.side_friction_stiffness)?,
            })
        }
    }
    #[repr(C)]
    #[derive(Copy, Clone, Default)]
    /// Copy of the current wheel pose, suspension, and contact state.
    /// @ingroup controllers
    pub struct RprWheelState {
        /// World-space wheel center.
        pub center: RprVector,
        /// World-space suspension direction.
        pub suspension: RprVector,
        /// World-space axle direction.
        pub axle: RprVector,
        /// Wheel rotation angle in radians.
        pub rotation: RprReal,
        /// Current suspension force.
        pub suspension_force: RprReal,
        /// Current suspension length.
        pub suspension_length: RprReal,
        /// Whether the wheel has ground contact.
        pub is_in_contact: RprBool,
        /// Ground collider handle, invalid when there is no contact.
        pub ground_object: RprColliderHandle,
        /// World-space ground contact point.
        pub contact_point: RprVector,
        /// World-space ground contact normal.
        pub contact_normal: RprVector,
    }
    /// Return native default wheel tuning. This POD value owns no resources.
    /// @ingroup controllers
    #[rapier_export]
    pub extern "C" fn rpr_default_wheel_tuning() -> RprWheelTuning {
        let t = WheelTuning::default();
        RprWheelTuning {
            suspension_stiffness: t.suspension_stiffness,
            suspension_compression: t.suspension_compression,
            suspension_damping: t.suspension_damping,
            max_suspension_travel: t.max_suspension_travel,
            friction_slip: t.friction_slip,
            max_suspension_force: t.max_suspension_force,
            side_friction_stiffness: t.side_friction_stiffness,
        }
    }
    /// Allocate a vehicle controller bound to its chassis body. The chassis world must outlive the
    /// controller. Release with rpr_free_dynamic_ray_cast_vehicle_controller.
    /// @ingroup controllers
    #[rapier_export]
    pub unsafe extern "C" fn rpr_new_dynamic_ray_cast_vehicle_controller(
        chassis: RprRigidBodyHandle,
    ) -> *mut RprDynamicRayCastVehicleController {
        let world = chassis.world;
        ffi_value(|out: *mut *mut RprDynamicRayCastVehicleController| {
            ffi(|| unsafe {
                chassis.check_world(world)?;
                let access = get(world)?.read()?;
                let raw = access.raw();

                let bodies: *const RprRigidBodySet = std::ptr::addr_of!((*raw).0.bodies).cast();

                out_ptr(out)?;
                let b = get(bodies)?.0.get(chassis.raw()).ok_or_else(missing)?;
                ensure(
                    b.is_dynamic() && b.soft_body().is_none(),
                    "vehicle chassis must be an ordinary dynamic body",
                )?;
                output(
                    out,
                    Box::into_raw(Box::new(RprDynamicRayCastVehicleController(
                        DynamicRayCastVehicleController::new(chassis.raw()),
                        world,
                    ))),
                )
            })
        })
    }

    /// Release an owned dynamic ray cast vehicle controller. NULL is allowed. Do not pass borrowed
    /// pointers or free the object twice.
    /// @ingroup controllers
    #[rapier_export]
    pub unsafe extern "C" fn rpr_free_dynamic_ray_cast_vehicle_controller(
        controller: *mut RprDynamicRayCastVehicleController,
    ) -> RprStatus {
        ffi(|| unsafe {
            if !controller.is_null() {
                get(controller)?;
                drop(Box::from_raw(controller));
            }
            Ok(())
        })
    }
    /// Append a wheel and return its zero-based index. Connection, suspension direction, and axle
    /// are in chassis-local coordinates.
    /// @ingroup controllers
    #[rapier_export(dynamic_ray_cast_vehicle_controller)]
    pub unsafe extern "C" fn rpr_dynamic_ray_cast_vehicle_controller_add_wheel(
        controller: *mut RprDynamicRayCastVehicleController,
        connection: RprVector,
        direction: RprVector,
        axle: RprVector,
        rest_length: RprReal,
        radius: RprReal,
        tuning: *const RprWheelTuning,
    ) -> usize {
        ffi_value(|out_index: *mut usize| {
            ffi(|| unsafe {
                out_ptr(out_index)?;
                let p = connection.raw()?;
                let d = direction.raw()?;
                let a = axle.raw()?;
                positive(d.length())?;
                positive(a.length())?;
                ensure(
                    d.cross(a).length_squared() > 1.0e-10,
                    "wheel direction and axle must not be parallel",
                )?;
                nonnegative(rest_length)?;
                positive(radius)?;
                let t = get(tuning)?.raw()?;
                let c = &mut get_mut(controller)?.0;
                let index = c.wheels().len();
                c.add_wheel(p, d.normalize(), a.normalize(), rest_length, radius, &t);
                output(out_index, index)
            })
        })
    }
    /// Set the chassis up/forward axis indices (0 = X, 1 = Y, 2 = Z).
    /// @ingroup controllers
    #[rapier_export(dynamic_ray_cast_vehicle_controller)]
    pub unsafe extern "C" fn rpr_dynamic_ray_cast_vehicle_controller_set_axes(
        controller: *mut RprDynamicRayCastVehicleController,
        up: usize,
        forward: usize,
    ) -> RprStatus {
        ffi(|| unsafe {
            ensure(
                up < 3 && forward < 3 && up != forward,
                "invalid vehicle axes",
            )?;
            let c = &mut get_mut(controller)?.0;
            c.index_up_axis = up;
            c.index_forward_axis = forward;
            Ok(())
        })
    }
    /// Set a wheel engine force, brake force, and steering angle in radians.
    /// @ingroup controllers
    #[rapier_export(dynamic_ray_cast_vehicle_controller)]
    pub unsafe extern "C" fn rpr_dynamic_ray_cast_vehicle_controller_set_wheel_controls(
        controller: *mut RprDynamicRayCastVehicleController,
        index: usize,
        steering: RprReal,
        engine_force: RprReal,
        brake: RprReal,
    ) -> RprStatus {
        ffi(|| unsafe {
            finite(steering)?;
            finite(engine_force)?;
            nonnegative(brake)?;
            let w = get_mut(controller)?
                .0
                .wheels_mut()
                .get_mut(index)
                .ok_or_else(|| invalid("wheel index out of range"))?;
            w.steering = steering;
            w.engine_force = engine_force;
            w.brake = brake;
            Ok(())
        })
    }
    /// Ray-cast wheel contacts and apply vehicle forces for dt seconds. Does not step the world.
    /// @ingroup controllers
    #[rapier_export(dynamic_ray_cast_vehicle_controller)]
    pub unsafe extern "C" fn rpr_dynamic_ray_cast_vehicle_controller_update_vehicle(
        controller: *mut RprDynamicRayCastVehicleController,
        dt: RprReal,
        filter: *const RprQueryFilter,
    ) -> RprStatus {
        ffi(|| unsafe {
            let world = get(controller)?.1;
            if !filter.is_null() {
                get(filter)?.check_world(world)?;
            }
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            positive(dt)?;
            let c = &mut get_mut(controller)?.0;
            let mut f = if filter.is_null() {
                RprQueryFilter::default()
            } else {
                *get(filter)?
            }
            .raw()?;
            f.exclude_rigid_body = Some(c.chassis);
            let w = &mut get_mut(world)?.0;
            let chassis = w.bodies.get(c.chassis).ok_or_else(missing)?;
            ensure(
                chassis.is_dynamic() && chassis.soft_body().is_none(),
                "vehicle chassis must be an ordinary dynamic body",
            )?;
            let q = w.broad_phase.as_query_pipeline_mut(
                w.narrow_phase.query_dispatcher(),
                &mut w.bodies,
                &mut w.colliders,
                f,
            );
            c.update_vehicle(dt, q);
            Ok(())
        })
    }

    /// Return signed chassis speed along its forward direction.
    /// @ingroup controllers
    #[rapier_export(dynamic_ray_cast_vehicle_controller)]
    pub unsafe extern "C" fn rpr_dynamic_ray_cast_vehicle_controller_current_vehicle_speed(
        controller: *const RprDynamicRayCastVehicleController,
    ) -> RprReal {
        ffi_value(|out: *mut RprReal| {
            ffi(|| unsafe { output(out, get(controller)?.0.current_vehicle_speed) })
        })
    }
    /// Copy current wheel state in wheel insertion order.
    /// @see @ref output_buffers
    /// @ingroup controllers
    #[rapier_export(dynamic_ray_cast_vehicle_controller)]
    pub unsafe extern "C" fn rpr_dynamic_ray_cast_vehicle_controller_wheels(
        controller: *const RprDynamicRayCastVehicleController,
        buffer: *mut RprWheelState,
        capacity: usize,
    ) -> usize {
        unsafe {
            ffi_world_array(
                get(controller).map_or(std::ptr::null_mut(), |c| c.1),
                buffer,
                capacity,
                |count: *mut usize| {
                    ffi(|| {
                        let v: Vec<_> = get(controller)?
                            .0
                            .wheels()
                            .iter()
                            .map(|w| {
                                let r = w.raycast_info();
                                RprWheelState {
                                    center: w.center().into(),
                                    suspension: w.suspension().into(),
                                    axle: w.axle().into(),
                                    rotation: w.rotation,
                                    suspension_force: w.wheel_suspension_force,
                                    suspension_length: r.suspension_length,
                                    is_in_contact: r.is_in_contact as u32,
                                    ground_object: r
                                        .ground_object
                                        .map(Into::into)
                                        .unwrap_or_default(),
                                    contact_point: r.contact_point_ws.into(),
                                    contact_normal: r.contact_normal_ws.into(),
                                }
                            })
                            .collect();
                        copy_out(&v, buffer, capacity, count)
                    })
                },
            )
        }
    }
}
#[cfg(feature = "dim3")]
pub use vehicle::*;

/// PID controller with persistent integral state.
/// Stateful proportional-integral-derivative controller. Release with the matching Free function.
/// @ingroup controllers
pub struct RprPidController(rapier::control::PidController);

/// Per-axis proportional, integral, and derivative controller gains.
/// @ingroup controllers
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprPidGains {
    /// Linear proportional gain per axis.
    pub lin_kp: RprVector,
    /// Linear integral gain per axis.
    pub lin_ki: RprVector,
    /// Linear derivative gain per axis.
    pub lin_kd: RprVector,
    /// Angular proportional gain per axis.
    pub ang_kp: RprAngVector,
    /// Angular integral gain per axis.
    pub ang_ki: RprAngVector,
    /// Angular derivative gain per axis.
    pub ang_kd: RprAngVector,
}

/// Allocate a PID controller with supplied gains and controlled axes. Release with
/// rpr_free_pid_controller.
/// @ingroup controllers
#[rapier_export]
pub unsafe extern "C" fn rpr_new_pid_controller() -> *mut RprPidController {
    ffi_value(|out: *mut *mut RprPidController| {
        ffi(|| unsafe {
            out_ptr(out)?;
            output(
                out,
                Box::into_raw(Box::new(RprPidController(Default::default()))),
            )
        })
    })
}
/// Release an owned pid controller. NULL is allowed. Do not pass borrowed pointers or free the
/// object twice.
/// @ingroup controllers
#[rapier_export]
pub unsafe extern "C" fn rpr_free_pid_controller(controller: *mut RprPidController) -> RprStatus {
    ffi(|| unsafe {
        if !controller.is_null() {
            drop(Box::from_raw(controller));
        }
        Ok(())
    })
}
/// Return a copy of the proportional, integral, and derivative gains.
/// @ingroup controllers
#[rapier_export(pid_controller)]
pub unsafe extern "C" fn rpr_pid_controller_gains(
    controller: *const RprPidController,
) -> RprPidGains {
    ffi_value(|out: *mut RprPidGains| {
        ffi(|| unsafe {
            let c = &get(controller)?.0;
            output(
                out,
                RprPidGains {
                    lin_kp: c.pd.lin_kp.into(),
                    lin_ki: c.lin_ki.into(),
                    lin_kd: c.pd.lin_kd.into(),
                    ang_kp: angular_out(c.pd.ang_kp),
                    ang_ki: angular_out(c.ang_ki),
                    ang_kd: angular_out(c.pd.ang_kd),
                },
            )
        })
    })
}
/// Replace the proportional, integral, and derivative gains.
/// @ingroup controllers
#[rapier_export(pid_controller)]
pub unsafe extern "C" fn rpr_pid_controller_set_gains(
    controller: *mut RprPidController,
    gains: RprPidGains,
) -> RprStatus {
    ffi(|| unsafe {
        let lin_kp = gains.lin_kp.raw()?;
        let lin_ki = gains.lin_ki.raw()?;
        let lin_kd = gains.lin_kd.raw()?;
        let ang_kp = angular(gains.ang_kp)?;
        let ang_ki = angular(gains.ang_ki)?;
        let ang_kd = angular(gains.ang_kd)?;
        let c = &mut get_mut(controller)?.0;
        c.pd.lin_kp = lin_kp;
        c.lin_ki = lin_ki;
        c.pd.lin_kd = lin_kd;
        c.pd.ang_kp = ang_kp;
        c.ang_ki = ang_ki;
        c.pd.ang_kd = ang_kd;
        Ok(())
    })
}
/// AxesMask bits match Rapier: linear X/Y/Z are 1/2/4, angular X/Y/Z are 8/16/32.
/// @ingroup controllers
#[rapier_export(pid_controller)]
pub unsafe extern "C" fn rpr_pid_controller_set_axes(
    controller: *mut RprPidController,
    axes: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let axes = u8::try_from(axes)
            .ok()
            .and_then(AxesMask::from_bits)
            .ok_or_else(|| invalid("unknown PID axes"))?;
        get_mut(controller)?.0.set_axes(axes);
        Ok(())
    })
}
/// Compute a velocity correction, preserving the body's state and updating PID integrals.
/// @ingroup controllers
#[rapier_export(pid_controller)]
pub unsafe extern "C" fn rpr_pid_controller_rigid_body_correction(
    controller: *mut RprPidController,
    dt: RprReal,
    body: RprRigidBodyHandle,
    target_pose: RprPose,
    target_linvel: RprVector,
    target_angvel: RprAngVector,
) -> RprVelocityCorrection {
    let world = body.world;
    ffi_value(|result: *mut RprVelocityCorrection| {
        let linear = unsafe { std::ptr::addr_of_mut!((*result).linear) };
        let angular_velocity = unsafe { std::ptr::addr_of_mut!((*result).angularVelocity) };

        ffi(|| unsafe {
            body.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            crate::handle_access::forward(native_pid_controller_rigid_body_correction(
                controller,
                dt,
                std::ptr::addr_of!((*raw).0.bodies).cast(),
                body,
                target_pose,
                target_linvel,
                target_angvel,
                linear,
                angular_velocity,
            ))
        })
    })
}

pub(crate) unsafe fn native_pid_controller_rigid_body_correction(
    controller: *mut RprPidController,
    dt: RprReal,
    bodies: *const RprRigidBodySet,
    body: RprRigidBodyHandle,
    target_pose: RprPose,
    target_linvel: RprVector,
    target_angvel: RprAngVector,
    linear: *mut RprVector,
    angular_velocity: *mut RprAngVector,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(linear)?;
        out_ptr(angular_velocity)?;
        let dt = positive(dt)?;
        let pose = target_pose.raw()?;
        let velocity = RigidBodyVelocity {
            linvel: target_linvel.raw()?,
            angvel: angular(target_angvel)?,
        };
        let correction = get_mut(controller)?.0.rigid_body_correction(
            dt,
            get(bodies)?.0.get(body.raw()).ok_or_else(missing)?,
            pose,
            velocity,
        );
        output(linear, correction.linvel.into())?;
        output(angular_velocity, angular_out(correction.angvel))
    })
}

/// Copy of character sliding, slope, and snapping settings.
/// @ingroup controllers
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprCharacterControllerSettings {
    /// Whether obstacle sliding is enabled.
    pub slide: RprBool,
    /// Maximum climbable slope angle in radians.
    pub max_slope_climb_angle: RprReal,
    /// Minimum slope angle for sliding, in radians.
    pub min_slope_slide_angle: RprReal,
    /// Whether downward ground snapping is enabled.
    pub snap_to_ground: RprBool,
    /// Maximum downward snapping distance.
    pub snap_distance: RprCharacterLength,
}
/// Return a copy of slide, slope, and ground-snap settings.
/// @ingroup controllers
#[rapier_export(kinematic_character_controller)]
pub unsafe extern "C" fn rpr_kinematic_character_controller_settings(
    controller: *const RprKinematicCharacterController,
) -> RprCharacterControllerSettings {
    ffi_value(|out: *mut RprCharacterControllerSettings| {
        ffi(|| unsafe {
            let c = &get(controller)?.inner;
            let snap_distance = match c.snap_to_ground {
                Some(CharacterLength::Relative(value)) => RprCharacterLength { value, relative: 1 },
                Some(CharacterLength::Absolute(value)) => RprCharacterLength { value, relative: 0 },
                None => RprCharacterLength {
                    value: 0.1,
                    relative: 1,
                },
            };
            output(
                out,
                RprCharacterControllerSettings {
                    slide: c.slide as u32,
                    max_slope_climb_angle: c.max_slope_climb_angle,
                    min_slope_slide_angle: c.min_slope_slide_angle,
                    snap_to_ground: c.snap_to_ground.is_some() as u32,
                    snap_distance,
                },
            )
        })
    })
}
