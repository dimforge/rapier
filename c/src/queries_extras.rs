//! Additional scene queries, contact-graph access, event callbacks, per-contact hook edits and
//! debug-render styles.
use crate::*;
use rapier::geometry::ContactManifold;
use rapier::parry::query::NonlinearRigidMotion;
use rapier::pipeline::{ContactModificationContext, DebugRenderStyle};
use std::ffi::c_void;

/// @ingroup events
/// Collision event flag: at least one of the colliders was a sensor when the event fired.
pub const RPR_COLLISION_EVENT_SENSOR: u32 = 1;
/// @ingroup events
/// Collision event flag: the collision stopped because at least one collider was removed.
pub const RPR_COLLISION_EVENT_REMOVED: u32 = 2;

pub(crate) fn shape_cast_hit(
    collider: ColliderHandle,
    hit: rapier::parry::query::ShapeCastHit,
) -> RprShapeCastHit {
    RprShapeCastHit {
        collider: collider.into(),
        time_of_impact: hit.time_of_impact,
        witness1: hit.witness1.into(),
        witness2: hit.witness2.into(),
        normal1: hit.normal1.into(),
        normal2: hit.normal2.into(),
        status: hit.status as u32,
    }
}

/// Copy the hits of every collider intersected by the ray, in no particular order. The ray is
/// origin + direction * t for 0 <= t <= max_toi; direction need not be normalized. solid treats an
/// interior origin as a hit at t = 0.
/// @see @ref output_buffers
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_intersect_ray(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    origin: RprVector,
    direction: RprVector,
    max_toi: RprReal,
    solid: RprBool,
    buffer: *mut RprRayHit,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                if !query_options.is_null() {
                    get(query_options)?.check_world(world)?;
                }
                let access = get(world)?.read()?;
                let raw = access.raw();
                let query = QueryAccess::from_world(world as *mut RprWorld, raw, query_options)?;

                let origin = origin.raw()?;
                let direction = direction.raw()?;
                positive(direction.length())?;
                nonnegative(max_toi)?;
                let solid = boolean(solid)?;
                query.with_raw(|q| {
                    let values: Vec<_> = q
                        .intersect_ray(Ray::new(origin, direction), max_toi, solid)
                        .map(|(h, _, hit)| {
                            let (feature_type, feature_id) = feature(hit.feature);
                            RprRayHit {
                                collider: h.into(),
                                time_of_impact: hit.time_of_impact,
                                normal: hit.normal.into(),
                                feature_type,
                                feature_id,
                            }
                        })
                        .collect();
                    copy_out(&values, buffer, capacity, count)
                })
            })
        })
    }
}

/// Optional shape-cast result. A miss is found = 0 with status OK.
/// @ingroup queries
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalShapeCastHit {
    /// Shape-cast impact details.
    pub hit: RprShapeCastHit,
    /// Whether a result exists; other result fields are meaningful only when this is 1.
    pub found: RprBool,
}

/// Optional point projection. A miss is found = 0 with status OK.
/// @ingroup queries
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalPointProjection {
    /// Closest projection details.
    pub projection: RprPointProjection,
    /// Whether a result exists; other result fields are meaningful only when this is 1.
    pub found: RprBool,
}

unsafe fn optional_shape_cast(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    cast: impl FnOnce(
        QueryPipeline<'_>,
    ) -> Result<Option<(ColliderHandle, rapier::parry::query::ShapeCastHit)>>,
) -> RprOptionalShapeCastHit {
    ffi_world_value(world, |result: *mut RprOptionalShapeCastHit| {
        let out = unsafe { std::ptr::addr_of_mut!((*result).hit) };
        let found = unsafe { std::ptr::addr_of_mut!((*result).found) };
        ffi(|| unsafe {
            if !query_options.is_null() {
                get(query_options)?.check_world(world)?;
            }
            let access = get(world)?.read()?;
            let raw = access.raw();
            let query = QueryAccess::from_world(world as *mut RprWorld, raw, query_options)?;
            out_ptr(out)?;
            out_ptr(found)?;
            query.with_raw(|q| match cast(q)? {
                Some((h, hit)) => {
                    output(out, shape_cast_hit(h, hit))?;
                    output(found, 1)
                }
                None => {
                    output(out, RprShapeCastHit::default())?;
                    output(found, 0)
                }
            })
        })
    })
}

/// Sweep shape from pose along velocity and return the first hit, with found = 0 on a miss
/// (RPR_OK). Time is bounded by options.max_time_of_impact. The shape is borrowed for this call.
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_try_cast_shape(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    pose: RprPose,
    velocity: RprVector,
    shape: *const RprSharedShape,
    options: RprShapeCastOptions,
) -> RprOptionalShapeCastHit {
    unsafe {
        optional_shape_cast(world, query_options, |q| {
            let p = pose.raw()?;
            let v = velocity.raw()?;
            let o = options.raw()?;
            Ok(q.cast_shape(&p, v, &*get(shape)?.0, o))
        })
    }
}

/// Return the closest surface projection within max_distance, with found = 0 if there is none
/// (RPR_OK). With solid = 1, an interior point projects to itself.
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_try_project_point(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    point: RprVector,
    max_distance: RprReal,
    solid: RprBool,
) -> RprOptionalPointProjection {
    ffi_world_value(world, |result: *mut RprOptionalPointProjection| {
        let out = unsafe { std::ptr::addr_of_mut!((*result).projection) };
        let found = unsafe { std::ptr::addr_of_mut!((*result).found) };
        ffi(|| unsafe {
            if !query_options.is_null() {
                get(query_options)?.check_world(world)?;
            }
            let access = get(world)?.read()?;
            let raw = access.raw();
            let query = QueryAccess::from_world(world as *mut RprWorld, raw, query_options)?;
            out_ptr(out)?;
            out_ptr(found)?;
            let p = point.raw()?;
            nonnegative(max_distance)?;
            let solid = boolean(solid)?;
            query.with_raw(|q| {
                if let Some((h, p)) = q.project_point(p, max_distance, solid) {
                    output(
                        out,
                        RprPointProjection {
                            collider: h.into(),
                            point: p.point.into(),
                            is_inside: p.is_inside as u32,
                        },
                    )?;
                    output(found, 1)
                } else {
                    output(out, RprPointProjection::default())?;
                    output(found, 0)
                }
            })
        })
    })
}

/// Rigid motion with constant linear and angular velocities. At time t, the shape at start is
/// rotated by angvel * t around its local_center point, then translated by linvel * t.
/// @ingroup queries
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprNonlinearRigidMotion {
    /// World-space pose at time zero.
    pub start: RprPose,
    /// Rotation center, in the local coordinates of the moving shape.
    pub local_center: RprVector,
    /// World-space linear velocity.
    pub linvel: RprVector,
    /// World-space angular velocity, in radians per second.
    pub angvel: RprAngVector,
}
impl RprNonlinearRigidMotion {
    fn raw(&self) -> Result<NonlinearRigidMotion> {
        Ok(NonlinearRigidMotion {
            start: self.start.raw()?,
            local_center: self.local_center.raw()?,
            linvel: self.linvel.raw()?,
            angvel: angular(self.angvel)?,
        })
    }
}

/// Return the world-space pose of the motion at the given time.
/// @ingroup queries
#[rapier_export(nonlinear_rigid_motion)]
pub unsafe extern "C" fn rpr_nonlinear_rigid_motion_position_at_time(
    motion: *const RprNonlinearRigidMotion,
    time: RprReal,
) -> RprPose {
    ffi_value(|out: *mut RprPose| {
        ffi(|| unsafe {
            let motion = get(motion)?.raw()?;
            output(out, motion.position_at_time(finite(time)?).into())
        })
    })
}

/// Sweep shape along a rotating motion and return the first hit between start_time and end_time,
/// with found = 0 on a miss (RPR_OK). With stop_at_penetration = 1, a shape already intersecting a
/// collider at start_time hits it at start_time; with 0, that penetration is ignored while the
/// motion separates the shapes. witness1/normal1 are world-space; witness2/normal2 are local to the
/// shape, posed by rpr_nonlinear_rigid_motion_position_at_time at the time of impact.
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_try_cast_shape_nonlinear(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    motion: *const RprNonlinearRigidMotion,
    shape: *const RprSharedShape,
    start_time: RprReal,
    end_time: RprReal,
    stop_at_penetration: RprBool,
) -> RprOptionalShapeCastHit {
    unsafe {
        optional_shape_cast(world, query_options, |q| {
            let motion = get(motion)?.raw()?;
            finite(start_time)?;
            finite(end_time)?;
            ensure(start_time <= end_time, "start_time exceeds end_time")?;
            let stop = boolean(stop_at_penetration)?;
            Ok(q.cast_shape_nonlinear(&motion, &*get(shape)?.0, start_time, end_time, stop))
        })
    }
}

/// Optional contact pair; check found before reading the pair.
/// @ingroup events
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalContactPair {
    /// Contact pair summary.
    pub pair: RprContactPair,
    /// Whether a result exists; other result fields are meaningful only when this is 1.
    pub found: RprBool,
}

/// Optional intersection pair; check found before reading the pair.
/// @ingroup events
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalIntersectionPair {
    /// Intersection pair state.
    pub pair: RprIntersectionPair,
    /// Whether a result exists; other result fields are meaningful only when this is 1.
    pub found: RprBool,
}

unsafe fn narrow_phase<'a>(world: *const RprPhysicsWorld) -> Result<&'a NarrowPhase> {
    unsafe { Ok(&get(world)?.0.narrow_phase) }
}

/// Return the narrow-phase contact pair for two colliders, with found = 0 if the broad phase
/// found no potential contact between them (RPR_OK). The pair's collider1 and collider2 follow the
/// narrow-phase order, which may differ from the argument order.
/// @ingroup events
#[rapier_export]
pub unsafe extern "C" fn rpr_try_contact_pair(
    collider1: RprColliderHandle,
    collider2: RprColliderHandle,
) -> RprOptionalContactPair {
    let world = collider1.world;
    ffi_world_value(world, |result: *mut RprOptionalContactPair| {
        let out = unsafe { std::ptr::addr_of_mut!((*result).pair) };
        let found = unsafe { std::ptr::addr_of_mut!((*result).found) };
        ffi(|| unsafe {
            collider1.check_world(world)?;
            collider2.check_world(world)?;
            let access = get(world)?.read()?;
            out_ptr(out)?;
            out_ptr(found)?;
            match narrow_phase(access.raw())?.contact_pair(collider1.raw(), collider2.raw()) {
                Some(p) => {
                    output(out, p.into())?;
                    output(found, 1)
                }
                None => {
                    output(out, RprContactPair::default())?;
                    output(found, 0)
                }
            }
        })
    })
}

/// Return the intersection state of two colliders involving a sensor, or report RPR_NOT_FOUND if
/// the broad phase found no potential intersection. The result keeps the argument order.
/// @ingroup events
#[rapier_export]
pub unsafe extern "C" fn rpr_intersection_pair(
    collider1: RprColliderHandle,
    collider2: RprColliderHandle,
) -> RprIntersectionPair {
    let world = collider1.world;
    ffi_world_value(world, |out: *mut RprIntersectionPair| {
        ffi(|| unsafe {
            collider1.check_world(world)?;
            collider2.check_world(world)?;
            let access = get(world)?.read()?;
            let intersecting = narrow_phase(access.raw())?
                .intersection_pair(collider1.raw(), collider2.raw())
                .ok_or((RPR_NOT_FOUND, "no intersection pair".into()))?;
            output(
                out,
                RprIntersectionPair {
                    collider1,
                    collider2,
                    intersecting: intersecting as u32,
                },
            )
        })
    })
}

/// Return the intersection state of two colliders involving a sensor, with found = 0 if the
/// broad phase found no potential intersection (RPR_OK). The pair keeps the argument order.
/// @ingroup events
#[rapier_export]
pub unsafe extern "C" fn rpr_try_intersection_pair(
    collider1: RprColliderHandle,
    collider2: RprColliderHandle,
) -> RprOptionalIntersectionPair {
    let world = collider1.world;
    ffi_world_value(world, |result: *mut RprOptionalIntersectionPair| {
        let out = unsafe { std::ptr::addr_of_mut!((*result).pair) };
        let found = unsafe { std::ptr::addr_of_mut!((*result).found) };
        ffi(|| unsafe {
            collider1.check_world(world)?;
            collider2.check_world(world)?;
            let access = get(world)?.read()?;
            out_ptr(out)?;
            out_ptr(found)?;
            match narrow_phase(access.raw())?.intersection_pair(collider1.raw(), collider2.raw()) {
                Some(intersecting) => {
                    output(
                        out,
                        RprIntersectionPair {
                            collider1,
                            collider2,
                            intersecting: intersecting as u32,
                        },
                    )?;
                    output(found, 1)
                }
                None => {
                    output(out, RprIntersectionPair::default())?;
                    output(found, 0)
                }
            }
        })
    })
}

/// Copy the narrow-phase contact pairs involving the collider, including pairs without active
/// solver contacts. The collider may be either collider1 or collider2 of each pair.
/// @see @ref output_buffers
/// @ingroup events
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_contact_pairs(
    handle: RprColliderHandle,
    buffer: *mut RprContactPair,
    capacity: usize,
) -> usize {
    let world = handle.world;
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                handle.check_world(world)?;
                let access = get(world)?.read()?;
                let raw = access.raw();
                get(raw)?
                    .0
                    .colliders
                    .get(handle.raw())
                    .ok_or_else(missing)?;
                let values: Vec<RprContactPair> = narrow_phase(raw)?
                    .contact_pairs_with(handle.raw())
                    .map(Into::into)
                    .collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    }
}

/// Copy the intersection pairs involving the collider, in the narrow-phase order. The collider may
/// be either collider1 or collider2 of each pair.
/// @see @ref output_buffers
/// @ingroup events
#[rapier_export(collider)]
pub unsafe extern "C" fn rpr_collider_intersection_pairs(
    handle: RprColliderHandle,
    buffer: *mut RprIntersectionPair,
    capacity: usize,
) -> usize {
    let world = handle.world;
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                handle.check_world(world)?;
                let access = get(world)?.read()?;
                let raw = access.raw();
                get(raw)?
                    .0
                    .colliders
                    .get(handle.raw())
                    .ok_or_else(missing)?;
                let values: Vec<_> = narrow_phase(raw)?
                    .intersection_pairs_with(handle.raw())
                    .map(|(a, b, hit)| RprIntersectionPair {
                        collider1: a.into(),
                        collider2: b.into(),
                        intersecting: hit as u32,
                    })
                    .collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    }
}

/// Geometric contact manifold of a contact pair: contacts sharing one normal. Local data follow the
/// pair's own collider1/collider2 order (see rpr_contact_pair).
/// @ingroup events
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprContactManifold {
    /// Contact normal in collider 1 local coordinates, pointing outward from it.
    pub local_n1: RprVector,
    /// Contact normal in collider 2 local coordinates, pointing outward from it.
    pub local_n2: RprVector,
    /// World-space contact normal, pointing from collider 1 toward collider 2.
    pub normal: RprVector,
    /// Index of the subshape of collider 1 (for composite shapes), zero otherwise.
    pub subshape1: u32,
    /// Index of the subshape of collider 2 (for composite shapes), zero otherwise.
    pub subshape2: u32,
    /// Number of geometric contact points; see rpr_contact_points.
    pub num_points: usize,
    /// Number of solver contacts; see rpr_solver_contacts.
    pub num_solver_contacts: usize,
    /// Application data, persistent across steps and editable by contact-modification hooks.
    pub user_data: u32,
}
impl From<&ContactManifold> for RprContactManifold {
    fn from(m: &ContactManifold) -> Self {
        Self {
            local_n1: m.local_n1.into(),
            local_n2: m.local_n2.into(),
            normal: m.data.normal.into(),
            subshape1: m.subshape1,
            subshape2: m.subshape2,
            num_points: m.points.len(),
            num_solver_contacts: m.data.solver_contacts.len(),
            user_data: m.data.user_data,
        }
    }
}

/// Contact seen by the constraint solver. Points are world-space, on each body's surface.
/// @ingroup events
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSolverContact {
    /// World-space contact point on collider 1's body.
    pub point1: RprVector,
    /// World-space contact point on collider 2's body.
    pub point2: RprVector,
    /// Signed separation along the normal, contact skins deducted; negative means penetration.
    pub distance: RprReal,
    /// Desired world-space tangent relative velocity, e.g. for conveyor belts; zero by default.
    pub tangent_velocity: RprVector,
}

unsafe fn with_contact_pair<T>(
    collider1: RprColliderHandle,
    collider2: RprColliderHandle,
    f: impl FnOnce(&ContactPair, &RigidBodySet) -> Result<T>,
) -> Result<T> {
    let world = collider1.world;
    unsafe {
        collider1.check_world(world)?;
        collider2.check_world(world)?;
        let access = get(world)?.read()?;
        let raw = access.raw();
        let pair = narrow_phase(raw)?
            .contact_pair(collider1.raw(), collider2.raw())
            .ok_or((RPR_NOT_FOUND, "no contact pair".into()))?;
        f(pair, &get(raw)?.0.bodies)
    }
}

/// Copy the geometric contact manifolds of a contact pair, or report RPR_NOT_FOUND without a pair.
/// Their order matches the manifold_index of rpr_contact_points. Soft pairs have no rigid
/// manifolds.
/// @see @ref output_buffers
/// @ingroup events
#[rapier_export]
pub unsafe extern "C" fn rpr_contact_manifolds(
    collider1: RprColliderHandle,
    collider2: RprColliderHandle,
    buffer: *mut RprContactManifold,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            with_contact_pair(collider1, collider2, |pair, _| {
                let values: Vec<_> = pair.manifolds().iter().map(Into::into).collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    })
}

/// Copy the solver contacts of one manifold of a contact pair, or report RPR_NOT_FOUND without a
/// pair. Points are resolved through the bodies' current poses. With contact clustering (3D
/// composite shapes), the solver may use merged manifolds instead; use contact pair totals then.
/// @see @ref output_buffers
/// @ingroup events
#[rapier_export]
pub unsafe extern "C" fn rpr_solver_contacts(
    collider1: RprColliderHandle,
    collider2: RprColliderHandle,
    manifold_index: usize,
    buffer: *mut RprSolverContact,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            with_contact_pair(collider1, collider2, |pair, bodies| {
                let m = pair
                    .manifolds()
                    .get(manifold_index)
                    .ok_or_else(|| invalid("manifold index out of range"))?;
                let values: Vec<_> = m
                    .data
                    .solver_contacts
                    .iter()
                    .map(|c| {
                        let (point1, point2) = m.data.solver_contact_world_points(c, bodies);
                        RprSolverContact {
                            point1: point1.into(),
                            point2: point2.into(),
                            distance: c.dist,
                            tangent_velocity: c.tangent_velocity.into(),
                        }
                    })
                    .collect();
                copy_out(&values, buffer, capacity, count)
            })
        })
    })
}

// The context wraps a native context borrowed for the duration of the hook call.
unsafe fn context_ref<'a>(
    context: *const RprContactModificationContext,
) -> Result<&'a ContactModificationContext<'a>> {
    unsafe { Ok(&*get(context)?.raw.cast::<ContactModificationContext<'_>>()) }
}
unsafe fn context_mut<'a>(
    context: *mut RprContactModificationContext,
) -> Result<&'a mut ContactModificationContext<'a>> {
    unsafe {
        Ok(&mut *get_mut(context)?
            .raw
            .cast::<ContactModificationContext<'_>>())
    }
}

/// Return whether the context holds the contact candidates of two soft surfaces rather than a
/// manifold. Solver-contact accessors see no contacts in a soft context.
/// @ingroup callbacks
#[rapier_export(contact_modification_context)]
pub unsafe extern "C" fn rpr_contact_modification_context_is_soft(
    context: *const RprContactModificationContext,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe { output(out, context_ref(context)?.rigid().is_none() as u32) })
    })
}

/// Return the number of solver contacts of the manifold; zero for a soft context.
/// @ingroup callbacks
#[rapier_export(contact_modification_context)]
pub unsafe extern "C" fn rpr_contact_modification_context_solver_contact_count(
    context: *const RprContactModificationContext,
) -> usize {
    ffi_value(|out: *mut usize| {
        ffi(|| unsafe {
            let count = context_ref(context)?
                .rigid()
                .map_or(0, |m| m.solver_contacts.len());
            output(out, count)
        })
    })
}

/// Return a solver contact of the manifold. Inside the hook, points are world-space.
/// @ingroup callbacks
#[rapier_export(contact_modification_context)]
pub unsafe extern "C" fn rpr_contact_modification_context_solver_contact(
    context: *const RprContactModificationContext,
    index: usize,
) -> RprSolverContact {
    ffi_value(|out: *mut RprSolverContact| {
        ffi(|| unsafe {
            let c = context_ref(context)?
                .rigid()
                .and_then(|m| m.solver_contacts.get(index))
                .ok_or_else(|| invalid("solver contact index out of range"))?;
            output(
                out,
                RprSolverContact {
                    point1: c.anchor1.into(),
                    point2: c.anchor2.into(),
                    distance: c.dist,
                    tangent_velocity: c.tangent_velocity.into(),
                },
            )
        })
    })
}

/// Replace the points, distance and tangent velocity of a solver contact of the manifold. Points
/// are world-space; a distance differing from their gap along the normal shifts the contact.
/// @ingroup callbacks
#[rapier_export(contact_modification_context)]
pub unsafe extern "C" fn rpr_contact_modification_context_set_solver_contact(
    context: *mut RprContactModificationContext,
    index: usize,
    contact: *const RprSolverContact,
) -> RprStatus {
    ffi(|| unsafe {
        let contact = *get(contact)?;
        let point1 = contact.point1.raw()?;
        let point2 = contact.point2.raw()?;
        let distance = finite(contact.distance)?;
        let tangent_velocity = contact.tangent_velocity.raw()?;
        let c = context_mut(context)?
            .rigid_mut()
            .and_then(|m| m.solver_contacts.get_mut(index))
            .ok_or_else(|| invalid("solver contact index out of range"))?;
        c.anchor1 = point1;
        c.anchor2 = point2;
        c.dist = distance;
        c.tangent_velocity = tangent_velocity;
        Ok(())
    })
}

/// Remove a solver contact of the manifold. The last solver contact takes its index.
/// @ingroup callbacks
#[rapier_export(contact_modification_context)]
pub unsafe extern "C" fn rpr_contact_modification_context_remove_solver_contact(
    context: *mut RprContactModificationContext,
    index: usize,
) -> RprStatus {
    ffi(|| unsafe {
        let contacts = context_mut(context)?
            .rigid_mut()
            .map(|m| &mut *m.solver_contacts)
            .filter(|contacts| index < contacts.len())
            .ok_or_else(|| invalid("solver contact index out of range"))?;
        contacts.swap_remove(index);
        Ok(())
    })
}

/// Called during the step for each collision event, after it was added to the collector. contacts
/// holds the geometric contacts of the pair at that time (none for sensors), in the event's
/// collider order; it is borrowed for this call only.
/// @ingroup events
pub type RprCollisionEventCallback = Option<
    unsafe extern "C" fn(
        user_data: *mut c_void,
        read: *const RprReadContext,
        event: *const RprCollisionEvent,
        contacts: *const RprContactPoint,
        contact_count: usize,
    ),
>;
/// Called during the step for each contact-force event, after it was added to the collector.
/// @ingroup events
pub type RprContactForceEventCallback = Option<
    unsafe extern "C" fn(
        user_data: *mut c_void,
        read: *const RprReadContext,
        event: *const RprContactForceEvent,
    ),
>;

/// Callbacks invoked while stepping, in addition to collecting the events. They follow the
/// RprPhysicsHooks rules: never unwind or retain arguments, read through the ReadContext, never
/// mutate the world, and be safe for concurrent invocation in parallel builds. NULL callbacks are
/// skipped.
/// @ingroup events
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprEventCallbacks {
    /// Application data; Rapier does not own pointers encoded in it.
    pub user_data: *mut c_void,
    /// Optional collision start/stop callback.
    pub collision_event: RprCollisionEventCallback,
    /// Optional contact-force callback.
    pub contact_force_event: RprContactForceEventCallback,
}
// SAFETY: The public callback contract requires thread-safe callbacks and user_data in parallel builds.
unsafe impl Send for RprEventCallbacks {}
unsafe impl Sync for RprEventCallbacks {}

/// Replace the callbacks invoked while stepping with this collector; NULL removes them. They take
/// effect from the next Step or DetectCollisions call.
/// @ingroup events
#[rapier_export(event_collector)]
pub unsafe extern "C" fn rpr_event_collector_set_callbacks(
    events: *mut RprEventCollector,
    callbacks: *const RprEventCallbacks,
) -> RprStatus {
    ffi(|| unsafe {
        let callbacks = if callbacks.is_null() {
            RprEventCallbacks::default()
        } else {
            *get(callbacks)?
        };
        // Shared access: a callback may replace the callbacks of the collector being filled.
        *get(events)?.callbacks.lock().unwrap() = callbacks;
        Ok(())
    })
}

/// Debug-render colors and sizes. Colors are HSLA: hue in degrees, then saturation, lightness and
/// alpha in [0, 1]; multipliers scale each component. Initialize with
/// rpr_default_debug_render_style.
/// @ingroup events
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprDebugRenderStyle {
    /// Positive number of subdivisions approximating curved shapes.
    pub subdivisions: u32,
    /// Positive number of subdivisions approximating the borders of round shapes.
    pub border_subdivisions: u32,
    /// Color of colliders attached to dynamic bodies.
    pub collider_dynamic_color: [f32; 4],
    /// Color of colliders attached to fixed bodies.
    pub collider_fixed_color: [f32; 4],
    /// Color of colliders attached to kinematic bodies.
    pub collider_kinematic_color: [f32; 4],
    /// Color of colliders without a parent body.
    pub collider_parentless_color: [f32; 4],
    /// Color of the lines from a body's center of mass to its impulse-joint anchors.
    pub impulse_joint_anchor_color: [f32; 4],
    /// Color of the line between the two anchors of an impulse joint.
    pub impulse_joint_separation_color: [f32; 4],
    /// Color of the lines from a body's center of mass to its multibody-joint anchors.
    pub multibody_joint_anchor_color: [f32; 4],
    /// Color of the line between the two anchors of a multibody joint.
    pub multibody_joint_separation_color: [f32; 4],
    /// Color multiplier for entities of sleeping bodies.
    pub sleep_color_multiplier: [f32; 4],
    /// Color multiplier for entities of awake bodies eligible for sleep.
    pub sleep_eligible_color_multiplier: [f32; 4],
    /// Color multiplier for entities of disabled bodies.
    pub disabled_color_multiplier: [f32; 4],
    /// Nonnegative length of the rendered body axes.
    pub rigid_body_axes_length: RprReal,
    /// Color of the segments joining the two points of a contact.
    pub contact_depth_color: [f32; 4],
    /// Color of the contact normals.
    pub contact_normal_color: [f32; 4],
    /// Nonnegative length of the contact normals.
    pub contact_normal_length: RprReal,
    /// Color of soft-body elements.
    pub soft_body_element_color: [f32; 4],
    /// Color of unloaded soft-body elements when coloring them by load.
    pub soft_body_slack_color: [f32; 4],
    /// Color of soft-body elements at their tear threshold when coloring them by load.
    pub soft_body_loaded_color: [f32; 4],
    /// Color of the soft-body cluster frames.
    pub soft_body_frame_color: [f32; 4],
    /// Color of the collider bounding boxes.
    pub collider_aabb_color: [f32; 4],
    /// Color of the vertex pseudo-normals of triangle meshes and polylines.
    pub vertex_pseudo_normal_color: [f32; 4],
    /// Color of the edge pseudo-normals of triangle meshes (3D only).
    pub edge_pseudo_normal_color: [f32; 4],
    /// Nonnegative length of the pseudo-normals.
    pub pseudo_normal_length: RprReal,
    /// Color of the normals of soft-body volume contacts.
    pub volume_contact_normal_color: [f32; 4],
    /// Color of the volume gradients drawn at the particles of a volume constraint.
    pub volume_gradient_color: [f32; 4],
}
macro_rules! debug_style_fields {
    ($m:ident) => {
        $m!(
            [subdivisions, border_subdivisions],
            [
                collider_dynamic_color,
                collider_fixed_color,
                collider_kinematic_color,
                collider_parentless_color,
                impulse_joint_anchor_color,
                impulse_joint_separation_color,
                multibody_joint_anchor_color,
                multibody_joint_separation_color,
                sleep_color_multiplier,
                sleep_eligible_color_multiplier,
                disabled_color_multiplier,
                contact_depth_color,
                contact_normal_color,
                soft_body_element_color,
                soft_body_slack_color,
                soft_body_loaded_color,
                soft_body_frame_color,
                collider_aabb_color,
                vertex_pseudo_normal_color,
                edge_pseudo_normal_color,
                volume_contact_normal_color,
                volume_gradient_color
            ],
            [
                rigid_body_axes_length,
                contact_normal_length,
                pseudo_normal_length
            ]
        )
    };
}
impl From<DebugRenderStyle> for RprDebugRenderStyle {
    fn from(s: DebugRenderStyle) -> Self {
        macro_rules! convert {
            ([$($count:ident),*], [$($color:ident),*], [$($length:ident),*]) => {
                Self { $($count: s.$count,)* $($color: s.$color,)* $($length: s.$length,)* }
            };
        }
        debug_style_fields!(convert)
    }
}
impl RprDebugRenderStyle {
    pub(crate) fn raw(&self) -> Result<DebugRenderStyle> {
        let s = self;
        macro_rules! convert {
            ([$($count:ident),*], [$($color:ident),*], [$($length:ident),*]) => {{
                $(ensure(s.$count > 0, concat!(stringify!($count), " must be positive"))?;)*
                $(ensure(
                    s.$color.iter().all(|c| c.is_finite()),
                    concat!(stringify!($color), " must be finite"),
                )?;)*
                $(nonnegative(s.$length)?;)*
                DebugRenderStyle { $($count: s.$count,)* $($color: s.$color,)* $($length: s.$length,)* }
            }};
        }
        Ok(debug_style_fields!(convert))
    }
}

/// Return native default debug-render style. This POD value owns no resources.
/// @ingroup events
#[rapier_export]
pub extern "C" fn rpr_default_debug_render_style() -> RprDebugRenderStyle {
    DebugRenderStyle::default().into()
}

/// Copy the debug-render lines of the world drawn with the given style. mode combines RPR_DEBUG_*
/// bits. NULL style uses the default style.
/// @see @ref output_buffers
/// @ingroup events
#[rapier_export]
pub unsafe extern "C" fn rpr_debug_render_with_style(
    world: *const RprWorld,
    mode: u32,
    style: *const RprDebugRenderStyle,
    buffer: *mut RprDebugLine,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let style = if style.is_null() {
                DebugRenderStyle::default()
            } else {
                get(style)?.raw()?
            };
            let access = get(world)?.read()?;
            debug_render_lines(access.raw(), mode, style, buffer, capacity, count)
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ptr;
    use std::sync::Mutex;

    // A fixed cube whose top face is at y = 0 and a dynamic ball resting on it.
    unsafe fn ball_on_ground(
        ball: RprColliderDesc,
    ) -> (*mut RprWorld, RprColliderHandle, RprColliderHandle) {
        unsafe {
            let world = rpr_new_world();
            let mut ground = rpr_cuboid_collider_desc(Vector::splat(5.0).into());
            ground.position.translation = (-Vector::Y * 5.0).into();
            let ground = rpr_insert_collider_without_parent(world, &ground);
            assert_eq!(rpr_last_status(), RPR_OK);
            let mut body = rpr_dynamic_rigid_body_desc();
            body.position.translation = (Vector::Y * 0.49).into();
            let body = rpr_insert_rigid_body(world, &body);
            let ball = rpr_insert_collider(body, &ball);
            assert_eq!(rpr_last_status(), RPR_OK);
            (world, ground, ball)
        }
    }

    #[test]
    fn ray_shape_and_point_queries_report_all_hits_and_misses() {
        unsafe {
            let world = rpr_new_world();
            for x in [2.0, 4.0] {
                let mut desc = rpr_ball_collider_desc(0.5);
                desc.position.translation = (Vector::X * x).into();
                rpr_insert_collider_without_parent(world, &desc);
            }
            assert_eq!(
                rpr_detect_collisions(world, ptr::null(), ptr::null()),
                RPR_OK
            );
            let (origin, dir) = (Vector::ZERO.into(), Vector::X.into());
            let count =
                rpr_intersect_ray(world, ptr::null(), origin, dir, 10.0, 1, ptr::null_mut(), 0);
            assert_eq!((rpr_last_status(), count), (RPR_OK, 2));
            let mut hits = [RprRayHit::default(); 2];
            rpr_intersect_ray(
                world,
                ptr::null(),
                origin,
                dir,
                10.0,
                1,
                hits.as_mut_ptr(),
                2,
            );
            assert_eq!(rpr_last_status(), RPR_OK);
            let mut tois: Vec<_> = hits.iter().map(|h| h.time_of_impact).collect();
            tois.sort_by(|a, b| a.partial_cmp(b).unwrap());
            assert!((tois[0] - 1.5).abs() < 1.0e-4 && (tois[1] - 3.5).abs() < 1.0e-4);
            assert!(hits.iter().all(|h| std::ptr::eq(h.collider.world, world)));
            let short =
                rpr_intersect_ray(world, ptr::null(), origin, dir, 2.0, 1, ptr::null_mut(), 0);
            assert_eq!(short, 1);

            let shape = rpr_ball_shared_shape(0.25);
            let mut options = rpr_default_shape_cast_options();
            options.max_time_of_impact = 10.0;
            let pose = RprPose::from(Pose::IDENTITY);
            let hit = rpr_try_cast_shape(world, ptr::null(), pose, dir, shape, options);
            assert_eq!((rpr_last_status(), hit.found), (RPR_OK, 1));
            assert!((hit.hit.time_of_impact - 1.25).abs() < 1.0e-3);
            assert!(std::ptr::eq(hit.hit.collider.world, world));
            let miss =
                rpr_try_cast_shape(world, ptr::null(), pose, Vector::Y.into(), shape, options);
            assert_eq!((rpr_last_status(), miss.found), (RPR_OK, 0));

            let far = (Vector::Y * 5.0).into();
            let miss = rpr_try_project_point(world, ptr::null(), far, 1.0, 1);
            assert_eq!((rpr_last_status(), miss.found), (RPR_OK, 0));
            let found = rpr_try_project_point(world, ptr::null(), far, 10.0, 1);
            assert_eq!((rpr_last_status(), found.found), (RPR_OK, 1));
            assert!(std::ptr::eq(found.projection.collider.world, world));

            let mut motion = RprNonlinearRigidMotion {
                start: pose,
                linvel: Vector::X.into(),
                ..Default::default()
            };
            let at = rpr_nonlinear_rigid_motion_position_at_time(&motion, 2.0);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert!((at.translation.x - 2.0).abs() < 1.0e-6);
            let hit =
                rpr_try_cast_shape_nonlinear(world, ptr::null(), &motion, shape, 0.0, 10.0, 1);
            assert_eq!((rpr_last_status(), hit.found), (RPR_OK, 1));
            assert!((hit.hit.time_of_impact - 1.25).abs() < 1.0e-2);
            motion.linvel = Vector::Y.into();
            let miss =
                rpr_try_cast_shape_nonlinear(world, ptr::null(), &motion, shape, 0.0, 10.0, 1);
            assert_eq!((rpr_last_status(), miss.found), (RPR_OK, 0));
            rpr_try_cast_shape_nonlinear(world, ptr::null(), &motion, shape, 1.0, 0.0, 1);
            assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
            assert_eq!(rpr_free_shared_shape(shape), RPR_OK);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[test]
    fn contact_graph_exposes_manifolds_solver_contacts_and_pairs() {
        unsafe {
            let (world, ground, ball) = ball_on_ground(rpr_ball_collider_desc(0.5));
            for _ in 0..5 {
                assert_eq!(rpr_step(world, ptr::null(), ptr::null()), RPR_OK);
            }
            let pair = rpr_try_contact_pair(ball, ground);
            assert_eq!((rpr_last_status(), pair.found), (RPR_OK, 1));
            assert_eq!(pair.pair.has_any_active_contact, 1);
            assert!(std::ptr::eq(pair.pair.collider1.world, world));

            let count = rpr_contact_manifolds(ball, ground, ptr::null_mut(), 0);
            assert_eq!((rpr_last_status(), count), (RPR_OK, 1));
            let mut manifold = RprContactManifold::default();
            rpr_contact_manifolds(ball, ground, &mut manifold, 1);
            assert_eq!(rpr_last_status(), RPR_OK);
            let points = rpr_contact_points(ball, ground, ptr::null_mut(), 0);
            assert_eq!(points, manifold.num_points);
            let mut point = RprContactPoint::default();
            rpr_contact_points(ball, ground, &mut point, 1);
            assert!(point.impulse > 0.0);
            assert!(point.tangent_impulse.iter().all(|i| i.is_finite()));

            let count = rpr_solver_contacts(ball, ground, 0, ptr::null_mut(), 0);
            assert_eq!(
                (rpr_last_status(), count),
                (RPR_OK, manifold.num_solver_contacts)
            );
            assert!(count > 0);
            let mut contacts = vec![RprSolverContact::default(); count];
            rpr_solver_contacts(ball, ground, 0, contacts.as_mut_ptr(), count);
            assert_eq!(rpr_last_status(), RPR_OK);
            // Both world-space points lie near the top face of the ground, under the ball.
            for c in &contacts {
                assert!(c.point1.y.abs() < 0.05 && c.point2.y.abs() < 0.05);
                assert!(c.point1.x.abs() < 0.05 && c.point2.x.abs() < 0.05);
            }
            rpr_solver_contacts(ball, ground, 1, ptr::null_mut(), 0);
            assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);

            let mut pairs = [RprContactPair::default(); 1];
            let count = rpr_collider_contact_pairs(ball, pairs.as_mut_ptr(), 1);
            assert_eq!((rpr_last_status(), count), (RPR_OK, 1));
            assert!(std::ptr::eq(pairs[0].collider1.world, world));
            let count = rpr_collider_intersection_pairs(ball, ptr::null_mut(), 0);
            assert_eq!((rpr_last_status(), count), (RPR_OK, 0));
            let missing = rpr_try_intersection_pair(ball, ground);
            assert_eq!((rpr_last_status(), missing.found), (RPR_OK, 0));
            rpr_intersection_pair(ball, ground);
            assert_eq!(rpr_last_status(), RPR_NOT_FOUND);

            rpr_remove_collider(ground, 1);
            rpr_collider_contact_pairs(ground, ptr::null_mut(), 0);
            assert_eq!(rpr_last_status(), RPR_INVALID_HANDLE);
            assert_eq!(rpr_step(world, ptr::null(), ptr::null()), RPR_OK);
            let pair = rpr_try_contact_pair(ball, ground);
            assert_eq!((rpr_last_status(), pair.found), (RPR_OK, 0));
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[test]
    fn intersection_graph_reports_sensor_pairs() {
        unsafe {
            let mut sensor = rpr_ball_collider_desc(0.5);
            sensor.isSensor = 1;
            let (world, ground, ball) = ball_on_ground(sensor);
            assert_eq!(
                rpr_detect_collisions(world, ptr::null(), ptr::null()),
                RPR_OK
            );
            let pair = rpr_try_intersection_pair(ball, ground);
            assert_eq!(
                (rpr_last_status(), pair.found, pair.pair.intersecting),
                (RPR_OK, 1, 1)
            );
            assert_eq!(pair.pair.collider1, ball);
            let pair = rpr_intersection_pair(ground, ball);
            assert_eq!((rpr_last_status(), pair.intersecting), (RPR_OK, 1));
            let mut pairs = [RprIntersectionPair::default(); 1];
            let count = rpr_collider_intersection_pairs(ground, pairs.as_mut_ptr(), 1);
            assert_eq!(
                (rpr_last_status(), count, pairs[0].intersecting),
                (RPR_OK, 1, 1)
            );
            let missing = rpr_try_contact_pair(ball, ground);
            assert_eq!((rpr_last_status(), missing.found), (RPR_OK, 0));
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[derive(Default)]
    struct Recorded {
        collisions: Vec<(RprCollisionEvent, usize)>,
        forces: Vec<RprBool>,
    }
    unsafe extern "C" fn on_collision(
        data: *mut c_void,
        read: *const RprReadContext,
        event: *const RprCollisionEvent,
        contacts: *const RprContactPoint,
        count: usize,
    ) {
        unsafe {
            let event = *event;
            rpr_read_collider_translation(read, event.collider1);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert!(count == 0 || !contacts.is_null());
            let recorded = &*data.cast::<Mutex<Recorded>>();
            recorded.lock().unwrap().collisions.push((event, count));
        }
    }
    unsafe extern "C" fn on_force(
        data: *mut c_void,
        _read: *const RprReadContext,
        event: *const RprContactForceEvent,
    ) {
        unsafe {
            let recorded = &*data.cast::<Mutex<Recorded>>();
            recorded.lock().unwrap().forces.push((*event).started);
        }
    }

    #[test]
    fn event_callbacks_run_during_the_step_besides_collection() {
        unsafe {
            let mut ball = rpr_ball_collider_desc(0.5);
            ball.activeEvents = RPR_COLLISION_EVENTS | RPR_CONTACT_FORCE_EVENTS;
            let (world, _ground, _ball) = ball_on_ground(ball);
            let recorded = Mutex::new(Recorded::default());
            let events = rpr_new_event_collector();
            let callbacks = RprEventCallbacks {
                user_data: (&recorded as *const Mutex<Recorded>).cast_mut().cast(),
                collision_event: Some(on_collision),
                contact_force_event: Some(on_force),
            };
            assert_eq!(
                rpr_event_collector_set_callbacks(events, &callbacks),
                RPR_OK
            );
            for _ in 0..3 {
                assert_eq!(rpr_step(world, ptr::null(), events), RPR_OK);
            }
            {
                let recorded = recorded.lock().unwrap();
                assert_eq!(recorded.collisions.len(), 1);
                let (event, count) = recorded.collisions[0];
                assert_eq!((event.started, event.flags), (1, 0));
                assert!(count > 0);
                assert!(std::ptr::eq(event.collider1.world, world));
                assert_eq!(recorded.forces.first(), Some(&1));
                assert!(recorded.forces.len() >= 2 && recorded.forces[1..].iter().all(|s| *s == 0));
            }
            // Callbacks do not replace the collection, which keeps accumulating across steps.
            let count = rpr_event_collector_collision_events(events, ptr::null_mut(), 0);
            assert_eq!(count, 1);
            let count = rpr_event_collector_contact_force_events(events, ptr::null_mut(), 0);
            assert_eq!(count, recorded.lock().unwrap().forces.len());
            assert_eq!(
                rpr_event_collector_set_callbacks(events, ptr::null()),
                RPR_OK
            );
            assert_eq!(rpr_step(world, ptr::null(), events), RPR_OK);
            let forces = rpr_event_collector_contact_force_events(events, ptr::null_mut(), 0);
            assert!(forces > recorded.lock().unwrap().forces.len());
            assert_eq!(rpr_free_event_collector(events), RPR_OK);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[test]
    fn sensor_and_removal_flags_match_native_bits() {
        assert_eq!(
            RPR_COLLISION_EVENT_SENSOR,
            CollisionEventFlags::SENSOR.bits()
        );
        assert_eq!(
            RPR_COLLISION_EVENT_REMOVED,
            CollisionEventFlags::REMOVED.bits()
        );
        unsafe {
            let mut sensor = rpr_ball_collider_desc(0.5);
            sensor.isSensor = 1;
            sensor.activeEvents = RPR_COLLISION_EVENTS;
            let (world, _ground, ball) = ball_on_ground(sensor);
            let events = rpr_new_event_collector();
            assert_eq!(rpr_detect_collisions(world, ptr::null(), events), RPR_OK);
            rpr_remove_collider(ball, 1);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!(rpr_detect_collisions(world, ptr::null(), events), RPR_OK);
            let mut collected = [RprCollisionEvent::default(); 2];
            let count = rpr_event_collector_collision_events(events, collected.as_mut_ptr(), 2);
            assert_eq!((rpr_last_status(), count), (RPR_OK, 2));
            assert_eq!(collected[0].flags, RPR_COLLISION_EVENT_SENSOR);
            assert_eq!(
                collected[1].flags,
                RPR_COLLISION_EVENT_SENSOR | RPR_COLLISION_EVENT_REMOVED
            );
            assert_eq!(rpr_free_event_collector(events), RPR_OK);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    struct HookState {
        counts: Mutex<Vec<(usize, usize)>>,
    }
    unsafe extern "C" fn edit_each_contact(
        data: *mut c_void,
        _read: *const RprReadContext,
        _a: RprColliderHandle,
        _b: RprColliderHandle,
        context: *mut RprContactModificationContext,
    ) {
        unsafe {
            assert_eq!(rpr_contact_modification_context_is_soft(context), 0);
            let before = rpr_contact_modification_context_solver_contact_count(context);
            rpr_contact_modification_context_solver_contact(context, before);
            assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
            assert_eq!(
                rpr_contact_modification_context_remove_solver_contact(context, before),
                RPR_INVALID_ARGUMENT
            );
            if before > 1 {
                assert_eq!(
                    rpr_contact_modification_context_remove_solver_contact(context, 0),
                    RPR_OK
                );
            }
            let after = rpr_contact_modification_context_solver_contact_count(context);
            for i in 0..after {
                let mut contact = rpr_contact_modification_context_solver_contact(context, i);
                assert_eq!(rpr_last_status(), RPR_OK);
                contact.tangent_velocity.x = 10.0;
                assert_eq!(
                    rpr_contact_modification_context_set_solver_contact(context, i, &contact),
                    RPR_OK
                );
                contact.distance = RprReal::NAN;
                assert_eq!(
                    rpr_contact_modification_context_set_solver_contact(context, i, &contact),
                    RPR_INVALID_ARGUMENT
                );
            }
            let state = &*data.cast::<HookState>();
            state.counts.lock().unwrap().push((before, after));
        }
    }

    #[test]
    fn hooks_edit_solver_contacts_one_by_one() {
        unsafe {
            let mut cube = rpr_cuboid_collider_desc(Vector::splat(0.5).into());
            cube.activeHooks = RPR_MODIFY_SOLVER_CONTACTS;
            let (world, ground, cube) = ball_on_ground(cube);
            let state = HookState {
                counts: Mutex::new(Vec::new()),
            };
            let hooks = RprPhysicsHooks {
                user_data: (&state as *const HookState).cast_mut().cast(),
                modify_solver_contacts_context: Some(edit_each_contact),
                ..Default::default()
            };
            for _ in 0..3 {
                assert_eq!(rpr_step(world, &hooks, ptr::null()), RPR_OK);
            }
            let counts = state.counts.lock().unwrap().clone();
            assert!(!counts.is_empty());
            assert!(
                counts
                    .iter()
                    .all(|(b, a)| *a == if *b > 1 { b - 1 } else { *b })
            );
            assert!(counts.iter().any(|(b, _)| *b > 1));
            let mut contacts = [RprSolverContact::default(); 8];
            let count = rpr_solver_contacts(cube, ground, 0, contacts.as_mut_ptr(), 8);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert_eq!(count, counts.last().unwrap().1);
            assert!(
                contacts[..count]
                    .iter()
                    .all(|c| c.tangent_velocity.x == 10.0)
            );
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }

    #[test]
    fn debug_render_style_round_trips_and_applies_colors() {
        unsafe {
            let style = rpr_default_debug_render_style();
            assert_eq!(style.raw().unwrap(), DebugRenderStyle::default());
            let (world, _ground, _ball) = ball_on_ground(rpr_ball_collider_desc(0.5));
            let mode = 1; // Collider shapes.
            let default_count = rpr_debug_render(world, mode, ptr::null_mut(), 0);
            let count = rpr_debug_render_with_style(world, mode, &style, ptr::null_mut(), 0);
            assert_eq!((rpr_last_status(), count), (RPR_OK, default_count));
            let null_count =
                rpr_debug_render_with_style(world, mode, ptr::null(), ptr::null_mut(), 0);
            assert_eq!(null_count, default_count);

            let mut blue = style;
            blue.collider_dynamic_color = [240.0, 1.0, 0.5, 1.0];
            let mut lines = vec![RprDebugLine::default(); count];
            rpr_debug_render_with_style(world, mode, &blue, lines.as_mut_ptr(), count);
            assert_eq!(rpr_last_status(), RPR_OK);
            assert!(lines.iter().any(|l| l.color == blue.collider_dynamic_color));
            assert!(lines.iter().any(|l| l.color == style.collider_fixed_color));

            let mut invalid = style;
            invalid.subdivisions = 0;
            rpr_debug_render_with_style(world, mode, &invalid, ptr::null_mut(), 0);
            assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
            invalid = style;
            invalid.contact_normal_length = -1.0;
            rpr_debug_render_with_style(world, mode, &invalid, ptr::null_mut(), 0);
            assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }
}
