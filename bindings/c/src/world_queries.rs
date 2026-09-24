//! World queries with operation-scoped native access.
#![allow(non_snake_case)]
use crate::*;
use rapier::parry::bounding_volume::Aabb;
#[derive(Clone, Copy)]
pub(crate) struct QueryAccess {
    pub world: *mut RprWorld,
    pub broadPhase: *const RprBroadPhaseBvh,
    pub narrowPhase: *const RprNarrowPhase,
    pub bodies: *const RprRigidBodySet,
    pub colliders: *const RprColliderSet,
    pub filter: RprQueryFilter,
    pub predicate: RprQueryPredicate,
    pub userData: *mut std::ffi::c_void,
}
/// Copyable query settings. They borrow callback data, never world components.
/// @ingroup queries
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RprQueryOptions {
    /// Query selection settings.
    pub filter: RprQueryFilter,
    /// Optional additional query filter; nonzero accepts a collider.
    pub predicate: RprQueryPredicate,
    /// Application data; Rapier does not own pointers encoded in it.
    pub userData: *mut std::ffi::c_void,
}
impl Default for RprQueryOptions {
    fn default() -> Self {
        Self {
            filter: RprQueryFilter::default(),
            predicate: None,
            userData: std::ptr::null_mut(),
        }
    }
}
/// Return native default query options. This POD value owns no resources.
/// @ingroup queries
#[rapier_export]
pub extern "C" fn rpr_default_query_options() -> RprQueryOptions {
    RprQueryOptions::default()
}
impl QueryAccess {
    pub(crate) unsafe fn from_world(
        owner: *mut RprWorld,
        world: *const RprPhysicsWorld,
        query_options: *const RprQueryOptions,
    ) -> Result<Self> {
        unsafe {
            let options = if query_options.is_null() {
                RprQueryOptions::default()
            } else {
                *get(query_options)?
            };
            let w = &get(world)?.0;
            options.filter.check_world(owner)?;
            options.filter.raw()?;
            Ok(Self {
                world: owner,
                broadPhase: std::ptr::addr_of!(w.broad_phase).cast(),
                narrowPhase: std::ptr::addr_of!(w.narrow_phase).cast(),
                bodies: std::ptr::addr_of!(w.bodies).cast(),
                colliders: std::ptr::addr_of!(w.colliders).cast(),
                filter: options.filter,
                predicate: options.predicate,
                userData: options.userData,
            })
        }
    }

    pub(crate) unsafe fn with_raw<T>(
        &self,
        f: impl FnOnce(QueryPipeline<'_>) -> Result<T>,
    ) -> Result<T> {
        unsafe {
            let read =
                RprReadContext::new(self.world, &get(self.bodies)?.0, &get(self.colliders)?.0);
            let callback = |handle: ColliderHandle, _collider: &Collider| {
                self.predicate.is_none_or(|p| {
                    p(
                        self.userData,
                        &read,
                        RprColliderHandle::from(handle).with_world(self.world),
                    ) != 0
                })
            };
            let mut filter = self.filter.raw()?;
            if self.predicate.is_some() {
                filter.predicate = Some(&callback);
            }
            let query = get(self.broadPhase)?.0.as_query_pipeline(
                get(self.narrowPhase)?.0.query_dispatcher(),
                &get(self.bodies)?.0,
                &get(self.colliders)?.0,
                filter,
            );
            f(query)
        }
    }
}

/// Return the closest ray hit, or report RPR_NOT_FOUND on a miss. The ray is origin + direction * t
/// for 0 <= t <= max_toi; direction need not be normalized. solid treats an interior origin as a
/// hit at t = 0.
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_cast_ray(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    origin: RprVector,
    direction: RprVector,
    max_toi: RprReal,
    solid: RprBool,
) -> RprRayHit {
    ffi_world_value(world, |out: *mut RprRayHit| {
        ffi(|| unsafe {
            if !query_options.is_null() {
                get(query_options)?.check_world(world)?;
            }
            let access = get(world)?.read()?;
            let raw = access.raw();
            let query = QueryAccess::from_world(world as *mut RprWorld, raw, query_options)?;
            let query: *const QueryAccess = &query;

            let origin = origin.raw()?;
            let direction = direction.raw()?;
            positive(direction.length())?;
            nonnegative(max_toi)?;
            let solid = boolean(solid)?;
            get(query)?.with_raw(|q| {
                let (h, hit) = q
                    .cast_ray_and_get_normal(&Ray::new(origin, direction), max_toi, solid)
                    .ok_or((RPR_NOT_FOUND, "ray missed".into()))?;
                let (feature_type, feature_id) = feature(hit.feature);
                output(
                    out,
                    RprRayHit {
                        collider: h.into(),
                        time_of_impact: hit.time_of_impact,
                        normal: hit.normal.into(),
                        feature_type,
                        feature_id,
                    },
                )
            })
        })
    })
}

/// Return the closest surface projection within max_distance, or report RPR_NOT_FOUND. With solid =
/// 1, an interior point projects to itself.
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_project_point(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    point: RprVector,
    max_distance: RprReal,
    solid: RprBool,
) -> RprPointProjection {
    ffi_world_value(world, |out: *mut RprPointProjection| {
        ffi(|| unsafe {
            if !query_options.is_null() {
                get(query_options)?.check_world(world)?;
            }
            let access = get(world)?.read()?;
            let raw = access.raw();
            let query = QueryAccess::from_world(world as *mut RprWorld, raw, query_options)?;
            let query: *const QueryAccess = &query;

            let p = point.raw()?;
            nonnegative(max_distance)?;
            let solid = boolean(solid)?;
            get(query)?.with_raw(|q| {
                let (h, p) = q
                    .project_point(p, max_distance, solid)
                    .ok_or((RPR_NOT_FOUND, "no projection".into()))?;
                output(
                    out,
                    RprPointProjection {
                        collider: h.into(),
                        point: p.point.into(),
                        is_inside: p.is_inside as u32,
                    },
                )
            })
        })
    })
}

/// Sweep shape from pose along velocity and return the first hit; report RPR_NOT_FOUND on a miss.
/// Time is bounded by options.max_time_of_impact.
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_cast_shape(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    pose: RprPose,
    velocity: RprVector,
    shape: *const RprSharedShape,
    options: RprShapeCastOptions,
) -> RprShapeCastHit {
    ffi_world_value(world, |out: *mut RprShapeCastHit| {
        ffi(|| unsafe {
            if !query_options.is_null() {
                get(query_options)?.check_world(world)?;
            }
            let access = get(world)?.read()?;
            let raw = access.raw();
            let query = QueryAccess::from_world(world as *mut RprWorld, raw, query_options)?;
            let query: *const QueryAccess = &query;

            let p = pose.raw()?;
            let v = velocity.raw()?;
            let o = options.raw()?;
            get(query)?.with_raw(|q| {
                let (h, r) = q
                    .cast_shape(&p, v, &*get(shape)?.0, o)
                    .ok_or((RPR_NOT_FOUND, "shape missed".into()))?;
                output(out, shape_cast_hit(h, r))
            })
        })
    })
}

/// Copy handles of colliders containing the world-space point.
/// @see @ref output_buffers
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_intersect_point(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    point: RprVector,
    buffer: *mut RprColliderHandle,
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
                let query: *const QueryAccess = &query;

                let point = point.raw()?;
                get(query)?.with_raw(|q| {
                    let values: Vec<_> = q.intersect_point(point).map(|(h, _)| h.into()).collect();
                    copy_out(&values, buffer, capacity, count)
                })
            })
        })
    }
}

/// Copy handles of colliders intersecting the shape at its world-space pose. The shape is borrowed
/// for this call.
/// @see @ref output_buffers
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_intersect_shape(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    pose: RprPose,
    shape: *const RprSharedShape,
    buffer: *mut RprColliderHandle,
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
                let query: *const QueryAccess = &query;

                let pose = pose.raw()?;
                get(query)?.with_raw(|q| {
                    let values: Vec<_> = q
                        .intersect_shape(pose, &*get(shape)?.0)
                        .map(|(h, _)| h.into())
                        .collect();
                    copy_out(&values, buffer, capacity, count)
                })
            })
        })
    }
}

/// Copy broad-phase candidates whose bounding boxes overlap the world-space AABB. Results may
/// include false positives.
/// @see @ref output_buffers
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_intersect_aabb_conservative(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    aabb: RprAabb,
    buffer: *mut RprColliderHandle,
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
                let query: *const QueryAccess = &query;

                let mins = aabb.mins.raw()?;
                let maxs = aabb.maxs.raw()?;
                ensure(mins.cmple(maxs).all(), "invalid AABB")?;
                get(query)?.with_raw(|q| {
                    let values: Vec<_> = q
                        .intersect_aabb_conservative(Aabb::new(mins, maxs))
                        .map(|(h, _)| h.into())
                        .collect();
                    copy_out(&values, buffer, capacity, count)
                })
            })
        })
    }
}

/// Return the closest ray collider and time, with found = 0 on a miss (RPR_OK). The ray is origin +
/// direction * t; max_toi bounds t.
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_cast_ray_toi(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    origin: RprVector,
    direction: RprVector,
    max_toi: RprReal,
    solid: RprBool,
) -> RprRayToi {
    ffi_world_value(world, |result: *mut RprRayToi| {
        let collider = unsafe { std::ptr::addr_of_mut!((*result).collider) };
        let toi = unsafe { std::ptr::addr_of_mut!((*result).toi) };
        let found = unsafe { std::ptr::addr_of_mut!((*result).found) };

        ffi(|| unsafe {
            if !query_options.is_null() {
                get(query_options)?.check_world(world)?;
            }
            let access = get(world)?.read()?;
            let raw = access.raw();
            let query = QueryAccess::from_world(world as *mut RprWorld, raw, query_options)?;
            let query: *const QueryAccess = &query;

            out_ptr(collider)?;
            out_ptr(toi)?;
            out_ptr(found)?;
            let origin = origin.raw()?;
            let direction = direction.raw()?;
            positive(direction.length())?;
            nonnegative(max_toi)?;
            get(query)?.with_raw(|q| {
                if let Some((handle, time)) =
                    q.cast_ray(&Ray::new(origin, direction), max_toi, boolean(solid)?)
                {
                    output(collider, handle.into())?;
                    output(toi, time)?;
                    output(found, 1)
                } else {
                    output(collider, Default::default())?;
                    output(toi, 0.0)?;
                    output(found, 0)
                }
            })
        })
    })
}

/// Return the closest ray hit with found = 0 on a miss (RPR_OK). The ray is origin + direction * t;
/// solid treats an interior origin as a hit at t = 0.
/// NULL query options use the default filter. Query state reflects the latest Step or
/// DetectCollisions call.
/// @ingroup queries
#[rapier_export]
pub unsafe extern "C" fn rpr_try_cast_ray(
    world: *const RprWorld,
    query_options: *const RprQueryOptions,
    origin: RprVector,
    direction: RprVector,
    max_toi: RprReal,
    solid: RprBool,
) -> RprOptionalRayHit {
    ffi_world_value(world, |result: *mut RprOptionalRayHit| {
        let out = unsafe { std::ptr::addr_of_mut!((*result).hit) };
        let found = unsafe { std::ptr::addr_of_mut!((*result).found) };

        ffi(|| unsafe {
            if !query_options.is_null() {
                get(query_options)?.check_world(world)?;
            }
            let access = get(world)?.read()?;
            let raw = access.raw();
            let query = QueryAccess::from_world(world as *mut RprWorld, raw, query_options)?;
            let query: *const QueryAccess = &query;

            out_ptr(out)?;
            out_ptr(found)?;
            let origin = origin.raw()?;
            let direction = direction.raw()?;
            positive(direction.length())?;
            nonnegative(max_toi)?;
            get(query)?.with_raw(|q| {
                if let Some((h, hit)) = q.cast_ray_and_get_normal(
                    &Ray::new(origin, direction),
                    max_toi,
                    boolean(solid)?,
                ) {
                    let (feature_type, feature_id) = feature(hit.feature);
                    output(
                        out,
                        RprRayHit {
                            collider: h.into(),
                            time_of_impact: hit.time_of_impact,
                            normal: hit.normal.into(),
                            feature_type,
                            feature_id,
                        },
                    )?;
                    output(found, 1)
                } else {
                    output(out, RprRayHit::default())?;
                    output(found, 0)
                }
            })
        })
    })
}
