use crate::dynamics::RawRigidBodySet;
use crate::geometry::{
    RawColliderSet, RawColliderShapeCastHit, RawNarrowPhase, RawPointColliderProjection,
    RawRayColliderHit, RawRayColliderIntersection, RawShape,
};
use crate::math::{RawRotation, RawVector};
use crate::utils::{self, FlatHandle};
use rapier::geometry::DefaultBroadPhase;
use rapier::geometry::{Aabb, ColliderHandle, Ray};
use rapier::math::{Isometry, Point};
use rapier::parry::query::ShapeCastOptions;
use rapier::pipeline::{QueryFilter, QueryFilterFlags};
use rapier::prelude::FeatureId;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawBroadPhase(pub(crate) DefaultBroadPhase);

#[wasm_bindgen]
impl RawBroadPhase {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawBroadPhase(DefaultBroadPhase::new())
    }

    pub fn castRay(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        rayOrig: &RawVector,
        rayDir: &RawVector,
        maxToi: f32,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> Option<RawRayColliderHit> {
        let (handle, timeOfImpact) = utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.0,
                &colliders.0,
                query_filter,
            );

            let ray = Ray::new(rayOrig.0.into(), rayDir.0);
            query_pipeline.cast_ray(&ray, maxToi, solid)
        })?;

        Some(RawRayColliderHit {
            handle,
            timeOfImpact,
        })
    }

    pub fn castRayAndGetNormal(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        rayOrig: &RawVector,
        rayDir: &RawVector,
        maxToi: f32,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> Option<RawRayColliderIntersection> {
        let (handle, inter) = utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.0,
                &colliders.0,
                query_filter,
            );

            let ray = Ray::new(rayOrig.0.into(), rayDir.0);
            query_pipeline.cast_ray_and_get_normal(&ray, maxToi, solid)
        })?;

        Some(RawRayColliderIntersection { handle, inter })
    }

    // The callback is of type (RawRayColliderIntersection) => bool
    pub fn intersectionsWithRay(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        rayOrig: &RawVector,
        rayDir: &RawVector,
        maxToi: f32,
        solid: bool,
        callback: &js_sys::Function,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let ray = Ray::new(rayOrig.0.into(), rayDir.0);
            let rcallback = |handle, inter| {
                let result = RawRayColliderIntersection { handle, inter };
                match callback.call1(&JsValue::null(), &JsValue::from(result)) {
                    Err(_) => true,
                    Ok(val) => val.as_bool().unwrap_or(true),
                }
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.0,
                &colliders.0,
                query_filter,
            );

            for (handle, _, inter) in query_pipeline.intersect_ray(ray, maxToi, solid) {
                if !rcallback(handle, inter) {
                    break;
                }
            }
        });
    }

    pub fn intersectionWithShape(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        shape: &RawShape,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> Option<FlatHandle> {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.0,
                &colliders.0,
                query_filter,
            );

            let pos = Isometry::from_parts(shapePos.0.into(), shapeRot.0);

            // TODO: take a callback as argument so we can yield all the intersecting shapes?
            for (handle, _) in query_pipeline.intersect_shape(pos, &*shape.0) {
                // Return the first intersection we find.
                return Some(utils::flat_handle(handle.0));
            }

            None
        })
    }

    pub fn projectPoint(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point: &RawVector,
        solid: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> Option<RawPointColliderProjection> {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.0,
                &colliders.0,
                query_filter,
            );

            query_pipeline
                .project_point(&point.0.into(), f32::MAX, solid)
                .map(|(handle, proj)| RawPointColliderProjection {
                    handle,
                    proj,
                    feature: FeatureId::Unknown,
                })
        })
    }

    pub fn projectPointAndGetFeature(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point: &RawVector,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> Option<RawPointColliderProjection> {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.0,
                &colliders.0,
                query_filter,
            );

            query_pipeline
                .project_point_and_get_feature(&point.0.into())
                .map(|(handle, proj, feature)| RawPointColliderProjection {
                    handle,
                    proj,
                    feature,
                })
        })
    }

    // The callback is of type (u32) => bool
    pub fn intersectionsWithPoint(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        point: &RawVector,
        callback: &js_sys::Function,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.0,
                &colliders.0,
                query_filter,
            );

            let rcallback = |handle: ColliderHandle| match callback.call1(
                &JsValue::null(),
                &JsValue::from(utils::flat_handle(handle.0)),
            ) {
                Err(_) => true,
                Ok(val) => val.as_bool().unwrap_or(true),
            };

            for (handle, _) in query_pipeline.intersect_point(point.0.into()) {
                if !rcallback(handle) {
                    break;
                }
            }
        });
    }

    pub fn castShape(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        shapeVel: &RawVector,
        shape: &RawShape,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) -> Option<RawColliderShapeCastHit> {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.0,
                &colliders.0,
                query_filter,
            );

            let pos = Isometry::from_parts(shapePos.0.into(), shapeRot.0);
            query_pipeline
                .cast_shape(
                    &pos,
                    &shapeVel.0,
                    &*shape.0,
                    ShapeCastOptions {
                        max_time_of_impact: maxToi,
                        stop_at_penetration,
                        compute_impact_geometry_on_penetration: true,
                        target_distance,
                    },
                )
                .map(|(handle, hit)| RawColliderShapeCastHit { handle, hit })
        })
    }

    // The callback has type (u32) => boolean
    pub fn intersectionsWithShape(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        shape: &RawShape,
        callback: &js_sys::Function,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_exclude_collider: Option<FlatHandle>,
        filter_exclude_rigid_body: Option<FlatHandle>,
        filter_predicate: &js_sys::Function,
    ) {
        utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                exclude_collider: filter_exclude_collider.map(crate::utils::collider_handle),
                exclude_rigid_body: filter_exclude_rigid_body.map(crate::utils::body_handle),
                predicate,
            };

            let query_pipeline = self.0.as_query_pipeline(
                narrow_phase.0.query_dispatcher(),
                &bodies.0,
                &colliders.0,
                query_filter,
            );

            let rcallback = |handle: ColliderHandle| match callback.call1(
                &JsValue::null(),
                &JsValue::from(utils::flat_handle(handle.0)),
            ) {
                Err(_) => true,
                Ok(val) => val.as_bool().unwrap_or(true),
            };

            let pos = Isometry::from_parts(shapePos.0.into(), shapeRot.0);
            for (handle, _) in query_pipeline.intersect_shape(pos, &*shape.0) {
                if !rcallback(handle) {
                    break;
                }
            }
        })
    }

    pub fn collidersWithAabbIntersectingAabb(
        &self,
        narrow_phase: &RawNarrowPhase,
        bodies: &RawRigidBodySet,
        colliders: &RawColliderSet,
        aabbCenter: &RawVector,
        aabbHalfExtents: &RawVector,
        callback: &js_sys::Function,
    ) {
        let rcallback = |handle: &ColliderHandle| match callback.call1(
            &JsValue::null(),
            &JsValue::from(utils::flat_handle(handle.0)),
        ) {
            Err(_) => true,
            Ok(val) => val.as_bool().unwrap_or(true),
        };

        let query_pipeline = self.0.as_query_pipeline(
            narrow_phase.0.query_dispatcher(),
            &bodies.0,
            &colliders.0,
            Default::default(),
        );

        let center = Point::from(aabbCenter.0);
        let aabb = Aabb::new(center - aabbHalfExtents.0, center + aabbHalfExtents.0);

        for (handle, _) in query_pipeline.intersect_aabb_conservative(aabb) {
            if !rcallback(&handle) {
                break;
            }
        }
    }
}
