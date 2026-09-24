use crate::*;
use rapier::parry::query::ShapeCastOptions;
/// Scene-query flags, groups, and excluded handles. Initialize with rpr_default_query_filter.
/// @ingroup queries
#[repr(C)]
#[derive(Copy, Clone)]
pub struct RprQueryFilter {
    /// RPR_QUERY_EXCLUDE_* bitmask selecting body types and sensors/solids.
    pub flags: u32,
    /// Whether to apply the groups filter.
    pub use_groups: RprBool,
    /// Groups to test when use_groups is 1.
    pub groups: RprInteractionGroups,
    /// Collider to exclude; use the explicit invalid handle to exclude none.
    pub exclude_collider: RprColliderHandle,
    /// Body whose colliders are excluded; use the explicit invalid handle for none.
    pub exclude_rigid_body: RprRigidBodyHandle,
}
impl Default for RprQueryFilter {
    fn default() -> Self {
        Self {
            flags: 0,
            use_groups: 0,
            groups: RprInteractionGroups {
                memberships: u32::MAX,
                filter: u32::MAX,
                test_mode: 0,
            },
            exclude_collider: Default::default(),
            exclude_rigid_body: Default::default(),
        }
    }
}
impl RprQueryFilter {
    pub(crate) fn raw(self) -> Result<QueryFilter<'static>> {
        Ok(QueryFilter {
            flags: QueryFilterFlags::from_bits(self.flags)
                .ok_or_else(|| invalid("unknown query flags"))?,
            groups: if boolean(self.use_groups)? {
                Some(self.groups.raw()?)
            } else {
                None
            },
            exclude_collider: (self.exclude_collider != RprColliderHandle::default())
                .then(|| self.exclude_collider.raw()),
            exclude_rigid_body: (self.exclude_rigid_body != RprRigidBodyHandle::default())
                .then(|| self.exclude_rigid_body.raw()),
            predicate: None,
        })
    }
}
/// Return native default query filter. This POD value owns no resources.
/// @ingroup queries
#[rapier_export]
pub extern "C" fn rpr_default_query_filter() -> RprQueryFilter {
    RprQueryFilter::default()
}

/// Closest ray intersection, with a world-space normal.
/// @ingroup queries
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprRayHit {
    /// World-bound collider handle.
    pub collider: RprColliderHandle,
    /// Ray/sweep parameter at first impact, bounded by the query options.
    pub time_of_impact: RprReal,
    /// World-space contact or surface normal.
    pub normal: RprVector,
    /// Shape feature kind: RPR_FEATURE_UNKNOWN, RPR_FEATURE_VERTEX, RPR_FEATURE_EDGE, or
    /// RPR_FEATURE_FACE.
    pub feature_type: u32,
    /// Index within the feature kind; zero for unknown.
    pub feature_id: u32,
}
pub(crate) fn feature(f: rapier::parry::shape::FeatureId) -> (u32, u32) {
    use rapier::parry::shape::FeatureId;
    match f {
        FeatureId::Vertex(i) => (1, i),
        #[cfg(feature = "dim3")]
        FeatureId::Edge(i) => (2, i),
        FeatureId::Face(i) => (3, i),
        FeatureId::Unknown => (0, 0),
    }
}

/// Closest projected world-space point and its collider.
/// @ingroup queries
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprPointProjection {
    /// World-bound collider handle.
    pub collider: RprColliderHandle,
    /// Projected world-space point.
    pub point: RprVector,
    /// Whether the original point was inside the collider.
    pub is_inside: RprBool,
}

/// Sweep termination settings. Initialize with rpr_default_shape_cast_options.
/// @ingroup queries
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprShapeCastOptions {
    /// Maximum sweep parameter; movement is velocity multiplied by this time.
    pub max_time_of_impact: RprReal,
    /// Nonnegative separation at which a shape cast counts as a hit.
    pub target_distance: RprReal,
    /// Whether to stop at t = 0 for an initial overlap.
    pub stop_at_penetration: RprBool,
    /// Whether to compute witness points/normals for an initial overlap.
    pub compute_impact_geometry_on_penetration: RprBool,
}
impl RprShapeCastOptions {
    pub(crate) fn raw(self) -> Result<ShapeCastOptions> {
        Ok(ShapeCastOptions {
            max_time_of_impact: nonnegative(self.max_time_of_impact)?,
            target_distance: nonnegative(self.target_distance)?,
            stop_at_penetration: boolean(self.stop_at_penetration)?,
            compute_impact_geometry_on_penetration: boolean(
                self.compute_impact_geometry_on_penetration,
            )?,
        })
    }
}
/// Shape-cast impact geometry; collider-side data is world-space, moving-shape data is local.
/// @ingroup queries
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprShapeCastHit {
    /// World-bound collider handle.
    pub collider: RprColliderHandle,
    /// Ray/sweep parameter at first impact, bounded by the query options.
    pub time_of_impact: RprReal,
    /// Impact witness on the collider, in world coordinates.
    pub witness1: RprVector,
    /// Impact witness on the moving shape, in its local coordinates.
    pub witness2: RprVector,
    /// Impact normal on the collider, in world coordinates.
    pub normal1: RprVector,
    /// Impact normal on the moving shape, in its local coordinates.
    pub normal2: RprVector,
    /// Native result: 0 out of iterations, 1 converged, 2 failed, 3 penetrating or within target
    /// distance.
    pub status: u32,
}

/// Return native default shape cast options. This POD value owns no resources.
/// @ingroup queries
#[rapier_export]
pub extern "C" fn rpr_default_shape_cast_options() -> RprShapeCastOptions {
    let o = ShapeCastOptions::default();
    RprShapeCastOptions {
        max_time_of_impact: o.max_time_of_impact,
        target_distance: o.target_distance,
        stop_at_penetration: o.stop_at_penetration as _,
        compute_impact_geometry_on_penetration: o.compute_impact_geometry_on_penetration as _,
    }
}

/// Called with scoped read access and a collider handle. Shared queries may nest;
/// world mutations are rejected until the outer query returns. Never retain the context.
/// @ingroup queries
pub type RprQueryPredicate = Option<
    unsafe extern "C" fn(
        user_data: *mut std::ffi::c_void,
        read: *const RprReadContext,
        handle: RprColliderHandle,
    ) -> RprBool,
>;
