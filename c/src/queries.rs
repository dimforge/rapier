use crate::*;
use rapier::parry::query::ShapeCastOptions;
#[repr(C)]
#[derive(Copy, Clone)]
pub struct RprQueryFilter {
    pub flags: u32,
    pub use_groups: RprBool,
    pub groups: RprInteractionGroups,
    pub exclude_collider: RprColliderHandle,
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
#[rapier_export]
pub extern "C" fn rpr_default_query_filter() -> RprQueryFilter {
    RprQueryFilter::default()
}

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprRayHit {
    pub collider: RprColliderHandle,
    pub time_of_impact: RprReal,
    pub normal: RprVector,
    pub feature_type: u32,
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

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprPointProjection {
    pub collider: RprColliderHandle,
    pub point: RprVector,
    pub is_inside: RprBool,
}

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprShapeCastOptions {
    pub max_time_of_impact: RprReal,
    pub target_distance: RprReal,
    pub stop_at_penetration: RprBool,
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
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprShapeCastHit {
    pub collider: RprColliderHandle,
    pub time_of_impact: RprReal,
    pub witness1: RprVector,
    pub witness2: RprVector,
    pub normal1: RprVector,
    pub normal2: RprVector,
    pub status: u32,
}

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
pub type RprQueryPredicate = Option<
    unsafe extern "C" fn(
        user_data: *mut std::ffi::c_void,
        read: *const RprReadContext,
        handle: RprColliderHandle,
    ) -> RprBool,
>;
