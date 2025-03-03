use parry::query::{NonlinearRigidMotion, ShapeCastOptions};

use crate::prelude::*;

/// Contains all arguments to be passed to [`PhysicsPipeline::step`]
#[allow(missing_docs)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct PhysicsContext {
    pub gravity: Vector<Real>,
    pub integration_parameters: IntegrationParameters,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: Option<QueryPipeline>,
}

impl Default for PhysicsContext {
    fn default() -> Self {
        Self {
            gravity: Vector::<Real>::default(),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhaseMultiSap::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: Some(QueryPipeline::new()),
        }
    }
}

impl PhysicsContext {
    /// Creates a default physics context without a query pipeline.
    pub fn default_without_query_pipeline() -> Self {
        Self {
            query_pipeline: None,
            ..PhysicsContext::default()
        }
    }

    /// Shortcut to [`PhysicsPipeline::step`]
    pub fn step(&mut self, hooks: &dyn PhysicsHooks, events: &dyn EventHandler) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            self.query_pipeline.as_mut(),
            hooks,
            events,
        );
    }

    /// Shortcut to [`ColliderSet::insert_with_parent`]
    pub fn insert_rigidbody(
        &mut self,
        parent_handle: RigidBodyHandle,
        collider: impl Into<Collider>,
    ) -> ColliderHandle {
        self.colliders
            .insert_with_parent(collider, parent_handle, &mut self.bodies)
    }

    /// Shortcut to [`RigidBodySet::insert`] and [`ColliderSet::insert_with_parent`]
    pub fn insert_body_and_collider(
        &mut self,
        rb: impl Into<RigidBody>,
        collider: impl Into<Collider>,
    ) -> (RigidBodyHandle, ColliderHandle) {
        let parent_handle = self.bodies.insert(rb);
        (
            parent_handle,
            self.colliders
                .insert_with_parent(collider, parent_handle, &mut self.bodies),
        )
    }

    /// Shortcut to [`RigidBodySet::remove`]
    pub fn remove_rigidbody(
        &mut self,
        handle: RigidBodyHandle,
        remove_attached_colliders: bool,
    ) -> Option<RigidBody> {
        self.bodies.remove(
            handle,
            &mut self.island_manager,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            remove_attached_colliders,
        )
    }
    /// Shortcut to [`ColliderSet::remove`]
    pub fn remove_collider(&mut self, handle: ColliderHandle, wake_up: bool) -> Option<Collider> {
        self.colliders
            .remove(handle, &mut self.island_manager, &mut self.bodies, wake_up)
    }
}

/// Shortcuts to [`QueryPipeline`]
impl PhysicsContext {
    /// Shortcut to [`QueryPipeline::update_incremental`]
    pub fn update_incremental(
        &mut self,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        refit_and_rebalance: bool,
    ) {
        let Some(query_pipeline) = &mut self.query_pipeline else {
            return;
        };
        query_pipeline.update_incremental(
            &self.colliders,
            modified_colliders,
            removed_colliders,
            refit_and_rebalance,
        );
    }

    /// Shortcut to [`QueryPipeline::update`]
    pub fn update(&mut self) {
        let Some(query_pipeline) = &mut self.query_pipeline else {
            return;
        };
        query_pipeline.update(&self.colliders)
    }

    /// Shortcut to [`QueryPipeline::cast_ray`]
    pub fn cast_ray(
        &self,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, Real)> {
        let Some(query_pipeline) = &self.query_pipeline else {
            return None;
        };
        query_pipeline.cast_ray(&self.bodies, &self.colliders, ray, max_toi, solid, filter)
    }

    /// Shortcut to [`QueryPipeline::cast_ray_and_get_normal`]
    pub fn cast_ray_and_get_normal(
        &self,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, RayIntersection)> {
        let Some(query_pipeline) = &self.query_pipeline else {
            return None;
        };
        query_pipeline.cast_ray_and_get_normal(
            &self.bodies,
            &self.colliders,
            ray,
            max_toi,
            solid,
            filter,
        )
    }

    /// Shortcut to [`QueryPipeline::intersections_with_ray`]
    pub fn intersections_with_ray(
        &self,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
        callback: impl FnMut(ColliderHandle, RayIntersection) -> bool,
    ) {
        let Some(query_pipeline) = &self.query_pipeline else {
            return;
        };
        query_pipeline.intersections_with_ray(
            &self.bodies,
            &self.colliders,
            ray,
            max_toi,
            solid,
            filter,
            callback,
        )
    }

    /// Shortcut to [`QueryPipeline::intersection_with_shape`]
    pub fn intersection_with_shape(
        &self,
        shape_pos: &Isometry<Real>,
        shape: &dyn Shape,
        filter: QueryFilter,
    ) -> Option<ColliderHandle> {
        let Some(query_pipeline) = &self.query_pipeline else {
            return None;
        };
        query_pipeline.intersection_with_shape(
            &self.bodies,
            &self.colliders,
            shape_pos,
            shape,
            filter,
        )
    }

    /// Shortcut to [`QueryPipeline::project_point`]
    pub fn project_point(
        &self,
        point: &Point<Real>,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, PointProjection)> {
        let Some(query_pipeline) = &self.query_pipeline else {
            return None;
        };
        query_pipeline.project_point(&self.bodies, &self.colliders, point, solid, filter)
    }

    /// Shortcut to [`QueryPipeline::intersections_with_point`]
    pub fn intersections_with_point(
        &self,
        point: &Point<Real>,
        filter: QueryFilter,
        callback: impl FnMut(ColliderHandle) -> bool,
    ) {
        let Some(query_pipeline) = &self.query_pipeline else {
            return;
        };
        query_pipeline.intersections_with_point(
            &self.bodies,
            &self.colliders,
            point,
            filter,
            callback,
        )
    }

    /// Shortcut to [`QueryPipeline::project_point_and_get_feature`]
    pub fn project_point_and_get_feature(
        &self,
        point: &Point<Real>,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, PointProjection, FeatureId)> {
        let Some(query_pipeline) = &self.query_pipeline else {
            return None;
        };
        query_pipeline.project_point_and_get_feature(&self.bodies, &self.colliders, point, filter)
    }

    /// Shortcut to [`QueryPipeline::cast_shape`]
    pub fn cast_shape(
        &self,
        shape_pos: &Isometry<Real>,
        shape_vel: &Vector<Real>,
        shape: &dyn Shape,
        options: ShapeCastOptions,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, ShapeCastHit)> {
        let Some(query_pipeline) = &self.query_pipeline else {
            return None;
        };
        query_pipeline.cast_shape(
            &self.bodies,
            &self.colliders,
            shape_pos,
            shape_vel,
            shape,
            options,
            filter,
        )
    }

    /// Shortcut to [`QueryPipeline::nonlinear_cast_shape`]
    pub fn nonlinear_cast_shape(
        &self,
        shape_motion: &NonlinearRigidMotion,
        shape: &dyn Shape,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, ShapeCastHit)> {
        let Some(query_pipeline) = &self.query_pipeline else {
            return None;
        };
        query_pipeline.nonlinear_cast_shape(
            &self.bodies,
            &self.colliders,
            shape_motion,
            shape,
            start_time,
            end_time,
            stop_at_penetration,
            filter,
        )
    }

    /// Shortcut to [`QueryPipeline::intersections_with_shape`]
    pub fn intersections_with_shape(
        &self,
        shape_pos: &Isometry<Real>,
        shape: &dyn Shape,
        filter: QueryFilter,
        callback: impl FnMut(ColliderHandle) -> bool,
    ) {
        let Some(query_pipeline) = &self.query_pipeline else {
            return;
        };
        query_pipeline.intersections_with_shape(
            &self.bodies,
            &self.colliders,
            shape_pos,
            shape,
            filter,
            callback,
        )
    }
}
