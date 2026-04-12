use crate::dynamics::{
    CCDSolver, GenericJoint, ImpulseJointHandle, ImpulseJointSet, IntegrationParameters,
    IslandManager, MultibodyJointHandle, MultibodyJointSet, RigidBody, RigidBodyHandle,
    RigidBodySet,
};
use crate::geometry::{
    BroadPhaseBvh, Collider, ColliderHandle, ColliderSet, ContactPair, DefaultBroadPhase,
    NarrowPhase,
};
use crate::math::{Real, Vector};
use crate::pipeline::{EventHandler, PhysicsHooks, PhysicsPipeline, QueryFilter, QueryPipeline};
use parry::query::details::ShapeCastOptions;
use parry::query::{NonlinearRigidMotion, ShapeCastHit};
use parry::shape::Shape;

use crate::geometry::{PointProjection, Ray, RayIntersection};
use crate::math::Pose;

#[cfg(feature = "debug-render")]
use crate::pipeline::{DebugRenderBackend, DebugRenderPipeline};

/// A convenience wrapper that bundles all the Rapier physics state into a single struct.
///
/// Rapier intentionally splits its state across many structs (`RigidBodySet`, `ColliderSet`,
/// `NarrowPhase`, etc.) to give you fine-grained control over borrowing. This is important
/// for advanced use cases, but it makes simple setups verbose.
///
/// `PhysicsWorld` gives you a single struct for the common case. All fields are `pub`, so
/// you can always reach in and borrow individual fields when the borrow checker requires it.
///
/// # Example
/// ```
/// # use rapier3d::prelude::*;
/// let mut world = PhysicsWorld::default();
///
/// // Create a ground plane
/// let (_ground, _) = world.insert(
///     RigidBodyBuilder::fixed(),
///     ColliderBuilder::cuboid(10.0, 0.1, 10.0),
/// );
///
/// // Create a falling ball
/// let (ball, _) = world.insert(
///     RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 5.0, 0.0)),
///     ColliderBuilder::ball(0.5),
/// );
///
/// // Simulate 100 steps
/// for _ in 0..100 {
///     world.step();
/// }
///
/// println!("Ball position: {:?}", world.bodies[ball].translation());
/// ```
pub struct PhysicsWorld {
    /// Gravity applied to all dynamic bodies each step.
    pub gravity: Vector,
    /// Parameters controlling the simulation (timestep, solver iterations, etc.).
    pub integration_parameters: IntegrationParameters,
    /// The main simulation pipeline that orchestrates each physics step.
    pub physics_pipeline: PhysicsPipeline,
    /// Manages active/sleeping body groups (islands) for efficient simulation.
    pub islands: IslandManager,
    /// The broad-phase acceleration structure for fast spatial queries.
    pub broad_phase: BroadPhaseBvh,
    /// Precise contact and intersection detection between collider pairs.
    pub narrow_phase: NarrowPhase,
    /// All rigid bodies in this world.
    pub bodies: RigidBodySet,
    /// All colliders (collision shapes) in this world.
    pub colliders: ColliderSet,
    /// All impulse-based joints (hinges, springs, ropes, etc.).
    pub impulse_joints: ImpulseJointSet,
    /// All multibody joints (kinematic chains, articulations).
    pub multibody_joints: MultibodyJointSet,
    /// The continuous collision detection solver.
    pub ccd_solver: CCDSolver,
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self {
            gravity: Vector::Y * -9.81,
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::default(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
        }
    }
}

impl PhysicsWorld {
    /// Creates a new physics world with default parameters and gravity `(0, -9.81, 0)`.
    pub fn new() -> Self {
        Self::default()
    }

    // ── Simulation ──────────────────────────────────────────────────────

    /// Advance the simulation by one timestep, using no hooks and no event handler.
    ///
    /// This is the simplest way to step. If you need collision events or physics hooks,
    /// use [`step_with_events`](Self::step_with_events).
    pub fn step(&mut self) {
        self.step_with_events(&(), &());
    }

    /// Advance the simulation by one timestep with custom physics hooks and event handling.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut world = PhysicsWorld::default();
    /// # use std::sync::mpsc::channel;
    /// let (collision_send, collision_recv) = channel();
    /// let (contact_force_send, contact_force_recv) = channel();
    /// let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);
    ///
    /// world.step_with_events(&(), &event_handler);
    ///
    /// while let Ok(event) = collision_recv.try_recv() {
    ///     println!("Collision event: {:?}", event);
    /// }
    /// ```
    pub fn step_with_events(
        &mut self,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        self.physics_pipeline.step(
            self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            hooks,
            events,
        );
    }

    // ── Rigid bodies ────────────────────────────────────────────────────

    /// Insert a rigid body with an attached collider, and return both handles.
    ///
    /// This bundles the two most common setup steps — creating a body and attaching
    /// its collider — into a single call. For the rare case of a rigid body without
    /// any collider (typically an anchor body for joints), use
    /// [`insert_body`](Self::insert_body) instead. For compound bodies with multiple
    /// colliders, pass the first collider here and use
    /// [`insert_collider_with_parent`](Self::insert_collider_with_parent) for the rest.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut world = PhysicsWorld::default();
    /// let (body, collider) = world.insert(
    ///     RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 5.0, 0.0)),
    ///     ColliderBuilder::ball(0.5),
    /// );
    /// ```
    pub fn insert(
        &mut self,
        body: impl Into<RigidBody>,
        collider: impl Into<Collider>,
    ) -> (RigidBodyHandle, ColliderHandle) {
        let body_handle = self.bodies.insert(body);
        let collider_handle =
            self.colliders
                .insert_with_parent(collider, body_handle, &mut self.bodies);
        (body_handle, collider_handle)
    }

    /// Insert a rigid body without any collider, and return its handle.
    ///
    /// Most bodies should be inserted together with a collider — prefer
    /// [`insert`](Self::insert) when possible. Use this method for bodies that
    /// truly don't need a collider, such as anchor bodies used purely as joint
    /// attachment points.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut world = PhysicsWorld::default();
    /// let anchor = world.insert_body(RigidBodyBuilder::fixed());
    /// ```
    pub fn insert_body(&mut self, body: impl Into<RigidBody>) -> RigidBodyHandle {
        self.bodies.insert(body)
    }

    /// Remove a rigid body and all its attached colliders and joints.
    ///
    /// Returns the removed body, or `None` if the handle was invalid.
    pub fn remove_body(&mut self, handle: RigidBodyHandle) -> Option<RigidBody> {
        self.bodies.remove(
            handle,
            &mut self.islands,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        )
    }

    // ── Colliders ───────────────────────────────────────────────────────

    /// Insert a collider attached to a rigid body, and return its handle.
    ///
    /// The collider's position is relative to its parent body.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut world = PhysicsWorld::default();
    /// let body = world.insert_body(RigidBodyBuilder::dynamic());
    /// let collider = world.insert_collider_with_parent(ColliderBuilder::ball(0.5), body);
    /// ```
    pub fn insert_collider_with_parent(
        &mut self,
        collider: impl Into<Collider>,
        parent: RigidBodyHandle,
    ) -> ColliderHandle {
        self.colliders
            .insert_with_parent(collider, parent, &mut self.bodies)
    }

    /// Insert a standalone collider (not attached to any body) and return its handle.
    ///
    /// Standalone colliders are useful for static collision geometry or sensors
    /// that don't need a rigid body.
    pub fn insert_collider(&mut self, collider: impl Into<Collider>) -> ColliderHandle {
        self.colliders.insert(collider)
    }

    /// Remove a collider from the world.
    ///
    /// Returns the removed collider, or `None` if the handle was invalid.
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> Option<Collider> {
        self.colliders
            .remove(handle, &mut self.islands, &mut self.bodies, true)
    }

    // ── Impulse joints ──────────────────────────────────────────────────

    /// Insert an impulse joint between two bodies and return its handle.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut world = PhysicsWorld::default();
    /// let body1 = world.insert_body(RigidBodyBuilder::dynamic());
    /// let body2 = world.insert_body(RigidBodyBuilder::dynamic());
    /// let joint = world.insert_impulse_joint(body1, body2, RevoluteJointBuilder::new(Vector::Z));
    /// ```
    pub fn insert_impulse_joint(
        &mut self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
        joint: impl Into<GenericJoint>,
    ) -> ImpulseJointHandle {
        self.impulse_joints.insert(body1, body2, joint, true)
    }

    /// Remove an impulse joint.
    ///
    /// Returns the removed joint data, or `None` if the handle was invalid.
    pub fn remove_impulse_joint(&mut self, handle: ImpulseJointHandle) -> Option<GenericJoint> {
        self.impulse_joints
            .remove(handle, true)
            .map(|j| j.data)
    }

    // ── Multibody joints ────────────────────────────────────────────────

    /// Insert a multibody joint between two bodies and return its handle.
    ///
    /// Returns `None` if the joint would create an invalid kinematic chain (e.g. a cycle).
    pub fn insert_multibody_joint(
        &mut self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
        joint: impl Into<GenericJoint>,
    ) -> Option<MultibodyJointHandle> {
        self.multibody_joints.insert(body1, body2, joint, true)
    }

    /// Remove a multibody joint.
    pub fn remove_multibody_joint(&mut self, handle: MultibodyJointHandle) {
        self.multibody_joints.remove(handle, true);
    }

    // ── Scene queries ───────────────────────────────────────────────────

    /// Get a [`QueryPipeline`] for performing spatial queries (raycasts, shape casts, etc.).
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut world = PhysicsWorld::default();
    /// # let (ground, _) = world.insert(
    /// #     RigidBodyBuilder::fixed(),
    /// #     ColliderBuilder::cuboid(10.0, 0.1, 10.0),
    /// # );
    /// # world.step();
    /// let query_pipeline = world.query_pipeline();
    /// let ray = Ray::new(Vector::new(0.0, 10.0, 0.0), Vector::new(0.0, -1.0, 0.0));
    /// if let Some((handle, toi)) = query_pipeline.cast_ray(&ray, Real::MAX, true) {
    ///     println!("Hit {:?} at distance {}", handle, toi);
    /// }
    /// ```
    pub fn query_pipeline(&self) -> QueryPipeline<'_> {
        self.query_pipeline_with_filter(QueryFilter::default())
    }

    /// Get a [`QueryPipeline`] with a custom [`QueryFilter`].
    pub fn query_pipeline_with_filter<'a>(
        &'a self,
        filter: QueryFilter<'a>,
    ) -> QueryPipeline<'a> {
        self.broad_phase.as_query_pipeline(
            self.narrow_phase.query_dispatcher(),
            &self.bodies,
            &self.colliders,
            filter,
        )
    }

    /// Cast a ray and return the first collider hit.
    ///
    /// Shorthand for `world.query_pipeline().cast_ray(...)`.
    ///
    /// Returns `Some((collider_handle, distance))`, or `None` if nothing was hit.
    pub fn cast_ray(
        &self,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
    ) -> Option<(ColliderHandle, Real)> {
        self.query_pipeline().cast_ray(ray, max_toi, solid)
    }

    /// Cast a ray and return the first hit with surface normal information.
    ///
    /// Shorthand for `world.query_pipeline().cast_ray_and_get_normal(...)`.
    pub fn cast_ray_and_get_normal(
        &self,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
    ) -> Option<(ColliderHandle, RayIntersection)> {
        self.query_pipeline()
            .cast_ray_and_get_normal(ray, max_toi, solid)
    }

    /// Cast (sweep) a shape through the world and return the first collider hit.
    ///
    /// Shorthand for `world.query_pipeline().cast_shape(...)`.
    pub fn cast_shape(
        &self,
        shape_pos: &Pose,
        shape_vel: Vector,
        shape: &dyn Shape,
        options: ShapeCastOptions,
    ) -> Option<(ColliderHandle, ShapeCastHit)> {
        self.query_pipeline()
            .cast_shape(shape_pos, shape_vel, shape, options)
    }

    /// Cast a shape with a nonlinear motion and return the first collider hit.
    ///
    /// Shorthand for `world.query_pipeline().cast_shape_nonlinear(...)`.
    pub fn cast_shape_nonlinear(
        &self,
        shape_motion: &NonlinearRigidMotion,
        shape: &dyn Shape,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
    ) -> Option<(ColliderHandle, ShapeCastHit)> {
        self.query_pipeline()
            .cast_shape_nonlinear(shape_motion, shape, start_time, end_time, stop_at_penetration)
    }

    /// Find the closest point on any collider to the given point.
    ///
    /// Shorthand for `world.query_pipeline().project_point(...)`.
    pub fn project_point(
        &self,
        point: Vector,
        max_dist: Real,
        solid: bool,
    ) -> Option<(ColliderHandle, PointProjection)> {
        self.query_pipeline().project_point(point, max_dist, solid)
    }

    // ── Contact / intersection queries (from NarrowPhase) ───────────────

    /// Get the contact pair between two specific colliders, if it exists.
    ///
    /// Returns `None` if the two colliders are not in contact or not neighbors
    /// in the broad-phase.
    pub fn contact_pair(
        &self,
        collider1: ColliderHandle,
        collider2: ColliderHandle,
    ) -> Option<&ContactPair> {
        self.narrow_phase.contact_pair(collider1, collider2)
    }

    /// Iterate over all contact pairs involving the given collider.
    pub fn contact_pairs_with(
        &self,
        collider: ColliderHandle,
    ) -> impl Iterator<Item = &ContactPair> {
        self.narrow_phase.contact_pairs_with(collider)
    }

    /// Iterate over all contact pairs in the world.
    pub fn contact_pairs(&self) -> impl Iterator<Item = &ContactPair> {
        self.narrow_phase.contact_pairs()
    }

    /// Check if two specific colliders are intersecting (for sensor colliders).
    ///
    /// Returns `None` if the pair doesn't exist, `Some(true)` if intersecting.
    pub fn intersection_pair(
        &self,
        collider1: ColliderHandle,
        collider2: ColliderHandle,
    ) -> Option<bool> {
        self.narrow_phase.intersection_pair(collider1, collider2)
    }

    /// Iterate over all intersection pairs involving the given collider.
    pub fn intersection_pairs_with(
        &self,
        collider: ColliderHandle,
    ) -> impl Iterator<Item = (ColliderHandle, ColliderHandle, bool)> + '_ {
        self.narrow_phase.intersection_pairs_with(collider)
    }

    /// Iterate over all intersection pairs in the world.
    pub fn intersection_pairs(
        &self,
    ) -> impl Iterator<Item = (ColliderHandle, ColliderHandle, bool)> + '_ {
        self.narrow_phase.intersection_pairs()
    }

    // ── Debug rendering ─────────────────────────────────────────────────

    /// Render debug information using the given debug-render pipeline and backend.
    ///
    /// This is a convenience method that passes all the required state to the
    /// debug-render pipeline.
    #[cfg(feature = "debug-render")]
    pub fn debug_render(&self, pipeline: &mut DebugRenderPipeline, backend: &mut impl DebugRenderBackend) {
        pipeline.render(
            backend,
            &self.bodies,
            &self.colliders,
            &self.impulse_joints,
            &self.multibody_joints,
            &self.narrow_phase,
        );
    }
}
