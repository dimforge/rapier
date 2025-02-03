use crate::prelude::*;

/// Contains all arguments to be passed to [`PhysicsPipeline::step`]
#[allow(missing_docs)]
pub struct PhysicsContext<PH: PhysicsHooks = (), EV: EventHandler = ()> {
    pub gravity: Vector<Real>,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: Option<QueryPipeline>,
    pub hooks: PH,
    pub events: EV,
}

impl<PH: PhysicsHooks + Default, EV: EventHandler + Default> Default for PhysicsContext<PH, EV> {
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
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: Some(QueryPipeline::new()),
            hooks: PH::default(),
            events: EV::default(),
        }
    }
}

impl<PH: PhysicsHooks, EV: EventHandler> PhysicsContext<PH, EV> {
    /// Create a new physics context with given hooks and event handling.
    pub fn default_with(hooks: PH, events: EV) -> Self {
        Self {
            gravity: Vector::<Real>::default(),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: Some(QueryPipeline::new()),
            hooks,
            events,
        }
    }

    /// Shortcut to [`PhysicsPipeline::step`]
    pub fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            self.query_pipeline.as_mut(),
            &self.hooks,
            &self.events,
        );
    }
}
