use crate::dynamics::{
    RawCCDSolver, RawImpulseJointSet, RawIntegrationParameters, RawIslandManager,
    RawMultibodyJointSet, RawRigidBodySet,
};
use crate::geometry::{RawBroadPhase, RawColliderSet, RawNarrowPhase};
use crate::math::RawVector;
use crate::pipeline::{RawEventQueue, RawPhysicsHooks};
use crate::rapier::pipeline::PhysicsPipeline;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawPhysicsPipeline(pub(crate) PhysicsPipeline);

#[wasm_bindgen]
impl RawPhysicsPipeline {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        let mut pipeline = PhysicsPipeline::new();
        pipeline.counters.disable(); // Disable perf counters by default.
        RawPhysicsPipeline(pipeline)
    }

    pub fn set_profiler_enabled(&mut self, enabled: bool) {
        if enabled {
            self.0.counters.enable();
        } else {
            self.0.counters.disable();
        }
    }

    pub fn is_profiler_enabled(&self) -> bool {
        self.0.counters.enabled()
    }

    pub fn timing_step(&self) -> f64 {
        self.0.counters.step_time_ms()
    }

    pub fn timing_collision_detection(&self) -> f64 {
        self.0.counters.collision_detection_time_ms()
    }

    pub fn timing_broad_phase(&self) -> f64 {
        self.0.counters.broad_phase_time_ms()
    }

    pub fn timing_narrow_phase(&self) -> f64 {
        self.0.counters.narrow_phase_time_ms()
    }

    pub fn timing_solver(&self) -> f64 {
        self.0.counters.solver_time_ms()
    }

    pub fn timing_velocity_assembly(&self) -> f64 {
        self.0.counters.solver.velocity_assembly_time.time_ms()
    }

    pub fn timing_velocity_resolution(&self) -> f64 {
        self.0.counters.velocity_resolution_time_ms()
    }

    pub fn timing_velocity_update(&self) -> f64 {
        self.0.counters.velocity_update_time_ms()
    }

    pub fn timing_velocity_writeback(&self) -> f64 {
        self.0.counters.solver.velocity_writeback_time.time_ms()
    }

    pub fn timing_ccd(&self) -> f64 {
        self.0.counters.ccd_time_ms()
    }

    pub fn timing_ccd_toi_computation(&self) -> f64 {
        self.0.counters.ccd.toi_computation_time.time_ms()
    }

    pub fn timing_ccd_broad_phase(&self) -> f64 {
        self.0.counters.ccd.broad_phase_time.time_ms()
    }

    pub fn timing_ccd_narrow_phase(&self) -> f64 {
        self.0.counters.ccd.narrow_phase_time.time_ms()
    }

    pub fn timing_ccd_solver(&self) -> f64 {
        self.0.counters.ccd.solver_time.time_ms()
    }

    pub fn timing_island_construction(&self) -> f64 {
        self.0.counters.island_construction_time_ms()
    }

    pub fn timing_user_changes(&self) -> f64 {
        self.0.counters.stages.user_changes.time_ms()
    }

    pub fn step(
        &mut self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
        ccd_solver: &mut RawCCDSolver,
    ) {
        self.0.step(
            &gravity.0,
            &integrationParameters.0,
            &mut islands.0,
            &mut broadPhase.0,
            &mut narrowPhase.0,
            &mut bodies.0,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            &mut ccd_solver.0,
            &(),
            &(),
        );
    }

    pub fn stepWithEvents(
        &mut self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
        ccd_solver: &mut RawCCDSolver,
        eventQueue: &mut RawEventQueue,
        hookObject: js_sys::Object,
        hookFilterContactPair: js_sys::Function,
        hookFilterIntersectionPair: js_sys::Function,
    ) {
        if eventQueue.auto_drain {
            eventQueue.clear();
        }

        let hooks = RawPhysicsHooks {
            this: hookObject,
            filter_contact_pair: hookFilterContactPair,
            filter_intersection_pair: hookFilterIntersectionPair,
        };

        self.0.step(
            &gravity.0,
            &integrationParameters.0,
            &mut islands.0,
            &mut broadPhase.0,
            &mut narrowPhase.0,
            &mut bodies.0,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            &mut ccd_solver.0,
            &hooks,
            &eventQueue.collector,
        );
    }
}
