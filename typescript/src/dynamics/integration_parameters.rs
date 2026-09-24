use crate::dynamics::RawSoftRecoverySettings;
use rapier::dynamics::IntegrationParameters;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawIntegrationParameters(pub(crate) IntegrationParameters);

#[wasm_bindgen]
impl RawIntegrationParameters {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawIntegrationParameters(IntegrationParameters::default())
    }

    #[wasm_bindgen(getter)]
    pub fn dt(&self) -> f32 {
        self.0.dt
    }

    #[wasm_bindgen(getter)]
    pub fn contact_erp(&self) -> f32 {
        self.0.contact_softness.erp(self.0.dt)
    }

    #[wasm_bindgen(getter)]
    pub fn normalizedAllowedLinearError(&self) -> f32 {
        self.0.normalized_allowed_linear_error
    }

    #[wasm_bindgen(getter)]
    pub fn normalizedPredictionDistance(&self) -> f32 {
        self.0.normalized_prediction_distance
    }

    #[wasm_bindgen(getter)]
    pub fn numSolverIterations(&self) -> usize {
        self.0.num_solver_iterations
    }

    #[wasm_bindgen(getter)]
    pub fn numInternalPgsIterations(&self) -> usize {
        self.0.num_internal_pgs_iterations
    }

    #[wasm_bindgen(getter)]
    pub fn maxCcdSubsteps(&self) -> usize {
        self.0.max_ccd_substeps
    }

    #[wasm_bindgen(getter)]
    pub fn lengthUnit(&self) -> f32 {
        self.0.length_unit
    }

    #[wasm_bindgen(getter)]
    pub fn softBodiesResweepStrain(&self) -> f32 {
        self.0.soft_bodies.resweep_strain
    }

    #[wasm_bindgen(setter)]
    pub fn set_softBodiesResweepStrain(&mut self, value: f32) {
        self.0.soft_bodies.resweep_strain = value;
    }

    #[wasm_bindgen(getter)]
    pub fn softBodiesMaxExtraSubsteps(&self) -> usize {
        self.0.soft_bodies.max_extra_substeps
    }

    #[wasm_bindgen(setter)]
    pub fn set_softBodiesMaxExtraSubsteps(&mut self, value: usize) {
        self.0.soft_bodies.max_extra_substeps = value;
    }

    #[wasm_bindgen(getter)]
    pub fn softBodiesContactStiffening(&self) -> f32 {
        self.0.soft_bodies.contact_stiffening
    }

    #[wasm_bindgen(setter)]
    pub fn set_softBodiesContactStiffening(&mut self, value: f32) {
        self.0.soft_bodies.contact_stiffening = value;
    }

    #[wasm_bindgen(getter)]
    pub fn softBodiesRecovery(&self) -> RawSoftRecoverySettings {
        RawSoftRecoverySettings(self.0.soft_bodies.recovery)
    }

    #[wasm_bindgen(setter)]
    pub fn set_softBodiesRecovery(&mut self, value: &RawSoftRecoverySettings) {
        self.0.soft_bodies.recovery = value.0;
    }

    #[wasm_bindgen(getter)]
    pub fn softBodiesFemLinearTolerance(&self) -> f32 {
        self.0.soft_bodies.fem.linear_tolerance
    }

    #[wasm_bindgen(setter)]
    pub fn set_softBodiesFemLinearTolerance(&mut self, value: f32) {
        self.0.soft_bodies.fem.linear_tolerance = value;
    }

    #[wasm_bindgen(getter)]
    pub fn softBodiesFemMaxLinearIterations(&self) -> usize {
        self.0.soft_bodies.fem.max_linear_iterations
    }

    #[wasm_bindgen(setter)]
    pub fn set_softBodiesFemMaxLinearIterations(&mut self, value: usize) {
        self.0.soft_bodies.fem.max_linear_iterations = value;
    }

    #[wasm_bindgen(getter)]
    pub fn softBodiesFemMaxDenseDofs(&self) -> usize {
        self.0.soft_bodies.fem.max_dense_dofs
    }

    #[wasm_bindgen(setter)]
    pub fn set_softBodiesFemMaxDenseDofs(&mut self, value: usize) {
        self.0.soft_bodies.fem.max_dense_dofs = value;
    }

    #[wasm_bindgen(setter)]
    pub fn set_dt(&mut self, value: f32) {
        self.0.dt = value;
    }

    #[wasm_bindgen(setter)]
    pub fn set_contact_natural_frequency(&mut self, value: f32) {
        self.0.contact_softness.natural_frequency = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_normalizedAllowedLinearError(&mut self, value: f32) {
        self.0.normalized_allowed_linear_error = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_normalizedPredictionDistance(&mut self, value: f32) {
        self.0.normalized_prediction_distance = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_numSolverIterations(&mut self, value: usize) {
        self.0.num_solver_iterations = value;
    }
    #[wasm_bindgen(setter)]
    pub fn set_numInternalPgsIterations(&mut self, value: usize) {
        self.0.num_internal_pgs_iterations = value;
    }

    #[wasm_bindgen(setter)]
    pub fn set_maxCcdSubsteps(&mut self, value: usize) {
        self.0.max_ccd_substeps = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_lengthUnit(&mut self, value: f32) {
        self.0.length_unit = value
    }
}
