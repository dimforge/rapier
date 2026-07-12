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
    pub fn minIslandSize(&self) -> usize {
        self.0.min_island_size
    }

    #[wasm_bindgen(getter)]
    pub fn maxCcdSubsteps(&self) -> usize {
        self.0.max_ccd_substeps
    }

    #[wasm_bindgen(getter)]
    pub fn lengthUnit(&self) -> f32 {
        self.0.length_unit
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
    pub fn set_minIslandSize(&mut self, value: usize) {
        self.0.min_island_size = value
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
