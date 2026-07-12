use rapier::dynamics::CCDSolver;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawCCDSolver(pub(crate) CCDSolver);

#[wasm_bindgen]
impl RawCCDSolver {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawCCDSolver(CCDSolver::new())
    }
}
