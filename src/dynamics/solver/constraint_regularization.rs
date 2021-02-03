use crate::math::Real;
use na::RealField;

pub struct SpringRegularization {}

impl Default for SpringRegularization {
    fn default() -> Self {
        SpringRegularization {}
    }
}

impl SpringRegularization {
    pub fn erp_cfm_impulse_scale(&self, dt: Real) -> (Real, Real, Real) {
        let erp = 0.2 / dt;
        let cfm = 1.0e-5 / dt;
        let impulse_scale = 0.9;

        (erp, cfm, impulse_scale)
    }
}
