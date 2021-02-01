use crate::math::Real;
use na::RealField;

pub struct SpringRegularization {
    pub angular_frequency: Real,
    pub damping: Real,
}

impl Default for SpringRegularization {
    fn default() -> Self {
        SpringRegularization {
            angular_frequency: 30.0 * Real::two_pi(),
            damping: 1.0,
        }
    }
}

impl SpringRegularization {
    pub fn erp_cfm_impulse_scale(&self, dt: Real) -> (Real, Real, Real) {
        let freq_dt = self.angular_frequency * dt;
        let erp = self.angular_frequency / (freq_dt + self.damping * 2.0);
        let extra = 1.0 / (freq_dt * (freq_dt + self.damping * 2.0));
        let cfm = 1.0 / (1.0 + extra);
        let impulse_scale = extra * cfm;

        let kd = 1.0;
        let kp = 10.0;
        let erp = 0.2 / dt; // kp / (dt * kp + kd);
        let cfm = 1.0e-5 / dt; // 1.0 / (dt * kp + kd);
        let impulse_scale = 0.9;

        (erp, cfm, impulse_scale)
    }
}
