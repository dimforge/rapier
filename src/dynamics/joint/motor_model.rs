use crate::math::Real;

/// The spring-like model used for constraints resolution.
#[derive(Default, Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum MotorModel {
    /// The solved spring-like equation is:
    /// `acceleration = stiffness * (pos - target_pos) + damping * (vel - target_vel)`
    #[default]
    AccelerationBased,
    /// The solved spring-like equation is:
    /// `force = stiffness * (pos - target_pos) + damping * (vel - target_vel)`
    ForceBased,
}

impl MotorModel {
    /// Combines the coefficients used for solving the spring equation.
    ///
    /// Returns the coefficients (erp_inv_dt, cfm_coeff, cfm_gain).
    pub fn combine_coefficients(
        self,
        dt: Real,
        stiffness: Real,
        damping: Real,
    ) -> (Real, Real, Real) {
        match self {
            MotorModel::AccelerationBased => {
                let erp_inv_dt = stiffness * crate::utils::inv(dt * stiffness + damping);
                let cfm_coeff = crate::utils::inv(dt * dt * stiffness + dt * damping);
                (erp_inv_dt, cfm_coeff, 0.0)
            }
            MotorModel::ForceBased => {
                let erp_inv_dt = stiffness * crate::utils::inv(dt * stiffness + damping);
                let cfm_gain = crate::utils::inv(dt * dt * stiffness + dt * damping);
                (erp_inv_dt, 0.0, cfm_gain)
            }
        }
    }
}
