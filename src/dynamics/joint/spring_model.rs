use crate::math::Real;

/// The spring-like model used for constraints resolution.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum SpringModel {
    /// No equation is solved.
    Disabled,
    /// The solved spring-like equation is:
    /// `delta_velocity(t + dt) = stiffness / dt * (target_pos - pos(t)) + damping * (target_vel - vel(t))`
    ///
    /// Here the `stiffness` is the ratio of position error to be solved at each timestep (like
    /// a velocity-based ERP), and the `damping` is the ratio of velocity error to be solved at
    /// each timestep.
    VelocityBased,
    /// The solved spring-like equation is:
    /// `acceleration(t + dt) = stiffness * (target_pos - pos(t)) + damping * (target_vel - vel(t))`
    AccelerationBased,
    /// The solved spring-like equation is:
    /// `force(t + dt) = stiffness * (target_pos - pos(t + dt)) + damping * (target_vel - vel(t + dt))`
    ForceBased,
}

impl Default for SpringModel {
    fn default() -> Self {
        SpringModel::VelocityBased
    }
}

impl SpringModel {
    /// Combines the coefficients used for solving the spring equation.
    ///
    /// Returns the new coefficients (stiffness, damping, inv_lhs_scale, keep_inv_lhs)
    /// coefficients for the equivalent impulse-based equation. These new
    /// coefficients must be used in the following way:
    /// - `rhs = (stiffness * pos_err + damping * vel_err) / gamma`.
    /// - `new_inv_lhs = gamma * if keep_inv_lhs { inv_lhs } else { 1.0 }`.
    /// Note that the returned `gamma` will be zero if both `stiffness` and `damping` are zero.
    pub fn combine_coefficients(
        self,
        dt: Real,
        stiffness: Real,
        damping: Real,
    ) -> (Real, Real, Real, bool) {
        match self {
            SpringModel::VelocityBased => (stiffness * crate::utils::inv(dt), damping, 1.0, true),
            SpringModel::AccelerationBased => {
                let effective_stiffness = stiffness * dt;
                let effective_damping = damping * dt;
                // TODO: Using gamma behaves very badly for some reasons.
                // Maybe I got the formulation wrong, so let's keep it to 1.0 for now,
                // and get back to this later.
                // let gamma = effective_stiffness * dt + effective_damping;
                (effective_stiffness, effective_damping, 1.0, true)
            }
            SpringModel::ForceBased => {
                let effective_stiffness = stiffness * dt;
                let effective_damping = damping * dt;
                let gamma = effective_stiffness * dt + effective_damping;
                (effective_stiffness, effective_damping, gamma, false)
            }
            SpringModel::Disabled => return (0.0, 0.0, 0.0, false),
        }
    }
}
