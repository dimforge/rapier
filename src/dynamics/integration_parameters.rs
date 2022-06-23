use crate::math::Real;

/// Parameters for a time-step of the physics engine.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct IntegrationParameters {
    /// The timestep length (default: `1.0 / 60.0`)
    pub dt: Real,
    /// Minimum timestep size when using CCD with multiple substeps (default `1.0 / 60.0 / 100.0`)
    ///
    /// When CCD with multiple substeps is enabled, the timestep is subdivided
    /// into smaller pieces. This timestep subdivision won't generate timestep
    /// lengths smaller than `min_ccd_dt`.
    ///
    /// Setting this to a large value will reduce the opportunity to performing
    /// CCD substepping, resulting in potentially more time dropped by the
    /// motion-clamping mechanism. Setting this to an very small value may lead
    /// to numerical instabilities.
    pub min_ccd_dt: Real,

    /// 0-1: multiplier for how much of the constraint violation (e.g. contact penetration)
    /// will be compensated for during the velocity solve.
    /// (default `0.8`).
    pub erp: Real,
    /// 0-1: the damping ratio used by the springs for Baumgarte constraints stabilization.
    /// Lower values make the constraints more compliant (more "springy", allowing more visible penetrations
    /// before stabilization).
    /// (default `0.25`).
    pub damping_ratio: Real,

    /// 0-1: multiplier for how much of the joint violation
    /// will be compensated for during the velocity solve.
    /// (default `1.0`).
    pub joint_erp: Real,

    /// The fraction of critical damping applied to the joint for constraints regularization.
    /// (default `0.25`).
    pub joint_damping_ratio: Real,

    /// Amount of penetration the engine wont attempt to correct (default: `0.001m`).
    pub allowed_linear_error: Real,
    /// Maximum amount of penetration the solver will attempt to resolve in one timestep.
    pub max_penetration_correction: Real,
    /// The maximal distance separating two objects that will generate predictive contacts (default: `0.002`).
    pub prediction_distance: Real,
    /// Maximum number of iterations performed to solve non-penetration and joint constraints (default: `4`).
    pub max_velocity_iterations: usize,
    /// Maximum number of iterations performed to solve friction constraints (default: `8`).
    pub max_velocity_friction_iterations: usize,
    /// Maximum number of iterations performed to remove the energy introduced by penetration corrections  (default: `1`).
    pub max_stabilization_iterations: usize,
    /// If `false`, friction and non-penetration constraints will be solved in the same loop. Otherwise,
    /// non-penetration constraints are solved first, and friction constraints are solved after (default: `true`).
    pub interleave_restitution_and_friction_resolution: bool,
    /// Minimum number of dynamic bodies in each active island (default: `128`).
    pub min_island_size: usize,
    /// Maximum number of substeps performed by the  solver (default: `1`).
    pub max_ccd_substeps: usize,
}

impl IntegrationParameters {
    /// The inverse of the time-stepping length, i.e. the steps per seconds (Hz).
    ///
    /// This is zero if `self.dt` is zero.
    #[inline(always)]
    pub fn inv_dt(&self) -> Real {
        if self.dt == 0.0 {
            0.0
        } else {
            1.0 / self.dt
        }
    }

    /// Sets the time-stepping length.
    #[inline]
    #[deprecated = "You can just set the `IntegrationParams::dt` value directly"]
    pub fn set_dt(&mut self, dt: Real) {
        assert!(dt >= 0.0, "The time-stepping length cannot be negative.");
        self.dt = dt;
    }

    /// Sets the inverse time-stepping length (i.e. the frequency).
    ///
    /// This automatically recompute `self.dt`.
    #[inline]
    pub fn set_inv_dt(&mut self, inv_dt: Real) {
        if inv_dt == 0.0 {
            self.dt = 0.0
        } else {
            self.dt = 1.0 / inv_dt
        }
    }

    /// The ERP coefficient, multiplied by the inverse timestep length.
    pub fn erp_inv_dt(&self) -> Real {
        self.erp * self.inv_dt()
    }

    /// The joint ERP coefficient, multiplied by the inverse timestep length.
    pub fn joint_erp_inv_dt(&self) -> Real {
        self.joint_erp * self.inv_dt()
    }

    /// The CFM factor to be used in the constraints resolution.
    pub fn cfm_factor(&self) -> Real {
        // Compute CFM assuming a critically damped spring multiplied by the damping ratio.
        let inv_erp_minus_one = 1.0 / self.erp - 1.0;

        // let stiffness = 4.0 * damping_ratio * damping_ratio * projected_mass
        //     / (dt * dt * inv_erp_minus_one * inv_erp_minus_one);
        // let damping = 4.0 * damping_ratio * damping_ratio * projected_mass
        //     / (dt * inv_erp_minus_one);
        // let cfm = 1.0 / (dt * dt * stiffness + dt * damping);
        // NOTE: This simplies to cfm = cfm_coefff / projected_mass:
        let cfm_coeff = inv_erp_minus_one * inv_erp_minus_one
            / ((1.0 + inv_erp_minus_one) * 4.0 * self.damping_ratio * self.damping_ratio);

        // Furthermore, we use this coefficient inside of the impulse resolution.
        // Surprisingly, several simplifications happen there.
        // Let `m` the projected mass of the constraint.
        // Let `m’` the projected mass that includes CFM: `m’ = 1 / (1 / m + cfm_coeff / m) = m / (1 + cfm_coeff)`
        // We have:
        // new_impulse = old_impulse - m’ (delta_vel - cfm * old_impulse)
        //             = old_impulse - m / (1 + cfm_coeff) * (delta_vel - cfm_coeff / m * old_impulse)
        //             = old_impulse * (1 - cfm_coeff / (1 + cfm_coeff)) - m / (1 + cfm_coeff) * delta_vel
        //             = old_impulse / (1 + cfm_coeff) - m * delta_vel / (1 + cfm_coeff)
        //             = 1 / (1 + cfm_coeff) * (old_impulse - m * delta_vel)
        // So, setting cfm_factor = 1 / (1 + cfm_coeff).
        // We obtain:
        // new_impulse = cfm_factor * (old_impulse - m * delta_vel)
        //
        // The value returned by this function is this cfm_factor that can be used directly
        // in the constraints solver.
        1.0 / (1.0 + cfm_coeff)
    }

    /// The CFM (constraints force mixing) coefficient applied to all joints for constraints regularization
    pub fn joint_cfm_coeff(&self) -> Real {
        // Compute CFM assuming a critically damped spring multiplied by the damping ratio.
        let inv_erp_minus_one = 1.0 / self.joint_erp - 1.0;
        inv_erp_minus_one * inv_erp_minus_one
            / ((1.0 + inv_erp_minus_one)
                * 4.0
                * self.joint_damping_ratio
                * self.joint_damping_ratio)
    }
}

impl Default for IntegrationParameters {
    fn default() -> Self {
        Self {
            dt: 1.0 / 60.0,
            min_ccd_dt: 1.0 / 60.0 / 100.0,
            erp: 0.8,
            damping_ratio: 0.25,
            joint_erp: 1.0,
            joint_damping_ratio: 1.0,
            allowed_linear_error: 0.001, // 0.005
            max_penetration_correction: Real::MAX,
            prediction_distance: 0.002,
            max_velocity_iterations: 4,
            max_velocity_friction_iterations: 8,
            max_stabilization_iterations: 1,
            interleave_restitution_and_friction_resolution: true, // Enabling this makes a big difference for 2D stability.
            // TODO: what is the optimal value for min_island_size?
            // It should not be too big so that we don't end up with
            // huge islands that don't fit in cache.
            // However we don't want it to be too small and end up with
            // tons of islands, reducing SIMD parallelism opportunities.
            min_island_size: 128,
            max_ccd_substeps: 1,
        }
    }
}
