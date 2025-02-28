use crate::math::Real;
use na::RealField;
use std::num::NonZeroUsize;

#[cfg(doc)]
use super::RigidBodyActivation;

// TODO: enabling the block solver in 3d introduces a lot of jitters in
//       the 3D domino demo. So for now we dont enable it in 3D.
pub(crate) static BLOCK_SOLVER_ENABLED: bool = cfg!(feature = "dim2");

/// Parameters for a time-step of the physics engine.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct IntegrationParameters {
    /// The timestep length (default: `1.0 / 60.0`).
    pub dt: Real,
    /// Minimum timestep size when using CCD with multiple substeps (default: `1.0 / 60.0 / 100.0`).
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

    /// > 0: the damping ratio used by the springs for contact constraint stabilization.
    ///
    /// Larger values make the constraints more compliant (allowing more visible
    /// penetrations before stabilization).
    /// (default `5.0`).
    pub contact_damping_ratio: Real,

    /// > 0: the natural frequency used by the springs for contact constraint regularization.
    ///
    /// Increasing this value will make it so that penetrations get fixed more quickly at the
    /// expense of potential jitter effects due to overshooting. In order to make the simulation
    /// look stiffer, it is recommended to increase the [`Self::contact_damping_ratio`] instead of this
    /// value.
    /// (default: `30.0`).
    pub contact_natural_frequency: Real,

    /// > 0: the natural frequency used by the springs for joint constraint regularization.
    ///
    /// Increasing this value will make it so that penetrations get fixed more quickly.
    /// (default: `1.0e6`).
    pub joint_natural_frequency: Real,

    /// The fraction of critical damping applied to the joint for constraints regularization.
    ///
    /// Larger values make the constraints more compliant (allowing more joint
    /// drift before stabilization).
    /// (default `1.0`).
    pub joint_damping_ratio: Real,

    /// The coefficient in `[0, 1]` applied to warmstart impulses, i.e., impulses that are used as the
    /// initial solution (instead of 0) at the next simulation step.
    ///
    /// This should generally be set to 1.
    ///
    /// (default `1.0`).
    pub warmstart_coefficient: Real,

    /// The approximate size of most dynamic objects in the scene.
    ///
    /// This value is used internally to estimate some length-based tolerance. In particular, the
    /// values [`IntegrationParameters::allowed_linear_error`],
    /// [`IntegrationParameters::max_corrective_velocity`],
    /// [`IntegrationParameters::prediction_distance`], [`RigidBodyActivation::normalized_linear_threshold`]
    /// are scaled by this value implicitly.
    ///
    /// This value can be understood as the number of units-per-meter in your physical world compared
    /// to a human-sized world in meter. For example, in a 2d game, if your typical object size is 100
    /// pixels, set the [`Self::length_unit`] parameter to 100.0. The physics engine will interpret
    /// it as if 100 pixels is equivalent to 1 meter in its various internal threshold.
    /// (default `1.0`).
    pub length_unit: Real,

    /// Amount of penetration the engine won’t attempt to correct (default: `0.001m`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_allowed_linear_error: Real,
    /// Maximum amount of penetration the solver will attempt to resolve in one timestep (default: `10.0`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_max_corrective_velocity: Real,
    /// The maximal distance separating two objects that will generate predictive contacts (default: `0.002m`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_prediction_distance: Real,
    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    pub num_solver_iterations: NonZeroUsize,
    /// Number of addition friction resolution iteration run during the last solver sub-step (default: `0`).
    pub num_additional_friction_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    /// The number of stabilization iterations run at each solver iterations (default: `2`).
    pub num_internal_stabilization_iterations: usize,
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

    /// The contact’s spring angular frequency for constraints regularization.
    pub fn contact_angular_frequency(&self) -> Real {
        self.contact_natural_frequency * Real::two_pi()
    }

    /// The [`Self::contact_erp`] coefficient, multiplied by the inverse timestep length.
    pub fn contact_erp_inv_dt(&self) -> Real {
        let ang_freq = self.contact_angular_frequency();
        ang_freq / (self.dt * ang_freq + 2.0 * self.contact_damping_ratio)
    }

    /// The effective Error Reduction Parameter applied for calculating regularization forces
    /// on contacts.
    ///
    /// This parameter is computed automatically from [`Self::contact_natural_frequency`],
    /// [`Self::contact_damping_ratio`] and the substep length.
    pub fn contact_erp(&self) -> Real {
        self.dt * self.contact_erp_inv_dt()
    }

    /// The joint’s spring angular frequency for constraint regularization.
    pub fn joint_angular_frequency(&self) -> Real {
        self.joint_natural_frequency * Real::two_pi()
    }

    /// The [`Self::joint_erp`] coefficient, multiplied by the inverse timestep length.
    pub fn joint_erp_inv_dt(&self) -> Real {
        let ang_freq = self.joint_angular_frequency();
        ang_freq / (self.dt * ang_freq + 2.0 * self.joint_damping_ratio)
    }

    /// The effective Error Reduction Parameter applied for calculating regularization forces
    /// on joints.
    ///
    /// This parameter is computed automatically from [`Self::joint_natural_frequency`],
    /// [`Self::joint_damping_ratio`] and the substep length.
    pub fn joint_erp(&self) -> Real {
        self.dt * self.joint_erp_inv_dt()
    }

    /// The CFM factor to be used in the constraint resolution.
    ///
    /// This parameter is computed automatically from [`Self::contact_natural_frequency`],
    /// [`Self::contact_damping_ratio`] and the substep length.
    pub fn contact_cfm_factor(&self) -> Real {
        // Compute CFM assuming a critically damped spring multiplied by the damping ratio.
        // The logic is similar to [`Self::joint_cfm_coeff`].
        let contact_erp = self.contact_erp();
        if contact_erp == 0.0 {
            return 0.0;
        }
        let inv_erp_minus_one = 1.0 / contact_erp - 1.0;

        // let stiffness = 4.0 * damping_ratio * damping_ratio * projected_mass
        //     / (dt * dt * inv_erp_minus_one * inv_erp_minus_one);
        // let damping = 4.0 * damping_ratio * damping_ratio * projected_mass
        //     / (dt * inv_erp_minus_one);
        // let cfm = 1.0 / (dt * dt * stiffness + dt * damping);
        // NOTE: This simplifies to cfm = cfm_coeff / projected_mass:
        let cfm_coeff = inv_erp_minus_one * inv_erp_minus_one
            / ((1.0 + inv_erp_minus_one)
                * 4.0
                * self.contact_damping_ratio
                * self.contact_damping_ratio);

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
        // in the constraint solver.
        1.0 / (1.0 + cfm_coeff)
    }

    /// The CFM (constraints force mixing) coefficient applied to all joints for constraints regularization.
    ///
    /// This parameter is computed automatically from [`Self::joint_natural_frequency`],
    /// [`Self::joint_damping_ratio`] and the substep length.
    pub fn joint_cfm_coeff(&self) -> Real {
        // Compute CFM assuming a critically damped spring multiplied by the damping ratio.
        // The logic is similar to `Self::contact_cfm_factor`.
        let joint_erp = self.joint_erp();
        if joint_erp == 0.0 {
            return 0.0;
        }
        let inv_erp_minus_one = 1.0 / joint_erp - 1.0;
        inv_erp_minus_one * inv_erp_minus_one
            / ((1.0 + inv_erp_minus_one)
                * 4.0
                * self.joint_damping_ratio
                * self.joint_damping_ratio)
    }

    /// Amount of penetration the engine won’t attempt to correct (default: `0.001` multiplied by
    /// [`Self::length_unit`]).
    pub fn allowed_linear_error(&self) -> Real {
        self.normalized_allowed_linear_error * self.length_unit
    }

    /// Maximum amount of penetration the solver will attempt to resolve in one timestep.
    ///
    /// This is equal to [`Self::normalized_max_corrective_velocity`] multiplied by
    /// [`Self::length_unit`].
    pub fn max_corrective_velocity(&self) -> Real {
        if self.normalized_max_corrective_velocity != Real::MAX {
            self.normalized_max_corrective_velocity * self.length_unit
        } else {
            Real::MAX
        }
    }

    /// The maximal distance separating two objects that will generate predictive contacts
    /// (default: `0.002m` multiped by [`Self::length_unit`]).
    pub fn prediction_distance(&self) -> Real {
        self.normalized_prediction_distance * self.length_unit
    }

    /// Initialize the simulation parameters with settings matching the TGS-soft solver
    /// with warmstarting.
    ///
    /// This is the default configuration, equivalent to [`IntegrationParameters::default()`].
    pub fn tgs_soft() -> Self {
        Self {
            dt: 1.0 / 60.0,
            min_ccd_dt: 1.0 / 60.0 / 100.0,
            contact_natural_frequency: 30.0,
            contact_damping_ratio: 5.0,
            joint_natural_frequency: 1.0e6,
            joint_damping_ratio: 1.0,
            warmstart_coefficient: 1.0,
            num_internal_pgs_iterations: 1,
            num_internal_stabilization_iterations: 2,
            num_additional_friction_iterations: 0,
            num_solver_iterations: NonZeroUsize::new(4).unwrap(),
            // TODO: what is the optimal value for min_island_size?
            // It should not be too big so that we don't end up with
            // huge islands that don't fit in cache.
            // However we don't want it to be too small and end up with
            // tons of islands, reducing SIMD parallelism opportunities.
            min_island_size: 128,
            normalized_allowed_linear_error: 0.001,
            normalized_max_corrective_velocity: 10.0,
            normalized_prediction_distance: 0.002,
            max_ccd_substeps: 1,
            length_unit: 1.0,
        }
    }

    /// Initialize the simulation parameters with settings matching the TGS-soft solver
    /// **without** warmstarting.
    ///
    /// The [`IntegrationParameters::tgs_soft()`] configuration should be preferred unless
    /// warmstarting proves to be undesirable for your use-case.
    pub fn tgs_soft_without_warmstart() -> Self {
        Self {
            contact_damping_ratio: 0.25,
            warmstart_coefficient: 0.0,
            num_additional_friction_iterations: 4,
            ..Self::tgs_soft()
        }
    }

    /// Initializes the integration parameters to match the legacy PGS solver from Rapier version <= 0.17.
    ///
    /// This exists mainly for testing and comparison purpose.
    pub fn pgs_legacy() -> Self {
        Self {
            num_solver_iterations: NonZeroUsize::new(1).unwrap(),
            num_internal_pgs_iterations: 4,
            ..Self::tgs_soft_without_warmstart()
        }
    }
}

impl Default for IntegrationParameters {
    fn default() -> Self {
        Self::tgs_soft()
    }
}
