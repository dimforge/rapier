#[cfg(all(doc, feature = "alloc"))]
use super::RigidBodyActivation;
use crate::math::Real;
use simba::simd::SimdRealField;

// NOTE: the 2x2 block solver (`block-solver` feature) runs in BOTH passes and solves the COMPLIANT
//       LCP `(K + C, b)`, `C = diag((1/ms_i - 1) k_ii)` — the coupled soft step — so it shares
//       the sequential sweep's fixed points; a rigid-LCP-then-cfm-scale variant left piles micro-jiggling.

/// Friction models used for all contact constraints between two rigid-bodies.
///
/// This selection does not apply to multibodies that always rely on the [`FrictionModel::Coulomb`].
#[cfg(feature = "dim3")]
#[derive(Default, Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum FrictionModel {
    /// A simplified friction model significantly faster to solve than [`Self::Coulomb`]
    /// but less accurate.
    ///
    /// Instead of solving one Coulomb friction constraint per contact in a contact manifold,
    /// this approximation only solves one Coulomb friction constraint per group of 4 contacts
    /// in a contact manifold, plus one "twist" constraint. The "twist" constraint is purely
    /// rotational and aims to eliminate angular movement in the manifold’s tangent plane.
    #[default]
    Simplified,
    /// The coulomb friction model.
    ///
    /// This results in one Coulomb friction constraint per contact point.
    Coulomb,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
// TODO: we should be able to combine this with MotorModel.
/// Coefficients for a spring, typically used for configuring constraint softness for contacts and
/// joints.
pub struct SpringCoefficients<N> {
    /// Sets the natural frequency (Hz) of the spring-like constraint.
    ///
    /// Higher values make the constraint stiffer and resolve constraint violations more quickly.
    pub natural_frequency: N,
    /// Sets the damping ratio for the spring-like constraint.
    ///
    /// Larger values make the joint more compliant (allowing more drift before stabilization).
    pub damping_ratio: N,
}

impl<N: SimdRealField<Element = Real> + Copy> SpringCoefficients<N> {
    /// Initializes spring coefficients from the spring natural frequency and damping ratio.
    pub fn new(natural_frequency: N, damping_ratio: N) -> Self {
        Self {
            natural_frequency,
            damping_ratio,
        }
    }

    /// Default softness coefficients for contacts (30 Hz, ζ = 10).
    /// The high ζ is load-bearing for large piles/stacks: softer contacts settle
    /// deeper under load, and the extra penetration keeps them wedging and creeping instead of resting.
    pub fn contact_defaults() -> Self {
        Self {
            natural_frequency: N::splat(30.0),
            damping_ratio: N::splat(10.0),
        }
    }

    /// Default softness coefficients for contacts touching a fixed body: twice the natural
    /// frequency of [`Self::contact_defaults`], holding piled/pushed
    /// bodies more firmly against walls and floors so they are less likely to squeeze through.
    pub fn contact_static_defaults() -> Self {
        Self {
            natural_frequency: N::splat(60.0),
            damping_ratio: N::splat(10.0),
        }
    }

    /// Default softness coefficients for joints.
    pub fn joint_defaults() -> Self {
        Self {
            natural_frequency: N::splat(1.0e6),
            damping_ratio: N::splat(1.0),
        }
    }

    /// The contact’s spring angular frequency for constraints regularization.
    pub fn angular_frequency(&self) -> N {
        self.natural_frequency * N::simd_two_pi()
    }

    /// The [`Self::erp`] coefficient, multiplied by the inverse timestep length.
    pub fn erp_inv_dt(&self, dt: N) -> N {
        let ang_freq = self.angular_frequency();
        ang_freq / (dt * ang_freq + N::splat(2.0) * self.damping_ratio)
    }

    /// The effective Error Reduction Parameter applied for calculating regularization forces.
    ///
    /// This parameter is computed automatically from [`Self::natural_frequency`],
    /// [`Self::damping_ratio`] and the substep length.
    pub fn erp(&self, dt: N) -> N {
        dt * self.erp_inv_dt(dt)
    }

    /// Compute CFM assuming a critically damped spring multiplied by the damping ratio.
    ///
    /// This coefficient softens the impulse applied at each solver iteration.
    pub fn cfm_coeff(&self, dt: N) -> N {
        let one = N::one();
        let erp = self.erp(dt);
        let erp_is_not_zero = erp.simd_ne(N::zero());
        let inv_erp_minus_one = one / erp - one;

        // let stiffness = 4.0 * damping_ratio * damping_ratio * projected_mass
        //     / (dt * dt * inv_erp_minus_one * inv_erp_minus_one);
        // let damping = 4.0 * damping_ratio * damping_ratio * projected_mass
        //     / (dt * inv_erp_minus_one);
        // let cfm = 1.0 / (dt * dt * stiffness + dt * damping);
        // NOTE: This simplifies to cfm = cfm_coeff / projected_mass:
        let result = inv_erp_minus_one * inv_erp_minus_one
            / ((one + inv_erp_minus_one) * N::splat(4.0) * self.damping_ratio * self.damping_ratio);
        result.select(erp_is_not_zero, N::zero())
    }

    /// The CFM factor to be used in the constraint resolution.
    ///
    /// This parameter is computed automatically from [`Self::natural_frequency`],
    /// [`Self::damping_ratio`] and the substep length.
    pub fn cfm_factor(&self, dt: N) -> N {
        let one = N::one();
        let cfm_coeff = self.cfm_coeff(dt);

        // We use this coefficient inside the impulse resolution.
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
        one / (one + cfm_coeff)
    }
}

/// Configuration parameters that control the physics simulation quality and behavior.
///
/// These parameters affect how the physics engine advances time, resolves collisions, and
/// maintains stability. The defaults work well for most games, but you may want to adjust
/// them based on your specific needs.
///
/// # Key parameters for beginners
///
/// - **`dt`**: Timestep duration (default: 1/60 second). Most games run physics at 60Hz.
/// - **`num_solver_iterations`**: More iterations = more accurate but slower (default: 4)
/// - **`length_unit`**: Scale factor if your world units aren't meters (e.g., 100 for pixel-based games)
///
/// # Example
///
/// ```
/// # use rapier3d::prelude::*;
/// // Standard 60 FPS physics with default settings
/// let mut integration_params = IntegrationParameters::default();
///
/// // For a more accurate (but slower) simulation:
/// integration_params.num_solver_iterations = 8;
///
/// // For pixel-based 2D games where 100 pixels = 1 meter:
/// integration_params.length_unit = 100.0;
/// ```
///
/// Most other parameters are advanced settings for fine-tuning stability and performance.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct IntegrationParameters {
    /// The timestep length - how much simulated time passes per physics step (default: `1.0 / 60.0`).
    ///
    /// Set this to `1.0 / your_target_fps`. For example:
    /// - 60 FPS: `1.0 / 60.0` ≈ 0.0167 seconds
    /// - 120 FPS: `1.0 / 120.0` ≈ 0.0083 seconds
    ///
    /// Smaller timesteps are more accurate but require more CPU time per second of simulated time.
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

    /// Softness coefficients for contact constraints.
    pub contact_softness: SpringCoefficients<Real>,

    /// Softness coefficients for contact constraints where one side is a fixed body.
    ///
    /// Stiffer than [`Self::contact_softness`] by default so bodies are
    /// held firmly against static walls/floors; set equal to [`Self::contact_softness`] to disable.
    pub static_contact_softness: SpringCoefficients<Real>,

    /// The coefficient in `[0, 1]` applied to warmstart impulses, i.e., impulses that are used as the
    /// initial solution (instead of 0) at the next simulation step.
    ///
    /// This should generally be set to 1.
    ///
    /// (default `1.0`).
    pub warmstart_coefficient: Real,

    /// The scale factor for your world if you're not using meters (default: `1.0`).
    ///
    /// Rapier is tuned for human-scale objects measured in meters. If your game uses different
    /// units, set this to how many of your units equal 1 meter in the real world.
    ///
    /// **Examples:**
    /// - Your game uses meters: `length_unit = 1.0` (default)
    /// - Your game uses centimeters: `length_unit = 100.0` (100 cm = 1 m)
    /// - Pixel-based 2D game where typical objects are 100 pixels tall: `length_unit = 100.0`
    /// - Your game uses feet: `length_unit = 3.28` (approximately)
    ///
    /// This automatically scales various internal tolerances and thresholds to work correctly
    /// with your chosen units.
    pub length_unit: Real,

    /// Geometric slop distance (default: `0.005`), e.g. the standoff kept
    /// by the CCD clamp. NOT a deadzone on the position-correction bias: penetrations are corrected
    /// all the way to zero; a deadzone would keep loaded piles wedging and creeping.
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_allowed_linear_error: Real,
    /// Maximum speed at which contact penetration is pushed out by the biased solve
    /// (default: `3.0`).
    ///
    /// Capping this recovery velocity keeps deep penetrations from being resolved explosively.
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_max_corrective_velocity: Real,
    /// The maximal distance separating two objects that will generate predictive contacts (default: `0.002m`).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_prediction_distance: Real,
    /// Maximum linear velocity a body may have after each solver substep (default: `400.0` m/s).
    /// Bounding per-step travel keeps CCD and speculative contacts
    /// reliable (a body cannot be flung or crushed to an arbitrary speed); set to `Real::MAX` to disable.
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_max_linear_velocity: Real,
    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    ///
    /// Higher values produce more accurate and stable simulations at the cost of performance.
    /// - `4` (default): Good balance for most games
    /// - `8-12`: Use for demanding scenarios (stacks of objects, complex machinery)
    /// - `1-2`: Use if performance is critical and accuracy can be sacrificed
    pub num_solver_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    /// The number of stabilization iterations run at each solver iterations (default: `1`).
    pub num_internal_stabilization_iterations: usize,
    /// Maximum number of CCD substeps performed by the solver (default: `1`).
    ///
    /// Also the global CCD on/off switch: `0` disables **all** CCD for the world (including the
    /// automatic CCD of fast dynamic bodies vs fixed colliders).
    pub max_ccd_substeps: usize,
    /// If enabled, contact manifolds of a collider pair sharing (nearly) the same normal are merged
    /// into one "cluster" manifold before constraint generation (default: `true`, 3D only), so at
    /// most 4 contact points are solved per contact plane — a large solver win on composite shapes
    /// (meshes, heightfields, compounds, voxels) that emit one manifold per subshape. When clustering
    /// applies, read solver contacts/impulses from [`crate::geometry::ContactPair::solver_clusters`],
    /// not [`crate::geometry::ContactPair::manifolds`].
    pub contact_clustering: bool,
    /// If enabled, a contact pair whose relative pose moved less than [`Self::contact_recycle_distance`]
    /// since its last full narrow-phase update skips contact determination and keeps its existing points
    /// (default: `true`) — a large speed-up for quasi-static scenes. Trade-offs:
    /// contact features and user-facing contact data (`dist`, is-new bits) may be stale by up to that
    /// distance, and per-step joint-based contact filtering is skipped until the pair moves.
    /// [`crate::pipeline::ActiveHooks`] pairs are never recycled.
    pub contact_recycling: bool,
    /// Maximum relative-pose drift (translation plus rotation-arc) below which a contact pair may
    /// be recycled instead of fully updated (default: `0.05`, i.e. ten times the linear slop,
    /// multiplied by [`Self::length_unit`]). Only used when [`Self::contact_recycling`] is enabled.
    pub normalized_contact_recycle_distance: Real,
    /// If `false`, friction is only solved during the unbiased (relax) pass of each substep instead
    /// of both passes (default: `false`, the "no friction when applying bias" rule).
    /// This makes contact kernels much cheaper and is load-bearing for tall stacks: friction
    /// reacting to bias velocities pumps their coherent lean mode until they topple. If
    /// [`Self::num_internal_stabilization_iterations`] is zero there is no unbiased pass and this flag is ignored.
    pub friction_in_bias_pass: bool,
    /// If enabled, impulse-joint constraints are warm-started like contacts: impulses accumulated
    /// by the previous step are re-applied (scaled by [`Self::warmstart_coefficient`]) at the start
    /// of each substep instead of restarting from zero (default: `false`). This
    /// noticeably improves convergence of stiff joint assemblies. Multibody joints are unaffected.
    pub warmstart_joints: bool,
    /// The type of friction constraints used in the simulation.
    #[cfg(feature = "dim3")]
    pub friction_model: FrictionModel,
}

impl IntegrationParameters {
    /// The inverse of the time-stepping length, i.e. the steps per seconds (Hz).
    ///
    /// This is zero if `self.dt` is zero.
    #[inline]
    pub fn inv_dt(&self) -> Real {
        if self.dt == 0.0 { 0.0 } else { 1.0 / self.dt }
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

    /// Amount of penetration the engine won't attempt to correct (default: `0.001` multiplied by
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

    /// Maximum linear velocity a body may have after each solver substep.
    ///
    /// This is equal to [`Self::normalized_max_linear_velocity`] multiplied by
    /// [`Self::length_unit`], or `Real::MAX` when the linear speed cap is disabled.
    pub fn max_linear_velocity(&self) -> Real {
        if self.normalized_max_linear_velocity != Real::MAX {
            self.normalized_max_linear_velocity * self.length_unit
        } else {
            Real::MAX
        }
    }

    /// Maximum relative-pose drift below which a contact pair can be recycled instead of fully
    /// updated: [`Self::normalized_contact_recycle_distance`] multiplied by [`Self::length_unit`].
    /// Only used when [`Self::contact_recycling`] is enabled.
    pub fn contact_recycle_distance(&self) -> Real {
        self.normalized_contact_recycle_distance * self.length_unit
    }
}

impl Default for IntegrationParameters {
    fn default() -> Self {
        Self {
            dt: 1.0 / 60.0,
            min_ccd_dt: 1.0 / 60.0 / 100.0,
            contact_softness: SpringCoefficients::contact_defaults(),
            static_contact_softness: SpringCoefficients::contact_static_defaults(),
            warmstart_coefficient: 1.0,
            num_internal_pgs_iterations: 1,
            num_internal_stabilization_iterations: 1,
            num_solver_iterations: 4,
            normalized_allowed_linear_error: 0.005,
            normalized_max_corrective_velocity: 3.0,
            // Four times the linear slop. A larger speculative
            // margin generates contacts earlier, which (together with oriented/one-sided static
            // geometry) keeps fast/piled bodies from tunneling through thin walls.
            normalized_prediction_distance: 0.02,
            normalized_max_linear_velocity: 400.0,
            max_ccd_substeps: 1,
            contact_clustering: true,
            contact_recycling: true,
            normalized_contact_recycle_distance: 0.05,
            friction_in_bias_pass: false,
            warmstart_joints: false,
            length_unit: 1.0,
            #[cfg(feature = "dim3")]
            friction_model: FrictionModel::default(),
        }
    }
}
