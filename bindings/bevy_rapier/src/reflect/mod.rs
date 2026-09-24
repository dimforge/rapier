use crate::math::Real;
use bevy::reflect::reflect_remote;
#[cfg(feature = "fem")]
use rapier::dynamics::SoftFemParameters;
use rapier::dynamics::{
    SoftBodiesSettings, SoftBodyMaterial as RapierSoftBodyMaterial, SoftEdgePlasticFlow,
    SoftPatchConstraints, SoftRecoverySettings,
};
use rapier::{dynamics::IntegrationParameters, prelude::SpringCoefficients};

#[cfg(feature = "dim3")]
use rapier::dynamics::FrictionModel;

/// Friction models used for all contact constraints between two rigid-bodies.
///
/// This selection does not apply to multibodies that always rely on the [`FrictionModel::Coulomb`].
#[cfg(feature = "dim3")]
#[reflect_remote(FrictionModel)]
#[derive(Default, Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum FrictionModelWrapper {
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

#[reflect_remote(SpringCoefficients<Real>)]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Coefficients for a spring, typically used for configuring constraint softness for contacts and
/// joints.
pub struct SpringCoefficientsWrapper {
    /// Sets the natural frequency (Hz) of the spring-like constraint.
    ///
    /// Higher values make the constraint stiffer and resolve constraint violations more quickly.
    pub natural_frequency: Real,
    /// Sets the damping ratio for the spring-like constraint.
    ///
    /// Larger values make the joint more compliant (allowing more drift before stabilization).
    pub damping_ratio: Real,
}

#[reflect_remote(SoftPatchConstraints)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// What the per-point constraints of the features a soft-body volume constraint acts on do.
pub enum SoftPatchConstraintsWrapper {
    /// Keep them as they are (they may fight the volume constraint).
    Keep,
    /// Stand them down: the volume constraint alone acts on those features.
    StandDown,
    /// Keep them but along the volume constraint's normal, with their separation measured along
    /// it, so they push the way the constraint does.
    AlongNormal,
}

#[reflect_remote(SoftRecoverySettings)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Runtime configuration of the soft-body tangle detection and recovery stack.
///
/// See [`SoftRecoverySettings`] for the details of each field.
pub struct SoftRecoverySettingsWrapper {
    /// Raise the speculative contact margin from velocities authored between steps (default:
    /// `true`).
    pub authored_velocity_margin: bool,
    /// Give the edge-vs-edge pass a speculative reach (default: `false`).
    pub edge_speculation: bool,
    /// Detect inverted cells each step (default: `true`).
    pub inverted_cell_detection: bool,
    /// Detect surface self-crossings each step (default: `true`).
    pub self_crossing_detection: bool,
    /// Skip the self-crossing sweep while the surface barely moved (default: `true`).
    pub detection_motion_gating: bool,
    /// Detect boundary crossings between pairs of soft surfaces (default: `true`).
    pub cross_body_detection: bool,
    /// Self contacts of tangled features stand down (default: `true`).
    pub self_stand_down: bool,
    /// A vertex constraint touching a boundary crossing between two surfaces may only expel
    /// (default: `true`).
    pub cross_body_expel_gate: bool,
    /// Edge constraints touching a cross-body boundary crossing stand down (default: `true`).
    pub edge_stand_down: bool,
    /// Constraints on crossing-flagged features repel instead of standing down (default:
    /// `false`).
    pub crossing_repulsion: bool,
    /// Guide the crossing repulsion by the pair's volume normal (default: `false`).
    pub crossing_repulsion_guide: bool,
    /// Guide the self-crossing repulsion by the fold's volume normal (default: `false`).
    pub crossing_repulsion_self_guide: bool,
    /// Material recovery pace, in length units per second (default: `0.5`).
    pub recovery_pace: Real,
    /// Intersection-volume contact constraints for closed surfaces (default: `true`).
    pub overlap_constraints: bool,
    /// Overlap constraints against rigid colliders too (default: `true`).
    pub overlap_rigid: bool,
    /// A self-crossed mesh takes no pair constraint (default: `true`).
    pub overlap_skip_self_tangled: bool,
    /// The 3D closed-closed edge constraints stand down on a pair an overlap constraint owns
    /// (default: `true`).
    pub overlap_edge_stand_down: bool,
    /// Bound on the velocity change the coupled constraint may hand any side per step, in
    /// multiples of the recovery pace (default: `1.0`).
    pub overlap_constraint_pace: Real,
    /// What the per-point constraints of the features inside a volume constraint's patch do
    /// (default: `AlongNormal`).
    #[reflect(remote = SoftPatchConstraintsWrapper)]
    pub overlap_patch_constraints: SoftPatchConstraints,
    /// Measure the intersection volume on the contact skins (default: `false`).
    pub overlap_skin_volume: bool,
    /// The skin overlap kept at rest, as a fraction of the pair's skins (default: `0.0`).
    pub overlap_kept_depth: Real,
    /// Volume constraints on a body's self-overlaps between distinct surface regions (default:
    /// `false`).
    pub overlap_self_regions: bool,
    /// Push along each constraint's normal instead of the volume gradients (default: `true`).
    pub overlap_normal_push: bool,
    /// Split each pair's patch into a grid of cells with their own constraint (default:
    /// `false`).
    pub overlap_multi_volume: bool,
    /// Cells per tangent axis of the multi-volume grid (default: `3`).
    pub overlap_split: u32,
    /// Steps without progress of a pair's volume estimate before its positional correction
    /// stands down (default: `240`).
    pub overlap_patience: u32,
    /// Relative drop of the estimate that counts as progress for the patience (default:
    /// `0.02`).
    pub overlap_progress_margin: Real,
}

#[cfg(feature = "fem")]
#[reflect_remote(SoftFemParameters)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Tuning of the FEM soft-body solver.
pub struct SoftFemParametersWrapper {
    /// Relative residual at which the conjugate gradient stops (default: `1.0e-5`).
    pub linear_tolerance: Real,
    /// Hard cap on the conjugate-gradient iterations (default: `20`).
    pub max_linear_iterations: usize,
    /// Largest number of degrees of freedom for which the step-start matrix of a body is
    /// factorized directly (default: `600`).
    pub max_dense_dofs: usize,
}

// Duplicated because `reflect_remote` doesn't support conditional fields.
#[cfg(not(feature = "fem"))]
#[reflect_remote(SoftBodiesSettings)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Simulation settings shared by every soft body of a Rapier context.
pub struct SoftBodiesSettingsWrapper {
    /// Runtime toggles and tuning for the soft-body tangle detection and recovery stack.
    #[reflect(remote = SoftRecoverySettingsWrapper)]
    pub recovery: SoftRecoverySettings,
    /// Strain beyond which a soft-body constraint is re-solved after the contacts inside every
    /// substep (default: `0.75`).
    pub resweep_strain: Real,
    /// Maximum number of extra substeps a soft body requests for its island while it is hit fast
    /// (default: `4`).
    pub max_extra_substeps: usize,
    /// Factor applied to the contact softness natural frequencies for the soft-body contacts
    /// (default: `4.0`).
    pub contact_stiffening: Real,
}

#[cfg(feature = "fem")]
#[reflect_remote(SoftBodiesSettings)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Simulation settings shared by every soft body of a Rapier context.
pub struct SoftBodiesSettingsWrapper {
    /// Runtime toggles and tuning for the soft-body tangle detection and recovery stack.
    #[reflect(remote = SoftRecoverySettingsWrapper)]
    pub recovery: SoftRecoverySettings,
    /// Strain beyond which a soft-body constraint is re-solved after the contacts inside every
    /// substep (default: `0.75`).
    pub resweep_strain: Real,
    /// Maximum number of extra substeps a soft body requests for its island while it is hit fast
    /// (default: `4`).
    pub max_extra_substeps: usize,
    /// Factor applied to the contact softness natural frequencies for the soft-body contacts
    /// (default: `4.0`).
    pub contact_stiffening: Real,
    /// Tuning of the FEM soft-body solver.
    #[reflect(remote = SoftFemParametersWrapper)]
    pub fem: SoftFemParameters,
}

#[reflect_remote(SoftEdgePlasticFlow)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
/// Which strains make a soft-body edge flow plastically.
pub enum SoftEdgePlasticFlowWrapper {
    /// Both a squeeze and a stretch past the yield take a permanent set.
    #[default]
    Both,
    /// Only a squeeze sets.
    Compression,
    /// Only a stretch sets.
    Tension,
}

#[reflect_remote(RapierSoftBodyMaterial)]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
/// The stiffness, damping, plasticity and tearing parameters of a soft body.
///
/// See [`RapierSoftBodyMaterial`] for the details of each field.
pub struct SoftBodyMaterialWrapper {
    /// Softness of the structural edges.
    #[reflect(remote = SpringCoefficientsWrapper)]
    pub edge_softness: SpringCoefficients<Real>,
    /// Softness of the bending edges and dihedral constraints.
    #[reflect(remote = SpringCoefficientsWrapper)]
    pub bend_softness: SpringCoefficients<Real>,
    /// Softness of the per-cell volume constraints and of the global volume preservation.
    #[reflect(remote = SpringCoefficientsWrapper)]
    pub volume_softness: SpringCoefficients<Real>,
    /// Softness of the shape-matching constraints.
    #[reflect(remote = SpringCoefficientsWrapper)]
    pub shape_matching_softness: SpringCoefficients<Real>,
    /// Young's modulus of the elastic cells.
    pub young_modulus: Real,
    /// Poisson's ratio of the elastic cells, in `[0, 0.5)`.
    pub poisson_ratio: Real,
    /// Damping ratio of the elastic cells.
    pub elastic_damping_ratio: Real,
    /// Plastic yield of the elastic cells (`0.0`: no plasticity).
    pub plastic_yield: Real,
    /// Rate (per second) at which the strain in excess of the yield is absorbed into the rest
    /// shape.
    pub plastic_creep: Real,
    /// Largest accumulated plastic deformation of a cell.
    pub plastic_max: Real,
    /// Rate (per second) at which the particles' velocities are pulled toward the body's best-fit
    /// rigid motion.
    pub deformation_damping: Real,
    /// Plastic yield of the edges (`0.0`: none).
    pub edge_plastic_yield: Real,
    /// Rate (per second) at which an edge's strain in excess of its yield is absorbed into its
    /// rest length.
    pub edge_plastic_creep: Real,
    /// Largest permanent set an edge may accumulate, as a fraction of its initial length.
    pub edge_plastic_max: Real,
    /// Whether the edges take a permanent set under a squeeze, a stretch, or both.
    #[reflect(remote = SoftEdgePlasticFlowWrapper)]
    pub edge_plastic_flow: SoftEdgePlasticFlow,
    /// Strain beyond which elements break (`None`: unbreakable).
    pub tear_strain: Option<Real>,
    /// Force beyond which the edges break (`None`: unbreakable).
    pub tear_force: Option<Real>,
    /// Time constant, in seconds, over which an element's load is smoothed before the tear test.
    pub tear_smoothing: Real,
    /// How much tougher an undamaged interior element is than a surface one.
    pub interior_strength: Real,
    /// Maximum number of edges torn per step.
    pub max_tears_per_step: u32,
    /// The smallest piece, in measure elements, a tear may split off.
    pub min_piece: Option<u32>,
}

#[cfg(not(feature = "dim3"))]
#[reflect_remote(IntegrationParameters)]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Parameters for a time-step of the physics engine.
pub struct IntegrationParametersWrapper {
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

    /// Softness coefficients for contact constraints.
    #[reflect(remote = SpringCoefficientsWrapper)]
    pub contact_softness: SpringCoefficients<Real>,

    /// Softness coefficients for contact constraints where one side is a fixed body.
    ///
    /// Stiffer than [`IntegrationParameters::contact_softness`] by default so bodies are
    /// held firmly against static walls/floors; set equal to
    /// [`IntegrationParameters::contact_softness`] to disable.
    #[reflect(remote = SpringCoefficientsWrapper)]
    pub static_contact_softness: SpringCoefficients<Real>,

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
    /// Maximum linear velocity a body may have after each solver substep (default: `400.0` m/s).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_max_linear_velocity: Real,
    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    pub num_solver_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    /// The number of stabilization iterations run at each solver iterations (default: `1`).
    pub num_internal_stabilization_iterations: usize,
    /// Maximum number of substeps performed by the  solver (default: `1`).
    pub max_ccd_substeps: usize,
    /// If enabled, contact manifolds of a collider pair sharing (nearly) the same normal are
    /// merged into one "cluster" manifold before constraint generation (default: `true`, 3D only).
    pub contact_clustering: bool,
    /// If enabled, a contact pair that barely moved since its last full narrow-phase update
    /// skips contact determination and keeps its existing contact points (default: `true`).
    pub contact_recycling: bool,
    /// Maximum relative-pose drift below which a contact pair may be recycled instead of fully
    /// updated (default: `0.05`). Only used when contact recycling is enabled.
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_contact_recycle_distance: Real,
    /// If `false`, friction is only solved during the unbiased (relax) pass of each substep
    /// instead of both passes (default: `false`).
    pub friction_in_bias_pass: bool,
    /// If enabled, impulse-joint constraints are warm-started like contacts (default: `false`).
    pub warmstart_joints: bool,
    /// Simulation settings shared by every soft body.
    #[reflect(remote = SoftBodiesSettingsWrapper)]
    pub soft_bodies: rapier::dynamics::SoftBodiesSettings,
}

// These structs are duplicated in their entirety due to [`FrictionModel`] not being available in 2D, and `bevy::reflect_remote` not supporting conditional fields.
#[cfg(feature = "dim3")]
#[reflect_remote(IntegrationParameters)]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Parameters for a time-step of the physics engine.
pub struct IntegrationParametersWrapper {
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

    /// Softness coefficients for contact constraints.
    #[reflect(remote = SpringCoefficientsWrapper)]
    pub contact_softness: SpringCoefficients<Real>,

    /// Softness coefficients for contact constraints where one side is a fixed body.
    ///
    /// Stiffer than [`IntegrationParameters::contact_softness`] by default so bodies are
    /// held firmly against static walls/floors; set equal to
    /// [`IntegrationParameters::contact_softness`] to disable.
    #[reflect(remote = SpringCoefficientsWrapper)]
    pub static_contact_softness: SpringCoefficients<Real>,

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
    /// Maximum linear velocity a body may have after each solver substep (default: `400.0` m/s).
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_max_linear_velocity: Real,
    /// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
    pub num_solver_iterations: usize,
    /// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
    pub num_internal_pgs_iterations: usize,
    /// The number of stabilization iterations run at each solver iterations (default: `1`).
    pub num_internal_stabilization_iterations: usize,
    /// Maximum number of substeps performed by the  solver (default: `1`).
    pub max_ccd_substeps: usize,
    /// If enabled, contact manifolds of a collider pair sharing (nearly) the same normal are
    /// merged into one "cluster" manifold before constraint generation (default: `true`, 3D only).
    pub contact_clustering: bool,
    /// If enabled, a contact pair that barely moved since its last full narrow-phase update
    /// skips contact determination and keeps its existing contact points (default: `true`).
    pub contact_recycling: bool,
    /// Maximum relative-pose drift below which a contact pair may be recycled instead of fully
    /// updated (default: `0.05`). Only used when contact recycling is enabled.
    ///
    /// This value is implicitly scaled by [`IntegrationParameters::length_unit`].
    pub normalized_contact_recycle_distance: Real,
    /// If `false`, friction is only solved during the unbiased (relax) pass of each substep
    /// instead of both passes (default: `false`).
    pub friction_in_bias_pass: bool,
    /// If enabled, impulse-joint constraints are warm-started like contacts (default: `false`).
    pub warmstart_joints: bool,
    /// Simulation settings shared by every soft body.
    #[reflect(remote = SoftBodiesSettingsWrapper)]
    pub soft_bodies: rapier::dynamics::SoftBodiesSettings,
    /// Friction models used for all contact constraints between two rigid-bodies.
    #[reflect(remote = FrictionModelWrapper)]
    pub friction_model: FrictionModel,
}

#[cfg(feature = "dim2")]
#[reflect_remote(rapier::dynamics::JointAxis)]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Identifiers of degrees of freedoms of a joint.
pub enum JointAxisWrapper {
    /// The linear (translational) degree of freedom along the joint’s local X axis.
    LinX = 0,
    /// The linear (translational) degree of freedom along the joint’s local Y axis.
    LinY,
    /// The rotational degree of freedom of the joint.
    AngX,
}

#[cfg(feature = "dim3")]
#[reflect_remote(rapier::dynamics::JointAxis)]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Identifiers of degrees of freedoms of a joint.
pub enum JointAxisWrapper {
    /// The linear (translational) degree of freedom along the joint’s local X axis.
    LinX = 0,
    /// The linear (translational) degree of freedom along the joint’s local Y axis.
    LinY,
    /// The linear (translational) degree of freedom along the joint’s local Z axis.
    LinZ,
    /// The rotational degree of freedom along the joint’s local X axis.
    AngX,
    /// The rotational degree of freedom along the joint’s local Y axis.
    AngY,
    /// The rotational degree of freedom along the joint’s local Z axis.
    AngZ,
}

#[cfg(feature = "dim3")]
#[reflect_remote(rapier::control::WheelTuning)]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Parameters affecting the physical behavior of a wheel of a ray-cast vehicle.
pub struct WheelTuningWrapper {
    /// The suspension stiffness.
    ///
    /// Increase this value if the suspension appears to not push the vehicle strong enough.
    pub suspension_stiffness: Real,
    /// The suspension’s damping when it is being compressed.
    pub suspension_compression: Real,
    /// The suspension’s damping when it is being released.
    ///
    /// Increase this value if the suspension appears to overshoot.
    pub suspension_damping: Real,
    /// The maximum distance the suspension can travel before and after its resting length.
    pub max_suspension_travel: Real,
    /// The multiplier of friction between a tire and the collider it's on top of.
    pub side_friction_stiffness: Real,
    /// Parameter controlling how much traction the tire has.
    ///
    /// The larger the value, the more instantaneous braking will happen (with the risk of
    /// causing the vehicle to flip if it’s too strong).
    pub friction_slip: Real,
    /// The maximum force applied by the suspension.
    pub max_suspension_force: Real,
}

#[cfg(any(feature = "debug-render-3d", feature = "debug-render-2d"))]
#[reflect_remote(rapier::pipeline::DebugRenderStyle)]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Style used for computing colors when rendering the scene with the debug-renderer.
///
/// Colors are in HSLA format: `[hue 0..=360, saturation 0..=1, lightness 0..=1, alpha 0..=1]`.
pub struct DebugRenderStyleWrapper {
    /// The number of subdivisions used to approximate the curved parts of a shape with smooth
    /// faces.
    pub subdivisions: u32,
    /// The number of subdivisions used to approximate the curved borders of round shapes.
    pub border_subdivisions: u32,
    /// The color of colliders attached to dynamic rigid-bodies.
    pub collider_dynamic_color: [f32; 4],
    /// The color of colliders attached to fixed rigid-bodies.
    pub collider_fixed_color: [f32; 4],
    /// The color of colliders attached to kinematic rigid-bodies.
    pub collider_kinematic_color: [f32; 4],
    /// The color of colliders not attached to any rigid-body.
    pub collider_parentless_color: [f32; 4],
    /// The color of the line between a rigid-body’s center-of-mass and the anchors of its
    /// attached impulse joints.
    pub impulse_joint_anchor_color: [f32; 4],
    /// The color of the line between the two anchors of an impulse joint.
    pub impulse_joint_separation_color: [f32; 4],
    /// The color of the line between a rigid-body’s center-of-mass and the anchors of its
    /// attached multibody joints.
    pub multibody_joint_anchor_color: [f32; 4],
    /// The color of the line between the two anchors of a multibody joint.
    pub multibody_joint_separation_color: [f32; 4],
    /// Color multiplier applied to the objects attached to sleeping rigid-bodies.
    pub sleep_color_multiplier: [f32; 4],
    /// Color multiplier applied to the objects attached to awake rigid-bodies eligible for sleep.
    pub sleep_eligible_color_multiplier: [f32; 4],
    /// Color multiplier applied to the objects attached to disabled rigid-bodies.
    pub disabled_color_multiplier: [f32; 4],
    /// The length of the local coordinate axes rendered for a rigid-body.
    pub rigid_body_axes_length: Real,
    /// The color for the segments joining the two contact points.
    pub contact_depth_color: [f32; 4],
    /// The color of the contact normals.
    pub contact_normal_color: [f32; 4],
    /// The length of the contact normals.
    pub contact_normal_length: Real,
    /// The color of the soft bodies' elements.
    pub soft_body_element_color: [f32; 4],
    /// The color of an unloaded soft-body element when coloring the elements by their load.
    pub soft_body_slack_color: [f32; 4],
    /// The color of a soft-body element at its tear threshold when coloring the elements by
    /// their load.
    pub soft_body_loaded_color: [f32; 4],
    /// Color of the soft-body cluster frames.
    pub soft_body_frame_color: [f32; 4],
    /// The color of the colliders' AABBs.
    pub collider_aabb_color: [f32; 4],
    /// The color of the vertex pseudo-normals of triangle-meshes and polylines.
    pub vertex_pseudo_normal_color: [f32; 4],
    /// The color of the edge pseudo-normals of triangle-meshes (3D only).
    pub edge_pseudo_normal_color: [f32; 4],
    /// The length of the pseudo-normals.
    pub pseudo_normal_length: Real,
    /// Color of the soft bodies' volume constraints' normals.
    pub volume_contact_normal_color: [f32; 4],
    /// Color of the volume gradients drawn at the particles of a volume constraint.
    pub volume_gradient_color: [f32; 4],
}
