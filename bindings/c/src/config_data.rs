//! Copyable configuration snapshots. Apply functions validate before replacing native state.
#![allow(non_snake_case)]
use crate::*;
#[cfg(feature = "dim3")]
use rapier::dynamics::FrictionModel;
#[cfg(feature = "fem")]
use rapier::dynamics::SoftFemParameters;
use rapier::dynamics::{
    SoftBodiesSettings, SoftEdgePlasticFlow, SoftPatchConstraints, SoftRecoverySettings,
};

/// Optional scalar override; enabled = 0 selects no override.
/// @ingroup math
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalReal {
    /// Whether this setting/object is enabled (0 or 1).
    pub enabled: RprBool,
    /// Value used when enabled is 1.
    pub value: RprReal,
}
/// Optional unsigned integer override; enabled = 0 selects no override.
/// @ingroup math
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalU32 {
    /// Whether this setting/object is enabled (0 or 1).
    pub enabled: RprBool,
    /// Value used when enabled is 1.
    pub value: u32,
}
/// Optional boolean override. When disabled, retain the recipe's native default.
/// @ingroup math
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalBool {
    /// Whether this setting/object is enabled (0 or 1).
    pub enabled: RprBool,
    /// Value used when enabled is 1.
    pub value: RprBool,
}
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
/// @ingroup soft_bodies
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftBodyMaterial {
    /// Spring coefficients for structural edge constraints.
    pub edgeSoftness: RprSpringCoefficients,
    /// Spring coefficients for bending edges and dihedrals.
    pub bendSoftness: RprSpringCoefficients,
    /// Spring coefficients for cell and global volume constraints.
    pub volumeSoftness: RprSpringCoefficients,
    /// Spring coefficients for shape-matching constraints.
    pub shapeMatchingSoftness: RprSpringCoefficients,
    /// Elastic modulus: force per area in 3D, force per length in 2D; nonnegative.
    pub youngModulus: RprReal,
    /// Poisson ratio for elastic cells, in [0, 0.5).
    pub poissonRatio: RprReal,
    /// Nonnegative damping ratio of elastic cells.
    pub elasticDampingRatio: RprReal,
    /// Cell strain threshold for plastic flow; zero disables plasticity.
    pub plasticYield: RprReal,
    /// Nonnegative rate per second at which excess cell strain becomes permanent.
    pub plasticCreep: RprReal,
    /// Maximum accumulated cell plastic stretch, measured by the norm of P - I.
    pub plasticMax: RprReal,
    /// Rate per second pulling particle velocities toward best-fit rigid motion; zero disables it.
    pub deformationDamping: RprReal,
    /// Edge strain threshold for plastic flow; zero disables plasticity.
    pub edgePlasticYield: RprReal,
    /// Nonnegative rate per second at which excess edge strain becomes permanent.
    pub edgePlasticCreep: RprReal,
    /// Maximum permanent edge-length change as a fraction of its initial length.
    pub edgePlasticMax: RprReal,
    /// Plastic flow direction: RPR_SOFT_EDGE_PLASTIC_FLOW_BOTH, _COMPRESSION or _TENSION.
    pub edgePlasticFlow: u32,
    /// Optional strain threshold for tearing; disabled means no strain-based tearing.
    pub tearStrain: RprOptionalReal,
    /// Optional tensile edge-force threshold for tearing.
    pub tearForce: RprOptionalReal,
    /// Exponential load-smoothing time constant in seconds; zero disables smoothing.
    pub tearSmoothing: RprReal,
    /// Tear-threshold multiplier for undamaged interior elements.
    pub interiorStrength: RprReal,
    /// Maximum ordinary edge tears per step; edges above twice their threshold bypass the limit.
    pub maxTearsPerStep: u32,
    /// Optional minimum particle count of tear pieces.
    pub minPiece: RprOptionalU32,
}
impl From<SoftBodyMaterial> for RprSoftBodyMaterial {
    fn from(value: SoftBodyMaterial) -> Self {
        Self {
            edgeSoftness: value.edge_softness.into(),
            bendSoftness: value.bend_softness.into(),
            volumeSoftness: value.volume_softness.into(),
            shapeMatchingSoftness: value.shape_matching_softness.into(),
            youngModulus: value.young_modulus,
            poissonRatio: value.poisson_ratio,
            elasticDampingRatio: value.elastic_damping_ratio,
            plasticYield: value.plastic_yield,
            plasticCreep: value.plastic_creep,
            plasticMax: value.plastic_max,
            deformationDamping: value.deformation_damping,
            edgePlasticYield: value.edge_plastic_yield,
            edgePlasticCreep: value.edge_plastic_creep,
            edgePlasticMax: value.edge_plastic_max,
            edgePlasticFlow: match value.edge_plastic_flow {
                SoftEdgePlasticFlow::Both => 0,
                SoftEdgePlasticFlow::Compression => 1,
                SoftEdgePlasticFlow::Tension => 2,
            },
            tearStrain: RprOptionalReal {
                enabled: value.tear_strain.is_some() as RprBool,
                value: value.tear_strain.unwrap_or_default(),
            },
            tearForce: RprOptionalReal {
                enabled: value.tear_force.is_some() as RprBool,
                value: value.tear_force.unwrap_or_default(),
            },
            tearSmoothing: value.tear_smoothing,
            interiorStrength: value.interior_strength,
            maxTearsPerStep: value.max_tears_per_step,
            minPiece: RprOptionalU32 {
                enabled: value.min_piece.is_some() as RprBool,
                value: value.min_piece.unwrap_or_default(),
            },
        }
    }
}
impl RprSoftBodyMaterial {
    pub(crate) fn raw(&self) -> Result<SoftBodyMaterial> {
        Ok(SoftBodyMaterial {
            edge_softness: self.edgeSoftness.raw()?,
            bend_softness: self.bendSoftness.raw()?,
            volume_softness: self.volumeSoftness.raw()?,
            shape_matching_softness: self.shapeMatchingSoftness.raw()?,
            young_modulus: nonnegative(self.youngModulus)?,
            poisson_ratio: {
                ensure(
                    (0.0..0.5).contains(&self.poissonRatio),
                    "Poisson ratio must be in [0, 0.5)",
                )?;
                self.poissonRatio
            },
            elastic_damping_ratio: nonnegative(self.elasticDampingRatio)?,
            plastic_yield: nonnegative(self.plasticYield)?,
            plastic_creep: nonnegative(self.plasticCreep)?,
            plastic_max: nonnegative(self.plasticMax)?,
            deformation_damping: nonnegative(self.deformationDamping)?,
            edge_plastic_yield: nonnegative(self.edgePlasticYield)?,
            edge_plastic_creep: nonnegative(self.edgePlasticCreep)?,
            edge_plastic_max: nonnegative(self.edgePlasticMax)?,
            edge_plastic_flow: match self.edgePlasticFlow {
                0 => SoftEdgePlasticFlow::Both,
                1 => SoftEdgePlasticFlow::Compression,
                2 => SoftEdgePlasticFlow::Tension,
                _ => return Err(invalid("invalid edge_plastic_flow")),
            },
            tear_strain: if boolean(self.tearStrain.enabled)? {
                Some(nonnegative(self.tearStrain.value)?)
            } else {
                None
            },
            tear_force: if boolean(self.tearForce.enabled)? {
                Some(nonnegative(self.tearForce.value)?)
            } else {
                None
            },
            tear_smoothing: nonnegative(self.tearSmoothing)?,
            interior_strength: nonnegative(self.interiorStrength)?,
            max_tears_per_step: self.maxTearsPerStep,
            min_piece: if boolean(self.minPiece.enabled)? {
                Some({
                    ensure(self.minPiece.value > 0, "min piece must be positive")?;
                    self.minPiece.value
                })
            } else {
                None
            },
        })
    }
}
/// Return native default soft body material. This POD value owns no resources.
/// @ingroup soft_bodies
#[rapier_export]
pub extern "C" fn rpr_default_soft_body_material() -> RprSoftBodyMaterial {
    SoftBodyMaterial::default().into()
}
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
/// @ingroup soft_bodies
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftRecoverySettings {
    /// Expand speculative contact margins to cover particle velocities set between steps.
    pub authoredVelocityMargin: RprBool,
    /// Enable speculative edge-edge collision constraints.
    pub edgeSpeculation: RprBool,
    /// Detect inverted cells to support self-contact recovery.
    pub invertedCellDetection: RprBool,
    /// Detect surface self-crossings each step.
    pub selfCrossingDetection: RprBool,
    /// Skip self-crossing detection when accumulated motion cannot have created a crossing.
    pub detectionMotionGating: RprBool,
    /// Detect boundary crossings between soft bodies.
    pub crossBodyDetection: RprBool,
    /// Disable contacts on tangled features so elasticity can untangle them.
    pub selfStandDown: RprBool,
    /// Allow contacts at cross-body crossings to expel, but not hold, the intruder.
    pub crossBodyExpelGate: RprBool,
    /// Disable edge constraints touching cross-body crossings.
    pub edgeStandDown: RprBool,
    /// Repel crossing features toward their neighborhood's side of the surface.
    pub crossingRepulsion: RprBool,
    /// Guide cross-body repulsion by overlap-volume normals; closed meshes only.
    pub crossingRepulsionGuide: RprBool,
    /// Guide self-crossing repulsion by self-intersection-volume normals; closed meshes only.
    pub crossingRepulsionSelfGuide: RprBool,
    /// Maximum recovery rate in length units per second, scaled by lengthUnit.
    pub recoveryPace: RprReal,
    /// Enable intersection-volume constraints for overlapping closed surfaces.
    pub overlapConstraints: RprBool,
    /// Enable intersection-volume constraints against rigid colliders.
    pub overlapRigid: RprBool,
    /// Skip pair overlap constraints for self-crossed meshes.
    pub overlapSkipSelfTangled: RprBool,
    /// Disable 3D closed-surface edge constraints where overlap constraints take over.
    pub overlapEdgeStandDown: RprBool,
    /// Velocity-change limit per step, as a multiple of recoveryPace.
    pub overlapConstraintPace: RprReal,
    /// Per-point constraints inside overlap patches: RPR_SOFT_PATCH_CONSTRAINTS_KEEP, _STAND_DOWN or
    /// _ALONG_NORMAL.
    pub overlapPatchConstraints: u32,
    /// Measure overlap on contact-skin surfaces rather than bare geometry.
    pub overlapSkinVolume: RprBool,
    /// Overlap depth retained by recovery, as a fraction of the pair's contact skins.
    pub overlapKeptDepth: RprReal,
    /// Enable recovery of self-intersection regions.
    pub overlapSelfRegions: RprBool,
    /// Use the overlap normal for recovery pushes.
    pub overlapNormalPush: RprBool,
    /// Use spatially split overlap-volume constraints.
    pub overlapMultiVolume: RprBool,
    /// Cells per tangent axis of the multi-volume grid.
    pub overlapSplit: u32,
    /// Recovery progress patience in steps.
    pub overlapPatience: u32,
    /// Relative overlap-volume decrease that counts as recovery progress.
    pub overlapProgressMargin: RprReal,
}
impl From<SoftRecoverySettings> for RprSoftRecoverySettings {
    fn from(value: SoftRecoverySettings) -> Self {
        Self {
            authoredVelocityMargin: value.authored_velocity_margin as RprBool,
            edgeSpeculation: value.edge_speculation as RprBool,
            invertedCellDetection: value.inverted_cell_detection as RprBool,
            selfCrossingDetection: value.self_crossing_detection as RprBool,
            detectionMotionGating: value.detection_motion_gating as RprBool,
            crossBodyDetection: value.cross_body_detection as RprBool,
            selfStandDown: value.self_stand_down as RprBool,
            crossBodyExpelGate: value.cross_body_expel_gate as RprBool,
            edgeStandDown: value.edge_stand_down as RprBool,
            crossingRepulsion: value.crossing_repulsion as RprBool,
            crossingRepulsionGuide: value.crossing_repulsion_guide as RprBool,
            crossingRepulsionSelfGuide: value.crossing_repulsion_self_guide as RprBool,
            recoveryPace: value.recovery_pace,
            overlapConstraints: value.overlap_constraints as RprBool,
            overlapRigid: value.overlap_rigid as RprBool,
            overlapSkipSelfTangled: value.overlap_skip_self_tangled as RprBool,
            overlapEdgeStandDown: value.overlap_edge_stand_down as RprBool,
            overlapConstraintPace: value.overlap_constraint_pace,
            overlapPatchConstraints: match value.overlap_patch_constraints {
                SoftPatchConstraints::Keep => 0,
                SoftPatchConstraints::StandDown => 1,
                SoftPatchConstraints::AlongNormal => 2,
            },
            overlapSkinVolume: value.overlap_skin_volume as RprBool,
            overlapKeptDepth: value.overlap_kept_depth,
            overlapSelfRegions: value.overlap_self_regions as RprBool,
            overlapNormalPush: value.overlap_normal_push as RprBool,
            overlapMultiVolume: value.overlap_multi_volume as RprBool,
            overlapSplit: value.overlap_split,
            overlapPatience: value.overlap_patience,
            overlapProgressMargin: value.overlap_progress_margin,
        }
    }
}
impl RprSoftRecoverySettings {
    pub(crate) fn raw(&self) -> Result<SoftRecoverySettings> {
        Ok(SoftRecoverySettings {
            authored_velocity_margin: boolean(self.authoredVelocityMargin)?,
            edge_speculation: boolean(self.edgeSpeculation)?,
            inverted_cell_detection: boolean(self.invertedCellDetection)?,
            self_crossing_detection: boolean(self.selfCrossingDetection)?,
            detection_motion_gating: boolean(self.detectionMotionGating)?,
            cross_body_detection: boolean(self.crossBodyDetection)?,
            self_stand_down: boolean(self.selfStandDown)?,
            cross_body_expel_gate: boolean(self.crossBodyExpelGate)?,
            edge_stand_down: boolean(self.edgeStandDown)?,
            crossing_repulsion: boolean(self.crossingRepulsion)?,
            crossing_repulsion_guide: boolean(self.crossingRepulsionGuide)?,
            crossing_repulsion_self_guide: boolean(self.crossingRepulsionSelfGuide)?,
            recovery_pace: nonnegative(self.recoveryPace)?,
            overlap_constraints: boolean(self.overlapConstraints)?,
            overlap_rigid: boolean(self.overlapRigid)?,
            overlap_skip_self_tangled: boolean(self.overlapSkipSelfTangled)?,
            overlap_edge_stand_down: boolean(self.overlapEdgeStandDown)?,
            overlap_constraint_pace: nonnegative(self.overlapConstraintPace)?,
            overlap_patch_constraints: match self.overlapPatchConstraints {
                0 => SoftPatchConstraints::Keep,
                1 => SoftPatchConstraints::StandDown,
                2 => SoftPatchConstraints::AlongNormal,
                _ => return Err(invalid("invalid overlap_patch_constraints")),
            },
            overlap_skin_volume: boolean(self.overlapSkinVolume)?,
            overlap_kept_depth: nonnegative(self.overlapKeptDepth)?,
            overlap_self_regions: boolean(self.overlapSelfRegions)?,
            overlap_normal_push: boolean(self.overlapNormalPush)?,
            overlap_multi_volume: boolean(self.overlapMultiVolume)?,
            overlap_split: self.overlapSplit,
            overlap_patience: self.overlapPatience,
            overlap_progress_margin: nonnegative(self.overlapProgressMargin)?,
        })
    }
}
/// Return native default soft recovery settings. This POD value owns no resources.
/// @ingroup soft_bodies
#[rapier_export]
pub extern "C" fn rpr_default_soft_recovery_settings() -> RprSoftRecoverySettings {
    SoftRecoverySettings::default().into()
}
#[cfg(feature = "fem")]
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
/// @ingroup soft_bodies
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftFemParameters {
    /// FEM iterative linear-solver tolerance.
    pub linearTolerance: RprReal,
    /// Maximum FEM linear-solver iterations.
    pub maxLinearIterations: usize,
    /// Maximum degrees of freedom solved by the dense FEM solver.
    pub maxDenseDofs: usize,
}
#[cfg(feature = "fem")]
impl From<SoftFemParameters> for RprSoftFemParameters {
    fn from(value: SoftFemParameters) -> Self {
        Self {
            linearTolerance: value.linear_tolerance,
            maxLinearIterations: value.max_linear_iterations,
            maxDenseDofs: value.max_dense_dofs,
        }
    }
}
#[cfg(feature = "fem")]
impl RprSoftFemParameters {
    pub(crate) fn raw(&self) -> Result<SoftFemParameters> {
        Ok(SoftFemParameters {
            linear_tolerance: positive(self.linearTolerance)?,
            max_linear_iterations: self.maxLinearIterations,
            max_dense_dofs: self.maxDenseDofs,
        })
    }
}
/// Return native default soft fem parameters. This POD value owns no resources.
/// @ingroup soft_bodies
#[cfg(feature = "fem")]
#[rapier_export]
pub extern "C" fn rpr_default_soft_fem_parameters() -> RprSoftFemParameters {
    SoftFemParameters::default().into()
}
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
/// @ingroup soft_bodies
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftBodiesSettings {
    /// Soft-body crossing detection and recovery settings.
    pub recovery: RprSoftRecoverySettings,
    /// Strain threshold for re-solving soft constraints after contacts within a substep.
    pub resweepStrain: RprReal,
    /// Maximum additional substeps requested by soft-body motion.
    pub maxExtraSubsteps: usize,
    /// Multiplier on contact natural frequency for soft-body contacts.
    pub contactStiffening: RprReal,
    #[cfg(feature = "fem")]
    /// FEM linear-solver settings, present only when RAPIER_FEM is enabled.
    pub fem: RprSoftFemParameters,
}
impl From<SoftBodiesSettings> for RprSoftBodiesSettings {
    fn from(value: SoftBodiesSettings) -> Self {
        Self {
            recovery: value.recovery.into(),
            resweepStrain: value.resweep_strain,
            maxExtraSubsteps: value.max_extra_substeps,
            contactStiffening: value.contact_stiffening,
            #[cfg(feature = "fem")]
            fem: value.fem.into(),
        }
    }
}
impl RprSoftBodiesSettings {
    pub(crate) fn raw(&self) -> Result<SoftBodiesSettings> {
        Ok(SoftBodiesSettings {
            recovery: self.recovery.raw()?,
            resweep_strain: nonnegative(self.resweepStrain)?,
            max_extra_substeps: self.maxExtraSubsteps,
            contact_stiffening: nonnegative(self.contactStiffening)?,
            #[cfg(feature = "fem")]
            fem: self.fem.raw()?,
        })
    }
}
/// Return native default soft bodies settings. This POD value owns no resources.
/// @ingroup soft_bodies
#[rapier_export]
pub extern "C" fn rpr_default_soft_bodies_settings() -> RprSoftBodiesSettings {
    SoftBodiesSettings::default().into()
}
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
/// @ingroup worlds
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprIntegrationParameters {
    /// Simulation step duration in seconds.
    pub dt: RprReal,
    /// Minimum CCD substep duration in seconds.
    pub minCcdDt: RprReal,
    /// Spring coefficients for dynamic contact constraints.
    pub contactSoftness: RprSpringCoefficients,
    /// Spring coefficients for contacts against fixed bodies.
    pub staticContactSoftness: RprSpringCoefficients,
    /// Scale applied to cached impulses when warmstarting.
    pub warmstartCoefficient: RprReal,
    /// Typical world-space length of one meter; scales solver tolerances, not geometry.
    pub lengthUnit: RprReal,
    /// Soft-body integration and recovery settings.
    pub softBodies: RprSoftBodiesSettings,
    /// Allowed penetration divided by lengthUnit.
    pub normalizedAllowedLinearError: RprReal,
    /// Maximum penetration-correction speed divided by lengthUnit.
    pub normalizedMaxCorrectiveVelocity: RprReal,
    /// Speculative-contact distance divided by lengthUnit.
    pub normalizedPredictionDistance: RprReal,
    /// Maximum linear speed divided by lengthUnit.
    pub normalizedMaxLinearVelocity: RprReal,
    /// Number of solver substeps/iterations; must be positive.
    pub numSolverIterations: usize,
    /// PGS iterations per solver substep; must be positive.
    pub numInternalPgsIterations: usize,
    /// Stabilization iterations after velocity solving.
    pub numInternalStabilizationIterations: usize,
    /// Maximum CCD substeps; 0 disables all CCD for the world.
    pub maxCcdSubsteps: usize,
    /// Whether to cluster contacts for solving.
    pub contactClustering: RprBool,
    /// Whether to reuse nearby contacts between steps.
    pub contactRecycling: RprBool,
    /// Contact recycling distance divided by lengthUnit.
    pub normalizedContactRecycleDistance: RprReal,
    /// Whether to solve friction in the bias pass.
    pub frictionInBiasPass: RprBool,
    /// Whether to warmstart joint constraints.
    pub warmstartJoints: RprBool,
    #[cfg(feature = "dim3")]
    /// Friction model of rigid-body contacts, RPR_FRICTION_MODEL_* (3D only).
    pub frictionModel: u32,
}
impl From<IntegrationParameters> for RprIntegrationParameters {
    fn from(value: IntegrationParameters) -> Self {
        Self {
            dt: value.dt,
            minCcdDt: value.min_ccd_dt,
            contactSoftness: value.contact_softness.into(),
            staticContactSoftness: value.static_contact_softness.into(),
            warmstartCoefficient: value.warmstart_coefficient,
            lengthUnit: value.length_unit,
            softBodies: value.soft_bodies.into(),
            normalizedAllowedLinearError: value.normalized_allowed_linear_error,
            normalizedMaxCorrectiveVelocity: value.normalized_max_corrective_velocity,
            normalizedPredictionDistance: value.normalized_prediction_distance,
            normalizedMaxLinearVelocity: value.normalized_max_linear_velocity,
            numSolverIterations: value.num_solver_iterations,
            numInternalPgsIterations: value.num_internal_pgs_iterations,
            numInternalStabilizationIterations: value.num_internal_stabilization_iterations,
            maxCcdSubsteps: value.max_ccd_substeps,
            contactClustering: value.contact_clustering as RprBool,
            contactRecycling: value.contact_recycling as RprBool,
            normalizedContactRecycleDistance: value.normalized_contact_recycle_distance,
            frictionInBiasPass: value.friction_in_bias_pass as RprBool,
            warmstartJoints: value.warmstart_joints as RprBool,
            #[cfg(feature = "dim3")]
            frictionModel: friction_model_value(value.friction_model),
        }
    }
}
impl RprIntegrationParameters {
    pub(crate) fn raw(&self) -> Result<IntegrationParameters> {
        Ok(IntegrationParameters {
            dt: positive(self.dt)?,
            min_ccd_dt: nonnegative(self.minCcdDt)?,
            contact_softness: self.contactSoftness.raw()?,
            static_contact_softness: self.staticContactSoftness.raw()?,
            warmstart_coefficient: nonnegative(self.warmstartCoefficient)?,
            length_unit: positive(self.lengthUnit)?,
            soft_bodies: self.softBodies.raw()?,
            normalized_allowed_linear_error: nonnegative(self.normalizedAllowedLinearError)?,
            normalized_max_corrective_velocity: nonnegative(self.normalizedMaxCorrectiveVelocity)?,
            normalized_prediction_distance: nonnegative(self.normalizedPredictionDistance)?,
            normalized_max_linear_velocity: nonnegative(self.normalizedMaxLinearVelocity)?,
            num_solver_iterations: iterations(self.numSolverIterations)?,
            num_internal_pgs_iterations: iterations(self.numInternalPgsIterations)?,
            num_internal_stabilization_iterations: self.numInternalStabilizationIterations,
            max_ccd_substeps: self.maxCcdSubsteps,
            contact_clustering: boolean(self.contactClustering)?,
            contact_recycling: boolean(self.contactRecycling)?,
            normalized_contact_recycle_distance: nonnegative(
                self.normalizedContactRecycleDistance,
            )?,
            friction_in_bias_pass: boolean(self.frictionInBiasPass)?,
            warmstart_joints: boolean(self.warmstartJoints)?,
            #[cfg(feature = "dim3")]
            friction_model: friction_model(self.frictionModel)?,
        })
    }
}
/// @ingroup worlds
/// Friction model solving one Coulomb friction constraint per group of up to 4 contacts plus a
/// twist constraint; faster but less accurate (default).
#[cfg(feature = "dim3")]
pub const RPR_FRICTION_MODEL_SIMPLIFIED: u32 = 0;
/// @ingroup worlds
/// Friction model solving one Coulomb friction constraint per contact point.
#[cfg(feature = "dim3")]
pub const RPR_FRICTION_MODEL_COULOMB: u32 = 1;
/// Validates a solver iteration count, which must be positive.
pub(crate) fn iterations(value: usize) -> Result<usize> {
    ensure(value > 0, "iteration count must be positive")?;
    Ok(value)
}
#[cfg(feature = "dim3")]
pub(crate) fn friction_model(value: u32) -> Result<FrictionModel> {
    match value {
        RPR_FRICTION_MODEL_SIMPLIFIED => Ok(FrictionModel::Simplified),
        RPR_FRICTION_MODEL_COULOMB => Ok(FrictionModel::Coulomb),
        _ => Err(invalid("unknown friction model")),
    }
}
#[cfg(feature = "dim3")]
pub(crate) fn friction_model_value(model: FrictionModel) -> u32 {
    match model {
        FrictionModel::Simplified => RPR_FRICTION_MODEL_SIMPLIFIED,
        FrictionModel::Coulomb => RPR_FRICTION_MODEL_COULOMB,
    }
}
/// Return native default integration parameters. This POD value owns no resources.
/// @ingroup worlds
#[rapier_export]
pub extern "C" fn rpr_default_integration_parameters() -> RprIntegrationParameters {
    IntegrationParameters::default().into()
}

/// Return a copy of all world integration settings.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_integration_parameters(
    world: *const RprWorld,
) -> RprIntegrationParameters {
    ffi_value(|out: *mut RprIntegrationParameters| {
        ffi(|| unsafe {
            let access = get(world)?.read()?;
            let raw = access.raw();

            let object: *const NativeIntegrationParameters =
                std::ptr::addr_of!((*raw).0.integration_parameters).cast();
            output(out, get(object)?.0.into())
        })
    })
}

/// Copies validated values; does not expose a writable alias to Rust memory.
/// @ingroup worlds
#[rapier_export]
pub unsafe extern "C" fn rpr_set_integration_parameters(
    world: *mut RprWorld,
    data: *const RprIntegrationParameters,
) -> RprStatus {
    ffi(|| unsafe {
        let access = get(world)?.write()?;
        let raw = access.raw();

        crate::handle_access::forward(native_integration_parameters_set_data(
            std::ptr::addr_of_mut!((*raw).0.integration_parameters).cast(),
            data,
        ))
    })
}

pub(crate) unsafe fn native_integration_parameters_set_data(
    object: *mut NativeIntegrationParameters,
    data: *const RprIntegrationParameters,
) -> RprStatus {
    ffi(|| unsafe {
        let value = get(data)?.raw()?;
        get_mut(object)?.0 = value;
        Ok(())
    })
}

/// Copies the live soft body's material into caller-owned data.
pub(crate) unsafe fn native_soft_body_read_material(
    body: *const RprSoftBody,
    out: *mut RprSoftBodyMaterial,
) -> RprStatus {
    ffi(|| unsafe { output(out, (*get(body)?.0.material()).into()) })
}
/// Validates all fields before applying the material to a live soft body.
pub(crate) unsafe fn native_soft_body_set_material_data(
    body: *mut RprSoftBody,
    data: *const RprSoftBodyMaterial,
) -> RprStatus {
    ffi(|| unsafe {
        let material = get(data)?.raw()?;
        get_mut(body)?.0.set_material(material);
        Ok(())
    })
}
