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

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalReal {
    pub enabled: RprBool,
    pub value: RprReal,
}
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalU32 {
    pub enabled: RprBool,
    pub value: u32,
}
/// Optional boolean override. When disabled, retain the recipe's native default.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprOptionalBool {
    pub enabled: RprBool,
    pub value: RprBool,
}
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftBodyMaterial {
    pub edgeSoftness: RprSpringCoefficients,
    pub bendSoftness: RprSpringCoefficients,
    pub volumeSoftness: RprSpringCoefficients,
    pub shapeMatchingSoftness: RprSpringCoefficients,
    pub youngModulus: RprReal,
    pub poissonRatio: RprReal,
    pub elasticDampingRatio: RprReal,
    pub plasticYield: RprReal,
    pub plasticCreep: RprReal,
    pub plasticMax: RprReal,
    pub deformationDamping: RprReal,
    pub edgePlasticYield: RprReal,
    pub edgePlasticCreep: RprReal,
    pub edgePlasticMax: RprReal,
    pub edgePlasticFlow: u32,
    pub tearStrain: RprOptionalReal,
    pub tearForce: RprOptionalReal,
    pub tearSmoothing: RprReal,
    pub interiorStrength: RprReal,
    pub maxTearsPerStep: u32,
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
#[rapier_export]
pub extern "C" fn rpr_default_soft_body_material() -> RprSoftBodyMaterial {
    SoftBodyMaterial::default().into()
}
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftRecoverySettings {
    pub authoredVelocityMargin: RprBool,
    pub edgeSpeculation: RprBool,
    pub invertedCellDetection: RprBool,
    pub selfCrossingDetection: RprBool,
    pub detectionMotionGating: RprBool,
    pub crossBodyDetection: RprBool,
    pub selfStandDown: RprBool,
    pub crossBodyExpelGate: RprBool,
    pub edgeStandDown: RprBool,
    pub crossingRepulsion: RprBool,
    pub crossingRepulsionGuide: RprBool,
    pub crossingRepulsionSelfGuide: RprBool,
    pub recoveryPace: RprReal,
    pub overlapConstraints: RprBool,
    pub overlapRigid: RprBool,
    pub overlapSkipSelfTangled: RprBool,
    pub overlapEdgeStandDown: RprBool,
    pub overlapConstraintPace: RprReal,
    pub overlapPatchConstraints: u32,
    pub overlapSkinVolume: RprBool,
    pub overlapKeptDepth: RprReal,
    pub overlapSelfRegions: RprBool,
    pub overlapNormalPush: RprBool,
    pub overlapMultiVolume: RprBool,
    pub overlapSplit: u32,
    pub overlapPatience: u32,
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
#[rapier_export]
pub extern "C" fn rpr_default_soft_recovery_settings() -> RprSoftRecoverySettings {
    SoftRecoverySettings::default().into()
}
#[cfg(feature = "fem")]
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftFemParameters {
    pub linearTolerance: RprReal,
    pub maxLinearIterations: usize,
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
#[cfg(feature = "fem")]
#[rapier_export]
pub extern "C" fn rpr_default_soft_fem_parameters() -> RprSoftFemParameters {
    SoftFemParameters::default().into()
}
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftBodiesSettings {
    pub recovery: RprSoftRecoverySettings,
    pub resweepStrain: RprReal,
    pub maxExtraSubsteps: usize,
    pub contactStiffening: RprReal,
    #[cfg(feature = "fem")]
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
#[rapier_export]
pub extern "C" fn rpr_default_soft_bodies_settings() -> RprSoftBodiesSettings {
    SoftBodiesSettings::default().into()
}
/// Plain configuration data; initialize defaults, edit, then apply. No destructor.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprIntegrationParameters {
    pub dt: RprReal,
    pub minCcdDt: RprReal,
    pub contactSoftness: RprSpringCoefficients,
    pub staticContactSoftness: RprSpringCoefficients,
    pub warmstartCoefficient: RprReal,
    pub lengthUnit: RprReal,
    pub softBodies: RprSoftBodiesSettings,
    pub normalizedAllowedLinearError: RprReal,
    pub normalizedMaxCorrectiveVelocity: RprReal,
    pub normalizedPredictionDistance: RprReal,
    pub normalizedMaxLinearVelocity: RprReal,
    pub numSolverIterations: usize,
    pub numInternalPgsIterations: usize,
    pub numInternalStabilizationIterations: usize,
    pub maxCcdSubsteps: usize,
    pub contactClustering: RprBool,
    pub contactRecycling: RprBool,
    pub normalizedContactRecycleDistance: RprReal,
    pub frictionInBiasPass: RprBool,
    pub warmstartJoints: RprBool,
    #[cfg(feature = "dim3")]
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
            frictionModel: match value.friction_model {
                FrictionModel::Simplified => 0,
                FrictionModel::Coulomb => 1,
            },
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
            num_solver_iterations: self.numSolverIterations,
            num_internal_pgs_iterations: self.numInternalPgsIterations,
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
            friction_model: match self.frictionModel {
                0 => FrictionModel::Simplified,
                1 => FrictionModel::Coulomb,
                _ => return Err(invalid("invalid friction_model")),
            },
        })
    }
}
#[rapier_export]
pub extern "C" fn rpr_default_integration_parameters() -> RprIntegrationParameters {
    IntegrationParameters::default().into()
}

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
