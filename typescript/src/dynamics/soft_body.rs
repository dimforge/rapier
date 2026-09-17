use crate::dynamics::{
    RawImpulseJointSet, RawIslandManager, RawMultibodyJointSet, RawRigidBodySet,
};
use crate::geometry::RawColliderSet;
use crate::math::{RawRotation, RawVector};
use crate::utils::{self, FlatHandle};
use rapier::dynamics::{
    SoftBody, SoftBodyBuilder, SoftBodyCellModel, SoftBodyMaterial, SoftBodySet, SoftBodyTearEvent,
    SoftEdgePlasticFlow, SpringCoefficients,
};
use rapier::math::{Pose, Vector, DIM};
use rapier::prelude::ColliderBuilder;
use wasm_bindgen::prelude::*;

/// Writes a vector into a JS scratch buffer, one float per dimension.
fn write_vector(scratch_buffer: &js_sys::Float32Array, v: Vector) {
    for (i, x) in v.to_array().iter().enumerate() {
        scratch_buffer.set_index(i as u32, *x);
    }
}

/// Reads `DIM` floats per vector out of a flat array.
fn read_vectors(data: &[f32]) -> Vec<Vector> {
    data.chunks_exact(DIM).map(Vector::from_slice).collect()
}

/// Reads `N` indices per element out of a flat array.
fn read_elements<const N: usize>(data: &[u32]) -> Vec<[u32; N]> {
    data.chunks_exact(N)
        .map(|e| {
            let mut out = [0; N];
            out.copy_from_slice(e);
            out
        })
        .collect()
}

/// Flattens elements of `N` indices each.
fn flatten_elements<const N: usize>(elements: &[[u32; N]]) -> Vec<u32> {
    elements.iter().flat_map(|e| e.iter().copied()).collect()
}

#[wasm_bindgen]
#[derive(Copy, Clone)]
pub enum RawSoftBodyCellModel {
    Volume = 0,
    Corotational = 1,
    NeoHookean = 2,
}

impl From<RawSoftBodyCellModel> for SoftBodyCellModel {
    fn from(model: RawSoftBodyCellModel) -> Self {
        match model {
            RawSoftBodyCellModel::Volume => SoftBodyCellModel::Volume,
            RawSoftBodyCellModel::Corotational => SoftBodyCellModel::Corotational,
            RawSoftBodyCellModel::NeoHookean => SoftBodyCellModel::NeoHookean,
        }
    }
}

impl From<SoftBodyCellModel> for RawSoftBodyCellModel {
    fn from(model: SoftBodyCellModel) -> Self {
        match model {
            SoftBodyCellModel::Volume => RawSoftBodyCellModel::Volume,
            SoftBodyCellModel::Corotational => RawSoftBodyCellModel::Corotational,
            SoftBodyCellModel::NeoHookean => RawSoftBodyCellModel::NeoHookean,
        }
    }
}

#[wasm_bindgen]
#[derive(Copy, Clone)]
pub enum RawSoftEdgePlasticFlow {
    Both = 0,
    Compression = 1,
    Tension = 2,
}

impl From<RawSoftEdgePlasticFlow> for SoftEdgePlasticFlow {
    fn from(flow: RawSoftEdgePlasticFlow) -> Self {
        match flow {
            RawSoftEdgePlasticFlow::Both => SoftEdgePlasticFlow::Both,
            RawSoftEdgePlasticFlow::Compression => SoftEdgePlasticFlow::Compression,
            RawSoftEdgePlasticFlow::Tension => SoftEdgePlasticFlow::Tension,
        }
    }
}

impl From<SoftEdgePlasticFlow> for RawSoftEdgePlasticFlow {
    fn from(flow: SoftEdgePlasticFlow) -> Self {
        match flow {
            SoftEdgePlasticFlow::Both => RawSoftEdgePlasticFlow::Both,
            SoftEdgePlasticFlow::Compression => RawSoftEdgePlasticFlow::Compression,
            SoftEdgePlasticFlow::Tension => RawSoftEdgePlasticFlow::Tension,
        }
    }
}

/// How a deformable collider's vertices follow the particles of its cluster.
#[wasm_bindgen]
#[derive(Copy, Clone)]
pub enum RawSoftMeshBindingMode {
    Direct = 0,
    DirectByPosition = 1,
    Skinned = 2,
}

/*
 * Material.
 */

#[wasm_bindgen]
#[derive(Clone)]
pub struct RawSoftBodyMaterial(pub(crate) SoftBodyMaterial);

#[wasm_bindgen]
impl RawSoftBodyMaterial {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawSoftBodyMaterial(SoftBodyMaterial::default())
    }

    pub fn uniform(natural_frequency: f32, damping_ratio: f32) -> Self {
        RawSoftBodyMaterial(SoftBodyMaterial::uniform(SpringCoefficients::new(
            natural_frequency,
            damping_ratio,
        )))
    }

    #[wasm_bindgen(getter)]
    pub fn edgeFrequency(&self) -> f32 {
        self.0.edge_softness.natural_frequency
    }
    #[wasm_bindgen(setter)]
    pub fn set_edgeFrequency(&mut self, value: f32) {
        self.0.edge_softness.natural_frequency = value;
    }
    #[wasm_bindgen(getter)]
    pub fn edgeDampingRatio(&self) -> f32 {
        self.0.edge_softness.damping_ratio
    }
    #[wasm_bindgen(setter)]
    pub fn set_edgeDampingRatio(&mut self, value: f32) {
        self.0.edge_softness.damping_ratio = value;
    }
    #[wasm_bindgen(getter)]
    pub fn bendFrequency(&self) -> f32 {
        self.0.bend_softness.natural_frequency
    }
    #[wasm_bindgen(setter)]
    pub fn set_bendFrequency(&mut self, value: f32) {
        self.0.bend_softness.natural_frequency = value;
    }
    #[wasm_bindgen(getter)]
    pub fn bendDampingRatio(&self) -> f32 {
        self.0.bend_softness.damping_ratio
    }
    #[wasm_bindgen(setter)]
    pub fn set_bendDampingRatio(&mut self, value: f32) {
        self.0.bend_softness.damping_ratio = value;
    }
    #[wasm_bindgen(getter)]
    pub fn volumeFrequency(&self) -> f32 {
        self.0.volume_softness.natural_frequency
    }
    #[wasm_bindgen(setter)]
    pub fn set_volumeFrequency(&mut self, value: f32) {
        self.0.volume_softness.natural_frequency = value;
    }
    #[wasm_bindgen(getter)]
    pub fn volumeDampingRatio(&self) -> f32 {
        self.0.volume_softness.damping_ratio
    }
    #[wasm_bindgen(setter)]
    pub fn set_volumeDampingRatio(&mut self, value: f32) {
        self.0.volume_softness.damping_ratio = value;
    }
    #[wasm_bindgen(getter)]
    pub fn shapeMatchingFrequency(&self) -> f32 {
        self.0.shape_matching_softness.natural_frequency
    }
    #[wasm_bindgen(setter)]
    pub fn set_shapeMatchingFrequency(&mut self, value: f32) {
        self.0.shape_matching_softness.natural_frequency = value;
    }
    #[wasm_bindgen(getter)]
    pub fn shapeMatchingDampingRatio(&self) -> f32 {
        self.0.shape_matching_softness.damping_ratio
    }
    #[wasm_bindgen(setter)]
    pub fn set_shapeMatchingDampingRatio(&mut self, value: f32) {
        self.0.shape_matching_softness.damping_ratio = value;
    }
    #[wasm_bindgen(getter)]
    pub fn youngModulus(&self) -> f32 {
        self.0.young_modulus
    }
    #[wasm_bindgen(setter)]
    pub fn set_youngModulus(&mut self, value: f32) {
        self.0.young_modulus = value;
    }
    #[wasm_bindgen(getter)]
    pub fn poissonRatio(&self) -> f32 {
        self.0.poisson_ratio
    }
    #[wasm_bindgen(setter)]
    pub fn set_poissonRatio(&mut self, value: f32) {
        self.0.poisson_ratio = value;
    }
    #[wasm_bindgen(getter)]
    pub fn elasticDampingRatio(&self) -> f32 {
        self.0.elastic_damping_ratio
    }
    #[wasm_bindgen(setter)]
    pub fn set_elasticDampingRatio(&mut self, value: f32) {
        self.0.elastic_damping_ratio = value;
    }
    #[wasm_bindgen(getter)]
    pub fn plasticYield(&self) -> f32 {
        self.0.plastic_yield
    }
    #[wasm_bindgen(setter)]
    pub fn set_plasticYield(&mut self, value: f32) {
        self.0.plastic_yield = value;
    }
    #[wasm_bindgen(getter)]
    pub fn plasticCreep(&self) -> f32 {
        self.0.plastic_creep
    }
    #[wasm_bindgen(setter)]
    pub fn set_plasticCreep(&mut self, value: f32) {
        self.0.plastic_creep = value;
    }
    #[wasm_bindgen(getter)]
    pub fn plasticMax(&self) -> f32 {
        self.0.plastic_max
    }
    #[wasm_bindgen(setter)]
    pub fn set_plasticMax(&mut self, value: f32) {
        self.0.plastic_max = value;
    }
    #[wasm_bindgen(getter)]
    pub fn deformationDamping(&self) -> f32 {
        self.0.deformation_damping
    }
    #[wasm_bindgen(setter)]
    pub fn set_deformationDamping(&mut self, value: f32) {
        self.0.deformation_damping = value;
    }
    #[wasm_bindgen(getter)]
    pub fn edgePlasticYield(&self) -> f32 {
        self.0.edge_plastic_yield
    }
    #[wasm_bindgen(setter)]
    pub fn set_edgePlasticYield(&mut self, value: f32) {
        self.0.edge_plastic_yield = value;
    }
    #[wasm_bindgen(getter)]
    pub fn edgePlasticCreep(&self) -> f32 {
        self.0.edge_plastic_creep
    }
    #[wasm_bindgen(setter)]
    pub fn set_edgePlasticCreep(&mut self, value: f32) {
        self.0.edge_plastic_creep = value;
    }
    #[wasm_bindgen(getter)]
    pub fn edgePlasticMax(&self) -> f32 {
        self.0.edge_plastic_max
    }
    #[wasm_bindgen(setter)]
    pub fn set_edgePlasticMax(&mut self, value: f32) {
        self.0.edge_plastic_max = value;
    }
    #[wasm_bindgen(getter)]
    pub fn tearSmoothing(&self) -> f32 {
        self.0.tear_smoothing
    }
    #[wasm_bindgen(setter)]
    pub fn set_tearSmoothing(&mut self, value: f32) {
        self.0.tear_smoothing = value;
    }
    #[wasm_bindgen(getter)]
    pub fn interiorStrength(&self) -> f32 {
        self.0.interior_strength
    }
    #[wasm_bindgen(setter)]
    pub fn set_interiorStrength(&mut self, value: f32) {
        self.0.interior_strength = value;
    }

    #[wasm_bindgen(getter)]
    pub fn edgePlasticFlow(&self) -> RawSoftEdgePlasticFlow {
        self.0.edge_plastic_flow.into()
    }
    #[wasm_bindgen(setter)]
    pub fn set_edgePlasticFlow(&mut self, value: RawSoftEdgePlasticFlow) {
        self.0.edge_plastic_flow = value.into();
    }

    #[wasm_bindgen(getter)]
    pub fn tearStrain(&self) -> Option<f32> {
        self.0.tear_strain
    }
    #[wasm_bindgen(setter)]
    pub fn set_tearStrain(&mut self, value: Option<f32>) {
        self.0.tear_strain = value;
    }

    #[wasm_bindgen(getter)]
    pub fn tearForce(&self) -> Option<f32> {
        self.0.tear_force
    }
    #[wasm_bindgen(setter)]
    pub fn set_tearForce(&mut self, value: Option<f32>) {
        self.0.tear_force = value;
    }

    #[wasm_bindgen(getter)]
    pub fn maxTearsPerStep(&self) -> u32 {
        self.0.max_tears_per_step
    }
    #[wasm_bindgen(setter)]
    pub fn set_maxTearsPerStep(&mut self, value: u32) {
        self.0.max_tears_per_step = value;
    }

    #[wasm_bindgen(getter)]
    pub fn minPiece(&self) -> Option<u32> {
        self.0.min_piece
    }
    #[wasm_bindgen(setter)]
    pub fn set_minPiece(&mut self, value: Option<u32>) {
        self.0.min_piece = value;
    }

    pub fn tears(&self) -> bool {
        self.0.tears()
    }
}

/*
 * Builder.
 */

#[wasm_bindgen]
#[derive(Clone)]
pub struct RawSoftBodyBuilder(pub(crate) SoftBodyBuilder);

#[wasm_bindgen]
impl RawSoftBodyBuilder {
    /// A builder over the given world-space particle positions (`DIM` floats per particle), with
    /// no element.
    #[wasm_bindgen(constructor)]
    pub fn new(positions: Vec<f32>) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::new(read_vectors(&positions)))
    }

    pub fn rope(start: &RawVector, end: &RawVector, num_particles: usize) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::rope(start.0, end.0, num_particles))
    }

    #[cfg(feature = "dim2")]
    pub fn polyline(vertices: Vec<f32>, indices: Vec<u32>) -> Option<RawSoftBodyBuilder> {
        let indices = if indices.is_empty() {
            None
        } else {
            Some(read_elements::<2>(&indices))
        };
        SoftBodyBuilder::polyline(read_vectors(&vertices), indices).map(RawSoftBodyBuilder)
    }

    #[cfg(feature = "dim3")]
    pub fn trimesh(vertices: Vec<f32>, indices: Vec<u32>) -> Option<RawSoftBodyBuilder> {
        SoftBodyBuilder::trimesh(read_vectors(&vertices), read_elements::<3>(&indices))
            .map(RawSoftBodyBuilder)
    }

    #[cfg(feature = "dim3")]
    pub fn cloth(origin: &RawVector, du: &RawVector, dv: &RawVector, nu: usize, nv: usize) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::cloth(origin.0, du.0, dv.0, nu, nv))
    }

    #[cfg(feature = "dim3")]
    pub fn clothTube(
        origin: &RawVector,
        axis: &RawVector,
        radius_start: f32,
        radius_end: f32,
        num_around: usize,
        num_along: usize,
    ) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::cloth_tube(
            origin.0,
            axis.0,
            radius_start,
            radius_end,
            num_around,
            num_along,
        ))
    }

    #[cfg(feature = "dim3")]
    pub fn clothAnisotropic(
        origin: &RawVector,
        du: &RawVector,
        dv: &RawVector,
        nu: usize,
        nv: usize,
        warp_frequency: f32,
        warp_damping: f32,
        weft_frequency: f32,
        weft_damping: f32,
        shear_frequency: f32,
        shear_damping: f32,
    ) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::cloth_anisotropic(
            origin.0,
            du.0,
            dv.0,
            nu,
            nv,
            SpringCoefficients::new(warp_frequency, warp_damping),
            SpringCoefficients::new(weft_frequency, weft_damping),
            SpringCoefficients::new(shear_frequency, shear_damping),
        ))
    }

    #[cfg(feature = "dim3")]
    pub fn cuboid(
        center: &RawVector,
        half_extents: &RawVector,
        nx: usize,
        ny: usize,
        nz: usize,
    ) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::cuboid(
            center.0,
            half_extents.0,
            nx,
            ny,
            nz,
        ))
    }

    #[cfg(feature = "dim3")]
    pub fn sphere(center: &RawVector, radius: f32, subdivisions: usize) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::sphere(center.0, radius, subdivisions))
    }

    #[cfg(feature = "dim2")]
    pub fn polygon(points: Vec<f32>) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::polygon(read_vectors(&points)))
    }

    #[cfg(feature = "dim2")]
    pub fn disk(center: &RawVector, radius: f32, num_particles: usize) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::disk(center.0, radius, num_particles))
    }

    #[cfg(feature = "dim2")]
    pub fn grid(center: &RawVector, half_extents: &RawVector, nx: usize, ny: usize) -> Self {
        RawSoftBodyBuilder(SoftBodyBuilder::grid(center.0, half_extents.0, nx, ny))
    }

    /// A volumetric body filling the closed surface (segments in 2D, triangles in 3D) with
    /// cells of the given size.
    pub fn volumetric(
        vertices: Vec<f32>,
        indices: Vec<u32>,
        cell_size: f32,
        skinned: bool,
    ) -> Option<RawSoftBodyBuilder> {
        let vertices = read_vectors(&vertices);
        let indices = read_elements::<DIM>(&indices);
        if skinned {
            SoftBodyBuilder::volumetric_skinned(&vertices, &indices, cell_size)
        } else {
            SoftBodyBuilder::volumetric(&vertices, &indices, cell_size)
        }
        .map(RawSoftBodyBuilder)
    }

    pub fn numParticles(&self) -> usize {
        self.0.positions.len()
    }

    pub fn particlePositions(&self) -> Vec<f32> {
        self.0.positions.iter().flat_map(|p| p.to_array()).collect()
    }

    pub fn surfaceEdges(&self) -> Vec<u32> {
        flatten_elements(&self.0.surface_edges())
    }

    pub fn cellEdges(&self) -> Vec<u32> {
        flatten_elements(&self.0.cell_edges())
    }

    #[cfg(feature = "dim3")]
    pub fn surfaceDihedrals(&self) -> Vec<u32> {
        flatten_elements(&self.0.surface_dihedrals())
    }

    pub fn append(&mut self, other: &RawSoftBodyBuilder) {
        let me = std::mem::take(&mut self.0);
        self.0 = me.append(other.0.clone());
    }

    pub fn translated(&mut self, translation: &RawVector) {
        let me = std::mem::take(&mut self.0);
        self.0 = me.translated(translation.0);
    }

    pub fn setPositions(&mut self, positions: Vec<f32>) {
        self.0.positions = read_vectors(&positions);
    }

    pub fn setParticleMass(&mut self, mass: f32) {
        self.0.particle_mass = mass;
        self.0.masses.clear();
    }

    pub fn setMass(&mut self, mass: f32) {
        let me = std::mem::take(&mut self.0);
        self.0 = me.mass(mass);
    }

    pub fn setMasses(&mut self, masses: Vec<f32>) {
        self.0.masses = masses;
    }

    pub fn setPinnedParticles(&mut self, pinned: Vec<u32>) {
        self.0.pinned = pinned;
    }

    pub fn setEdges(&mut self, edges: Vec<u32>) {
        self.0.edges = read_elements::<2>(&edges);
    }

    pub fn addEdges(&mut self, edges: Vec<u32>) {
        self.0.edges.extend(read_elements::<2>(&edges));
    }

    pub fn setBendEdges(&mut self, edges: Vec<u32>) {
        self.0.bend_edges = read_elements::<2>(&edges);
    }

    pub fn setTensionOnly(&mut self) {
        let me = std::mem::take(&mut self.0);
        self.0 = me.tension_only();
    }

    #[cfg(feature = "dim3")]
    pub fn setDihedrals(&mut self, dihedrals: Vec<u32>) {
        self.0.dihedrals = read_elements::<4>(&dihedrals);
    }

    pub fn setCells(&mut self, cells: Vec<u32>) {
        self.0.cells = read_elements::<{ DIM + 1 }>(&cells);
    }

    pub fn setSurface(&mut self, surface: Vec<u32>) {
        self.0.surface = read_elements::<DIM>(&surface);
    }

    pub fn setSkin(&mut self, vertices: Vec<f32>, indices: Vec<u32>) {
        self.0.skin = Some((read_vectors(&vertices), read_elements::<DIM>(&indices)));
    }

    pub fn setSkinCollision(&mut self, enabled: bool) {
        self.0.skin_collision = enabled;
    }

    #[cfg(feature = "dim3")]
    pub fn setWire(&mut self, segments: Vec<u32>) {
        self.0.wire = read_elements::<2>(&segments);
    }

    pub fn setMaterial(&mut self, material: &RawSoftBodyMaterial) {
        self.0.material = material.0.clone();
    }

    pub fn material(&self) -> RawSoftBodyMaterial {
        RawSoftBodyMaterial(self.0.material.clone())
    }

    pub fn setSoftness(&mut self, natural_frequency: f32, damping_ratio: f32) {
        let me = std::mem::take(&mut self.0);
        self.0 = me.softness(SpringCoefficients::new(natural_frequency, damping_ratio));
    }

    pub fn setEdgeSoftness(&mut self, edges: Vec<u32>, frequencies: Vec<f32>, dampings: Vec<f32>) {
        self.0.edge_softness = edges
            .iter()
            .zip(frequencies.iter().zip(dampings.iter()))
            .map(|(e, (f, d))| (*e, SpringCoefficients::new(*f, *d)))
            .collect();
    }

    pub fn setEdgeTearResistance(&mut self, edges: Vec<u32>, resistances: Vec<f32>) {
        self.0.edge_tear_resistance = edges.into_iter().zip(resistances).collect();
    }

    pub fn setCellModel(&mut self, model: RawSoftBodyCellModel) {
        self.0.cell_model = model.into();
    }

    pub fn setVolumePreservation(&mut self, enabled: bool) {
        self.0.volume_preservation = enabled;
    }

    pub fn setVolumeFactor(&mut self, factor: f32) {
        let me = std::mem::take(&mut self.0);
        self.0 = me.volume_factor(factor);
    }

    pub fn setShapeMatching(&mut self, enabled: bool) {
        self.0.shape_matching = enabled;
    }

    pub fn setSelfContacts(&mut self, enabled: bool) {
        self.0.self_contacts = enabled;
    }

    pub fn setParticleRadius(&mut self, radius: f32) {
        self.0.particle_radius = radius;
    }

    /// The template of the body's colliders: its shape is replaced by the deformable surface,
    /// or by a ball of `particleRadius` for a body colliding through its particles.
    pub fn setSurfaceCollider(
        &mut self,
        friction: f32,
        restitution: f32,
        frictionCombineRule: u32,
        restitutionCombineRule: u32,
        isSensor: bool,
        collisionGroups: u32,
        solverGroups: u32,
        activeCollisionTypes: u16,
        activeHooks: u32,
        activeEvents: u32,
        contactForceEventThreshold: f32,
        contactSkin: f32,
    ) {
        use rapier::geometry::ActiveCollisionTypes;
        use rapier::pipeline::{ActiveEvents, ActiveHooks};
        let radius = self.0.particle_radius.max(0.05);
        self.0.collider_template = Some(
            ColliderBuilder::ball(radius)
                .density(0.0)
                .friction(friction)
                .restitution(restitution)
                .friction_combine_rule(crate::geometry::combine_rule_from_u32(frictionCombineRule))
                .restitution_combine_rule(crate::geometry::combine_rule_from_u32(
                    restitutionCombineRule,
                ))
                .sensor(isSensor)
                .collision_groups(crate::geometry::unpack_interaction_groups(collisionGroups))
                .solver_groups(crate::geometry::unpack_interaction_groups(solverGroups))
                .active_collision_types(
                    ActiveCollisionTypes::from_bits(activeCollisionTypes)
                        .unwrap_or(ActiveCollisionTypes::empty()),
                )
                .active_hooks(ActiveHooks::from_bits(activeHooks).unwrap_or(ActiveHooks::empty()))
                .active_events(
                    ActiveEvents::from_bits(activeEvents).unwrap_or(ActiveEvents::empty()),
                )
                .contact_force_event_threshold(contactForceEventThreshold)
                .contact_skin(contactSkin),
        );
    }

    pub fn setNoSurfaceCollider(&mut self) {
        self.0.collider_template = None;
    }

    pub fn setLinearDamping(&mut self, damping: f32) {
        self.0.particle_settings.linear_damping = damping;
    }

    pub fn setGravityScale(&mut self, scale: f32) {
        self.0.particle_settings.gravity_scale = scale;
    }

    pub fn setAdditionalSolverIterations(&mut self, iterations: usize) {
        self.0.particle_settings.additional_solver_iterations = iterations;
    }

    pub fn setAdditionalPgsIterations(&mut self, iterations: usize) {
        self.0.particle_settings.additional_pgs_iterations = iterations;
    }

    pub fn setCanSleep(&mut self, can_sleep: bool) {
        self.0.particle_settings.can_sleep = can_sleep;
    }

    pub fn setDominanceGroup(&mut self, group: i8) {
        self.0.particle_settings.dominance_group = group;
    }
}

/*
 * Tear events.
 */

#[wasm_bindgen]
pub struct RawSoftBodyTearEvent(pub(crate) SoftBodyTearEvent);

#[wasm_bindgen]
impl RawSoftBodyTearEvent {
    pub fn softBody(&self) -> FlatHandle {
        utils::flat_handle(self.0.soft_body.0)
    }

    pub fn tornEdges(&self) -> Vec<u32> {
        flatten_elements(&self.0.torn_edges)
    }

    pub fn tornCells(&self) -> Vec<u32> {
        flatten_elements(&self.0.torn_cells)
    }

    pub fn removedEdges(&self) -> Vec<u32> {
        flatten_elements(&self.0.removed_edges)
    }

    pub fn splitParticles(&self) -> Vec<u32> {
        self.0
            .split_particles
            .iter()
            .flat_map(|(a, b)| [*a, *b])
            .collect()
    }

    pub fn insertedParticles(&self) -> Vec<u32> {
        self.0.inserted_particles.clone()
    }

    pub fn seeds(&self) -> Vec<u32> {
        flatten_elements(&self.0.seeds())
    }

    pub fn numPieces(&self) -> usize {
        self.0.pieces.len()
    }

    pub fn pieceSoftBody(&self, i: usize) -> FlatHandle {
        utils::flat_handle(self.0.pieces[i].soft_body.0)
    }

    pub fn pieceParticles(&self, i: usize) -> Vec<u32> {
        self.0.pieces[i].particles.clone()
    }

    pub fn pieceClusters(&self, i: usize) -> Vec<u32> {
        flatten_elements(&self.0.pieces[i].clusters)
    }

    pub fn numClusterSplits(&self) -> usize {
        self.0.clusters.len()
    }

    pub fn clusterSplitSource(&self, i: usize) -> u32 {
        self.0.clusters[i].source_cluster
    }

    pub fn clusterSplitSoftBody(&self, i: usize) -> FlatHandle {
        utils::flat_handle(self.0.clusters[i].soft_body.0)
    }

    pub fn clusterSplitCluster(&self, i: usize) -> u32 {
        self.0.clusters[i].cluster
    }

    pub fn clusterSplitProxy(&self, i: usize) -> FlatHandle {
        utils::flat_handle(self.0.clusters[i].proxy.0)
    }

    pub fn clusterSplitKeepsProxy(&self, i: usize) -> bool {
        self.0.clusters[i].keeps_proxy
    }

    pub fn numMovedJoints(&self) -> usize {
        self.0.moved_joints.len()
    }

    pub fn movedJoint(&self, i: usize) -> FlatHandle {
        utils::flat_handle(self.0.moved_joints[i].joint.0)
    }

    pub fn movedJointFrom(&self, i: usize) -> FlatHandle {
        utils::flat_handle(self.0.moved_joints[i].from.0)
    }

    pub fn movedJointTo(&self, i: usize) -> FlatHandle {
        utils::flat_handle(self.0.moved_joints[i].to.0)
    }

    pub fn particleDestinationBody(&self, particle: u32) -> Option<FlatHandle> {
        self.0
            .particle_destination(particle)
            .map(|(body, _)| utils::flat_handle(body.0))
    }

    pub fn particleDestinationIndex(&self, particle: u32) -> Option<u32> {
        self.0.particle_destination(particle).map(|(_, i)| i)
    }
}

/*
 * Set.
 */

#[wasm_bindgen]
pub struct RawSoftBodySet(pub(crate) SoftBodySet);

impl RawSoftBodySet {
    pub(crate) fn map<T>(&self, handle: FlatHandle, f: impl FnOnce(&SoftBody) -> T) -> T {
        let body = self
            .0
            .get(utils::soft_body_handle(handle))
            .expect("Invalid SoftBody reference. It may have been removed from the physics World.");
        f(body)
    }

    pub(crate) fn map_mut<T>(
        &mut self,
        handle: FlatHandle,
        f: impl FnOnce(&mut SoftBody) -> T,
    ) -> T {
        let body = self
            .0
            .get_mut(utils::soft_body_handle(handle))
            .expect("Invalid SoftBody reference. It may have been removed from the physics World.");
        f(body)
    }
}

#[wasm_bindgen]
impl RawSoftBodySet {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawSoftBodySet(SoftBodySet::new())
    }

    /// Inserts the soft body described by the builder, creating its hidden root rigid body and
    /// its colliders.
    pub fn insert(
        &mut self,
        builder: &RawSoftBodyBuilder,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
    ) -> FlatHandle {
        let handle = self
            .0
            .insert(builder.0.clone(), &mut bodies.0, &mut colliders.0);
        utils::flat_handle(handle.0)
    }

    pub fn remove(
        &mut self,
        handle: FlatHandle,
        islands: &mut RawIslandManager,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
    ) {
        self.0.remove(
            utils::soft_body_handle(handle),
            &mut islands.0,
            &mut bodies.0,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
        );
    }

    /// The number of soft bodies on this set.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Checks if a soft body with the given integer handle exists.
    pub fn contains(&self, handle: FlatHandle) -> bool {
        self.0.contains(utils::soft_body_handle(handle))
    }

    /// Applies the given JavaScript function to the integer handle of each soft body managed by
    /// this set.
    pub fn forEachSoftBodyHandle(&self, f: &js_sys::Function) {
        let this = JsValue::null();
        for (handle, _) in self.0.iter() {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }

    pub fn addCluster(
        &mut self,
        handle: FlatHandle,
        particles: Vec<u32>,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
    ) -> Option<u32> {
        self.0.add_cluster(
            utils::soft_body_handle(handle),
            &particles,
            &mut bodies.0,
            &mut colliders.0,
        )
    }

    pub fn removeCluster(
        &mut self,
        handle: FlatHandle,
        cluster: u32,
        islands: &mut RawIslandManager,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
    ) -> bool {
        self.0
            .remove_cluster(
                utils::soft_body_handle(handle),
                cluster,
                &mut islands.0,
                &mut bodies.0,
                &mut colliders.0,
                &mut joints.0,
                &mut articulations.0,
            )
            .is_some()
    }

    pub fn tear(
        &mut self,
        handle: FlatHandle,
        edges: Vec<u32>,
        cells: Vec<u32>,
        islands: &mut RawIslandManager,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
    ) -> Option<RawSoftBodyTearEvent> {
        self.0
            .tear(
                utils::soft_body_handle(handle),
                &edges,
                &cells,
                &mut islands.0,
                &mut bodies.0,
                &mut colliders.0,
                &mut joints.0,
                &mut articulations.0,
            )
            .map(RawSoftBodyTearEvent)
    }

    /// Cuts a soft body along a blade: a segment (two points) in 2D, a triangle (three points)
    /// in 3D, given as `DIM` floats per point.
    pub fn cut(
        &mut self,
        handle: FlatHandle,
        blade: Vec<f32>,
        islands: &mut RawIslandManager,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
    ) -> Option<RawSoftBodyTearEvent> {
        let points = read_vectors(&blade);
        if points.len() != DIM {
            return None;
        }
        let mut blade = [Vector::ZERO; DIM];
        blade.copy_from_slice(&points);
        self.0
            .cut(
                utils::soft_body_handle(handle),
                &blade,
                &mut islands.0,
                &mut bodies.0,
                &mut colliders.0,
                &mut joints.0,
                &mut articulations.0,
            )
            .map(RawSoftBodyTearEvent)
    }

    pub fn wakeUp(&mut self, handle: FlatHandle, bodies: &mut RawRigidBodySet, strong: bool) {
        self.0
            .wake_up(utils::soft_body_handle(handle), &mut bodies.0, strong);
    }

    /*
     * Particles.
     */

    pub fn sbTopologyVersion(&self, handle: FlatHandle) -> u32 {
        self.map(handle, |sb| sb.topology_version())
    }

    pub fn sbNumParticles(&self, handle: FlatHandle) -> usize {
        self.map(handle, |sb| sb.num_particles())
    }

    pub fn sbParticlePosition(
        &self,
        handle: FlatHandle,
        i: usize,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        self.map(handle, |sb| {
            write_vector(scratch_buffer, sb.particle_position(i))
        })
    }

    pub fn sbParticlePositions(&self, handle: FlatHandle) -> Vec<f32> {
        self.map(handle, |sb| {
            sb.particle_positions().flat_map(|p| p.to_array()).collect()
        })
    }

    pub fn sbParticleVelocity(
        &self,
        handle: FlatHandle,
        i: usize,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        self.map(handle, |sb| {
            write_vector(scratch_buffer, sb.particle_velocity(i))
        })
    }

    pub fn sbParticleVelocities(&self, handle: FlatHandle) -> Vec<f32> {
        self.map(handle, |sb| {
            sb.particle_velocities()
                .flat_map(|v| v.to_array())
                .collect()
        })
    }

    pub fn sbParticleRestPosition(
        &self,
        handle: FlatHandle,
        i: usize,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        self.map(handle, |sb| {
            write_vector(scratch_buffer, sb.particles()[i].rest_position())
        })
    }

    pub fn sbParticleMass(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.particles()[i].mass())
    }

    pub fn sbIsParticlePinned(&self, handle: FlatHandle, i: usize) -> bool {
        self.map(handle, |sb| sb.particles()[i].is_pinned())
    }

    pub fn sbIsParticleOnSurface(&self, handle: FlatHandle, i: usize) -> bool {
        self.map(handle, |sb| sb.particles()[i].is_on_surface())
    }

    pub fn sbIsParticleDamaged(&self, handle: FlatHandle, i: usize) -> bool {
        self.map(handle, |sb| sb.particles()[i].is_damaged())
    }

    pub fn sbSetParticlePosition(&mut self, handle: FlatHandle, i: usize, position: &RawVector) {
        self.map_mut(handle, |sb| sb.set_particle_position(i, position.0))
    }

    pub fn sbSetParticleVelocity(&mut self, handle: FlatHandle, i: usize, velocity: &RawVector) {
        self.map_mut(handle, |sb| sb.set_particle_velocity(i, velocity.0))
    }

    pub fn sbSetParticleKinematicTarget(
        &mut self,
        handle: FlatHandle,
        i: usize,
        position: &RawVector,
    ) {
        self.map_mut(handle, |sb| sb.set_particle_kinematic_target(i, position.0))
    }

    pub fn sbSetParticlePinned(&mut self, handle: FlatHandle, i: usize, pinned: bool) {
        self.map_mut(handle, |sb| sb.set_particle_pinned(i, pinned))
    }

    pub fn sbAttachParticle(
        &mut self,
        handle: FlatHandle,
        i: usize,
        body: FlatHandle,
        bodies: &RawRigidBodySet,
    ) {
        self.map_mut(handle, |sb| {
            sb.attach_particle(i, utils::body_handle(body), &bodies.0)
        })
    }

    pub fn sbDetachParticle(&mut self, handle: FlatHandle, i: usize) -> bool {
        self.map_mut(handle, |sb| sb.detach_particle(i))
    }

    pub fn sbNumAttachments(&self, handle: FlatHandle) -> usize {
        self.map(handle, |sb| sb.particle_attachments().len())
    }

    pub fn sbAttachmentParticle(&self, handle: FlatHandle, i: usize) -> u32 {
        self.map(handle, |sb| sb.particle_attachments()[i].particle)
    }

    pub fn sbAttachmentBody(&self, handle: FlatHandle, i: usize) -> FlatHandle {
        self.map(handle, |sb| {
            utils::flat_handle(sb.particle_attachments()[i].body.0)
        })
    }

    /*
     * Elements.
     */

    pub fn sbNumEdges(&self, handle: FlatHandle) -> usize {
        self.map(handle, |sb| sb.edges().len())
    }

    /// The particle pairs of every edge, two indices per edge.
    pub fn sbEdges(&self, handle: FlatHandle) -> Vec<u32> {
        self.map(handle, |sb| {
            sb.edges().iter().flat_map(|e| e.vertices).collect()
        })
    }

    pub fn sbEdgeRestLength(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.edges()[i].rest_length)
    }

    pub fn sbEdgeIsBend(&self, handle: FlatHandle, i: usize) -> bool {
        self.map(handle, |sb| {
            sb.edges()[i].kind == rapier::dynamics::SoftBodyEdgeKind::Bend
        })
    }

    pub fn sbEdgeImpulse(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.edges()[i].impulse())
    }

    pub fn sbEdgeStress(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.edges()[i].stress())
    }

    pub fn sbEdgePlasticStrain(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.edges()[i].plastic_strain())
    }

    pub fn sbEdgeTearResistance(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.edges()[i].tear_resistance)
    }

    pub fn sbNumCells(&self, handle: FlatHandle) -> usize {
        self.map(handle, |sb| sb.cells().len())
    }

    /// The particles of every cell, `DIM + 1` indices per cell.
    pub fn sbCells(&self, handle: FlatHandle) -> Vec<u32> {
        self.map(handle, |sb| {
            sb.cells().iter().flat_map(|c| c.vertices).collect()
        })
    }

    pub fn sbCellRestVolume(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.cells()[i].rest_volume)
    }

    pub fn sbCellStress(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.cells()[i].stress())
    }

    pub fn sbCellStiffnessScale(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.cells()[i].stiffness_scale)
    }

    pub fn sbCellTearResistance(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.cells()[i].tear_resistance)
    }

    #[cfg(feature = "dim3")]
    pub fn sbNumDihedrals(&self, handle: FlatHandle) -> usize {
        self.map(handle, |sb| sb.dihedrals().len())
    }

    /// The particles of every dihedral (3D), four indices per dihedral.
    #[cfg(feature = "dim3")]
    pub fn sbDihedrals(&self, handle: FlatHandle) -> Vec<u32> {
        self.map(handle, |sb| {
            sb.dihedrals().iter().flat_map(|d| d.vertices).collect()
        })
    }

    #[cfg(feature = "dim3")]
    pub fn sbDihedralRestAngle(&self, handle: FlatHandle, i: usize) -> f32 {
        self.map(handle, |sb| sb.dihedrals()[i].rest_angle)
    }

    /// The boundary elements of the body (segments in 2D, triangles in 3D), `DIM` particle
    /// indices per element.
    pub fn sbBoundary(&self, handle: FlatHandle) -> Vec<u32> {
        self.map(handle, |sb| flatten_elements(sb.boundary()))
    }

    /*
     * Material and models.
     */

    pub fn sbMaterial(&self, handle: FlatHandle) -> RawSoftBodyMaterial {
        self.map(handle, |sb| RawSoftBodyMaterial(sb.material().clone()))
    }

    pub fn sbSetMaterial(&mut self, handle: FlatHandle, material: &RawSoftBodyMaterial) {
        self.map_mut(handle, |sb| sb.set_material(material.0.clone()))
    }

    pub fn sbCellModel(&self, handle: FlatHandle) -> RawSoftBodyCellModel {
        self.map(handle, |sb| sb.cell_model().into())
    }

    pub fn sbVolumePreservationEnabled(&self, handle: FlatHandle) -> bool {
        self.map(handle, |sb| sb.volume_preservation_enabled())
    }

    pub fn sbEnableVolumePreservation(&mut self, handle: FlatHandle, enabled: bool) {
        self.map_mut(handle, |sb| sb.enable_volume_preservation(enabled))
    }

    pub fn sbRestVolume(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |sb| sb.rest_volume())
    }

    pub fn sbVolume(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |sb| sb.volume())
    }

    pub fn sbVolumeFactor(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |sb| sb.volume_factor())
    }

    pub fn sbSetVolumeFactor(&mut self, handle: FlatHandle, factor: f32) {
        self.map_mut(handle, |sb| sb.set_volume_factor(factor))
    }

    pub fn sbParticleRadius(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |sb| sb.particle_radius())
    }

    pub fn sbResetPlasticity(&mut self, handle: FlatHandle) {
        self.map_mut(handle, |sb| sb.reset_plasticity())
    }

    /*
     * Whole-body state.
     */

    pub fn sbRootBody(&self, handle: FlatHandle) -> FlatHandle {
        self.map(handle, |sb| utils::flat_handle(sb.root_body().0))
    }

    pub fn sbOrigin(&self, handle: FlatHandle) -> Option<FlatHandle> {
        self.map(handle, |sb| sb.origin().map(|h| utils::flat_handle(h.0)))
    }

    pub fn sbPieces(&self, handle: FlatHandle) -> Vec<f64> {
        self.map(handle, |sb| {
            sb.pieces()
                .iter()
                .map(|h| utils::flat_handle(h.0))
                .collect()
        })
    }

    pub fn sbCenterOfMass(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |sb| {
            write_vector(scratch_buffer, sb.center_of_mass())
        })
    }

    pub fn sbMass(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |sb| sb.mass())
    }

    pub fn sbIsSleeping(&self, handle: FlatHandle) -> bool {
        self.map(handle, |sb| sb.is_sleeping())
    }

    pub fn sbWakeUp(&mut self, handle: FlatHandle) {
        self.map_mut(handle, |sb| sb.wake_up())
    }

    pub fn sbIsEnabled(&self, handle: FlatHandle) -> bool {
        self.map(handle, |sb| sb.is_enabled())
    }

    pub fn sbSetEnabled(&mut self, handle: FlatHandle, enabled: bool) {
        self.map_mut(handle, |sb| sb.set_enabled(enabled))
    }

    pub fn sbSetAdditionalPgsIterations(&mut self, handle: FlatHandle, iterations: usize) {
        self.map_mut(handle, |sb| sb.set_additional_pgs_iterations(iterations))
    }

    pub fn sbLinearDamping(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |sb| sb.particle_settings().linear_damping)
    }

    pub fn sbGravityScale(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |sb| sb.particle_settings().gravity_scale)
    }

    pub fn sbUserData(&self, handle: FlatHandle) -> f64 {
        self.map(handle, |sb| sb.user_data as f64)
    }

    pub fn sbSetUserData(&mut self, handle: FlatHandle, data: f64) {
        self.map_mut(handle, |sb| sb.user_data = data as u128)
    }

    /*
     * Forces and impulses.
     */

    pub fn sbAddForce(&mut self, handle: FlatHandle, force: &RawVector, wake_up: bool) {
        self.map_mut(handle, |sb| sb.add_force(force.0, wake_up))
    }

    pub fn sbAddParticleForce(
        &mut self,
        handle: FlatHandle,
        i: usize,
        force: &RawVector,
        wake_up: bool,
    ) {
        self.map_mut(handle, |sb| sb.add_particle_force(i, force.0, wake_up))
    }

    pub fn sbResetForces(&mut self, handle: FlatHandle, wake_up: bool) {
        self.map_mut(handle, |sb| sb.reset_forces(wake_up))
    }

    pub fn sbApplyImpulse(&mut self, handle: FlatHandle, impulse: &RawVector, wake_up: bool) {
        self.map_mut(handle, |sb| sb.apply_impulse(impulse.0, wake_up))
    }

    pub fn sbApplyParticleImpulse(
        &mut self,
        handle: FlatHandle,
        i: usize,
        impulse: &RawVector,
        wake_up: bool,
    ) {
        self.map_mut(handle, |sb| {
            sb.apply_particle_impulse(i, impulse.0, wake_up)
        })
    }

    pub fn sbApplyImpulseAtPoint(
        &mut self,
        handle: FlatHandle,
        impulse: &RawVector,
        point: &RawVector,
        falloff_radius: f32,
        wake_up: bool,
    ) {
        self.map_mut(handle, |sb| {
            sb.apply_impulse_at_point(impulse.0, point.0, falloff_radius, wake_up)
        })
    }

    pub fn sbApplyRadialImpulse(
        &mut self,
        handle: FlatHandle,
        center: &RawVector,
        magnitude: f32,
        falloff_radius: f32,
        wake_up: bool,
    ) {
        self.map_mut(handle, |sb| {
            sb.apply_radial_impulse(center.0, magnitude, falloff_radius, wake_up)
        })
    }

    /*
     * Tearing.
     */

    pub fn sbTearEdge(&mut self, handle: FlatHandle, i: usize) {
        self.map_mut(handle, |sb| sb.tear_edge(i))
    }

    pub fn sbTearCell(&mut self, handle: FlatHandle, i: usize) {
        self.map_mut(handle, |sb| sb.tear_cell(i))
    }

    pub fn sbHasPendingTears(&self, handle: FlatHandle) -> bool {
        self.map(handle, |sb| sb.has_pending_tears())
    }

    /*
     * Clusters.
     */

    pub fn sbNumClusters(&self, handle: FlatHandle) -> usize {
        self.map(handle, |sb| sb.clusters().len())
    }

    pub fn sbIsClusterLive(&self, handle: FlatHandle, i: u32) -> bool {
        self.map(handle, |sb| sb.cluster(i).is_some_and(|c| c.is_live()))
    }

    pub fn sbClusterProxy(&self, handle: FlatHandle, i: u32) -> Option<FlatHandle> {
        self.map(handle, |sb| {
            sb.cluster_proxy(i).map(|h| utils::flat_handle(h.0))
        })
    }

    pub fn sbClusterParticles(&self, handle: FlatHandle, i: u32) -> Vec<u32> {
        self.map(handle, |sb| {
            sb.cluster(i)
                .map(|c| c.particles().to_vec())
                .unwrap_or_default()
        })
    }

    pub fn sbClusterShapeMatchingEnabled(&self, handle: FlatHandle, i: u32) -> bool {
        self.map(handle, |sb| {
            sb.cluster(i).is_some_and(|c| c.shape_matching_enabled())
        })
    }

    pub fn sbEnableClusterShapeMatching(&mut self, handle: FlatHandle, i: u32, enabled: bool) {
        self.map_mut(handle, |sb| sb.enable_cluster_shape_matching(i, enabled))
    }

    pub fn sbSetClusterStiffnessScale(&mut self, handle: FlatHandle, i: u32, scale: f32) {
        self.map_mut(handle, |sb| sb.set_cluster_stiffness_scale(i, scale))
    }

    pub fn sbSetClusterEdgeSoftness(
        &mut self,
        handle: FlatHandle,
        i: u32,
        natural_frequency: Option<f32>,
        damping_ratio: Option<f32>,
    ) {
        let softness = match (natural_frequency, damping_ratio) {
            (Some(f), Some(d)) => Some(SpringCoefficients::new(f, d)),
            _ => None,
        };
        self.map_mut(handle, |sb| sb.set_cluster_edge_softness(i, softness))
    }

    pub fn sbSetClusterTearResistance(&mut self, handle: FlatHandle, i: u32, resistance: f32) {
        self.map_mut(handle, |sb| sb.set_cluster_tear_resistance(i, resistance))
    }

    pub fn sbSetClusterPinned(&mut self, handle: FlatHandle, i: u32, pinned: bool) {
        self.map_mut(handle, |sb| sb.set_cluster_pinned(i, pinned))
    }

    pub fn sbSetClusterKinematicTarget(
        &mut self,
        handle: FlatHandle,
        i: u32,
        translation: &RawVector,
        rotation: &RawRotation,
    ) {
        let pose = Pose::from_parts(translation.0, rotation.0);
        self.map_mut(handle, |sb| sb.set_cluster_kinematic_target(i, pose))
    }

    /*
     * Collision meshes, indexed in the order the body holds them.
     */

    pub fn sbNumMeshes(&self, handle: FlatHandle) -> usize {
        self.map(handle, |sb| sb.meshes().count())
    }

    pub fn sbMeshCluster(&self, handle: FlatHandle, i: usize) -> Option<u32> {
        self.map(handle, |sb| sb.meshes().nth(i).map(|m| m.id().cluster))
    }

    pub fn sbMeshCollider(&self, handle: FlatHandle, i: usize) -> Option<FlatHandle> {
        self.map(handle, |sb| {
            sb.meshes()
                .nth(i)
                .map(|m| utils::flat_handle(m.collider().0))
        })
    }

    pub fn sbMeshIsSkinned(&self, handle: FlatHandle, i: usize) -> bool {
        self.map(handle, |sb| {
            sb.meshes().nth(i).is_some_and(|m| m.is_skinned())
        })
    }

    pub fn sbMeshCollisionEnabled(&self, handle: FlatHandle, i: usize) -> bool {
        self.map(handle, |sb| {
            sb.meshes().nth(i).is_some_and(|m| m.collision_enabled())
        })
    }

    /// The world-space vertex positions of a collision mesh, `DIM` floats per vertex.
    pub fn sbMeshVertices(&self, handle: FlatHandle, i: usize) -> Vec<f32> {
        self.map(handle, |sb| {
            sb.meshes()
                .nth(i)
                .map(|m| m.vertex_positions(sb).flat_map(|v| v.to_array()).collect())
                .unwrap_or_default()
        })
    }

    /// The elements of a collision mesh, `DIM` vertex indices per element.
    pub fn sbMeshIndices(&self, handle: FlatHandle, i: usize) -> Vec<u32> {
        self.map(handle, |sb| {
            sb.meshes()
                .nth(i)
                .map(|m| flatten_elements(m.indices()))
                .unwrap_or_default()
        })
    }

    /// The index, in the body's mesh list, of the mesh a deformable collider holds.
    pub fn sbMeshOfCollider(&self, handle: FlatHandle, collider: FlatHandle) -> Option<usize> {
        let collider = utils::collider_handle(collider);
        self.map(handle, |sb| {
            sb.meshes().position(|m| m.collider() == collider)
        })
    }
}

impl Default for RawSoftBodySet {
    fn default() -> Self {
        Self::new()
    }
}
