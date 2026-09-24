//! Soft-body recipes and borrowed topology. All arrays are copied during insertion.
#![allow(non_snake_case)]
use crate::*;
pub const RPR_SOFT_DESC_PARTICLES: u32 = 0;
pub const RPR_SOFT_DESC_ROPE: u32 = 1;
pub const RPR_SOFT_DESC_GRID: u32 = 2;
pub const RPR_SOFT_DESC_CLOTH: u32 = 3;
pub const RPR_SOFT_DESC_CUBOID: u32 = 4;
pub const RPR_SOFT_DESC_SURFACE: u32 = 5;
pub const RPR_SOFT_DESC_DISK: u32 = 6;
pub const RPR_SOFT_DESC_SPHERE: u32 = 7;
pub const RPR_SOFT_DESC_CLOTH_TUBE: u32 = 8;
pub const RPR_SOFT_DESC_VOLUMETRIC: u32 = 9;
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftEdgeSoftness {
    pub edge: u32,
    pub softness: RprSpringCoefficients,
}
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftEdgeTear {
    pub edge: u32,
    pub resistance: RprReal,
}
/// Copyable recipe, not an owned procedural builder. Initialize before editing.
/// All array views borrow caller data until build/insert returns; counts
/// for topology arrays are element counts (edges, triangles, or tetrahedra).
/// Nonempty topology overrides the generator's topology. Zero counts retain it.
/// Generator inputs: a/b are rope ends or center/half-extents; cloth uses a/du/dv.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RprSoftBodyDesc {
    pub kind: u32,
    pub a: RprVector,
    pub b: RprVector,
    pub du: RprVector,
    pub dv: RprVector,
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    pub radius: RprReal,
    pub radiusEnd: RprReal,
    pub translation: RprVector,
    pub totalMass: RprOptionalReal,
    pub meshing: RprVolumeMeshParameters,
    pub positions: RprVectorView,
    pub masses: RprRealView,
    pub pinned: RprIndexView,
    pub edges: RprEdgeView,
    pub bendEdges: RprEdgeView,
    pub tensionOnlyEdges: RprIndexView,
    pub edgeSoftness: RprSoftEdgeSoftnessView,
    pub edgeTearResistance: RprSoftEdgeTearView,
    pub cells: RprCellView,
    pub surface: RprSurfaceElementView,
    #[cfg(feature = "dim3")]
    pub dihedrals: RprDihedralView,
    #[cfg(feature = "dim3")]
    pub wire: RprEdgeView,
    pub skinVertices: RprVectorView,
    pub skinIndices: RprSurfaceElementView,
    pub material: RprSoftBodyMaterial,
    pub cellModel: u32,
    /// 0 = constraints, 1 = FEM (requires a library built with FEM).
    pub solver: u32,
    pub particleMass: RprReal,
    /// Disabled by default: retain the radius computed by the generator.
    pub particleRadius: RprOptionalReal,
    pub volumePreservation: RprBool,
    pub volumeFactor: RprReal,
    pub shapeMatching: RprOptionalBool,
    pub selfContacts: RprBool,
    pub skinCollision: RprBool,
    pub collisionEnabled: RprBool,
    pub collider: RprColliderDesc,
    pub linearDamping: RprReal,
    pub gravityScale: RprReal,
    pub additionalSolverIterations: usize,
    pub additionalPgsIterations: usize,
    pub canSleep: RprBool,
    pub dominanceGroup: i8,
    pub userData: RprUserData,
}
impl Default for RprSoftBodyDesc {
    fn default() -> Self {
        let settings = rapier::dynamics::SoftBodyParticleSettings::default();
        let mut collider = RprColliderDesc::default();
        collider.shape.radius = 0.05;
        collider.density = 0.0;
        let meshing = rapier::parry::transformation::VolumeMeshParameters::new(0.1);
        Self {
            kind: RPR_SOFT_DESC_PARTICLES,
            radius: 0.5,
            radiusEnd: 0.5,
            translation: Vector::ZERO.into(),
            totalMass: RprOptionalReal::default(),
            meshing: RprVolumeMeshParameters {
                cell_size: meshing.cell_size,
                #[cfg(feature = "dim2")]
                min_angle: meshing.min_angle,
                #[cfg(feature = "dim3")]
                enclosure: 0,
                #[cfg(feature = "dim3")]
                cover_smoothing: meshing.cover_smoothing,
                #[cfg(feature = "dim3")]
                cover_guard: meshing.cover_guard,
                #[cfg(feature = "dim3")]
                cover_subdivisions: meshing.cover_subdivisions,
            },
            a: Vector::ZERO.into(),
            b: Vector::ONE.into(),
            du: Vector::X.into(),
            dv: Vector::Y.into(),
            nx: 2,
            ny: 2,
            nz: 2,
            positions: RprVectorView::default(),
            masses: RprRealView::default(),
            pinned: RprIndexView::default(),
            edges: RprEdgeView::default(),
            bendEdges: RprEdgeView::default(),
            tensionOnlyEdges: RprIndexView::default(),
            edgeSoftness: RprSoftEdgeSoftnessView::default(),
            edgeTearResistance: RprSoftEdgeTearView::default(),
            cells: RprCellView::default(),
            surface: RprSurfaceElementView::default(),
            #[cfg(feature = "dim3")]
            dihedrals: RprDihedralView::default(),
            #[cfg(feature = "dim3")]
            wire: RprEdgeView::default(),
            skinVertices: RprVectorView::default(),
            skinIndices: RprSurfaceElementView::default(),
            material: SoftBodyMaterial::default().into(),
            cellModel: match SoftBodyCellModel::default() {
                SoftBodyCellModel::Volume => 0,
                SoftBodyCellModel::Corotational => 1,
                SoftBodyCellModel::NeoHookean => 2,
            },
            solver: 0,
            particleMass: 1.0,
            particleRadius: RprOptionalReal::default(),
            volumePreservation: 0,
            volumeFactor: 1.0,
            shapeMatching: RprOptionalBool::default(),
            selfContacts: 0,
            skinCollision: 0,
            collisionEnabled: 1,
            collider,
            linearDamping: settings.linear_damping,
            gravityScale: settings.gravity_scale,
            additionalSolverIterations: settings.additional_solver_iterations,
            additionalPgsIterations: settings.additional_pgs_iterations,
            canSleep: settings.can_sleep as _,
            dominanceGroup: settings.dominance_group,
            userData: 0u128.into(),
        }
    }
}
impl RprSoftBodyDesc {
    pub(crate) unsafe fn raw(&self) -> Result<SoftBodyBuilder> {
        use crate::geometry::indices_array;
        let points = || {
            unsafe { input(self.positions.data, self.positions.count)? }
                .iter()
                .map(|v| v.raw())
                .collect::<Result<Vec<_>>>()
        };
        let mut b = match self.kind {
            RPR_SOFT_DESC_PARTICLES | RPR_SOFT_DESC_SURFACE => {
                ensure(
                    self.positions.count > 0 && self.positions.count <= u32::MAX as usize,
                    "invalid particle count",
                )?;
                let vertices = points()?;
                if self.kind == RPR_SOFT_DESC_PARTICLES {
                    SoftBodyBuilder::new(vertices)
                } else {
                    let idx = unsafe {
                        indices_array::<{ rapier::math::DIM }>(
                            self.surface.data.cast(),
                            self.surface.count,
                            vertices.len(),
                        )?
                    };
                    #[cfg(feature = "dim2")]
                    let b = SoftBodyBuilder::polyline(vertices, Some(idx));
                    #[cfg(feature = "dim3")]
                    let b = SoftBodyBuilder::trimesh(vertices, idx);
                    b.ok_or_else(|| invalid("invalid soft surface"))?
                }
            }
            #[cfg(feature = "dim2")]
            RPR_SOFT_DESC_DISK => {
                ensure(
                    (3..=1_000_000).contains(&self.nx),
                    "particle count out of range",
                )?;
                SoftBodyBuilder::disk(self.a.raw()?, positive(self.radius)?, self.nx)
            }
            #[cfg(feature = "dim3")]
            RPR_SOFT_DESC_SPHERE => {
                ensure(self.nx <= 6, "sphere subdivisions exceed 6")?;
                SoftBodyBuilder::sphere(self.a.raw()?, positive(self.radius)?, self.nx)
            }
            #[cfg(feature = "dim3")]
            RPR_SOFT_DESC_CLOTH_TUBE => {
                ensure(
                    self.nx
                        .max(3)
                        .checked_mul(self.ny.max(2))
                        .is_some_and(|n| n <= u32::MAX as usize),
                    "cloth tube too large",
                )?;
                SoftBodyBuilder::cloth_tube(
                    self.a.raw()?,
                    self.b.raw()?,
                    nonnegative(self.radius)?,
                    nonnegative(self.radiusEnd)?,
                    self.nx,
                    self.ny,
                )
            }
            RPR_SOFT_DESC_VOLUMETRIC => {
                let points = points()?;
                let indices = unsafe {
                    indices_array::<{ rapier::math::DIM }>(
                        self.surface.data.cast(),
                        self.surface.count,
                        points.len(),
                    )?
                };
                let p = &self.meshing;
                let mut params = rapier::parry::transformation::VolumeMeshParameters::new(
                    positive(p.cell_size)?,
                );
                #[cfg(feature = "dim2")]
                {
                    params.min_angle = nonnegative(p.min_angle)?;
                }
                #[cfg(feature = "dim3")]
                {
                    use rapier::parry::transformation::MeshEnclosure;
                    params.enclosure = match p.enclosure {
                        0 => MeshEnclosure::Cover,
                        1 => MeshEnclosure::Crust,
                        _ => return Err(invalid("unknown mesh enclosure")),
                    };
                    params.cover_smoothing = p.cover_smoothing;
                    params.cover_guard = nonnegative(p.cover_guard)?;
                    params.cover_subdivisions = p.cover_subdivisions;
                }
                SoftBodyBuilder::volumetric_with(&points, &indices, &params)
                    .ok_or_else(|| invalid("volume meshing failed"))?
            }
            RPR_SOFT_DESC_ROPE => {
                ensure(
                    (2..=u32::MAX as usize).contains(&self.nx),
                    "invalid rope count",
                )?;
                SoftBodyBuilder::rope(self.a.raw()?, self.b.raw()?, self.nx)
            }
            #[cfg(feature = "dim2")]
            RPR_SOFT_DESC_GRID => {
                self.check_grid(false)?;
                SoftBodyBuilder::grid(self.a.raw()?, self.b.raw()?, self.nx, self.ny)
            }
            #[cfg(feature = "dim3")]
            RPR_SOFT_DESC_CUBOID => {
                self.check_grid(true)?;
                SoftBodyBuilder::cuboid(self.a.raw()?, self.b.raw()?, self.nx, self.ny, self.nz)
            }
            #[cfg(feature = "dim3")]
            RPR_SOFT_DESC_CLOTH => {
                ensure(
                    self.nx >= 2
                        && self.ny >= 2
                        && self
                            .nx
                            .checked_mul(self.ny)
                            .is_some_and(|n| n <= u32::MAX as usize),
                    "invalid cloth size",
                )?;
                SoftBodyBuilder::cloth(
                    self.a.raw()?,
                    self.du.raw()?,
                    self.dv.raw()?,
                    self.nx,
                    self.ny,
                )
            }
            _ => return Err(invalid("unsupported soft-body recipe")),
        };
        let n = b.positions.len();
        if self.masses.count != 0 {
            ensure(self.masses.count == n, "particle masses length mismatch")?;
            b.masses = unsafe { input(self.masses.data, n)? }
                .iter()
                .map(|v| positive(*v))
                .collect::<Result<_>>()?;
        }
        b.pinned = unsafe { input(self.pinned.data, self.pinned.count)? }.to_vec();
        ensure(
            b.pinned.iter().all(|i| (*i as usize) < n),
            "pinned particle out of bounds",
        )?;
        if self.edges.count != 0 {
            b.edges = unsafe { indices_array::<2>(self.edges.data.cast(), self.edges.count, n)? };
        }
        if self.bendEdges.count != 0 {
            b.bend_edges =
                unsafe { indices_array::<2>(self.bendEdges.data.cast(), self.bendEdges.count, n)? };
        }
        if self.cells.count != 0 {
            b.cells = unsafe {
                indices_array::<{ rapier::math::DIM + 1 }>(
                    self.cells.data.cast(),
                    self.cells.count,
                    n,
                )?
            };
        }
        if self.surface.count != 0 && self.kind != RPR_SOFT_DESC_VOLUMETRIC {
            b.surface = unsafe {
                indices_array::<{ rapier::math::DIM }>(
                    self.surface.data.cast(),
                    self.surface.count,
                    n,
                )?
            };
        }
        #[cfg(feature = "dim3")]
        {
            if self.dihedrals.count != 0 {
                b.dihedrals = unsafe {
                    indices_array::<4>(self.dihedrals.data.cast(), self.dihedrals.count, n)?
                };
            }
            if self.wire.count != 0 {
                b.wire = unsafe { indices_array::<2>(self.wire.data.cast(), self.wire.count, n)? };
            }
        }
        let edge_count = b.edges.len() + b.bend_edges.len();
        b.tension_only_edges =
            unsafe { input(self.tensionOnlyEdges.data, self.tensionOnlyEdges.count)? }.to_vec();
        ensure(
            b.tension_only_edges
                .iter()
                .all(|i| (*i as usize) < edge_count),
            "tension edge out of bounds",
        )?;
        b.edge_softness = unsafe { input(self.edgeSoftness.data, self.edgeSoftness.count)? }
            .iter()
            .map(|v| {
                ensure(
                    (v.edge as usize) < edge_count,
                    "softness edge out of bounds",
                )?;
                Ok((v.edge, v.softness.raw()?))
            })
            .collect::<Result<_>>()?;
        b.edge_tear_resistance =
            unsafe { input(self.edgeTearResistance.data, self.edgeTearResistance.count)? }
                .iter()
                .map(|v| {
                    ensure((v.edge as usize) < edge_count, "tear edge out of bounds")?;
                    Ok((v.edge, nonnegative(v.resistance)?))
                })
                .collect::<Result<_>>()?;
        if self.skinVertices.count != 0 {
            let vertices = unsafe { input(self.skinVertices.data, self.skinVertices.count)? }
                .iter()
                .map(|v| v.raw())
                .collect::<Result<Vec<_>>>()?;
            let indices = unsafe {
                indices_array::<{ rapier::math::DIM }>(
                    self.skinIndices.data.cast(),
                    self.skinIndices.count,
                    vertices.len(),
                )?
            };
            b.skin = Some((vertices, indices));
        } else {
            ensure(self.skinIndices.count == 0, "skin indices without vertices")?;
        }
        b.material = self.material.raw()?;
        b.cell_model = match self.cellModel {
            0 => SoftBodyCellModel::Volume,
            1 => SoftBodyCellModel::Corotational,
            2 => SoftBodyCellModel::NeoHookean,
            _ => return Err(invalid("invalid cell model")),
        };
        #[cfg(feature = "fem")]
        {
            b.solver = match self.solver {
                0 => SoftBodySolver::Constraints,
                1 => SoftBodySolver::Fem,
                _ => return Err(invalid("unknown soft solver")),
            };
        }
        #[cfg(not(feature = "fem"))]
        ensure(self.solver == 0, "FEM support is not enabled")?;
        b.particle_mass = positive(self.particleMass)?;
        if boolean(self.particleRadius.enabled)? {
            b.particle_radius = nonnegative(self.particleRadius.value)?;
        }
        b.volume_preservation = boolean(self.volumePreservation)?;
        b.volume_factor = positive(self.volumeFactor)?;
        if boolean(self.shapeMatching.enabled)? {
            b.shape_matching = boolean(self.shapeMatching.value)?;
        }
        b.self_contacts = boolean(self.selfContacts)?;
        b.skin_collision = boolean(self.skinCollision)?;
        b.collider_template = if boolean(self.collisionEnabled)? {
            Some(unsafe { self.collider.raw()? })
        } else {
            None
        };
        b.particle_settings.linear_damping = nonnegative(self.linearDamping)?;
        b.particle_settings.gravity_scale = finite(self.gravityScale)?;
        b.particle_settings.additional_solver_iterations = self.additionalSolverIterations;
        b.particle_settings.additional_pgs_iterations = self.additionalPgsIterations;
        b.particle_settings.can_sleep = boolean(self.canSleep)?;
        b.particle_settings.dominance_group = self.dominanceGroup;
        b.user_data = self.userData.raw();
        if boolean(self.totalMass.enabled)? {
            b = b.mass(positive(self.totalMass.value)?);
        }
        b = b.translated(self.translation.raw()?);
        Ok(b)
    }
    fn check_grid(&self, dim3: bool) -> Result<()> {
        let count = self
            .nx
            .checked_add(1)
            .and_then(|x| self.ny.checked_add(1).and_then(|y| x.checked_mul(y)))
            .and_then(|n| {
                if dim3 {
                    self.nz.checked_add(1).and_then(|z| n.checked_mul(z))
                } else {
                    Some(n)
                }
            });
        ensure(
            self.nx > 0
                && self.ny > 0
                && (!dim3 || self.nz > 0)
                && count.is_some_and(|n| n <= u32::MAX as usize)
                && self.b.raw()?.min_element() > 0.0,
            "invalid soft grid size",
        )
    }
}
#[rapier_export]
pub extern "C" fn rpr_default_soft_body_desc() -> RprSoftBodyDesc {
    RprSoftBodyDesc::default()
}
/// Consumes no caller-owned resources. All borrowed arrays may be released on return.
#[rapier_export]
pub unsafe extern "C" fn rpr_insert_soft_body(
    world: *mut RprWorld,
    desc: *const RprSoftBodyDesc,
) -> RprSoftBodyHandle {
    ffi_world_value(world, |out: *mut RprSoftBodyHandle| {
        ffi(|| unsafe {
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            if !out.is_null() {
                out_ptr(out)?;
            }
            let builder = get(desc)?.raw()?;
            let h = get_mut(world)?.0.insert_soft_body(builder);
            if !out.is_null() {
                output(out, h.into())?;
            }
            Ok(())
        })
    })
}

pub const RPR_SOFT_BINDING_SKINNED: u32 = 0;
pub const RPR_SOFT_BINDING_DIRECT: u32 = 1;
pub const RPR_SOFT_BINDING_DIRECT_BY_POSITION: u32 = 2;
/// Non-owning deformable binding description. Direct particle indices are borrowed.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftMeshBindingDesc {
    pub kind: u32,
    pub particles: RprIndexView,
    pub epsilon: RprReal,
    pub selfContacts: RprBool,
}
impl RprSoftMeshBindingDesc {
    unsafe fn raw(&self) -> Result<SoftMeshBinding> {
        let b = match self.kind {
            RPR_SOFT_BINDING_SKINNED => SoftMeshBinding::skinned(),
            RPR_SOFT_BINDING_DIRECT => SoftMeshBinding::direct(
                unsafe { input(self.particles.data, self.particles.count)? }.to_vec(),
            ),
            RPR_SOFT_BINDING_DIRECT_BY_POSITION => {
                SoftMeshBinding::direct_by_position(nonnegative(self.epsilon)?)
            }
            _ => return Err(invalid("unknown soft binding")),
        };
        Ok(b.self_contacts(boolean(self.selfContacts)?))
    }
}
#[rapier_export]
pub extern "C" fn rpr_default_soft_mesh_binding_desc() -> RprSoftMeshBindingDesc {
    RprSoftMeshBindingDesc {
        kind: 0,
        particles: RprIndexView::default(),
        epsilon: 0.0,
        selfContacts: 0,
    }
}
#[rapier_export]
pub unsafe extern "C" fn rpr_insert_deformable_collider(
    collider: *const RprColliderDesc,
    binding: *const RprSoftMeshBindingDesc,
    parent: RprRigidBodyHandle,
) -> RprColliderHandle {
    let world = parent.world;
    ffi_world_value(world, |out: *mut RprColliderHandle| {
        ffi(|| unsafe {
            parent.check_world(world)?;
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            if !out.is_null() {
                out_ptr(out)?;
            }
            let collider = get(collider)?.raw()?;
            let binding = get(binding)?.raw()?;
            let w = &mut get_mut(world)?.0;
            w.bodies.get(parent.raw()).ok_or_else(missing)?;
            let handle = w
                .insert_deformable(collider, binding, parent.raw())
                .map_err(|e| invalid(e.to_string()))?;
            if !out.is_null() {
                output(out, handle.into())?;
            }
            Ok(())
        })
    })
}
