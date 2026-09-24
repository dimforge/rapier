//! Soft bodies: deformable bodies made of particles linked by elastic constraints, simulated
//! together with the rigid bodies, contacts and joints of a world.

use crate::conv::{PyIsometry, PyVector, Real};
use crate::dynamics::{IslandManager, RigidBodyHandle, RigidBodySet, SpringCoefficients};
use crate::geometry::{
    ColliderBuilder, ColliderHandle, ColliderSet, extract_index_list, extract_index_rows,
};
use crate::joints::{ImpulseJointHandle, ImpulseJointSet, MultibodyJointSet};
use crate::math::Vec3;
use crate::numpy::PyArray2;
use crate::pyo3::create_exception;
use crate::pyo3::exceptions::{PyIndexError, PyTypeError, PyValueError};
use crate::pyo3::prelude::*;
use crate::pyo3::types::PyDict;
use rapier3d as rapier;

create_exception!(
    rapier,
    SoftBindingError,
    crate::errors::RapierError,
    "A deformable collider could not be bound to a soft-body cluster."
);

// ----------------------------------------------------------------------
// Array helpers.
// ----------------------------------------------------------------------

/// An `(N, 3)` float ndarray from vectors.
pub(crate) fn vectors_to_array<'py>(
    py: Python<'py>,
    it: impl Iterator<Item = rapier::math::Vector>,
) -> Bound<'py, PyArray2<Real>> {
    let rows: Vec<Vec<Real>> = it.map(|v| v.to_array().to_vec()).collect();
    if rows.is_empty() {
        PyArray2::zeros_bound(py, [0, 3], false)
    } else {
        PyArray2::from_vec2_bound(py, &rows).expect("contiguous ndarray")
    }
}

/// An `(M, N)` uint32 ndarray from elements of `N` indices.
pub(crate) fn elements_to_array<'py, const N: usize>(
    py: Python<'py>,
    elements: &[[u32; N]],
) -> Bound<'py, PyArray2<u32>> {
    if elements.is_empty() {
        PyArray2::zeros_bound(py, [0, N], false)
    } else {
        let rows: Vec<Vec<u32>> = elements.iter().map(|e| e.to_vec()).collect();
        PyArray2::from_vec2_bound(py, &rows).expect("contiguous ndarray")
    }
}

fn vec3(v: rapier::math::Vector) -> Vec3 {
    let nav: crate::na::SVector<Real, 3> = v.into();
    Vec3(nav)
}

fn spring(obj: &Bound<'_, PyAny>) -> PyResult<rapier::dynamics::SpringCoefficients<Real>> {
    if let Ok(s) = obj.extract::<PyRef<'_, SpringCoefficients>>() {
        return Ok(s.0);
    }
    let (f, d): (Real, Real) = obj.extract().map_err(|_| {
        PyTypeError::new_err(
            "expected a SpringCoefficients or a (natural_frequency, damping_ratio) tuple",
        )
    })?;
    Ok(rapier::dynamics::SpringCoefficients::new(f, d))
}

// ----------------------------------------------------------------------
// Handle.
// ----------------------------------------------------------------------

/// Opaque identifier of a soft body stored in a :class:`SoftBodySet`.
#[pyclass(name = "SoftBodyHandle", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SoftBodyHandle(pub rapier::dynamics::SoftBodyHandle);

#[pymethods]
impl SoftBodyHandle {
    /// Rebuild a handle from its ``(index, generation)`` parts.
    #[staticmethod]
    fn from_raw_parts(index: u32, generation: u32) -> Self {
        Self(rapier::dynamics::SoftBodyHandle::from_raw_parts(
            index, generation,
        ))
    }
    /// The ``(index, generation)`` parts of the handle.
    fn into_raw_parts(&self) -> (u32, u32) {
        self.0.into_raw_parts()
    }
    /// Arena slot index.
    #[getter]
    fn index(&self) -> u32 {
        self.0.into_raw_parts().0
    }
    /// Arena slot generation.
    #[getter]
    fn generation(&self) -> u32 {
        self.0.into_raw_parts().1
    }
    /// An invalid handle, matching no soft body.
    #[staticmethod]
    fn invalid() -> Self {
        Self(rapier::dynamics::SoftBodyHandle::invalid())
    }
    fn __repr__(&self) -> String {
        let (i, g) = self.0.into_raw_parts();
        format!("SoftBodyHandle(index={i}, generation={g})")
    }
    fn __hash__(&self) -> u64 {
        let (i, g) = self.0.into_raw_parts();
        ((g as u64) << 32) | i as u64
    }
    fn __eq__(&self, other: &Self) -> bool {
        self.0 == other.0
    }
    /// Pickle support.
    fn __reduce__(slf: PyRef<'_, Self>) -> PyResult<(Py<PyAny>, (u32, u32))> {
        let py = slf.py();
        let cls = py.get_type_bound::<Self>();
        let ctor = cls.getattr("from_raw_parts")?;
        Ok((ctor.unbind(), slf.0.into_raw_parts()))
    }
}

// ----------------------------------------------------------------------
// Enums.
// ----------------------------------------------------------------------

/// The constitutive model of a soft body's cells (tetrahedra).
#[pyclass(
    name = "SoftBodyCellModel",
    module = "rapier",
    eq,
    eq_int,
    hash,
    frozen
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SoftBodyCellModel {
    /// Per-cell volume preservation constraints; the shape is held by the edges.
    VOLUME,
    /// Corotational linear elasticity: a linear material in the cell's rotated frame.
    COROTATIONAL,
    /// Neo-Hookean hyperelasticity, which resists inversion and large compressions.
    NEO_HOOKEAN,
}

impl SoftBodyCellModel {
    pub(crate) fn to_rapier(self) -> rapier::dynamics::SoftBodyCellModel {
        match self {
            Self::VOLUME => rapier::dynamics::SoftBodyCellModel::Volume,
            Self::COROTATIONAL => rapier::dynamics::SoftBodyCellModel::Corotational,
            Self::NEO_HOOKEAN => rapier::dynamics::SoftBodyCellModel::NeoHookean,
        }
    }
    pub(crate) fn from_rapier(m: rapier::dynamics::SoftBodyCellModel) -> Self {
        match m {
            rapier::dynamics::SoftBodyCellModel::Volume => Self::VOLUME,
            rapier::dynamics::SoftBodyCellModel::Corotational => Self::COROTATIONAL,
            rapier::dynamics::SoftBodyCellModel::NeoHookean => Self::NEO_HOOKEAN,
        }
    }
}

/// Which strains make a soft body's edge rest lengths flow plastically.
#[pyclass(
    name = "SoftEdgePlasticFlow",
    module = "rapier",
    eq,
    eq_int,
    hash,
    frozen
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SoftEdgePlasticFlow {
    BOTH,
    COMPRESSION,
    TENSION,
}

impl SoftEdgePlasticFlow {
    pub(crate) fn to_rapier(self) -> rapier::dynamics::SoftEdgePlasticFlow {
        match self {
            Self::BOTH => rapier::dynamics::SoftEdgePlasticFlow::Both,
            Self::COMPRESSION => rapier::dynamics::SoftEdgePlasticFlow::Compression,
            Self::TENSION => rapier::dynamics::SoftEdgePlasticFlow::Tension,
        }
    }
    pub(crate) fn from_rapier(f: rapier::dynamics::SoftEdgePlasticFlow) -> Self {
        match f {
            rapier::dynamics::SoftEdgePlasticFlow::Both => Self::BOTH,
            rapier::dynamics::SoftEdgePlasticFlow::Compression => Self::COMPRESSION,
            rapier::dynamics::SoftEdgePlasticFlow::Tension => Self::TENSION,
        }
    }
}

/// The kind of a soft body's edge.
#[pyclass(name = "SoftBodyEdgeKind", module = "rapier", eq, eq_int, hash, frozen)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SoftBodyEdgeKind {
    /// A structural edge holding the distance between two particles.
    STRUCTURAL,
    /// A bending edge across two elements, resisting folds.
    BEND,
}

/// What the soft-body tangle recovery does with the contact patches of crossed surfaces.
#[pyclass(
    name = "SoftPatchConstraints",
    module = "rapier",
    eq,
    eq_int,
    hash,
    frozen
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SoftPatchConstraints {
    KEEP,
    STAND_DOWN,
    ALONG_NORMAL,
}

impl SoftPatchConstraints {
    fn to_rapier(self) -> rapier::dynamics::SoftPatchConstraints {
        match self {
            Self::KEEP => rapier::dynamics::SoftPatchConstraints::Keep,
            Self::STAND_DOWN => rapier::dynamics::SoftPatchConstraints::StandDown,
            Self::ALONG_NORMAL => rapier::dynamics::SoftPatchConstraints::AlongNormal,
        }
    }
    fn from_rapier(c: rapier::dynamics::SoftPatchConstraints) -> Self {
        match c {
            rapier::dynamics::SoftPatchConstraints::Keep => Self::KEEP,
            rapier::dynamics::SoftPatchConstraints::StandDown => Self::STAND_DOWN,
            rapier::dynamics::SoftPatchConstraints::AlongNormal => Self::ALONG_NORMAL,
        }
    }
}

/// Which solver simulates a soft body's elasticity.
#[pyclass(name = "SoftBodySolver", module = "rapier", eq, eq_int, hash, frozen)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SoftBodySolver {
    /// Every elastic element becomes constraints, swept with the contacts and joints once per
    /// substep (the default). Cheap and robust, but converged only as far as the sweep count: a
    /// stiff body keeps a residual compliance, and a load crosses a long body slowly.
    CONSTRAINTS,
    /// Implicit Euler elasticity solved over the whole body once per substep: the stiffness does
    /// not depend on the solver iterations. Costs a factorization per step and a solve per
    /// constraint (see :class:`SoftFemParameters`).
    FEM,
}

impl SoftBodySolver {
    pub(crate) fn to_rapier(self) -> rapier::dynamics::SoftBodySolver {
        match self {
            Self::CONSTRAINTS => rapier::dynamics::SoftBodySolver::Constraints,
            Self::FEM => rapier::dynamics::SoftBodySolver::Fem,
        }
    }
    pub(crate) fn from_rapier(s: rapier::dynamics::SoftBodySolver) -> Self {
        match s {
            rapier::dynamics::SoftBodySolver::Constraints => Self::CONSTRAINTS,
            rapier::dynamics::SoftBodySolver::Fem => Self::FEM,
        }
    }
}

// ----------------------------------------------------------------------
// Volume meshing.
// ----------------------------------------------------------------------

/// Which cover a volume mesh is (see :class:`VolumeMeshParameters`): of the shape's volume, or
/// of its surface alone.
#[pyclass(name = "MeshEnclosure", module = "rapier", eq, eq_int, hash, frozen)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MeshEnclosure {
    /// Every lattice cell the shape reaches is kept whole: a solid fill containing the shape,
    /// blocky at the cell size (see :attr:`VolumeMeshParameters.cover_smoothing` and
    /// :attr:`VolumeMeshParameters.cover_subdivisions`). Needs a closed, consistently oriented
    /// mesh.
    COVER,
    /// Only the cells the surface crosses are kept: a hollow shell that deforms like a shell,
    /// not like a solid. The mesh need not be closed or oriented.
    CRUST,
}

impl MeshEnclosure {
    fn to_rapier(self) -> rapier::parry::transformation::MeshEnclosure {
        match self {
            Self::COVER => rapier::parry::transformation::MeshEnclosure::Cover,
            Self::CRUST => rapier::parry::transformation::MeshEnclosure::Crust,
        }
    }
    fn from_rapier(e: rapier::parry::transformation::MeshEnclosure) -> Self {
        match e {
            rapier::parry::transformation::MeshEnclosure::Cover => Self::COVER,
            rapier::parry::transformation::MeshEnclosure::Crust => Self::CRUST,
        }
    }
}

/// The parameters filling a closed triangle mesh with tetrahedral cells (see
/// :meth:`SoftBody.volumetric_with`): the cell size, and how the lattice cover of the mesh is
/// refined and smoothed.
#[pyclass(name = "VolumeMeshParameters", module = "rapier")]
#[derive(Debug, Clone, Copy)]
pub struct VolumeMeshParameters(pub rapier::parry::transformation::VolumeMeshParameters);

#[pymethods]
impl VolumeMeshParameters {
    /// Parameters generating cells of size ``cell_size``; the defaults give a raw cover (no
    /// smoothing, no subdivision).
    #[new]
    #[pyo3(signature = (
        cell_size,
        enclosure=MeshEnclosure::COVER,
        cover_smoothing=0,
        cover_guard=0.15,
        cover_subdivisions=0,
    ))]
    fn new(
        cell_size: Real,
        enclosure: MeshEnclosure,
        cover_smoothing: u32,
        cover_guard: Real,
        cover_subdivisions: u32,
    ) -> Self {
        let mut params = rapier::parry::transformation::VolumeMeshParameters::new(cell_size);
        params.enclosure = enclosure.to_rapier();
        params.cover_smoothing = cover_smoothing;
        params.cover_guard = cover_guard;
        params.cover_subdivisions = cover_subdivisions;
        Self(params)
    }
    /// Target size of the generated cells.
    #[getter]
    fn cell_size(&self) -> Real {
        self.0.cell_size
    }
    #[setter]
    fn set_cell_size(&mut self, v: Real) {
        self.0.cell_size = v;
    }
    /// Whether the whole shape is covered or its surface alone.
    #[getter]
    fn enclosure(&self) -> MeshEnclosure {
        MeshEnclosure::from_rapier(self.0.enclosure)
    }
    #[setter]
    fn set_enclosure(&mut self, v: MeshEnclosure) {
        self.0.enclosure = v.to_rapier();
    }
    /// How many shrink-wrap iterations smooth the staircase of a ``COVER`` mesh (``0`` leaves
    /// it raw). Each iteration pulls the boundary toward the shape, held off by
    /// :attr:`cover_guard`.
    #[getter]
    fn cover_smoothing(&self) -> u32 {
        self.0.cover_smoothing
    }
    #[setter]
    fn set_cover_smoothing(&mut self, v: u32) {
        self.0.cover_smoothing = v;
    }
    /// How close to the shape the smoothed cover may pull its boundary, as a fraction of the
    /// local cell size.
    #[getter]
    fn cover_guard(&self) -> Real {
        self.0.cover_guard
    }
    #[setter]
    fn set_cover_guard(&mut self, v: Real) {
        self.0.cover_guard = v;
    }
    /// How many halvings below :attr:`cell_size` a ``COVER`` cell crossing the shape's boundary
    /// may be refined (``0`` keeps the boundary at the cell size); runs before the smoothing.
    #[getter]
    fn cover_subdivisions(&self) -> u32 {
        self.0.cover_subdivisions
    }
    #[setter]
    fn set_cover_subdivisions(&mut self, v: u32) {
        self.0.cover_subdivisions = v;
    }
    fn __repr__(&self) -> String {
        format!(
            "VolumeMeshParameters(cell_size={}, enclosure=MeshEnclosure.{:?}, cover_smoothing={}, cover_guard={}, cover_subdivisions={})",
            self.0.cell_size,
            MeshEnclosure::from_rapier(self.0.enclosure),
            self.0.cover_smoothing,
            self.0.cover_guard,
            self.0.cover_subdivisions
        )
    }
}

// ----------------------------------------------------------------------
// Material.
// ----------------------------------------------------------------------

/// The material of a soft body: the softness of its constraints, the elasticity of its
/// cells, its plasticity and its tearing thresholds.
///
/// Every field can be passed as a keyword to the constructor::
///
///     material = SoftBodyMaterial(young_modulus=2.0e3, poisson_ratio=0.35, tear_strain=0.4)
///
/// :attr:`SoftBody.material` gives a live view of a body's material: setting one of its fields
/// changes the body. :meth:`copy` detaches a standalone copy.
#[pyclass(name = "SoftBodyMaterial", module = "rapier")]
#[derive(Debug, Clone)]
pub struct SoftBodyMaterial {
    backing: SoftBodyMaterialBacking,
}

/// Storage backing a `SoftBodyMaterial`: a standalone value, or a live view into the material of
/// a soft body.
#[derive(Debug)]
enum SoftBodyMaterialBacking {
    Owned(rapier::dynamics::SoftBodyMaterial),
    InBody(Py<SoftBody>),
}

impl Clone for SoftBodyMaterialBacking {
    fn clone(&self) -> Self {
        match self {
            Self::Owned(m) => Self::Owned(*m),
            Self::InBody(body) => Python::with_gil(|py| Self::InBody(body.clone_ref(py))),
        }
    }
}

impl SoftBodyMaterial {
    pub(crate) fn owned(material: rapier::dynamics::SoftBodyMaterial) -> Self {
        Self {
            backing: SoftBodyMaterialBacking::Owned(material),
        }
    }

    /// A copy of the material's value.
    pub(crate) fn get(&self) -> PyResult<rapier::dynamics::SoftBodyMaterial> {
        self.read(|m| *m)
    }

    fn read<R>(&self, f: impl FnOnce(&rapier::dynamics::SoftBodyMaterial) -> R) -> PyResult<R> {
        match &self.backing {
            SoftBodyMaterialBacking::Owned(m) => Ok(f(m)),
            SoftBodyMaterialBacking::InBody(body) => {
                Python::with_gil(|py| body.bind(py).try_borrow()?.with_ref(|b| f(b.material())))
            }
        }
    }

    fn write<R>(
        &mut self,
        f: impl FnOnce(&mut rapier::dynamics::SoftBodyMaterial) -> R,
    ) -> PyResult<R> {
        match &mut self.backing {
            SoftBodyMaterialBacking::Owned(m) => Ok(f(m)),
            SoftBodyMaterialBacking::InBody(body) => Python::with_gil(|py| {
                body.bind(py)
                    .try_borrow_mut()?
                    .with_mut(|b| f(b.material_mut()))
            }),
        }
    }

    fn apply_kwarg(&mut self, key: &str, v: &Bound<'_, PyAny>) -> PyResult<()> {
        let mut m = self.get()?;
        match key {
            "edge_softness" => m.edge_softness = spring(v)?,
            "bend_softness" => m.bend_softness = spring(v)?,
            "volume_softness" => m.volume_softness = spring(v)?,
            "shape_matching_softness" => m.shape_matching_softness = spring(v)?,
            "young_modulus" => m.young_modulus = v.extract()?,
            "poisson_ratio" => m.poisson_ratio = v.extract()?,
            "elastic_damping_ratio" => m.elastic_damping_ratio = v.extract()?,
            "plastic_yield" => m.plastic_yield = v.extract()?,
            "plastic_creep" => m.plastic_creep = v.extract()?,
            "plastic_max" => m.plastic_max = v.extract()?,
            "deformation_damping" => m.deformation_damping = v.extract()?,
            "edge_plastic_yield" => m.edge_plastic_yield = v.extract()?,
            "edge_plastic_creep" => m.edge_plastic_creep = v.extract()?,
            "edge_plastic_max" => m.edge_plastic_max = v.extract()?,
            "edge_plastic_flow" => {
                m.edge_plastic_flow = v.extract::<SoftEdgePlasticFlow>()?.to_rapier()
            }
            "tear_strain" => m.tear_strain = v.extract()?,
            "tear_force" => m.tear_force = v.extract()?,
            "tear_smoothing" => m.tear_smoothing = v.extract()?,
            "interior_strength" => m.interior_strength = v.extract()?,
            "max_tears_per_step" => m.max_tears_per_step = v.extract()?,
            "min_piece" => m.min_piece = v.extract()?,
            other => {
                return Err(PyTypeError::new_err(format!(
                    "SoftBodyMaterial: unknown keyword argument '{other}'"
                )));
            }
        }
        self.write(|dst| *dst = m)
    }
}

#[pymethods]
impl SoftBodyMaterial {
    /// The default material, with every field given as a keyword overridden.
    #[new]
    #[pyo3(signature = (**kwargs))]
    fn new(kwargs: Option<&Bound<'_, PyDict>>) -> PyResult<Self> {
        let mut me = Self::owned(rapier::dynamics::SoftBodyMaterial::default());
        if let Some(kw) = kwargs {
            for (k, v) in kw.iter() {
                let key: String = k.extract()?;
                me.apply_kwarg(&key, &v)?;
            }
        }
        Ok(me)
    }

    /// A material whose edge, bend, volume and shape-matching softness all take the same
    /// value (a :class:`SpringCoefficients` or a ``(frequency, damping)`` tuple).
    #[staticmethod]
    fn uniform(softness: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self::owned(rapier::dynamics::SoftBodyMaterial::uniform(
            spring(softness)?,
        )))
    }

    /// A standalone copy of the material (detached from the soft body it may be a view of).
    fn copy(&self) -> PyResult<Self> {
        Ok(Self::owned(self.get()?))
    }

    /// Does the material tear (a strain or force threshold is set)?
    fn tears(&self) -> PyResult<bool> {
        self.read(|s| s.tears())
    }

    /// The Lamé parameters ``(lambda, mu)`` of the elastic cells.
    fn lame_parameters(&self) -> PyResult<(Real, Real)> {
        self.read(|s| s.lame_parameters())
    }

    /// Softness of the structural edges.
    #[getter]
    fn edge_softness(&self) -> PyResult<SpringCoefficients> {
        self.read(|s| SpringCoefficients(s.edge_softness))
    }
    #[setter]
    fn set_edge_softness(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
        let spring = spring(v)?;
        self.write(|s| s.edge_softness = spring)
    }
    /// Softness of the bending constraints (bend edges and dihedrals).
    #[getter]
    fn bend_softness(&self) -> PyResult<SpringCoefficients> {
        self.read(|s| SpringCoefficients(s.bend_softness))
    }
    #[setter]
    fn set_bend_softness(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
        let spring = spring(v)?;
        self.write(|s| s.bend_softness = spring)
    }
    /// Softness of the volume preservation constraints.
    #[getter]
    fn volume_softness(&self) -> PyResult<SpringCoefficients> {
        self.read(|s| SpringCoefficients(s.volume_softness))
    }
    #[setter]
    fn set_volume_softness(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
        let spring = spring(v)?;
        self.write(|s| s.volume_softness = spring)
    }
    /// Softness of the shape-matching constraints.
    #[getter]
    fn shape_matching_softness(&self) -> PyResult<SpringCoefficients> {
        self.read(|s| SpringCoefficients(s.shape_matching_softness))
    }
    #[setter]
    fn set_shape_matching_softness(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
        let spring = spring(v)?;
        self.write(|s| s.shape_matching_softness = spring)
    }
    /// Young's modulus of the elastic cells (``COROTATIONAL`` and ``NEO_HOOKEAN`` models).
    #[getter]
    fn young_modulus(&self) -> PyResult<Real> {
        self.read(|s| s.young_modulus)
    }
    #[setter]
    fn set_young_modulus(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.young_modulus = v)
    }
    /// Poisson's ratio of the elastic cells.
    #[getter]
    fn poisson_ratio(&self) -> PyResult<Real> {
        self.read(|s| s.poisson_ratio)
    }
    #[setter]
    fn set_poisson_ratio(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.poisson_ratio = v)
    }
    /// Damping ratio of the elastic cells.
    #[getter]
    fn elastic_damping_ratio(&self) -> PyResult<Real> {
        self.read(|s| s.elastic_damping_ratio)
    }
    #[setter]
    fn set_elastic_damping_ratio(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.elastic_damping_ratio = v)
    }
    /// Strain beyond which the rest shape of an elastic cell flows (``0`` disables it).
    #[getter]
    fn plastic_yield(&self) -> PyResult<Real> {
        self.read(|s| s.plastic_yield)
    }
    #[setter]
    fn set_plastic_yield(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.plastic_yield = v)
    }
    /// Rate (per second) at which a cell's rest shape follows its deformation past the yield.
    #[getter]
    fn plastic_creep(&self) -> PyResult<Real> {
        self.read(|s| s.plastic_creep)
    }
    #[setter]
    fn set_plastic_creep(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.plastic_creep = v)
    }
    /// Largest accumulated plastic deformation of a cell.
    #[getter]
    fn plastic_max(&self) -> PyResult<Real> {
        self.read(|s| s.plastic_max)
    }
    #[setter]
    fn set_plastic_max(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.plastic_max = v)
    }
    /// Damping of the deformation velocity of the elastic cells.
    #[getter]
    fn deformation_damping(&self) -> PyResult<Real> {
        self.read(|s| s.deformation_damping)
    }
    #[setter]
    fn set_deformation_damping(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.deformation_damping = v)
    }
    /// Strain beyond which an edge's rest length flows (``0`` disables it).
    #[getter]
    fn edge_plastic_yield(&self) -> PyResult<Real> {
        self.read(|s| s.edge_plastic_yield)
    }
    #[setter]
    fn set_edge_plastic_yield(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.edge_plastic_yield = v)
    }
    /// Rate (per second) at which an edge's rest length follows its stretch past the yield.
    #[getter]
    fn edge_plastic_creep(&self) -> PyResult<Real> {
        self.read(|s| s.edge_plastic_creep)
    }
    #[setter]
    fn set_edge_plastic_creep(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.edge_plastic_creep = v)
    }
    /// Largest relative change of an edge's rest length.
    #[getter]
    fn edge_plastic_max(&self) -> PyResult<Real> {
        self.read(|s| s.edge_plastic_max)
    }
    #[setter]
    fn set_edge_plastic_max(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.edge_plastic_max = v)
    }
    /// Which strains make the edge rest lengths flow.
    #[getter]
    fn edge_plastic_flow(&self) -> PyResult<SoftEdgePlasticFlow> {
        self.read(|s| SoftEdgePlasticFlow::from_rapier(s.edge_plastic_flow))
    }
    #[setter]
    fn set_edge_plastic_flow(&mut self, v: SoftEdgePlasticFlow) -> PyResult<()> {
        self.write(|s| s.edge_plastic_flow = v.to_rapier())
    }
    /// Strain beyond which an element tears, or ``None``.
    #[getter]
    fn tear_strain(&self) -> PyResult<Option<Real>> {
        self.read(|s| s.tear_strain)
    }
    #[setter]
    fn set_tear_strain(&mut self, v: Option<Real>) -> PyResult<()> {
        self.write(|s| s.tear_strain = v)
    }
    /// Force beyond which an element tears, or ``None``.
    #[getter]
    fn tear_force(&self) -> PyResult<Option<Real>> {
        self.read(|s| s.tear_force)
    }
    #[setter]
    fn set_tear_force(&mut self, v: Option<Real>) -> PyResult<()> {
        self.write(|s| s.tear_force = v)
    }
    /// Time constant (seconds) of the load smoothing compared against ``tear_force``.
    #[getter]
    fn tear_smoothing(&self) -> PyResult<Real> {
        self.read(|s| s.tear_smoothing)
    }
    #[setter]
    fn set_tear_smoothing(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.tear_smoothing = v)
    }
    /// Multiplier of the tear thresholds of the interior elements.
    #[getter]
    fn interior_strength(&self) -> PyResult<Real> {
        self.read(|s| s.interior_strength)
    }
    #[setter]
    fn set_interior_strength(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.interior_strength = v)
    }
    /// Largest number of elements torn per step.
    #[getter]
    fn max_tears_per_step(&self) -> PyResult<u32> {
        self.read(|s| s.max_tears_per_step)
    }
    #[setter]
    fn set_max_tears_per_step(&mut self, v: u32) -> PyResult<()> {
        self.write(|s| s.max_tears_per_step = v)
    }
    /// Smallest piece (in elements) a tear may split off, or ``None`` for the default.
    #[getter]
    fn min_piece(&self) -> PyResult<Option<u32>> {
        self.read(|s| s.min_piece)
    }
    #[setter]
    fn set_min_piece(&mut self, v: Option<u32>) -> PyResult<()> {
        self.write(|s| s.min_piece = v)
    }

    fn __repr__(&self) -> PyResult<String> {
        self.read(|m| {
            format!(
                "SoftBodyMaterial(young_modulus={}, poisson_ratio={}, tear_strain={:?}, tear_force={:?})",
                m.young_modulus, m.poisson_ratio, m.tear_strain, m.tear_force
            )
        })
    }
}

// ----------------------------------------------------------------------
// Settings.
// ----------------------------------------------------------------------

/// Dynamics settings of a soft body's particles.
#[pyclass(name = "SoftBodyParticleSettings", module = "rapier")]
#[derive(Debug, Clone, Copy)]
pub struct SoftBodyParticleSettings(pub rapier::dynamics::SoftBodyParticleSettings);

#[pymethods]
impl SoftBodyParticleSettings {
    #[new]
    fn new() -> Self {
        Self(rapier::dynamics::SoftBodyParticleSettings::default())
    }
    /// Linear damping of the particles.
    #[getter]
    fn linear_damping(&self) -> Real {
        self.0.linear_damping
    }
    #[setter]
    fn set_linear_damping(&mut self, v: Real) {
        self.0.linear_damping = v;
    }
    /// Gravity scale of the particles.
    #[getter]
    fn gravity_scale(&self) -> Real {
        self.0.gravity_scale
    }
    #[setter]
    fn set_gravity_scale(&mut self, v: Real) {
        self.0.gravity_scale = v;
    }
    /// Extra solver substeps requested for the particles and everything they touch.
    #[getter]
    fn additional_solver_iterations(&self) -> usize {
        self.0.additional_solver_iterations
    }
    #[setter]
    fn set_additional_solver_iterations(&mut self, v: usize) {
        self.0.additional_solver_iterations = v;
    }
    /// Extra internal PGS iterations per substep for the particles and everything they touch
    /// (default ``3``).
    #[getter]
    fn additional_pgs_iterations(&self) -> usize {
        self.0.additional_pgs_iterations
    }
    #[setter]
    fn set_additional_pgs_iterations(&mut self, v: usize) {
        self.0.additional_pgs_iterations = v;
    }
    /// Whether the soft body may fall asleep.
    #[getter]
    fn can_sleep(&self) -> bool {
        self.0.can_sleep
    }
    #[setter]
    fn set_can_sleep(&mut self, v: bool) {
        self.0.can_sleep = v;
    }
    /// Dominance group of the soft body.
    #[getter]
    fn dominance_group(&self) -> i8 {
        self.0.dominance_group
    }
    #[setter]
    fn set_dominance_group(&mut self, v: i8) {
        self.0.dominance_group = v;
    }
}

/// A settings group nested in [`rapier::dynamics::SoftBodiesSettings`].
trait SoftBodiesSettingsField: Copy {
    fn field(settings: &rapier::dynamics::SoftBodiesSettings) -> &Self;
    fn field_mut(settings: &mut rapier::dynamics::SoftBodiesSettings) -> &mut Self;
}

impl SoftBodiesSettingsField for rapier::dynamics::SoftRecoverySettings {
    fn field(settings: &rapier::dynamics::SoftBodiesSettings) -> &Self {
        &settings.recovery
    }
    fn field_mut(settings: &mut rapier::dynamics::SoftBodiesSettings) -> &mut Self {
        &mut settings.recovery
    }
}

impl SoftBodiesSettingsField for rapier::dynamics::SoftFemParameters {
    fn field(settings: &rapier::dynamics::SoftBodiesSettings) -> &Self {
        &settings.fem
    }
    fn field_mut(settings: &mut rapier::dynamics::SoftBodiesSettings) -> &mut Self {
        &mut settings.fem
    }
}

/// Storage backing a settings group nested in `SoftBodiesSettings`: a standalone value, or a
/// live view into the group of a `SoftBodiesSettings` (itself standalone or a view).
#[derive(Debug)]
enum NestedSettingsBacking<T> {
    Owned(T),
    InSettings(Py<SoftBodiesSettings>),
}

impl<T: Copy> Clone for NestedSettingsBacking<T> {
    fn clone(&self) -> Self {
        match self {
            Self::Owned(v) => Self::Owned(*v),
            Self::InSettings(s) => Python::with_gil(|py| Self::InSettings(s.clone_ref(py))),
        }
    }
}

impl<T: SoftBodiesSettingsField> NestedSettingsBacking<T> {
    fn read<R>(&self, f: impl FnOnce(&T) -> R) -> PyResult<R> {
        match self {
            Self::Owned(v) => Ok(f(v)),
            Self::InSettings(s) => {
                Python::with_gil(|py| s.bind(py).try_borrow()?.read(|s| f(T::field(s))))
            }
        }
    }

    fn write<R>(&mut self, f: impl FnOnce(&mut T) -> R) -> PyResult<R> {
        match self {
            Self::Owned(v) => Ok(f(v)),
            Self::InSettings(s) => {
                Python::with_gil(|py| s.bind(py).try_borrow_mut()?.write(|s| f(T::field_mut(s))))
            }
        }
    }
}

/// Runtime toggles and tuning of the soft-body tangle detection and recovery stack. Every
/// mechanism can be switched off individually.
///
/// :attr:`SoftBodiesSettings.recovery` gives a live view: setting one of its fields changes the
/// settings it was read from. :meth:`copy` detaches a standalone copy.
#[pyclass(name = "SoftRecoverySettings", module = "rapier")]
#[derive(Debug, Clone)]
pub struct SoftRecoverySettings {
    backing: NestedSettingsBacking<rapier::dynamics::SoftRecoverySettings>,
}

impl SoftRecoverySettings {
    /// A copy of the settings' value.
    fn get(&self) -> PyResult<rapier::dynamics::SoftRecoverySettings> {
        self.read(|s| *s)
    }
    fn read<R>(&self, f: impl FnOnce(&rapier::dynamics::SoftRecoverySettings) -> R) -> PyResult<R> {
        self.backing.read(f)
    }
    fn write<R>(
        &mut self,
        f: impl FnOnce(&mut rapier::dynamics::SoftRecoverySettings) -> R,
    ) -> PyResult<R> {
        self.backing.write(f)
    }
}

#[pymethods]
impl SoftRecoverySettings {
    /// The default settings.
    #[new]
    fn new() -> Self {
        Self {
            backing: NestedSettingsBacking::Owned(rapier::dynamics::SoftRecoverySettings::default()),
        }
    }
    /// A standalone copy of the settings (detached from the settings it may be a view of).
    fn copy(&self) -> PyResult<Self> {
        Ok(Self {
            backing: NestedSettingsBacking::Owned(self.get()?),
        })
    }
    #[getter]
    fn authored_velocity_margin(&self) -> PyResult<bool> {
        self.read(|s| s.authored_velocity_margin)
    }
    #[setter]
    fn set_authored_velocity_margin(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.authored_velocity_margin = v)
    }
    #[getter]
    fn edge_speculation(&self) -> PyResult<bool> {
        self.read(|s| s.edge_speculation)
    }
    #[setter]
    fn set_edge_speculation(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.edge_speculation = v)
    }
    #[getter]
    fn inverted_cell_detection(&self) -> PyResult<bool> {
        self.read(|s| s.inverted_cell_detection)
    }
    #[setter]
    fn set_inverted_cell_detection(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.inverted_cell_detection = v)
    }
    #[getter]
    fn self_crossing_detection(&self) -> PyResult<bool> {
        self.read(|s| s.self_crossing_detection)
    }
    #[setter]
    fn set_self_crossing_detection(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.self_crossing_detection = v)
    }
    #[getter]
    fn detection_motion_gating(&self) -> PyResult<bool> {
        self.read(|s| s.detection_motion_gating)
    }
    #[setter]
    fn set_detection_motion_gating(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.detection_motion_gating = v)
    }
    #[getter]
    fn cross_body_detection(&self) -> PyResult<bool> {
        self.read(|s| s.cross_body_detection)
    }
    #[setter]
    fn set_cross_body_detection(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.cross_body_detection = v)
    }
    #[getter]
    fn self_stand_down(&self) -> PyResult<bool> {
        self.read(|s| s.self_stand_down)
    }
    #[setter]
    fn set_self_stand_down(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.self_stand_down = v)
    }
    #[getter]
    fn cross_body_expel_gate(&self) -> PyResult<bool> {
        self.read(|s| s.cross_body_expel_gate)
    }
    #[setter]
    fn set_cross_body_expel_gate(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.cross_body_expel_gate = v)
    }
    #[getter]
    fn edge_stand_down(&self) -> PyResult<bool> {
        self.read(|s| s.edge_stand_down)
    }
    #[setter]
    fn set_edge_stand_down(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.edge_stand_down = v)
    }
    #[getter]
    fn crossing_repulsion(&self) -> PyResult<bool> {
        self.read(|s| s.crossing_repulsion)
    }
    #[setter]
    fn set_crossing_repulsion(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.crossing_repulsion = v)
    }
    #[getter]
    fn crossing_repulsion_guide(&self) -> PyResult<bool> {
        self.read(|s| s.crossing_repulsion_guide)
    }
    #[setter]
    fn set_crossing_repulsion_guide(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.crossing_repulsion_guide = v)
    }
    #[getter]
    fn crossing_repulsion_self_guide(&self) -> PyResult<bool> {
        self.read(|s| s.crossing_repulsion_self_guide)
    }
    #[setter]
    fn set_crossing_repulsion_self_guide(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.crossing_repulsion_self_guide = v)
    }
    #[getter]
    fn recovery_pace(&self) -> PyResult<Real> {
        self.read(|s| s.recovery_pace)
    }
    #[setter]
    fn set_recovery_pace(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.recovery_pace = v)
    }
    #[getter]
    fn overlap_constraints(&self) -> PyResult<bool> {
        self.read(|s| s.overlap_constraints)
    }
    #[setter]
    fn set_overlap_constraints(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.overlap_constraints = v)
    }
    #[getter]
    fn overlap_rigid(&self) -> PyResult<bool> {
        self.read(|s| s.overlap_rigid)
    }
    #[setter]
    fn set_overlap_rigid(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.overlap_rigid = v)
    }
    #[getter]
    fn overlap_skip_self_tangled(&self) -> PyResult<bool> {
        self.read(|s| s.overlap_skip_self_tangled)
    }
    #[setter]
    fn set_overlap_skip_self_tangled(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.overlap_skip_self_tangled = v)
    }
    #[getter]
    fn overlap_edge_stand_down(&self) -> PyResult<bool> {
        self.read(|s| s.overlap_edge_stand_down)
    }
    #[setter]
    fn set_overlap_edge_stand_down(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.overlap_edge_stand_down = v)
    }
    #[getter]
    fn overlap_constraint_pace(&self) -> PyResult<Real> {
        self.read(|s| s.overlap_constraint_pace)
    }
    #[setter]
    fn set_overlap_constraint_pace(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.overlap_constraint_pace = v)
    }
    #[getter]
    fn overlap_skin_volume(&self) -> PyResult<bool> {
        self.read(|s| s.overlap_skin_volume)
    }
    #[setter]
    fn set_overlap_skin_volume(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.overlap_skin_volume = v)
    }
    #[getter]
    fn overlap_kept_depth(&self) -> PyResult<Real> {
        self.read(|s| s.overlap_kept_depth)
    }
    #[setter]
    fn set_overlap_kept_depth(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.overlap_kept_depth = v)
    }
    #[getter]
    fn overlap_self_regions(&self) -> PyResult<bool> {
        self.read(|s| s.overlap_self_regions)
    }
    #[setter]
    fn set_overlap_self_regions(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.overlap_self_regions = v)
    }
    #[getter]
    fn overlap_normal_push(&self) -> PyResult<bool> {
        self.read(|s| s.overlap_normal_push)
    }
    #[setter]
    fn set_overlap_normal_push(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.overlap_normal_push = v)
    }
    #[getter]
    fn overlap_multi_volume(&self) -> PyResult<bool> {
        self.read(|s| s.overlap_multi_volume)
    }
    #[setter]
    fn set_overlap_multi_volume(&mut self, v: bool) -> PyResult<()> {
        self.write(|s| s.overlap_multi_volume = v)
    }
    #[getter]
    fn overlap_split(&self) -> PyResult<u32> {
        self.read(|s| s.overlap_split)
    }
    #[setter]
    fn set_overlap_split(&mut self, v: u32) -> PyResult<()> {
        self.write(|s| s.overlap_split = v)
    }
    #[getter]
    fn overlap_patience(&self) -> PyResult<u32> {
        self.read(|s| s.overlap_patience)
    }
    #[setter]
    fn set_overlap_patience(&mut self, v: u32) -> PyResult<()> {
        self.write(|s| s.overlap_patience = v)
    }
    #[getter]
    fn overlap_progress_margin(&self) -> PyResult<Real> {
        self.read(|s| s.overlap_progress_margin)
    }
    #[setter]
    fn set_overlap_progress_margin(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.overlap_progress_margin = v)
    }
    /// What the recovery does with the contact patches of crossed surfaces.
    #[getter]
    fn overlap_patch_constraints(&self) -> PyResult<SoftPatchConstraints> {
        self.read(|s| SoftPatchConstraints::from_rapier(s.overlap_patch_constraints))
    }
    #[setter]
    fn set_overlap_patch_constraints(&mut self, v: SoftPatchConstraints) -> PyResult<()> {
        self.write(|s| s.overlap_patch_constraints = v.to_rapier())
    }
}

/// Tuning of the FEM soft-body solver (see :attr:`SoftBodySolver.FEM`), which solves a linear
/// system per substep by conjugate gradient and factorizes it once per step (see
/// :attr:`max_dense_dofs`); lives on :attr:`SoftBodiesSettings.fem`.
///
/// Every field can be passed as a keyword to the constructor::
///
///     fem = SoftFemParameters(linear_tolerance=1.0e-6, max_linear_iterations=50)
///
/// :attr:`SoftBodiesSettings.fem` gives a live view: setting one of its fields changes the
/// settings it was read from. :meth:`copy` detaches a standalone copy.
#[pyclass(name = "SoftFemParameters", module = "rapier")]
#[derive(Debug, Clone)]
pub struct SoftFemParameters {
    backing: NestedSettingsBacking<rapier::dynamics::SoftFemParameters>,
}

impl SoftFemParameters {
    /// A copy of the parameters' value.
    fn get(&self) -> PyResult<rapier::dynamics::SoftFemParameters> {
        self.read(|s| *s)
    }
    fn read<R>(&self, f: impl FnOnce(&rapier::dynamics::SoftFemParameters) -> R) -> PyResult<R> {
        self.backing.read(f)
    }
    fn write<R>(
        &mut self,
        f: impl FnOnce(&mut rapier::dynamics::SoftFemParameters) -> R,
    ) -> PyResult<R> {
        self.backing.write(f)
    }
}

#[pymethods]
impl SoftFemParameters {
    /// The default parameters, with every field given as a keyword overridden.
    #[new]
    #[pyo3(signature = (**kwargs))]
    fn new(kwargs: Option<&Bound<'_, PyDict>>) -> PyResult<Self> {
        let mut params = rapier::dynamics::SoftFemParameters::default();
        if let Some(kw) = kwargs {
            for (k, v) in kw.iter() {
                let key: String = k.extract()?;
                match key.as_str() {
                    "linear_tolerance" => params.linear_tolerance = v.extract()?,
                    "max_linear_iterations" => params.max_linear_iterations = v.extract()?,
                    "max_dense_dofs" => params.max_dense_dofs = v.extract()?,
                    other => {
                        return Err(PyTypeError::new_err(format!(
                            "SoftFemParameters: unknown keyword argument '{other}'"
                        )));
                    }
                }
            }
        }
        Ok(Self {
            backing: NestedSettingsBacking::Owned(params),
        })
    }
    /// A standalone copy of the parameters (detached from the settings they may be a view of).
    fn copy(&self) -> PyResult<Self> {
        Ok(Self {
            backing: NestedSettingsBacking::Owned(self.get()?),
        })
    }
    /// Relative residual at which the conjugate gradient stops (default ``1.0e-5``).
    #[getter]
    fn linear_tolerance(&self) -> PyResult<Real> {
        self.read(|s| s.linear_tolerance)
    }
    #[setter]
    fn set_linear_tolerance(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.linear_tolerance = v)
    }
    /// Hard cap on the conjugate-gradient iterations, whatever the residual (default ``20``). A
    /// truncated solve is under-relaxed (safe), only slower to settle.
    #[getter]
    fn max_linear_iterations(&self) -> PyResult<usize> {
        self.read(|s| s.max_linear_iterations)
    }
    #[setter]
    fn set_max_linear_iterations(&mut self, v: usize) -> PyResult<()> {
        self.write(|s| s.max_linear_iterations = v)
    }
    /// Largest number of degrees of freedom (3 per particle) for which the step-start matrix of
    /// a body is factorized directly (default ``600``); larger bodies fall back to a conjugate
    /// gradient for the solves their constraints need.
    #[getter]
    fn max_dense_dofs(&self) -> PyResult<usize> {
        self.read(|s| s.max_dense_dofs)
    }
    #[setter]
    fn set_max_dense_dofs(&mut self, v: usize) -> PyResult<()> {
        self.write(|s| s.max_dense_dofs = v)
    }
    fn __repr__(&self) -> PyResult<String> {
        self.read(|s| {
            format!(
                "SoftFemParameters(linear_tolerance={}, max_linear_iterations={}, max_dense_dofs={})",
                s.linear_tolerance, s.max_linear_iterations, s.max_dense_dofs
            )
        })
    }
}

/// Simulation settings shared by every soft body of a world; lives on
/// :attr:`IntegrationParameters.soft_bodies`.
///
/// :attr:`IntegrationParameters.soft_bodies` gives a live view: setting one of its fields (or of
/// its nested :attr:`recovery` and :attr:`fem` groups, also live views) changes the parameters
/// it was read from. :meth:`copy` detaches a standalone copy.
#[pyclass(name = "SoftBodiesSettings", module = "rapier")]
#[derive(Debug, Clone)]
pub struct SoftBodiesSettings {
    backing: SoftBodiesSettingsBacking,
}

/// Storage backing a `SoftBodiesSettings`: a standalone value, or a live view into the
/// `soft_bodies` of an `IntegrationParameters`.
#[derive(Debug)]
enum SoftBodiesSettingsBacking {
    Owned(rapier::dynamics::SoftBodiesSettings),
    InParams(Py<crate::dynamics::IntegrationParameters>),
}

impl Clone for SoftBodiesSettingsBacking {
    fn clone(&self) -> Self {
        match self {
            Self::Owned(s) => Self::Owned(*s),
            Self::InParams(p) => Python::with_gil(|py| Self::InParams(p.clone_ref(py))),
        }
    }
}

impl SoftBodiesSettings {
    /// A live view of the soft-body settings of `params`.
    pub(crate) fn in_params(params: Py<crate::dynamics::IntegrationParameters>) -> Self {
        Self {
            backing: SoftBodiesSettingsBacking::InParams(params),
        }
    }

    /// A copy of the settings' value.
    pub(crate) fn get(&self) -> PyResult<rapier::dynamics::SoftBodiesSettings> {
        self.read(|s| *s)
    }

    fn read<R>(&self, f: impl FnOnce(&rapier::dynamics::SoftBodiesSettings) -> R) -> PyResult<R> {
        match &self.backing {
            SoftBodiesSettingsBacking::Owned(s) => Ok(f(s)),
            SoftBodiesSettingsBacking::InParams(p) => {
                Python::with_gil(|py| Ok(f(&p.bind(py).try_borrow()?.0.soft_bodies)))
            }
        }
    }

    fn write<R>(
        &mut self,
        f: impl FnOnce(&mut rapier::dynamics::SoftBodiesSettings) -> R,
    ) -> PyResult<R> {
        match &mut self.backing {
            SoftBodiesSettingsBacking::Owned(s) => Ok(f(s)),
            SoftBodiesSettingsBacking::InParams(p) => {
                Python::with_gil(|py| Ok(f(&mut p.bind(py).try_borrow_mut()?.0.soft_bodies)))
            }
        }
    }
}

#[pymethods]
impl SoftBodiesSettings {
    /// The default settings.
    #[new]
    fn new() -> Self {
        Self {
            backing: SoftBodiesSettingsBacking::Owned(
                rapier::dynamics::SoftBodiesSettings::default(),
            ),
        }
    }
    /// A standalone copy of the settings (detached from the parameters they may be a view of).
    fn copy(&self) -> PyResult<Self> {
        Ok(Self {
            backing: SoftBodiesSettingsBacking::Owned(self.get()?),
        })
    }
    /// The tangle detection and recovery settings, as a live view; assigning a
    /// :class:`SoftRecoverySettings` replaces them all.
    #[getter]
    fn recovery(slf: &Bound<'_, Self>) -> SoftRecoverySettings {
        SoftRecoverySettings {
            backing: NestedSettingsBacking::InSettings(slf.clone().unbind()),
        }
    }
    #[setter]
    fn set_recovery(slf: &Bound<'_, Self>, v: &Bound<'_, SoftRecoverySettings>) -> PyResult<()> {
        // Read the value first: `v` may be a view of these very settings.
        let value = v.try_borrow()?.get()?;
        slf.try_borrow_mut()?.write(|s| s.recovery = value)
    }
    /// The tuning of the FEM solver (see :class:`SoftFemParameters`), as a live view; assigning
    /// a :class:`SoftFemParameters` replaces it.
    #[getter]
    fn fem(slf: &Bound<'_, Self>) -> SoftFemParameters {
        SoftFemParameters {
            backing: NestedSettingsBacking::InSettings(slf.clone().unbind()),
        }
    }
    #[setter]
    fn set_fem(slf: &Bound<'_, Self>, v: &Bound<'_, SoftFemParameters>) -> PyResult<()> {
        let value = v.try_borrow()?.get()?;
        slf.try_borrow_mut()?.write(|s| s.fem = value)
    }
    /// Strain beyond which a soft-body constraint is re-solved after the contacts inside every
    /// substep (default ``0.75``).
    #[getter]
    fn resweep_strain(&self) -> PyResult<Real> {
        self.read(|s| s.resweep_strain)
    }
    #[setter]
    fn set_resweep_strain(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.resweep_strain = v)
    }
    /// Maximum number of extra substeps a soft body requests while it is hit fast (default
    /// ``4``; ``0`` disables them).
    #[getter]
    fn max_extra_substeps(&self) -> PyResult<usize> {
        self.read(|s| s.max_extra_substeps)
    }
    #[setter]
    fn set_max_extra_substeps(&mut self, v: usize) -> PyResult<()> {
        self.write(|s| s.max_extra_substeps = v)
    }
    /// Factor applied to the contact softness natural frequencies for the soft-body contacts
    /// (default ``4.0``).
    #[getter]
    fn contact_stiffening(&self) -> PyResult<Real> {
        self.read(|s| s.contact_stiffening)
    }
    #[setter]
    fn set_contact_stiffening(&mut self, v: Real) -> PyResult<()> {
        self.write(|s| s.contact_stiffening = v)
    }
}

// ----------------------------------------------------------------------
// Builder.
// ----------------------------------------------------------------------

/// Builder of a :class:`SoftBody`: particles in world space, and elements referencing them by
/// index. Start from a generator (:meth:`SoftBody.rope`, :meth:`SoftBody.cloth`,
/// :meth:`SoftBody.cuboid`, :meth:`SoftBody.sphere`, :meth:`SoftBody.trimesh`,
/// :meth:`SoftBody.volumetric`) or from raw positions, then chain the setters, and insert
/// the result with :meth:`SoftBodySet.insert` or :meth:`PhysicsWorld.add_soft_body`.
#[pyclass(name = "SoftBodyBuilder", module = "rapier")]
#[derive(Debug, Clone)]
pub struct SoftBodyBuilder {
    pub builder: rapier::dynamics::SoftBodyBuilder,
}

impl SoftBodyBuilder {
    pub fn from_kwargs(
        base: rapier::dynamics::SoftBodyBuilder,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<Self> {
        let mut me = Self { builder: base };
        if let Some(kw) = kwargs {
            for (k, v) in kw.iter() {
                let key: String = k.extract()?;
                me.apply_kwarg(&key, &v)?;
            }
        }
        Ok(me)
    }

    fn apply_kwarg(&mut self, key: &str, v: &Bound<'_, PyAny>) -> PyResult<()> {
        let b = std::mem::take(&mut self.builder);
        self.builder = match key {
            "particle_mass" => b.particle_mass(v.extract()?),
            "mass" => b.mass(v.extract()?),
            "masses" => b.masses(v.extract()?),
            "pinned_particles" => b.pinned_particles(extract_index_list(v)?),
            "softness" => b.softness(spring(v)?),
            "material" => b.material(v.extract::<PyRef<'_, SoftBodyMaterial>>()?.get()?),
            "tear_strain" => b.tear_strain(v.extract()?),
            "tear_force" => b.tear_force(v.extract()?),
            "min_piece" => b.min_piece(v.extract()?),
            "tear_smoothing" => b.tear_smoothing(v.extract()?),
            "interior_strength" => b.interior_strength(v.extract()?),
            "cell_model" => b.cell_model(v.extract::<SoftBodyCellModel>()?.to_rapier()),
            "solver" => b.solver(v.extract::<SoftBodySolver>()?.to_rapier()),
            "volume_preservation" => b.volume_preservation(v.extract()?),
            "volume_factor" => b.volume_factor(v.extract()?),
            "shape_matching" => b.shape_matching(v.extract()?),
            "self_contacts" => b.self_contacts(v.extract()?),
            "oriented" => b.oriented(v.extract()?),
            "particle_radius" => b.particle_radius(v.extract()?),
            "surface_collider" => {
                b.surface_collider(v.extract::<PyRef<'_, ColliderBuilder>>()?.builder.clone())
            }
            "skin_collision" => b.skin_collision(v.extract()?),
            "translation" => b.translated(v.extract::<PyVector>()?.0.into()),
            "linear_damping" => b.linear_damping(v.extract()?),
            "gravity_scale" => b.gravity_scale(v.extract()?),
            "additional_solver_iterations" => b.additional_solver_iterations(v.extract()?),
            "additional_pgs_iterations" => b.additional_pgs_iterations(v.extract()?),
            "can_sleep" => b.can_sleep(v.extract()?),
            "user_data" => b.user_data(v.extract()?),
            other => {
                self.builder = b;
                return Err(PyTypeError::new_err(format!(
                    "SoftBodyBuilder: unknown keyword argument '{other}'"
                )));
            }
        };
        Ok(())
    }

    fn chained(
        &self,
        f: impl FnOnce(rapier::dynamics::SoftBodyBuilder) -> rapier::dynamics::SoftBodyBuilder,
    ) -> Self {
        Self {
            builder: f(self.builder.clone()),
        }
    }
}

#[pymethods]
impl SoftBodyBuilder {
    /// A builder over raw world-space particle positions (an ``(N, 3)`` ndarray or a
    /// sequence of 3-tuples), with no element yet; the setters add them.
    #[new]
    #[pyo3(signature = (positions, **kwargs))]
    fn new(positions: &Bound<'_, PyAny>, kwargs: Option<&Bound<'_, PyDict>>) -> PyResult<Self> {
        let positions = crate::geometry::extract_verts_for_dim(positions)?;
        Self::from_kwargs(rapier::dynamics::SoftBodyBuilder::new(positions), kwargs)
    }

    /// The number of particles the builder holds.
    #[getter]
    fn num_particles(&self) -> usize {
        self.builder.positions.len()
    }

    /// The world-space particle positions as an ``(N, 3)`` ndarray.
    #[getter]
    fn positions<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<Real>> {
        vectors_to_array(py, self.builder.positions.iter().copied())
    }

    /// The structural edges as an ``(E, 2)`` ndarray.
    #[getter]
    fn current_edges<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.builder.edges)
    }

    /// The cells (tetrahedra) as a ``(C, 4)`` ndarray.
    #[getter]
    fn current_cells<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.builder.cells)
    }

    /// The boundary triangles as a ``(B, 3)`` ndarray (empty when derived from the cells at
    /// build time).
    #[getter]
    fn current_surface<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.builder.surface)
    }

    /// The material (a copy).
    #[getter]
    fn current_material(&self) -> SoftBodyMaterial {
        SoftBodyMaterial::owned(self.builder.material)
    }

    /// The edges of the surface elements, deduplicated, as an ``(E, 2)`` ndarray.
    fn surface_edges<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.builder.surface_edges())
    }

    /// The edges of the cells, deduplicated, as an ``(E, 2)`` ndarray.
    fn cell_edges<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.builder.cell_edges())
    }

    /// The dihedrals across the surface's interior edges, as a ``(D, 4)`` ndarray.
    fn surface_dihedrals<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.builder.surface_dihedrals())
    }

    /// Replace the particle positions.
    fn set_positions(&self, positions: &Bound<'_, PyAny>) -> PyResult<Self> {
        let positions = crate::geometry::extract_verts_for_dim(positions)?;
        Ok(self.chained(|b| b.positions(positions)))
    }
    /// Set the uniform mass of the particles.
    fn particle_mass(&self, mass: Real) -> Self {
        self.chained(|b| b.particle_mass(mass))
    }
    /// Set the total mass of the body, spread over its particles.
    fn mass(&self, mass: Real) -> Self {
        self.chained(|b| b.mass(mass))
    }
    /// Set per-particle masses.
    fn masses(&self, masses: Vec<Real>) -> Self {
        self.chained(|b| b.masses(masses))
    }
    /// Pin the given particles in place.
    fn pinned_particles(&self, pinned: &Bound<'_, PyAny>) -> PyResult<Self> {
        let pinned = extract_index_list(pinned)?;
        Ok(self.chained(|b| b.pinned_particles(pinned)))
    }
    /// Replace the structural edges (an ``(E, 2)`` ndarray or a sequence of pairs).
    fn edges(&self, edges: &Bound<'_, PyAny>) -> PyResult<Self> {
        let edges = extract_index_rows::<2>(edges)?;
        Ok(self.chained(|b| b.edges(edges)))
    }
    /// Add structural edges.
    fn add_edges(&self, edges: &Bound<'_, PyAny>) -> PyResult<Self> {
        let edges = extract_index_rows::<2>(edges)?;
        Ok(self.chained(|b| b.add_edges(edges)))
    }
    /// Replace the bending edges.
    fn bend_edges(&self, edges: &Bound<'_, PyAny>) -> PyResult<Self> {
        let edges = extract_index_rows::<2>(edges)?;
        Ok(self.chained(|b| b.bend_edges(edges)))
    }
    /// Make every edge resist stretching only (a rope or a net that folds freely).
    fn tension_only(&self) -> Self {
        self.chained(|b| b.tension_only())
    }
    /// Replace the dihedral bending constraints (``(D, 4)``: the shared edge, then the two
    /// opposite vertices).
    fn dihedrals(&self, dihedrals: &Bound<'_, PyAny>) -> PyResult<Self> {
        let dihedrals = extract_index_rows::<4>(dihedrals)?;
        Ok(self.chained(|b| b.dihedrals(dihedrals)))
    }
    /// Replace the cells (``(C, 4)`` tetrahedra).
    fn cells(&self, cells: &Bound<'_, PyAny>) -> PyResult<Self> {
        let cells = extract_index_rows::<4>(cells)?;
        Ok(self.chained(|b| b.cells(cells)))
    }
    /// Replace the boundary triangles (``(B, 3)``), oriented outward.
    fn surface(&self, surface: &Bound<'_, PyAny>) -> PyResult<Self> {
        let surface = extract_index_rows::<3>(surface)?;
        Ok(self.chained(|b| b.surface(surface)))
    }
    /// Set a skin: a finer mesh (world-space vertices and triangles) embedded in the cells.
    fn skin(&self, vertices: &Bound<'_, PyAny>, indices: &Bound<'_, PyAny>) -> PyResult<Self> {
        let vertices = crate::geometry::extract_verts_for_dim(vertices)?;
        let indices = extract_index_rows::<3>(indices)?;
        Ok(self.chained(|b| b.skin(vertices, indices)))
    }
    /// Make the body collide through its skin rather than through its cells' boundary.
    fn skin_collision(&self, enabled: bool) -> Self {
        self.chained(|b| b.skin_collision(enabled))
    }
    /// Set the segments a body without surface collides through (a wire).
    fn wire(&self, segments: &Bound<'_, PyAny>) -> PyResult<Self> {
        let segments = extract_index_rows::<2>(segments)?;
        Ok(self.chained(|b| b.wire(segments)))
    }
    /// Set the material.
    fn material(&self, material: &SoftBodyMaterial) -> PyResult<Self> {
        let m = material.get()?;
        Ok(self.chained(|b| b.material(m)))
    }
    /// Set a uniform softness (a :class:`SpringCoefficients` or a ``(frequency, damping)``
    /// tuple) for every constraint of the material.
    fn softness(&self, softness: &Bound<'_, PyAny>) -> PyResult<Self> {
        let s = spring(softness)?;
        Ok(self.chained(|b| b.softness(s)))
    }
    /// Strain beyond which an element tears.
    fn tear_strain(&self, strain: Real) -> Self {
        self.chained(|b| b.tear_strain(strain))
    }
    /// Force beyond which an element tears.
    fn tear_force(&self, force: Real) -> Self {
        self.chained(|b| b.tear_force(force))
    }
    /// Smallest piece (in elements) a tear may split off.
    fn min_piece(&self, elements: u32) -> Self {
        self.chained(|b| b.min_piece(elements))
    }
    /// Time constant (seconds) of the load smoothing compared against the tear force.
    fn tear_smoothing(&self, seconds: Real) -> Self {
        self.chained(|b| b.tear_smoothing(seconds))
    }
    /// Multiplier of the tear thresholds of the interior elements.
    fn interior_strength(&self, strength: Real) -> Self {
        self.chained(|b| b.interior_strength(strength))
    }
    /// Multiply the tear thresholds of the given edges: ``resistances`` maps edge indices to
    /// multipliers.
    fn edge_tear_resistance(&self, resistances: Vec<(u32, Real)>) -> Self {
        self.chained(|b| b.edge_tear_resistance(resistances))
    }
    /// Set the constitutive model of the cells.
    fn cell_model(&self, model: SoftBodyCellModel) -> Self {
        self.chained(|b| b.cell_model(model.to_rapier()))
    }
    /// Select the solver simulating the body's elasticity (default
    /// :attr:`SoftBodySolver.CONSTRAINTS`).
    fn solver(&self, solver: SoftBodySolver) -> Self {
        self.chained(|b| b.solver(solver.to_rapier()))
    }
    /// Enable the preservation of the volume enclosed by the body's closed surfaces.
    fn volume_preservation(&self, enabled: bool) -> Self {
        self.chained(|b| b.volume_preservation(enabled))
    }
    /// Set the target volume multiplier (``> 1`` inflates the body); this also enables the
    /// volume preservation.
    fn volume_factor(&self, factor: Real) -> Self {
        self.chained(|b| b.volume_factor(factor))
    }
    /// Hold the body's shape by shape matching.
    fn shape_matching(&self, enabled: bool) -> Self {
        self.chained(|b| b.shape_matching(enabled))
    }
    /// Make the body's surface collide with itself.
    fn self_contacts(&self, enabled: bool) -> Self {
        self.chained(|b| b.self_contacts(enabled))
    }
    /// Set whether the shape of the body's collision surface is built with the ``ORIENTED``
    /// flag, like a polyline or mesh. Left unset, it is whenever the surface is closed, which is
    /// what a solid body wants: an oriented closed surface encloses matter, so nothing is held
    /// inside it. Set it to ``False`` for a shell, whose inner side holds the bodies inside it.
    fn oriented(&self, oriented: bool) -> Self {
        self.chained(|b| b.oriented(oriented))
    }
    /// Set the thickness of the particles.
    fn particle_radius(&self, radius: Real) -> Self {
        self.chained(|b| b.particle_radius(radius))
    }
    /// Set the template of the body's colliders: its friction, groups, events and other
    /// settings are kept, its shape is replaced by the body's deformable surface.
    fn surface_collider(&self, collider: &ColliderBuilder) -> Self {
        let c = collider.builder.clone();
        self.chained(|b| b.surface_collider(c))
    }
    /// Remove the body's colliders: it will not collide with anything.
    fn no_surface_collider(&self) -> Self {
        self.chained(|b| b.no_surface_collider())
    }
    /// Translate every particle.
    fn translated(&self, translation: PyVector) -> Self {
        let t: rapier::math::Vector = translation.0.into();
        self.chained(|b| b.translated(t))
    }
    /// Set the dynamics settings of the particles.
    fn particle_settings(&self, settings: &SoftBodyParticleSettings) -> Self {
        let s = settings.0;
        self.chained(|b| b.particle_settings(s))
    }
    /// Set the linear damping of the particles.
    fn linear_damping(&self, damping: Real) -> Self {
        self.chained(|b| b.linear_damping(damping))
    }
    /// Set the gravity scale of the particles.
    fn gravity_scale(&self, scale: Real) -> Self {
        self.chained(|b| b.gravity_scale(scale))
    }
    /// Extra solver substeps requested for the body and everything it touches.
    fn additional_solver_iterations(&self, iterations: usize) -> Self {
        self.chained(|b| b.additional_solver_iterations(iterations))
    }
    /// Extra internal PGS iterations per substep for the body and everything it touches
    /// (default ``3``).
    fn additional_pgs_iterations(&self, iterations: usize) -> Self {
        self.chained(|b| b.additional_pgs_iterations(iterations))
    }
    /// Whether the body may fall asleep.
    fn can_sleep(&self, can_sleep: bool) -> Self {
        self.chained(|b| b.can_sleep(can_sleep))
    }
    /// Arbitrary integer user data.
    fn user_data(&self, data: u128) -> Self {
        self.chained(|b| b.user_data(data))
    }
    /// Append the particles and elements of another builder.
    fn append(&self, other: &SoftBodyBuilder) -> Self {
        let o = other.builder.clone();
        self.chained(|b| b.append(o))
    }

    fn __repr__(&self) -> String {
        format!(
            "SoftBodyBuilder(particles={}, edges={}, cells={})",
            self.builder.positions.len(),
            self.builder.edges.len(),
            self.builder.cells.len()
        )
    }
}

// ----------------------------------------------------------------------
// Element snapshots.
// ----------------------------------------------------------------------

/// A snapshot of one particle of a soft body.
#[pyclass(name = "SoftBodyParticle", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy)]
pub struct SoftBodyParticle(pub rapier::dynamics::SoftBodyParticle);

#[pymethods]
impl SoftBodyParticle {
    /// World-space position.
    #[getter]
    fn position(&self) -> Vec3 {
        vec3(self.0.position())
    }
    /// Velocity.
    #[getter]
    fn velocity(&self) -> Vec3 {
        vec3(self.0.velocity())
    }
    /// User force accumulated for the next step.
    #[getter]
    fn force(&self) -> Vec3 {
        vec3(self.0.force())
    }
    /// The position the particle is driven to over the next step, if any.
    #[getter]
    fn kinematic_target(&self) -> Option<Vec3> {
        self.0.kinematic_target().map(vec3)
    }
    /// The rest position (the position its constraints hold it at).
    #[getter]
    fn rest_position(&self) -> Vec3 {
        vec3(self.0.rest_position())
    }
    /// The rest position before any plastic flow.
    #[getter]
    fn initial_rest_position(&self) -> Vec3 {
        vec3(self.0.initial_rest_position())
    }
    /// Mass.
    #[getter]
    fn mass(&self) -> Real {
        self.0.mass()
    }
    /// Inverse mass (``0`` for a pinned particle).
    #[getter]
    fn inv_mass(&self) -> Real {
        self.0.inv_mass()
    }
    /// Is the particle pinned?
    #[getter]
    fn is_pinned(&self) -> bool {
        self.0.is_pinned()
    }
    /// Has an element of the particle torn?
    #[getter]
    fn is_damaged(&self) -> bool {
        self.0.is_damaged()
    }
    /// Does the particle lie on the body's surface?
    #[getter]
    fn is_on_surface(&self) -> bool {
        self.0.is_on_surface()
    }
    fn __repr__(&self) -> String {
        let p = self.0.position();
        format!(
            "SoftBodyParticle(position=({}, {}, {}), mass={})",
            p.x,
            p.y,
            p.z,
            self.0.mass()
        )
    }
}

/// A snapshot of one edge of a soft body.
#[pyclass(name = "SoftBodyEdge", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy)]
pub struct SoftBodyEdge(pub rapier::dynamics::SoftBodyEdge);

#[pymethods]
impl SoftBodyEdge {
    /// The two particles.
    #[getter]
    fn vertices(&self) -> (u32, u32) {
        (self.0.vertices[0], self.0.vertices[1])
    }
    /// Rest length.
    #[getter]
    fn rest_length(&self) -> Real {
        self.0.rest_length
    }
    /// Structural or bending.
    #[getter]
    fn kind(&self) -> SoftBodyEdgeKind {
        match self.0.kind {
            rapier::dynamics::SoftBodyEdgeKind::Structural => SoftBodyEdgeKind::STRUCTURAL,
            rapier::dynamics::SoftBodyEdgeKind::Bend => SoftBodyEdgeKind::BEND,
        }
    }
    /// Does the edge resist stretching only?
    #[getter]
    fn tension_only(&self) -> bool {
        self.0.tension_only
    }
    /// Per-edge softness override, if any.
    #[getter]
    fn softness(&self) -> Option<SpringCoefficients> {
        self.0.softness.map(SpringCoefficients)
    }
    /// Multiplier of the tear thresholds.
    #[getter]
    fn tear_resistance(&self) -> Real {
        self.0.tear_resistance
    }
    /// Impulse applied during the last step.
    #[getter]
    fn impulse(&self) -> Real {
        self.0.impulse()
    }
    /// Load relative to the tear threshold (``1`` tears).
    #[getter]
    fn stress(&self) -> Real {
        self.0.stress()
    }
    /// Relative change of the rest length due to plastic flow.
    #[getter]
    fn plastic_strain(&self) -> Real {
        self.0.plastic_strain()
    }
    /// The rest length before any plastic flow.
    #[getter]
    fn initial_rest_length(&self) -> Real {
        self.0.initial_rest_length()
    }
    fn __repr__(&self) -> String {
        format!(
            "SoftBodyEdge(vertices=({}, {}), rest_length={})",
            self.0.vertices[0], self.0.vertices[1], self.0.rest_length
        )
    }
}

/// A snapshot of one dihedral bending constraint of a soft body.
#[pyclass(name = "SoftBodyDihedral", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy)]
pub struct SoftBodyDihedral(pub rapier::dynamics::SoftBodyDihedral);

#[pymethods]
impl SoftBodyDihedral {
    /// The four particles: the shared edge, then the two opposite vertices.
    #[getter]
    fn vertices(&self) -> (u32, u32, u32, u32) {
        let v = self.0.vertices;
        (v[0], v[1], v[2], v[3])
    }
    /// The rest angle between the two triangle normals.
    #[getter]
    fn rest_angle(&self) -> Real {
        self.0.rest_angle
    }
    /// The permanent set of the rest angle due to plastic flow, in radians.
    #[getter]
    fn plastic_set(&self) -> Real {
        self.0.plastic_set()
    }
    /// The rest angle before any plastic flow.
    #[getter]
    fn initial_rest_angle(&self) -> Real {
        self.0.initial_rest_angle()
    }
}

/// A snapshot of one cell (tetrahedron) of a soft body.
#[pyclass(name = "SoftBodyCell", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy)]
pub struct SoftBodyCell(pub rapier::dynamics::SoftBodyCell);

#[pymethods]
impl SoftBodyCell {
    /// The four particles.
    #[getter]
    fn vertices(&self) -> (u32, u32, u32, u32) {
        let v = self.0.vertices;
        (v[0], v[1], v[2], v[3])
    }
    /// Rest volume.
    #[getter]
    fn rest_volume(&self) -> Real {
        self.0.rest_volume
    }
    /// Stiffness multiplier.
    #[getter]
    fn stiffness_scale(&self) -> Real {
        self.0.stiffness_scale
    }
    /// Multiplier of the tear thresholds.
    #[getter]
    fn tear_resistance(&self) -> Real {
        self.0.tear_resistance
    }
    /// Load relative to the tear threshold (``1`` tears).
    #[getter]
    fn stress(&self) -> Real {
        self.0.stress()
    }
    /// The accumulated plastic stretch of the rest shape, as a ``(3, 3)`` ndarray.
    fn plastic_stretch<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<Real>> {
        let m = self.0.plastic_stretch();
        let rows: Vec<Vec<Real>> = (0..3)
            .map(|i| (0..3).map(|j| m.col(j)[i]).collect())
            .collect();
        PyArray2::from_vec2_bound(py, &rows).expect("contiguous ndarray")
    }
}

/// A particle attached to a rigid body.
#[pyclass(name = "SoftParticleAttachment", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy)]
pub struct SoftParticleAttachment(pub rapier::dynamics::SoftParticleAttachment);

#[pymethods]
impl SoftParticleAttachment {
    /// The attached particle.
    #[getter]
    fn particle(&self) -> u32 {
        self.0.particle
    }
    /// The rigid body it follows.
    #[getter]
    fn body(&self) -> RigidBodyHandle {
        RigidBodyHandle(self.0.body)
    }
    /// The attachment point in the body's local frame.
    #[getter]
    fn local_anchor(&self) -> Vec3 {
        vec3(self.0.local_anchor)
    }
    /// The impulse the attachment applied during the last step.
    #[getter]
    fn impulse(&self) -> Vec3 {
        vec3(self.0.impulse())
    }
}

/// A piece of material enclosed by a closed surface, whose volume is preserved.
#[pyclass(name = "SoftVolumePiece", module = "rapier", frozen)]
#[derive(Debug, Clone)]
pub struct SoftVolumePiece {
    #[pyo3(get)]
    elements: Vec<u32>,
    #[pyo3(get)]
    particles: Vec<u32>,
    #[pyo3(get)]
    rest_volume: Real,
    #[pyo3(get)]
    volume: Real,
}

// ----------------------------------------------------------------------
// Clusters and collision meshes.
// ----------------------------------------------------------------------

/// A snapshot of a soft body's cluster: a set of particles with a rigid proxy that joints and
/// colliders attach to.
#[pyclass(name = "SoftBodyCluster", module = "rapier", frozen)]
#[derive(Debug, Clone)]
pub struct SoftBodyCluster {
    /// The cluster's index in its soft body.
    #[pyo3(get)]
    index: u32,
    /// The particles of the cluster.
    #[pyo3(get)]
    particles: Vec<u32>,
    /// The rigid-body proxy standing for the cluster.
    #[pyo3(get)]
    proxy: RigidBodyHandle,
    /// Does the cluster still exist?
    #[pyo3(get)]
    is_live: bool,
    /// Does shape matching hold the cluster's shape?
    #[pyo3(get)]
    shape_matching_enabled: bool,
    /// The identifiers of the collision meshes the cluster holds.
    #[pyo3(get)]
    meshes: Vec<SoftMeshId>,
}

#[pymethods]
impl SoftBodyCluster {
    fn __repr__(&self) -> String {
        format!(
            "SoftBodyCluster(index={}, particles={}, live={})",
            self.index,
            self.particles.len(),
            self.is_live
        )
    }
}

/// The identifier of a collision mesh within its soft body: the cluster holding it and its
/// index in that cluster.
#[pyclass(name = "SoftMeshId", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SoftMeshId(pub rapier::dynamics::SoftMeshId);

#[pymethods]
impl SoftMeshId {
    #[new]
    fn new(cluster: u32, mesh: u32) -> Self {
        Self(rapier::dynamics::SoftMeshId { cluster, mesh })
    }
    #[getter]
    fn cluster(&self) -> u32 {
        self.0.cluster
    }
    #[getter]
    fn mesh(&self) -> u32 {
        self.0.mesh
    }
    fn __repr__(&self) -> String {
        format!(
            "SoftMeshId(cluster={}, mesh={})",
            self.0.cluster, self.0.mesh
        )
    }
    fn __eq__(&self, other: &Self) -> bool {
        self.0 == other.0
    }
    fn __hash__(&self) -> u64 {
        ((self.0.cluster as u64) << 32) | self.0.mesh as u64
    }
}

/// A collision mesh of a soft body (a deformable collider), by body and identifier.
#[pyclass(name = "SoftMeshRef", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy)]
pub struct SoftMeshRef(pub rapier::dynamics::SoftMeshRef);

#[pymethods]
impl SoftMeshRef {
    /// The soft body holding the mesh.
    #[getter]
    fn body(&self) -> SoftBodyHandle {
        SoftBodyHandle(self.0.body)
    }
    /// The mesh's identifier within the body.
    #[getter]
    fn id(&self) -> SoftMeshId {
        SoftMeshId(self.0.id)
    }
    fn __repr__(&self) -> String {
        format!(
            "SoftMeshRef(body={:?}, cluster={}, mesh={})",
            self.0.body.into_raw_parts(),
            self.0.id.cluster,
            self.0.id.mesh
        )
    }
}

/// A snapshot of a soft body's collision mesh: the triangles a cluster collides through, with
/// their current world-space vertex positions.
#[pyclass(name = "SoftCollisionMesh", module = "rapier", frozen)]
#[derive(Debug, Clone)]
pub struct SoftCollisionMesh {
    /// The mesh's identifier within its body.
    #[pyo3(get)]
    id: SoftMeshId,
    /// The collider holding the mesh.
    #[pyo3(get)]
    collider: ColliderHandle,
    /// Does the mesh collide?
    #[pyo3(get)]
    collision_enabled: bool,
    /// Is the mesh embedded in the cells (rather than bound vertex-to-particle)?
    #[pyo3(get)]
    is_skinned: bool,
    /// Is the surface closed?
    #[pyo3(get)]
    is_closed: bool,
    /// Is the mesh made of segments (a wire)?
    #[pyo3(get)]
    is_wire: bool,
    /// Does the mesh collide with itself?
    #[pyo3(get)]
    self_contacts_enabled: bool,
    /// Does the mesh's collider shape carry the ``ORIENTED`` flag (see
    /// :meth:`SoftBodyBuilder.oriented`)?
    #[pyo3(get)]
    is_oriented: bool,
    vertices: Vec<rapier::math::Vector>,
    indices: Vec<Vec<u32>>,
}

#[pymethods]
impl SoftCollisionMesh {
    /// The number of vertices.
    #[getter]
    fn vertex_count(&self) -> usize {
        self.vertices.len()
    }
    /// The world-space vertex positions as an ``(N, 3)`` ndarray.
    #[getter]
    fn vertices<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<Real>> {
        vectors_to_array(py, self.vertices.iter().copied())
    }
    /// The elements as an ``(M, 3)`` ndarray (``(M, 2)`` for a wire).
    #[getter]
    fn indices<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        let cols = self.indices.first().map_or(3, |e| e.len());
        if self.indices.is_empty() {
            PyArray2::zeros_bound(py, [0, cols], false)
        } else {
            PyArray2::from_vec2_bound(py, &self.indices).expect("contiguous ndarray")
        }
    }
    fn __repr__(&self) -> String {
        format!(
            "SoftCollisionMesh(id={:?}, vertices={}, elements={})",
            (self.id.0.cluster, self.id.0.mesh),
            self.vertices.len(),
            self.indices.len()
        )
    }
}

impl SoftCollisionMesh {
    fn from_rapier(
        body: &rapier::dynamics::SoftBody,
        mesh: &rapier::dynamics::SoftCollisionMesh,
    ) -> Self {
        let indices = if mesh.is_wire() {
            (0..mesh.indices().len())
                .map(|i| mesh.element(i).to_vec())
                .collect()
        } else {
            mesh.indices().iter().map(|e| e.to_vec()).collect()
        };
        Self {
            id: SoftMeshId(mesh.id()),
            collider: ColliderHandle(mesh.collider()),
            collision_enabled: mesh.collision_enabled(),
            is_skinned: mesh.is_skinned(),
            is_closed: mesh.is_closed(),
            is_wire: mesh.is_wire(),
            self_contacts_enabled: mesh.self_contacts_enabled(),
            is_oriented: mesh.is_oriented(),
            vertices: mesh.vertex_positions(body).collect(),
            indices,
        }
    }
}

/// How a deformable collider's vertices follow the particles of its cluster (see
/// :meth:`ColliderSet.insert_deformable`).
#[pyclass(name = "SoftMeshBinding", module = "rapier")]
#[derive(Debug, Clone)]
pub struct SoftMeshBinding(pub rapier::dynamics::SoftMeshBinding);

#[pymethods]
impl SoftMeshBinding {
    /// Bind vertex ``i`` of the mesh to ``particles[i]``, which must belong to the cluster.
    #[staticmethod]
    fn direct(particles: &Bound<'_, PyAny>) -> PyResult<Self> {
        Ok(Self(rapier::dynamics::SoftMeshBinding::direct(
            extract_index_list(particles)?,
        )))
    }
    /// Bind every vertex to the particle of the cluster closest to it, within ``eps``.
    #[staticmethod]
    fn direct_by_position(eps: Real) -> Self {
        Self(rapier::dynamics::SoftMeshBinding::direct_by_position(eps))
    }
    /// Bind every vertex to the cell of the cluster holding it (cage simulation).
    #[staticmethod]
    fn skinned() -> Self {
        Self(rapier::dynamics::SoftMeshBinding::skinned())
    }
    /// Enable collisions of the mesh with itself.
    fn self_contacts(&self, enabled: bool) -> Self {
        Self(self.0.clone().self_contacts(enabled))
    }
}

// ----------------------------------------------------------------------
// Tear events.
// ----------------------------------------------------------------------

/// A soft body a tear left: the torn body itself, or a body split off it (see
/// :attr:`SoftBodyTearEvent.pieces`).
#[pyclass(name = "SoftBodyPiece", module = "rapier", frozen)]
#[derive(Debug, Clone)]
pub struct SoftBodyPiece(pub rapier::dynamics::SoftBodyPiece);

#[pymethods]
impl SoftBodyPiece {
    /// The soft body holding the piece.
    #[getter]
    fn soft_body(&self) -> SoftBodyHandle {
        SoftBodyHandle(self.0.soft_body)
    }
    /// The particles of the piece: ``particles[i]`` is the index, in the torn body after the
    /// tear (the indices the other fields of the event use), of the piece's ``i``-th particle.
    #[getter]
    fn particles(&self) -> Vec<u32> {
        self.0.particles.clone()
    }
    /// The clusters of the piece, as ``(index in the torn body before the split, index in the
    /// piece)`` pairs.
    #[getter]
    fn clusters(&self) -> Vec<(u32, u32)> {
        self.0.clusters.iter().map(|c| (c[0], c[1])).collect()
    }
}

/// A cluster split by a tear.
#[pyclass(name = "SoftClusterSplit", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy)]
pub struct SoftClusterSplit(pub rapier::dynamics::SoftClusterSplit);

#[pymethods]
impl SoftClusterSplit {
    /// The cluster of the torn body the split came from.
    #[getter]
    fn source_cluster(&self) -> u32 {
        self.0.source_cluster
    }
    /// The soft body holding the new cluster.
    #[getter]
    fn soft_body(&self) -> SoftBodyHandle {
        SoftBodyHandle(self.0.soft_body)
    }
    /// The index of the new cluster.
    #[getter]
    fn cluster(&self) -> u32 {
        self.0.cluster
    }
    /// The proxy of the new cluster.
    #[getter]
    fn proxy(&self) -> RigidBodyHandle {
        RigidBodyHandle(self.0.proxy)
    }
    /// Does the new cluster keep the source cluster's proxy?
    #[getter]
    fn keeps_proxy(&self) -> bool {
        self.0.keeps_proxy
    }
}

/// An impulse joint a tear moved to another proxy.
#[pyclass(name = "SoftJointMove", module = "rapier", frozen)]
#[derive(Debug, Clone, Copy)]
pub struct SoftJointMove(pub rapier::dynamics::SoftJointMove);

#[pymethods]
impl SoftJointMove {
    #[getter]
    fn joint(&self) -> ImpulseJointHandle {
        ImpulseJointHandle(self.0.joint)
    }
    #[getter]
    fn from_body(&self) -> RigidBodyHandle {
        RigidBodyHandle(self.0.from)
    }
    #[getter]
    fn to_body(&self) -> RigidBodyHandle {
        RigidBodyHandle(self.0.to)
    }
}

/// The record of a soft body tearing: the elements it lost, the particles the tear split,
/// the pieces that became soft bodies of their own, and the clusters and joints that moved
/// with them.
#[pyclass(name = "SoftBodyTearEvent", module = "rapier", frozen)]
#[derive(Debug, Clone)]
pub struct SoftBodyTearEvent(pub rapier::dynamics::SoftBodyTearEvent);

#[pymethods]
impl SoftBodyTearEvent {
    /// The soft body that tore.
    #[getter]
    fn soft_body(&self) -> SoftBodyHandle {
        SoftBodyHandle(self.0.soft_body)
    }
    /// The particle pairs of the edges that tore, as an ``(E, 2)`` ndarray.
    #[getter]
    fn torn_edges<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.0.torn_edges)
    }
    /// The particles of the cells that tore, as a ``(C, 4)`` ndarray.
    #[getter]
    fn torn_cells<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.0.torn_cells)
    }
    /// The particle pairs of the edges the tear removed, as an ``(E, 2)`` ndarray.
    #[getter]
    fn removed_edges<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.0.removed_edges)
    }
    /// The particles the tear passed through, duplicated one copy per piece, as ``(copy,
    /// source)`` pairs.
    #[getter]
    fn split_particles(&self) -> Vec<(u32, u32)> {
        self.0.split_particles.clone()
    }
    /// The particles the tear inserted.
    #[getter]
    fn inserted_particles(&self) -> Vec<u32> {
        self.0.inserted_particles.clone()
    }
    /// The soft bodies the torn body came apart into: the piece that keeps the torn body's handle
    /// (the one with the largest rest measure) first, then the new soft bodies split off it.
    /// Empty when nothing split off.
    #[getter]
    fn pieces(&self) -> Vec<SoftBodyPiece> {
        self.0.pieces.iter().cloned().map(SoftBodyPiece).collect()
    }
    /// The clusters the tear split.
    #[getter]
    fn clusters(&self) -> Vec<SoftClusterSplit> {
        self.0
            .clusters
            .iter()
            .copied()
            .map(SoftClusterSplit)
            .collect()
    }
    /// The impulse joints the tear moved to another proxy.
    #[getter]
    fn moved_joints(&self) -> Vec<SoftJointMove> {
        self.0
            .moved_joints
            .iter()
            .copied()
            .map(SoftJointMove)
            .collect()
    }
    /// The soft bodies the torn body is in after the tear, the one keeping its handle first: the
    /// torn body alone when nothing split off, the bodies of :attr:`pieces` otherwise.
    fn bodies(&self) -> Vec<SoftBodyHandle> {
        self.0.bodies().map(SoftBodyHandle).collect()
    }
    /// Where a particle of the torn body is after the tear: ``(soft_body, index)``, or
    /// ``None`` if it was removed.
    fn particle_destination(&self, particle: u32) -> Option<(SoftBodyHandle, u32)> {
        self.0
            .particle_destination(particle)
            .map(|(h, i)| (SoftBodyHandle(h), i))
    }
    /// The particle pairs the tear started from, as an ``(S, 2)`` ndarray.
    fn seeds<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
        elements_to_array(py, &self.0.seeds())
    }
    fn __repr__(&self) -> String {
        format!(
            "SoftBodyTearEvent(soft_body={:?}, torn_edges={}, torn_cells={}, pieces={})",
            self.0.soft_body.into_raw_parts(),
            self.0.torn_edges.len(),
            self.0.torn_cells.len(),
            self.0.pieces.len()
        )
    }
}

// ----------------------------------------------------------------------
// SoftBody.
// ----------------------------------------------------------------------

/// Storage backing a `SoftBody`: an owned body (removed from a set) or a handle-backed view
/// into a `SoftBodySet`.
#[derive(Debug)]
pub enum SoftBodyBacking {
    Owned(Box<rapier::dynamics::SoftBody>),
    InSet {
        set: Py<SoftBodySet>,
        handle: rapier::dynamics::SoftBodyHandle,
    },
}

impl Clone for SoftBodyBacking {
    fn clone(&self) -> Self {
        match self {
            SoftBodyBacking::Owned(b) => SoftBodyBacking::Owned(b.clone()),
            SoftBodyBacking::InSet { set, handle } => {
                Python::with_gil(|py| SoftBodyBacking::InSet {
                    set: set.clone_ref(py),
                    handle: *handle,
                })
            }
        }
    }
}

/// The error raised by a view of a soft body removed from its set.
fn removed_body(handle: rapier::dynamics::SoftBodyHandle) -> PyErr {
    crate::errors::InvalidHandle::new_err(format!(
        "the soft body {:?} was removed from its set",
        handle.into_raw_parts()
    ))
}

/// A soft body: particles linked by edges, bending constraints and cells, simulated together
/// with the rigid bodies, contacts and joints of a world.
///
/// Instances returned by :class:`SoftBodySet` are live **views**: reads and writes go straight
/// through to the set, and raise :class:`InvalidHandle` once the body is removed from it. The
/// static constructors (:meth:`rope`, :meth:`cloth`, ...) return a
/// :class:`SoftBodyBuilder` to insert with :meth:`SoftBodySet.insert` or
/// :meth:`PhysicsWorld.add_soft_body`.
#[pyclass(name = "SoftBody", module = "rapier")]
#[derive(Debug, Clone)]
pub struct SoftBody {
    pub backing: SoftBodyBacking,
}

impl SoftBody {
    pub(crate) fn new_owned(body: rapier::dynamics::SoftBody) -> Self {
        SoftBody {
            backing: SoftBodyBacking::Owned(Box::new(body)),
        }
    }

    fn with_ref<R>(&self, f: impl FnOnce(&rapier::dynamics::SoftBody) -> R) -> PyResult<R> {
        match &self.backing {
            SoftBodyBacking::Owned(b) => Ok(f(b)),
            SoftBodyBacking::InSet { set, handle } => Python::with_gil(|py| {
                SoftBodySet::read(set.bind(py), |set| set.get(*handle).map(f))?
                    .ok_or_else(|| removed_body(*handle))
            }),
        }
    }

    fn with_mut<R>(&mut self, f: impl FnOnce(&mut rapier::dynamics::SoftBody) -> R) -> PyResult<R> {
        match &mut self.backing {
            SoftBodyBacking::Owned(b) => Ok(f(b)),
            SoftBodyBacking::InSet { set, handle } => Python::with_gil(|py| {
                let mut set = set
                    .bind(py)
                    .try_borrow_mut()
                    .map_err(|_| crate::events_hooks::stepping_error("SoftBodySet"))?;
                let body = set
                    .0
                    .get_mut(*handle)
                    .ok_or_else(|| removed_body(*handle))?;
                Ok(f(body))
            }),
        }
    }

    /// One vector per particle, from an `(N, 3)` ndarray or a sequence of 3-tuples.
    fn extract_per_particle(&self, obj: &Bound<'_, PyAny>) -> PyResult<Vec<rapier::math::Vector>> {
        let vectors = crate::geometry::extract_verts_for_dim(obj)?;
        let n = self.with_ref(|b| b.num_particles())?;
        if vectors.len() != n {
            return Err(PyValueError::new_err(format!(
                "expected one row per particle ({n}); got {}",
                vectors.len()
            )));
        }
        Ok(vectors)
    }

    fn check_index(
        &self,
        what: &str,
        i: usize,
        len: impl FnOnce(&rapier::dynamics::SoftBody) -> usize,
    ) -> PyResult<()> {
        let n = self.with_ref(len)?;
        if i < n {
            Ok(())
        } else {
            Err(PyIndexError::new_err(format!(
                "{what} index {i} out of range (the body has {n})"
            )))
        }
    }

    fn check_particle(&self, i: usize) -> PyResult<()> {
        let n = self.with_ref(|b| b.num_particles())?;
        if i < n {
            Ok(())
        } else {
            Err(PyIndexError::new_err(format!(
                "particle index {i} out of range (the body has {n} particles)"
            )))
        }
    }
}

#[pymethods]
impl SoftBody {
    /*
     * Generators: each returns a builder, with the builder's keywords accepted.
     */

    /// A rope of ``num_particles`` particles from ``start`` to ``end``.
    #[staticmethod]
    #[pyo3(signature = (start, end, num_particles, **kwargs))]
    fn rope(
        start: PyVector,
        end: PyVector,
        num_particles: usize,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<SoftBodyBuilder> {
        SoftBodyBuilder::from_kwargs(
            rapier::dynamics::SoftBodyBuilder::rope(start.0.into(), end.0.into(), num_particles),
            kwargs,
        )
    }

    /// A cloth of ``nu`` by ``nv`` particles: particle ``(i, j)`` is at
    /// ``origin + i * du + j * dv``.
    #[staticmethod]
    #[pyo3(signature = (origin, du, dv, nu, nv, **kwargs))]
    fn cloth(
        origin: PyVector,
        du: PyVector,
        dv: PyVector,
        nu: usize,
        nv: usize,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<SoftBodyBuilder> {
        SoftBodyBuilder::from_kwargs(
            rapier::dynamics::SoftBodyBuilder::cloth(
                origin.0.into(),
                du.0.into(),
                dv.0.into(),
                nu,
                nv,
            ),
            kwargs,
        )
    }

    /// A tube of cloth around ``axis``, from ``radius_start`` at ``origin`` to ``radius_end``
    /// at its other end, with ``num_around`` particles per ring and ``num_along`` rings.
    #[staticmethod]
    #[pyo3(signature = (origin, axis, radius_start, radius_end, num_around, num_along, **kwargs))]
    #[allow(clippy::too_many_arguments)]
    fn cloth_tube(
        origin: PyVector,
        axis: PyVector,
        radius_start: Real,
        radius_end: Real,
        num_around: usize,
        num_along: usize,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<SoftBodyBuilder> {
        SoftBodyBuilder::from_kwargs(
            rapier::dynamics::SoftBodyBuilder::cloth_tube(
                origin.0.into(),
                axis.0.into(),
                radius_start,
                radius_end,
                num_around,
                num_along,
            ),
            kwargs,
        )
    }

    /// A cloth with different softness along ``du`` (warp), along ``dv`` (weft) and across the
    /// diagonals (shear); each softness is a :class:`SpringCoefficients` or a
    /// ``(frequency, damping)`` tuple.
    #[staticmethod]
    #[pyo3(signature = (origin, du, dv, nu, nv, warp, weft, shear, **kwargs))]
    #[allow(clippy::too_many_arguments)]
    fn cloth_anisotropic(
        origin: PyVector,
        du: PyVector,
        dv: PyVector,
        nu: usize,
        nv: usize,
        warp: &Bound<'_, PyAny>,
        weft: &Bound<'_, PyAny>,
        shear: &Bound<'_, PyAny>,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<SoftBodyBuilder> {
        SoftBodyBuilder::from_kwargs(
            rapier::dynamics::SoftBodyBuilder::cloth_anisotropic(
                origin.0.into(),
                du.0.into(),
                dv.0.into(),
                nu,
                nv,
                spring(warp)?,
                spring(weft)?,
                spring(shear)?,
            ),
            kwargs,
        )
    }

    /// A box of ``nx`` by ``ny`` by ``nz`` particles filled with tetrahedral cells.
    #[staticmethod]
    #[pyo3(signature = (center, half_extents, nx, ny, nz, **kwargs))]
    fn cuboid(
        center: PyVector,
        half_extents: PyVector,
        nx: usize,
        ny: usize,
        nz: usize,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<SoftBodyBuilder> {
        SoftBodyBuilder::from_kwargs(
            rapier::dynamics::SoftBodyBuilder::cuboid(
                center.0.into(),
                half_extents.0.into(),
                nx,
                ny,
                nz,
            ),
            kwargs,
        )
    }

    /// A hollow sphere: an icosphere surface with ``subdivisions`` refinement levels, holding
    /// its volume (a balloon).
    #[staticmethod]
    #[pyo3(signature = (center, radius, subdivisions, **kwargs))]
    fn sphere(
        center: PyVector,
        radius: Real,
        subdivisions: usize,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<SoftBodyBuilder> {
        SoftBodyBuilder::from_kwargs(
            rapier::dynamics::SoftBodyBuilder::sphere(center.0.into(), radius, subdivisions),
            kwargs,
        )
    }

    /// A triangle-mesh body without cells: the vertices are particles, the triangles the
    /// surface, held by dihedral bending and shape matching.
    ///
    /// :raises MeshConversionError: if the mesh is empty.
    #[staticmethod]
    #[pyo3(signature = (vertices, indices, **kwargs))]
    fn trimesh(
        vertices: &Bound<'_, PyAny>,
        indices: &Bound<'_, PyAny>,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<SoftBodyBuilder> {
        let vertices = crate::geometry::extract_verts_for_dim(vertices)?;
        let indices = extract_index_rows::<3>(indices)?;
        let builder = rapier::dynamics::SoftBodyBuilder::trimesh(vertices, indices)
            .ok_or_else(|| crate::errors::MeshConversionError::new_err("empty triangle mesh"))?;
        SoftBodyBuilder::from_kwargs(builder, kwargs)
    }

    /// A body filling the closed triangle surface with tetrahedral cells of the given size;
    /// ``skinned`` keeps the surface as a skin embedded in the cells.
    ///
    /// :raises MeshConversionError: if the surface cannot be meshed.
    #[staticmethod]
    #[pyo3(signature = (vertices, indices, cell_size, skinned=false, **kwargs))]
    fn volumetric(
        vertices: &Bound<'_, PyAny>,
        indices: &Bound<'_, PyAny>,
        cell_size: Real,
        skinned: bool,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<SoftBodyBuilder> {
        let vertices = crate::geometry::extract_verts_for_dim(vertices)?;
        let indices = extract_index_rows::<3>(indices)?;
        let builder = if skinned {
            rapier::dynamics::SoftBodyBuilder::volumetric_skinned(&vertices, &indices, cell_size)
        } else {
            rapier::dynamics::SoftBodyBuilder::volumetric(&vertices, &indices, cell_size)
        }
        .ok_or_else(|| {
            crate::errors::MeshConversionError::new_err(
                "the surface could not be filled with cells (is it closed?)",
            )
        })?;
        SoftBodyBuilder::from_kwargs(builder, kwargs)
    }

    /// The same as :meth:`volumetric`, with the meshing parameters spelled out (see
    /// :class:`VolumeMeshParameters`); ``skinned`` keeps the surface as a skin embedded in the
    /// cells.
    ///
    /// :raises MeshConversionError: if the surface cannot be meshed.
    #[staticmethod]
    #[pyo3(signature = (vertices, indices, params, skinned=false, **kwargs))]
    fn volumetric_with(
        vertices: &Bound<'_, PyAny>,
        indices: &Bound<'_, PyAny>,
        params: &VolumeMeshParameters,
        skinned: bool,
        kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<SoftBodyBuilder> {
        let vertices = crate::geometry::extract_verts_for_dim(vertices)?;
        let indices = extract_index_rows::<3>(indices)?;
        let mut builder =
            rapier::dynamics::SoftBodyBuilder::volumetric_with(&vertices, &indices, &params.0)
                .ok_or_else(|| {
                    crate::errors::MeshConversionError::new_err(
                        "the surface could not be filled with cells (is it closed?)",
                    )
                })?;
        if skinned {
            builder = builder.skin(vertices, indices);
        }
        SoftBodyBuilder::from_kwargs(builder, kwargs)
    }

    /*
     * Particles.
     */

    /// A counter incremented by every change of the body's topology (tears, cuts).
    #[getter]
    fn topology_version(&self) -> PyResult<u32> {
        self.with_ref(|b| b.topology_version())
    }
    /// The number of particles.
    #[getter]
    fn num_particles(&self) -> PyResult<usize> {
        self.with_ref(|b| b.num_particles())
    }
    /// A snapshot of the ``i``-th particle.
    fn particle(&self, i: usize) -> PyResult<SoftBodyParticle> {
        self.check_particle(i)?;
        self.with_ref(|b| SoftBodyParticle(b.particles()[i]))
    }
    /// Snapshots of every particle.
    fn particles(&self) -> PyResult<Vec<SoftBodyParticle>> {
        self.with_ref(|b| {
            b.particles()
                .iter()
                .copied()
                .map(SoftBodyParticle)
                .collect()
        })
    }
    /// The world-space position of the ``i``-th particle.
    fn particle_position(&self, i: usize) -> PyResult<Vec3> {
        self.check_particle(i)?;
        self.with_ref(|b| vec3(b.particle_position(i)))
    }
    /// The world-space positions of every particle as an ``(N, 3)`` ndarray.
    #[getter]
    fn particle_positions<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<Real>>> {
        self.with_ref(|b| vectors_to_array(py, b.particle_positions()))
    }
    /// Teleport every particle (no velocity change), from an ``(N, 3)`` ndarray or a sequence
    /// of 3-tuples with one row per particle.
    #[setter]
    fn set_particle_positions(&mut self, positions: &Bound<'_, PyAny>) -> PyResult<()> {
        let positions = self.extract_per_particle(positions)?;
        self.with_mut(|b| {
            for (i, p) in positions.into_iter().enumerate() {
                b.set_particle_position(i, p);
            }
        })
    }
    /// The velocity of the ``i``-th particle.
    fn particle_velocity(&self, i: usize) -> PyResult<Vec3> {
        self.check_particle(i)?;
        self.with_ref(|b| vec3(b.particle_velocity(i)))
    }
    /// The velocities of every particle as an ``(N, 3)`` ndarray.
    #[getter]
    fn particle_velocities<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<Real>>> {
        self.with_ref(|b| vectors_to_array(py, b.particle_velocities()))
    }
    /// Set the velocity of every particle, from an ``(N, 3)`` ndarray or a sequence of 3-tuples
    /// with one row per particle.
    #[setter]
    fn set_particle_velocities(&mut self, velocities: &Bound<'_, PyAny>) -> PyResult<()> {
        let velocities = self.extract_per_particle(velocities)?;
        self.with_mut(|b| {
            for (i, v) in velocities.into_iter().enumerate() {
                b.set_particle_velocity(i, v);
            }
        })
    }
    /// Set the world-space position of the ``i``-th particle.
    fn set_particle_position(&mut self, i: usize, position: PyVector) -> PyResult<()> {
        self.check_particle(i)?;
        self.with_mut(|b| b.set_particle_position(i, position.0.into()))?;
        Ok(())
    }
    /// Set the velocity of the ``i``-th particle.
    fn set_particle_velocity(&mut self, i: usize, velocity: PyVector) -> PyResult<()> {
        self.check_particle(i)?;
        self.with_mut(|b| b.set_particle_velocity(i, velocity.0.into()))?;
        Ok(())
    }
    /// Move the pinned ``i``-th particle to ``position`` over the next step, like a
    /// position-based kinematic body (it pushes what it meets), then hold it there. Ignored for
    /// a free particle.
    fn set_particle_kinematic_target(&mut self, i: usize, position: PyVector) -> PyResult<()> {
        self.check_particle(i)?;
        self.with_mut(|b| b.set_particle_kinematic_target(i, position.0.into()))?;
        Ok(())
    }
    /// Pin (or release) the ``i``-th particle.
    fn set_particle_pinned(&mut self, i: usize, pinned: bool) -> PyResult<()> {
        self.check_particle(i)?;
        self.with_mut(|b| b.set_particle_pinned(i, pinned))?;
        Ok(())
    }
    /// Attach the ``i``-th particle to a rigid body, at the particle's current position.
    fn attach_particle(
        &mut self,
        i: usize,
        body: &RigidBodyHandle,
        bodies: &RigidBodySet,
    ) -> PyResult<()> {
        self.check_particle(i)?;
        self.with_mut(|b| b.attach_particle(i, body.0, &bodies.0))?;
        Ok(())
    }
    /// Detach the ``i``-th particle from the rigid body it follows; ``False`` if it was not
    /// attached.
    fn detach_particle(&mut self, i: usize) -> PyResult<bool> {
        self.with_mut(|b| b.detach_particle(i))
    }
    /// The particles attached to rigid bodies.
    #[getter]
    fn particle_attachments(&self) -> PyResult<Vec<SoftParticleAttachment>> {
        self.with_ref(|b| {
            b.particle_attachments()
                .iter()
                .copied()
                .map(SoftParticleAttachment)
                .collect()
        })
    }

    /*
     * Elements.
     */

    /// The number of edges (structural and bending).
    #[getter]
    fn num_edges(&self) -> PyResult<usize> {
        self.with_ref(|b| b.edges().len())
    }
    /// A snapshot of the ``i``-th edge.
    fn edge(&self, i: usize) -> PyResult<SoftBodyEdge> {
        self.with_ref(|b| {
            b.edges()
                .get(i)
                .copied()
                .map(SoftBodyEdge)
                .ok_or_else(|| PyIndexError::new_err(format!("edge index {i} out of range")))
        })?
    }
    /// The particle pairs of every edge as an ``(E, 2)`` ndarray.
    #[getter]
    fn edges<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<u32>>> {
        self.with_ref(|b| {
            let e: Vec<[u32; 2]> = b.edges().iter().map(|e| e.vertices).collect();
            elements_to_array(py, &e)
        })
    }
    /// The number of dihedral bending constraints.
    #[getter]
    fn num_dihedrals(&self) -> PyResult<usize> {
        self.with_ref(|b| b.dihedrals().len())
    }
    /// A snapshot of the ``i``-th dihedral.
    fn dihedral(&self, i: usize) -> PyResult<SoftBodyDihedral> {
        self.with_ref(|b| {
            b.dihedrals()
                .get(i)
                .copied()
                .map(SoftBodyDihedral)
                .ok_or_else(|| PyIndexError::new_err(format!("dihedral index {i} out of range")))
        })?
    }
    /// The particles of every dihedral as a ``(D, 4)`` ndarray.
    #[getter]
    fn dihedrals<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<u32>>> {
        self.with_ref(|b| {
            let d: Vec<[u32; 4]> = b.dihedrals().iter().map(|d| d.vertices).collect();
            elements_to_array(py, &d)
        })
    }
    /// The number of cells (tetrahedra).
    #[getter]
    fn num_cells(&self) -> PyResult<usize> {
        self.with_ref(|b| b.cells().len())
    }
    /// A snapshot of the ``i``-th cell.
    fn cell(&self, i: usize) -> PyResult<SoftBodyCell> {
        self.with_ref(|b| {
            b.cells()
                .get(i)
                .copied()
                .map(SoftBodyCell)
                .ok_or_else(|| PyIndexError::new_err(format!("cell index {i} out of range")))
        })?
    }
    /// The particles of every cell as a ``(C, 4)`` ndarray.
    #[getter]
    fn cells<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<u32>>> {
        self.with_ref(|b| {
            let c: Vec<[u32; 4]> = b.cells().iter().map(|c| c.vertices).collect();
            elements_to_array(py, &c)
        })
    }
    /// The boundary triangles, oriented outward, as a ``(B, 3)`` ndarray.
    #[getter]
    fn boundary<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<u32>>> {
        self.with_ref(|b| elements_to_array(py, b.boundary()))
    }

    /*
     * Material and models.
     */

    /// The material, as a live view: setting one of its fields changes the body (use
    /// :meth:`SoftBodyMaterial.copy` for a detached copy). Assigning a :class:`SoftBodyMaterial`
    /// replaces the whole material.
    #[getter]
    fn material(slf: &Bound<'_, Self>) -> PyResult<SoftBodyMaterial> {
        slf.try_borrow()?.with_ref(|_| ())?;
        Ok(SoftBodyMaterial {
            backing: SoftBodyMaterialBacking::InBody(slf.clone().unbind()),
        })
    }
    #[setter]
    fn set_material(slf: &Bound<'_, Self>, material: &Bound<'_, SoftBodyMaterial>) -> PyResult<()> {
        // Read the value first: `material` may be a view of this very body.
        let m = material.try_borrow()?.get()?;
        slf.try_borrow_mut()?.with_mut(|b| b.set_material(m))
    }
    /// The solver simulating the body's elasticity.
    #[getter]
    fn solver(&self) -> PyResult<SoftBodySolver> {
        self.with_ref(|b| SoftBodySolver::from_rapier(b.solver()))
    }
    #[setter]
    fn set_solver(&mut self, solver: SoftBodySolver) -> PyResult<()> {
        self.with_mut(|b| b.set_solver(solver.to_rapier()))
    }
    /// The constitutive model of the cells.
    #[getter]
    fn cell_model(&self) -> PyResult<SoftBodyCellModel> {
        self.with_ref(|b| SoftBodyCellModel::from_rapier(b.cell_model()))
    }
    /// Is the volume enclosed by the body's closed surfaces preserved?
    #[getter]
    fn volume_preservation_enabled(&self) -> PyResult<bool> {
        self.with_ref(|b| b.volume_preservation_enabled())
    }
    /// Enable or disable the preservation of the enclosed volume.
    fn enable_volume_preservation(&mut self, enabled: bool) -> PyResult<()> {
        self.with_mut(|b| b.enable_volume_preservation(enabled))
    }
    /// The pieces of material whose volume is preserved.
    #[getter]
    fn volume_pieces(&self) -> PyResult<Vec<SoftVolumePiece>> {
        self.with_ref(|b| {
            b.volume_pieces()
                .iter()
                .map(|p| SoftVolumePiece {
                    elements: p.elements().to_vec(),
                    particles: p.particles().to_vec(),
                    rest_volume: p.rest_volume(),
                    volume: p.volume(b),
                })
                .collect()
        })
    }
    /// The rest volume enclosed by the closed surfaces.
    #[getter]
    fn rest_volume(&self) -> PyResult<Real> {
        self.with_ref(|b| b.rest_volume())
    }
    /// The current volume enclosed by the closed surfaces.
    #[getter]
    fn volume(&self) -> PyResult<Real> {
        self.with_ref(|b| b.volume())
    }
    /// The target volume multiplier (``> 1`` inflates the body).
    #[getter]
    fn volume_factor(&self) -> PyResult<Real> {
        self.with_ref(|b| b.volume_factor())
    }
    #[setter]
    fn set_volume_factor(&mut self, factor: Real) -> PyResult<()> {
        self.with_mut(|b| b.set_volume_factor(factor))
    }
    /// The thickness of the particles.
    #[getter]
    fn particle_radius(&self) -> PyResult<Real> {
        self.with_ref(|b| b.particle_radius())
    }
    /// The dynamics settings of the particles (a copy).
    #[getter]
    fn particle_settings(&self) -> PyResult<SoftBodyParticleSettings> {
        self.with_ref(|b| SoftBodyParticleSettings(*b.particle_settings()))
    }
    /// Forget every plastic deformation: the rest shapes return to their initial values.
    fn reset_plasticity(&mut self) -> PyResult<()> {
        self.with_mut(|b| b.reset_plasticity())
    }

    /*
     * Whole-body state.
     */

    /// The hidden rigid body standing for the whole soft body in joints and islands.
    #[getter]
    fn root_body(&self) -> PyResult<RigidBodyHandle> {
        self.with_ref(|b| RigidBodyHandle(b.root_body()))
    }
    /// The soft body this one was split off from by a tear, if any.
    #[getter]
    fn origin(&self) -> PyResult<Option<SoftBodyHandle>> {
        self.with_ref(|b| b.origin().map(SoftBodyHandle))
    }
    /// The soft bodies that tears split off from this one.
    #[getter]
    fn pieces(&self) -> PyResult<Vec<SoftBodyHandle>> {
        self.with_ref(|b| b.pieces().iter().copied().map(SoftBodyHandle).collect())
    }
    /// The center of mass of the particles.
    #[getter]
    fn center_of_mass(&self) -> PyResult<Vec3> {
        self.with_ref(|b| vec3(b.center_of_mass()))
    }
    /// The total mass of the particles.
    #[getter]
    fn mass(&self) -> PyResult<Real> {
        self.with_ref(|b| b.mass())
    }
    /// Is the soft body sleeping?
    #[getter]
    fn is_sleeping(&self) -> PyResult<bool> {
        self.with_ref(|b| b.is_sleeping())
    }
    /// Wake the soft body up.
    fn wake_up(&mut self) -> PyResult<()> {
        self.with_mut(|b| b.wake_up())
    }
    /// Is the soft body enabled (simulated)? Writable, like :attr:`RigidBody.is_enabled`.
    #[getter]
    fn is_enabled(&self) -> PyResult<bool> {
        self.with_ref(|b| b.is_enabled())
    }
    /// Enable or disable the soft body.
    #[setter(is_enabled)]
    fn set_is_enabled(&mut self, enabled: bool) -> PyResult<()> {
        self.with_mut(|b| b.set_enabled(enabled))
    }
    /// Enable or disable the soft body (same as setting :attr:`is_enabled`).
    fn set_enabled(&mut self, enabled: bool) -> PyResult<()> {
        self.with_mut(|b| b.set_enabled(enabled))
    }
    /// Set the extra internal PGS iterations run per substep for this body and everything it
    /// touches.
    fn set_additional_pgs_iterations(&mut self, iterations: usize) -> PyResult<()> {
        self.with_mut(|b| b.set_additional_pgs_iterations(iterations))
    }
    /// The number of colors the parallel solver splits the body's constraints into.
    #[getter]
    fn num_solver_colors(&self) -> PyResult<usize> {
        self.with_ref(|b| b.num_solver_colors())
    }
    /// Arbitrary integer user data.
    #[getter]
    fn user_data(&self) -> PyResult<u128> {
        self.with_ref(|b| b.user_data)
    }
    #[setter]
    fn set_user_data(&mut self, data: u128) -> PyResult<()> {
        self.with_mut(|b| b.user_data = data)
    }

    /*
     * Forces and impulses.
     */

    /// Add a force to every particle (spread by mass).
    #[pyo3(signature = (force, wake_up=true))]
    fn add_force(&mut self, force: PyVector, wake_up: bool) -> PyResult<()> {
        self.with_mut(|b| b.add_force(force.0.into(), wake_up))
    }
    /// Add a force to the ``i``-th particle.
    #[pyo3(signature = (i, force, wake_up=true))]
    fn add_particle_force(&mut self, i: usize, force: PyVector, wake_up: bool) -> PyResult<()> {
        self.check_particle(i)?;
        self.with_mut(|b| b.add_particle_force(i, force.0.into(), wake_up))?;
        Ok(())
    }
    /// Reset the user forces applied to the particles.
    #[pyo3(signature = (wake_up=true))]
    fn reset_forces(&mut self, wake_up: bool) -> PyResult<()> {
        self.with_mut(|b| b.reset_forces(wake_up))
    }
    /// Apply an impulse to every particle (spread by mass).
    #[pyo3(signature = (impulse, wake_up=true))]
    fn apply_impulse(&mut self, impulse: PyVector, wake_up: bool) -> PyResult<()> {
        self.with_mut(|b| b.apply_impulse(impulse.0.into(), wake_up))
    }
    /// Apply an impulse to the ``i``-th particle.
    #[pyo3(signature = (i, impulse, wake_up=true))]
    fn apply_particle_impulse(
        &mut self,
        i: usize,
        impulse: PyVector,
        wake_up: bool,
    ) -> PyResult<()> {
        self.check_particle(i)?;
        self.with_mut(|b| b.apply_particle_impulse(i, impulse.0.into(), wake_up))?;
        Ok(())
    }
    /// Apply an impulse to the particles within ``falloff_radius`` of ``point``, scaled down
    /// linearly with their distance to it (``falloff_radius <= 0`` applies it everywhere).
    #[pyo3(signature = (impulse, point, falloff_radius, wake_up=true))]
    fn apply_impulse_at_point(
        &mut self,
        impulse: PyVector,
        point: PyVector,
        falloff_radius: Real,
        wake_up: bool,
    ) -> PyResult<()> {
        self.with_mut(|b| {
            b.apply_impulse_at_point(impulse.0.into(), point.0.into(), falloff_radius, wake_up)
        })
    }
    /// Apply an impulse of the given magnitude pushing the particles away from ``center`` (a
    /// blast), scaled down linearly up to ``falloff_radius``.
    #[pyo3(signature = (center, magnitude, falloff_radius, wake_up=true))]
    fn apply_radial_impulse(
        &mut self,
        center: PyVector,
        magnitude: Real,
        falloff_radius: Real,
        wake_up: bool,
    ) -> PyResult<()> {
        self.with_mut(|b| {
            b.apply_radial_impulse(center.0.into(), magnitude, falloff_radius, wake_up)
        })
    }

    /*
     * Tearing.
     */

    /// Request the ``i``-th edge to tear at the end of the next step.
    fn tear_edge(&mut self, i: usize) -> PyResult<()> {
        self.with_mut(|b| b.tear_edge(i))
    }
    /// Request the ``i``-th cell to tear at the end of the next step.
    fn tear_cell(&mut self, i: usize) -> PyResult<()> {
        self.with_mut(|b| b.tear_cell(i))
    }
    /// Are there tears requested for the next step?
    #[getter]
    fn has_pending_tears(&self) -> PyResult<bool> {
        self.with_ref(|b| b.has_pending_tears())
    }
    /// Set the tear-threshold multiplier of the ``i``-th edge (``1.0`` restores the material's
    /// threshold, see :attr:`SoftBodyEdge.tear_resistance`).
    fn set_edge_tear_resistance(&mut self, i: usize, resistance: Real) -> PyResult<()> {
        self.check_index("edge", i, |b| b.edges().len())?;
        self.with_mut(|b| b.set_edge_tear_resistance(i, resistance))
    }
    /// Set the tear-threshold multiplier of the ``i``-th cell (``1.0`` restores the material's
    /// threshold, see :attr:`SoftBodyCell.tear_resistance`).
    fn set_cell_tear_resistance(&mut self, i: usize, resistance: Real) -> PyResult<()> {
        self.check_index("cell", i, |b| b.cells().len())?;
        self.with_mut(|b| b.set_cell_tear_resistance(i, resistance))
    }
    /// Mark the ``i``-th particle as damaged, or repair it (see
    /// :attr:`SoftBodyParticle.is_damaged`): a way to seed a weak spot where a tear should start,
    /// since the material's :attr:`~SoftBodyMaterial.interior_strength` does not apply to it.
    fn set_particle_damaged(&mut self, i: usize, damaged: bool) -> PyResult<()> {
        self.check_particle(i)?;
        self.with_mut(|b| b.set_particle_damaged(i, damaged))
    }
    /// The ``(edges, cells)`` a blade (a world-space triangle given as three points) crosses,
    /// as would be torn by :meth:`SoftBodySet.cut`.
    fn crossing_elements(
        &self,
        blade: (PyVector, PyVector, PyVector),
    ) -> PyResult<(Vec<u32>, Vec<u32>)> {
        let blade = [blade.0.0.into(), blade.1.0.into(), blade.2.0.into()];
        self.with_ref(|b| b.crossing_elements(&blade))
    }

    /*
     * Clusters.
     */

    /// The number of cluster slots (some may have been removed: see
    /// :attr:`SoftBodyCluster.is_live`).
    #[getter]
    fn num_clusters(&self) -> PyResult<usize> {
        self.with_ref(|b| b.clusters().len())
    }
    /// The number of live clusters.
    #[getter]
    fn num_live_clusters(&self) -> PyResult<usize> {
        self.with_ref(|b| b.num_live_clusters())
    }
    /// A snapshot of the ``i``-th cluster, or ``None``.
    fn cluster(&self, i: u32) -> PyResult<Option<SoftBodyCluster>> {
        self.with_ref(|b| {
            b.cluster(i).map(|c| SoftBodyCluster {
                index: i,
                particles: c.particles().to_vec(),
                proxy: RigidBodyHandle(c.proxy()),
                is_live: c.is_live(),
                shape_matching_enabled: c.shape_matching_enabled(),
                meshes: c.meshes().map(|m| SoftMeshId(m.id())).collect(),
            })
        })
    }
    /// Snapshots of every live cluster.
    #[getter]
    fn clusters(&self) -> PyResult<Vec<SoftBodyCluster>> {
        self.with_ref(|b| {
            b.live_clusters()
                .map(|(i, c)| SoftBodyCluster {
                    index: i,
                    particles: c.particles().to_vec(),
                    proxy: RigidBodyHandle(c.proxy()),
                    is_live: true,
                    shape_matching_enabled: c.shape_matching_enabled(),
                    meshes: c.meshes().map(|m| SoftMeshId(m.id())).collect(),
                })
                .collect()
        })
    }
    /// The rigid-body proxy of the ``i``-th cluster: joints and colliders attach to it.
    fn cluster_proxy(&self, i: u32) -> PyResult<Option<RigidBodyHandle>> {
        self.with_ref(|b| b.cluster_proxy(i).map(RigidBodyHandle))
    }
    /// Enable or disable shape matching on the ``i``-th cluster.
    fn enable_cluster_shape_matching(&mut self, i: u32, enabled: bool) -> PyResult<()> {
        self.with_mut(|b| b.enable_cluster_shape_matching(i, enabled))
    }
    /// Scale the stiffness of the elements of the ``i``-th cluster.
    fn set_cluster_stiffness_scale(&mut self, i: u32, scale: Real) -> PyResult<()> {
        self.with_mut(|b| b.set_cluster_stiffness_scale(i, scale))
    }
    /// Override the softness of the edges of the ``i``-th cluster (``None`` restores the
    /// material's).
    #[pyo3(signature = (i, softness))]
    fn set_cluster_edge_softness(
        &mut self,
        i: u32,
        softness: Option<&Bound<'_, PyAny>>,
    ) -> PyResult<()> {
        let s = softness.map(spring).transpose()?;
        self.with_mut(|b| b.set_cluster_edge_softness(i, s))?;
        Ok(())
    }
    /// Scale the tear thresholds of the elements of the ``i``-th cluster.
    fn set_cluster_tear_resistance(&mut self, i: u32, resistance: Real) -> PyResult<()> {
        self.with_mut(|b| b.set_cluster_tear_resistance(i, resistance))
    }
    /// Pin (or release) every particle of the ``i``-th cluster.
    fn set_cluster_pinned(&mut self, i: u32, pinned: bool) -> PyResult<()> {
        self.with_mut(|b| b.set_cluster_pinned(i, pinned))
    }
    /// Move the ``i``-th cluster rigidly to the given pose over the next step.
    fn set_cluster_kinematic_target(&mut self, i: u32, pose: PyIsometry) -> PyResult<()> {
        let p: rapier::math::Pose = pose.0.into();
        self.with_mut(|b| b.set_cluster_kinematic_target(i, p))
    }
    /// Set the pose the shape matching of the ``i``-th cluster (see
    /// :meth:`enable_cluster_shape_matching`) pulls its particles toward, as a kinematic path
    /// (``None``: the cluster's own frame). Ignored if there is no live ``i``-th cluster.
    #[pyo3(signature = (i, target))]
    fn set_cluster_shape_matching_target(
        &mut self,
        i: u32,
        target: Option<PyIsometry>,
    ) -> PyResult<()> {
        let target: Option<rapier::math::Pose> = target.map(|t| t.0.into());
        self.with_mut(|b| {
            if let Some(cluster) = b.cluster_mut(i) {
                cluster.set_shape_matching_target(target);
            }
        })
    }

    /*
     * Collision meshes.
     */

    /// Snapshots of every collision mesh held by the clusters (the body's own deformable
    /// surface included), with their current world-space vertices.
    #[getter]
    fn meshes(&self) -> PyResult<Vec<SoftCollisionMesh>> {
        self.with_ref(|b| {
            b.meshes()
                .map(|m| SoftCollisionMesh::from_rapier(b, m))
                .collect()
        })
    }
    /// A snapshot of the collision mesh with the given identifier, or ``None``.
    fn mesh(&self, id: &SoftMeshId) -> PyResult<Option<SoftCollisionMesh>> {
        self.with_ref(|b| b.mesh(id.0).map(|m| SoftCollisionMesh::from_rapier(b, m)))
    }
    /// A snapshot of the collision mesh a deformable collider holds, or ``None``.
    fn mesh_of(&self, collider: &ColliderHandle) -> PyResult<Option<SoftCollisionMesh>> {
        self.with_ref(|b| {
            b.mesh_of(collider.0)
                .map(|m| SoftCollisionMesh::from_rapier(b, m))
        })
    }
    /// A snapshot of the body's own deformable surface mesh, or ``None`` when the body
    /// collides through its particles.
    fn collision_mesh(&self) -> PyResult<Option<SoftCollisionMesh>> {
        self.with_ref(|b| {
            b.collision_mesh()
                .map(|m| SoftCollisionMesh::from_rapier(b, m))
        })
    }

    fn __repr__(&self) -> PyResult<String> {
        self.with_ref(|b| {
            format!(
                "SoftBody(particles={}, edges={}, cells={}, mass={})",
                b.num_particles(),
                b.edges().len(),
                b.cells().len(),
                b.mass()
            )
        })
    }
}

// ----------------------------------------------------------------------
// SoftBodySet.
// ----------------------------------------------------------------------

/// Container holding every :class:`SoftBody` of a physics world.
///
/// Bodies are addressed by :class:`SoftBodyHandle` and live in the set until removed. The
/// set supports ``len()``, ``in``, iteration (yielding ``(handle, body)`` pairs) and
/// ``[handle]`` lookup; the bodies it returns are live views.
#[pyclass(name = "SoftBodySet", module = "rapier")]
pub struct SoftBodySet(pub rapier::dynamics::SoftBodySet);

impl SoftBodySet {
    /// Run `f` on the set, also while a step lends it to an event handler.
    pub(crate) fn read<R>(
        slf: &Bound<'_, Self>,
        f: impl FnOnce(&rapier::dynamics::SoftBodySet) -> R,
    ) -> PyResult<R> {
        // A lent set is read first: the running step holds the set mutably borrowed, so
        // `try_borrow` fails until it ends.
        crate::events_hooks::with_lent_or(slf.as_ptr(), f, |f| match slf.try_borrow() {
            Ok(set) => Ok(f(&set.0)),
            Err(_) => Err(crate::events_hooks::stepping_error("SoftBodySet")),
        })
    }
}

#[pymethods]
impl SoftBodySet {
    /// Build an empty soft-body set.
    #[new]
    fn new() -> Self {
        Self(rapier::dynamics::SoftBodySet::new())
    }

    /// Insert the soft body described by ``builder``, creating its hidden root rigid body in
    /// ``bodies`` and its colliders in ``colliders``.
    fn insert(
        &mut self,
        builder: &SoftBodyBuilder,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> SoftBodyHandle {
        SoftBodyHandle(
            self.0
                .insert(builder.builder.clone(), &mut bodies.0, &mut colliders.0),
        )
    }

    /// Remove a soft body with its proxies, colliders and attached joints.
    ///
    /// :returns: the removed body, or ``None`` if the handle matched nothing.
    #[allow(clippy::too_many_arguments)]
    fn remove(
        &mut self,
        handle: &SoftBodyHandle,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> Option<SoftBody> {
        self.0
            .remove(
                handle.0,
                &mut islands.0,
                &mut bodies.0,
                &mut colliders.0,
                &mut impulse_joints.0,
                &mut multibody_joints.0,
            )
            .map(SoftBody::new_owned)
    }

    /// Add a cluster over the given particles: a rigid proxy in ``bodies`` that joints and
    /// colliders can attach to. Returns the cluster's index, or ``None`` if no particle was
    /// valid.
    pub fn add_cluster(
        &mut self,
        handle: &SoftBodyHandle,
        particles: &Bound<'_, PyAny>,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> PyResult<Option<u32>> {
        let particles = extract_index_list(particles)?;
        Ok(self
            .0
            .add_cluster(handle.0, &particles, &mut bodies.0, &mut colliders.0))
    }

    /// Remove a cluster with its proxy, colliders and joints; ``False`` if it did not exist.
    #[allow(clippy::too_many_arguments)]
    pub fn remove_cluster(
        &mut self,
        handle: &SoftBodyHandle,
        cluster: u32,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> bool {
        self.0
            .remove_cluster(
                handle.0,
                cluster,
                &mut islands.0,
                &mut bodies.0,
                &mut colliders.0,
                &mut impulse_joints.0,
                &mut multibody_joints.0,
            )
            .is_some()
    }

    /// Tear a soft body at once along the given edges and through the given cells (indices
    /// into its edge and cell lists), without removing material; pieces disconnected by the
    /// tear become soft bodies of their own.
    ///
    /// :returns: the :class:`SoftBodyTearEvent`, or ``None`` when nothing changed.
    #[allow(clippy::too_many_arguments)]
    pub fn tear(
        &mut self,
        handle: &SoftBodyHandle,
        edges: &Bound<'_, PyAny>,
        cells: &Bound<'_, PyAny>,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> PyResult<Option<SoftBodyTearEvent>> {
        let edges = extract_index_list(edges)?;
        let cells = extract_index_list(cells)?;
        Ok(self
            .0
            .tear(
                handle.0,
                &edges,
                &cells,
                &mut islands.0,
                &mut bodies.0,
                &mut colliders.0,
                &mut impulse_joints.0,
                &mut multibody_joints.0,
            )
            .map(SoftBodyTearEvent))
    }

    /// Cut a soft body along a blade (a world-space triangle given as three points) at once,
    /// without removing material; pieces disconnected by the cut become soft bodies of their
    /// own.
    ///
    /// :returns: the :class:`SoftBodyTearEvent`, or ``None`` when nothing changed.
    #[allow(clippy::too_many_arguments)]
    pub fn cut(
        &mut self,
        handle: &SoftBodyHandle,
        blade: (PyVector, PyVector, PyVector),
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> Option<SoftBodyTearEvent> {
        let blade = [blade.0.0.into(), blade.1.0.into(), blade.2.0.into()];
        self.0
            .cut(
                handle.0,
                &blade,
                &mut islands.0,
                &mut bodies.0,
                &mut colliders.0,
                &mut impulse_joints.0,
                &mut multibody_joints.0,
            )
            .map(SoftBodyTearEvent)
    }

    /// Wake a soft body and everything it touches up; ``strong`` keeps them awake for a while
    /// even at rest.
    #[pyo3(signature = (handle, bodies, strong=false))]
    fn wake_up(&mut self, handle: &SoftBodyHandle, bodies: &mut RigidBodySet, strong: bool) {
        self.0.wake_up(handle.0, &mut bodies.0, strong)
    }

    /// A live view of the body for ``handle``, or ``None``.
    fn get(slf: &Bound<'_, Self>, handle: &SoftBodyHandle) -> PyResult<Option<SoftBody>> {
        if !Self::read(slf, |set| set.contains(handle.0))? {
            return Ok(None);
        }
        Ok(Some(SoftBody {
            backing: SoftBodyBacking::InSet {
                set: slf.clone().unbind(),
                handle: handle.0,
            },
        }))
    }

    /// Indexing form of ``get``.
    ///
    /// :raises InvalidHandle: if ``handle`` matches no body.
    fn __getitem__(slf: &Bound<'_, Self>, handle: &SoftBodyHandle) -> PyResult<SoftBody> {
        if !Self::read(slf, |set| set.contains(handle.0))? {
            return Err(crate::errors::InvalidHandle::new_err(format!(
                "no soft body for {:?}",
                handle.0.into_raw_parts()
            )));
        }
        Ok(SoftBody {
            backing: SoftBodyBacking::InSet {
                set: slf.clone().unbind(),
                handle: handle.0,
            },
        })
    }

    fn __contains__(slf: &Bound<'_, Self>, handle: &SoftBodyHandle) -> PyResult<bool> {
        Self::read(slf, |set| set.contains(handle.0))
    }

    fn __len__(slf: &Bound<'_, Self>) -> PyResult<usize> {
        Self::read(slf, |set| set.len())
    }

    /// Is the set empty?
    fn is_empty(slf: &Bound<'_, Self>) -> PyResult<bool> {
        Self::read(slf, |set| set.is_empty())
    }

    /// Iterate over ``(handle, body)`` pairs; each body is a live view.
    fn __iter__(slf: &Bound<'_, Self>) -> PyResult<Py<SoftBodySetIter>> {
        let handles: Vec<rapier::dynamics::SoftBodyHandle> =
            Self::read(slf, |set| set.iter().map(|(h, _)| h).collect())?;
        Py::new(
            slf.py(),
            SoftBodySetIter {
                set: slf.clone().unbind(),
                handles,
                i: 0,
            },
        )
    }

    /// The handles of every body in the set.
    fn handles(slf: &Bound<'_, Self>) -> PyResult<Vec<SoftBodyHandle>> {
        Self::read(slf, |set| {
            set.iter().map(|(h, _)| SoftBodyHandle(h)).collect()
        })
    }

    fn __repr__(&self) -> String {
        format!("SoftBodySet(len={})", self.0.len())
    }
}

/// Iterator yielding ``(SoftBodyHandle, SoftBody)`` pairs from a :class:`SoftBodySet`.
#[pyclass]
pub struct SoftBodySetIter {
    set: Py<SoftBodySet>,
    handles: Vec<rapier::dynamics::SoftBodyHandle>,
    i: usize,
}

#[pymethods]
impl SoftBodySetIter {
    fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }
    fn __next__(mut slf: PyRefMut<'_, Self>) -> Option<(SoftBodyHandle, SoftBody)> {
        if slf.i >= slf.handles.len() {
            return None;
        }
        let py = slf.py();
        let handle = slf.handles[slf.i];
        slf.i += 1;
        let set = slf.set.clone_ref(py);
        Some((
            SoftBodyHandle(handle),
            SoftBody {
                backing: SoftBodyBacking::InSet { set, handle },
            },
        ))
    }
}

pub fn register_soft_bodies(
    py: Python<'_>,
    m: &Bound<'_, crate::pyo3::types::PyModule>,
) -> PyResult<()> {
    m.add("SoftBindingError", py.get_type_bound::<SoftBindingError>())?;
    m.add_class::<SoftBodyHandle>()?;
    m.add_class::<SoftBodyCellModel>()?;
    m.add_class::<SoftEdgePlasticFlow>()?;
    m.add_class::<SoftBodyEdgeKind>()?;
    m.add_class::<SoftPatchConstraints>()?;
    m.add_class::<SoftBodySolver>()?;
    m.add_class::<MeshEnclosure>()?;
    m.add_class::<VolumeMeshParameters>()?;
    m.add_class::<SoftFemParameters>()?;
    m.add_class::<SoftBodyMaterial>()?;
    m.add_class::<SoftBodyParticleSettings>()?;
    m.add_class::<SoftRecoverySettings>()?;
    m.add_class::<SoftBodiesSettings>()?;
    m.add_class::<SoftBodyBuilder>()?;
    m.add_class::<SoftBodyParticle>()?;
    m.add_class::<SoftBodyEdge>()?;
    m.add_class::<SoftBodyDihedral>()?;
    m.add_class::<SoftBodyCell>()?;
    m.add_class::<SoftParticleAttachment>()?;
    m.add_class::<SoftVolumePiece>()?;
    m.add_class::<SoftBodyCluster>()?;
    m.add_class::<SoftMeshId>()?;
    m.add_class::<SoftMeshRef>()?;
    m.add_class::<SoftCollisionMesh>()?;
    m.add_class::<SoftMeshBinding>()?;
    m.add_class::<SoftBodyPiece>()?;
    m.add_class::<SoftClusterSplit>()?;
    m.add_class::<SoftJointMove>()?;
    m.add_class::<SoftBodyTearEvent>()?;
    m.add_class::<SoftBody>()?;
    m.add_class::<SoftBodySet>()?;
    m.add_class::<SoftBodySetIter>()?;
    Ok(())
}
