//! Concrete 3D / f32 loader `#[pyclass]` types (URDF / MJCF / Mesh) and the
//! `register_loaders` entry point.
//!
//! The mesh, URDF and MJCF loaders were previously emitted by the
//! `__define_loaders_3d!` / `__define_loaders_urdf!` / `__define_loaders_mjcf!`
//! macros (spread across `loaders.rs` / `urdf.rs` / `mjcf.rs` in the old
//! `rapier-py-core` layer). With a single concrete target — 3D, `f32` — they
//! are written out directly here, with no macros.
//!
//! `register_loaders(py, m)` registers all three loaders for the `#[pymodule]`
//! entry point.
//!
//! `rapier3d-urdf` / `rapier3d-meshloader` / `rapier3d-mjcf` are 3D/f32-only
//! upstream, which is why the loaders only ever existed in this variant.

use crate::*;
use rapier3d as rapier;

// ==============================================================
// Mesh loader
// ==============================================================

/// One shape extracted by :func:`load_from_path` or
/// :func:`load_from_raw_mesh`.
///
/// Bundles the converted :class:`SharedShape`, its world-relative
/// :class:`Isometry3` pose (taken from any ``visual.origin`` /
/// ``collision.origin`` element in the source file), and the raw
/// triangle mesh ``(vertices, indices)`` used to build the shape.
///
/// :ivar shape: Converted collision shape.
/// :ivar pose: World-relative pose of the shape.
/// :ivar vertices: ``(N, 3)`` float32 NumPy array of mesh vertices.
/// :ivar indices: ``(M, 3)`` uint32 NumPy array of triangle indices.
#[pyclass(name = "LoadedShape", module = "rapier")]
pub struct LoadedShape {
    #[pyo3(get)]
    pub shape: SharedShape,
    #[pyo3(get)]
    pub pose: Isometry3,
    vertices: Vec<[f32; 3]>,
    indices: Vec<[u32; 3]>,
}

#[pymethods]
impl LoadedShape {
    /// ``(N, 3)`` float32 NumPy array of raw mesh vertex positions.
    #[getter]
    fn vertices<'py>(
        &self,
        py: crate::pyo3::Python<'py>,
    ) -> crate::pyo3::Bound<'py, crate::numpy::PyArray2<f32>> {
        use crate::numpy::PyArray2;
        let n = self.vertices.len();
        let mut flat: Vec<f32> = Vec::with_capacity(n * 3);
        for v in &self.vertices {
            flat.extend_from_slice(v);
        }
        PyArray2::from_vec2_bound(
            py,
            &self.vertices.iter().map(|v| v.to_vec()).collect::<Vec<_>>(),
        )
        .unwrap_or_else(|_| PyArray2::<f32>::zeros_bound(py, [0, 3], false))
    }

    /// ``(M, 3)`` uint32 NumPy array of triangle face indices.
    #[getter]
    fn indices<'py>(
        &self,
        py: crate::pyo3::Python<'py>,
    ) -> crate::pyo3::Bound<'py, crate::numpy::PyArray2<u32>> {
        use crate::numpy::PyArray2;
        PyArray2::from_vec2_bound(
            py,
            &self.indices.iter().map(|v| v.to_vec()).collect::<Vec<_>>(),
        )
        .unwrap_or_else(|_| PyArray2::<u32>::zeros_bound(py, [0, 3], false))
    }

    /// Return the ``LoadedShape(...)`` repr.
    fn __repr__(&self) -> String {
        format!(
            "LoadedShape(shape={:?}, n_vertices={}, n_faces={})",
            self.shape.0.shape_type(),
            self.vertices.len(),
            self.indices.len(),
        )
    }
}

fn _loaded_shape_from_meshloader(loaded: rapier3d_meshloader::LoadedShape) -> LoadedShape {
    let iso: crate::na::Isometry<Real, _, 3> = loaded.pose.into();
    LoadedShape {
        shape: SharedShape(loaded.shape),
        pose: Isometry3(iso),
        vertices: loaded.raw_mesh.vertices.clone(),
        indices: loaded.raw_mesh.faces.clone(),
    }
}

/// Load shapes from a mesh file on disk.
///
/// The file (STL, COLLADA ``.dae`` or Wavefront ``.obj``, picked from
/// its extension) is parsed into one or more meshes; each mesh is
/// independently converted into a shape using ``converter``.
///
/// :param path: Path to the source mesh file.
/// :param converter: :class:`MeshConverter` to use (defaults to
///     :attr:`MeshConverter.TRIMESH`).
/// :param scale: Uniform scale applied during conversion.
/// :returns: A list with one entry per source group, each either a
///     :class:`LoadedShape` (success) or a ``MeshConversionError``
///     instance (failure).
/// :raises MeshLoaderError: if the file itself failed to load.
#[pyfunction]
#[pyo3(name = "load_from_path")]
#[pyo3(signature = (path, converter=None, scale=1.0))]
fn loaders_mesh_load_from_path(
    py: crate::pyo3::Python<'_>,
    path: &str,
    converter: Option<MeshConverter>,
    scale: Real,
) -> crate::pyo3::PyResult<Vec<crate::pyo3::PyObject>> {
    use crate::pyo3::IntoPy;
    let converter = converter.unwrap_or(MeshConverter(rapier::geometry::MeshConverter::TriMesh));
    let scale_v = rapier::math::Vector::new(scale, scale, scale);
    let results = rapier3d_meshloader::load_from_path(path, &converter.0, scale_v)
        .map_err(|e| crate::errors::MeshLoaderError::new_err(format!("{e}")))?;
    let mut out: Vec<crate::pyo3::PyObject> = Vec::with_capacity(results.len());
    for r in results {
        match r {
            Ok(loaded) => {
                let py_loaded = _loaded_shape_from_meshloader(loaded);
                out.push(crate::pyo3::Py::new(py, py_loaded)?.into_py(py));
            }
            Err(e) => {
                let exc = crate::errors::MeshConversionError::new_err(format!("{e}"));
                out.push(exc.value_bound(py).clone().into());
            }
        }
    }
    Ok(out)
}

/// Load a single shape from an in-memory triangle mesh.
///
/// Identical to :func:`load_from_path` except the mesh data comes
/// from NumPy arrays / sequences directly. No file IO.
///
/// :param vertices: Sequence of 3D vertex positions.
/// :param indices: Sequence of triangle indices ``(i, j, k)``.
/// :param converter: :class:`MeshConverter` (defaults to
///     :attr:`MeshConverter.TRIMESH`).
/// :param scale: Uniform scale applied during conversion.
/// :returns: A :class:`LoadedShape`.
/// :raises MeshConversionError: if the conversion failed.
#[pyfunction]
#[pyo3(name = "load_from_raw_mesh")]
#[pyo3(signature = (vertices, indices, converter=None, scale=1.0))]
fn loaders_mesh_load_from_raw_mesh(
    _py: crate::pyo3::Python<'_>,
    vertices: &crate::pyo3::Bound<'_, crate::pyo3::PyAny>,
    indices: &crate::pyo3::Bound<'_, crate::pyo3::PyAny>,
    converter: Option<MeshConverter>,
    scale: Real,
) -> crate::pyo3::PyResult<LoadedShape> {
    let verts = crate::geometry::extract_verts_for_dim(vertices)?;
    let idx = crate::geometry::extract_indices(indices)?;
    let converter = converter.unwrap_or(MeshConverter(rapier::geometry::MeshConverter::TriMesh));
    let mut mesh = mesh_loader::Mesh::default();
    mesh.vertices = verts.iter().map(|v| [v.x, v.y, v.z]).collect();
    mesh.faces = idx.clone();
    let scale_v = rapier::math::Vector::new(scale, scale, scale);
    let (shape, pose) = rapier3d_meshloader::load_from_raw_mesh(&mesh, &converter.0, scale_v)
        .map_err(|e| crate::errors::MeshConversionError::new_err(format!("{e}")))?;
    let iso: crate::na::Isometry<Real, _, 3> = pose.into();
    Ok(LoadedShape {
        shape: SharedShape(shape),
        pose: Isometry3(iso),
        vertices: mesh.vertices,
        indices: mesh.faces,
    })
}

// ==============================================================
// URDF loader
// ==============================================================

// `rapier3d-urdf` resolves mesh references against a base directory.
use std::path::Path as _StdPath;

// ----- URDF options ------------------------------------------------

/// Bitflags controlling the multibody-joint URDF insertion path.
///
/// Combine flags with the usual ``|`` and ``&`` operators. Pass
/// the result to
/// :meth:`UrdfRobot.insert_using_multibody_joints`.
#[pyclass(name = "UrdfMultibodyOptions", module = "rapier", frozen)]
#[derive(Clone, Copy, Debug, Default)]
pub struct UrdfMultibodyOptions(pub rapier3d_urdf::UrdfMultibodyOptions);

#[pymethods]
impl UrdfMultibodyOptions {
    /// Build from a raw bit pattern.
    #[new]
    #[pyo3(signature = (bits = 0))]
    fn new(bits: u8) -> Self {
        Self(rapier3d_urdf::UrdfMultibodyOptions::from_bits_truncate(
            bits,
        ))
    }
    /// Return the empty flag set (no flags).
    #[staticmethod]
    fn empty() -> Self {
        Self(rapier3d_urdf::UrdfMultibodyOptions::empty())
    }
    /// Treat URDF joints as kinematic rather than dynamic.
    #[classattr]
    const JOINTS_ARE_KINEMATIC: UrdfMultibodyOptions =
        UrdfMultibodyOptions(rapier3d_urdf::UrdfMultibodyOptions::JOINTS_ARE_KINEMATIC);
    /// Disable self-collisions between links of the same robot.
    #[classattr]
    const DISABLE_SELF_CONTACTS: UrdfMultibodyOptions =
        UrdfMultibodyOptions(rapier3d_urdf::UrdfMultibodyOptions::DISABLE_SELF_CONTACTS);
    /// Raw bits as an unsigned int.
    #[getter]
    fn bits(&self) -> u8 {
        self.0.bits()
    }
    /// Bitwise OR: union of two flag sets.
    fn __or__(&self, other: &UrdfMultibodyOptions) -> Self {
        Self(self.0 | other.0)
    }
    /// Bitwise AND: intersection of two flag sets.
    fn __and__(&self, other: &UrdfMultibodyOptions) -> Self {
        Self(self.0 & other.0)
    }
    /// ``other in self``: ``True`` if every flag of ``other`` is set here.
    fn __contains__(&self, other: &UrdfMultibodyOptions) -> bool {
        self.0.contains(other.0)
    }
    /// Return ``UrdfMultibodyOptions(bits=0b...)`` repr.
    fn __repr__(&self) -> String {
        format!("UrdfMultibodyOptions(bits={:#06b})", self.0.bits())
    }
}

/// Configuration for the URDF loader.
///
/// Mirrors ``rapier3d_urdf::UrdfLoaderOptions``. Pass an instance
/// to :meth:`UrdfRobot.from_file` / :meth:`from_str` /
/// :meth:`from_robot` to influence which colliders are produced
/// and how the robot is grounded.
///
/// :ivar create_colliders_from_collision_shapes: If ``True``,
///     materialize ``<collision>`` blocks as colliders.
/// :ivar create_colliders_from_visual_shapes: If ``True``, also
///     materialize ``<visual>`` blocks as colliders (off by default).
/// :ivar apply_imported_mass_props: If ``True``, use the URDF
///     ``<inertial>`` block as the body's mass properties.
/// :ivar enable_joint_collisions: If ``True``, do not disable
///     collisions between links joined by a URDF joint.
/// :ivar make_roots_fixed: If ``True``, the robot's root bodies
///     are created as ``Fixed`` instead of ``Dynamic``.
/// :ivar trimesh_flags: :class:`TriMeshFlags` applied to mesh
///     colliders.
/// :ivar mesh_converter: Optional :class:`MeshConverter` controlling
///     how every referenced mesh is turned into a collider shape.
///     Defaults to ``None`` (trimesh, using ``trimesh_flags``). Set
///     e.g. ``MeshConverter.OBB`` to get cheap proxy shapes while
///     keeping the original mesh available as a visual override (see
///     :attr:`UrdfColliderHandle.visual`).
/// :ivar shift: Rigid transform applied to every body of the
///     robot on import (applied *after* ``scale``).
/// :ivar scale: Uniform scale applied to every length read from the
///     URDF (link positions, joint anchors, mesh sizes, primitive
///     shape sizes, inertial offsets, prismatic joint limits). Mass
///     and inertia tensors are left unchanged. Defaults to ``1.0``.
/// :ivar squeeze_empty_fixed_links: If ``True`` (default), URDF links
///     with no ``<visual>``/``<collision>``/``<inertial>`` connected
///     by fixed joints are removed and the chain spliced together, so
///     bodyless "frame-only" links (e.g. ``world``, ``*_tcp``) no
///     longer create mass-less rigid-bodies.
#[pyclass(name = "UrdfLoaderOptions", module = "rapier")]
#[derive(Clone)]
pub struct UrdfLoaderOptions {
    #[pyo3(get, set)]
    pub create_colliders_from_collision_shapes: bool,
    #[pyo3(get, set)]
    pub create_colliders_from_visual_shapes: bool,
    #[pyo3(get, set)]
    pub apply_imported_mass_props: bool,
    #[pyo3(get, set)]
    pub enable_joint_collisions: bool,
    #[pyo3(get, set)]
    pub make_roots_fixed: bool,
    #[pyo3(get, set)]
    pub trimesh_flags: TriMeshFlags,
    #[pyo3(get, set)]
    pub mesh_converter: Option<MeshConverter>,
    #[pyo3(get, set)]
    pub shift: Isometry3,
    #[pyo3(get, set)]
    pub scale: Real,
    #[pyo3(get, set)]
    pub squeeze_empty_fixed_links: bool,
    /// Template collider applied to every imported collider before
    /// its shape/material are filled in. ``None`` keeps the rapier
    /// default (density 0).
    #[pyo3(get, set)]
    pub collider_blueprint: Option<ColliderBuilder>,
    /// Template rigid-body used for every imported link. ``None``
    /// keeps the rapier default (a dynamic body).
    #[pyo3(get, set)]
    pub rigid_body_blueprint: Option<RigidBodyBuilder>,
}

#[pymethods]
impl UrdfLoaderOptions {
    /// Build a ``UrdfLoaderOptions`` with optional overrides.
    ///
    /// See the class docstring for the meaning of each parameter.
    #[new]
    #[pyo3(signature = (
        create_colliders_from_collision_shapes = true,
        create_colliders_from_visual_shapes = false,
        apply_imported_mass_props = true,
        enable_joint_collisions = false,
        make_roots_fixed = false,
        trimesh_flags = None,
        mesh_converter = None,
        shift = None,
        scale = 1.0,
        squeeze_empty_fixed_links = true,
        collider_blueprint = None,
        rigid_body_blueprint = None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        create_colliders_from_collision_shapes: bool,
        create_colliders_from_visual_shapes: bool,
        apply_imported_mass_props: bool,
        enable_joint_collisions: bool,
        make_roots_fixed: bool,
        trimesh_flags: Option<TriMeshFlags>,
        mesh_converter: Option<MeshConverter>,
        shift: Option<Isometry3>,
        scale: Real,
        squeeze_empty_fixed_links: bool,
        collider_blueprint: Option<ColliderBuilder>,
        rigid_body_blueprint: Option<RigidBodyBuilder>,
    ) -> Self {
        let default = rapier3d_urdf::UrdfLoaderOptions::default();
        let identity_iso: crate::na::Isometry<Real, _, 3> = rapier::math::Pose::IDENTITY.into();
        Self {
            create_colliders_from_collision_shapes,
            create_colliders_from_visual_shapes,
            apply_imported_mass_props,
            enable_joint_collisions,
            make_roots_fixed,
            trimesh_flags: trimesh_flags.unwrap_or(TriMeshFlags(default.trimesh_flags)),
            mesh_converter,
            shift: shift.unwrap_or(Isometry3(identity_iso)),
            scale,
            squeeze_empty_fixed_links,
            collider_blueprint,
            rigid_body_blueprint,
        }
    }

    /// Return a debug string with the main loader-option booleans.
    fn __repr__(&self) -> String {
        format!(
            "UrdfLoaderOptions(create_colliders_from_collision_shapes={}, create_colliders_from_visual_shapes={}, apply_imported_mass_props={}, enable_joint_collisions={}, make_roots_fixed={}, scale={})",
            self.create_colliders_from_collision_shapes,
            self.create_colliders_from_visual_shapes,
            self.apply_imported_mass_props,
            self.enable_joint_collisions,
            self.make_roots_fixed,
            self.scale,
        )
    }
}

impl UrdfLoaderOptions {
    pub(crate) fn to_rapier(&self) -> rapier3d_urdf::UrdfLoaderOptions {
        let mut o = rapier3d_urdf::UrdfLoaderOptions::default();
        o.create_colliders_from_collision_shapes = self.create_colliders_from_collision_shapes;
        o.create_colliders_from_visual_shapes = self.create_colliders_from_visual_shapes;
        o.apply_imported_mass_props = self.apply_imported_mass_props;
        o.enable_joint_collisions = self.enable_joint_collisions;
        o.make_roots_fixed = self.make_roots_fixed;
        o.trimesh_flags = self.trimesh_flags.0;
        o.mesh_converter = self.mesh_converter.as_ref().map(|m| m.0);
        o.shift = self.shift.0.into();
        o.scale = self.scale;
        o.squeeze_empty_fixed_links = self.squeeze_empty_fixed_links;
        if let Some(cb) = &self.collider_blueprint {
            o.collider_blueprint = cb.builder.clone();
        }
        if let Some(rb) = &self.rigid_body_blueprint {
            o.rigid_body_blueprint = rb.builder.clone();
        }
        o
    }
}

// ----- URDF read-only views ----------------------------------------

/// Read-only view of a URDF ``<link>`` element.
///
/// Wraps the raw ``urdf_rs::Link`` for inspection from Python.
/// Returned by :attr:`Robot.links`.
#[pyclass(name = "UrdfLink", module = "rapier")]
#[derive(Clone)]
pub struct UrdfLink {
    pub raw: urdf_rs::Link,
}

#[pymethods]
impl UrdfLink {
    /// Name of the link as declared in the URDF.
    #[getter]
    fn name(&self) -> String {
        self.raw.name.clone()
    }
    /// Number of ``<visual>`` elements on the link.
    #[getter]
    fn n_visuals(&self) -> usize {
        self.raw.visual.len()
    }
    /// Number of ``<collision>`` elements on the link.
    #[getter]
    fn n_collisions(&self) -> usize {
        self.raw.collision.len()
    }
    /// Mass from the link's ``<inertial>`` block.
    #[getter]
    fn mass(&self) -> f64 {
        self.raw.inertial.mass.value
    }
    /// Return the ``UrdfLink(...)`` repr.
    fn __repr__(&self) -> String {
        format!(
            "UrdfLink(name={:?}, n_visuals={}, n_collisions={}, mass={})",
            self.raw.name,
            self.raw.visual.len(),
            self.raw.collision.len(),
            self.raw.inertial.mass.value,
        )
    }
}

/// Read-only view of a URDF ``<joint>`` element.
///
/// Wraps the raw ``urdf_rs::Joint`` for inspection from Python.
/// Returned by :attr:`Robot.joints`.
#[pyclass(name = "UrdfJoint", module = "rapier")]
#[derive(Clone)]
pub struct UrdfJoint {
    pub raw: urdf_rs::Joint,
}

#[pymethods]
impl UrdfJoint {
    /// Joint name as declared in the URDF.
    #[getter]
    fn name(&self) -> String {
        self.raw.name.clone()
    }
    /// Joint kind as a debug string (``"Revolute"``, ``"Fixed"``, ...).
    #[getter]
    fn joint_type(&self) -> String {
        format!("{:?}", self.raw.joint_type)
    }
    /// Name of the parent link.
    #[getter]
    fn parent(&self) -> String {
        self.raw.parent.link.clone()
    }
    /// Name of the child link.
    #[getter]
    fn child(&self) -> String {
        self.raw.child.link.clone()
    }
    /// Axis vector ``[x, y, z]`` in the parent link's frame.
    #[getter]
    fn axis(&self) -> Vec<f64> {
        self.raw.axis.xyz.0.to_vec()
    }
    /// Return the ``UrdfJoint(...)`` repr.
    fn __repr__(&self) -> String {
        format!(
            "UrdfJoint(name={:?}, type={:?}, parent={:?}, child={:?})",
            self.raw.name, self.raw.joint_type, self.raw.parent.link, self.raw.child.link,
        )
    }
}

/// Raw URDF document view (links + joints, pre-simulation).
///
/// Returned by :meth:`UrdfRobot.from_file` / :meth:`from_str`
/// alongside the simulation-ready :class:`UrdfRobot`. Use this to
/// inspect link / joint names before insertion, or to re-import the
/// same source via :meth:`UrdfRobot.from_robot`.
#[pyclass(name = "Robot", module = "rapier")]
#[derive(Clone)]
pub struct UrdfRobotSource {
    pub raw: urdf_rs::Robot,
}

#[pymethods]
impl UrdfRobotSource {
    /// Robot name from the URDF ``<robot name="...">`` attribute.
    #[getter]
    fn name(&self) -> String {
        self.raw.name.clone()
    }
    /// List of :class:`UrdfLink` views, in document order.
    #[getter]
    fn links(&self) -> Vec<UrdfLink> {
        self.raw
            .links
            .iter()
            .map(|l| UrdfLink { raw: l.clone() })
            .collect()
    }
    /// List of :class:`UrdfJoint` views, in document order.
    #[getter]
    fn joints(&self) -> Vec<UrdfJoint> {
        self.raw
            .joints
            .iter()
            .map(|j| UrdfJoint { raw: j.clone() })
            .collect()
    }
    /// Return the ``Robot(...)`` repr.
    fn __repr__(&self) -> String {
        format!(
            "Robot(name={:?}, n_links={}, n_joints={})",
            self.raw.name,
            self.raw.links.len(),
            self.raw.joints.len(),
        )
    }
}

// ----- URDF handles ------------------------------------------------

/// Visual mesh override paired with a URDF collider.
///
/// Populated by the loader only when a non-default
/// :attr:`UrdfLoaderOptions.mesh_converter` (e.g.
/// ``MeshConverter.OBB``) replaced the source mesh with a cheap
/// proxy collider — this keeps the original high-resolution mesh
/// available for rendering.
///
/// :ivar shape: The :class:`SharedShape` representing the visual mesh.
/// :ivar local_pose: Pose of the visual mesh in the collider's local
///     frame, as an :class:`Isometry3`.
#[pyclass(name = "UrdfVisual", module = "rapier")]
#[derive(Clone)]
pub struct UrdfVisual {
    #[pyo3(get)]
    pub shape: SharedShape,
    #[pyo3(get)]
    pub local_pose: Isometry3,
}

/// Handle of one collider inserted from a URDF link.
///
/// :ivar handle: Underlying :class:`ColliderHandle`.
/// :ivar visual: Optional :class:`UrdfVisual` mesh override (``None``
///     unless a proxy :attr:`UrdfLoaderOptions.mesh_converter` was used).
#[pyclass(name = "UrdfColliderHandle", module = "rapier", frozen)]
#[derive(Clone, Debug)]
pub struct UrdfColliderHandle {
    #[pyo3(get)]
    pub handle: ColliderHandle,
    pub visual: Option<rapier3d_urdf::UrdfVisual>,
}

#[pymethods]
impl UrdfColliderHandle {
    /// Optional visual mesh override for this collider, or ``None``.
    #[getter]
    fn visual(&self) -> Option<UrdfVisual> {
        self.visual.as_ref().map(|v| UrdfVisual {
            shape: SharedShape(v.shape.clone()),
            local_pose: Isometry3(v.local_pose.into()),
        })
    }
    /// Return the ``UrdfColliderHandle(...)`` repr.
    fn __repr__(&self) -> String {
        let (i, g) = self.handle.0.into_raw_parts();
        format!(
            "UrdfColliderHandle(handle=ColliderHandle(index={i}, generation={g}), has_visual={})",
            if self.visual.is_some() {
                "True"
            } else {
                "False"
            }
        )
    }
}

/// Handle of one URDF link (its rigid body plus its colliders).
///
/// :ivar body: :class:`RigidBodyHandle` for the link's body.
/// :ivar colliders: List of :class:`UrdfColliderHandle` attached to it.
#[pyclass(name = "UrdfLinkHandle", module = "rapier", frozen)]
#[derive(Clone, Debug)]
pub struct UrdfLinkHandle {
    #[pyo3(get)]
    pub body: RigidBodyHandle,
    #[pyo3(get)]
    pub colliders: Vec<UrdfColliderHandle>,
}

#[pymethods]
impl UrdfLinkHandle {
    /// Return the ``UrdfLinkHandle(...)`` repr.
    fn __repr__(&self) -> String {
        let (i, g) = self.body.0.into_raw_parts();
        format!(
            "UrdfLinkHandle(body=RigidBodyHandle(index={i}, generation={g}), n_colliders={})",
            self.colliders.len()
        )
    }
}

/// Handle of one URDF joint after insertion.
///
/// :ivar joint: Either an :class:`ImpulseJointHandle` (when the
///     impulse-joint insertion path was used) or
///     ``Optional[MultibodyJointHandle]`` (multibody path; ``None``
///     for collapsed fixed joints).
/// :ivar link1: Parent link's :class:`RigidBodyHandle`.
/// :ivar link2: Child link's :class:`RigidBodyHandle`.
#[pyclass(name = "UrdfJointHandle", module = "rapier")]
pub struct UrdfJointHandle {
    #[pyo3(get)]
    pub joint: crate::pyo3::PyObject,
    #[pyo3(get)]
    pub link1: RigidBodyHandle,
    #[pyo3(get)]
    pub link2: RigidBodyHandle,
}

/// Aggregate handle set returned by URDF insertion functions.
///
/// :ivar links: One :class:`UrdfLinkHandle` per URDF link.
/// :ivar joints: One :class:`UrdfJointHandle` per URDF joint.
#[pyclass(name = "UrdfRobotHandles", module = "rapier")]
pub struct UrdfRobotHandles {
    #[pyo3(get)]
    pub links: Vec<UrdfLinkHandle>,
    #[pyo3(get)]
    pub joints: Vec<crate::pyo3::Py<UrdfJointHandle>>,
}

#[pymethods]
impl UrdfRobotHandles {
    /// Return the ``UrdfRobotHandles(...)`` repr.
    fn __repr__(&self) -> String {
        format!(
            "UrdfRobotHandles(n_links={}, n_joints={})",
            self.links.len(),
            self.joints.len()
        )
    }
}

// ----- UrdfRobot ---------------------------------------------------

/// A URDF robot, ready to be inserted into the simulation.
///
/// Build one via :meth:`from_file`, :meth:`from_str`, or
/// :meth:`from_robot`, optionally transform the whole robot with
/// :meth:`append_transform`, then commit it to the world via either
/// :meth:`insert_using_impulse_joints` (full 6-DOF joints with
/// constraints) or :meth:`insert_using_multibody_joints`
/// (reduced-coordinate articulation).
///
/// The insertion methods *consume* the robot — subsequent calls
/// raise :class:`UrdfError`.
#[pyclass(name = "UrdfRobot", module = "rapier", unsendable)]
pub struct UrdfRobot {
    pub inner: Option<rapier3d_urdf::UrdfRobot>,
}

#[pymethods]
impl UrdfRobot {
    /// Parse a URDF file and return ``(UrdfRobot, Robot)``.
    ///
    /// Mesh references inside the URDF are resolved relative to
    /// ``mesh_dir`` (default: the URDF file's parent directory).
    ///
    /// :param path: Path to the ``.urdf`` file on disk.
    /// :param options: Optional :class:`UrdfLoaderOptions`.
    /// :param mesh_dir: Directory used to resolve mesh refs.
    /// :returns: ``(UrdfRobot, Robot)`` tuple — sim-ready robot
    ///     plus the raw document view.
    /// :raises UrdfError: if the file can't be opened or parsed.
    #[staticmethod]
    #[pyo3(signature = (path, options=None, mesh_dir=None))]
    fn from_file(
        path: &str,
        options: Option<UrdfLoaderOptions>,
        mesh_dir: Option<&str>,
    ) -> crate::pyo3::PyResult<(UrdfRobot, UrdfRobotSource)> {
        let opts = options.map(|o| o.to_rapier()).unwrap_or_default();
        let mesh_dir = mesh_dir.map(_StdPath::new);
        let (robot, raw) = rapier3d_urdf::UrdfRobot::from_file(path, opts, mesh_dir)
            .map_err(|e| crate::errors::UrdfError::new_err(format!("{e}")))?;
        Ok((UrdfRobot { inner: Some(robot) }, UrdfRobotSource { raw }))
    }

    /// Parse a URDF XML string and return ``(UrdfRobot, Robot)``.
    ///
    /// Equivalent to :meth:`from_file` but the source XML is
    /// provided in memory. Mesh references are resolved relative
    /// to ``mesh_dir`` (default: the current working directory).
    ///
    /// :param xml: Full URDF XML document.
    /// :param options: Optional :class:`UrdfLoaderOptions`.
    /// :param mesh_dir: Directory used to resolve mesh refs.
    /// :returns: ``(UrdfRobot, Robot)`` tuple.
    /// :raises UrdfError: if the XML can't be parsed.
    #[staticmethod]
    #[pyo3(signature = (xml, options=None, mesh_dir=None))]
    fn from_str(
        xml: &str,
        options: Option<UrdfLoaderOptions>,
        mesh_dir: Option<&str>,
    ) -> crate::pyo3::PyResult<(UrdfRobot, UrdfRobotSource)> {
        let opts = options.map(|o| o.to_rapier()).unwrap_or_default();
        let mesh_dir = mesh_dir
            .map(_StdPath::new)
            .unwrap_or_else(|| _StdPath::new("."));
        let (robot, raw) = rapier3d_urdf::UrdfRobot::from_str(xml, opts, mesh_dir)
            .map_err(|e| crate::errors::UrdfError::new_err(format!("{e}")))?;
        Ok((UrdfRobot { inner: Some(robot) }, UrdfRobotSource { raw }))
    }

    /// Build a sim-ready :class:`UrdfRobot` from a previously
    /// parsed :class:`Robot` view.
    ///
    /// Lets you re-import the same document with different loader
    /// options without re-parsing.
    ///
    /// :param robot: Source :class:`Robot` document view.
    /// :param options: Optional :class:`UrdfLoaderOptions`.
    /// :param mesh_dir: Directory used to resolve mesh refs.
    #[staticmethod]
    #[pyo3(signature = (robot, options=None, mesh_dir=None))]
    fn from_robot(
        robot: &UrdfRobotSource,
        options: Option<UrdfLoaderOptions>,
        mesh_dir: Option<&str>,
    ) -> crate::pyo3::PyResult<UrdfRobot> {
        let opts = options.map(|o| o.to_rapier()).unwrap_or_default();
        let mesh_dir = mesh_dir
            .map(_StdPath::new)
            .unwrap_or_else(|| _StdPath::new("."));
        let r = rapier3d_urdf::UrdfRobot::from_robot(&robot.raw, opts, mesh_dir);
        Ok(UrdfRobot { inner: Some(r) })
    }

    /// Number of links waiting to be inserted.
    ///
    /// :raises UrdfError: if this robot has already been consumed.
    #[getter]
    fn n_links(&self) -> crate::pyo3::PyResult<usize> {
        self.inner
            .as_ref()
            .map(|r| r.links.len())
            .ok_or_else(|| crate::errors::UrdfError::new_err("UrdfRobot was already consumed"))
    }

    /// Number of joints waiting to be inserted.
    ///
    /// :raises UrdfError: if this robot has already been consumed.
    #[getter]
    fn n_joints(&self) -> crate::pyo3::PyResult<usize> {
        self.inner
            .as_ref()
            .map(|r| r.joints.len())
            .ok_or_else(|| crate::errors::UrdfError::new_err("UrdfRobot was already consumed"))
    }

    /// Apply a rigid transform to every body of the robot.
    ///
    /// Mutates the robot in place. World-anchored joints are kept
    /// satisfied across the transform.
    ///
    /// :param iso: World-space :class:`Isometry3` to prepend to
    ///     every body pose.
    /// :raises UrdfError: if this robot has already been consumed.
    fn append_transform(&mut self, iso: Isometry3) -> crate::pyo3::PyResult<()> {
        let robot = self
            .inner
            .as_mut()
            .ok_or_else(|| crate::errors::UrdfError::new_err("UrdfRobot was already consumed"))?;
        let pose: rapier::math::Pose = iso.0.into();
        robot.append_transform(&pose);
        Ok(())
    }

    /// Insert the robot into the world using *impulse joints*.
    ///
    /// Each URDF joint becomes a full 6-DOF impulse joint with
    /// position/velocity constraints. Compared to the multibody
    /// path this is simpler conceptually but typically slower and
    /// less stable for long kinematic chains.
    ///
    /// This call **consumes** the :class:`UrdfRobot`.
    ///
    /// :param bodies: :class:`RigidBodySet` to insert into.
    /// :param colliders: :class:`ColliderSet` to insert into.
    /// :param joints: :class:`ImpulseJointSet` to insert into.
    /// :returns: :class:`UrdfRobotHandles` with all created handles.
    /// :raises UrdfError: if this robot has already been consumed.
    fn insert_using_impulse_joints(
        &mut self,
        py: crate::pyo3::Python<'_>,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        joints: &mut ImpulseJointSet,
    ) -> crate::pyo3::PyResult<crate::pyo3::Py<UrdfRobotHandles>> {
        use crate::pyo3::IntoPy;
        let robot = self
            .inner
            .take()
            .ok_or_else(|| crate::errors::UrdfError::new_err("UrdfRobot was already consumed"))?;
        let handles =
            robot.insert_using_impulse_joints(&mut bodies.0, &mut colliders.0, &mut joints.0);
        let links: Vec<UrdfLinkHandle> = handles
            .links
            .into_iter()
            .map(|lh| {
                let cols = lh
                    .colliders
                    .into_iter()
                    .map(|h| UrdfColliderHandle {
                        handle: ColliderHandle(h.handle),
                        visual: h.visual,
                    })
                    .collect();
                UrdfLinkHandle {
                    body: RigidBodyHandle(lh.body),
                    colliders: cols,
                }
            })
            .collect();
        let joints_handles: Vec<crate::pyo3::Py<UrdfJointHandle>> = handles
            .joints
            .into_iter()
            .map(|jh| {
                let joint_py = ImpulseJointHandle(jh.joint).into_py(py);
                crate::pyo3::Py::new(
                    py,
                    UrdfJointHandle {
                        joint: joint_py,
                        link1: RigidBodyHandle(jh.link1),
                        link2: RigidBodyHandle(jh.link2),
                    },
                )
                .expect("alloc UrdfJointHandle")
            })
            .collect();
        crate::pyo3::Py::new(
            py,
            UrdfRobotHandles {
                links,
                joints: joints_handles,
            },
        )
    }

    /// Insert the robot into the world using *multibody joints*.
    ///
    /// Builds a reduced-coordinate articulation: each link's pose
    /// is implicitly constrained by its joint, eliminating most
    /// drift on long kinematic chains. Recommended for robotic
    /// arms, humanoids, and other branched mechanisms.
    ///
    /// This call **consumes** the :class:`UrdfRobot`.
    ///
    /// :param bodies: :class:`RigidBodySet` to insert into.
    /// :param colliders: :class:`ColliderSet` to insert into.
    /// :param multibody_joints: :class:`MultibodyJointSet` to insert into.
    /// :param options: Optional :class:`UrdfMultibodyOptions` flags.
    /// :returns: :class:`UrdfRobotHandles` with all created handles.
    /// :raises UrdfError: if this robot has already been consumed.
    #[pyo3(signature = (bodies, colliders, multibody_joints, options=None))]
    fn insert_using_multibody_joints(
        &mut self,
        py: crate::pyo3::Python<'_>,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        multibody_joints: &mut MultibodyJointSet,
        options: Option<UrdfMultibodyOptions>,
    ) -> crate::pyo3::PyResult<crate::pyo3::Py<UrdfRobotHandles>> {
        use crate::pyo3::IntoPy;
        let robot = self
            .inner
            .take()
            .ok_or_else(|| crate::errors::UrdfError::new_err("UrdfRobot was already consumed"))?;
        let opts = options.map(|o| o.0).unwrap_or_default();
        let handles = robot.insert_using_multibody_joints(
            &mut bodies.0,
            &mut colliders.0,
            &mut multibody_joints.0,
            opts,
        );
        let links: Vec<UrdfLinkHandle> = handles
            .links
            .into_iter()
            .map(|lh| {
                let cols = lh
                    .colliders
                    .into_iter()
                    .map(|h| UrdfColliderHandle {
                        handle: ColliderHandle(h.handle),
                        visual: h.visual,
                    })
                    .collect();
                UrdfLinkHandle {
                    body: RigidBodyHandle(lh.body),
                    colliders: cols,
                }
            })
            .collect();
        let joints_handles: Vec<crate::pyo3::Py<UrdfJointHandle>> = handles
            .joints
            .into_iter()
            .map(|jh| {
                let joint_py = jh.joint.map(MultibodyJointHandle).into_py(py);
                crate::pyo3::Py::new(
                    py,
                    UrdfJointHandle {
                        joint: joint_py,
                        link1: RigidBodyHandle(jh.link1),
                        link2: RigidBodyHandle(jh.link2),
                    },
                )
                .expect("alloc UrdfJointHandle")
            })
            .collect();
        crate::pyo3::Py::new(
            py,
            UrdfRobotHandles {
                links,
                joints: joints_handles,
            },
        )
    }

    /// Return the ``UrdfRobot(...)`` repr (or ``"UrdfRobot(consumed)"``
    /// after insertion).
    fn __repr__(&self) -> String {
        match &self.inner {
            Some(r) => format!(
                "UrdfRobot(n_links={}, n_joints={})",
                r.links.len(),
                r.joints.len()
            ),
            None => "UrdfRobot(consumed)".to_string(),
        }
    }
}

/// Register the URDF loader `#[pyclass]`-es into `m`.
pub fn register_loaders_urdf(
    m: &crate::pyo3::Bound<'_, crate::pyo3::types::PyModule>,
) -> crate::pyo3::PyResult<()> {
    use crate::pyo3::prelude::*;
    m.add_class::<UrdfMultibodyOptions>()?;
    m.add_class::<UrdfLoaderOptions>()?;
    m.add_class::<UrdfLink>()?;
    m.add_class::<UrdfJoint>()?;
    m.add_class::<UrdfRobotSource>()?;
    m.add_class::<UrdfVisual>()?;
    m.add_class::<UrdfColliderHandle>()?;
    m.add_class::<UrdfLinkHandle>()?;
    m.add_class::<UrdfJointHandle>()?;
    m.add_class::<UrdfRobotHandles>()?;
    m.add_class::<UrdfRobot>()?;
    Ok(())
}

// ==============================================================
// MJCF loader
// ==============================================================

/// Options applied to multibody joints created from MJCF joints.
///
/// A flag set; combine flags with ``|`` and pass the result to
/// :meth:`MjcfRobot.insert_using_multibody_joints`.
#[pyclass(name = "MjcfMultibodyOptions", module = "rapier", frozen)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MjcfMultibodyOptions(pub rapier3d_mjcf::MjcfMultibodyOptions);

#[pymethods]
impl MjcfMultibodyOptions {
    /// Build from a raw bit pattern.
    #[new]
    #[pyo3(signature = (bits = 0))]
    fn new(bits: u8) -> Self {
        Self(rapier3d_mjcf::MjcfMultibodyOptions::from_bits_truncate(
            bits,
        ))
    }
    /// Return the empty flag set (no flags).
    #[staticmethod]
    fn empty() -> Self {
        Self(rapier3d_mjcf::MjcfMultibodyOptions::empty())
    }
    /// Treat the created multibody joints as kinematic.
    #[classattr]
    const JOINTS_ARE_KINEMATIC: MjcfMultibodyOptions =
        MjcfMultibodyOptions(rapier3d_mjcf::MjcfMultibodyOptions::JOINTS_ARE_KINEMATIC);
    /// Disable self-collisions between links of the same multibody.
    #[classattr]
    const DISABLE_SELF_CONTACTS: MjcfMultibodyOptions =
        MjcfMultibodyOptions(rapier3d_mjcf::MjcfMultibodyOptions::DISABLE_SELF_CONTACTS);
    /// Do not insert ``<equality>`` loop-closure constraints as
    /// impulse joints alongside the multibody.
    #[classattr]
    const SKIP_LOOP_CLOSURES: MjcfMultibodyOptions =
        MjcfMultibodyOptions(rapier3d_mjcf::MjcfMultibodyOptions::SKIP_LOOP_CLOSURES);
    /// Strip the per-joint motors baked in by the loader before
    /// handing joints to the multibody solver.
    #[classattr]
    const SKIP_JOINT_MOTORS: MjcfMultibodyOptions =
        MjcfMultibodyOptions(rapier3d_mjcf::MjcfMultibodyOptions::SKIP_JOINT_MOTORS);
    /// Strip per-joint limits before handing joints to the multibody.
    #[classattr]
    const SKIP_JOINT_LIMITS: MjcfMultibodyOptions =
        MjcfMultibodyOptions(rapier3d_mjcf::MjcfMultibodyOptions::SKIP_JOINT_LIMITS);
    /// Don't install the ``<joint stiffness>`` passive springs on the
    /// multibody joints.
    #[classattr]
    const SKIP_JOINT_SPRINGS: MjcfMultibodyOptions =
        MjcfMultibodyOptions(rapier3d_mjcf::MjcfMultibodyOptions::SKIP_JOINT_SPRINGS);
    /// Raw bits as an unsigned int.
    #[getter]
    fn bits(&self) -> u8 {
        self.0.bits()
    }
    /// Bitwise OR: union of two flag sets.
    fn __or__(&self, other: &MjcfMultibodyOptions) -> Self {
        Self(self.0 | other.0)
    }
    /// Bitwise AND: intersection of two flag sets.
    fn __and__(&self, other: &MjcfMultibodyOptions) -> Self {
        Self(self.0 & other.0)
    }
    /// ``other in self``: ``True`` if every flag of ``other`` is set here.
    fn __contains__(&self, other: &MjcfMultibodyOptions) -> bool {
        self.0.contains(other.0)
    }
    /// Return ``MjcfMultibodyOptions(bits=0b...)`` repr.
    fn __repr__(&self) -> String {
        format!("MjcfMultibodyOptions(bits={:#08b})", self.0.bits())
    }
}

/// How MJCF ``contype`` / ``conaffinity`` masks map onto rapier
/// :class:`InteractionGroups`.
#[pyclass(
    name = "ContactFilterMode",
    module = "rapier",
    eq,
    eq_int,
    hash,
    frozen
)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum ContactFilterMode {
    /// ``memberships = filter = contype | conaffinity`` (default).
    Symmetric,
    /// ``memberships = contype``, ``filter = conaffinity``.
    Asymmetric,
}

impl ContactFilterMode {
    pub(crate) fn to_rapier(self) -> rapier3d_mjcf::ContactFilterMode {
        match self {
            ContactFilterMode::Symmetric => rapier3d_mjcf::ContactFilterMode::Symmetric,
            ContactFilterMode::Asymmetric => rapier3d_mjcf::ContactFilterMode::Asymmetric,
        }
    }
}

/// Configuration for the MJCF loader.
///
/// Mirrors the common subset of ``rapier3d_mjcf::MjcfLoaderOptions``.
/// Pass an instance to :meth:`MjcfRobot.from_file` / :meth:`from_str`.
///
/// :ivar create_colliders_from_collision_shapes: Build colliders for
///     ``<geom>`` elements that participate in contact. Default ``True``.
/// :ivar create_colliders_from_visual_shapes: Build colliders for
///     visual-only ``<geom>`` elements. Default ``False``.
/// :ivar apply_imported_mass_props: Use the model's inertial data for
///     body mass properties. Default ``True``.
/// :ivar enable_joint_collisions: Allow contacts between two bodies
///     sharing a joint. Default ``False``.
/// :ivar make_roots_fixed: Create root bodies as ``Fixed``. Default
///     ``False``.
/// :ivar trimesh_flags: :class:`TriMeshFlags` applied to mesh colliders.
/// :ivar mesh_converter: Optional :class:`MeshConverter` controlling
///     how meshes become collider shapes. Default ``None`` (trimesh).
/// :ivar shift: Rigid transform appended to every created body
///     (applied after ``scale``).
/// :ivar scale: Uniform scale applied to lengths read from the MJCF.
///     Default ``1.0``.
/// :ivar skip_plane_geoms: Skip ``<geom type="plane">`` elements
///     entirely. Default ``True``.
/// :ivar disable_joint_motors: Skip the per-joint motor setup the
///     loader normally applies (springs / friction). Default ``False``.
#[pyclass(name = "MjcfLoaderOptions", module = "rapier")]
#[derive(Clone)]
pub struct MjcfLoaderOptions {
    #[pyo3(get, set)]
    pub create_colliders_from_collision_shapes: bool,
    #[pyo3(get, set)]
    pub create_colliders_from_visual_shapes: bool,
    #[pyo3(get, set)]
    pub apply_imported_mass_props: bool,
    #[pyo3(get, set)]
    pub enable_joint_collisions: bool,
    #[pyo3(get, set)]
    pub make_roots_fixed: bool,
    #[pyo3(get, set)]
    pub trimesh_flags: TriMeshFlags,
    #[pyo3(get, set)]
    pub mesh_converter: Option<MeshConverter>,
    #[pyo3(get, set)]
    pub shift: Isometry3,
    #[pyo3(get, set)]
    pub scale: Real,
    #[pyo3(get, set)]
    pub skip_plane_geoms: bool,
    #[pyo3(get, set)]
    pub disable_joint_motors: bool,
    /// Template collider applied to every imported collider. ``None``
    /// keeps the rapier default (density 0).
    #[pyo3(get, set)]
    pub collider_blueprint: Option<ColliderBuilder>,
    /// Template rigid-body used for every imported body. ``None``
    /// keeps the rapier default (a dynamic body).
    #[pyo3(get, set)]
    pub rigid_body_blueprint: Option<RigidBodyBuilder>,
    /// How ``contype``/``conaffinity`` map onto collision groups.
    #[pyo3(get, set)]
    pub contact_filter_mode: ContactFilterMode,
}

#[pymethods]
impl MjcfLoaderOptions {
    /// Build an ``MjcfLoaderOptions`` with optional overrides.
    #[new]
    #[pyo3(signature = (
        create_colliders_from_collision_shapes = true,
        create_colliders_from_visual_shapes = false,
        apply_imported_mass_props = true,
        enable_joint_collisions = false,
        make_roots_fixed = false,
        trimesh_flags = None,
        mesh_converter = None,
        shift = None,
        scale = 1.0,
        skip_plane_geoms = true,
        disable_joint_motors = false,
        collider_blueprint = None,
        rigid_body_blueprint = None,
        contact_filter_mode = ContactFilterMode::Symmetric,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        create_colliders_from_collision_shapes: bool,
        create_colliders_from_visual_shapes: bool,
        apply_imported_mass_props: bool,
        enable_joint_collisions: bool,
        make_roots_fixed: bool,
        trimesh_flags: Option<TriMeshFlags>,
        mesh_converter: Option<MeshConverter>,
        shift: Option<Isometry3>,
        scale: Real,
        skip_plane_geoms: bool,
        disable_joint_motors: bool,
        collider_blueprint: Option<ColliderBuilder>,
        rigid_body_blueprint: Option<RigidBodyBuilder>,
        contact_filter_mode: ContactFilterMode,
    ) -> Self {
        let default = rapier3d_mjcf::MjcfLoaderOptions::default();
        let identity_iso: crate::na::Isometry<Real, _, 3> = rapier::math::Pose::IDENTITY.into();
        Self {
            create_colliders_from_collision_shapes,
            create_colliders_from_visual_shapes,
            apply_imported_mass_props,
            enable_joint_collisions,
            make_roots_fixed,
            trimesh_flags: trimesh_flags.unwrap_or(TriMeshFlags(default.trimesh_flags)),
            mesh_converter,
            shift: shift.unwrap_or(Isometry3(identity_iso)),
            scale,
            skip_plane_geoms,
            disable_joint_motors,
            collider_blueprint,
            rigid_body_blueprint,
            contact_filter_mode,
        }
    }

    /// Return a debug string with the main loader-option booleans.
    fn __repr__(&self) -> String {
        format!(
            "MjcfLoaderOptions(create_colliders_from_collision_shapes={}, create_colliders_from_visual_shapes={}, make_roots_fixed={}, scale={}, skip_plane_geoms={})",
            self.create_colliders_from_collision_shapes,
            self.create_colliders_from_visual_shapes,
            self.make_roots_fixed,
            self.scale,
            self.skip_plane_geoms,
        )
    }
}

impl MjcfLoaderOptions {
    pub(crate) fn to_rapier(&self) -> rapier3d_mjcf::MjcfLoaderOptions {
        let mut o = rapier3d_mjcf::MjcfLoaderOptions::default();
        o.create_colliders_from_collision_shapes = self.create_colliders_from_collision_shapes;
        o.create_colliders_from_visual_shapes = self.create_colliders_from_visual_shapes;
        o.apply_imported_mass_props = self.apply_imported_mass_props;
        o.enable_joint_collisions = self.enable_joint_collisions;
        o.make_roots_fixed = self.make_roots_fixed;
        o.trimesh_flags = self.trimesh_flags.0;
        o.mesh_converter = self.mesh_converter.as_ref().map(|m| m.0);
        o.shift = self.shift.0.into();
        o.scale = self.scale;
        o.skip_plane_geoms = self.skip_plane_geoms;
        o.disable_joint_motors = self.disable_joint_motors;
        o.contact_filter_mode = self.contact_filter_mode.to_rapier();
        if let Some(cb) = &self.collider_blueprint {
            o.collider_blueprint = cb.builder.clone();
        }
        if let Some(rb) = &self.rigid_body_blueprint {
            o.rigid_body_blueprint = rb.builder.clone();
        }
        o
    }
}

/// Parsed MJCF model document (read-only).
///
/// Returned by :meth:`MjcfRobot.from_file` / :meth:`from_str`
/// alongside the simulation-ready :class:`MjcfRobot`.
#[pyclass(name = "MjcfModel", module = "rapier", unsendable)]
pub struct MjcfModel {
    pub raw: rapier3d_mjcf::mjcf_rs::model::Model,
}

#[pymethods]
impl MjcfModel {
    /// The ``<mujoco model="...">`` name, if any.
    #[getter]
    fn name(&self) -> Option<String> {
        self.raw.name.clone()
    }
    /// Return the ``MjcfModel(...)`` repr.
    fn __repr__(&self) -> String {
        match &self.raw.name {
            Some(name) => format!("MjcfModel(name={name:?})"),
            None => "MjcfModel(name=None)".to_string(),
        }
    }
}

/// Handle of one collider inserted from an MJCF ``<geom>``.
///
/// :ivar handle: Underlying :class:`ColliderHandle`.
#[pyclass(name = "MjcfColliderHandle", module = "rapier", frozen)]
#[derive(Clone, Debug)]
pub struct MjcfColliderHandle {
    #[pyo3(get)]
    pub handle: ColliderHandle,
}

/// Handle of one MJCF body (its rigid body plus its colliders).
///
/// :ivar body: :class:`RigidBodyHandle` for the body.
/// :ivar colliders: List of :class:`MjcfColliderHandle` attached to it.
#[pyclass(name = "MjcfBodyHandle", module = "rapier", frozen)]
#[derive(Clone, Debug)]
pub struct MjcfBodyHandle {
    #[pyo3(get)]
    pub body: RigidBodyHandle,
    #[pyo3(get)]
    pub colliders: Vec<MjcfColliderHandle>,
}

/// Handle of one MJCF joint after insertion.
///
/// :ivar joint: Either an :class:`ImpulseJointHandle` (impulse-joint
///     path) or ``Optional[MultibodyJointHandle]`` (multibody path).
/// :ivar link1: Parent body's :class:`RigidBodyHandle`.
/// :ivar link2: Child body's :class:`RigidBodyHandle`.
#[pyclass(name = "MjcfJointHandle", module = "rapier")]
pub struct MjcfJointHandle {
    #[pyo3(get)]
    pub joint: crate::pyo3::PyObject,
    #[pyo3(get)]
    pub link1: RigidBodyHandle,
    #[pyo3(get)]
    pub link2: RigidBodyHandle,
}

/// Handle of one MJCF ``<actuator>`` after insertion.
///
/// :ivar name: The actuator's ``name`` attribute, if any.
/// :ivar kind: Actuator subtype, one of ``"Motor"``, ``"Position"``,
///     ``"Velocity"``, ``"IntVelocity"``, ``"Damper"``, ``"General"``.
/// :ivar joint_name: Name of the MJCF joint it drives, if any.
/// :ivar joint: The joint the actuator drives: an
///     :class:`ImpulseJointHandle` (impulse-joint path), a
///     :class:`MultibodyJointHandle` (multibody path), or ``None`` if it
///     drives no joint (e.g. a tendon) or its joint was dropped as a loop
///     closure.
/// :ivar gear: The six ``gear`` entries.
/// :ivar ctrl_range: ``[min, max]`` from ``ctrlrange``, or ``None``.
/// :ivar force_range: ``[min, max]`` from ``forcerange``, or ``None``.
/// :ivar kp: ``kp`` of a ``<position>`` actuator, or ``None``.
/// :ivar kv: ``kv`` of a ``<velocity>`` / ``<damper>`` actuator, or
///     ``None``.
#[pyclass(name = "MjcfActuatorHandle", module = "rapier")]
pub struct MjcfActuatorHandle {
    #[pyo3(get)]
    pub name: Option<String>,
    #[pyo3(get)]
    pub kind: String,
    #[pyo3(get)]
    pub joint_name: Option<String>,
    #[pyo3(get)]
    pub joint: crate::pyo3::PyObject,
    #[pyo3(get)]
    pub gear: Vec<f64>,
    #[pyo3(get)]
    pub ctrl_range: Option<[f64; 2]>,
    #[pyo3(get)]
    pub force_range: Option<[f64; 2]>,
    #[pyo3(get)]
    pub kp: Option<f64>,
    #[pyo3(get)]
    pub kv: Option<f64>,
}

#[pymethods]
impl MjcfActuatorHandle {
    /// Return the ``MjcfActuatorHandle(...)`` repr.
    fn __repr__(&self, py: crate::pyo3::Python<'_>) -> crate::pyo3::PyResult<String> {
        let name = match &self.name {
            Some(name) => format!("{name:?}"),
            None => "None".to_string(),
        };
        let joint = self.joint.bind(py).repr()?;
        Ok(format!("MjcfActuatorHandle(name={name}, joint={joint})"))
    }
}

/// A keyframe of an MJCF model, by index (negative indices count from the end) or by name.
#[derive(crate::pyo3::FromPyObject)]
pub enum MjcfKeyframeRef {
    Index(isize),
    Name(String),
}

/// The handles returned by the insertion, with the joint type of the path used.
enum MjcfInsertedHandles {
    Impulse(rapier3d_mjcf::MjcfRobotHandles<rapier::dynamics::ImpulseJointHandle>),
    Multibody(rapier3d_mjcf::MjcfRobotHandles<Option<rapier::dynamics::MultibodyJointHandle>>),
}

/// Aggregate handle set returned by MJCF insertion functions.
///
/// Besides the handles, it keeps the model data needed by the runtime
/// helpers: :meth:`apply_controls` (actuators), :meth:`apply_keyframe` /
/// :meth:`keyframe_controls` (``<keyframe>``) and :meth:`contact_hooks`
/// (``<contact>``).
///
/// :ivar bodies: One ``Optional[MjcfBodyHandle]`` per MJCF body, in
///     model order (entry 0 is the implicit world body and is usually
///     ``None``).
/// :ivar joints: One :class:`MjcfJointHandle` per inserted joint.
/// :ivar equality_joints: One :class:`MjcfJointHandle` per
///     ``<equality>`` loop-closure constraint (always impulse joints).
/// :ivar actuators: One :class:`MjcfActuatorHandle` per ``<actuator>``,
///     in model order (the order of the ``ctrl`` values of
///     :meth:`apply_controls`).
/// :ivar body_names: MJCF name of each body, aligned with
///     :py:attr:`bodies`. Copied off the robot at insertion time, since
///     inserting consumes it.
/// :ivar joint_names: MJCF name of each joint, aligned with
///     :py:attr:`joints`.
/// :ivar body_name_to_idx: Body name to index into :py:attr:`bodies`.
/// :ivar joint_name_to_idx: Joint name to index into
///     :py:attr:`joints`.
#[pyclass(name = "MjcfRobotHandles", module = "rapier")]
pub struct MjcfRobotHandles {
    #[pyo3(get)]
    pub bodies: Vec<Option<MjcfBodyHandle>>,
    #[pyo3(get)]
    pub joints: Vec<crate::pyo3::Py<MjcfJointHandle>>,
    #[pyo3(get)]
    pub equality_joints: Vec<crate::pyo3::Py<MjcfJointHandle>>,
    #[pyo3(get)]
    pub actuators: Vec<crate::pyo3::Py<MjcfActuatorHandle>>,
    #[pyo3(get)]
    pub body_names: Vec<Option<String>>,
    #[pyo3(get)]
    pub joint_names: Vec<Option<String>>,
    #[pyo3(get)]
    pub body_name_to_idx: std::collections::HashMap<String, usize>,
    #[pyo3(get)]
    pub joint_name_to_idx: std::collections::HashMap<String, usize>,
    /// The inserted robot, without its colliders and visual meshes (only its
    /// metadata is needed by the runtime helpers).
    robot: Box<rapier3d_mjcf::MjcfRobot>,
    inserted: MjcfInsertedHandles,
}

impl MjcfRobotHandles {
    /// Keep what the runtime helpers need from `robot` before it is consumed by an insertion.
    fn robot_metadata(robot: &rapier3d_mjcf::MjcfRobot) -> Box<rapier3d_mjcf::MjcfRobot> {
        let mut robot = robot.clone();
        for body in &mut robot.bodies {
            body.colliders.clear();
            body.visual_meshes.clear();
        }
        Box::new(robot)
    }

    fn keyframe(
        &self,
        keyframe: &MjcfKeyframeRef,
    ) -> crate::pyo3::PyResult<&rapier3d_mjcf::mjcf_rs::extras::Keyframe> {
        let keyframes = &self.robot.keyframes;
        let index = match keyframe {
            MjcfKeyframeRef::Name(name) => {
                return self.robot.keyframe_by_name(name).ok_or_else(|| {
                    crate::pyo3::exceptions::PyKeyError::new_err(format!(
                        "the MJCF model has no keyframe named {name:?}"
                    ))
                });
            }
            MjcfKeyframeRef::Index(index) => *index,
        };
        let resolved = if index < 0 {
            index + keyframes.len() as isize
        } else {
            index
        };
        usize::try_from(resolved)
            .ok()
            .and_then(|i| keyframes.get(i))
            .ok_or_else(|| {
                crate::pyo3::exceptions::PyIndexError::new_err(format!(
                    "keyframe index {index} out of range (the MJCF model has {} keyframes)",
                    keyframes.len()
                ))
            })
    }
}

#[pymethods]
impl MjcfRobotHandles {
    /// The ``name`` of each ``<keyframe><key>`` of the model (``None`` for
    /// an unnamed key), in model order.
    #[getter]
    fn keyframe_names(&self) -> Vec<Option<String>> {
        self.robot
            .keyframes
            .iter()
            .map(|k| k.name.clone())
            .collect()
    }

    /// Drive the actuators of the model: one control value per actuator.
    ///
    /// Follows MuJoCo's semantics for each actuator kind: ``<motor>``
    /// applies the force ``ctrl * gear``, ``<position>`` and
    /// ``<velocity>`` configure a joint motor toward the ``ctrl``
    /// position or velocity (with the ``kp``/``kv`` gains and
    /// ``forcerange``), ``<damper>`` damps the joint, and an affine
    /// ``<general>`` becomes a position servo. Other actuators are
    /// ignored. Call it before each step whose controls changed.
    ///
    /// :param bodies: :class:`RigidBodySet` of the robot (its driven
    ///     bodies are woken up).
    /// :param joints: The joint set the robot was inserted with: the
    ///     :class:`ImpulseJointSet` (``insert_using_impulse_joints``) or
    ///     the :class:`MultibodyJointSet` (``insert_using_multibody_joints``).
    /// :param ctrl: One control value per actuator, in the order of
    ///     :attr:`actuators`.
    /// :param gain_scale: Uniform scale of the actuators' strength
    ///     (gains and force limits). Default ``1.0``.
    /// :raises ValueError: If ``len(ctrl) != len(actuators)``.
    /// :raises TypeError: If ``joints`` isn't the kind of joint set the
    ///     robot was inserted with.
    #[pyo3(signature = (bodies, joints, ctrl, gain_scale=1.0))]
    fn apply_controls(
        &self,
        bodies: &mut RigidBodySet,
        joints: &crate::pyo3::Bound<'_, crate::pyo3::PyAny>,
        ctrl: Vec<Real>,
        gain_scale: Real,
    ) -> crate::pyo3::PyResult<()> {
        if ctrl.len() != self.actuators.len() {
            return Err(crate::pyo3::exceptions::PyValueError::new_err(format!(
                "expected {} control values (one per actuator), got {}",
                self.actuators.len(),
                ctrl.len()
            )));
        }
        match &self.inserted {
            MjcfInsertedHandles::Impulse(handles) => {
                let mut joints = joints
                    .extract::<crate::pyo3::PyRefMut<'_, ImpulseJointSet>>()
                    .map_err(|_| {
                        crate::pyo3::exceptions::PyTypeError::new_err(
                            "the robot was inserted with impulse joints: pass its ImpulseJointSet",
                        )
                    })?;
                handles.apply_controls_scaled(&mut joints.0, &ctrl, gain_scale);
            }
            MjcfInsertedHandles::Multibody(handles) => {
                let mut joints = joints
                    .extract::<crate::pyo3::PyRefMut<'_, MultibodyJointSet>>()
                    .map_err(|_| {
                        crate::pyo3::exceptions::PyTypeError::new_err(
                            "the robot was inserted with multibody joints: pass its MultibodyJointSet",
                        )
                    })?;
                handles.apply_controls_multibody_scaled(
                    &mut bodies.0,
                    &mut joints.0,
                    &ctrl,
                    gain_scale,
                );
            }
        }
        Ok(())
    }

    /// Reset the robot to one of the model's keyframes.
    ///
    /// Applies the keyframe's joint positions (``qpos``) and velocities
    /// (``qvel``), and the poses of its mocap bodies. With impulse joints
    /// the body poses are set by forward kinematics and only the velocity
    /// of a floating base is applied. The actuators are left unchanged:
    /// drive them with :meth:`keyframe_controls` to hold the pose.
    ///
    /// :param bodies: :class:`RigidBodySet` of the robot.
    /// :param multibody_joints: The :class:`MultibodyJointSet` the robot
    ///     was inserted with (required for a robot inserted with
    ///     ``insert_using_multibody_joints``, ignored otherwise).
    /// :param keyframe: The keyframe's index (negative indices count from
    ///     the end) or name. Default ``0``.
    /// :raises IndexError: If the index is out of range.
    /// :raises KeyError: If no keyframe has this name.
    /// :raises TypeError: If ``multibody_joints`` is missing for a robot
    ///     inserted with multibody joints.
    #[pyo3(signature = (bodies, multibody_joints=None, keyframe=MjcfKeyframeRef::Index(0)))]
    fn apply_keyframe(
        &self,
        bodies: &mut RigidBodySet,
        multibody_joints: Option<&mut MultibodyJointSet>,
        keyframe: MjcfKeyframeRef,
    ) -> crate::pyo3::PyResult<()> {
        let key = self.keyframe(&keyframe)?;
        match &self.inserted {
            MjcfInsertedHandles::Impulse(handles) => {
                handles.apply_keyframe(&mut bodies.0, &self.robot, key);
            }
            MjcfInsertedHandles::Multibody(handles) => {
                let multibody_joints = multibody_joints.ok_or_else(|| {
                    crate::pyo3::exceptions::PyTypeError::new_err(
                        "the robot was inserted with multibody joints: pass its MultibodyJointSet",
                    )
                })?;
                handles.apply_keyframe(&mut bodies.0, &mut multibody_joints.0, &self.robot, key);
            }
        }
        Ok(())
    }

    /// The control values holding one of the model's keyframes: one per
    /// actuator, to pass to :meth:`apply_controls`.
    ///
    /// Each value is the keyframe's ``ctrl`` entry if it has one,
    /// otherwise the keyframe position of the hinge or slide joint the
    /// actuator drives, otherwise ``0``.
    ///
    /// :param keyframe: The keyframe's index (negative indices count from
    ///     the end) or name.
    /// :raises IndexError: If the index is out of range.
    /// :raises KeyError: If no keyframe has this name.
    fn keyframe_controls(&self, keyframe: MjcfKeyframeRef) -> crate::pyo3::PyResult<Vec<Real>> {
        Ok(self.robot.keyframe_controls(self.keyframe(&keyframe)?))
    }

    /// The physics hooks applying the model's ``<contact>`` rules:
    /// ``<exclude>`` (no contacts between two bodies) and ``<pair>``
    /// (friction of a pair of geoms).
    ///
    /// Assign the result to :attr:`PhysicsWorld.physics_hooks` (or pass
    /// it to :meth:`PhysicsPipeline.step`).
    fn contact_hooks(&self) -> MjcfContactHooks {
        let hooks = match &self.inserted {
            MjcfInsertedHandles::Impulse(handles) => handles.contact_hooks(&self.robot),
            MjcfInsertedHandles::Multibody(handles) => handles.contact_hooks(&self.robot),
        };
        MjcfContactHooks(std::sync::Arc::new(hooks))
    }

    /// Return the ``MjcfRobotHandles(...)`` repr.
    fn __repr__(&self) -> String {
        format!(
            "MjcfRobotHandles(n_bodies={}, n_joints={}, n_equality_joints={}, n_actuators={}, n_keyframes={})",
            self.bodies.len(),
            self.joints.len(),
            self.equality_joints.len(),
            self.actuators.len(),
            self.robot.keyframes.len(),
        )
    }
}

/// Physics hooks applying the ``<contact>`` rules of an MJCF model:
/// ``<exclude>`` removes the contacts between the colliders of two
/// bodies and ``<pair>`` overrides the friction of a pair of geoms.
///
/// Returned by :meth:`MjcfRobotHandles.contact_hooks`. Assign it to
/// :attr:`PhysicsWorld.physics_hooks`: the rules are then applied
/// natively, without calling back into Python. The colliders created by
/// the loader already enable the hooks
/// (``ActiveHooks.FILTER_CONTACT_PAIRS | ActiveHooks.MODIFY_SOLVER_CONTACTS``).
/// Its methods can also be called by your own hooks object, to combine
/// these rules with others.
#[pyclass(name = "MjcfContactHooks", module = "rapier", frozen)]
#[derive(Clone)]
pub struct MjcfContactHooks(pub std::sync::Arc<rapier3d_mjcf::MjcfContactHooks>);

/// Native `PhysicsHooks` adapter sharing the hooks of a `MjcfContactHooks`.
pub struct SharedMjcfContactHooks(std::sync::Arc<rapier3d_mjcf::MjcfContactHooks>);

impl rapier::pipeline::PhysicsHooks for SharedMjcfContactHooks {
    fn filter_contact_pair(
        &self,
        context: &rapier::pipeline::PairFilterContext,
    ) -> Option<rapier::geometry::SolverFlags> {
        self.0.filter_contact_pair(context)
    }

    fn filter_intersection_pair(&self, context: &rapier::pipeline::PairFilterContext) -> bool {
        self.0.filter_intersection_pair(context)
    }

    fn modify_solver_contacts(&self, context: &mut rapier::pipeline::ContactModificationContext) {
        self.0.modify_solver_contacts(context)
    }
}

impl MjcfContactHooks {
    pub(crate) fn native(&self) -> SharedMjcfContactHooks {
        SharedMjcfContactHooks(self.0.clone())
    }
}

#[pymethods]
impl MjcfContactHooks {
    /// ``None`` (no contacts) if the pair of colliders is excluded by an
    /// ``<exclude>`` rule, ``SolverFlags.COMPUTE_RIGID_IMPULSES`` otherwise.
    fn filter_contact_pair(&self, ctx: &PairFilterContext) -> Option<SolverFlags> {
        let (collider1, collider2) = ctx.collider_handles();
        if self.0.is_excluded(collider1, collider2) {
            None
        } else {
            Some(SolverFlags(
                rapier::geometry::SolverFlags::COMPUTE_RIGID_IMPULSES,
            ))
        }
    }

    /// Apply the friction of the ``<pair>`` rule matching the pair of
    /// colliders, if any.
    ///
    /// :raises RuntimeError: If ``ctx`` is used outside of its callback.
    fn modify_solver_contacts(
        &self,
        mut ctx: crate::pyo3::PyRefMut<'_, ContactModificationContext>,
    ) -> crate::pyo3::PyResult<()> {
        let (collider1, collider2) = ctx.collider_handles();
        if let Some(friction) = self
            .0
            .pair_override(collider1, collider2)
            .and_then(|o| o.friction)
        {
            ctx.set_friction(friction)?;
        }
        Ok(())
    }

    /// Return the ``MjcfContactHooks(...)`` repr.
    fn __repr__(&self) -> String {
        let flag = |b: bool| if b { "True" } else { "False" };
        format!(
            "MjcfContactHooks(has_excludes={}, has_overrides={})",
            flag(self.0.has_excludes()),
            flag(self.0.has_overrides())
        )
    }
}

/// A MuJoCo MJCF model, ready to be inserted into the simulation.
///
/// Build one via :meth:`from_file` / :meth:`from_str`, then commit it
/// to the world via :meth:`insert_using_impulse_joints` or
/// :meth:`insert_using_multibody_joints`. The insertion methods
/// *consume* the robot — subsequent calls raise :class:`MjcfError`.
#[pyclass(name = "MjcfRobot", module = "rapier", unsendable)]
pub struct MjcfRobot {
    pub inner: Option<rapier3d_mjcf::MjcfRobot>,
}

/// The MJCF naming tables, copied out of an `MjcfRobot` so they
/// survive the insertion that consumes it.
struct MjcfNames {
    body_names: Vec<Option<String>>,
    joint_names: Vec<Option<String>>,
    body_name_to_idx: std::collections::HashMap<String, usize>,
    joint_name_to_idx: std::collections::HashMap<String, usize>,
}

impl MjcfNames {
    fn collect(robot: &rapier3d_mjcf::MjcfRobot) -> Self {
        Self {
            body_names: robot.bodies.iter().map(|b| b.name.clone()).collect(),
            joint_names: robot.joints.iter().map(|j| j.name.clone()).collect(),
            body_name_to_idx: robot.body_name_to_idx.clone(),
            joint_name_to_idx: robot.joint_name_to_idx.clone(),
        }
    }
}

/// Convert the per-actuator handles of an insertion, mapping each
/// driven joint handle to Python with `conv`.
fn actuator_handles<H>(
    py: crate::pyo3::Python<'_>,
    actuators: Vec<rapier3d_mjcf::MjcfActuatorHandle<H>>,
    conv: impl Fn(H) -> crate::pyo3::PyObject,
) -> crate::pyo3::PyResult<Vec<crate::pyo3::Py<MjcfActuatorHandle>>> {
    actuators
        .into_iter()
        .map(|ah| {
            let a = ah.actuator;
            crate::pyo3::Py::new(
                py,
                MjcfActuatorHandle {
                    name: a.name,
                    kind: format!("{:?}", a.kind),
                    joint_name: a.joint,
                    joint: match ah.joint {
                        Some(h) => conv(h),
                        None => py.None(),
                    },
                    gear: a.gear.to_vec(),
                    ctrl_range: a.ctrl_range,
                    force_range: a.force_range,
                    kp: a.kp,
                    kv: a.kv,
                },
            )
        })
        .collect()
}

impl MjcfRobot {
    /// The still-unconsumed robot, or the "already consumed" error.
    fn robot(&self) -> crate::pyo3::PyResult<&rapier3d_mjcf::MjcfRobot> {
        self.inner
            .as_ref()
            .ok_or_else(|| crate::errors::MjcfError::new_err("MjcfRobot was already consumed"))
    }
}

#[pymethods]
impl MjcfRobot {
    /// Parse an MJCF file and return ``(MjcfRobot, MjcfModel)``.
    ///
    /// ``<include>`` directives and mesh references resolve relative
    /// to the file's parent directory.
    ///
    /// :param path: Path to the ``.xml`` file on disk.
    /// :param options: Optional :class:`MjcfLoaderOptions`.
    /// :returns: ``(MjcfRobot, MjcfModel)`` tuple.
    /// :raises MjcfError: if the file can't be opened or parsed.
    #[staticmethod]
    #[pyo3(signature = (path, options=None))]
    fn from_file(
        path: &str,
        options: Option<MjcfLoaderOptions>,
    ) -> crate::pyo3::PyResult<(MjcfRobot, MjcfModel)> {
        let opts = options.map(|o| o.to_rapier()).unwrap_or_default();
        let (robot, model) = rapier3d_mjcf::MjcfRobot::from_file(path, opts)
            .map_err(|e| crate::errors::MjcfError::new_err(format!("{e}")))?;
        Ok((MjcfRobot { inner: Some(robot) }, MjcfModel { raw: model }))
    }

    /// Parse an MJCF XML string and return ``(MjcfRobot, MjcfModel)``.
    ///
    /// ``<include>`` directives resolve relative to ``base_dir``
    /// (default: the current working directory).
    ///
    /// :param xml: Full MJCF XML document.
    /// :param options: Optional :class:`MjcfLoaderOptions`.
    /// :param base_dir: Directory used to resolve includes / mesh refs.
    /// :returns: ``(MjcfRobot, MjcfModel)`` tuple.
    /// :raises MjcfError: if the XML can't be parsed.
    #[staticmethod]
    #[pyo3(signature = (xml, options=None, base_dir=None))]
    fn from_str(
        xml: &str,
        options: Option<MjcfLoaderOptions>,
        base_dir: Option<&str>,
    ) -> crate::pyo3::PyResult<(MjcfRobot, MjcfModel)> {
        let opts = options.map(|o| o.to_rapier()).unwrap_or_default();
        let base_dir = base_dir.unwrap_or(".");
        let (robot, model) = rapier3d_mjcf::MjcfRobot::from_str(xml, opts, base_dir)
            .map_err(|e| crate::errors::MjcfError::new_err(format!("{e}")))?;
        Ok((MjcfRobot { inner: Some(robot) }, MjcfModel { raw: model }))
    }

    /// MJCF name of every body, in model order.
    ///
    /// Entry ``i`` lines up with ``MjcfRobotHandles.bodies[i]``. It is
    /// ``None`` for bodies the loader synthesized (intermediate links
    /// of a multi-joint ``<body>``).
    ///
    /// :raises MjcfError: if this robot has already been consumed.
    #[getter]
    fn body_names(&self) -> crate::pyo3::PyResult<Vec<Option<String>>> {
        Ok(self
            .robot()?
            .bodies
            .iter()
            .map(|b| b.name.clone())
            .collect())
    }
    /// MJCF name of every joint, in model order.
    ///
    /// Entry ``i`` lines up with ``MjcfRobotHandles.joints[i]``. It is
    /// ``None`` for the fixed joints the loader synthesizes to attach
    /// jointless bodies to their parent.
    ///
    /// :raises MjcfError: if this robot has already been consumed.
    #[getter]
    fn joint_names(&self) -> crate::pyo3::PyResult<Vec<Option<String>>> {
        Ok(self
            .robot()?
            .joints
            .iter()
            .map(|j| j.name.clone())
            .collect())
    }
    /// Map from MJCF body name to its index in :py:attr:`body_names`.
    ///
    /// :raises MjcfError: if this robot has already been consumed.
    #[getter]
    fn body_name_to_idx(&self) -> crate::pyo3::PyResult<std::collections::HashMap<String, usize>> {
        Ok(self.robot()?.body_name_to_idx.clone())
    }
    /// Map from MJCF joint name to its index in
    /// :py:attr:`joint_names`.
    ///
    /// :raises MjcfError: if this robot has already been consumed.
    #[getter]
    fn joint_name_to_idx(&self) -> crate::pyo3::PyResult<std::collections::HashMap<String, usize>> {
        Ok(self.robot()?.joint_name_to_idx.clone())
    }

    /// Prepend ``transform`` to the robot's root poses.
    ///
    /// Repositions the whole robot before insertion (e.g. to place a
    /// second copy beside the first). Must be called before the robot
    /// is consumed by an ``insert_*`` call.
    ///
    /// :param transform: world-space :class:`Isometry3` to apply.
    /// :raises MjcfError: if this robot has already been consumed.
    fn append_transform(&mut self, transform: Isometry3) -> crate::pyo3::PyResult<()> {
        let robot = self
            .inner
            .as_mut()
            .ok_or_else(|| crate::errors::MjcfError::new_err("MjcfRobot was already consumed"))?;
        let pose: rapier::math::Pose = transform.0.into();
        robot.append_transform(&pose);
        Ok(())
    }

    /// The gravity declared by the model (``<option gravity>``), in the
    /// frame of the model file.
    ///
    /// It isn't applied automatically: set :attr:`PhysicsWorld.gravity`
    /// (rotated like :attr:`MjcfLoaderOptions.shift` if the model was
    /// rotated).
    ///
    /// :raises MjcfError: if this robot has already been consumed.
    #[getter]
    fn gravity(&self) -> crate::pyo3::PyResult<Vec3> {
        let robot = self
            .inner
            .as_ref()
            .ok_or_else(|| crate::errors::MjcfError::new_err("MjcfRobot was already consumed"))?;
        let g: crate::na::SVector<Real, 3> = robot.gravity.into();
        Ok(Vec3(g))
    }

    /// The ``name`` of each ``<keyframe><key>`` of the model (``None`` for
    /// an unnamed key), in model order.
    ///
    /// :raises MjcfError: if this robot has already been consumed.
    #[getter]
    fn keyframe_names(&self) -> crate::pyo3::PyResult<Vec<Option<String>>> {
        let robot = self
            .inner
            .as_ref()
            .ok_or_else(|| crate::errors::MjcfError::new_err("MjcfRobot was already consumed"))?;
        Ok(robot.keyframes.iter().map(|k| k.name.clone()).collect())
    }

    /// Insert the model into the world using *impulse joints*.
    ///
    /// This call **consumes** the :class:`MjcfRobot`.
    ///
    /// :param bodies: :class:`RigidBodySet` to insert into.
    /// :param colliders: :class:`ColliderSet` to insert into.
    /// :param impulse_joints: :class:`ImpulseJointSet` to insert into.
    /// :returns: :class:`MjcfRobotHandles` with all created handles.
    /// :raises MjcfError: if this robot has already been consumed.
    fn insert_using_impulse_joints(
        &mut self,
        py: crate::pyo3::Python<'_>,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
    ) -> crate::pyo3::PyResult<crate::pyo3::Py<MjcfRobotHandles>> {
        use crate::pyo3::IntoPy;
        let robot = self
            .inner
            .take()
            .ok_or_else(|| crate::errors::MjcfError::new_err("MjcfRobot was already consumed"))?;
        let metadata = MjcfRobotHandles::robot_metadata(&robot);
        let names = MjcfNames::collect(&robot);
        let handles = robot.insert_using_impulse_joints(
            &mut bodies.0,
            &mut colliders.0,
            &mut impulse_joints.0,
        );
        {
            let inserted = handles.clone();
            let conv = |h| ImpulseJointHandle(h).into_py(py);
            let actuators = actuator_handles(py, handles.actuators, &conv)?;
            let bodies: Vec<Option<MjcfBodyHandle>> = handles
                .bodies
                .into_iter()
                .map(|b| {
                    b.map(|b| {
                        let cols = b
                            .colliders
                            .into_iter()
                            .map(|c| MjcfColliderHandle {
                                handle: ColliderHandle(c.handle),
                            })
                            .collect();
                        MjcfBodyHandle {
                            body: RigidBodyHandle(b.body),
                            colliders: cols,
                        }
                    })
                })
                .collect();
            let joints: Vec<crate::pyo3::Py<MjcfJointHandle>> = handles
                .joints
                .into_iter()
                .map(|jh| {
                    crate::pyo3::Py::new(
                        py,
                        MjcfJointHandle {
                            joint: conv(jh.joint),
                            link1: RigidBodyHandle(jh.link1),
                            link2: RigidBodyHandle(jh.link2),
                        },
                    )
                    .expect("alloc MjcfJointHandle")
                })
                .collect();
            let equality_joints: Vec<crate::pyo3::Py<MjcfJointHandle>> = handles
                .equality_joints
                .into_iter()
                .map(|jh| {
                    use crate::pyo3::IntoPy;
                    crate::pyo3::Py::new(
                        py,
                        MjcfJointHandle {
                            joint: ImpulseJointHandle(jh.joint).into_py(py),
                            link1: RigidBodyHandle(jh.link1),
                            link2: RigidBodyHandle(jh.link2),
                        },
                    )
                    .expect("alloc MjcfJointHandle")
                })
                .collect();
            crate::pyo3::Py::new(
                py,
                MjcfRobotHandles {
                    bodies,
                    joints,
                    equality_joints,
                    actuators,
                    robot: metadata,
                    inserted: MjcfInsertedHandles::Impulse(inserted),
                    body_names: names.body_names,
                    joint_names: names.joint_names,
                    body_name_to_idx: names.body_name_to_idx,
                    joint_name_to_idx: names.joint_name_to_idx,
                },
            )
        }
    }

    /// Insert the model into the world using *multibody joints*.
    ///
    /// ``<equality>`` loop-closure constraints are still inserted as
    /// impulse joints (rapier multibodies are tree-structured), so an
    /// :class:`ImpulseJointSet` is required as well.
    ///
    /// This call **consumes** the :class:`MjcfRobot`.
    ///
    /// :param bodies: :class:`RigidBodySet` to insert into.
    /// :param colliders: :class:`ColliderSet` to insert into.
    /// :param multibody_joints: :class:`MultibodyJointSet` to insert into.
    /// :param impulse_joints: :class:`ImpulseJointSet` for equalities.
    /// :param options: Optional :class:`MjcfMultibodyOptions` flags.
    /// :returns: :class:`MjcfRobotHandles` with all created handles.
    /// :raises MjcfError: if this robot has already been consumed.
    #[pyo3(signature = (bodies, colliders, multibody_joints, impulse_joints, options=None))]
    fn insert_using_multibody_joints(
        &mut self,
        py: crate::pyo3::Python<'_>,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        multibody_joints: &mut MultibodyJointSet,
        impulse_joints: &mut ImpulseJointSet,
        options: Option<MjcfMultibodyOptions>,
    ) -> crate::pyo3::PyResult<crate::pyo3::Py<MjcfRobotHandles>> {
        use crate::pyo3::IntoPy;
        let robot = self
            .inner
            .take()
            .ok_or_else(|| crate::errors::MjcfError::new_err("MjcfRobot was already consumed"))?;
        let names = MjcfNames::collect(&robot);
        let opts = options.map(|o| o.0).unwrap_or_default();
        let metadata = MjcfRobotHandles::robot_metadata(&robot);
        let handles = robot.insert_using_multibody_joints(
            &mut bodies.0,
            &mut colliders.0,
            &mut multibody_joints.0,
            &mut impulse_joints.0,
            opts,
        );
        {
            let inserted = handles.clone();
            let conv = |h: Option<_>| h.map(MultibodyJointHandle).into_py(py);
            let actuators = actuator_handles(py, handles.actuators, &conv)?;
            let bodies: Vec<Option<MjcfBodyHandle>> = handles
                .bodies
                .into_iter()
                .map(|b| {
                    b.map(|b| {
                        let cols = b
                            .colliders
                            .into_iter()
                            .map(|c| MjcfColliderHandle {
                                handle: ColliderHandle(c.handle),
                            })
                            .collect();
                        MjcfBodyHandle {
                            body: RigidBodyHandle(b.body),
                            colliders: cols,
                        }
                    })
                })
                .collect();
            let joints: Vec<crate::pyo3::Py<MjcfJointHandle>> = handles
                .joints
                .into_iter()
                .map(|jh| {
                    crate::pyo3::Py::new(
                        py,
                        MjcfJointHandle {
                            joint: conv(jh.joint),
                            link1: RigidBodyHandle(jh.link1),
                            link2: RigidBodyHandle(jh.link2),
                        },
                    )
                    .expect("alloc MjcfJointHandle")
                })
                .collect();
            let equality_joints: Vec<crate::pyo3::Py<MjcfJointHandle>> = handles
                .equality_joints
                .into_iter()
                .map(|jh| {
                    use crate::pyo3::IntoPy;
                    crate::pyo3::Py::new(
                        py,
                        MjcfJointHandle {
                            joint: ImpulseJointHandle(jh.joint).into_py(py),
                            link1: RigidBodyHandle(jh.link1),
                            link2: RigidBodyHandle(jh.link2),
                        },
                    )
                    .expect("alloc MjcfJointHandle")
                })
                .collect();
            crate::pyo3::Py::new(
                py,
                MjcfRobotHandles {
                    bodies,
                    joints,
                    equality_joints,
                    actuators,
                    robot: metadata,
                    inserted: MjcfInsertedHandles::Multibody(inserted),
                    body_names: names.body_names,
                    joint_names: names.joint_names,
                    body_name_to_idx: names.body_name_to_idx,
                    joint_name_to_idx: names.joint_name_to_idx,
                },
            )
        }
    }

    /// Return the ``MjcfRobot(...)`` repr (or ``"MjcfRobot(consumed)"``).
    fn __repr__(&self) -> String {
        match &self.inner {
            Some(r) => format!(
                "MjcfRobot(n_bodies={}, n_joints={})",
                r.bodies.len(),
                r.joints.len()
            ),
            None => "MjcfRobot(consumed)".to_string(),
        }
    }
}

/// Register the MJCF loader `#[pyclass]`-es into `m`.
pub fn register_loaders_mjcf(
    m: &crate::pyo3::Bound<'_, crate::pyo3::types::PyModule>,
) -> crate::pyo3::PyResult<()> {
    use crate::pyo3::prelude::*;
    m.add_class::<MjcfMultibodyOptions>()?;
    m.add_class::<ContactFilterMode>()?;
    m.add_class::<MjcfLoaderOptions>()?;
    m.add_class::<MjcfModel>()?;
    m.add_class::<MjcfColliderHandle>()?;
    m.add_class::<MjcfBodyHandle>()?;
    m.add_class::<MjcfJointHandle>()?;
    m.add_class::<MjcfActuatorHandle>()?;
    m.add_class::<MjcfRobotHandles>()?;
    m.add_class::<MjcfContactHooks>()?;
    m.add_class::<MjcfRobot>()?;
    Ok(())
}

// ==============================================================
// Registration
// ==============================================================

pub fn register_loaders(
    _py: crate::pyo3::Python<'_>,
    m: &crate::pyo3::Bound<'_, crate::pyo3::types::PyModule>,
) -> crate::pyo3::PyResult<()> {
    use crate::pyo3::prelude::*;
    // Mesh loader
    m.add_class::<LoadedShape>()?;
    m.add_function(crate::pyo3::wrap_pyfunction!(
        loaders_mesh_load_from_path,
        m
    )?)?;
    m.add_function(crate::pyo3::wrap_pyfunction!(
        loaders_mesh_load_from_raw_mesh,
        m
    )?)?;
    // URDF + MJCF loaders.
    register_loaders_urdf(m)?;
    register_loaders_mjcf(m)?;
    Ok(())
}
