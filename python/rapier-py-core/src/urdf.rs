//! URDF loader `#[pyclass]`-es for the Python bindings (3D, f32 only).
//!
//! The class definitions are emitted by the `__define_loaders_urdf!` macro,
//! invoked from `__define_loaders_3d!` in [`crate::loaders`]. Keeping them in
//! their own file mirrors the upstream `rapier3d-urdf` crate boundary; see
//! [`crate::loaders`] for the shared loader plumbing (mesh loader, dimension
//! gating, and the top-level `register_loaders`).

/// Materialize the URDF loader `#[pyclass]` types and `register_loaders_urdf`.
#[doc(hidden)]
#[macro_export]
macro_rules! __define_loaders_urdf {
    () => {
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
                Self(rapier3d_urdf::UrdfMultibodyOptions::from_bits_truncate(bits))
            }
            /// Return the empty flag set (no flags).
            #[staticmethod]
            fn empty() -> Self { Self(rapier3d_urdf::UrdfMultibodyOptions::empty()) }
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
            fn bits(&self) -> u8 { self.0.bits() }
            /// Bitwise OR: union of two flag sets.
            fn __or__(&self, other: &UrdfMultibodyOptions) -> Self { Self(self.0 | other.0) }
            /// Bitwise AND: intersection of two flag sets.
            fn __and__(&self, other: &UrdfMultibodyOptions) -> Self { Self(self.0 & other.0) }
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
        ///     e.g. ``MeshConverter.Obb()`` to get cheap proxy shapes while
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
            #[pyo3(get, set)] pub create_colliders_from_collision_shapes: bool,
            #[pyo3(get, set)] pub create_colliders_from_visual_shapes: bool,
            #[pyo3(get, set)] pub apply_imported_mass_props: bool,
            #[pyo3(get, set)] pub enable_joint_collisions: bool,
            #[pyo3(get, set)] pub make_roots_fixed: bool,
            #[pyo3(get, set)] pub trimesh_flags: TriMeshFlags,
            #[pyo3(get, set)] pub mesh_converter: Option<MeshConverter>,
            #[pyo3(get, set)] pub shift: Isometry3,
            #[pyo3(get, set)] pub scale: Real,
            #[pyo3(get, set)] pub squeeze_empty_fixed_links: bool,
            /// Template collider applied to every imported collider before
            /// its shape/material are filled in. ``None`` keeps the rapier
            /// default (density 0).
            #[pyo3(get, set)] pub collider_blueprint: Option<ColliderBuilder>,
            /// Template rigid-body used for every imported link. ``None``
            /// keeps the rapier default (a dynamic body).
            #[pyo3(get, set)] pub rigid_body_blueprint: Option<RigidBodyBuilder>,
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
                let identity_iso: $crate::na::Isometry<Real, _, 3> = rapier::math::Pose::IDENTITY.into();
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
            fn to_rapier(&self) -> rapier3d_urdf::UrdfLoaderOptions {
                let mut o = rapier3d_urdf::UrdfLoaderOptions::default();
                o.create_colliders_from_collision_shapes = self.create_colliders_from_collision_shapes;
                o.create_colliders_from_visual_shapes = self.create_colliders_from_visual_shapes;
                o.apply_imported_mass_props = self.apply_imported_mass_props;
                o.enable_joint_collisions = self.enable_joint_collisions;
                o.make_roots_fixed = self.make_roots_fixed;
                o.trimesh_flags = self.trimesh_flags.0;
                o.mesh_converter = self.mesh_converter.as_ref().map(|m| m.0.clone());
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
            fn name(&self) -> String { self.raw.name.clone() }
            /// Number of ``<visual>`` elements on the link.
            #[getter]
            fn n_visuals(&self) -> usize { self.raw.visual.len() }
            /// Number of ``<collision>`` elements on the link.
            #[getter]
            fn n_collisions(&self) -> usize { self.raw.collision.len() }
            /// Mass from the link's ``<inertial>`` block.
            #[getter]
            fn mass(&self) -> f64 { self.raw.inertial.mass.value }
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
            fn name(&self) -> String { self.raw.name.clone() }
            /// Joint kind as a debug string (``"Revolute"``, ``"Fixed"``, ...).
            #[getter]
            fn joint_type(&self) -> String { format!("{:?}", self.raw.joint_type) }
            /// Name of the parent link.
            #[getter]
            fn parent(&self) -> String { self.raw.parent.link.clone() }
            /// Name of the child link.
            #[getter]
            fn child(&self) -> String { self.raw.child.link.clone() }
            /// Axis vector ``[x, y, z]`` in the parent link's frame.
            #[getter]
            fn axis(&self) -> Vec<f64> { self.raw.axis.xyz.0.to_vec() }
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
            fn name(&self) -> String { self.raw.name.clone() }
            /// List of :class:`UrdfLink` views, in document order.
            #[getter]
            fn links(&self) -> Vec<UrdfLink> {
                self.raw.links.iter().map(|l| UrdfLink { raw: l.clone() }).collect()
            }
            /// List of :class:`UrdfJoint` views, in document order.
            #[getter]
            fn joints(&self) -> Vec<UrdfJoint> {
                self.raw.joints.iter().map(|j| UrdfJoint { raw: j.clone() }).collect()
            }
            /// Return the ``Robot(...)`` repr.
            fn __repr__(&self) -> String {
                format!(
                    "Robot(name={:?}, n_links={}, n_joints={})",
                    self.raw.name, self.raw.links.len(), self.raw.joints.len(),
                )
            }
        }

        // ----- URDF handles ------------------------------------------------

        /// Visual mesh override paired with a URDF collider.
        ///
        /// Populated by the loader only when a non-default
        /// :attr:`UrdfLoaderOptions.mesh_converter` (e.g.
        /// ``MeshConverter.Obb()``) replaced the source mesh with a cheap
        /// proxy collider — this keeps the original high-resolution mesh
        /// available for rendering.
        ///
        /// :ivar shape: The :class:`SharedShape` representing the visual mesh.
        /// :ivar local_pose: Pose of the visual mesh in the collider's local
        ///     frame, as an :class:`Isometry3`.
        #[pyclass(name = "UrdfVisual", module = "rapier")]
        #[derive(Clone)]
        pub struct UrdfVisual {
            #[pyo3(get)] pub shape: SharedShape,
            #[pyo3(get)] pub local_pose: Isometry3,
        }

        /// Handle of one collider inserted from a URDF link.
        ///
        /// :ivar handle: Underlying :class:`ColliderHandle`.
        /// :ivar visual: Optional :class:`UrdfVisual` mesh override (``None``
        ///     unless a proxy :attr:`UrdfLoaderOptions.mesh_converter` was used).
        #[pyclass(name = "UrdfColliderHandle", module = "rapier", frozen)]
        #[derive(Clone, Debug)]
        pub struct UrdfColliderHandle {
            #[pyo3(get)] pub handle: ColliderHandle,
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
        }

        /// Handle of one URDF link (its rigid body plus its colliders).
        ///
        /// :ivar body: :class:`RigidBodyHandle` for the link's body.
        /// :ivar colliders: List of :class:`UrdfColliderHandle` attached to it.
        #[pyclass(name = "UrdfLinkHandle", module = "rapier", frozen)]
        #[derive(Clone, Debug)]
        pub struct UrdfLinkHandle {
            #[pyo3(get)] pub body: RigidBodyHandle,
            #[pyo3(get)] pub colliders: Vec<UrdfColliderHandle>,
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
            #[pyo3(get)] pub joint: $crate::pyo3::PyObject,
            #[pyo3(get)] pub link1: RigidBodyHandle,
            #[pyo3(get)] pub link2: RigidBodyHandle,
        }

        /// Aggregate handle set returned by URDF insertion functions.
        ///
        /// :ivar links: One :class:`UrdfLinkHandle` per URDF link.
        /// :ivar joints: One :class:`UrdfJointHandle` per URDF joint.
        #[pyclass(name = "UrdfRobotHandles", module = "rapier")]
        pub struct UrdfRobotHandles {
            #[pyo3(get)] pub links: Vec<UrdfLinkHandle>,
            #[pyo3(get)] pub joints: Vec<$crate::pyo3::Py<UrdfJointHandle>>,
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
            ) -> $crate::pyo3::PyResult<(UrdfRobot, UrdfRobotSource)> {
                let opts = options
                    .map(|o| o.to_rapier())
                    .unwrap_or_else(rapier3d_urdf::UrdfLoaderOptions::default);
                let mesh_dir = mesh_dir.map(_StdPath::new);
                let (robot, raw) = rapier3d_urdf::UrdfRobot::from_file(path, opts, mesh_dir)
                    .map_err(|e| $crate::errors::UrdfError::new_err(format!("{e}")))?;
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
            ) -> $crate::pyo3::PyResult<(UrdfRobot, UrdfRobotSource)> {
                let opts = options
                    .map(|o| o.to_rapier())
                    .unwrap_or_else(rapier3d_urdf::UrdfLoaderOptions::default);
                let mesh_dir = mesh_dir
                    .map(_StdPath::new)
                    .unwrap_or_else(|| _StdPath::new("."));
                let (robot, raw) = rapier3d_urdf::UrdfRobot::from_str(xml, opts, mesh_dir)
                    .map_err(|e| $crate::errors::UrdfError::new_err(format!("{e}")))?;
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
            ) -> $crate::pyo3::PyResult<UrdfRobot> {
                let opts = options
                    .map(|o| o.to_rapier())
                    .unwrap_or_else(rapier3d_urdf::UrdfLoaderOptions::default);
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
            fn n_links(&self) -> $crate::pyo3::PyResult<usize> {
                self.inner.as_ref()
                    .map(|r| r.links.len())
                    .ok_or_else(|| $crate::errors::UrdfError::new_err("UrdfRobot was already consumed"))
            }

            /// Number of joints waiting to be inserted.
            ///
            /// :raises UrdfError: if this robot has already been consumed.
            #[getter]
            fn n_joints(&self) -> $crate::pyo3::PyResult<usize> {
                self.inner.as_ref()
                    .map(|r| r.joints.len())
                    .ok_or_else(|| $crate::errors::UrdfError::new_err("UrdfRobot was already consumed"))
            }

            /// Apply a rigid transform to every body of the robot.
            ///
            /// Mutates the robot in place. World-anchored joints are kept
            /// satisfied across the transform.
            ///
            /// :param iso: World-space :class:`Isometry3` to prepend to
            ///     every body pose.
            /// :raises UrdfError: if this robot has already been consumed.
            fn append_transform(&mut self, iso: Isometry3) -> $crate::pyo3::PyResult<()> {
                let robot = self.inner.as_mut().ok_or_else(|| {
                    $crate::errors::UrdfError::new_err("UrdfRobot was already consumed")
                })?;
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
                py: $crate::pyo3::Python<'_>,
                bodies: &mut RigidBodySet,
                colliders: &mut ColliderSet,
                joints: &mut ImpulseJointSet,
            ) -> $crate::pyo3::PyResult<$crate::pyo3::Py<UrdfRobotHandles>> {
                use $crate::pyo3::IntoPy;
                let robot = self.inner.take().ok_or_else(|| {
                    $crate::errors::UrdfError::new_err("UrdfRobot was already consumed")
                })?;
                let handles = robot.insert_using_impulse_joints(
                    &mut bodies.0,
                    &mut colliders.0,
                    &mut joints.0,
                );
                let links: Vec<UrdfLinkHandle> = handles.links.into_iter().map(|lh| {
                    let cols = lh.colliders.into_iter().map(|h| UrdfColliderHandle {
                        handle: ColliderHandle(h.handle),
                        visual: h.visual,
                    }).collect();
                    UrdfLinkHandle {
                        body: RigidBodyHandle(lh.body),
                        colliders: cols,
                    }
                }).collect();
                let joints_handles: Vec<$crate::pyo3::Py<UrdfJointHandle>> = handles.joints.into_iter().map(|jh| {
                    let joint_py = ImpulseJointHandle(jh.joint).into_py(py);
                    $crate::pyo3::Py::new(py, UrdfJointHandle {
                        joint: joint_py,
                        link1: RigidBodyHandle(jh.link1),
                        link2: RigidBodyHandle(jh.link2),
                    }).expect("alloc UrdfJointHandle")
                }).collect();
                $crate::pyo3::Py::new(py, UrdfRobotHandles { links, joints: joints_handles })
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
                py: $crate::pyo3::Python<'_>,
                bodies: &mut RigidBodySet,
                colliders: &mut ColliderSet,
                multibody_joints: &mut MultibodyJointSet,
                options: Option<UrdfMultibodyOptions>,
            ) -> $crate::pyo3::PyResult<$crate::pyo3::Py<UrdfRobotHandles>> {
                use $crate::pyo3::IntoPy;
                let robot = self.inner.take().ok_or_else(|| {
                    $crate::errors::UrdfError::new_err("UrdfRobot was already consumed")
                })?;
                let opts = options.map(|o| o.0).unwrap_or_default();
                let handles = robot.insert_using_multibody_joints(
                    &mut bodies.0,
                    &mut colliders.0,
                    &mut multibody_joints.0,
                    opts,
                );
                let links: Vec<UrdfLinkHandle> = handles.links.into_iter().map(|lh| {
                    let cols = lh.colliders.into_iter().map(|h| UrdfColliderHandle {
                        handle: ColliderHandle(h.handle),
                        visual: h.visual,
                    }).collect();
                    UrdfLinkHandle {
                        body: RigidBodyHandle(lh.body),
                        colliders: cols,
                    }
                }).collect();
                let joints_handles: Vec<$crate::pyo3::Py<UrdfJointHandle>> = handles.joints.into_iter().map(|jh| {
                    let joint_py = jh.joint.map(MultibodyJointHandle).into_py(py);
                    $crate::pyo3::Py::new(py, UrdfJointHandle {
                        joint: joint_py,
                        link1: RigidBodyHandle(jh.link1),
                        link2: RigidBodyHandle(jh.link2),
                    }).expect("alloc UrdfJointHandle")
                }).collect();
                $crate::pyo3::Py::new(py, UrdfRobotHandles { links, joints: joints_handles })
            }

            /// Return the ``UrdfRobot(...)`` repr (or ``"UrdfRobot(consumed)"``
            /// after insertion).
            fn __repr__(&self) -> String {
                match &self.inner {
                    Some(r) => format!("UrdfRobot(n_links={}, n_joints={})", r.links.len(), r.joints.len()),
                    None => "UrdfRobot(consumed)".to_string(),
                }
            }
        }

        /// Register the URDF loader `#[pyclass]`-es into `m`.
        pub fn register_loaders_urdf(
            m: &$crate::pyo3::Bound<'_, $crate::pyo3::types::PyModule>,
        ) -> $crate::pyo3::PyResult<()> {
            use $crate::pyo3::prelude::*;
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
    };
}
