//! MJCF (MuJoCo XML) loader `#[pyclass]`-es for the Python bindings (3D, f32 only).
//!
//! The class definitions are emitted by the `__define_loaders_mjcf!` macro,
//! invoked from `__define_loaders_3d!` in [`crate::loaders`]. Keeping them in
//! their own file mirrors the upstream `rapier3d-mjcf` crate boundary.
//!
//! Only a common subset of `MjcfLoaderOptions` is exposed (the
//! `collider_blueprint` / `rigid_body_blueprint` and `contact_filter_mode`
//! fields are left at their defaults), and actuator / sensor / keyframe
//! bindings are not yet surfaced.

/// Materialize the MJCF loader `#[pyclass]` types and `register_loaders_mjcf`.
#[doc(hidden)]
#[macro_export]
macro_rules! __define_loaders_mjcf {
    () => {
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
                Self(rapier3d_mjcf::MjcfMultibodyOptions::from_bits_truncate(bits))
            }
            /// Return the empty flag set (no flags).
            #[staticmethod]
            fn empty() -> Self { Self(rapier3d_mjcf::MjcfMultibodyOptions::empty()) }
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
            /// Raw bits as an unsigned int.
            #[getter]
            fn bits(&self) -> u8 { self.0.bits() }
            /// Bitwise OR: union of two flag sets.
            fn __or__(&self, other: &MjcfMultibodyOptions) -> Self { Self(self.0 | other.0) }
            /// Bitwise AND: intersection of two flag sets.
            fn __and__(&self, other: &MjcfMultibodyOptions) -> Self { Self(self.0 & other.0) }
            /// Return ``MjcfMultibodyOptions(bits=0b...)`` repr.
            fn __repr__(&self) -> String {
                format!("MjcfMultibodyOptions(bits={:#08b})", self.0.bits())
            }
        }

        /// How MJCF ``contype`` / ``conaffinity`` masks map onto rapier
        /// :class:`InteractionGroups`.
        #[pyclass(name = "ContactFilterMode", module = "rapier", eq, eq_int)]
        #[derive(Clone, Copy, PartialEq, Eq)]
        pub enum ContactFilterMode {
            /// ``memberships = filter = contype | conaffinity`` (default).
            Symmetric,
            /// ``memberships = contype``, ``filter = conaffinity``.
            Asymmetric,
        }

        impl ContactFilterMode {
            fn to_rapier(self) -> rapier3d_mjcf::ContactFilterMode {
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
            #[pyo3(get, set)] pub create_colliders_from_collision_shapes: bool,
            #[pyo3(get, set)] pub create_colliders_from_visual_shapes: bool,
            #[pyo3(get, set)] pub apply_imported_mass_props: bool,
            #[pyo3(get, set)] pub enable_joint_collisions: bool,
            #[pyo3(get, set)] pub make_roots_fixed: bool,
            #[pyo3(get, set)] pub trimesh_flags: TriMeshFlags,
            #[pyo3(get, set)] pub mesh_converter: Option<MeshConverter>,
            #[pyo3(get, set)] pub shift: Isometry3,
            #[pyo3(get, set)] pub scale: Real,
            #[pyo3(get, set)] pub skip_plane_geoms: bool,
            #[pyo3(get, set)] pub disable_joint_motors: bool,
            /// Template collider applied to every imported collider. ``None``
            /// keeps the rapier default (density 0).
            #[pyo3(get, set)] pub collider_blueprint: Option<ColliderBuilder>,
            /// Template rigid-body used for every imported body. ``None``
            /// keeps the rapier default (a dynamic body).
            #[pyo3(get, set)] pub rigid_body_blueprint: Option<RigidBodyBuilder>,
            /// How ``contype``/``conaffinity`` map onto collision groups.
            #[pyo3(get, set)] pub contact_filter_mode: ContactFilterMode,
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
            fn to_rapier(&self) -> rapier3d_mjcf::MjcfLoaderOptions {
                let mut o = rapier3d_mjcf::MjcfLoaderOptions::default();
                o.create_colliders_from_collision_shapes = self.create_colliders_from_collision_shapes;
                o.create_colliders_from_visual_shapes = self.create_colliders_from_visual_shapes;
                o.apply_imported_mass_props = self.apply_imported_mass_props;
                o.enable_joint_collisions = self.enable_joint_collisions;
                o.make_roots_fixed = self.make_roots_fixed;
                o.trimesh_flags = self.trimesh_flags.0;
                o.mesh_converter = self.mesh_converter.as_ref().map(|m| m.0.clone());
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
            fn name(&self) -> Option<String> { self.raw.name.clone() }
            /// Return the ``MjcfModel(...)`` repr.
            fn __repr__(&self) -> String {
                format!("MjcfModel(name={:?})", self.raw.name)
            }
        }

        /// Handle of one collider inserted from an MJCF ``<geom>``.
        ///
        /// :ivar handle: Underlying :class:`ColliderHandle`.
        #[pyclass(name = "MjcfColliderHandle", module = "rapier", frozen)]
        #[derive(Clone, Debug)]
        pub struct MjcfColliderHandle {
            #[pyo3(get)] pub handle: ColliderHandle,
        }

        /// Handle of one MJCF body (its rigid body plus its colliders).
        ///
        /// :ivar body: :class:`RigidBodyHandle` for the body.
        /// :ivar colliders: List of :class:`MjcfColliderHandle` attached to it.
        #[pyclass(name = "MjcfBodyHandle", module = "rapier", frozen)]
        #[derive(Clone, Debug)]
        pub struct MjcfBodyHandle {
            #[pyo3(get)] pub body: RigidBodyHandle,
            #[pyo3(get)] pub colliders: Vec<MjcfColliderHandle>,
        }

        /// Handle of one MJCF joint after insertion.
        ///
        /// :ivar joint: Either an :class:`ImpulseJointHandle` (impulse-joint
        ///     path) or ``Optional[MultibodyJointHandle]`` (multibody path).
        /// :ivar link1: Parent body's :class:`RigidBodyHandle`.
        /// :ivar link2: Child body's :class:`RigidBodyHandle`.
        #[pyclass(name = "MjcfJointHandle", module = "rapier")]
        pub struct MjcfJointHandle {
            #[pyo3(get)] pub joint: $crate::pyo3::PyObject,
            #[pyo3(get)] pub link1: RigidBodyHandle,
            #[pyo3(get)] pub link2: RigidBodyHandle,
        }

        /// Aggregate handle set returned by MJCF insertion functions.
        ///
        /// :ivar bodies: One ``Optional[MjcfBodyHandle]`` per MJCF body, in
        ///     model order (entry 0 is the implicit world body and is usually
        ///     ``None``).
        /// :ivar joints: One :class:`MjcfJointHandle` per inserted joint.
        /// :ivar equality_joints: One :class:`MjcfJointHandle` per
        ///     ``<equality>`` loop-closure constraint (always impulse joints).
        #[pyclass(name = "MjcfRobotHandles", module = "rapier")]
        pub struct MjcfRobotHandles {
            #[pyo3(get)] pub bodies: Vec<Option<MjcfBodyHandle>>,
            #[pyo3(get)] pub joints: Vec<$crate::pyo3::Py<MjcfJointHandle>>,
            #[pyo3(get)] pub equality_joints: Vec<$crate::pyo3::Py<MjcfJointHandle>>,
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
            ) -> $crate::pyo3::PyResult<(MjcfRobot, MjcfModel)> {
                let opts = options
                    .map(|o| o.to_rapier())
                    .unwrap_or_else(rapier3d_mjcf::MjcfLoaderOptions::default);
                let (robot, model) = rapier3d_mjcf::MjcfRobot::from_file(path, opts)
                    .map_err(|e| $crate::errors::MjcfError::new_err(format!("{e}")))?;
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
            ) -> $crate::pyo3::PyResult<(MjcfRobot, MjcfModel)> {
                let opts = options
                    .map(|o| o.to_rapier())
                    .unwrap_or_else(rapier3d_mjcf::MjcfLoaderOptions::default);
                let base_dir = base_dir.unwrap_or(".");
                let (robot, model) = rapier3d_mjcf::MjcfRobot::from_str(xml, opts, base_dir)
                    .map_err(|e| $crate::errors::MjcfError::new_err(format!("{e}")))?;
                Ok((MjcfRobot { inner: Some(robot) }, MjcfModel { raw: model }))
            }

            /// Prepend ``transform`` to the robot's root poses.
            ///
            /// Repositions the whole robot before insertion (e.g. to place a
            /// second copy beside the first). Must be called before the robot
            /// is consumed by an ``insert_*`` call.
            ///
            /// :param transform: world-space :class:`Isometry3` to apply.
            /// :raises MjcfError: if this robot has already been consumed.
            fn append_transform(&mut self, transform: Isometry3) -> $crate::pyo3::PyResult<()> {
                let robot = self.inner.as_mut().ok_or_else(|| {
                    $crate::errors::MjcfError::new_err("MjcfRobot was already consumed")
                })?;
                let pose: rapier::math::Pose = transform.0.into();
                robot.append_transform(&pose);
                Ok(())
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
                py: $crate::pyo3::Python<'_>,
                bodies: &mut RigidBodySet,
                colliders: &mut ColliderSet,
                impulse_joints: &mut ImpulseJointSet,
            ) -> $crate::pyo3::PyResult<$crate::pyo3::Py<MjcfRobotHandles>> {
                use $crate::pyo3::IntoPy;
                let robot = self.inner.take().ok_or_else(|| {
                    $crate::errors::MjcfError::new_err("MjcfRobot was already consumed")
                })?;
                let handles = robot.insert_using_impulse_joints(
                    &mut bodies.0,
                    &mut colliders.0,
                    &mut impulse_joints.0,
                );
                $crate::__mjcf_build_handles!(py, handles, |h| ImpulseJointHandle(h).into_py(py))
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
                py: $crate::pyo3::Python<'_>,
                bodies: &mut RigidBodySet,
                colliders: &mut ColliderSet,
                multibody_joints: &mut MultibodyJointSet,
                impulse_joints: &mut ImpulseJointSet,
                options: Option<MjcfMultibodyOptions>,
            ) -> $crate::pyo3::PyResult<$crate::pyo3::Py<MjcfRobotHandles>> {
                use $crate::pyo3::IntoPy;
                let robot = self.inner.take().ok_or_else(|| {
                    $crate::errors::MjcfError::new_err("MjcfRobot was already consumed")
                })?;
                let opts = options.map(|o| o.0).unwrap_or_default();
                let handles = robot.insert_using_multibody_joints(
                    &mut bodies.0,
                    &mut colliders.0,
                    &mut multibody_joints.0,
                    &mut impulse_joints.0,
                    opts,
                );
                $crate::__mjcf_build_handles!(py, handles, |h: Option<_>| h.map(MultibodyJointHandle).into_py(py))
            }

            /// Return the ``MjcfRobot(...)`` repr (or ``"MjcfRobot(consumed)"``).
            fn __repr__(&self) -> String {
                match &self.inner {
                    Some(r) => format!("MjcfRobot(n_bodies={}, n_joints={})", r.bodies.len(), r.joints.len()),
                    None => "MjcfRobot(consumed)".to_string(),
                }
            }
        }

        /// Register the MJCF loader `#[pyclass]`-es into `m`.
        pub fn register_loaders_mjcf(
            m: &$crate::pyo3::Bound<'_, $crate::pyo3::types::PyModule>,
        ) -> $crate::pyo3::PyResult<()> {
            use $crate::pyo3::prelude::*;
            m.add_class::<MjcfMultibodyOptions>()?;
            m.add_class::<ContactFilterMode>()?;
            m.add_class::<MjcfLoaderOptions>()?;
            m.add_class::<MjcfModel>()?;
            m.add_class::<MjcfColliderHandle>()?;
            m.add_class::<MjcfBodyHandle>()?;
            m.add_class::<MjcfJointHandle>()?;
            m.add_class::<MjcfRobotHandles>()?;
            m.add_class::<MjcfRobot>()?;
            Ok(())
        }
    };
}

/// Shared body/joint-handle marshalling for both MJCF insertion paths.
///
/// `$conv` maps the path-specific joint-handle type to a `PyObject`.
#[doc(hidden)]
#[macro_export]
macro_rules! __mjcf_build_handles {
    ($py:expr, $handles:expr, $conv:expr) => {{
        let handles = $handles;
        let conv = $conv;
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
        let joints: Vec<$crate::pyo3::Py<MjcfJointHandle>> = handles
            .joints
            .into_iter()
            .map(|jh| {
                $crate::pyo3::Py::new(
                    $py,
                    MjcfJointHandle {
                        joint: conv(jh.joint),
                        link1: RigidBodyHandle(jh.link1),
                        link2: RigidBodyHandle(jh.link2),
                    },
                )
                .expect("alloc MjcfJointHandle")
            })
            .collect();
        let equality_joints: Vec<$crate::pyo3::Py<MjcfJointHandle>> = handles
            .equality_joints
            .into_iter()
            .map(|jh| {
                use $crate::pyo3::IntoPy;
                $crate::pyo3::Py::new(
                    $py,
                    MjcfJointHandle {
                        joint: ImpulseJointHandle(jh.joint).into_py($py),
                        link1: RigidBodyHandle(jh.link1),
                        link2: RigidBodyHandle(jh.link2),
                    },
                )
                .expect("alloc MjcfJointHandle")
            })
            .collect();
        $crate::pyo3::Py::new(
            $py,
            MjcfRobotHandles {
                bodies,
                joints,
                equality_joints,
            },
        )
    }};
}
