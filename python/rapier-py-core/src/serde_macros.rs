//! pickle / snapshot macro emitted per-cdylib.
//!
//! Each cdylib invokes `define_serde_types!(DIM = 2|3)` AFTER all the other
//! macros (`define_math_types!`, `define_dynamics_types!`, etc.). This macro
//! adds extra `#[pymethods]` blocks (enabled by pyo3's `multiple-pymethods`
//! feature) to attach `to_bytes`, `from_bytes`, and `__reduce__` to the
//! relevant pyclasses, plus `snapshot`, `restore`, `snapshot_json`, and
//! `restore_json` to `PhysicsWorld`.
//!
//! The implementations rely on rapier being compiled with `serde-serialize`,
//! which each `rapier-py-{2d,2d-f64,3d,3d-f64}` cdylib enables in its
//! Cargo.toml.

/// Materialize the per-cdylib serde glue.
///
/// Must be invoked **after** every other `define_*_types!` macro because it
/// adds extra `#[pymethods]` blocks on those pyclasses.
#[macro_export]
macro_rules! define_serde_types {
    (DIM = 2) => {
        $crate::__define_serde_common!(2);
    };
    (DIM = 3) => {
        $crate::__define_serde_common!(3);
    };
}

/// Internal: emit a `__reduce__` for a frozen, copy-able handle-style class.
///
/// `$Cls` must already expose a `from_raw_parts(index, generation) -> Self`
/// staticmethod and `.index()` / `.generation()` getters.
#[doc(hidden)]
#[macro_export]
macro_rules! __reduce_handle {
    ($Cls:ident) => {
        #[pymethods]
        impl $Cls {
            /// Pickle support: `(type(self), (index, generation))`.
            fn __reduce__(slf: PyRef<'_, Self>) -> PyResult<(Py<PyAny>, (u32, u32))> {
                let py = slf.py();
                let (i, g) = slf.0.into_raw_parts();
                let typ: Py<PyAny> = py
                    .get_type_bound::<$Cls>()
                    .getattr("from_raw_parts")?
                    .unbind();
                Ok((typ, (i, g)))
            }
        }
    };
}

/// Internal: emit `to_bytes` / `from_bytes` / `__reduce__` for an owned-value
/// pyclass that wraps a serde-able `rapier` value.
///
/// `$Cls` is the pyclass name (the wrapper, e.g. `RigidBodyBuilder`).
/// `$inner` is the field path on `Self` ending in the rapier inner value
///         (e.g. `body`, `builder`, `0`).
#[doc(hidden)]
#[macro_export]
macro_rules! __serde_owned_value {
    ($Cls:ident, $field:tt, $constructor:expr, $ctor_name:literal) => {
        #[pymethods]
        impl $Cls {
            /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
            /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
            fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
                let inner = &self.$field;
                let payload =
                    $crate::bincode::serialize(inner).map_err($crate::serde_io::bincode_err)?;
                Ok($crate::serde_io::bytes_to_py(
                    py,
                    &$crate::serde_io::wrap_bincode(payload),
                ))
            }

            /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
            #[staticmethod]
            fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
                let buf = blob.as_bytes();
                let body = $crate::serde_io::unwrap_bincode(buf)?;
                let inner =
                    $crate::bincode::deserialize(body).map_err($crate::serde_io::bincode_err)?;
                Ok(($constructor)(inner))
            }

            /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
            fn __reduce__<'py>(
                &self,
                py: Python<'py>,
            ) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
                let blob = self.to_bytes(py)?;
                let ctor: Py<PyAny> = py.get_type_bound::<$Cls>().getattr($ctor_name)?.unbind();
                Ok((ctor, (blob,)))
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_serde_common {
    ($dim:tt) => {
        // pyo3::types::PyBytes already in scope from earlier macros, but
        // earlier macros didn't import it on every dim-specific block. Bring
        // it in unconditionally so the templates below compile.
        use $crate::pyo3::exceptions::PyTypeError as _PickleTypeError;
        #[allow(unused_imports)]
        use $crate::pyo3::types::PyBytes;

        // ============================================================
        // Handle types — `__reduce__ = (from_raw_parts, (idx, gen))`.
        // ============================================================
        $crate::__reduce_handle!(RigidBodyHandle);
        $crate::__reduce_handle!(ColliderHandle);
        $crate::__reduce_handle!(ImpulseJointHandle);
        $crate::__reduce_handle!(MultibodyJointHandle);

        // MultibodyIndex / MultibodyLinkId have a slightly different shape
        // (no `from_raw_parts` constructor on the Python side). Provide them.
        #[pymethods]
        impl MultibodyIndex {
            #[staticmethod]
            fn from_raw_parts(index: u32, generation: u32) -> Self {
                Self(rapier::dynamics::MultibodyIndex(
                    rapier::data::arena::Index::from_raw_parts(index, generation),
                ))
            }
            fn into_raw_parts(&self) -> (u32, u32) {
                self.0.0.into_raw_parts()
            }
            fn __reduce__(slf: PyRef<'_, Self>) -> PyResult<(Py<PyAny>, (u32, u32))> {
                let py = slf.py();
                let (i, g) = slf.0.0.into_raw_parts();
                let ctor: Py<PyAny> = py
                    .get_type_bound::<MultibodyIndex>()
                    .getattr("from_raw_parts")?
                    .unbind();
                Ok((ctor, (i, g)))
            }
        }

        // `MultibodyLinkId` has a private `graph_id` field; we round-trip
        // through bincode (it's serde-derived upstream).
        #[pymethods]
        impl MultibodyLinkId {
            fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
                let payload =
                    $crate::bincode::serialize(&self.0).map_err($crate::serde_io::bincode_err)?;
                Ok($crate::serde_io::bytes_to_py(
                    py,
                    &$crate::serde_io::wrap_bincode(payload),
                ))
            }
            #[staticmethod]
            fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
                let body = $crate::serde_io::unwrap_bincode(blob.as_bytes())?;
                let inner: rapier::dynamics::MultibodyLinkId =
                    $crate::bincode::deserialize(body).map_err($crate::serde_io::bincode_err)?;
                Ok(Self(inner))
            }
            fn __reduce__<'py>(
                &self,
                py: Python<'py>,
            ) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
                let blob = self.to_bytes(py)?;
                let ctor: Py<PyAny> = py
                    .get_type_bound::<MultibodyLinkId>()
                    .getattr("from_bytes")?
                    .unbind();
                Ok((ctor, (blob,)))
            }
        }

        // ============================================================
        // RigidBody / RigidBodyBuilder / Collider / ColliderBuilder
        // ============================================================

        impl RigidBody {
            /// Used by `from_bytes` / pickle to reconstruct.
            #[inline]
            pub(crate) fn _from_inner(body: rapier::dynamics::RigidBody) -> Self {
                Self { body }
            }
        }
        $crate::__serde_owned_value!(RigidBody, body, RigidBody::_from_inner, "from_bytes");

        // RigidBodyBuilder is NOT `Serialize`/`Deserialize` upstream. To make
        // it picklable, we round-trip through its built form (a RigidBody),
        // which is serde-derived. This preserves every field that influences
        // the spawned body but discards the "builder vs body" distinction —
        // round-tripped builders behave identically when `.build()` is called.
        #[pymethods]
        impl RigidBodyBuilder {
            fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
                let body = self.builder.clone().build();
                let payload =
                    $crate::bincode::serialize(&body).map_err($crate::serde_io::bincode_err)?;
                Ok($crate::serde_io::bytes_to_py(
                    py,
                    &$crate::serde_io::wrap_bincode(payload),
                ))
            }

            #[staticmethod]
            fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
                let buf = blob.as_bytes();
                let body_buf = $crate::serde_io::unwrap_bincode(buf)?;
                let body: rapier::dynamics::RigidBody = $crate::bincode::deserialize(body_buf)
                    .map_err($crate::serde_io::bincode_err)?;
                // RigidBodyBuilder has no `.from_body` constructor — rebuild
                // a fresh builder of the correct type and copy in fields
                // we can read back. (`can_sleep` is not retrievable, and the
                // builder's sleeping/enabled bits aren't observable on a built
                // body either, so we approximate.)
                let mut b = rapier::dynamics::RigidBodyBuilder::new(body.body_type());
                b = b
                    .pose(*body.position())
                    .linvel(body.linvel())
                    .linear_damping(body.linear_damping())
                    .angular_damping(body.angular_damping())
                    .sleeping(body.is_sleeping())
                    .gravity_scale(body.gravity_scale())
                    .ccd_enabled(body.is_ccd_enabled())
                    .soft_ccd_prediction(body.soft_ccd_prediction())
                    .dominance_group(body.dominance_group())
                    .additional_solver_iterations(body.additional_solver_iterations())
                    .user_data(body.user_data)
                    .enabled(body.is_enabled())
                    .locked_axes(body.locked_axes());
                $crate::__rb_builder_angvel_from_body!($dim, b, body);
                Ok(Self { builder: b })
            }

            fn __reduce__<'py>(
                &self,
                py: Python<'py>,
            ) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
                let blob = self.to_bytes(py)?;
                let ctor: Py<PyAny> = py
                    .get_type_bound::<RigidBodyBuilder>()
                    .getattr("from_bytes")?
                    .unbind();
                Ok((ctor, (blob,)))
            }
        }

        impl Collider {
            #[inline]
            pub(crate) fn _from_inner(collider: rapier::geometry::Collider) -> Self {
                Self { collider }
            }
        }
        $crate::__serde_owned_value!(Collider, collider, Collider::_from_inner, "from_bytes");

        impl ColliderBuilder {
            #[inline]
            pub(crate) fn _from_inner(builder: rapier::geometry::ColliderBuilder) -> Self {
                Self { builder }
            }
        }
        $crate::__serde_owned_value!(
            ColliderBuilder,
            builder,
            ColliderBuilder::_from_inner,
            "from_bytes"
        );

        // ============================================================
        // Sets (newtype `Self(inner)`).
        // ============================================================
        $crate::__serde_owned_value!(RigidBodySet, 0, RigidBodySet, "from_bytes");
        $crate::__serde_owned_value!(ColliderSet, 0, ColliderSet, "from_bytes");
        $crate::__serde_owned_value!(ImpulseJointSet, 0, ImpulseJointSet, "from_bytes");
        $crate::__serde_owned_value!(MultibodyJointSet, 0, MultibodyJointSet, "from_bytes");

        // ============================================================
        // Parameters / scratch / phases
        // ============================================================
        $crate::__serde_owned_value!(
            IntegrationParameters,
            0,
            IntegrationParameters,
            "from_bytes"
        );
        $crate::__serde_owned_value!(IslandManager, 0, IslandManager, "from_bytes");
        $crate::__serde_owned_value!(CCDSolver, 0, CCDSolver, "from_bytes");
        $crate::__serde_owned_value!(BroadPhaseBvh, 0, BroadPhaseBvh, "from_bytes");
        $crate::__serde_owned_value!(NarrowPhase, 0, NarrowPhase, "from_bytes");

        // ============================================================
        // Mass / interaction / shape types
        // ============================================================
        $crate::__serde_owned_value!(MassProperties, 0, MassProperties, "from_bytes");
        $crate::__serde_owned_value!(InteractionGroups, 0, InteractionGroups, "from_bytes");
        $crate::__serde_owned_value!(SharedShape, 0, SharedShape, "from_bytes");

        // ============================================================
        // Joint types (concrete + builders + generic).
        // ============================================================
        $crate::__serde_owned_value!(FixedJoint, 0, FixedJoint, "from_bytes");
        $crate::__serde_owned_value!(FixedJointBuilder, 0, FixedJointBuilder, "from_bytes");
        $crate::__serde_owned_value!(RevoluteJoint, 0, RevoluteJoint, "from_bytes");
        $crate::__serde_owned_value!(RevoluteJointBuilder, 0, RevoluteJointBuilder, "from_bytes");
        $crate::__serde_owned_value!(PrismaticJoint, 0, PrismaticJoint, "from_bytes");
        $crate::__serde_owned_value!(
            PrismaticJointBuilder,
            0,
            PrismaticJointBuilder,
            "from_bytes"
        );
        $crate::__serde_owned_value!(RopeJoint, 0, RopeJoint, "from_bytes");
        $crate::__serde_owned_value!(RopeJointBuilder, 0, RopeJointBuilder, "from_bytes");
        $crate::__serde_owned_value!(SpringJoint, 0, SpringJoint, "from_bytes");
        $crate::__serde_owned_value!(SpringJointBuilder, 0, SpringJointBuilder, "from_bytes");
        $crate::__serde_owned_value!(GenericJoint, 0, GenericJoint, "from_bytes");
        $crate::__serde_owned_value!(GenericJointBuilder, 0, GenericJointBuilder, "from_bytes");

        $crate::__define_serde_dim_specific_joints!($dim);

        // ============================================================
        // Math types — Vec, Point, Rotation, Isometry.
        // ============================================================
        $crate::__define_serde_math!($dim);

        // ============================================================
        // Controllers — try to serialize. Wrap each in `__reduce__` only if
        // serde-derivable on the inner. KinematicCharacterController, PdController,
        // PidController, DynamicRayCastVehicleController are all
        // `Serialize + Deserialize` upstream.
        // ============================================================
        $crate::__serde_owned_value!(
            KinematicCharacterController,
            0,
            KinematicCharacterController,
            "from_bytes"
        );
        $crate::__serde_owned_value!(PdController, 0, PdController, "from_bytes");
        $crate::__serde_owned_value!(PidController, 0, PidController, "from_bytes");

        // `DynamicRayCastVehicleController` + `WheelTuning` are 3D-only
        // upstream. Skip in 2D builds.
        $crate::__define_serde_vehicle!($dim);

        // ============================================================
        // Pipelines + collectors — REFUSE pickle.
        // ============================================================
        #[pymethods]
        impl PhysicsPipeline {
            fn __reduce__(&self) -> PyResult<()> {
                Err($crate::errors::SerializationError::new_err(
                    "PhysicsPipeline is not serializable; it's stateless except for \
                     per-step counters. Re-create a PhysicsPipeline after restoring \
                     the world.",
                ))
            }
        }

        #[pymethods]
        impl CollisionPipeline {
            fn __reduce__(&self) -> PyResult<()> {
                Err($crate::errors::SerializationError::new_err(
                    "CollisionPipeline is not serializable; re-create one after restore.",
                ))
            }
        }

        #[pymethods]
        impl ChannelEventCollector {
            fn __reduce__(&self) -> PyResult<()> {
                Err($crate::errors::SerializationError::new_err(
                    "ChannelEventCollector is not serializable; drain events first.",
                ))
            }
        }

        // ============================================================
        // PhysicsWorld — full snapshot / restore + pickle.
        //
        // Serialized payload is a tuple of the inner rapier values:
        //   (bodies, colliders, impulse_joints, multibody_joints,
        //    broad_phase, narrow_phase, islands, ccd_solver,
        //    integration_parameters, gravity)
        //
        // `event_handler`, `physics_hooks`, `event_error_policy` and
        // `auto_update_query` are NOT serialized — restored worlds are
        // returned with `event_handler = None`, `physics_hooks = None`,
        // `event_error_policy = "defer"`, `auto_update_query = false`.
        // ============================================================

        // Borrow-aware shared serialization payload.
        // Owned form used for deserialization; borrowed form used for
        // serialization to avoid cloning huge component sets.
        type _PhysicsWorldOwned = (
            rapier::dynamics::RigidBodySet,
            rapier::geometry::ColliderSet,
            rapier::dynamics::ImpulseJointSet,
            rapier::dynamics::MultibodyJointSet,
            rapier::geometry::BroadPhaseBvh,
            rapier::geometry::NarrowPhase,
            rapier::dynamics::IslandManager,
            rapier::dynamics::CCDSolver,
            rapier::dynamics::IntegrationParameters,
            $crate::na::SVector<Real, $dim>,
        );

        #[pymethods]
        impl PhysicsWorld {
            /// Take a snapshot of the world's physical state.
            ///
            /// Returns a `bytes` blob containing every internal collection
            /// (rigid bodies, colliders, joints, broad phase, narrow phase,
            /// island manager, CCD solver, integration parameters, gravity).
            ///
            /// User-supplied `event_handler` / `physics_hooks` and the
            /// `event_error_policy` are **NOT** serialized — re-attach them
            /// after `restore`.
            fn snapshot<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
                let bodies = self.bodies.borrow(py);
                let colliders = self.colliders.borrow(py);
                let ij = self.impulse_joints.borrow(py);
                let mj = self.multibody_joints.borrow(py);
                let bp = self.broad_phase.borrow(py);
                let np = self.narrow_phase.borrow(py);
                let islands = self.islands.borrow(py);
                let ccd = self.ccd_solver.borrow(py);
                let ip = self.integration_parameters.borrow(py);
                let tup = (
                    &bodies.0,
                    &colliders.0,
                    &ij.0,
                    &mj.0,
                    &bp.0,
                    &np.0,
                    &islands.0,
                    &ccd.0,
                    &ip.0,
                    &self.gravity.0,
                );
                let payload =
                    $crate::bincode::serialize(&tup).map_err($crate::serde_io::bincode_err)?;
                Ok($crate::serde_io::bytes_to_py(
                    py,
                    &$crate::serde_io::wrap_bincode(payload),
                ))
            }

            /// Re-create a `PhysicsWorld` from a snapshot.
            #[staticmethod]
            fn restore(py: Python<'_>, blob: &Bound<'_, PyBytes>) -> PyResult<Py<PhysicsWorld>> {
                let buf = blob.as_bytes();
                let body = $crate::serde_io::unwrap_bincode(buf)?;
                let owned: _PhysicsWorldOwned =
                    $crate::bincode::deserialize(body).map_err($crate::serde_io::bincode_err)?;
                let (bs, cs, ijs, mjs, bps, nps, isl, ccd, ip, g) = owned;
                let bodies = Py::new(py, RigidBodySet(bs))?;
                let colliders = Py::new(py, ColliderSet(cs))?;
                let impulse_joints = Py::new(py, ImpulseJointSet(ijs))?;
                let multibody_joints = Py::new(py, MultibodyJointSet(mjs))?;
                let broad_phase = Py::new(py, BroadPhaseBvh(bps))?;
                let narrow_phase = Py::new(py, NarrowPhase(nps))?;
                let islands = Py::new(py, IslandManager(isl))?;
                let ccd_solver = Py::new(py, CCDSolver(ccd))?;
                let integration_parameters = Py::new(py, IntegrationParameters(ip))?;
                let physics_pipeline = Py::new(
                    py,
                    PhysicsPipeline(rapier::pipeline::PhysicsPipeline::new()),
                )?;
                let query_pipeline = Py::new(
                    py,
                    QueryPipeline {
                        broad_phase: broad_phase.clone_ref(py),
                        narrow_phase: narrow_phase.clone_ref(py),
                        bodies: bodies.clone_ref(py),
                        colliders: colliders.clone_ref(py),
                    },
                )?;
                Py::new(
                    py,
                    PhysicsWorld {
                        bodies,
                        colliders,
                        impulse_joints,
                        multibody_joints,
                        broad_phase,
                        narrow_phase,
                        islands,
                        ccd_solver,
                        integration_parameters,
                        physics_pipeline,
                        query_pipeline,
                        gravity: $crate::serde_io_make_vec!($dim, g),
                        event_handler: None,
                        physics_hooks: None,
                        auto_update_query: false,
                        event_error_policy: "defer".to_string(),
                    },
                )
            }

            /// JSON debug snapshot — slower & much larger but human-readable.
            ///
            /// Returns a JSON string with a self-describing envelope:
            ///   `{"_magic": "RPYS", "_version": N, "payload": <inner>}`.
            fn snapshot_json(&self, py: Python<'_>) -> PyResult<String> {
                let bodies = self.bodies.borrow(py);
                let colliders = self.colliders.borrow(py);
                let ij = self.impulse_joints.borrow(py);
                let mj = self.multibody_joints.borrow(py);
                let bp = self.broad_phase.borrow(py);
                let np = self.narrow_phase.borrow(py);
                let islands = self.islands.borrow(py);
                let ccd = self.ccd_solver.borrow(py);
                let ip = self.integration_parameters.borrow(py);
                let tup = (
                    &bodies.0,
                    &colliders.0,
                    &ij.0,
                    &mj.0,
                    &bp.0,
                    &np.0,
                    &islands.0,
                    &ccd.0,
                    &ip.0,
                    &self.gravity.0,
                );
                let payload =
                    $crate::serde_json::to_value(&tup).map_err($crate::serde_io::json_err)?;
                let env = $crate::serde_io::wrap_json(payload);
                $crate::serde_json::to_string(&env).map_err($crate::serde_io::json_err)
            }

            /// Restore a world from a JSON snapshot.
            #[staticmethod]
            fn restore_json(py: Python<'_>, s: &str) -> PyResult<Py<PhysicsWorld>> {
                let env: $crate::serde_json::Value =
                    $crate::serde_json::from_str(s).map_err($crate::serde_io::json_err)?;
                let payload = $crate::serde_io::unwrap_json(env)?;
                let owned: _PhysicsWorldOwned =
                    $crate::serde_json::from_value(payload).map_err($crate::serde_io::json_err)?;
                let (bs, cs, ijs, mjs, bps, nps, isl, ccd, ip, g) = owned;
                let bodies = Py::new(py, RigidBodySet(bs))?;
                let colliders = Py::new(py, ColliderSet(cs))?;
                let impulse_joints = Py::new(py, ImpulseJointSet(ijs))?;
                let multibody_joints = Py::new(py, MultibodyJointSet(mjs))?;
                let broad_phase = Py::new(py, BroadPhaseBvh(bps))?;
                let narrow_phase = Py::new(py, NarrowPhase(nps))?;
                let islands = Py::new(py, IslandManager(isl))?;
                let ccd_solver = Py::new(py, CCDSolver(ccd))?;
                let integration_parameters = Py::new(py, IntegrationParameters(ip))?;
                let physics_pipeline = Py::new(
                    py,
                    PhysicsPipeline(rapier::pipeline::PhysicsPipeline::new()),
                )?;
                let query_pipeline = Py::new(
                    py,
                    QueryPipeline {
                        broad_phase: broad_phase.clone_ref(py),
                        narrow_phase: narrow_phase.clone_ref(py),
                        bodies: bodies.clone_ref(py),
                        colliders: colliders.clone_ref(py),
                    },
                )?;
                Py::new(
                    py,
                    PhysicsWorld {
                        bodies,
                        colliders,
                        impulse_joints,
                        multibody_joints,
                        broad_phase,
                        narrow_phase,
                        islands,
                        ccd_solver,
                        integration_parameters,
                        physics_pipeline,
                        query_pipeline,
                        gravity: $crate::serde_io_make_vec!($dim, g),
                        event_handler: None,
                        physics_hooks: None,
                        auto_update_query: false,
                        event_error_policy: "defer".to_string(),
                    },
                )
            }

            /// Pickle support. Returns `(PhysicsWorld.restore, (snapshot_bytes,))`.
            fn __reduce__<'py>(
                &self,
                py: Python<'py>,
            ) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
                let blob = self.snapshot(py)?;
                let ctor: Py<PyAny> = py
                    .get_type_bound::<PhysicsWorld>()
                    .getattr("restore")?
                    .unbind();
                Ok((ctor, (blob,)))
            }
        }
    };
}

// 3D-only vehicle controller types.
#[doc(hidden)]
#[macro_export]
macro_rules! __define_serde_vehicle {
    (3) => {
        $crate::__serde_owned_value!(
            DynamicRayCastVehicleController,
            0,
            DynamicRayCastVehicleController,
            "from_bytes"
        );
        $crate::__serde_owned_value!(WheelTuning, 0, WheelTuning, "from_bytes");
    };
    (2) => {};
}

// Dim-specific joints (SphericalJoint in 3D, PinSlotJoint in 2D).
#[doc(hidden)]
#[macro_export]
macro_rules! __define_serde_dim_specific_joints {
    (3) => {
        $crate::__serde_owned_value!(SphericalJoint, 0, SphericalJoint, "from_bytes");
        $crate::__serde_owned_value!(
            SphericalJointBuilder,
            0,
            SphericalJointBuilder,
            "from_bytes"
        );
    };
    (2) => {
        $crate::__serde_owned_value!(PinSlotJoint, 0, PinSlotJoint, "from_bytes");
        $crate::__serde_owned_value!(PinSlotJointBuilder, 0, PinSlotJointBuilder, "from_bytes");
    };
}

// Wrap a deserialized `na::SVector<Real, DIM>` into the dim-specific Vec
// type (`Vec2` or `Vec3`).
#[doc(hidden)]
#[macro_export]
macro_rules! serde_io_make_vec {
    (2, $g:expr) => {
        Vec2($g)
    };
    (3, $g:expr) => {
        Vec3($g)
    };
}

// `body.angvel()` has different shapes in 2D (scalar) vs 3D (Vector). Used
// inside `RigidBodyBuilder::from_bytes`. Returned by value in both cases.
#[doc(hidden)]
#[macro_export]
macro_rules! __rb_builder_angvel_from_body {
    (2, $b:ident, $body:ident) => {
        $b = $b.angvel($body.angvel());
    };
    (3, $b:ident, $body:ident) => {
        $b = $b.angvel($body.angvel());
    };
}

// Math types per dimension — Vec, Point, Rotation, Isometry.
#[doc(hidden)]
#[macro_export]
macro_rules! __define_serde_math {
    (3) => {
        $crate::__serde_owned_value!(Vec3, 0, Vec3, "from_bytes");
        $crate::__serde_owned_value!(Point3, 0, Point3, "from_bytes");
        $crate::__serde_owned_value!(Rotation3, 0, Rotation3, "from_bytes");
        $crate::__serde_owned_value!(Isometry3, 0, Isometry3, "from_bytes");
    };
    (2) => {
        $crate::__serde_owned_value!(Vec2, 0, Vec2, "from_bytes");
        $crate::__serde_owned_value!(Point2, 0, Point2, "from_bytes");
        $crate::__serde_owned_value!(Rotation2, 0, Rotation2, "from_bytes");
        $crate::__serde_owned_value!(Isometry2, 0, Isometry2, "from_bytes");
    };
}
