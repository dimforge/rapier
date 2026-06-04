//! Macro that emits the user-facing rigid-body / dynamics `#[pyclass]`-es
//! per cdylib.
//!
//! Like `define_math_types!`, this macro is invoked *once per cdylib*. It
//! expects the surrounding cdylib to already have:
//!   - The `Real`/`DIM` aliases and `Py{Vector,Point,Rotation,Isometry,AngVector}`
//!     adapter newtypes (from `define_conv_types!`).
//!   - The user-facing math `#[pyclass]`-es (`Vec3`/`Point3`/`Rotation3`/
//!     `Isometry3` or the 2D analogues) (from `define_math_types!`).
//!   - `use rapier3d as rapier;` (or the appropriate dim/scalar pair).
//!
//! Forward-compat with later phases: the macro emits **minimal stub**
//! `ColliderHandle`, `ColliderSet`, `ImpulseJointSet`, `MultibodyJointSet`
//! so the rigid-body API can compile. Those will be replaced by phases 04
//! and 06.

/// Materialize the dynamics `#[pyclass]` types for a given `(Real, DIM)` pair.
///
/// Invoke once per cdylib after `define_conv_types!` and `define_math_types!`.
/// Produces `register_dynamics(py, m) -> PyResult<()>` for `#[pymodule]`.
#[macro_export]
macro_rules! define_dynamics_types {
    (DIM = 2) => {
        $crate::__define_dynamics_shared!(2);
        $crate::__define_dynamics_2d!();
        $crate::__define_dynamics_register!(2);
    };
    (DIM = 3) => {
        $crate::__define_dynamics_shared!(3);
        $crate::__define_dynamics_3d!();
        $crate::__define_dynamics_register!(3);
    };
}

// ----------------------------------------------------------------------
// Shared dim-agnostic dynamics types
// ----------------------------------------------------------------------

#[doc(hidden)]
#[macro_export]
macro_rules! __define_dynamics_shared {
    ($dim:literal) => {
        // Note: we don't `use` PyTypeError/CompareOp here because
        // `define_math_types!` already brings them into scope. We do bring
        // in PyNotImplementedError (math doesn't need it).
        use $crate::pyo3::exceptions::PyNotImplementedError;

        // -------- Glam <-> nalgebra conversion helpers --------

        /// Convert a nalgebra `SVector<Real, DIM>` to the engine's `Vector`.
        #[allow(dead_code)]
        #[inline]
        fn _na_vec_to_engine(v: $crate::na::SVector<Real, $dim>) -> rapier::math::Vector {
            v.into()
        }

        /// Convert the engine's `Vector` back to nalgebra.
        #[allow(dead_code)]
        #[inline]
        fn _engine_vec_to_na(v: rapier::math::Vector) -> $crate::na::SVector<Real, $dim> {
            v.into()
        }

        // ============================================================
        // ColliderHandle / ColliderSet live in `geometry.rs`.
        // The dynamics macro is invoked AFTER the geometry macro by each
        // cdylib, so those types are already in scope here.
        // ============================================================

        // ============================================================
        // ImpulseJointSet / MultibodyJointSet live in `joints.rs`. The joints macro is invoked BEFORE this one in each cdylib,
        // so those types are already in scope here.
        // ============================================================

        // ============================================================
        // RigidBodyHandle
        // ============================================================

        /// Opaque handle identifying a rigid body inside a `RigidBodySet`.
        ///
        /// Handles are stable across structural mutations of the set and
        /// remain valid until the corresponding body is removed. They wrap an
        /// ``(index, generation)`` pair: the generation guards against reusing
        /// a slot freed by a previous body.
        ///
        /// Use ``RigidBodySet.insert(...)`` to obtain a handle. Handles are
        /// hashable and may be used as dict keys.
        #[pyclass(name = "RigidBodyHandle", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct RigidBodyHandle(pub rapier::dynamics::RigidBodyHandle);

        #[pymethods]
        impl RigidBodyHandle {
            /// Build a handle from raw ``(index, generation)`` parts.
            ///
            /// Most code should obtain handles from ``RigidBodySet.insert``
            /// rather than constructing them directly.
            ///
            /// :param index: slot index inside the set (default ``0``).
            /// :param generation: generation tag for the slot (default ``0``).
            #[new]
            #[pyo3(signature = (index=0, generation=0))]
            fn new(index: u32, generation: u32) -> Self {
                Self(rapier::dynamics::RigidBodyHandle::from_raw_parts(
                    index, generation,
                ))
            }
            /// Build a handle from raw ``(index, generation)`` parts.
            ///
            /// Equivalent to the constructor; provided for symmetry with the
            /// Rust API and for use as a named static method.
            #[staticmethod]
            fn from_raw_parts(index: u32, generation: u32) -> Self {
                Self(rapier::dynamics::RigidBodyHandle::from_raw_parts(
                    index, generation,
                ))
            }
            /// Return a sentinel handle that does not match any real body.
            ///
            /// Useful as a default value or as a placeholder before a body is
            /// actually inserted.
            #[staticmethod]
            fn invalid() -> Self {
                Self(rapier::dynamics::RigidBodyHandle::invalid())
            }
            /// Slot index portion of the handle (read-only).
            #[getter]
            fn index(&self) -> u32 {
                self.0.into_raw_parts().0
            }
            /// Generation portion of the handle (read-only).
            #[getter]
            fn generation(&self) -> u32 {
                self.0.into_raw_parts().1
            }
            /// Hash combining index and generation, enabling use as dict key.
            fn __hash__(&self) -> u64 {
                let (i, g) = self.0.into_raw_parts();
                ((i as u64) << 32) | (g as u64)
            }
            /// Compare two handles for equality. Only ``==`` and ``!=`` are
            /// supported.
            ///
            /// :raises TypeError: when an ordering comparison is requested.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err(
                        "RigidBodyHandle supports only == and !=",
                    )),
                }
            }
            fn __repr__(&self) -> String {
                let (i, g) = self.0.into_raw_parts();
                format!("RigidBodyHandle(index={}, generation={})", i, g)
            }
        }

        // ============================================================
        // RigidBodyType
        // ============================================================

        /// Behavior class of a rigid body.
        ///
        /// Determines how the body responds to forces, contacts and joints:
        ///
        /// - ``DYNAMIC`` — fully simulated; responds to forces and contacts.
        /// - ``FIXED`` — immovable; treated as having infinite mass.
        /// - ``KINEMATIC_VELOCITY_BASED`` — animated by setting ``linvel`` /
        ///   ``angvel`` each frame; pushes dynamic bodies, ignores contacts.
        /// - ``KINEMATIC_POSITION_BASED`` — animated by setting ``position``
        ///   each frame; pushes dynamic bodies, ignores contacts.
        #[pyclass(name = "RigidBodyType", module = "rapier", eq, eq_int)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum RigidBodyType {
            /// Fully simulated: responds to forces, gravity and contacts.
            DYNAMIC,
            /// Immovable: behaves as if it had infinite mass.
            FIXED,
            /// Animated by writing to ``linvel`` / ``angvel`` each step.
            KINEMATIC_VELOCITY_BASED,
            /// Animated by writing to ``position`` each step.
            KINEMATIC_POSITION_BASED,
        }

        impl RigidBodyType {
            #[inline]
            fn to_rapier(self) -> rapier::dynamics::RigidBodyType {
                match self {
                    Self::DYNAMIC => rapier::dynamics::RigidBodyType::Dynamic,
                    Self::FIXED => rapier::dynamics::RigidBodyType::Fixed,
                    Self::KINEMATIC_VELOCITY_BASED => {
                        rapier::dynamics::RigidBodyType::KinematicVelocityBased
                    }
                    Self::KINEMATIC_POSITION_BASED => {
                        rapier::dynamics::RigidBodyType::KinematicPositionBased
                    }
                }
            }
            #[inline]
            fn from_rapier(t: rapier::dynamics::RigidBodyType) -> Self {
                match t {
                    rapier::dynamics::RigidBodyType::Dynamic => Self::DYNAMIC,
                    rapier::dynamics::RigidBodyType::Fixed => Self::FIXED,
                    rapier::dynamics::RigidBodyType::KinematicVelocityBased => {
                        Self::KINEMATIC_VELOCITY_BASED
                    }
                    rapier::dynamics::RigidBodyType::KinematicPositionBased => {
                        Self::KINEMATIC_POSITION_BASED
                    }
                }
            }
        }

        // ============================================================
        // CoefficientCombineRule
        // ============================================================

        /// Rule used to combine per-collider friction/restitution coefficients
        /// at a contact.
        ///
        /// When two colliders touch, each has its own coefficient (friction or
        /// restitution) and a rule. The effective coefficient is derived by
        /// applying the combine rule of the *highest priority* among the two
        /// (``MAX > MULTIPLY > MIN > AVERAGE``).
        #[pyclass(name = "CoefficientCombineRule", module = "rapier", eq, eq_int)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum CoefficientCombineRule {
            /// Arithmetic mean of the two coefficients.
            AVERAGE,
            /// Minimum of the two coefficients.
            MIN,
            /// Product of the two coefficients.
            MULTIPLY,
            /// Maximum of the two coefficients.
            MAX,
            /// Sum of the two coefficients, clamped to ``[0, 1]``.
            CLAMPED_SUM,
        }

        impl CoefficientCombineRule {
            #[allow(dead_code)]
            #[inline]
            fn to_rapier(self) -> rapier::dynamics::CoefficientCombineRule {
                match self {
                    Self::AVERAGE => rapier::dynamics::CoefficientCombineRule::Average,
                    Self::MIN => rapier::dynamics::CoefficientCombineRule::Min,
                    Self::MULTIPLY => rapier::dynamics::CoefficientCombineRule::Multiply,
                    Self::MAX => rapier::dynamics::CoefficientCombineRule::Max,
                    Self::CLAMPED_SUM => rapier::dynamics::CoefficientCombineRule::ClampedSum,
                }
            }
            #[allow(dead_code)]
            #[inline]
            fn from_rapier(r: rapier::dynamics::CoefficientCombineRule) -> Self {
                match r {
                    rapier::dynamics::CoefficientCombineRule::Average => Self::AVERAGE,
                    rapier::dynamics::CoefficientCombineRule::Min => Self::MIN,
                    rapier::dynamics::CoefficientCombineRule::Multiply => Self::MULTIPLY,
                    rapier::dynamics::CoefficientCombineRule::Max => Self::MAX,
                    rapier::dynamics::CoefficientCombineRule::ClampedSum => Self::CLAMPED_SUM,
                }
            }
        }

        // ============================================================
        // RigidBodyActivation
        // ============================================================

        /// Sleep / activation state of a rigid body.
        ///
        /// A body whose linear and angular velocity stays below the sleep
        /// thresholds for ``time_until_sleep`` seconds is put to sleep to save
        /// compute. A sleeping body is skipped by the solver until something
        /// wakes it (contact, force, joint, manual ``wake_up``).
        ///
        /// Construct via ``RigidBodyActivation()`` (active by default),
        /// ``RigidBodyActivation.inactive()`` for an initially-sleeping body
        /// or ``RigidBodyActivation.cannot_sleep()`` to disable sleep.
        #[pyclass(name = "RigidBodyActivation", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyActivation(pub rapier::dynamics::RigidBodyActivation);

        #[pymethods]
        impl RigidBodyActivation {
            /// Build the default (active, can sleep) activation state.
            #[new]
            fn new() -> Self {
                Self(rapier::dynamics::RigidBodyActivation::active())
            }
            /// Default active activation: body is awake and may sleep later.
            #[staticmethod]
            fn active() -> Self {
                Self(rapier::dynamics::RigidBodyActivation::active())
            }
            /// Initially sleeping activation: body starts asleep.
            #[staticmethod]
            fn inactive() -> Self {
                Self(rapier::dynamics::RigidBodyActivation::inactive())
            }
            /// Activation that never sleeps — useful for player avatars and
            /// other always-simulated bodies.
            #[staticmethod]
            fn cannot_sleep() -> Self {
                Self(rapier::dynamics::RigidBodyActivation::cannot_sleep())
            }

            /// Normalized linear-velocity threshold below which the body may
            /// fall asleep (read+write).
            #[getter]
            fn linear_threshold(&self) -> Real {
                self.0.normalized_linear_threshold
            }
            /// Set the normalized linear sleep threshold.
            #[setter]
            fn set_linear_threshold(&mut self, v: Real) {
                self.0.normalized_linear_threshold = v;
            }
            /// Alias for ``linear_threshold`` (read+write).
            #[getter]
            fn sleep_threshold(&self) -> Real {
                self.0.normalized_linear_threshold
            }
            /// Alias for ``set_linear_threshold``.
            #[setter]
            fn set_sleep_threshold(&mut self, v: Real) {
                self.0.normalized_linear_threshold = v;
            }
            /// Angular-velocity threshold below which the body may fall
            /// asleep (read+write).
            #[getter]
            fn angular_threshold(&self) -> Real {
                self.0.angular_threshold
            }
            /// Set the angular sleep threshold.
            #[setter]
            fn set_angular_threshold(&mut self, v: Real) {
                self.0.angular_threshold = v;
            }
            /// Seconds the body must stay below thresholds before sleeping.
            ///
            /// Negative values disable sleep for this body.
            #[getter]
            fn time_until_sleep(&self) -> Real {
                self.0.time_until_sleep
            }
            /// Set the sleep delay in seconds.
            #[setter]
            fn set_time_until_sleep(&mut self, v: Real) {
                self.0.time_until_sleep = v;
            }
            /// Whether the body is currently sleeping (read+write).
            #[getter]
            fn sleeping(&self) -> bool {
                self.0.sleeping
            }
            /// Force the sleeping flag directly. Prefer ``RigidBody.wake_up``
            /// / ``RigidBody.sleep`` for proper bookkeeping.
            #[setter]
            fn set_sleeping(&mut self, v: bool) {
                self.0.sleeping = v;
            }
            /// Whether the body is currently active (i.e. not sleeping).
            ///
            /// :returns: ``True`` if the body participates in the simulation
            ///     this step.
            fn is_active(&self) -> bool {
                self.0.is_active()
            }
            fn __repr__(&self) -> String {
                format!(
                    "RigidBodyActivation(linear={}, angular={}, time_until_sleep={}, sleeping={})",
                    self.0.normalized_linear_threshold,
                    self.0.angular_threshold,
                    self.0.time_until_sleep,
                    self.0.sleeping,
                )
            }
        }

        // ============================================================
        // SpringCoefficients
        // ============================================================

        /// Soft-constraint spring parameters used for contact and joint
        /// regularization.
        ///
        /// Internally Rapier converts ``stiffness`` (natural frequency, Hz)
        /// and ``damping`` (damping ratio, unitless, ``1`` = critical) to the
        /// effective spring/damper coefficients used by the solver. Defaults
        /// suitable for typical contacts and joints can be obtained via
        /// ``SpringCoefficients.contact_defaults()`` and
        /// ``SpringCoefficients.joint_defaults()``.
        #[pyclass(name = "SpringCoefficients", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct SpringCoefficients(pub rapier::dynamics::SpringCoefficients<Real>);

        #[pymethods]
        impl SpringCoefficients {
            /// Build a spring with the given natural frequency and damping
            /// ratio.
            ///
            /// :param stiffness: natural frequency in Hz (default ``30.0``).
            /// :param damping: damping ratio (default ``5.0``).
            #[new]
            #[pyo3(signature = (stiffness=30.0 as Real, damping=5.0 as Real))]
            fn new(stiffness: Real, damping: Real) -> Self {
                Self(rapier::dynamics::SpringCoefficients {
                    natural_frequency: stiffness,
                    damping_ratio: damping,
                })
            }
            /// Return the default spring coefficients used for contact
            /// regularization.
            #[staticmethod]
            fn contact_defaults() -> Self {
                Self(rapier::dynamics::SpringCoefficients::contact_defaults())
            }
            /// Return the default spring coefficients used for joint
            /// regularization.
            #[staticmethod]
            fn joint_defaults() -> Self {
                Self(rapier::dynamics::SpringCoefficients::joint_defaults())
            }
            /// Spring natural frequency (alias for ``natural_frequency``).
            #[getter]
            fn stiffness(&self) -> Real {
                self.0.natural_frequency
            }
            /// Set the natural frequency.
            #[setter]
            fn set_stiffness(&mut self, v: Real) {
                self.0.natural_frequency = v;
            }
            /// Natural frequency of the spring, in Hz.
            #[getter]
            fn natural_frequency(&self) -> Real {
                self.0.natural_frequency
            }
            /// Set the natural frequency, in Hz.
            #[setter]
            fn set_natural_frequency(&mut self, v: Real) {
                self.0.natural_frequency = v;
            }
            /// Damping ratio (alias for ``damping_ratio``).
            #[getter]
            fn damping(&self) -> Real {
                self.0.damping_ratio
            }
            /// Set the damping ratio.
            #[setter]
            fn set_damping(&mut self, v: Real) {
                self.0.damping_ratio = v;
            }
            /// Damping ratio (unitless, ``1.0`` is critically damped).
            #[getter]
            fn damping_ratio(&self) -> Real {
                self.0.damping_ratio
            }
            /// Set the damping ratio.
            #[setter]
            fn set_damping_ratio(&mut self, v: Real) {
                self.0.damping_ratio = v;
            }
            fn __repr__(&self) -> String {
                format!(
                    "SpringCoefficients(stiffness={}, damping={})",
                    self.0.natural_frequency, self.0.damping_ratio
                )
            }
        }

        // ============================================================
        // IslandManager
        // ============================================================

        /// Tracks groups of interacting (connected) rigid bodies — *islands*
        /// — and which of them are currently active.
        ///
        /// The island manager is updated by ``PhysicsWorld.step`` each frame
        /// and is mainly used internally by the solver. Iterating the
        /// manager yields the handles of bodies that are awake this step.
        #[pyclass(name = "IslandManager", module = "rapier", unsendable)]
        pub struct IslandManager(pub rapier::dynamics::IslandManager);

        #[pymethods]
        impl IslandManager {
            /// Build an empty island manager.
            #[new]
            fn new() -> Self {
                Self(rapier::dynamics::IslandManager::new())
            }

            /// Snapshot of every awake dynamic body currently tracked.
            ///
            /// :returns: a list of handles of bodies that are active this step.
            fn active_dynamic_set(&self) -> Vec<RigidBodyHandle> {
                self.0.active_bodies().map(RigidBodyHandle).collect()
            }

            /// Number of currently active bodies (``len(islands)``).
            fn __len__(&self) -> usize {
                self.0.active_bodies().count()
            }

            /// Iterate over currently active body handles.
            fn __iter__(slf: PyRef<'_, Self>) -> PyResult<Py<RigidBodyHandleIter>> {
                let handles: Vec<RigidBodyHandle> =
                    slf.0.active_bodies().map(RigidBodyHandle).collect();
                Py::new(slf.py(), RigidBodyHandleIter { handles, i: 0 })
            }

            /// Whether ``h`` is currently active.
            ///
            /// Sleeping bodies are not part of the active set and will return
            /// ``False`` here.
            fn contains_handle(&self, h: &RigidBodyHandle) -> bool {
                // No public `contains` query; we iterate the active set.
                // Sleeping bodies aren't in `active_bodies`, so this returns
                // false for them — a later revision may revisit this.
                self.0.active_bodies().any(|x| x == h.0)
            }

            /// Whether ``h1`` and ``h2`` are part of the same active island.
            ///
            /// Currently approximated as both being in the active set;
            /// returns ``False`` if either body is sleeping.
            fn same_island(&self, h1: &RigidBodyHandle, h2: &RigidBodyHandle) -> bool {
                // No public per-island id; approximate "same island" as both
                // being in the active set. The pipeline integration wires this properly.
                let mut a = false;
                let mut b = false;
                for h in self.0.active_bodies() {
                    if h == h1.0 {
                        a = true;
                    }
                    if h == h2.0 {
                        b = true;
                    }
                    if a && b {
                        break;
                    }
                }
                a && b
            }

            /// Position of ``h`` inside the active set.
            ///
            /// Reserved for future use; currently always returns ``0``.
            #[allow(unused_variables)]
            fn active_set_offset(&self, h: &RigidBodyHandle) -> usize {
                // Not exposed publicly in the engine; a later revision may surface it.
                0
            }
        }

        /// Iterator yielding ``RigidBodyHandle`` values.
        ///
        /// Produced by ``IslandManager.__iter__`` and ``RigidBodySet.handles``;
        /// not intended to be constructed directly.
        #[pyclass]
        pub struct RigidBodyHandleIter {
            handles: Vec<RigidBodyHandle>,
            i: usize,
        }

        #[pymethods]
        impl RigidBodyHandleIter {
            /// Return ``self`` so the object satisfies the iterator protocol.
            fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
                slf
            }
            /// Return the next handle, or raise ``StopIteration``.
            fn __next__(mut slf: PyRefMut<'_, Self>) -> Option<RigidBodyHandle> {
                if slf.i >= slf.handles.len() {
                    return None;
                }
                let v = slf.handles[slf.i];
                slf.i += 1;
                Some(v)
            }
        }

        // ============================================================
        // CCDSolver — constructor + clear; solve_ccd handled by the pipeline.
        // ============================================================

        /// Continuous Collision Detection (CCD) solver.
        ///
        /// CCD prevents fast-moving bodies from tunneling through thin
        /// obstacles by performing sub-step time-of-impact queries. Most users
        /// should rely on ``PhysicsWorld.step``, which manages a ``CCDSolver``
        /// internally; this class exists mainly to mirror the engine's
        /// structure.
        #[pyclass(name = "CCDSolver", module = "rapier", unsendable)]
        pub struct CCDSolver(pub rapier::dynamics::CCDSolver);

        #[pymethods]
        impl CCDSolver {
            /// Build a fresh CCD solver.
            #[new]
            fn new() -> Self {
                Self(rapier::dynamics::CCDSolver::new())
            }
            /// Clear any per-step CCD state.
            fn clear(&mut self) {
                // Stateless w.r.t. the public API; may be revisited.
            }
            /// Run a CCD pass. *Not exposed directly* — CCD is driven by
            /// ``PhysicsWorld.step``.
            ///
            /// :raises NotImplementedError: always; use ``PhysicsWorld.step``
            ///     which handles CCD internally.
            #[allow(unused_variables)]
            #[pyo3(signature = (*args, **kwargs))]
            fn solve_ccd(
                &self,
                args: &Bound<'_, $crate::pyo3::types::PyTuple>,
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<()> {
                Err(PyNotImplementedError::new_err(
                    "CCDSolver.solve_ccd is wired in the pipeline. \
                     Call PhysicsWorld.step instead.",
                ))
            }
        }
    };
}

// ----------------------------------------------------------------------
// 3D-specific dynamics types
// ----------------------------------------------------------------------

#[doc(hidden)]
#[macro_export]
macro_rules! __define_dynamics_3d {
    () => {
        // ============================================================
        // FrictionModel (3D only)
        // ============================================================

        /// Friction model used by the 3D contact solver.
        ///
        /// - ``COEFFICIENT`` — simplified pyramidal friction. Faster and
        ///   numerically friendly; the default.
        /// - ``COULOMB`` — circular Coulomb friction cone. More physically
        ///   correct, slightly more expensive.
        #[pyclass(name = "FrictionModel", module = "rapier", eq, eq_int)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum FrictionModel {
            /// Cheap pyramidal approximation of the friction cone.
            COEFFICIENT,
            /// True circular Coulomb friction cone.
            COULOMB,
        }

        impl FrictionModel {
            #[inline]
            fn to_rapier(self) -> rapier::dynamics::FrictionModel {
                match self {
                    Self::COEFFICIENT => rapier::dynamics::FrictionModel::Simplified,
                    Self::COULOMB => rapier::dynamics::FrictionModel::Coulomb,
                }
            }
            #[inline]
            fn from_rapier(r: rapier::dynamics::FrictionModel) -> Self {
                match r {
                    rapier::dynamics::FrictionModel::Simplified => Self::COEFFICIENT,
                    rapier::dynamics::FrictionModel::Coulomb => Self::COULOMB,
                }
            }
        }

        // ============================================================
        // MassProperties (3D)
        // ============================================================

        /// Mass, center of mass and inertia tensor of a rigid body (3D).
        ///
        /// In 3D the inertia tensor is stored in its diagonal (principal)
        /// form together with the rotation ``principal_inertia_local_frame``
        /// aligning the principal axes with the body's local frame.
        ///
        /// Use one of the ``from_*`` factories to derive mass properties
        /// from a shape and a density, or construct directly with
        /// ``MassProperties(local_com, mass, principal_inertia, frame)``.
        /// Mass properties of compound bodies can be combined with ``+``;
        /// subtracting (``-``) inverts the operation.
        #[pyclass(name = "MassProperties", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct MassProperties(pub rapier::dynamics::MassProperties);

        #[pymethods]
        impl MassProperties {
            /// Build mass properties from explicit values.
            ///
            /// :param local_com: center of mass in body-local space (default
            ///     origin).
            /// :param mass: total mass (default ``0``).
            /// :param principal_inertia: diagonal of the principal inertia
            ///     tensor (default zero).
            /// :param principal_inertia_local_frame: rotation aligning the
            ///     principal axes with the body frame (default identity).
            #[new]
            #[pyo3(signature = (local_com=None, mass=0.0 as Real, principal_inertia=None, principal_inertia_local_frame=None))]
            fn new(
                local_com: Option<PyPoint>,
                mass: Real,
                principal_inertia: Option<PyVector>,
                principal_inertia_local_frame: Option<PyRotation>,
            ) -> Self {
                let com = local_com.map(|p| p.0.coords).unwrap_or_else($crate::na::Vector3::zeros);
                let inertia = principal_inertia.map(|v| v.0).unwrap_or_else($crate::na::Vector3::zeros);
                let frame = principal_inertia_local_frame
                    .map(|r| r.0)
                    .unwrap_or_else($crate::na::UnitQuaternion::identity);
                Self(rapier::dynamics::MassProperties::with_principal_inertia_frame(
                    com.into(),
                    mass,
                    inertia.into(),
                    frame.into(),
                ))
            }

            /// Return the zero mass properties (mass = inertia = ``0``).
            #[staticmethod]
            fn zero() -> Self { Self(rapier::dynamics::MassProperties::default()) }

            /// Mass properties of a uniform-density ball.
            ///
            /// :param density: mass per unit volume.
            /// :param radius: ball radius.
            #[staticmethod]
            fn from_ball(density: Real, radius: Real) -> Self {
                Self(rapier::dynamics::MassProperties::from_ball(density, radius))
            }

            /// Mass properties of a uniform-density axis-aligned cuboid.
            ///
            /// :param density: mass per unit volume.
            /// :param half_extents: half-extents along ``x``, ``y``, ``z``.
            #[staticmethod]
            fn from_cuboid(density: Real, half_extents: PyVector) -> Self {
                Self(rapier::dynamics::MassProperties::from_cuboid(
                    density, half_extents.0.into(),
                ))
            }

            /// Mass properties of a uniform-density capsule between two
            /// endpoints.
            ///
            /// :param density: mass per unit volume.
            /// :param a: capsule segment start (body-local).
            /// :param b: capsule segment end (body-local).
            /// :param radius: capsule radius.
            #[staticmethod]
            fn from_capsule(density: Real, a: PyPoint, b: PyPoint, radius: Real) -> Self {
                Self(rapier::dynamics::MassProperties::from_capsule(
                    density, a.0.into(), b.0.into(), radius,
                ))
            }

            /// Mass properties of a uniform-density cylinder along the local
            /// ``y`` axis.
            ///
            /// :param density: mass per unit volume.
            /// :param half_height: half the cylinder height.
            /// :param radius: cylinder radius.
            #[staticmethod]
            fn from_cylinder(density: Real, half_height: Real, radius: Real) -> Self {
                Self(rapier::dynamics::MassProperties::from_cylinder(
                    density, half_height, radius,
                ))
            }

            /// Mass properties of a uniform-density cone along the local
            /// ``y`` axis.
            ///
            /// :param density: mass per unit volume.
            /// :param half_height: half the cone height.
            /// :param radius: base radius.
            #[staticmethod]
            fn from_cone(density: Real, half_height: Real, radius: Real) -> Self {
                Self(rapier::dynamics::MassProperties::from_cone(
                    density, half_height, radius,
                ))
            }

            /// Total mass (read-only).
            #[getter] fn mass(&self) -> Real { self.0.mass() }
            /// Inverse mass — ``0`` for an infinite-mass body (read-only).
            #[getter] fn inv_mass(&self) -> Real { self.0.inv_mass }

            /// Center of mass expressed in body-local coordinates (read-only).
            #[getter]
            fn local_com(&self) -> Point3 {
                let v: $crate::na::Vector3<Real> = self.0.local_com.into();
                Point3($crate::na::Point3::from(v))
            }

            /// Principal inertia (diagonal of the inertia tensor in the
            /// principal frame) (read-only).
            #[getter]
            fn principal_inertia(&self) -> Vec3 {
                let v: $crate::na::Vector3<Real> = self.0.principal_inertia().into();
                Vec3(v)
            }

            /// Componentwise square root of the inverse principal inertia
            /// (read-only). Used by the solver to scale angular impulses.
            #[getter]
            fn inv_principal_inertia_sqrt(&self) -> Vec3 {
                let v = self.0.inv_principal_inertia;
                Vec3($crate::na::Vector3::new(v.x.sqrt(), v.y.sqrt(), v.z.sqrt()))
            }

            /// Rotation aligning the principal axes with the body frame
            /// (read-only).
            #[getter]
            fn principal_inertia_local_frame(&self) -> Rotation3 {
                let q: $crate::na::UnitQuaternion<Real> = self.0.principal_inertia_local_frame.into();
                Rotation3(q)
            }

            /// Return a copy transformed by ``iso``.
            ///
            /// Useful when expressing one body's mass properties in another
            /// frame, e.g. when attaching as a child.
            fn transform_by(&self, iso: PyIsometry) -> Self {
                let pose: rapier::math::Pose = iso.0.into();
                Self(self.0.transform_by(&pose))
            }

            /// Sum two mass distributions (parallel-axis theorem).
            fn __add__(&self, other: &MassProperties) -> Self { Self(self.0 + other.0) }
            /// Subtract ``other`` from ``self``; inverse of ``__add__``.
            fn __sub__(&self, other: &MassProperties) -> Self { Self(self.0 - other.0) }

            fn __repr__(&self) -> String {
                format!(
                    "MassProperties(mass={}, local_com=({}, {}, {}))",
                    self.0.mass(),
                    self.0.local_com.x, self.0.local_com.y, self.0.local_com.z,
                )
            }
        }

        // ============================================================
        // RigidBodyAdditionalMassProps (3D)
        // ============================================================

        /// Extra mass / mass properties to add to a body on top of what its
        /// colliders contribute.
        ///
        /// Useful when you want a body's collider geometry to provide most of
        /// the inertia but still bias the total mass.
        #[pyclass(name = "RigidBodyAdditionalMassProps", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyAdditionalMassProps(pub rapier::dynamics::RigidBodyAdditionalMassProps);

        #[pymethods]
        impl RigidBodyAdditionalMassProps {
            /// Add ``mass`` to the body without changing its center of mass
            /// or inertia tensor.
            #[staticmethod]
            fn from_mass(mass: Real) -> Self {
                Self(rapier::dynamics::RigidBodyAdditionalMassProps::Mass(mass))
            }
            /// Add the given full mass properties to the body.
            #[staticmethod]
            fn from_mass_properties(mp: &MassProperties) -> Self {
                Self(rapier::dynamics::RigidBodyAdditionalMassProps::MassProps(mp.0))
            }
        }

        // ============================================================
        // RigidBody component views (3D)
        // ============================================================

        /// Linear and angular damping coefficients (per-second decay rates)
        /// applied to a body's velocity each step.
        ///
        /// Damping bleeds energy away independent of contacts; useful for
        /// stabilizing simulations and modeling drag.
        #[pyclass(name = "RigidBodyDamping", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyDamping {
            /// Linear-velocity damping coefficient (read+write).
            #[pyo3(get, set)] pub linear_damping: Real,
            /// Angular-velocity damping coefficient (read+write).
            #[pyo3(get, set)] pub angular_damping: Real,
        }
        #[pymethods]
        impl RigidBodyDamping {
            /// Build damping coefficients (default zero / no damping).
            #[new]
            #[pyo3(signature = (linear_damping=0.0 as Real, angular_damping=0.0 as Real))]
            fn new(linear_damping: Real, angular_damping: Real) -> Self {
                Self { linear_damping, angular_damping }
            }
        }

        /// Dominance group used to bias contact resolution between bodies.
        ///
        /// In a contact between bodies of different dominance, the
        /// higher-dominance body behaves as if it had infinite mass with
        /// respect to the lower-dominance one. Groups range in ``[-127, 127]``.
        #[pyclass(name = "RigidBodyDominance", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyDominance {
            /// Dominance group identifier (read+write).
            #[pyo3(get, set)] pub group: i8,
        }
        #[pymethods]
        impl RigidBodyDominance {
            /// Build a dominance descriptor (default group ``0``).
            #[new]
            #[pyo3(signature = (group=0))]
            fn new(group: i8) -> Self { Self { group } }
        }

        /// Per-body CCD configuration.
        ///
        /// ``ccd_enabled`` toggles substep CCD; ``soft_ccd_prediction`` is a
        /// distance over which "soft" CCD pushes the body back from imminent
        /// penetrations (``0`` disables soft CCD).
        #[pyclass(name = "RigidBodyCcd", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyCcd {
            /// Whether substep CCD is enabled for this body (read+write).
            #[pyo3(get, set)] pub ccd_enabled: bool,
            /// Soft-CCD prediction distance (read+write); ``0`` disables it.
            #[pyo3(get, set)] pub soft_ccd_prediction: Real,
        }
        #[pymethods]
        impl RigidBodyCcd {
            /// Build a CCD descriptor (defaults: disabled, ``0`` soft CCD).
            #[new]
            #[pyo3(signature = (ccd_enabled=false, soft_ccd_prediction=0.0 as Real))]
            fn new(ccd_enabled: bool, soft_ccd_prediction: Real) -> Self {
                Self { ccd_enabled, soft_ccd_prediction }
            }
        }

        /// Linear and angular velocity of a rigid body (3D).
        ///
        /// In 3D both ``linvel`` and ``angvel`` are 3-component vectors. The
        /// angular velocity is expressed in world space, with magnitude in
        /// radians per second.
        #[pyclass(name = "RigidBodyVelocity", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyVelocity {
            /// Linear velocity in world space (read+write).
            #[pyo3(get, set)] pub linvel: Vec3,
            /// Angular velocity in world space, rad/s (read+write).
            #[pyo3(get, set)] pub angvel: Vec3,
        }
        #[pymethods]
        impl RigidBodyVelocity {
            /// Build a velocity (default ``linvel = angvel = 0``).
            #[new]
            #[pyo3(signature = (linvel=None, angvel=None))]
            fn new(linvel: Option<PyVector>, angvel: Option<PyVector>) -> Self {
                let lin = linvel.map(|v| v.0).unwrap_or_else($crate::na::Vector3::zeros);
                let ang = angvel.map(|v| v.0).unwrap_or_else($crate::na::Vector3::zeros);
                Self { linvel: Vec3(lin), angvel: Vec3(ang) }
            }
        }

        /// Accumulated external forces, torques and gravity scaling.
        ///
        /// ``force`` and ``torque`` accumulate over a single step and are
        /// cleared automatically before the next ``step()`` (unlike impulses,
        /// which act once instantaneously).
        #[pyclass(name = "RigidBodyForces", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyForces {
            /// Accumulated linear force, world space (read+write).
            #[pyo3(get, set)] pub force: Vec3,
            /// Accumulated torque, world space (read+write).
            #[pyo3(get, set)] pub torque: Vec3,
            /// Per-body multiplier on the world gravity (read+write).
            #[pyo3(get, set)] pub gravity_scale: Real,
        }

        /// Body-local mass properties view.
        ///
        /// Mirror of the ``MassProperties`` stored on a ``RigidBody``.
        #[pyclass(name = "RigidBodyMassProps", module = "rapier")]
        #[derive(Debug, Clone)]
        pub struct RigidBodyMassProps {
            /// Mass properties expressed in body-local space (read-only).
            #[pyo3(get)] pub local_mprops: MassProperties,
        }

        /// Current and predicted pose of a rigid body.
        ///
        /// ``position`` is the pose used for queries; ``next_position`` is the
        /// pose that will be committed after integration.
        #[pyclass(name = "RigidBodyPosition", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyPosition {
            /// Current world-space pose (read+write).
            #[pyo3(get, set)] pub position: Isometry3,
            /// Predicted world-space pose after the next step (read+write).
            #[pyo3(get, set)] pub next_position: Isometry3,
        }

        // ============================================================
        // LockedAxes (3D)
        // ============================================================

        /// Bitflags selecting which translation / rotation degrees of freedom
        /// of a rigid body are locked (3D).
        ///
        /// Combine flags with ``|`` (union), ``&`` (intersection), ``^`` (xor)
        /// and ``-`` (set difference). ``in`` / ``contains`` test subset
        /// membership.
        ///
        /// Example::
        ///
        ///     # Lock translation along Y and all rotations:
        ///     axes = LockedAxes.TRANSLATION_LOCKED_Y | LockedAxes.ROTATION_LOCKED
        #[pyclass(name = "LockedAxes", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct LockedAxes(pub rapier::dynamics::LockedAxes);

        #[pymethods]
        impl LockedAxes {
            /// Build a ``LockedAxes`` value from a raw bitmask.
            ///
            /// :param bits: bitmask combining the ``LockedAxes`` class
            ///     attributes (default ``0`` = no axes locked).
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u8) -> Self {
                Self(rapier::dynamics::LockedAxes::from_bits_truncate(bits))
            }
            /// Return the empty flag set (no axes locked).
            #[staticmethod]
            fn empty() -> Self { Self(rapier::dynamics::LockedAxes::empty()) }
            /// Return the flag set locking every translation and rotation
            /// axis.
            #[staticmethod]
            fn all_axes() -> Self { Self(rapier::dynamics::LockedAxes::all()) }

            /// Lock translation along the local ``x`` axis.
            #[classattr]
            const TRANSLATION_LOCKED_X: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_X);
            /// Lock translation along the local ``y`` axis.
            #[classattr]
            const TRANSLATION_LOCKED_Y: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_Y);
            /// Lock translation along the local ``z`` axis.
            #[classattr]
            const TRANSLATION_LOCKED_Z: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_Z);
            /// Lock all three translation axes.
            #[classattr]
            const TRANSLATION_LOCKED: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED);
            /// Lock rotation around the local ``x`` axis.
            #[classattr]
            const ROTATION_LOCKED_X: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::ROTATION_LOCKED_X);
            /// Lock rotation around the local ``y`` axis.
            #[classattr]
            const ROTATION_LOCKED_Y: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::ROTATION_LOCKED_Y);
            /// Lock rotation around the local ``z`` axis.
            #[classattr]
            const ROTATION_LOCKED_Z: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::ROTATION_LOCKED_Z);
            /// Lock all three rotation axes.
            #[classattr]
            const ROTATION_LOCKED: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::ROTATION_LOCKED);
            /// Lock every translation and rotation axis.
            #[classattr]
            const ALL: LockedAxes = LockedAxes(rapier::dynamics::LockedAxes::all());

            /// Raw bitmask backing this flag set (read-only).
            #[getter] fn bits(&self) -> u8 { self.0.bits() }
            /// Whether ``self`` is a superset of ``other``.
            fn contains(&self, other: &Self) -> bool { self.0.contains(other.0) }
            /// Whether no axes are locked.
            fn is_empty(&self) -> bool { self.0.is_empty() }
            /// ``other in self`` — subset test (alias for ``contains``).
            fn __contains__(&self, other: &Self) -> bool { self.0.contains(other.0) }
            /// Union of two flag sets (``self | other``).
            fn __or__(&self, other: &Self) -> Self { Self(self.0 | other.0) }
            /// Intersection of two flag sets (``self & other``).
            fn __and__(&self, other: &Self) -> Self { Self(self.0 & other.0) }
            /// Symmetric difference (``self ^ other``).
            fn __xor__(&self, other: &Self) -> Self { Self(self.0 ^ other.0) }
            /// Complement of ``self`` within the full set of axes.
            fn __invert__(&self) -> Self { Self(!self.0) }
            /// Set difference (``self - other``): axes in ``self`` not in
            /// ``other``.
            fn __sub__(&self, other: &Self) -> Self { Self(self.0 - other.0) }
            /// Truthiness: ``True`` if any axis is locked.
            fn __bool__(&self) -> bool { !self.0.is_empty() }
            /// Equality comparison. Only ``==`` and ``!=`` are supported.
            ///
            /// :raises TypeError: when an ordering comparison is requested.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err("LockedAxes supports only == and !=")),
                }
            }
            /// Hash equal to the underlying bitmask.
            fn __hash__(&self) -> u64 { self.0.bits() as u64 }
            fn __repr__(&self) -> String {
                format!("LockedAxes(bits={:#010b})", self.0.bits())
            }
        }

        // ============================================================
        // IntegrationParameters (3D — has friction_model)
        // ============================================================

        /// Tunable parameters for the time-step and contact/joint solver.
        ///
        /// The defaults (``IntegrationParameters()``) are appropriate for
        /// most games and robotics simulations at ``dt ≈ 1/60`` seconds.
        ///
        /// Many fields are stored in *normalized* units of ``length_unit``:
        /// e.g. ``normalized_allowed_linear_error`` is in fractions of
        /// ``length_unit``. Use the helper methods ``allowed_linear_error()``,
        /// ``max_corrective_velocity()`` and ``prediction_distance()`` to get
        /// the de-normalized values.
        #[pyclass(name = "IntegrationParameters", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct IntegrationParameters(pub rapier::dynamics::IntegrationParameters);

        #[pymethods]
        impl IntegrationParameters {
            /// Build the default integration parameters.
            #[new]
            fn new() -> Self { Self(rapier::dynamics::IntegrationParameters::default()) }

            /// Return the default integration parameters.
            #[staticmethod]
            fn default_params() -> Self {
                Self(rapier::dynamics::IntegrationParameters::default())
            }

            /// Simulation timestep in seconds (default ``≈ 1/60``).
            #[getter] fn dt(&self) -> Real { self.0.dt }
            /// Set the simulation timestep in seconds.
            #[setter] fn set_dt(&mut self, v: Real) { self.0.dt = v; }
            /// Minimum substep length used by CCD, in seconds.
            #[getter] fn min_ccd_dt(&self) -> Real { self.0.min_ccd_dt }
            /// Set the minimum CCD substep length.
            #[setter] fn set_min_ccd_dt(&mut self, v: Real) { self.0.min_ccd_dt = v; }
            /// Warmstart coefficient in ``[0, 1]`` for the solver's impulse
            /// reuse between steps; higher values converge faster but may
            /// jitter.
            #[getter] fn warmstart_coefficient(&self) -> Real { self.0.warmstart_coefficient }
            /// Set the warmstart coefficient.
            #[setter] fn set_warmstart_coefficient(&mut self, v: Real) { self.0.warmstart_coefficient = v; }
            /// Reference length unit (1 meter by default). Other "normalized"
            /// parameters are expressed in fractions of this unit.
            #[getter] fn length_unit(&self) -> Real { self.0.length_unit }
            /// Set the reference length unit.
            #[setter] fn set_length_unit(&mut self, v: Real) { self.0.length_unit = v; }
            /// Allowed penetration error, in fractions of ``length_unit``.
            #[getter] fn normalized_allowed_linear_error(&self) -> Real { self.0.normalized_allowed_linear_error }
            /// Set the normalized allowed linear error.
            #[setter] fn set_normalized_allowed_linear_error(&mut self, v: Real) { self.0.normalized_allowed_linear_error = v; }
            /// Maximum penetration-correction velocity, in fractions of
            /// ``length_unit`` per second.
            #[getter] fn normalized_max_corrective_velocity(&self) -> Real { self.0.normalized_max_corrective_velocity }
            /// Set the normalized maximum corrective velocity.
            #[setter] fn set_normalized_max_corrective_velocity(&mut self, v: Real) { self.0.normalized_max_corrective_velocity = v; }
            /// Contact prediction distance, in fractions of ``length_unit``.
            #[getter] fn normalized_prediction_distance(&self) -> Real { self.0.normalized_prediction_distance }
            /// Set the normalized prediction distance.
            #[setter] fn set_normalized_prediction_distance(&mut self, v: Real) { self.0.normalized_prediction_distance = v; }
            /// Outer (TGS) solver iterations per step.
            #[getter] fn num_solver_iterations(&self) -> usize { self.0.num_solver_iterations }
            /// Set the outer solver iteration count.
            #[setter] fn set_num_solver_iterations(&mut self, v: usize) { self.0.num_solver_iterations = v; }
            /// Inner PGS iterations per outer solver iteration.
            #[getter] fn num_internal_pgs_iterations(&self) -> usize { self.0.num_internal_pgs_iterations }
            /// Set the inner PGS iteration count.
            #[setter] fn set_num_internal_pgs_iterations(&mut self, v: usize) { self.0.num_internal_pgs_iterations = v; }
            /// Inner stabilization iterations per outer solver iteration.
            #[getter] fn num_internal_stabilization_iterations(&self) -> usize { self.0.num_internal_stabilization_iterations }
            /// Set the inner stabilization iteration count.
            #[setter] fn set_num_internal_stabilization_iterations(&mut self, v: usize) { self.0.num_internal_stabilization_iterations = v; }
            /// Minimum number of bodies in an island before it is solved in
            /// parallel.
            #[getter] fn min_island_size(&self) -> usize { self.0.min_island_size }
            /// Set the minimum parallel-island size.
            #[setter] fn set_min_island_size(&mut self, v: usize) { self.0.min_island_size = v; }
            /// Maximum number of CCD substeps per simulation step.
            #[getter] fn max_ccd_substeps(&self) -> usize { self.0.max_ccd_substeps }
            /// Set the maximum CCD substep count.
            #[setter] fn set_max_ccd_substeps(&mut self, v: usize) { self.0.max_ccd_substeps = v; }

            /// Spring coefficients used to regularize contacts.
            #[getter]
            fn contact_softness(&self) -> SpringCoefficients {
                SpringCoefficients(self.0.contact_softness)
            }
            /// Set the contact-spring coefficients.
            #[setter]
            fn set_contact_softness(&mut self, v: SpringCoefficients) {
                self.0.contact_softness = v.0;
            }

            /// 3D friction model used by the contact solver.
            #[getter]
            fn friction_model(&self) -> FrictionModel {
                FrictionModel::from_rapier(self.0.friction_model)
            }
            /// Set the 3D friction model.
            #[setter]
            fn set_friction_model(&mut self, v: FrictionModel) {
                self.0.friction_model = v.to_rapier();
            }

            /// Reciprocal of ``dt`` (``1 / dt``).
            fn inv_dt(&self) -> Real { self.0.inv_dt() }
            /// Allowed penetration error in world units (denormalized).
            fn allowed_linear_error(&self) -> Real { self.0.allowed_linear_error() }
            /// Maximum penetration-correction velocity in world units
            /// (denormalized).
            fn max_corrective_velocity(&self) -> Real { self.0.max_corrective_velocity() }
            /// Contact prediction distance in world units (denormalized).
            fn prediction_distance(&self) -> Real { self.0.prediction_distance() }

            /// Stiffness component of the contact regularization spring.
            fn contact_spring_stiffness(&self) -> Real { self.0.contact_softness.natural_frequency }
            /// Damping component of the contact regularization spring.
            fn contact_spring_damping(&self) -> Real { self.0.contact_softness.damping_ratio }
            /// Default joint-spring stiffness (engine constant).
            fn joint_spring_stiffness(&self) -> Real {
                rapier::dynamics::SpringCoefficients::<Real>::joint_defaults().natural_frequency
            }
            /// Default joint-spring damping (engine constant).
            fn joint_spring_damping(&self) -> Real {
                rapier::dynamics::SpringCoefficients::<Real>::joint_defaults().damping_ratio
            }

            /// Equality comparison. Only ``==`` and ``!=`` are supported.
            ///
            /// :raises TypeError: when an ordering comparison is requested.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err("IntegrationParameters supports only == and !=")),
                }
            }
            fn __repr__(&self) -> String {
                format!("IntegrationParameters(dt={}, num_solver_iterations={})", self.0.dt, self.0.num_solver_iterations)
            }
        }

        $crate::__define_dynamics_rigid_body_3d!();
        $crate::__define_dynamics_rigid_body_set!();
    };
}

// ----------------------------------------------------------------------
// 2D-specific dynamics types
// ----------------------------------------------------------------------

#[doc(hidden)]
#[macro_export]
macro_rules! __define_dynamics_2d {
    () => {
        // ============================================================
        // MassProperties (2D)
        // ============================================================

        /// Mass, center of mass and scalar moment of inertia (2D).
        ///
        /// In 2D the inertia tensor reduces to a single scalar — the moment
        /// of inertia about the out-of-plane (``z``) axis through the center
        /// of mass.
        ///
        /// Use ``from_ball``, ``from_cuboid`` or ``from_capsule`` to derive
        /// values from a shape and density, or construct directly with
        /// ``MassProperties(local_com, mass, principal_inertia)``. Mass
        /// properties combine with ``+`` (and invert with ``-``).
        #[pyclass(name = "MassProperties", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct MassProperties(pub rapier::dynamics::MassProperties);

        #[pymethods]
        impl MassProperties {
            /// Build mass properties from explicit values.
            ///
            /// :param local_com: center of mass in body-local space (default
            ///     origin).
            /// :param mass: total mass (default ``0``).
            /// :param principal_inertia: scalar moment of inertia about the
            ///     center of mass (default ``0``).
            #[new]
            #[pyo3(signature = (local_com=None, mass=0.0 as Real, principal_inertia=0.0 as Real))]
            fn new(local_com: Option<PyPoint>, mass: Real, principal_inertia: Real) -> Self {
                let com = local_com
                    .map(|p| p.0.coords)
                    .unwrap_or_else($crate::na::Vector2::zeros);
                Self(rapier::dynamics::MassProperties::new(
                    com.into(),
                    mass,
                    principal_inertia,
                ))
            }

            /// Zero mass properties (mass = inertia = ``0``).
            #[staticmethod]
            fn zero() -> Self {
                Self(rapier::dynamics::MassProperties::default())
            }

            /// Mass properties of a uniform-density disk.
            ///
            /// :param density: mass per unit area.
            /// :param radius: disk radius.
            #[staticmethod]
            fn from_ball(density: Real, radius: Real) -> Self {
                Self(rapier::dynamics::MassProperties::from_ball(density, radius))
            }

            /// Mass properties of a uniform-density axis-aligned rectangle.
            ///
            /// :param density: mass per unit area.
            /// :param half_extents: half-extents along ``x`` and ``y``.
            #[staticmethod]
            fn from_cuboid(density: Real, half_extents: PyVector) -> Self {
                Self(rapier::dynamics::MassProperties::from_cuboid(
                    density,
                    half_extents.0.into(),
                ))
            }

            /// Mass properties of a uniform-density capsule between two
            /// endpoints.
            ///
            /// :param density: mass per unit area.
            /// :param a: capsule segment start (body-local).
            /// :param b: capsule segment end (body-local).
            /// :param radius: capsule radius.
            #[staticmethod]
            fn from_capsule(density: Real, a: PyPoint, b: PyPoint, radius: Real) -> Self {
                Self(rapier::dynamics::MassProperties::from_capsule(
                    density,
                    a.0.into(),
                    b.0.into(),
                    radius,
                ))
            }

            /// Total mass (read-only).
            #[getter]
            fn mass(&self) -> Real {
                self.0.mass()
            }
            /// Inverse mass — ``0`` for an infinite-mass body (read-only).
            #[getter]
            fn inv_mass(&self) -> Real {
                self.0.inv_mass
            }

            /// Center of mass in body-local space (read-only).
            #[getter]
            fn local_com(&self) -> Point2 {
                let v: $crate::na::Vector2<Real> = self.0.local_com.into();
                Point2($crate::na::Point2::from(v))
            }

            /// Scalar moment of inertia about the center of mass (read-only).
            #[getter]
            fn principal_inertia(&self) -> Real {
                self.0.principal_inertia()
            }

            /// Square root of the inverse moment of inertia (read-only).
            /// Used by the solver to scale angular impulses.
            #[getter]
            fn inv_principal_inertia_sqrt(&self) -> Real {
                self.0.inv_principal_inertia.sqrt()
            }

            /// Return a copy transformed by ``iso``.
            fn transform_by(&self, iso: PyIsometry) -> Self {
                let pose: rapier::math::Pose = iso.0.into();
                Self(self.0.transform_by(&pose))
            }

            /// Sum two mass distributions (parallel-axis theorem).
            fn __add__(&self, other: &MassProperties) -> Self {
                Self(self.0 + other.0)
            }
            /// Subtract ``other`` from ``self``; inverse of ``__add__``.
            fn __sub__(&self, other: &MassProperties) -> Self {
                Self(self.0 - other.0)
            }

            fn __repr__(&self) -> String {
                format!(
                    "MassProperties(mass={}, local_com=({}, {}))",
                    self.0.mass(),
                    self.0.local_com.x,
                    self.0.local_com.y,
                )
            }
        }

        // ============================================================
        // RigidBodyAdditionalMassProps (2D)
        // ============================================================

        /// Extra mass / mass properties to add to a body on top of what its
        /// colliders contribute.
        ///
        /// Useful when you want a body's collider geometry to provide most of
        /// the inertia but still bias the total mass.
        #[pyclass(name = "RigidBodyAdditionalMassProps", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyAdditionalMassProps(pub rapier::dynamics::RigidBodyAdditionalMassProps);

        #[pymethods]
        impl RigidBodyAdditionalMassProps {
            /// Add ``mass`` to the body without changing its center of mass
            /// or moment of inertia.
            #[staticmethod]
            fn from_mass(mass: Real) -> Self {
                Self(rapier::dynamics::RigidBodyAdditionalMassProps::Mass(mass))
            }
            /// Add the given full mass properties to the body.
            #[staticmethod]
            fn from_mass_properties(mp: &MassProperties) -> Self {
                Self(rapier::dynamics::RigidBodyAdditionalMassProps::MassProps(
                    mp.0,
                ))
            }
        }

        // ============================================================
        // RigidBody component views (2D)
        // ============================================================

        /// Linear and angular damping coefficients (per-second decay rates)
        /// applied to a body's velocity each step.
        #[pyclass(name = "RigidBodyDamping", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyDamping {
            /// Linear-velocity damping coefficient (read+write).
            #[pyo3(get, set)]
            pub linear_damping: Real,
            /// Angular-velocity damping coefficient (read+write).
            #[pyo3(get, set)]
            pub angular_damping: Real,
        }
        #[pymethods]
        impl RigidBodyDamping {
            /// Build damping coefficients (default zero / no damping).
            #[new]
            #[pyo3(signature = (linear_damping=0.0 as Real, angular_damping=0.0 as Real))]
            fn new(linear_damping: Real, angular_damping: Real) -> Self {
                Self {
                    linear_damping,
                    angular_damping,
                }
            }
        }

        /// Dominance group used to bias contact resolution between bodies.
        ///
        /// In a contact between bodies of different dominance, the
        /// higher-dominance body behaves as if it had infinite mass with
        /// respect to the lower-dominance one. Groups range in ``[-127, 127]``.
        #[pyclass(name = "RigidBodyDominance", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyDominance {
            /// Dominance group identifier (read+write).
            #[pyo3(get, set)]
            pub group: i8,
        }
        #[pymethods]
        impl RigidBodyDominance {
            /// Build a dominance descriptor (default group ``0``).
            #[new]
            #[pyo3(signature = (group=0))]
            fn new(group: i8) -> Self {
                Self { group }
            }
        }

        /// Per-body CCD configuration.
        ///
        /// ``ccd_enabled`` toggles substep CCD; ``soft_ccd_prediction`` is a
        /// distance over which "soft" CCD pushes the body back from imminent
        /// penetrations (``0`` disables soft CCD).
        #[pyclass(name = "RigidBodyCcd", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyCcd {
            /// Whether substep CCD is enabled for this body (read+write).
            #[pyo3(get, set)]
            pub ccd_enabled: bool,
            /// Soft-CCD prediction distance (read+write); ``0`` disables it.
            #[pyo3(get, set)]
            pub soft_ccd_prediction: Real,
        }
        #[pymethods]
        impl RigidBodyCcd {
            /// Build a CCD descriptor (defaults: disabled, ``0`` soft CCD).
            #[new]
            #[pyo3(signature = (ccd_enabled=false, soft_ccd_prediction=0.0 as Real))]
            fn new(ccd_enabled: bool, soft_ccd_prediction: Real) -> Self {
                Self {
                    ccd_enabled,
                    soft_ccd_prediction,
                }
            }
        }

        /// Linear and angular velocity of a rigid body (2D).
        ///
        /// In 2D the linear velocity is a 2-vector and the angular velocity
        /// is a scalar (rotation around the out-of-plane axis), in rad/s.
        #[pyclass(name = "RigidBodyVelocity", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyVelocity {
            /// Linear velocity in world space (read+write).
            #[pyo3(get, set)]
            pub linvel: Vec2,
            /// Angular velocity, in rad/s (read+write).
            #[pyo3(get, set)]
            pub angvel: Real,
        }
        #[pymethods]
        impl RigidBodyVelocity {
            /// Build a velocity (default ``linvel = 0``, ``angvel = 0``).
            #[new]
            #[pyo3(signature = (linvel=None, angvel=0.0 as Real))]
            fn new(linvel: Option<PyVector>, angvel: Real) -> Self {
                let lin = linvel
                    .map(|v| v.0)
                    .unwrap_or_else($crate::na::Vector2::zeros);
                Self {
                    linvel: Vec2(lin),
                    angvel,
                }
            }
        }

        /// Accumulated external forces, torques and gravity scaling (2D).
        ///
        /// ``force`` and ``torque`` accumulate over a single step and are
        /// cleared automatically before the next ``step()`` (unlike impulses,
        /// which act once instantaneously).
        #[pyclass(name = "RigidBodyForces", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyForces {
            /// Accumulated linear force, world space (read+write).
            #[pyo3(get, set)]
            pub force: Vec2,
            /// Accumulated torque (scalar in 2D) (read+write).
            #[pyo3(get, set)]
            pub torque: Real,
            /// Per-body multiplier on the world gravity (read+write).
            #[pyo3(get, set)]
            pub gravity_scale: Real,
        }

        /// Body-local mass properties view.
        ///
        /// Mirror of the ``MassProperties`` stored on a ``RigidBody``.
        #[pyclass(name = "RigidBodyMassProps", module = "rapier")]
        #[derive(Debug, Clone)]
        pub struct RigidBodyMassProps {
            /// Mass properties expressed in body-local space (read-only).
            #[pyo3(get)]
            pub local_mprops: MassProperties,
        }

        /// Current and predicted pose of a rigid body (2D).
        ///
        /// ``position`` is the pose used for queries; ``next_position`` is
        /// the pose that will be committed after integration.
        #[pyclass(name = "RigidBodyPosition", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct RigidBodyPosition {
            /// Current world-space pose (read+write).
            #[pyo3(get, set)]
            pub position: Isometry2,
            /// Predicted world-space pose after the next step (read+write).
            #[pyo3(get, set)]
            pub next_position: Isometry2,
        }

        // ============================================================
        // LockedAxes (2D)
        // ============================================================

        /// Bitflags selecting which translation / rotation degrees of freedom
        /// of a rigid body are locked (2D).
        ///
        /// Only ``x`` / ``y`` translations and the (single) out-of-plane
        /// rotation are meaningful in 2D. ``ROTATION_LOCKED`` locks the only
        /// rotation. Combine flags with ``|``, ``&``, ``^``, ``-``.
        ///
        /// Example::
        ///
        ///     # Pin the body in place along Y while still allowing it to
        ///     # rotate freely:
        ///     axes = LockedAxes.TRANSLATION_LOCKED_Y
        #[pyclass(name = "LockedAxes", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct LockedAxes(pub rapier::dynamics::LockedAxes);

        #[pymethods]
        impl LockedAxes {
            /// Build a ``LockedAxes`` value from a raw bitmask.
            ///
            /// :param bits: bitmask combining the ``LockedAxes`` class
            ///     attributes (default ``0`` = no axes locked).
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u8) -> Self {
                Self(rapier::dynamics::LockedAxes::from_bits_truncate(bits))
            }
            /// Empty flag set (no axes locked).
            #[staticmethod]
            fn empty() -> Self {
                Self(rapier::dynamics::LockedAxes::empty())
            }
            /// Flag set locking every available 2D axis.
            #[staticmethod]
            fn all_axes() -> Self {
                Self(rapier::dynamics::LockedAxes::all())
            }

            // 2D names: only X/Y translation and Z rotation are meaningful.
            // `ROTATION_LOCKED` is an alias for ROTATION_LOCKED_Z in 2D.
            /// Lock translation along the local ``x`` axis.
            #[classattr]
            const TRANSLATION_LOCKED_X: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_X);
            /// Lock translation along the local ``y`` axis.
            #[classattr]
            const TRANSLATION_LOCKED_Y: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_Y);
            /// Lock translation along both ``x`` and ``y``.
            #[classattr]
            const TRANSLATION_LOCKED: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::from_bits_truncate(
                    rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_X.bits()
                        | rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_Y.bits(),
                ));
            /// Lock the out-of-plane rotation.
            #[classattr]
            const ROTATION_LOCKED: LockedAxes =
                LockedAxes(rapier::dynamics::LockedAxes::ROTATION_LOCKED);
            /// Lock both translations and the rotation.
            #[classattr]
            const ALL: LockedAxes = LockedAxes(rapier::dynamics::LockedAxes::from_bits_truncate(
                rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_X.bits()
                    | rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_Y.bits()
                    | rapier::dynamics::LockedAxes::ROTATION_LOCKED.bits(),
            ));

            /// Raw bitmask backing this flag set (read-only).
            #[getter]
            fn bits(&self) -> u8 {
                self.0.bits()
            }
            /// Whether ``self`` is a superset of ``other``.
            fn contains(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// Whether no axes are locked.
            fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
            /// ``other in self`` — subset test (alias for ``contains``).
            fn __contains__(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// Union of two flag sets (``self | other``).
            fn __or__(&self, other: &Self) -> Self {
                Self(self.0 | other.0)
            }
            /// Intersection of two flag sets (``self & other``).
            fn __and__(&self, other: &Self) -> Self {
                Self(self.0 & other.0)
            }
            /// Symmetric difference (``self ^ other``).
            fn __xor__(&self, other: &Self) -> Self {
                Self(self.0 ^ other.0)
            }
            /// Complement of ``self`` within the full set of axes.
            fn __invert__(&self) -> Self {
                Self(!self.0)
            }
            /// Set difference (``self - other``).
            fn __sub__(&self, other: &Self) -> Self {
                Self(self.0 - other.0)
            }
            /// Truthiness: ``True`` if any axis is locked.
            fn __bool__(&self) -> bool {
                !self.0.is_empty()
            }
            /// Equality comparison. Only ``==`` and ``!=`` are supported.
            ///
            /// :raises TypeError: when an ordering comparison is requested.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err("LockedAxes supports only == and !=")),
                }
            }
            /// Hash equal to the underlying bitmask.
            fn __hash__(&self) -> u64 {
                self.0.bits() as u64
            }
            fn __repr__(&self) -> String {
                format!("LockedAxes(bits={:#010b})", self.0.bits())
            }
        }

        // ============================================================
        // IntegrationParameters (2D — no friction_model)
        // ============================================================

        /// Tunable parameters for the time-step and contact/joint solver
        /// (2D — friction is always pyramidal, so ``friction_model`` is not
        /// exposed).
        ///
        /// The defaults (``IntegrationParameters()``) are appropriate for
        /// most 2D games at ``dt ≈ 1/60`` seconds. Many fields are stored
        /// in fractions of ``length_unit``; use ``allowed_linear_error()``,
        /// ``max_corrective_velocity()`` and ``prediction_distance()`` to
        /// get the values in world units.
        #[pyclass(name = "IntegrationParameters", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct IntegrationParameters(pub rapier::dynamics::IntegrationParameters);

        #[pymethods]
        impl IntegrationParameters {
            /// Build the default integration parameters.
            #[new]
            fn new() -> Self {
                Self(rapier::dynamics::IntegrationParameters::default())
            }

            /// Return the default integration parameters.
            #[staticmethod]
            fn default_params() -> Self {
                Self(rapier::dynamics::IntegrationParameters::default())
            }

            /// Simulation timestep in seconds (default ``≈ 1/60``).
            #[getter]
            fn dt(&self) -> Real {
                self.0.dt
            }
            /// Set the simulation timestep in seconds.
            #[setter]
            fn set_dt(&mut self, v: Real) {
                self.0.dt = v;
            }
            /// Minimum substep length used by CCD, in seconds.
            #[getter]
            fn min_ccd_dt(&self) -> Real {
                self.0.min_ccd_dt
            }
            /// Set the minimum CCD substep length.
            #[setter]
            fn set_min_ccd_dt(&mut self, v: Real) {
                self.0.min_ccd_dt = v;
            }
            /// Warmstart coefficient in ``[0, 1]`` for impulse reuse between
            /// steps.
            #[getter]
            fn warmstart_coefficient(&self) -> Real {
                self.0.warmstart_coefficient
            }
            /// Set the warmstart coefficient.
            #[setter]
            fn set_warmstart_coefficient(&mut self, v: Real) {
                self.0.warmstart_coefficient = v;
            }
            /// Reference length unit (typically ``1.0``). Other "normalized"
            /// parameters are expressed in fractions of this.
            #[getter]
            fn length_unit(&self) -> Real {
                self.0.length_unit
            }
            /// Set the reference length unit.
            #[setter]
            fn set_length_unit(&mut self, v: Real) {
                self.0.length_unit = v;
            }
            /// Allowed penetration error, in fractions of ``length_unit``.
            #[getter]
            fn normalized_allowed_linear_error(&self) -> Real {
                self.0.normalized_allowed_linear_error
            }
            /// Set the normalized allowed linear error.
            #[setter]
            fn set_normalized_allowed_linear_error(&mut self, v: Real) {
                self.0.normalized_allowed_linear_error = v;
            }
            /// Maximum penetration-correction velocity, in fractions of
            /// ``length_unit`` per second.
            #[getter]
            fn normalized_max_corrective_velocity(&self) -> Real {
                self.0.normalized_max_corrective_velocity
            }
            /// Set the normalized maximum corrective velocity.
            #[setter]
            fn set_normalized_max_corrective_velocity(&mut self, v: Real) {
                self.0.normalized_max_corrective_velocity = v;
            }
            /// Contact prediction distance, in fractions of ``length_unit``.
            #[getter]
            fn normalized_prediction_distance(&self) -> Real {
                self.0.normalized_prediction_distance
            }
            /// Set the normalized prediction distance.
            #[setter]
            fn set_normalized_prediction_distance(&mut self, v: Real) {
                self.0.normalized_prediction_distance = v;
            }
            /// Outer (TGS) solver iterations per step.
            #[getter]
            fn num_solver_iterations(&self) -> usize {
                self.0.num_solver_iterations
            }
            /// Set the outer solver iteration count.
            #[setter]
            fn set_num_solver_iterations(&mut self, v: usize) {
                self.0.num_solver_iterations = v;
            }
            /// Inner PGS iterations per outer solver iteration.
            #[getter]
            fn num_internal_pgs_iterations(&self) -> usize {
                self.0.num_internal_pgs_iterations
            }
            /// Set the inner PGS iteration count.
            #[setter]
            fn set_num_internal_pgs_iterations(&mut self, v: usize) {
                self.0.num_internal_pgs_iterations = v;
            }
            /// Inner stabilization iterations per outer solver iteration.
            #[getter]
            fn num_internal_stabilization_iterations(&self) -> usize {
                self.0.num_internal_stabilization_iterations
            }
            /// Set the inner stabilization iteration count.
            #[setter]
            fn set_num_internal_stabilization_iterations(&mut self, v: usize) {
                self.0.num_internal_stabilization_iterations = v;
            }
            /// Minimum number of bodies in an island before it is solved in
            /// parallel.
            #[getter]
            fn min_island_size(&self) -> usize {
                self.0.min_island_size
            }
            /// Set the minimum parallel-island size.
            #[setter]
            fn set_min_island_size(&mut self, v: usize) {
                self.0.min_island_size = v;
            }
            /// Maximum number of CCD substeps per simulation step.
            #[getter]
            fn max_ccd_substeps(&self) -> usize {
                self.0.max_ccd_substeps
            }
            /// Set the maximum CCD substep count.
            #[setter]
            fn set_max_ccd_substeps(&mut self, v: usize) {
                self.0.max_ccd_substeps = v;
            }

            /// Spring coefficients used to regularize contacts.
            #[getter]
            fn contact_softness(&self) -> SpringCoefficients {
                SpringCoefficients(self.0.contact_softness)
            }
            /// Set the contact-spring coefficients.
            #[setter]
            fn set_contact_softness(&mut self, v: SpringCoefficients) {
                self.0.contact_softness = v.0;
            }

            /// Reciprocal of ``dt`` (``1 / dt``).
            fn inv_dt(&self) -> Real {
                self.0.inv_dt()
            }
            /// Allowed penetration error in world units (denormalized).
            fn allowed_linear_error(&self) -> Real {
                self.0.allowed_linear_error()
            }
            /// Maximum penetration-correction velocity in world units.
            fn max_corrective_velocity(&self) -> Real {
                self.0.max_corrective_velocity()
            }
            /// Contact prediction distance in world units (denormalized).
            fn prediction_distance(&self) -> Real {
                self.0.prediction_distance()
            }

            /// Stiffness component of the contact regularization spring.
            fn contact_spring_stiffness(&self) -> Real {
                self.0.contact_softness.natural_frequency
            }
            /// Damping component of the contact regularization spring.
            fn contact_spring_damping(&self) -> Real {
                self.0.contact_softness.damping_ratio
            }
            /// Default joint-spring stiffness (engine constant).
            fn joint_spring_stiffness(&self) -> Real {
                rapier::dynamics::SpringCoefficients::<Real>::joint_defaults().natural_frequency
            }
            /// Default joint-spring damping (engine constant).
            fn joint_spring_damping(&self) -> Real {
                rapier::dynamics::SpringCoefficients::<Real>::joint_defaults().damping_ratio
            }

            /// Equality comparison. Only ``==`` and ``!=`` are supported.
            ///
            /// :raises TypeError: when an ordering comparison is requested.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err(
                        "IntegrationParameters supports only == and !=",
                    )),
                }
            }
            fn __repr__(&self) -> String {
                format!(
                    "IntegrationParameters(dt={}, num_solver_iterations={})",
                    self.0.dt, self.0.num_solver_iterations
                )
            }
        }

        $crate::__define_dynamics_rigid_body_2d!();
        $crate::__define_dynamics_rigid_body_set!();
    };
}

// ----------------------------------------------------------------------
// RigidBody, RigidBodyBuilder — 3D
// ----------------------------------------------------------------------
#[doc(hidden)]
#[macro_export]
macro_rules! __define_dynamics_rigid_body_3d {
    () => {
        $crate::__define_dynamics_rigid_body_common!(3, Vec3, Point3, Rotation3, Isometry3);

        #[pymethods]
        impl RigidBody {
            // 3D-specific accessors (vector angvel)
            /// Angular velocity as a world-space 3-vector (read+write).
            #[getter]
            fn angvel(&self) -> Vec3 {
                let v: $crate::na::Vector3<Real> = self.with_ref(|b| b.angvel()).into();
                Vec3(v)
            }
            /// Set the angular velocity (wakes the body).
            #[setter]
            fn set_angvel(&mut self, v: PyVector) {
                let g: rapier::math::Vector = v.0.into();
                self.with_mut(|b| b.set_angvel(g.into(), true));
            }

            /// Torque accumulated by user calls during the current step
            /// (read-only). Cleared automatically at the next step.
            #[getter]
            fn user_torque(&self) -> Vec3 {
                let v: $crate::na::Vector3<Real> = self.with_ref(|b| b.user_torque()).into();
                Vec3(v)
            }

            /// Per-axis rotation enable mask as ``(rx, ry, rz)`` booleans
            /// (``True`` = free to rotate).
            #[getter]
            fn enabled_rotations(&self) -> (bool, bool, bool) {
                let arr = self.with_ref(|b| b.is_rotation_locked());
                (!arr[0], !arr[1], !arr[2])
            }
            /// Enable / disable rotation around each axis.
            ///
            /// :param v: tuple ``(rx, ry, rz)`` of booleans where ``True``
            ///     allows rotation and ``False`` locks it.
            #[setter]
            fn set_enabled_rotations(&mut self, v: (bool, bool, bool)) {
                self.with_mut(|b| b.set_enabled_rotations(v.0, v.1, v.2, true));
            }

            /// Per-axis translation enable mask as ``(tx, ty, tz)`` booleans
            /// (``True`` = free to translate).
            #[getter]
            fn enabled_translations(&self) -> (bool, bool, bool) {
                let la = self.with_ref(|b| b.locked_axes());
                (
                    !la.contains(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_X),
                    !la.contains(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_Y),
                    !la.contains(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_Z),
                )
            }
            /// Enable / disable translation along each axis.
            ///
            /// :param v: tuple ``(tx, ty, tz)`` of booleans.
            #[setter]
            fn set_enabled_translations(&mut self, v: (bool, bool, bool)) {
                self.with_mut(|b| b.set_enabled_translations(v.0, v.1, v.2, true));
            }

            /// Accumulate a torque to be applied during the next step.
            ///
            /// The torque is cleared after the step (like ``add_force``);
            /// use ``apply_torque_impulse`` for an instantaneous change of
            /// angular velocity.
            ///
            /// :param torque: torque vector in world space.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (torque, wake_up=true))]
            fn add_torque(&mut self, torque: PyVector, wake_up: bool) {
                let g: rapier::math::Vector = torque.0.into();
                self.with_mut(|b| b.add_torque(g.into(), wake_up));
            }

            /// Apply an instantaneous angular impulse (units: ``N·m·s``).
            ///
            /// :param torque_impulse: torque impulse vector in world space.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (torque_impulse, wake_up=true))]
            fn apply_torque_impulse(&mut self, torque_impulse: PyVector, wake_up: bool) {
                let g: rapier::math::Vector = torque_impulse.0.into();
                self.with_mut(|b| b.apply_torque_impulse(g.into(), wake_up));
            }

            /// Whether gyroscopic forces are accounted for during integration
            /// (read+write). Useful for asymmetric inertia tensors (e.g.
            /// spinning tops).
            #[getter]
            fn gyroscopic_forces_enabled(&self) -> bool {
                self.with_ref(|b| b.gyroscopic_forces_enabled())
            }
            /// Enable or disable gyroscopic forces.
            #[setter]
            fn set_gyroscopic_forces_enabled(&mut self, v: bool) {
                self.with_mut(|b| b.enable_gyroscopic_forces(v));
            }

            /// Predict the angular velocity after ``dt`` seconds *with*
            /// gyroscopic forces taken into account.
            ///
            /// The body is not modified.
            ///
            /// :param dt: integration timestep in seconds.
            fn angvel_with_gyroscopic_forces(&self, dt: Real) -> Vec3 {
                let v: $crate::na::Vector3<Real> = self
                    .with_ref(|b| b.angvel_with_gyroscopic_forces(dt))
                    .into();
                Vec3(v)
            }

            // 3D-only builder static methods
            /// Start building a dynamic rigid body. Returns a ``RigidBodyBuilder``.
            ///
            /// Accepts optional kwargs configuring builder fields
            /// (``translation``, ``rotation``, ``linvel``, ``angvel``,
            /// ``locked_axes``, ``ccd_enabled``, ``gravity_scale``, ...).
            #[staticmethod]
            #[pyo3(signature = (**kwargs))]
            fn dynamic(
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(rapier::dynamics::RigidBodyBuilder::dynamic(), kwargs)
            }
            /// Start building a fixed (immovable) rigid body.
            ///
            /// Same kwargs as ``dynamic``.
            #[staticmethod]
            #[pyo3(signature = (**kwargs))]
            fn fixed(
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(rapier::dynamics::RigidBodyBuilder::fixed(), kwargs)
            }
            /// Start building a velocity-based kinematic rigid body.
            #[staticmethod]
            #[pyo3(signature = (**kwargs))]
            fn kinematic_velocity_based(
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(
                    rapier::dynamics::RigidBodyBuilder::kinematic_velocity_based(),
                    kwargs,
                )
            }
            /// Start building a position-based kinematic rigid body.
            #[staticmethod]
            #[pyo3(signature = (**kwargs))]
            fn kinematic_position_based(
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(
                    rapier::dynamics::RigidBodyBuilder::kinematic_position_based(),
                    kwargs,
                )
            }

            /// Start building a body of the given ``body_type``.
            ///
            /// :param body_type: a ``RigidBodyType`` selecting the behavior
            ///     class.
            #[staticmethod]
            #[pyo3(signature = (body_type, **kwargs))]
            fn new_body(
                body_type: RigidBodyType,
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(
                    rapier::dynamics::RigidBodyBuilder::new(body_type.to_rapier()),
                    kwargs,
                )
            }
        }

        impl RigidBodyBuilder {
            fn apply_angvel_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                let pv: PyVector = v.extract()?;
                let g: rapier::math::Vector = pv.0.into();
                *builder = builder.clone().angvel(g.into());
                Ok(())
            }

            fn apply_rotation_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                // 3D rotation kwarg is a scaled-axis-angle vector.
                let pv: PyVector = v.extract()?;
                let g: rapier::math::Vector = pv.0.into();
                *builder = builder.clone().rotation(g.into());
                Ok(())
            }

            fn apply_enabled_translations_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                let t: (bool, bool, bool) = v.extract()?;
                *builder = builder.clone().enabled_translations(t.0, t.1, t.2);
                Ok(())
            }

            fn apply_enabled_rotations_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                let t: (bool, bool, bool) = v.extract()?;
                *builder = builder.clone().enabled_rotations(t.0, t.1, t.2);
                Ok(())
            }

            fn apply_gyroscopic_forces_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                let b: bool = v.extract()?;
                *builder = builder.clone().gyroscopic_forces_enabled(b);
                Ok(())
            }
        }
    };
}

// ----------------------------------------------------------------------
// RigidBody, RigidBodyBuilder — 2D
// ----------------------------------------------------------------------
#[doc(hidden)]
#[macro_export]
macro_rules! __define_dynamics_rigid_body_2d {
    () => {
        $crate::__define_dynamics_rigid_body_common!(2, Vec2, Point2, Rotation2, Isometry2);

        #[pymethods]
        impl RigidBody {
            /// Angular velocity around the out-of-plane axis, in rad/s
            /// (read+write).
            #[getter]
            fn angvel(&self) -> Real {
                self.with_ref(|b| b.angvel())
            }
            /// Set the angular velocity (wakes the body).
            #[setter]
            fn set_angvel(&mut self, v: Real) {
                self.with_mut(|b| b.set_angvel(v, true));
            }

            /// Torque accumulated by user calls during the current step
            /// (read-only). Cleared automatically at the next step.
            #[getter]
            fn user_torque(&self) -> Real {
                self.with_ref(|b| b.user_torque())
            }

            /// Whether rotation is enabled, as a single-element tuple
            /// ``(rz,)`` for API symmetry with 3D (``True`` = free to rotate).
            #[getter]
            fn enabled_rotations(&self) -> (bool,) {
                (!self.with_ref(|b| b.is_rotation_locked()),)
            }
            /// Enable or disable rotation. Pass ``(True,)`` to allow rotation.
            #[setter]
            fn set_enabled_rotations(&mut self, v: (bool,)) {
                self.with_mut(|b| b.lock_rotations(!v.0, true));
            }

            /// Per-axis translation enable mask as ``(tx, ty)`` booleans
            /// (``True`` = free to translate).
            #[getter]
            fn enabled_translations(&self) -> (bool, bool) {
                let la = self.with_ref(|b| b.locked_axes());
                (
                    !la.contains(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_X),
                    !la.contains(rapier::dynamics::LockedAxes::TRANSLATION_LOCKED_Y),
                )
            }
            /// Enable / disable translation along ``x`` and ``y``.
            #[setter]
            fn set_enabled_translations(&mut self, v: (bool, bool)) {
                self.with_mut(|b| b.set_enabled_translations(v.0, v.1, true));
            }

            /// Accumulate a (scalar) torque to be applied during the next
            /// step. Cleared after the step.
            ///
            /// :param torque: torque magnitude.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (torque, wake_up=true))]
            fn add_torque(&mut self, torque: Real, wake_up: bool) {
                self.with_mut(|b| b.add_torque(torque, wake_up));
            }

            /// Apply an instantaneous (scalar) angular impulse.
            ///
            /// :param torque_impulse: angular impulse magnitude.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (torque_impulse, wake_up=true))]
            fn apply_torque_impulse(&mut self, torque_impulse: Real, wake_up: bool) {
                self.with_mut(|b| b.apply_torque_impulse(torque_impulse, wake_up));
            }

            /// Start building a dynamic rigid body.
            ///
            /// Accepts optional kwargs configuring builder fields
            /// (``translation``, ``rotation``, ``linvel``, ``angvel``,
            /// ``locked_axes``, ``ccd_enabled``, ``gravity_scale``, ...).
            #[staticmethod]
            #[pyo3(signature = (**kwargs))]
            fn dynamic(
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(rapier::dynamics::RigidBodyBuilder::dynamic(), kwargs)
            }
            /// Start building a fixed (immovable) rigid body.
            #[staticmethod]
            #[pyo3(signature = (**kwargs))]
            fn fixed(
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(rapier::dynamics::RigidBodyBuilder::fixed(), kwargs)
            }
            /// Start building a velocity-based kinematic rigid body.
            #[staticmethod]
            #[pyo3(signature = (**kwargs))]
            fn kinematic_velocity_based(
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(
                    rapier::dynamics::RigidBodyBuilder::kinematic_velocity_based(),
                    kwargs,
                )
            }
            /// Start building a position-based kinematic rigid body.
            #[staticmethod]
            #[pyo3(signature = (**kwargs))]
            fn kinematic_position_based(
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(
                    rapier::dynamics::RigidBodyBuilder::kinematic_position_based(),
                    kwargs,
                )
            }

            /// Start building a body of the given ``body_type``.
            #[staticmethod]
            #[pyo3(signature = (body_type, **kwargs))]
            fn new_body(
                body_type: RigidBodyType,
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<RigidBodyBuilder> {
                RigidBodyBuilder::from_kwargs(
                    rapier::dynamics::RigidBodyBuilder::new(body_type.to_rapier()),
                    kwargs,
                )
            }
        }

        impl RigidBodyBuilder {
            fn apply_angvel_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                let a: Real = v.extract()?;
                *builder = builder.clone().angvel(a);
                Ok(())
            }

            fn apply_rotation_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                // 2D rotation kwarg is a scalar angle.
                let a: Real = v.extract()?;
                *builder = builder.clone().rotation(a);
                Ok(())
            }

            fn apply_enabled_translations_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                let t: (bool, bool) = v.extract()?;
                *builder = builder.clone().enabled_translations(t.0, t.1);
                Ok(())
            }

            fn apply_enabled_rotations_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                let t: (bool,) = v.extract()?;
                if t.0 {
                    *builder = builder.clone().locked_axes(
                        builder.clone().build().locked_axes()
                            & !rapier::dynamics::LockedAxes::ROTATION_LOCKED,
                    );
                } else {
                    *builder = builder.clone().locked_axes(
                        builder.clone().build().locked_axes()
                            | rapier::dynamics::LockedAxes::ROTATION_LOCKED,
                    );
                }
                Ok(())
            }

            #[allow(unused_variables)]
            fn apply_gyroscopic_forces_kwarg(
                builder: &mut rapier::dynamics::RigidBodyBuilder,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                // No gyroscopic forces in 2D — silently ignore.
                Ok(())
            }
        }
    };
}

// ----------------------------------------------------------------------
// RigidBody (dim-agnostic methods) + RigidBodyBuilder
// ----------------------------------------------------------------------
#[doc(hidden)]
#[macro_export]
macro_rules! __define_dynamics_rigid_body_common {
    ($dim:literal, $Vec:ident, $Point:ident, $Rot:ident, $Iso:ident) => {
        // ============================================================
        // RigidBody — owned standalone (NOT a view, holds a full body).
        // ============================================================
        // When inserted into a RigidBodySet, the set takes ownership and
        // we return handles. When obtained via `set.get(handle).clone()`
        // we hand back a fresh RigidBody that mirrors the stored one.

        /// A simulated rigid body.
        ///
        /// Holds pose, velocity, accumulated forces, mass properties and
        /// solver flags. To create one, use a factory on this class to obtain
        /// a ``RigidBodyBuilder``, configure it and ``build()`` (or insert
        /// the builder directly into a ``RigidBodySet``)::
        ///
        ///     rb = RigidBody.dynamic(translation=Vec3(0, 5, 0)).build()
        ///     handle = bodies.insert(rb)
        ///
        /// A body obtained from a ``RigidBodySet`` (e.g. ``world.rigid_bodies[h]``)
        /// is a lightweight **view** into the stored body: reads and writes go
        /// straight through to the set with no copy, so ``body.linvel = v``
        /// persists immediately. A body created via ``build()`` (not yet in a
        /// set) owns its data until inserted.
        #[pyclass(name = "RigidBody", module = "rapier")]
        #[derive(Debug)]
        pub struct RigidBody {
            pub backing: RigidBodyBacking,
        }

        /// Storage backing a `RigidBody`: either a standalone owned body
        /// (pre-insertion) or a handle-backed view into a `RigidBodySet`.
        #[derive(Debug)]
        pub enum RigidBodyBacking {
            Owned(Box<rapier::dynamics::RigidBody>),
            InSet {
                set: Py<RigidBodySet>,
                handle: rapier::dynamics::RigidBodyHandle,
            },
        }

        // `Py<T>` is not `derive`-Clone (cloning bumps the refcount under the
        // GIL), so implement Clone by hand.
        impl Clone for RigidBodyBacking {
            fn clone(&self) -> Self {
                match self {
                    RigidBodyBacking::Owned(b) => RigidBodyBacking::Owned(b.clone()),
                    RigidBodyBacking::InSet { set, handle } => {
                        Python::with_gil(|py| RigidBodyBacking::InSet {
                            set: set.clone_ref(py),
                            handle: *handle,
                        })
                    }
                }
            }
        }
        impl Clone for RigidBody {
            fn clone(&self) -> Self {
                RigidBody {
                    backing: self.backing.clone(),
                }
            }
        }

        impl RigidBody {
            fn new_owned(body: rapier::dynamics::RigidBody) -> Self {
                RigidBody {
                    backing: RigidBodyBacking::Owned(Box::new(body)),
                }
            }

            /// Run `f` with a shared reference to the underlying body. For an
            /// `InSet` view this briefly borrows the set; a stale handle (body
            /// already removed) panics, surfacing as a Python exception.
            fn with_ref<R>(&self, f: impl FnOnce(&rapier::dynamics::RigidBody) -> R) -> R {
                match &self.backing {
                    RigidBodyBacking::Owned(b) => f(b),
                    RigidBodyBacking::InSet { set, handle } => Python::with_gil(|py| {
                        let set = set.bind(py).borrow();
                        let body = set
                            .0
                            .get(*handle)
                            .expect("RigidBody refers to a body that was removed from its set");
                        f(body)
                    }),
                }
            }

            /// Run `f` with a mutable reference to the underlying body, writing
            /// straight through to the set for an `InSet` view.
            fn with_mut<R>(&mut self, f: impl FnOnce(&mut rapier::dynamics::RigidBody) -> R) -> R {
                match &mut self.backing {
                    RigidBodyBacking::Owned(b) => f(b),
                    RigidBodyBacking::InSet { set, handle } => Python::with_gil(|py| {
                        let mut set = set.bind(py).borrow_mut();
                        let body = set
                            .0
                            .get_mut(*handle)
                            .expect("RigidBody refers to a body that was removed from its set");
                        f(body)
                    }),
                }
            }

            /// Clone the underlying body out (used by `insert` and by callers
            /// that need an owned `&rapier::RigidBody`).
            pub fn to_owned_body(&self) -> rapier::dynamics::RigidBody {
                self.with_ref(|b| b.clone())
            }
        }

        #[pymethods]
        impl RigidBody {
            // No `__new__`: construction goes through builders.

            /// World-space pose of the body (read+write).
            #[getter]
            fn position(&self) -> $Iso {
                self.with_ref(|b| {
                    let r = b.position();
                    let na_iso: $crate::na::Isometry<Real, _, $dim> = (*r).into();
                    $Iso(na_iso)
                })
            }
            /// Teleport the body to ``p`` (wakes it). Use sparingly on
            /// dynamic bodies as this bypasses the integrator.
            #[setter]
            fn set_position(&mut self, p: PyIsometry) {
                let g: rapier::math::Pose = p.0.into();
                self.with_mut(|b| b.set_position(g, true));
            }

            /// Translation portion of the pose (read+write).
            #[getter]
            fn translation(&self) -> $Vec {
                self.with_ref(|b| {
                    let v: $crate::na::SVector<Real, $dim> = b.translation().into();
                    $Vec(v)
                })
            }
            /// Set the translation, leaving the rotation unchanged.
            #[setter]
            fn set_translation(&mut self, v: PyVector) {
                let g: rapier::math::Vector = v.0.into();
                self.with_mut(|b| b.set_translation(g, true));
            }

            /// Rotation portion of the pose (read+write).
            #[getter]
            fn rotation(&self) -> $Rot {
                self.with_ref(|b| {
                    let r = *b.rotation();
                    $Rot(r.into())
                })
            }
            /// Set the rotation, leaving the translation unchanged.
            #[setter]
            fn set_rotation(&mut self, r: PyRotation) {
                let g: rapier::math::Rotation = r.0.into();
                self.with_mut(|b| b.set_rotation(g, true));
            }

            /// Linear velocity in world space (read+write).
            #[getter]
            fn linvel(&self) -> $Vec {
                self.with_ref(|b| {
                    let v: $crate::na::SVector<Real, $dim> = b.linvel().into();
                    $Vec(v)
                })
            }
            /// Set the linear velocity (wakes the body).
            #[setter]
            fn set_linvel(&mut self, v: PyVector) {
                let g: rapier::math::Vector = v.0.into();
                self.with_mut(|b| b.set_linvel(g, true));
            }

            /// Total mass of the body (colliders + additional) (read-only).
            #[getter]
            fn mass(&self) -> Real {
                self.with_ref(|b| b.mass())
            }
            /// Inverse mass — ``0`` for fixed or infinite-mass bodies
            /// (read-only).
            #[getter]
            fn inv_mass(&self) -> Real {
                self.with_ref(|b| b.mass_properties().local_mprops.inv_mass)
            }

            /// Center of mass in world space (read-only).
            #[getter]
            fn center_of_mass(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> =
                    self.with_ref(|b| b.center_of_mass()).into();
                $Point($crate::na::Point::from(v))
            }

            /// Center of mass in body-local space (read-only).
            #[getter]
            fn local_center_of_mass(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> =
                    self.with_ref(|b| b.local_center_of_mass()).into();
                $Point($crate::na::Point::from(v))
            }

            /// Body-local mass properties (read-only).
            ///
            /// To change them, use ``set_additional_mass``,
            /// ``set_additional_mass_properties`` or
            /// ``recompute_mass_properties_from_colliders``.
            #[getter]
            fn mass_properties(&self) -> MassProperties {
                self.with_ref(|b| MassProperties(b.mass_properties().local_mprops))
            }

            /// Behavior class of the body (dynamic/fixed/kinematic)
            /// (read+write).
            #[getter]
            fn body_type(&self) -> RigidBodyType {
                RigidBodyType::from_rapier(self.with_ref(|b| b.body_type()))
            }
            /// Change the body's behavior class.
            #[setter]
            fn set_body_type(&mut self, t: RigidBodyType) {
                let rt = t.to_rapier();
                self.with_mut(|b| b.set_body_type(rt, true));
            }

            /// Multiplier applied to the world gravity for this body
            /// (read+write). ``0`` disables gravity for the body.
            #[getter]
            fn gravity_scale(&self) -> Real {
                self.with_ref(|b| b.gravity_scale())
            }
            /// Set the gravity scale.
            #[setter]
            fn set_gravity_scale(&mut self, v: Real) {
                self.with_mut(|b| b.set_gravity_scale(v, true));
            }

            /// Per-second linear-velocity damping coefficient (read+write).
            #[getter]
            fn linear_damping(&self) -> Real {
                self.with_ref(|b| b.linear_damping())
            }
            /// Set the linear damping coefficient.
            #[setter]
            fn set_linear_damping(&mut self, v: Real) {
                self.with_mut(|b| b.set_linear_damping(v));
            }

            /// Per-second angular-velocity damping coefficient (read+write).
            #[getter]
            fn angular_damping(&self) -> Real {
                self.with_ref(|b| b.angular_damping())
            }
            /// Set the angular damping coefficient.
            #[setter]
            fn set_angular_damping(&mut self, v: Real) {
                self.with_mut(|b| b.set_angular_damping(v));
            }

            /// Dominance group of the body (read+write); see
            /// ``RigidBodyDominance``.
            #[getter]
            fn dominance_group(&self) -> i8 {
                self.with_ref(|b| b.dominance_group())
            }
            /// Set the dominance group (``[-127, 127]``).
            #[setter]
            fn set_dominance_group(&mut self, v: i8) {
                self.with_mut(|b| b.set_dominance_group(v));
            }

            /// Extra solver iterations spent on this body's island
            /// (read+write). ``0`` keeps the global default.
            #[getter]
            fn additional_solver_iterations(&self) -> usize {
                self.with_ref(|b| b.additional_solver_iterations())
            }
            /// Set the number of additional solver iterations.
            #[setter]
            fn set_additional_solver_iterations(&mut self, v: usize) {
                self.with_mut(|b| b.set_additional_solver_iterations(v));
            }

            /// Currently locked degrees of freedom (read+write).
            #[getter]
            fn locked_axes(&self) -> LockedAxes {
                LockedAxes(self.with_ref(|b| b.locked_axes()))
            }
            /// Replace the locked axes flag set (wakes the body).
            #[setter]
            fn set_locked_axes(&mut self, v: LockedAxes) {
                self.with_mut(|b| b.set_locked_axes(v.0, true));
            }

            /// Whether the body is currently sleeping (read-only).
            #[getter]
            fn is_sleeping(&self) -> bool {
                self.with_ref(|b| b.is_sleeping())
            }
            /// Whether the body is currently moving (read-only).
            #[getter]
            fn is_moving(&self) -> bool {
                self.with_ref(|b| b.is_moving())
            }
            /// Whether the body is dynamic (read-only).
            #[getter]
            fn is_dynamic(&self) -> bool {
                self.with_ref(|b| b.is_dynamic())
            }
            /// Whether the body is kinematic (either variant) (read-only).
            #[getter]
            fn is_kinematic(&self) -> bool {
                self.with_ref(|b| b.is_kinematic())
            }
            /// Whether the body is fixed (read-only).
            #[getter]
            fn is_fixed(&self) -> bool {
                self.with_ref(|b| b.is_fixed())
            }

            /// Whether the body participates in simulation (read+write).
            ///
            /// A disabled body is fully skipped by the solver and queries.
            #[getter]
            fn is_enabled(&self) -> bool {
                self.with_ref(|b| b.is_enabled())
            }
            /// Enable or disable the body.
            #[setter]
            fn set_is_enabled(&mut self, v: bool) {
                self.with_mut(|b| b.set_enabled(v));
            }

            /// Whether substep CCD is enabled for this body (read+write).
            #[getter]
            fn ccd_enabled(&self) -> bool {
                self.with_ref(|b| b.is_ccd_enabled())
            }
            /// Enable or disable substep CCD for this body.
            #[setter]
            fn set_ccd_enabled(&mut self, v: bool) {
                self.with_mut(|b| b.enable_ccd(v));
            }

            /// Soft-CCD prediction distance (read+write); ``0`` disables it.
            #[getter]
            fn soft_ccd_prediction(&self) -> Real {
                self.with_ref(|b| b.soft_ccd_prediction())
            }
            /// Set the soft-CCD prediction distance.
            #[setter]
            fn set_soft_ccd_prediction(&mut self, v: Real) {
                self.with_mut(|b| b.set_soft_ccd_prediction(v));
            }

            /// Application-defined ``u128`` payload attached to this body
            /// (read+write).
            #[getter]
            fn user_data(&self) -> u128 {
                self.with_ref(|b| b.user_data)
            }
            /// Set the application-defined payload.
            #[setter]
            fn set_user_data(&mut self, v: u128) {
                self.with_mut(|b| b.user_data = v);
            }

            /// Force accumulated by user calls during the current step
            /// (read-only). Cleared automatically before the next step.
            #[getter]
            fn user_force(&self) -> $Vec {
                let v: $crate::na::SVector<Real, $dim> = self.with_ref(|b| b.user_force()).into();
                $Vec(v)
            }

            /// Handles of all colliders attached to this body (read-only).
            #[getter]
            fn colliders(&self) -> Vec<ColliderHandle> {
                self.with_ref(|b| b.colliders().iter().copied().map(ColliderHandle).collect())
            }

            /// Activation / sleep state of this body (read+write).
            #[getter]
            fn activation(&self) -> RigidBodyActivation {
                self.with_ref(|b| RigidBodyActivation(*b.activation()))
            }
            /// Replace the activation state.
            #[setter]
            fn set_activation(&mut self, v: RigidBodyActivation) {
                self.with_mut(|b| *b.activation_mut() = v.0);
            }

            /// Predicted world-space pose after the next step (read-only).
            #[getter]
            fn next_position(&self) -> $Iso {
                self.with_ref(|b| {
                    let p: $crate::na::Isometry<Real, _, $dim> = (*b.next_position()).into();
                    $Iso(p)
                })
            }

            // ---- mutating methods ----

            /// Accumulate a force to be applied during the next step.
            ///
            /// Forces accumulate over the step and are cleared automatically
            /// before the next ``step()``. Use ``apply_impulse`` for an
            /// instantaneous velocity change.
            ///
            /// :param force: force vector in world space.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (force, wake_up=true))]
            fn add_force(&mut self, force: PyVector, wake_up: bool) {
                let g: rapier::math::Vector = force.0.into();
                self.with_mut(|b| b.add_force(g, wake_up));
            }

            /// Accumulate a force applied at a given world-space point.
            ///
            /// Equivalent to applying ``force`` at the body's center of mass
            /// plus a torque ``(point - com) × force``.
            ///
            /// :param force: force vector in world space.
            /// :param point: application point in world space.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (force, point, wake_up=true))]
            fn add_force_at_point(&mut self, force: PyVector, point: PyPoint, wake_up: bool) {
                let f: rapier::math::Vector = force.0.into();
                let p: rapier::math::Vector = point.0.coords.into();
                self.with_mut(|b| b.add_force_at_point(f, p, wake_up));
            }

            /// Apply an instantaneous linear impulse (units: ``N·s``).
            ///
            /// :param impulse: impulse vector in world space.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (impulse, wake_up=true))]
            fn apply_impulse(&mut self, impulse: PyVector, wake_up: bool) {
                let g: rapier::math::Vector = impulse.0.into();
                self.with_mut(|b| b.apply_impulse(g, wake_up));
            }

            /// Apply an instantaneous impulse at a given world-space point.
            ///
            /// :param impulse: impulse vector in world space.
            /// :param point: application point in world space.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (impulse, point, wake_up=true))]
            fn apply_impulse_at_point(&mut self, impulse: PyVector, point: PyPoint, wake_up: bool) {
                let imp: rapier::math::Vector = impulse.0.into();
                let p: rapier::math::Vector = point.0.coords.into();
                self.with_mut(|b| b.apply_impulse_at_point(imp, p, wake_up));
            }

            /// Clear any accumulated forces on this body.
            ///
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (wake_up=true))]
            fn reset_forces(&mut self, wake_up: bool) {
                self.with_mut(|b| b.reset_forces(wake_up));
            }
            /// Clear any accumulated torques on this body.
            ///
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (wake_up=true))]
            fn reset_torques(&mut self, wake_up: bool) {
                self.with_mut(|b| b.reset_torques(wake_up));
            }

            /// Wake the body, allowing it to participate in the next step.
            ///
            /// :param strong: if ``True`` (default), reset the sleep timer
            ///     so the body stays awake longer before re-sleeping.
            #[pyo3(signature = (strong=true))]
            fn wake_up(&mut self, strong: bool) {
                self.with_mut(|b| b.wake_up(strong));
            }
            /// Put the body to sleep immediately.
            fn sleep(&mut self) {
                self.with_mut(|b| b.sleep());
            }

            /// World-space velocity at the given world-space point.
            ///
            /// Includes both translational and rotational contributions:
            /// ``v_point = linvel + angvel × (point - com)``.
            ///
            /// :param point: world-space point.
            fn velocity_at_point(&self, point: PyPoint) -> $Vec {
                let p: rapier::math::Vector = point.0.coords.into();
                let v: $crate::na::SVector<Real, $dim> =
                    self.with_ref(|b| b.velocity_at_point(p)).into();
                $Vec(v)
            }

            /// Kinetic energy of the body.
            ///
            /// :returns: :math:`E_k = \\tfrac{1}{2} m \\|v\\|^2 +
            ///     \\tfrac{1}{2} \\omega^\\top I \\omega`.
            fn kinetic_energy(&self) -> Real {
                self.with_ref(|b| b.kinetic_energy())
            }

            /// Gravitational potential energy after integrating for ``dt``
            /// seconds under ``gravity``.
            ///
            /// :returns: :math:`-m\\, \\mathbf{g} \\cdot \\mathbf{p}` where
            ///     :math:`\\mathbf{p}` is the predicted position.
            /// :param dt: integration step in seconds.
            /// :param gravity: gravity vector in world space.
            fn gravitational_potential_energy(&self, dt: Real, gravity: PyVector) -> Real {
                let g: rapier::math::Vector = gravity.0.into();
                self.with_ref(|b| b.gravitational_potential_energy(dt, g))
            }

            /// Predict the body's pose after ``dt`` seconds using its
            /// *current velocity only* (no forces, no integration). The body
            /// is not modified.
            ///
            /// :param dt: prediction horizon in seconds.
            fn predict_position_using_velocity(&self, dt: Real) -> $Iso {
                let p: $crate::na::Isometry<Real, _, $dim> = self
                    .with_ref(|b| b.predict_position_using_velocity(dt))
                    .into();
                $Iso(p)
            }

            /// Predict the body's pose after ``dt`` seconds using its
            /// current velocity *and* accumulated forces, without actually
            /// integrating (the body is not modified).
            ///
            /// :param dt: prediction horizon in seconds.
            fn predict_position_using_velocity_and_forces(&self, dt: Real) -> $Iso {
                self.with_ref(|b| {
                    let p: $crate::na::Isometry<Real, _, $dim> =
                        b.predict_position_using_velocity_and_forces(dt).into();
                    $Iso(p)
                })
            }

            /// Overwrite this body's mass properties from the attached
            /// colliders.
            ///
            /// Equivalent to summing the mass properties of every collider
            /// in ``colliders`` that is attached to this body. Useful after
            /// adding, removing or rescaling colliders.
            ///
            /// :param colliders: the ``ColliderSet`` holding this body's
            ///     colliders.
            fn recompute_mass_properties_from_colliders(&mut self, colliders: &ColliderSet) {
                self.with_mut(|b| b.recompute_mass_properties_from_colliders(&colliders.0));
            }

            /// Add ``mass`` on top of the colliders' contribution.
            ///
            /// :param mass: extra mass to add.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (mass, wake_up=true))]
            fn set_additional_mass(&mut self, mass: Real, wake_up: bool) {
                self.with_mut(|b| b.set_additional_mass(mass, wake_up));
            }

            /// Add full mass properties on top of the colliders'
            /// contribution.
            ///
            /// :param mp: extra mass properties.
            /// :param wake_up: if ``True`` (default), wake the body up.
            #[pyo3(signature = (mp, wake_up=true))]
            fn set_additional_mass_properties(&mut self, mp: &MassProperties, wake_up: bool) {
                self.with_mut(|b| b.set_additional_mass_properties(mp.0, wake_up));
            }

            /// Convenience for accumulating ``gravity * mass`` as a force
            /// during the current step (useful when stepping with gravity
            /// disabled globally).
            ///
            /// :param gravity: gravity acceleration vector.
            fn add_gravitational_force(&mut self, gravity: PyVector) {
                // Add gravity * mass as a force.
                let g: rapier::math::Vector = gravity.0.into();
                let m = self.with_ref(|b| b.mass());
                self.with_mut(|b| b.add_force(g * m, true));
            }

            fn __repr__(&self) -> String {
                let p = self.with_ref(|b| b.translation());
                format!(
                    "RigidBody(translation={:?}, type={:?})",
                    p,
                    self.with_ref(|b| b.body_type())
                )
            }
        }

        // ============================================================
        // RigidBodyBuilder
        // ============================================================

        /// Fluent builder for ``RigidBody`` instances.
        ///
        /// Each setter returns ``self`` for chaining; call ``build()`` to
        /// produce the actual body, or pass the builder directly to
        /// ``RigidBodySet.insert``::
        ///
        ///     builder = (RigidBody.dynamic()
        ///                 .translation(Vec3(0, 5, 0))
        ///                 .linvel(Vec3(1, 0, 0)))
        ///     handle = bodies.insert(builder)
        ///
        /// Most factories also accept keyword arguments matching builder
        /// methods, e.g. ``RigidBody.dynamic(translation=Vec3(0, 5, 0))``.
        #[pyclass(name = "RigidBodyBuilder", module = "rapier")]
        #[derive(Debug, Clone)]
        pub struct RigidBodyBuilder {
            pub builder: rapier::dynamics::RigidBodyBuilder,
        }

        impl RigidBodyBuilder {
            pub fn from_kwargs(
                base: rapier::dynamics::RigidBodyBuilder,
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
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
                match key {
                    "translation" => {
                        let pv: PyVector = v.extract()?;
                        let g: rapier::math::Vector = pv.0.into();
                        self.builder = self.builder.clone().translation(g);
                    }
                    "linvel" => {
                        let pv: PyVector = v.extract()?;
                        let g: rapier::math::Vector = pv.0.into();
                        self.builder = self.builder.clone().linvel(g);
                    }
                    "linear_damping" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().linear_damping(f);
                    }
                    "angular_damping" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().angular_damping(f);
                    }
                    "gravity_scale" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().gravity_scale(f);
                    }
                    "can_sleep" => {
                        let b: bool = v.extract()?;
                        self.builder = self.builder.clone().can_sleep(b);
                    }
                    "sleeping" => {
                        let b: bool = v.extract()?;
                        self.builder = self.builder.clone().sleeping(b);
                    }
                    "ccd_enabled" => {
                        let b: bool = v.extract()?;
                        self.builder = self.builder.clone().ccd_enabled(b);
                    }
                    "soft_ccd_prediction" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().soft_ccd_prediction(f);
                    }
                    "dominance_group" => {
                        let g: i8 = v.extract()?;
                        self.builder = self.builder.clone().dominance_group(g);
                    }
                    "additional_solver_iterations" => {
                        let n: usize = v.extract()?;
                        self.builder = self.builder.clone().additional_solver_iterations(n);
                    }
                    "user_data" => {
                        let d: u128 = v.extract()?;
                        self.builder = self.builder.clone().user_data(d);
                    }
                    "enabled" => {
                        let b: bool = v.extract()?;
                        self.builder = self.builder.clone().enabled(b);
                    }
                    "additional_mass" => {
                        let m: Real = v.extract()?;
                        self.builder = self.builder.clone().additional_mass(m);
                    }
                    "locked_axes" => {
                        let la: LockedAxes = v.extract()?;
                        self.builder = self.builder.clone().locked_axes(la.0);
                    }
                    "angvel" => {
                        Self::apply_angvel_kwarg(&mut self.builder, v)?;
                    }
                    "rotation" => {
                        Self::apply_rotation_kwarg(&mut self.builder, v)?;
                    }
                    "position" => {
                        let pi: PyIsometry = v.extract()?;
                        let g: rapier::math::Pose = pi.0.into();
                        self.builder = self.builder.clone().pose(g);
                    }
                    "enabled_translations" => {
                        Self::apply_enabled_translations_kwarg(&mut self.builder, v)?;
                    }
                    "enabled_rotations" => {
                        Self::apply_enabled_rotations_kwarg(&mut self.builder, v)?;
                    }
                    "gyroscopic_forces" => {
                        Self::apply_gyroscopic_forces_kwarg(&mut self.builder, v)?;
                    }
                    "body_type" => {
                        let bt: RigidBodyType = v.extract()?;
                        // Re-create builder of that type, preserving fields.
                        // Simplest: clone and `.set_body_type` doesn't exist on
                        // the builder. We swap base type while keeping the rest.
                        // RigidBodyBuilder has no `.body_type` setter; build &
                        // re-create. For simplicity, just panic if
                        // people mix base ctor with body_type kwarg.
                        let _ = bt;
                        return Err(PyTypeError::new_err(
                            "Use RigidBody.new_body(body_type=...) to set body_type — \
                             can't combine with the existing builder factory.",
                        ));
                    }
                    _ => {
                        return Err(PyTypeError::new_err(format!(
                            "unknown RigidBodyBuilder kwarg: '{}'",
                            key
                        )));
                    }
                }
                Ok(())
            }
        }

        #[pymethods]
        impl RigidBodyBuilder {
            /// Build a fresh ``RigidBodyBuilder`` for the given body type.
            ///
            /// :param body_type: a ``RigidBodyType`` selecting the behavior
            ///     class (default ``DYNAMIC``).
            #[new]
            #[pyo3(signature = (body_type=RigidBodyType::DYNAMIC))]
            fn new(body_type: RigidBodyType) -> Self {
                Self {
                    builder: rapier::dynamics::RigidBodyBuilder::new(body_type.to_rapier()),
                }
            }

            /// Set the initial translation.
            fn translation(&self, v: PyVector) -> Self {
                let g: rapier::math::Vector = v.0.into();
                Self {
                    builder: self.builder.clone().translation(g),
                }
            }

            /// Set the initial linear velocity.
            fn linvel(&self, v: PyVector) -> Self {
                let g: rapier::math::Vector = v.0.into();
                Self {
                    builder: self.builder.clone().linvel(g),
                }
            }

            /// Set the linear-velocity damping coefficient (per second).
            fn linear_damping(&self, f: Real) -> Self {
                Self {
                    builder: self.builder.clone().linear_damping(f),
                }
            }
            /// Set the angular-velocity damping coefficient (per second).
            fn angular_damping(&self, f: Real) -> Self {
                Self {
                    builder: self.builder.clone().angular_damping(f),
                }
            }
            /// Set the gravity multiplier for this body (``0`` disables
            /// gravity).
            fn gravity_scale(&self, f: Real) -> Self {
                Self {
                    builder: self.builder.clone().gravity_scale(f),
                }
            }
            /// Whether the body is allowed to fall asleep (default ``True``).
            fn can_sleep(&self, b: bool) -> Self {
                Self {
                    builder: self.builder.clone().can_sleep(b),
                }
            }
            /// Whether the body starts asleep.
            fn sleeping(&self, b: bool) -> Self {
                Self {
                    builder: self.builder.clone().sleeping(b),
                }
            }
            /// Enable substep CCD for this body.
            fn ccd_enabled(&self, b: bool) -> Self {
                Self {
                    builder: self.builder.clone().ccd_enabled(b),
                }
            }
            /// Set the soft-CCD prediction distance (``0`` disables it).
            fn soft_ccd_prediction(&self, f: Real) -> Self {
                Self {
                    builder: self.builder.clone().soft_ccd_prediction(f),
                }
            }
            /// Set the dominance group for contact biasing.
            fn dominance_group(&self, g: i8) -> Self {
                Self {
                    builder: self.builder.clone().dominance_group(g),
                }
            }
            /// Request additional solver iterations on this body's island.
            fn additional_solver_iterations(&self, n: usize) -> Self {
                Self {
                    builder: self.builder.clone().additional_solver_iterations(n),
                }
            }
            /// Attach an application-defined ``u128`` payload to the body.
            fn user_data(&self, d: u128) -> Self {
                Self {
                    builder: self.builder.clone().user_data(d),
                }
            }
            /// Whether the body is enabled at creation (default ``True``).
            fn enabled(&self, b: bool) -> Self {
                Self {
                    builder: self.builder.clone().enabled(b),
                }
            }
            /// Add extra mass on top of the colliders' contribution.
            fn additional_mass(&self, m: Real) -> Self {
                Self {
                    builder: self.builder.clone().additional_mass(m),
                }
            }
            /// Add extra mass properties on top of the colliders'
            /// contribution.
            fn additional_mass_properties(&self, mp: &MassProperties) -> Self {
                Self {
                    builder: self.builder.clone().additional_mass_properties(mp.0),
                }
            }
            /// Set which degrees of freedom are locked.
            fn locked_axes(&self, la: &LockedAxes) -> Self {
                Self {
                    builder: self.builder.clone().locked_axes(la.0),
                }
            }
            /// Set the initial pose (translation and rotation).
            fn position(&self, p: PyIsometry) -> Self {
                let g: rapier::math::Pose = p.0.into();
                Self {
                    builder: self.builder.clone().pose(g),
                }
            }

            /// Set the initial rotation.
            ///
            /// In 3D, ``v`` is a scaled-axis-angle vector; in 2D, a scalar
            /// angle in radians.
            fn rotation(&self, v: &Bound<'_, PyAny>) -> PyResult<Self> {
                let mut b = self.builder.clone();
                Self::apply_rotation_kwarg(&mut b, v)?;
                Ok(Self { builder: b })
            }

            /// Set the initial angular velocity.
            ///
            /// In 3D, ``v`` is a vector; in 2D, a scalar.
            fn angvel(&self, v: &Bound<'_, PyAny>) -> PyResult<Self> {
                let mut b = self.builder.clone();
                Self::apply_angvel_kwarg(&mut b, v)?;
                Ok(Self { builder: b })
            }

            /// Enable or disable individual translation axes.
            ///
            /// In 3D, ``v`` is a tuple ``(tx, ty, tz)``; in 2D, ``(tx, ty)``.
            fn enabled_translations(&self, v: &Bound<'_, PyAny>) -> PyResult<Self> {
                let mut b = self.builder.clone();
                Self::apply_enabled_translations_kwarg(&mut b, v)?;
                Ok(Self { builder: b })
            }

            /// Alias for ``enabled_translations``.
            fn restrict_translations(&self, v: &Bound<'_, PyAny>) -> PyResult<Self> {
                self.enabled_translations(v)
            }

            /// Enable or disable individual rotation axes.
            ///
            /// In 3D, ``v`` is a tuple ``(rx, ry, rz)``; in 2D, ``(rz,)``.
            fn enabled_rotations(&self, v: &Bound<'_, PyAny>) -> PyResult<Self> {
                let mut b = self.builder.clone();
                Self::apply_enabled_rotations_kwarg(&mut b, v)?;
                Ok(Self { builder: b })
            }

            /// Alias for ``enabled_rotations``.
            fn restrict_rotations(&self, v: &Bound<'_, PyAny>) -> PyResult<Self> {
                self.enabled_rotations(v)
            }

            /// Enable or disable gyroscopic-force integration (3D only).
            ///
            /// In 2D this builder ignores the kwarg.
            fn gyroscopic_forces(&self, v: &Bound<'_, PyAny>) -> PyResult<Self> {
                let mut b = self.builder.clone();
                Self::apply_gyroscopic_forces_kwarg(&mut b, v)?;
                Ok(Self { builder: b })
            }

            /// Finalize the builder and return the configured ``RigidBody``.
            fn build(&self) -> RigidBody {
                RigidBody::new_owned(self.builder.build())
            }
        }
    };
}

// ----------------------------------------------------------------------
// RigidBodySet (shared between 2D and 3D)
// ----------------------------------------------------------------------
#[doc(hidden)]
#[macro_export]
macro_rules! __define_dynamics_rigid_body_set {
    () => {
        /// Container holding every ``RigidBody`` in a physics world.
        ///
        /// Bodies are addressed by ``RigidBodyHandle`` and live in the set
        /// until ``remove``-d. The set supports ``len()``, ``in``, iteration
        /// (yielding ``(handle, body)`` pairs) and ``[handle]`` lookup.
        ///
        /// Bodies returned by ``get`` / ``__getitem__`` / iteration are live
        /// **views** into the set: ``set[h].linvel = v`` mutates the stored
        /// body in place, with no copy.
        #[pyclass(name = "RigidBodySet", module = "rapier", unsendable)]
        pub struct RigidBodySet(pub rapier::dynamics::RigidBodySet);

        #[pymethods]
        impl RigidBodySet {
            /// Build an empty rigid-body set.
            #[new]
            fn new() -> Self { Self(rapier::dynamics::RigidBodySet::new()) }

            /// Insert a rigid body and return its handle.
            ///
            /// Accepts either a ``RigidBody`` or a ``RigidBodyBuilder``;
            /// builders are built before insertion.
            ///
            /// :raises TypeError: if ``body`` is neither a ``RigidBody`` nor
            ///     a ``RigidBodyBuilder``.
            fn insert(&mut self, body: &Bound<'_, PyAny>) -> PyResult<RigidBodyHandle> {
                // Accept either a RigidBodyBuilder or a RigidBody.
                if let Ok(b) = body.extract::<PyRef<'_, RigidBodyBuilder>>() {
                    let rb = b.builder.build();
                    return Ok(RigidBodyHandle(self.0.insert(rb)));
                }
                if let Ok(rb) = body.extract::<PyRef<'_, RigidBody>>() {
                    let cloned = rb.to_owned_body();
                    return Ok(RigidBodyHandle(self.0.insert(cloned)));
                }
                Err(PyTypeError::new_err(
                    "RigidBodySet.insert expects a RigidBody or RigidBodyBuilder",
                ))
            }

            /// Remove the rigid body identified by ``handle``.
            ///
            /// Joints anchored to the body are removed from
            /// ``impulse_joints`` and ``multibody_joints``. If
            /// ``remove_attached_colliders`` is ``True`` (default), every
            /// collider attached to the body is also removed from
            /// ``colliders``; otherwise those colliders are simply detached.
            ///
            /// :returns: the removed body, or ``None`` if no body matched
            ///     ``handle``.
            #[pyo3(signature = (handle, islands, colliders, impulse_joints, multibody_joints, remove_attached_colliders=true))]
            fn remove(
                &mut self,
                handle: &RigidBodyHandle,
                islands: &mut IslandManager,
                colliders: &mut ColliderSet,
                impulse_joints: &mut ImpulseJointSet,
                multibody_joints: &mut MultibodyJointSet,
                remove_attached_colliders: bool,
            ) -> Option<RigidBody> {
                self.0
                    .remove(
                        handle.0,
                        &mut islands.0,
                        &mut colliders.0,
                        &mut impulse_joints.0,
                        &mut multibody_joints.0,
                        remove_attached_colliders,
                    )
                    .map(RigidBody::new_owned)
            }

            /// Return a live **view** of the body identified by ``handle``, or
            /// ``None`` if there is none.
            ///
            /// The returned body reads and writes straight through to the set:
            /// ``set.get(h).linvel = v`` persists immediately, with no copy.
            fn get(slf: &Bound<'_, Self>, handle: &RigidBodyHandle) -> Option<RigidBody> {
                slf.borrow().0.get(handle.0)?;
                Some(RigidBody {
                    backing: RigidBodyBacking::InSet {
                        set: slf.clone().unbind(),
                        handle: handle.0,
                    },
                })
            }

            /// Indexing form of ``get`` — returns a live view into the set.
            ///
            /// :raises InvalidHandle: if ``handle`` does not match any body.
            fn __getitem__(
                slf: &Bound<'_, Self>,
                handle: &RigidBodyHandle,
            ) -> PyResult<RigidBody> {
                if slf.borrow().0.get(handle.0).is_none() {
                    return Err($crate::errors::InvalidHandle::new_err(format!(
                        "no rigid body for {:?}",
                        handle.0.into_raw_parts()
                    )));
                }
                Ok(RigidBody {
                    backing: RigidBodyBacking::InSet {
                        set: slf.clone().unbind(),
                        handle: handle.0,
                    },
                })
            }

            /// ``handle in self`` — whether ``handle`` refers to a body
            /// stored in this set.
            fn __contains__(&self, handle: &RigidBodyHandle) -> bool {
                self.0.contains(handle.0)
            }

            /// Number of bodies in the set.
            fn __len__(&self) -> usize { self.0.len() }

            /// Remove every body from the set.
            fn clear(&mut self) {
                // RigidBodySet doesn't expose `clear()`, so we re-init.
                self.0 = rapier::dynamics::RigidBodySet::new();
            }

            /// Iterate over ``(handle, body)`` pairs.
            ///
            /// Each yielded body is a live view into the set (no copy).
            fn __iter__(slf: &Bound<'_, Self>) -> PyResult<Py<RigidBodySetIter>> {
                let handles: Vec<rapier::dynamics::RigidBodyHandle> =
                    slf.borrow().0.iter().map(|(h, _)| h).collect();
                Py::new(
                    slf.py(),
                    RigidBodySetIter { set: slf.clone().unbind(), handles, i: 0 },
                )
            }

            /// Iterate over the handles of every body in the set.
            fn handles(slf: PyRef<'_, Self>) -> PyResult<Py<RigidBodyHandleIter>> {
                let handles: Vec<RigidBodyHandle> =
                    slf.0.iter().map(|(h, _)| RigidBodyHandle(h)).collect();
                Py::new(slf.py(), RigidBodyHandleIter { handles, i: 0 })
            }

        }

        /// Iterator yielding ``(RigidBodyHandle, RigidBody)`` pairs from a
        /// ``RigidBodySet``.
        ///
        /// Each ``RigidBody`` is a live view into the set.
        #[pyclass]
        pub struct RigidBodySetIter {
            set: Py<RigidBodySet>,
            handles: Vec<rapier::dynamics::RigidBodyHandle>,
            i: usize,
        }

        #[pymethods]
        impl RigidBodySetIter {
            /// Return ``self`` so the object satisfies the iterator protocol.
            fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> { slf }
            /// Return the next ``(handle, body)`` pair, or raise
            /// ``StopIteration``. The body is built lazily as a live view.
            fn __next__(mut slf: PyRefMut<'_, Self>) -> Option<(RigidBodyHandle, RigidBody)> {
                if slf.i >= slf.handles.len() { return None; }
                let py = slf.py();
                let handle = slf.handles[slf.i];
                slf.i += 1;
                let set = slf.set.clone_ref(py);
                Some((
                    RigidBodyHandle(handle),
                    RigidBody { backing: RigidBodyBacking::InSet { set, handle } },
                ))
            }
        }
    };
}

// ----------------------------------------------------------------------
// register_dynamics — per-dim entry point.
// ----------------------------------------------------------------------
#[doc(hidden)]
#[macro_export]
macro_rules! __define_dynamics_register {
    (3) => {
        pub fn register_dynamics(
            _py: $crate::pyo3::Python<'_>,
            m: &$crate::pyo3::Bound<'_, $crate::pyo3::types::PyModule>,
        ) -> $crate::pyo3::PyResult<()> {
            use $crate::pyo3::prelude::*;
            // ImpulseJointSet / MultibodyJointSet registered by joints.rs.
            m.add_class::<RigidBodyHandle>()?;
            m.add_class::<RigidBodyType>()?;
            m.add_class::<CoefficientCombineRule>()?;
            m.add_class::<LockedAxes>()?;
            m.add_class::<RigidBodyActivation>()?;
            m.add_class::<SpringCoefficients>()?;
            m.add_class::<IslandManager>()?;
            m.add_class::<RigidBodyHandleIter>()?;
            m.add_class::<CCDSolver>()?;
            m.add_class::<FrictionModel>()?;
            m.add_class::<MassProperties>()?;
            m.add_class::<RigidBodyAdditionalMassProps>()?;
            m.add_class::<RigidBodyDamping>()?;
            m.add_class::<RigidBodyDominance>()?;
            m.add_class::<RigidBodyCcd>()?;
            m.add_class::<RigidBodyVelocity>()?;
            m.add_class::<RigidBodyForces>()?;
            m.add_class::<RigidBodyMassProps>()?;
            m.add_class::<RigidBodyPosition>()?;
            m.add_class::<IntegrationParameters>()?;
            m.add_class::<RigidBody>()?;
            m.add_class::<RigidBodyBuilder>()?;
            m.add_class::<RigidBodySet>()?;
            m.add_class::<RigidBodySetIter>()?;
            Ok(())
        }
    };
    (2) => {
        pub fn register_dynamics(
            _py: $crate::pyo3::Python<'_>,
            m: &$crate::pyo3::Bound<'_, $crate::pyo3::types::PyModule>,
        ) -> $crate::pyo3::PyResult<()> {
            use $crate::pyo3::prelude::*;
            // ImpulseJointSet / MultibodyJointSet registered by joints.rs.
            m.add_class::<RigidBodyHandle>()?;
            m.add_class::<RigidBodyType>()?;
            m.add_class::<CoefficientCombineRule>()?;
            m.add_class::<LockedAxes>()?;
            m.add_class::<RigidBodyActivation>()?;
            m.add_class::<SpringCoefficients>()?;
            m.add_class::<IslandManager>()?;
            m.add_class::<RigidBodyHandleIter>()?;
            m.add_class::<CCDSolver>()?;
            m.add_class::<MassProperties>()?;
            m.add_class::<RigidBodyAdditionalMassProps>()?;
            m.add_class::<RigidBodyDamping>()?;
            m.add_class::<RigidBodyDominance>()?;
            m.add_class::<RigidBodyCcd>()?;
            m.add_class::<RigidBodyVelocity>()?;
            m.add_class::<RigidBodyForces>()?;
            m.add_class::<RigidBodyMassProps>()?;
            m.add_class::<RigidBodyPosition>()?;
            m.add_class::<IntegrationParameters>()?;
            m.add_class::<RigidBody>()?;
            m.add_class::<RigidBodyBuilder>()?;
            m.add_class::<RigidBodySet>()?;
            m.add_class::<RigidBodySetIter>()?;
            Ok(())
        }
    };
}
