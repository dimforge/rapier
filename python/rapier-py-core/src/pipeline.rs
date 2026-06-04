//! Macro emitting the user-facing pipeline / query `#[pyclass]`-es per cdylib.
//!
//! Like `define_dynamics_types!` and `define_geometry_types!`, this macro is
//! invoked once per cdylib AFTER the math/geometry/dynamics macros. It expects
//! the surrounding cdylib to already have:
//!   - The `Real`/`DIM` aliases and `Py{Vector,Point,Rotation,Isometry,AngVector}`
//!     adapters in scope (from `define_conv_types!`).
//!   - The user-facing math types (`define_math_types!`).
//!   - The geometry types (`define_geometry_types!`).
//!   - The dynamics types (`define_dynamics_types!`).
//!   - `use rapier{2,3}d{,_f64} as rapier;` aliasing.
//!
//! Produces `register_pipeline(py, m) -> PyResult<()>` for the `#[pymodule]`.

/// Materialize the pipeline / query `#[pyclass]` types for a given
/// `(Real, DIM)` pair.
#[macro_export]
macro_rules! define_pipeline_types {
    (DIM = 2) => {
        $crate::__define_pipeline_shared!(2, Vec2, Point2, Rotation2, Isometry2);
        $crate::__define_pipeline_register!();
    };
    (DIM = 3) => {
        $crate::__define_pipeline_shared!(3, Vec3, Point3, Rotation3, Isometry3);
        $crate::__define_pipeline_register!();
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_pipeline_shared {
    ($dim:tt, $Vec:ident, $Point:ident, $Rot:ident, $Iso:ident) => {
        // PyO3 / pyo3 utility imports — math/geometry macros already brought
        // `PyTypeError`, `PyAny`, `Bound`, `Py`, etc. into scope, so we only
        // pull in what's new for pipeline.
        use $crate::pyo3::types::PyList as _PyList;

        // =====================================================================
        // FeatureId — tagged-union over parry's `FeatureId` (Vertex/Edge/Face/Unknown).
        // =====================================================================

        /// Tagged-union identifier for a shape feature returned by a query.
        ///
        /// Wraps parry's ``FeatureId``: one of ``Vertex(id)``, ``Edge(id)``
        /// (3D only), ``Face(id)``, or ``Unknown``. The :attr:`kind` field
        /// holds the variant name (``"vertex"``, ``"edge"``, ``"face"``,
        /// ``"unknown"``); :attr:`id` holds the numeric index (``0`` for
        /// ``Unknown``).
        ///
        /// Construct via the static factories :meth:`Vertex`, :meth:`Edge`,
        /// :meth:`Face`, :meth:`Unknown`.
        #[pyclass(name = "FeatureId", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct FeatureId {
            /// `"vertex"`, `"edge"` (3D only), `"face"`, or `"unknown"`.
            #[pyo3(get)] pub kind: &'static str,
            /// 0 when `kind == "unknown"`, otherwise the parry id.
            #[pyo3(get)] pub id: u32,
        }

        #[pymethods]
        impl FeatureId {
            /// Construct a ``Vertex(id)`` feature identifier.
            #[staticmethod]
            #[pyo3(name = "Vertex")]
            fn vertex(id: u32) -> Self { Self { kind: "vertex", id } }
            /// Construct a ``Face(id)`` feature identifier.
            #[staticmethod]
            #[pyo3(name = "Face")]
            fn face(id: u32) -> Self { Self { kind: "face", id } }
            /// Construct an ``Edge(id)`` feature identifier.
            ///
            /// Edges are only returned for 3D queries; the constructor is
            /// exposed on both dimensions for portability.
            // FeatureId::Edge: 3D only in parry, but we expose it on both
            // dims as a generic constructor for portability.
            #[staticmethod]
            #[pyo3(name = "Edge")]
            fn edge(id: u32) -> Self { Self { kind: "edge", id } }
            /// Construct the ``Unknown`` feature identifier.
            #[staticmethod]
            #[pyo3(name = "Unknown")]
            fn unknown() -> Self { Self { kind: "unknown", id: 0 } }
            /// ``True`` iff this is a ``Vertex(_)``.
            #[getter] fn is_vertex(&self) -> bool { self.kind == "vertex" }
            /// ``True`` iff this is an ``Edge(_)``.
            #[getter] fn is_edge(&self) -> bool { self.kind == "edge" }
            /// ``True`` iff this is a ``Face(_)``.
            #[getter] fn is_face(&self) -> bool { self.kind == "face" }
            /// ``True`` iff this is ``Unknown``.
            #[getter] fn is_unknown(&self) -> bool { self.kind == "unknown" }
            /// Human-readable ``FeatureId.Variant(id)`` repr.
            fn __repr__(&self) -> String {
                if self.kind == "unknown" {
                    "FeatureId.Unknown".to_string()
                } else {
                    format!("FeatureId.{}({})",
                        match self.kind {
                            "vertex" => "Vertex",
                            "edge" => "Edge",
                            "face" => "Face",
                            _ => "Unknown",
                        }, self.id)
                }
            }
            /// Equality on (kind, id); other comparisons raise.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.kind == other.kind && self.id == other.id),
                    CompareOp::Ne => Ok(!(self.kind == other.kind && self.id == other.id)),
                    _ => Err(PyTypeError::new_err("FeatureId supports only == and !=")),
                }
            }
        }

        impl FeatureId {
            #[allow(dead_code)]
            #[inline]
            pub(crate) fn from_parry(f: rapier::parry::shape::FeatureId) -> Self {
                $crate::__feature_id_from_parry!($dim, f)
            }
        }

        // =====================================================================
        // Ray
        // =====================================================================

        /// A half-line defined by an :attr:`origin` point and a :attr:`dir`
        /// vector.
        ///
        /// Passed to the various ``cast_ray*`` / ``intersect_ray`` queries.
        /// The direction need not be normalized — query time-of-impact (TOI)
        /// values are expressed in units of ``dir``.
        #[pyclass(name = "Ray", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct Ray(pub rapier::parry::query::Ray);

        #[pymethods]
        impl Ray {
            /// Construct a ray from an origin point and direction vector.
            ///
            /// :param origin: World-space origin point.
            /// :param dir: World-space direction (does not need to be unit).
            #[new]
            fn new(origin: PyPoint, dir: PyVector) -> Self {
                let o: rapier::math::Vector = origin.0.coords.into();
                let d: rapier::math::Vector = dir.0.into();
                Self(rapier::parry::query::Ray::new(o, d))
            }
            /// World-space origin point.
            #[getter]
            fn origin(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.origin.into();
                $Point($crate::na::Point::from(v))
            }
            /// World-space direction vector.
            #[getter]
            fn dir(&self) -> $Vec {
                let v: $crate::na::SVector<Real, $dim> = self.0.dir.into();
                $Vec(v)
            }
            /// Evaluate the ray's position at parameter ``t``.
            ///
            /// Computes ``origin + dir * t``. When ``dir`` is unit-length, ``t``
            /// is the Euclidean distance from the origin.
            ///
            /// :param t: Ray parameter (typically the TOI returned by a query).
            /// :returns: World-space point on the ray.
            fn point_at(&self, t: Real) -> $Point {
                let p: rapier::math::Vector = self.0.origin + self.0.dir * t;
                let v: $crate::na::SVector<Real, $dim> = p.into();
                $Point($crate::na::Point::from(v))
            }
            /// Debug repr — shows the origin and direction.
            fn __repr__(&self) -> String {
                format!("Ray(origin={:?}, dir={:?})", self.0.origin, self.0.dir)
            }
        }

        // =====================================================================
        // RayIntersection
        // =====================================================================

        /// Result of a ray-vs-shape intersection query.
        ///
        /// :attr:`toi` is the ray parameter at the hit (see :meth:`Ray.point_at`).
        /// :attr:`normal` is the world-space surface normal pointing away from
        /// the hit collider. :attr:`feature` identifies the shape feature
        /// (vertex / edge / face) that was hit, when known.
        #[pyclass(name = "RayIntersection", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct RayIntersection {
            /// Time of impact — ray parameter at the hit.
            #[pyo3(get)] pub toi: Real,
            /// Surface normal at the hit, pointing away from the collider.
            #[pyo3(get)] pub normal: $Vec,
            /// Shape feature that was hit.
            #[pyo3(get)] pub feature: FeatureId,
        }

        #[pymethods]
        impl RayIntersection {
            /// Alias for :attr:`toi` matching parry's naming.
            #[getter] fn time_of_impact(&self) -> Real { self.toi }
            /// Debug repr — shows TOI and normal.
            fn __repr__(&self) -> String {
                format!("RayIntersection(toi={}, normal={:?})", self.toi, self.normal.0)
            }
        }

        impl RayIntersection {
            #[allow(dead_code)]
            pub(crate) fn from_parry(r: rapier::parry::query::RayIntersection) -> Self {
                let n: $crate::na::SVector<Real, $dim> = r.normal.into();
                Self {
                    toi: r.time_of_impact,
                    normal: $Vec(n),
                    feature: FeatureId::from_parry(r.feature),
                }
            }
        }

        // =====================================================================
        // PointProjection
        // =====================================================================

        /// Result of a project-point query.
        ///
        /// :attr:`point` is the world-space closest point on (or inside) the
        /// collider's shape; :attr:`is_inside` is ``True`` iff the query
        /// point lies inside the (solid) shape — in which case the projected
        /// point may coincide with the query point.
        #[pyclass(name = "PointProjection", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct PointProjection {
            /// ``True`` iff the query point lies inside the shape.
            #[pyo3(get)] pub is_inside: bool,
            /// World-space closest point on (or inside) the shape.
            #[pyo3(get)] pub point: $Point,
        }

        #[pymethods]
        impl PointProjection {
            /// Debug repr — shows the projected point and ``is_inside`` flag.
            fn __repr__(&self) -> String {
                format!("PointProjection(is_inside={}, point={:?})",
                    self.is_inside, self.point.0)
            }
        }

        impl PointProjection {
            #[allow(dead_code)]
            pub(crate) fn from_parry(p: rapier::parry::query::PointProjection) -> Self {
                // parry's PointProjection.point is a glam Vector, not a Point.
                let v: $crate::na::SVector<Real, $dim> = p.point.into();
                Self {
                    is_inside: p.is_inside,
                    point: $Point($crate::na::Point::from(v)),
                }
            }
        }

        // =====================================================================
        // ShapeCastStatus
        // =====================================================================

        /// Outcome status of a shape-cast query.
        ///
        /// - ``OUT_OF_ITERATIONS``: solver hit the iteration cap before
        ///   converging; the returned TOI is approximate.
        /// - ``CONVERGED``: the solver converged on a TOI within tolerance.
        /// - ``FAILED``: the solver failed (degenerate geometry or numerical
        ///   issue); the result should be ignored.
        /// - ``PENETRATING_OR_WITHIN_TARGET_DIST``: the shapes are already in
        ///   contact (or within ``target_distance``) at ``t = 0``.
        #[pyclass(name = "ShapeCastStatus", module = "rapier", eq, eq_int)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum ShapeCastStatus {
            OUT_OF_ITERATIONS,
            CONVERGED,
            FAILED,
            PENETRATING_OR_WITHIN_TARGET_DIST,
        }

        impl ShapeCastStatus {
            #[allow(dead_code)]
            fn from_parry(s: rapier::parry::query::ShapeCastStatus) -> Self {
                use rapier::parry::query::ShapeCastStatus as S;
                match s {
                    S::OutOfIterations => Self::OUT_OF_ITERATIONS,
                    S::Converged => Self::CONVERGED,
                    S::Failed => Self::FAILED,
                    S::PenetratingOrWithinTargetDist => Self::PENETRATING_OR_WITHIN_TARGET_DIST,
                }
            }
        }

        // =====================================================================
        // ShapeCastOptions
        // =====================================================================

        /// Tuning parameters for shape-cast queries.
        ///
        /// Fields:
        ///
        /// - :attr:`max_time_of_impact`: upper bound on the returned TOI;
        ///   defaults to ``+inf``.
        /// - :attr:`target_distance`: shapes are considered "in contact"
        ///   when their closest distance is within this margin (useful for
        ///   character-controller-style queries that want to stop just shy
        ///   of penetration).
        /// - :attr:`stop_at_penetration`: when ``True`` (default), the query
        ///   reports a hit with ``t = 0`` if the shapes already overlap; when
        ///   ``False``, the cast continues past the initial penetration.
        /// - :attr:`compute_impact_geometry_on_penetration`: when ``True``
        ///   (default), compute witness points / normals even when the
        ///   ``PENETRATING_OR_WITHIN_TARGET_DIST`` status is returned.
        #[pyclass(name = "ShapeCastOptions", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct ShapeCastOptions(pub rapier::parry::query::ShapeCastOptions);

        #[pymethods]
        impl ShapeCastOptions {
            /// Construct a ``ShapeCastOptions`` from explicit field values.
            ///
            /// :param max_time_of_impact: Upper bound on the TOI.
            /// :param target_distance: Tolerance for "in contact".
            /// :param stop_at_penetration: Whether to report ``t = 0`` hits.
            /// :param compute_impact_geometry_on_penetration: Whether to
            ///     compute witness geometry on penetration.
            #[new]
            #[pyo3(signature = (
                max_time_of_impact=Real::MAX,
                target_distance=0.0 as Real,
                stop_at_penetration=true,
                compute_impact_geometry_on_penetration=true,
            ))]
            fn new(
                max_time_of_impact: Real,
                target_distance: Real,
                stop_at_penetration: bool,
                compute_impact_geometry_on_penetration: bool,
            ) -> Self {
                Self(rapier::parry::query::ShapeCastOptions {
                    max_time_of_impact,
                    target_distance,
                    stop_at_penetration,
                    compute_impact_geometry_on_penetration,
                })
            }
            /// Shortcut constructor — defaults plus an explicit TOI bound.
            ///
            /// :param t: ``max_time_of_impact`` value.
            #[staticmethod]
            fn with_max_time_of_impact(t: Real) -> Self {
                Self(rapier::parry::query::ShapeCastOptions::with_max_time_of_impact(t))
            }
            /// Upper bound on the returned time of impact.
            #[getter] fn max_time_of_impact(&self) -> Real { self.0.max_time_of_impact }
            /// Set :attr:`max_time_of_impact`.
            #[setter] fn set_max_time_of_impact(&mut self, v: Real) { self.0.max_time_of_impact = v; }
            /// Distance below which the shapes are considered to be in contact.
            #[getter] fn target_distance(&self) -> Real { self.0.target_distance }
            /// Set :attr:`target_distance`.
            #[setter] fn set_target_distance(&mut self, v: Real) { self.0.target_distance = v; }
            /// Whether the cast reports ``t = 0`` hits when shapes overlap at the start.
            #[getter] fn stop_at_penetration(&self) -> bool { self.0.stop_at_penetration }
            /// Set :attr:`stop_at_penetration`.
            #[setter] fn set_stop_at_penetration(&mut self, v: bool) { self.0.stop_at_penetration = v; }
            /// Whether witness points / normals are computed on penetration.
            #[getter] fn compute_impact_geometry_on_penetration(&self) -> bool {
                self.0.compute_impact_geometry_on_penetration
            }
            /// Set :attr:`compute_impact_geometry_on_penetration`.
            #[setter] fn set_compute_impact_geometry_on_penetration(&mut self, v: bool) {
                self.0.compute_impact_geometry_on_penetration = v;
            }
            /// Debug repr — shows the main TOI and target-distance settings.
            fn __repr__(&self) -> String {
                format!("ShapeCastOptions(max_time_of_impact={}, target_distance={})",
                    self.0.max_time_of_impact, self.0.target_distance)
            }
        }

        // =====================================================================
        // ShapeCastHit
        // =====================================================================

        /// Result of a shape-cast query.
        ///
        /// :attr:`time_of_impact` is the parametric TOI along the cast
        /// motion. :attr:`witness1` / :attr:`witness2` are the world-space
        /// closest points on the cast shape and the hit collider at the time
        /// of impact; :attr:`normal1` / :attr:`normal2` are the corresponding
        /// outward surface normals. :attr:`status` is a :class:`ShapeCastStatus`
        /// describing how the cast terminated.
        #[pyclass(name = "ShapeCastHit", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct ShapeCastHit {
            /// Parametric TOI along the cast motion.
            #[pyo3(get)] pub time_of_impact: Real,
            /// World-space witness point on the cast shape at impact.
            #[pyo3(get)] pub witness1: $Point,
            /// World-space witness point on the hit collider at impact.
            #[pyo3(get)] pub witness2: $Point,
            /// Outward surface normal on the cast shape at impact.
            #[pyo3(get)] pub normal1: $Vec,
            /// Outward surface normal on the hit collider at impact.
            #[pyo3(get)] pub normal2: $Vec,
            /// Termination status of the cast (see :class:`ShapeCastStatus`).
            #[pyo3(get)] pub status: ShapeCastStatus,
        }

        #[pymethods]
        impl ShapeCastHit {
            /// Debug repr — shows the TOI and status.
            fn __repr__(&self) -> String {
                format!("ShapeCastHit(toi={}, status={:?})",
                    self.time_of_impact, self.status)
            }
        }

        impl ShapeCastHit {
            #[allow(dead_code)]
            pub(crate) fn from_parry(h: rapier::parry::query::ShapeCastHit) -> Self {
                let w1: $crate::na::SVector<Real, $dim> = h.witness1.into();
                let w2: $crate::na::SVector<Real, $dim> = h.witness2.into();
                let n1: $crate::na::SVector<Real, $dim> = h.normal1.into();
                let n2: $crate::na::SVector<Real, $dim> = h.normal2.into();
                Self {
                    time_of_impact: h.time_of_impact,
                    witness1: $Point($crate::na::Point::from(w1)),
                    witness2: $Point($crate::na::Point::from(w2)),
                    normal1: $Vec(n1),
                    normal2: $Vec(n2),
                    status: ShapeCastStatus::from_parry(h.status),
                }
            }
        }

        // =====================================================================
        // NonlinearRigidMotion
        // =====================================================================

        /// Curved rigid-body motion used by nonlinear shape casts.
        ///
        /// Describes a constant-velocity screw motion around a body's
        /// :attr:`local_center`, starting from an initial pose :attr:`start`
        /// with linear velocity :attr:`linvel` and angular velocity
        /// :attr:`angvel`. Passed to
        /// :meth:`QueryPipeline.cast_shape_nonlinear` to capture rotational
        /// effects (e.g. swinging characters) that a straight-line
        /// :meth:`QueryPipeline.cast_shape` would miss.
        #[pyclass(name = "NonlinearRigidMotion", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct NonlinearRigidMotion(pub rapier::parry::query::NonlinearRigidMotion);

        #[pymethods]
        impl NonlinearRigidMotion {
            /// Construct a nonlinear motion from its constituent fields.
            ///
            /// :param start: Initial pose; defaults to identity.
            /// :param local_center: Local-space pivot point; defaults to origin.
            /// :param linvel: Linear velocity; defaults to zero.
            /// :param angvel: Angular velocity (3D: ``Vec3``, 2D: scalar);
            ///     defaults to zero.
            #[new]
            #[pyo3(signature = (start=None, local_center=None, linvel=None, angvel=None))]
            fn new(
                start: Option<PyIsometry>,
                local_center: Option<PyPoint>,
                linvel: Option<PyVector>,
                angvel: Option<&Bound<'_, PyAny>>,
            ) -> PyResult<Self> {
                let s: rapier::math::Pose = start
                    .map(|i| i.0.into())
                    .unwrap_or_else(rapier::math::Pose::identity);
                let lc_vec: rapier::math::Vector = local_center
                    .map(|p| p.0.coords.into())
                    .unwrap_or(rapier::math::Vector::ZERO);
                let lv: rapier::math::Vector = linvel
                    .map(|v| v.0.into())
                    .unwrap_or(rapier::math::Vector::ZERO);
                let av = $crate::__nonlinear_angvel_extract!($dim, angvel)?;
                Ok(Self(rapier::parry::query::NonlinearRigidMotion {
                    start: s,
                    local_center: lc_vec,
                    linvel: lv,
                    angvel: av,
                }))
            }
            /// Identity motion — no rotation, no translation, identity start.
            #[staticmethod]
            fn identity() -> Self {
                Self(rapier::parry::query::NonlinearRigidMotion::identity())
            }
            /// Stationary motion held at ``pos`` for all times.
            ///
            /// :param pos: Pose to hold.
            #[staticmethod]
            fn constant_position(pos: PyIsometry) -> Self {
                let p: rapier::math::Pose = pos.0.into();
                Self(rapier::parry::query::NonlinearRigidMotion::constant_position(p))
            }
            /// Initial pose at ``t = 0``.
            #[getter]
            fn start(&self) -> $Iso {
                let iso: $crate::na::Isometry<Real, _, $dim> = self.0.start.into();
                $Iso(iso)
            }
            /// Local-space pivot the angular velocity rotates around.
            #[getter]
            fn local_center(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.local_center.into();
                $Point($crate::na::Point::from(v))
            }
            /// Linear velocity (world-space, units per unit time).
            #[getter]
            fn linvel(&self) -> $Vec {
                let v: $crate::na::SVector<Real, $dim> = self.0.linvel.into();
                $Vec(v)
            }
            /// Evaluate the motion at time ``t``.
            ///
            /// :param t: Time parameter (``0`` returns :attr:`start`).
            /// :returns: Pose at time ``t``.
            fn position_at_time(&self, t: Real) -> $Iso {
                let p: $crate::na::Isometry<Real, _, $dim> = self.0.position_at_time(t).into();
                $Iso(p)
            }
        }

        // Dim-specific `angvel` getter — emitted in a separate pymethods block
        // (multiple-pymethods feature) because the angvel type differs.
        $crate::__nonlinear_angvel_getter!($dim, $Vec);

        // =====================================================================
        // QueryFilterFlags
        // =====================================================================

        /// Bitflags controlling which colliders / bodies a scene query touches.
        ///
        /// Combine flags with ``|``. Common preset combinations are also
        /// exposed as named static constructors on :class:`QueryFilter`
        /// (e.g. ``QueryFilter.only_dynamic()``).
        #[pyclass(name = "QueryFilterFlags", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct QueryFilterFlags(pub rapier::pipeline::QueryFilterFlags);

        #[pymethods]
        impl QueryFilterFlags {
            /// Construct from a raw bit pattern (unknown bits are truncated).
            ///
            /// :param bits: Raw bit pattern.
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u32) -> Self {
                Self(rapier::pipeline::QueryFilterFlags::from_bits_truncate(bits))
            }
            /// Return an empty flag set (no filtering).
            #[staticmethod]
            fn empty() -> Self { Self(rapier::pipeline::QueryFilterFlags::empty()) }

            /// Skip colliders attached to fixed (static) bodies.
            #[classattr]
            const EXCLUDE_FIXED: QueryFilterFlags =
                QueryFilterFlags(rapier::pipeline::QueryFilterFlags::EXCLUDE_FIXED);
            /// Skip colliders attached to kinematic bodies.
            #[classattr]
            const EXCLUDE_KINEMATIC: QueryFilterFlags =
                QueryFilterFlags(rapier::pipeline::QueryFilterFlags::EXCLUDE_KINEMATIC);
            /// Skip colliders attached to dynamic bodies.
            #[classattr]
            const EXCLUDE_DYNAMIC: QueryFilterFlags =
                QueryFilterFlags(rapier::pipeline::QueryFilterFlags::EXCLUDE_DYNAMIC);
            /// Skip sensor colliders.
            #[classattr]
            const EXCLUDE_SENSORS: QueryFilterFlags =
                QueryFilterFlags(rapier::pipeline::QueryFilterFlags::EXCLUDE_SENSORS);
            /// Skip non-sensor (solid) colliders.
            #[classattr]
            const EXCLUDE_SOLIDS: QueryFilterFlags =
                QueryFilterFlags(rapier::pipeline::QueryFilterFlags::EXCLUDE_SOLIDS);
            /// Keep only colliders attached to dynamic bodies.
            #[classattr]
            const ONLY_DYNAMIC: QueryFilterFlags =
                QueryFilterFlags(rapier::pipeline::QueryFilterFlags::ONLY_DYNAMIC);
            /// Keep only colliders attached to kinematic bodies.
            #[classattr]
            const ONLY_KINEMATIC: QueryFilterFlags =
                QueryFilterFlags(rapier::pipeline::QueryFilterFlags::ONLY_KINEMATIC);
            /// Keep only colliders attached to fixed (static) bodies.
            #[classattr]
            const ONLY_FIXED: QueryFilterFlags =
                QueryFilterFlags(rapier::pipeline::QueryFilterFlags::ONLY_FIXED);
            /// Reserved — currently maps to an empty set (forward-compat).
            // `EXCLUDE_COLLISION_INVALID` doesn't exist upstream; we map it to
            // an empty flag set for forward-compat.
            #[classattr]
            const EXCLUDE_COLLISION_INVALID: QueryFilterFlags =
                QueryFilterFlags(rapier::pipeline::QueryFilterFlags::empty());

            /// Raw underlying bit pattern.
            #[getter] fn bits(&self) -> u32 { self.0.bits() }
            /// ``True`` if every bit set in ``other`` is also set here.
            fn contains(&self, other: &Self) -> bool { self.0.contains(other.0) }
            /// ``True`` if no flag is set.
            fn is_empty(&self) -> bool { self.0.is_empty() }
            /// ``other in self`` — alias for :meth:`contains`.
            fn __contains__(&self, other: &Self) -> bool { self.0.contains(other.0) }
            /// Bitwise OR — union of flag sets.
            fn __or__(&self, other: &Self) -> Self { Self(self.0 | other.0) }
            /// Bitwise AND — intersection of flag sets.
            fn __and__(&self, other: &Self) -> Self { Self(self.0 & other.0) }
            /// Bitwise XOR — symmetric difference.
            fn __xor__(&self, other: &Self) -> Self { Self(self.0 ^ other.0) }
            /// Bitwise NOT — flip every known flag.
            fn __invert__(&self) -> Self { Self(!self.0) }
            /// Set difference — remove flags present in ``other``.
            fn __sub__(&self, other: &Self) -> Self { Self(self.0 - other.0) }
            /// Truthy iff at least one flag is set.
            fn __bool__(&self) -> bool { !self.0.is_empty() }
            /// Equality on the raw bit pattern; other comparisons raise.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err("QueryFilterFlags supports only == and !=")),
                }
            }
            /// Hash on the raw bit pattern (makes the value set-safe).
            fn __hash__(&self) -> u64 { self.0.bits() as u64 }
            /// Debug repr — shows the raw bit pattern in binary.
            fn __repr__(&self) -> String {
                format!("QueryFilterFlags(bits={:#010b})", self.0.bits())
            }
        }

        // =====================================================================
        // QueryFilter — Python-side mutable builder.
        //
        // Storing a `rapier::pipeline::QueryFilter<'_>` directly is awkward
        // because the upstream type carries a lifetime for the predicate
        // borrow. We instead keep the constituent fields and reconstruct a
        // borrowed `QueryFilter` inside each query call via `with_filter`.
        // =====================================================================

        /// Filter passed to scene queries to narrow the set of inspected colliders.
        ///
        /// A ``QueryFilter`` composes:
        ///
        /// - :class:`QueryFilterFlags` (membership: include/exclude
        ///   dynamic / kinematic / fixed / sensor / solid).
        /// - :class:`InteractionGroups` (group-mask membership / filter bits).
        /// - explicit exclusion of a single :class:`ColliderHandle` and/or a
        ///   single :class:`RigidBodyHandle`.
        /// - an optional Python predicate ``f(handle, collider) -> bool``
        ///   invoked per-collider to make the final keep/skip decision.
        ///
        /// All builder methods (``exclude_sensors``, ``groups``,
        /// ``exclude_collider``, ``predicate``, ...) return a new
        /// :class:`QueryFilter` rather than mutating in place, so they can be
        /// chained.
        #[pyclass(name = "QueryFilter", module = "rapier")]
        pub struct QueryFilter {
            pub flags: rapier::pipeline::QueryFilterFlags,
            pub groups: Option<rapier::geometry::InteractionGroups>,
            pub exclude_collider: Option<rapier::geometry::ColliderHandle>,
            pub exclude_rigid_body: Option<rapier::dynamics::RigidBodyHandle>,
            pub predicate: Option<Py<PyAny>>,
        }

        impl QueryFilter {
            /// Deep-clone (the predicate `Py<PyAny>` requires a `Python<'_>` token).
            fn clone_with(&self, py: Python<'_>) -> Self {
                Self {
                    flags: self.flags,
                    groups: self.groups,
                    exclude_collider: self.exclude_collider,
                    exclude_rigid_body: self.exclude_rigid_body,
                    predicate: self.predicate.as_ref().map(|p| p.clone_ref(py)),
                }
            }
        }

        impl QueryFilter {
            /// Build a borrowed `QueryFilter<'a>` for upstream calls.
            ///
            /// `pred` is the predicate closure produced by the caller, kept
            /// alive on the stack for the duration of one query call.
            pub fn as_rapier<'a>(
                &self,
                pred: Option<&'a dyn Fn(rapier::geometry::ColliderHandle, &rapier::geometry::Collider) -> bool>,
            ) -> rapier::pipeline::QueryFilter<'a> {
                let mut qf = rapier::pipeline::QueryFilter {
                    flags: self.flags,
                    groups: self.groups,
                    exclude_collider: self.exclude_collider,
                    exclude_rigid_body: self.exclude_rigid_body,
                    predicate: None,
                };
                if let Some(p) = pred {
                    qf.predicate = Some(p);
                }
                qf
            }
        }

        #[pymethods]
        impl QueryFilter {
            /// Construct a ``QueryFilter`` from explicit field values.
            ///
            /// :param flags: :class:`QueryFilterFlags` (defaults to empty).
            /// :param groups: :class:`InteractionGroups` membership/filter.
            /// :param exclude_collider: Single collider to skip.
            /// :param exclude_rigid_body: Skip every collider attached to
            ///     this rigid body.
            /// :param predicate: Optional ``f(ColliderHandle, Collider) -> bool``
            ///     per-collider predicate; returning ``False`` skips the
            ///     collider.
            #[new]
            #[pyo3(signature = (flags=None, groups=None, exclude_collider=None, exclude_rigid_body=None, predicate=None))]
            fn py_new(
                flags: Option<&QueryFilterFlags>,
                groups: Option<&InteractionGroups>,
                exclude_collider: Option<&ColliderHandle>,
                exclude_rigid_body: Option<&RigidBodyHandle>,
                predicate: Option<Py<PyAny>>,
            ) -> Self {
                Self {
                    flags: flags.map(|f| f.0).unwrap_or_else(rapier::pipeline::QueryFilterFlags::empty),
                    groups: groups.map(|g| g.0),
                    exclude_collider: exclude_collider.map(|h| h.0),
                    exclude_rigid_body: exclude_rigid_body.map(|h| h.0),
                    predicate,
                }
            }

            /// Return an empty filter (no flags, no groups, no exclusions).
            #[staticmethod]
            fn new() -> Self { Self::default_empty() }

            /// Return a filter that excludes colliders attached to fixed bodies.
            #[staticmethod]
            fn exclude_fixed() -> Self {
                let mut me = Self::default_empty();
                me.flags = rapier::pipeline::QueryFilterFlags::EXCLUDE_FIXED;
                me
            }
            /// Return a filter that excludes colliders attached to kinematic bodies.
            #[staticmethod]
            fn exclude_kinematic() -> Self {
                let mut me = Self::default_empty();
                me.flags = rapier::pipeline::QueryFilterFlags::EXCLUDE_KINEMATIC;
                me
            }
            /// Return a filter that excludes colliders attached to dynamic bodies.
            #[staticmethod]
            fn exclude_dynamic() -> Self {
                let mut me = Self::default_empty();
                me.flags = rapier::pipeline::QueryFilterFlags::EXCLUDE_DYNAMIC;
                me
            }
            /// Return a filter keeping only colliders attached to dynamic bodies.
            #[staticmethod]
            fn only_dynamic() -> Self {
                let mut me = Self::default_empty();
                me.flags = rapier::pipeline::QueryFilterFlags::ONLY_DYNAMIC;
                me
            }
            /// Return a filter keeping only colliders attached to kinematic bodies.
            #[staticmethod]
            fn only_kinematic() -> Self {
                let mut me = Self::default_empty();
                me.flags = rapier::pipeline::QueryFilterFlags::ONLY_KINEMATIC;
                me
            }
            /// Return a filter keeping only colliders attached to fixed bodies.
            #[staticmethod]
            fn only_fixed() -> Self {
                let mut me = Self::default_empty();
                me.flags = rapier::pipeline::QueryFilterFlags::ONLY_FIXED;
                me
            }

            /// Builder: also exclude sensor colliders.
            ///
            /// :returns: A new :class:`QueryFilter` with ``EXCLUDE_SENSORS``
            ///     added.
            fn exclude_sensors(&self, py: Python<'_>) -> Self {
                let mut me = self.clone_with(py);
                me.flags |= rapier::pipeline::QueryFilterFlags::EXCLUDE_SENSORS;
                me
            }
            /// Builder: also exclude non-sensor (solid) colliders.
            ///
            /// :returns: A new :class:`QueryFilter` with ``EXCLUDE_SOLIDS``
            ///     added.
            fn exclude_solids(&self, py: Python<'_>) -> Self {
                let mut me = self.clone_with(py);
                me.flags |= rapier::pipeline::QueryFilterFlags::EXCLUDE_SOLIDS;
                me
            }
            /// Builder: restrict to colliders whose interaction groups overlap ``groups``.
            ///
            /// :param groups: Membership / filter masks.
            /// :returns: A new :class:`QueryFilter`.
            fn groups(&self, py: Python<'_>, groups: &InteractionGroups) -> Self {
                let mut me = self.clone_with(py);
                me.groups = Some(groups.0);
                me
            }
            /// Builder: skip this specific collider.
            ///
            /// :param h: Collider handle to exclude.
            /// :returns: A new :class:`QueryFilter`.
            fn exclude_collider(&self, py: Python<'_>, h: &ColliderHandle) -> Self {
                let mut me = self.clone_with(py);
                me.exclude_collider = Some(h.0);
                me
            }
            /// Builder: skip every collider attached to this body.
            ///
            /// :param h: Rigid-body handle whose colliders to exclude.
            /// :returns: A new :class:`QueryFilter`.
            fn exclude_rigid_body(&self, py: Python<'_>, h: &RigidBodyHandle) -> Self {
                let mut me = self.clone_with(py);
                me.exclude_rigid_body = Some(h.0);
                me
            }
            /// Builder: install a per-collider predicate.
            ///
            /// The predicate is invoked with ``(ColliderHandle, Collider)``
            /// and must return a ``bool``; ``False`` skips the collider.
            ///
            /// :param fun: Python callable.
            /// :returns: A new :class:`QueryFilter`.
            fn predicate(&self, py: Python<'_>, fun: Py<PyAny>) -> Self {
                let mut me = self.clone_with(py);
                me.predicate = Some(fun.clone_ref(py));
                me
            }

            /// Current :class:`QueryFilterFlags`.
            #[getter] fn get_flags(&self) -> QueryFilterFlags { QueryFilterFlags(self.flags) }
            /// Current :class:`InteractionGroups`, or ``None``.
            #[getter] fn get_groups(&self) -> Option<InteractionGroups> {
                self.groups.map(InteractionGroups)
            }
            /// Handle of the explicitly excluded collider, or ``None``.
            #[getter]
            #[pyo3(name = "exclude_collider_handle")]
            fn get_exclude_collider(&self) -> Option<ColliderHandle> {
                self.exclude_collider.map(ColliderHandle)
            }
            /// Handle of the rigid body whose colliders are excluded, or ``None``.
            #[getter]
            #[pyo3(name = "exclude_rigid_body_handle")]
            fn get_exclude_rigid_body(&self) -> Option<RigidBodyHandle> {
                self.exclude_rigid_body.map(RigidBodyHandle)
            }
            /// Current per-collider predicate, or ``None``.
            #[getter] fn get_predicate(&self, py: Python<'_>) -> Option<PyObject> {
                self.predicate.as_ref().map(|p| p.clone_ref(py))
            }

            /// Debug repr — shows all filter fields.
            fn __repr__(&self) -> String {
                format!(
                    "QueryFilter(flags={:#x}, groups={:?}, exclude_collider={:?}, exclude_rigid_body={:?}, predicate={})",
                    self.flags.bits(),
                    self.groups.map(|g| (g.memberships.bits(), g.filter.bits())),
                    self.exclude_collider.map(|h| h.into_raw_parts()),
                    self.exclude_rigid_body.map(|h| h.into_raw_parts()),
                    if self.predicate.is_some() { "<py callable>" } else { "None" },
                )
            }
        }

        impl QueryFilter {
            fn default_empty() -> Self {
                Self {
                    flags: rapier::pipeline::QueryFilterFlags::empty(),
                    groups: None,
                    exclude_collider: None,
                    exclude_rigid_body: None,
                    predicate: None,
                }
            }
        }

        // =====================================================================
        // QueryPipeline — built on demand from the world's broad_phase.
        //
        // We expose two views: an immutable `QueryPipeline` and a mutable
        // `QueryPipelineMut`. Both share the same query implementations.
        // =====================================================================

        /// Scene-query accelerator — a view over the broad-phase BVH.
        ///
        /// A :class:`QueryPipeline` holds shared references to the world's
        /// :class:`BroadPhaseBvh`, :class:`NarrowPhase`, :class:`RigidBodySet`,
        /// and :class:`ColliderSet`. Each query method (``cast_ray``,
        /// ``cast_shape``, ``intersect_*``, ``project_point*``) re-borrows
        /// those sets, builds the underlying parry ``QueryPipeline``, and
        /// runs the requested query.
        ///
        /// **Validity:** the pipeline reflects the state of the broad-phase
        /// BVH as of the last :meth:`update` (or
        /// :meth:`PhysicsWorld.step` when ``auto_update_query=True``).
        /// After mutating the collider set or moving bodies, call
        /// :meth:`update` to refresh the BVH before issuing queries.
        #[pyclass(name = "QueryPipeline", module = "rapier", unsendable)]
        pub struct QueryPipeline {
            /// The "owned" BVH that backs scene queries.
            pub broad_phase: Py<BroadPhaseBvh>,
            pub narrow_phase: Py<NarrowPhase>,
            pub bodies: Py<RigidBodySet>,
            pub colliders: Py<ColliderSet>,
        }

        #[pymethods]
        impl QueryPipeline {
            /// Construct a query pipeline from explicit sub-set references.
            ///
            /// Usually you'll use :attr:`PhysicsWorld.query_pipeline` instead
            /// of constructing this directly.
            ///
            /// :param broad_phase: Broad-phase BVH backing the queries.
            /// :param narrow_phase: Narrow-phase store (for the query dispatcher).
            /// :param bodies: Rigid-body set referenced by collider parents.
            /// :param colliders: Collider set the queries operate on.
            #[new]
            fn new(
                broad_phase: Py<BroadPhaseBvh>,
                narrow_phase: Py<NarrowPhase>,
                bodies: Py<RigidBodySet>,
                colliders: Py<ColliderSet>,
            ) -> Self {
                Self { broad_phase, narrow_phase, bodies, colliders }
            }

            /// Re-build the broad-phase BVH from the current body/collider state.
            ///
            /// Must be called after inserting/removing colliders or moving
            /// bodies before issuing queries — otherwise the queries see a
            /// stale BVH. :meth:`PhysicsWorld.step` calls this automatically
            /// when ``world.auto_update_query`` is ``True``; the manual
            /// variant lets you avoid a redundant rebuild when the BVH is
            /// already up to date.
            ///
            /// :param bodies: Rigid-body set (accepted for API parity; the
            ///     pipeline uses its stored reference).
            /// :param colliders: Collider set (same caveat as ``bodies``).
            fn update(
                &self,
                py: Python<'_>,
                bodies: Py<RigidBodySet>,
                colliders: Py<ColliderSet>,
            ) -> PyResult<()> {
                let _ = (bodies, colliders); // accepted for API parity
                let mut bp = self.broad_phase.borrow_mut(py);
                let bodies = self.bodies.borrow(py);
                let mut colliders = self.colliders.borrow_mut(py);
                let params = rapier::dynamics::IntegrationParameters::default();
                let modified = colliders.0.take_modified();
                let removed = colliders.0.take_removed();
                let mut events: Vec<rapier::geometry::BroadPhasePairEvent> = Vec::new();
                bp.0.update(&params, &colliders.0, &bodies.0, &modified, &removed, &mut events);
                Ok(())
            }

            /// Cast a ray and return the closest hit (TOI only).
            ///
            /// :param ray: Ray to cast.
            /// :param max_toi: Upper bound on the returned TOI.
            /// :param solid: If ``True``, a ray that starts inside a solid
            ///     shape hits at ``t = 0``; if ``False``, the ray exits
            ///     through the shape's back face.
            /// :param filter: Optional :class:`QueryFilter`.
            /// :returns: ``(ColliderHandle, toi)`` or ``None`` if no hit.
            #[pyo3(signature = (ray, max_toi, solid, filter=None))]
            fn cast_ray(
                &self,
                py: Python<'_>,
                ray: &Ray,
                max_toi: Real,
                solid: bool,
                filter: Option<&QueryFilter>,
            ) -> PyResult<Option<(ColliderHandle, Real)>> {
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    Ok(qp.cast_ray(&ray.0, max_toi, solid)
                        .map(|(h, t)| (ColliderHandle(h), t)))
                })
            }

            /// Cast a ray and return the closest hit with surface geometry.
            ///
            /// :param ray: Ray to cast.
            /// :param max_toi: Upper bound on the returned TOI.
            /// :param solid: See :meth:`cast_ray`.
            /// :param filter: Optional :class:`QueryFilter`.
            /// :returns: ``(ColliderHandle, RayIntersection)`` or ``None``.
            #[pyo3(signature = (ray, max_toi, solid, filter=None))]
            fn cast_ray_and_get_normal(
                &self,
                py: Python<'_>,
                ray: &Ray,
                max_toi: Real,
                solid: bool,
                filter: Option<&QueryFilter>,
            ) -> PyResult<Option<(ColliderHandle, RayIntersection)>> {
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    Ok(qp.cast_ray_and_get_normal(&ray.0, max_toi, solid)
                        .map(|(h, ri)| (ColliderHandle(h), RayIntersection::from_parry(ri))))
                })
            }

            /// Iterate over every collider the ray passes through.
            ///
            /// The Python callback is invoked with ``(ColliderHandle,
            /// RayIntersection)`` for each hit. Return ``True`` (or any
            /// non-bool) to continue, ``False`` to stop iteration.
            ///
            /// :param ray: Ray to cast.
            /// :param max_toi: Upper bound on the hit TOI.
            /// :param solid: See :meth:`cast_ray`.
            /// :param callback: ``(ColliderHandle, RayIntersection) -> bool``.
            /// :param filter: Optional :class:`QueryFilter`.
            #[pyo3(signature = (ray, max_toi, solid, callback, filter=None))]
            fn intersect_ray(
                &self,
                py: Python<'_>,
                ray: &Ray,
                max_toi: Real,
                solid: bool,
                callback: Py<PyAny>,
                filter: Option<&QueryFilter>,
            ) -> PyResult<()> {
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    for (h, _co, ri) in qp.intersect_ray(ray.0, max_toi, solid) {
                        let py_ri = RayIntersection::from_parry(ri);
                        let res = callback.call1(py, (ColliderHandle(h), py_ri))?;
                        if let Ok(b) = res.extract::<bool>(py) {
                            if !b { break; }
                        }
                    }
                    Ok(())
                })
            }

            /// Find the collider whose shape is closest to ``point``.
            ///
            /// :param point: World-space query point.
            /// :param solid: If ``True`` a point inside a solid shape
            ///     projects to itself (distance ``0``); if ``False`` it
            ///     projects to the shape's boundary.
            /// :param filter: Optional :class:`QueryFilter`.
            /// :returns: ``(ColliderHandle, PointProjection)`` or ``None`` if
            ///     no collider passed the filter.
            #[pyo3(signature = (point, solid, filter=None))]
            fn project_point(
                &self,
                py: Python<'_>,
                point: PyPoint,
                solid: bool,
                filter: Option<&QueryFilter>,
            ) -> PyResult<Option<(ColliderHandle, PointProjection)>> {
                let p: rapier::math::Vector = point.0.coords.into();
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    Ok(qp.project_point(p, Real::MAX, solid)
                        .map(|(h, pp)| (ColliderHandle(h), PointProjection::from_parry(pp))))
                })
            }

            /// Like :meth:`project_point`, but also return the touched shape feature.
            ///
            /// :param point: World-space query point.
            /// :param max_dist: Largest distance to search; colliders farther
            ///     than this from ``point`` are ignored. Defaults to unbounded.
            /// :param filter: Optional :class:`QueryFilter`.
            /// :returns: ``(ColliderHandle, PointProjection, FeatureId)`` or
            ///     ``None``.
            #[pyo3(signature = (point, max_dist=None, filter=None))]
            fn project_point_and_get_feature(
                &self,
                py: Python<'_>,
                point: PyPoint,
                max_dist: Option<Real>,
                filter: Option<&QueryFilter>,
            ) -> PyResult<Option<(ColliderHandle, PointProjection, FeatureId)>> {
                let p: rapier::math::Vector = point.0.coords.into();
                let max_dist = max_dist.unwrap_or(Real::MAX);
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    Ok(qp.project_point_and_get_feature(p, max_dist)
                        .map(|(h, pp, fi)| (
                            ColliderHandle(h),
                            PointProjection::from_parry(pp),
                            FeatureId::from_parry(fi),
                        )))
                })
            }

            /// Invoke ``callback(ColliderHandle)`` for every collider containing ``point``.
            ///
            /// Return ``True`` (or any non-bool) from the callback to
            /// continue, ``False`` to stop iteration.
            ///
            /// :param point: World-space query point.
            /// :param callback: ``(ColliderHandle) -> bool``.
            /// :param filter: Optional :class:`QueryFilter`.
            #[pyo3(signature = (point, callback, filter=None))]
            fn intersect_point(
                &self,
                py: Python<'_>,
                point: PyPoint,
                callback: Py<PyAny>,
                filter: Option<&QueryFilter>,
            ) -> PyResult<()> {
                let p: rapier::math::Vector = point.0.coords.into();
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    for (h, _co) in qp.intersect_point(p) {
                        let res = callback.call1(py, (ColliderHandle(h),))?;
                        if let Ok(b) = res.extract::<bool>(py) {
                            if !b { break; }
                        }
                    }
                    Ok(())
                })
            }

            /// Linear shape cast — sweep ``shape`` along ``shape_vel`` from ``shape_pose``.
            ///
            /// :param shape_pose: Initial pose of the cast shape.
            /// :param shape_vel: Linear velocity vector (units per unit time).
            /// :param shape: Shape to sweep.
            /// :param options: :class:`ShapeCastOptions` (TOI bound, target
            ///     distance, penetration handling).
            /// :param filter: Optional :class:`QueryFilter`.
            /// :returns: ``(ColliderHandle, ShapeCastHit)`` for the earliest
            ///     hit, or ``None``.
            #[pyo3(signature = (shape_pose, shape_vel, shape, options, filter=None))]
            fn cast_shape(
                &self,
                py: Python<'_>,
                shape_pose: PyIsometry,
                shape_vel: PyVector,
                shape: &SharedShape,
                options: &ShapeCastOptions,
                filter: Option<&QueryFilter>,
            ) -> PyResult<Option<(ColliderHandle, ShapeCastHit)>> {
                let pose: rapier::math::Pose = shape_pose.0.into();
                let vel: rapier::math::Vector = shape_vel.0.into();
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    Ok(qp.cast_shape(&pose, vel, &*shape.0.0, options.0)
                        .map(|(h, hit)| (ColliderHandle(h), ShapeCastHit::from_parry(hit))))
                })
            }

            /// Nonlinear shape cast — sweep ``shape`` along a curved :class:`NonlinearRigidMotion`.
            ///
            /// Use this when the moving body rotates or has an offset center
            /// (e.g. a hammer head swinging around a wrist), which a straight
            /// :meth:`cast_shape` would miss.
            ///
            /// :param motion: Curved motion to sweep along.
            /// :param shape: Shape to sweep.
            /// :param options: :class:`ShapeCastOptions` — only
            ///     ``stop_at_penetration`` is honored by this query.
            /// :param start_time: Lower bound on the cast parameter.
            /// :param end_time: Upper bound on the cast parameter.
            /// :param filter: Optional :class:`QueryFilter`.
            /// :returns: ``(ColliderHandle, ShapeCastHit)`` or ``None``.
            #[pyo3(signature = (motion, shape, options, start_time=0.0 as Real, end_time=Real::MAX, filter=None))]
            #[allow(clippy::too_many_arguments)]
            fn cast_shape_nonlinear(
                &self,
                py: Python<'_>,
                motion: &NonlinearRigidMotion,
                shape: &SharedShape,
                options: &ShapeCastOptions,
                start_time: Real,
                end_time: Real,
                filter: Option<&QueryFilter>,
            ) -> PyResult<Option<(ColliderHandle, ShapeCastHit)>> {
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    Ok(qp.cast_shape_nonlinear(
                        &motion.0,
                        &*shape.0.0,
                        start_time,
                        end_time,
                        options.0.stop_at_penetration,
                    ).map(|(h, hit)| (ColliderHandle(h), ShapeCastHit::from_parry(hit))))
                })
            }

            /// Invoke ``callback(ColliderHandle)`` for each collider intersecting ``shape`` at ``shape_pose``.
            ///
            /// Return ``True`` (or any non-bool) from the callback to
            /// continue, ``False`` to stop iteration.
            ///
            /// :param shape_pose: World pose of the test shape.
            /// :param shape: Test shape.
            /// :param callback: ``(ColliderHandle) -> bool``.
            /// :param filter: Optional :class:`QueryFilter`.
            #[pyo3(signature = (shape_pose, shape, callback, filter=None))]
            fn intersect_shape(
                &self,
                py: Python<'_>,
                shape_pose: PyIsometry,
                shape: &SharedShape,
                callback: Py<PyAny>,
                filter: Option<&QueryFilter>,
            ) -> PyResult<()> {
                let pose: rapier::math::Pose = shape_pose.0.into();
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    for (h, _co) in qp.intersect_shape(pose, &*shape.0.0) {
                        let res = callback.call1(py, (ColliderHandle(h),))?;
                        if let Ok(b) = res.extract::<bool>(py) {
                            if !b { break; }
                        }
                    }
                    Ok(())
                })
            }

            /// Conservative AABB sweep — invoke ``callback(ColliderHandle)`` for every collider whose AABB intersects ``aabb``.
            ///
            /// The test is conservative: a candidate's AABB may overlap the
            /// query AABB even when the underlying shapes don't (i.e. false
            /// positives are possible — there are no false negatives).
            /// Return ``False`` from the callback to stop iteration.
            ///
            /// :param aabb: Query AABB.
            /// :param callback: ``(ColliderHandle) -> bool``.
            /// :param filter: Optional :class:`QueryFilter`.
            #[pyo3(signature = (aabb, callback, filter=None))]
            fn intersect_aabb_conservative(
                &self,
                py: Python<'_>,
                aabb: &Aabb,
                callback: Py<PyAny>,
                filter: Option<&QueryFilter>,
            ) -> PyResult<()> {
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    for (h, _co) in qp.intersect_aabb_conservative(aabb.0) {
                        let res = callback.call1(py, (ColliderHandle(h),))?;
                        if let Ok(b) = res.extract::<bool>(py) {
                            if !b { break; }
                        }
                    }
                    Ok(())
                })
            }

            /// Alias for :meth:`intersect_aabb_conservative`.
            #[pyo3(signature = (aabb, callback, filter=None))]
            fn colliders_with_aabb_intersecting_aabb(
                &self,
                py: Python<'_>,
                aabb: &Aabb,
                callback: Py<PyAny>,
                filter: Option<&QueryFilter>,
            ) -> PyResult<()> {
                self.intersect_aabb_conservative(py, aabb, callback, filter)
            }

            /// Return ``True`` iff any collider's AABB intersects ``aabb``.
            ///
            /// Conservative test (see :meth:`intersect_aabb_conservative`).
            ///
            /// :param aabb: Query AABB.
            /// :param filter: Optional :class:`QueryFilter`.
            #[pyo3(signature = (aabb, filter=None))]
            fn test_aabb(
                &self,
                py: Python<'_>,
                aabb: &Aabb,
                filter: Option<&QueryFilter>,
            ) -> PyResult<bool> {
                $crate::__qp_query!(self, py, filter, |qp: rapier::pipeline::QueryPipeline<'_>| {
                    Ok(qp.intersect_aabb_conservative(aabb.0).next().is_some())
                })
            }
        }

        // =====================================================================
        // Counters and its sub-structs
        // =====================================================================

        /// Read-only per-stage timing breakdown for a single ``step()``.
        ///
        /// All values are in milliseconds and are only populated when the
        /// owning :class:`Counters` is enabled via :meth:`Counters.enable`.
        #[pyclass(name = "StagesCounters", module = "rapier", frozen)]
        #[derive(Clone, Copy)]
        pub struct StagesCounters(pub rapier::counters::StagesCounters);

        #[pymethods]
        impl StagesCounters {
            /// Time spent updating broad-phase / collider state, in ms.
            #[getter] fn update_time_ms(&self) -> f64 { self.0.update_time.time_ms() }
            /// Total collision-detection time (broad + narrow phase), in ms.
            #[getter] fn collision_detection_time_ms(&self) -> f64 { self.0.collision_detection_time.time_ms() }
            /// Time spent grouping bodies into solver islands, in ms.
            #[getter] fn island_construction_time_ms(&self) -> f64 { self.0.island_construction_time.time_ms() }
            /// Time spent assembling per-island constraint data, in ms.
            #[getter] fn island_constraints_collection_time_ms(&self) -> f64 { self.0.island_constraints_collection_time.time_ms() }
            /// Constraint-solver time, in ms.
            #[getter] fn solver_time_ms(&self) -> f64 { self.0.solver_time.time_ms() }
            /// Continuous collision detection time, in ms.
            #[getter] fn ccd_time_ms(&self) -> f64 { self.0.ccd_time.time_ms() }
            /// Time spent applying user-modified body / collider state, in ms.
            #[getter] fn user_changes_time_ms(&self) -> f64 { self.0.user_changes.time_ms() }
            /// Debug repr — shows the dominant per-stage times.
            fn __repr__(&self) -> String {
                format!("StagesCounters(solver={}ms, collision_detection={}ms)",
                    self.0.solver_time.time_ms(),
                    self.0.collision_detection_time.time_ms())
            }
        }

        /// Read-only collision-detection counters for a single ``step()``.
        ///
        /// Only populated when the owning :class:`Counters` is enabled.
        #[pyclass(name = "CollisionDetectionCounters", module = "rapier", frozen)]
        #[derive(Clone, Copy)]
        pub struct CollisionDetectionCounters(pub rapier::counters::CollisionDetectionCounters);

        #[pymethods]
        impl CollisionDetectionCounters {
            /// Number of contact pairs detected.
            #[getter] fn ncontact_pairs(&self) -> usize { self.0.ncontact_pairs }
            /// Broad-phase time, in ms.
            #[getter] fn broad_phase_time_ms(&self) -> f64 { self.0.broad_phase_time.time_ms() }
            /// Narrow-phase time, in ms.
            #[getter] fn narrow_phase_time_ms(&self) -> f64 { self.0.narrow_phase_time.time_ms() }
            /// Final broad-phase pass time (post-CCD), in ms.
            #[getter] fn final_broad_phase_time_ms(&self) -> f64 { self.0.final_broad_phase_time.time_ms() }
        }

        /// Read-only solver counters for a single ``step()``.
        ///
        /// Only populated when the owning :class:`Counters` is enabled.
        #[pyclass(name = "SolverCounters", module = "rapier", frozen)]
        #[derive(Clone, Copy)]
        pub struct SolverCounters(pub rapier::counters::SolverCounters);

        #[pymethods]
        impl SolverCounters {
            /// Number of solver constraints assembled.
            #[getter] fn nconstraints(&self) -> usize { self.0.nconstraints }
            /// Number of contacts driven by the solver.
            #[getter] fn ncontacts(&self) -> usize { self.0.ncontacts }
            /// Velocity-resolution (impulse) time, in ms.
            #[getter] fn velocity_resolution_time_ms(&self) -> f64 { self.0.velocity_resolution_time.time_ms() }
            /// Velocity-constraint assembly time, in ms.
            #[getter] fn velocity_assembly_time_ms(&self) -> f64 { self.0.velocity_assembly_time.time_ms() }
            /// Post-solve velocity-integration time, in ms.
            #[getter] fn velocity_update_time_ms(&self) -> f64 { self.0.velocity_update_time.time_ms() }
        }

        /// Read-only continuous-collision-detection (CCD) counters.
        ///
        /// Only populated when the owning :class:`Counters` is enabled.
        #[pyclass(name = "CCDCounters", module = "rapier", frozen)]
        #[derive(Clone, Copy)]
        pub struct CCDCounters(pub rapier::counters::CCDCounters);

        #[pymethods]
        impl CCDCounters {
            /// Number of CCD substeps taken this step.
            #[getter] fn num_substeps(&self) -> usize { self.0.num_substeps }
            /// Time spent computing time-of-impact, in ms.
            #[getter] fn toi_computation_time_ms(&self) -> f64 { self.0.toi_computation_time.time_ms() }
            /// Per-substep solver time, in ms.
            #[getter] fn solver_time_ms(&self) -> f64 { self.0.solver_time.time_ms() }
            /// Per-substep broad-phase time, in ms.
            #[getter] fn broad_phase_time_ms(&self) -> f64 { self.0.broad_phase_time.time_ms() }
            /// Per-substep narrow-phase time, in ms.
            #[getter] fn narrow_phase_time_ms(&self) -> f64 { self.0.narrow_phase_time.time_ms() }
        }

        /// Aggregate read-only timing counters from a :class:`PhysicsPipeline`.
        ///
        /// Disabled by default — call :meth:`enable` before stepping the
        /// pipeline to collect data, then read the per-stage sub-counters
        /// (:attr:`stages`, :attr:`cd`, :attr:`solver`, :attr:`ccd`).
        ///
        /// Timing is best-effort and may be zero on platforms without a
        /// high-resolution clock.
        #[pyclass(name = "Counters", module = "rapier")]
        #[derive(Clone, Copy)]
        pub struct Counters(pub rapier::counters::Counters);

        #[pymethods]
        impl Counters {
            /// Construct a disabled ``Counters``. Call :meth:`enable` to start collecting.
            #[new]
            fn new() -> Self { Self(rapier::counters::Counters::new(false)) }

            /// Enable timing collection on the next ``step()``.
            fn enable(&mut self) { self.0.enable() }
            /// Disable timing collection.
            fn disable(&mut self) { self.0.disable() }
            /// Zero out every sub-counter.
            fn reset(&mut self) { self.0.reset() }
            /// ``True`` iff timing collection is currently active.
            #[getter] fn enabled(&self) -> bool { self.0.enabled() }
            /// Total time for the last ``step()`` call, in ms.
            #[getter] fn step_time_ms(&self) -> f64 { self.0.step_time_ms() }
            /// Time accumulated under the "custom" stage hook, in ms.
            #[getter] fn custom_time_ms(&self) -> f64 { self.0.custom_time_ms() }
            /// Per-stage breakdown (see :class:`StagesCounters`).
            #[getter] fn stages(&self) -> StagesCounters { StagesCounters(self.0.stages) }
            /// Collision-detection breakdown (see :class:`CollisionDetectionCounters`).
            #[getter] fn cd(&self) -> CollisionDetectionCounters { CollisionDetectionCounters(self.0.cd) }
            /// Solver breakdown (see :class:`SolverCounters`).
            #[getter] fn solver(&self) -> SolverCounters { SolverCounters(self.0.solver) }
            /// CCD breakdown (see :class:`CCDCounters`).
            #[getter] fn ccd(&self) -> CCDCounters { CCDCounters(self.0.ccd) }
            /// Pretty-print the entire counter tree to stdout.
            #[pyo3(name = "print")]
            fn py_print(&self) {
                use std::println;
                println!("{}", self.0);
            }
            /// Debug repr — shows enabled state and total step time.
            fn __repr__(&self) -> String {
                format!("Counters(enabled={}, step_time_ms={})",
                    self.0.enabled(), self.0.step_time_ms())
            }
        }

        // =====================================================================
        // PhysicsPipeline
        // =====================================================================

        /// Low-level physics-step driver.
        ///
        /// The :class:`PhysicsPipeline` owns the persistent solver state but
        /// requires you to hand it every sub-set (bodies, colliders, islands,
        /// broad / narrow phase, joints, CCD solver) on each call to
        /// :meth:`step`. For most users the higher-level :class:`PhysicsWorld`
        /// aggregates these and is more convenient.
        #[pyclass(name = "PhysicsPipeline", module = "rapier", unsendable)]
        pub struct PhysicsPipeline(pub rapier::pipeline::PhysicsPipeline);

        #[pymethods]
        impl PhysicsPipeline {
            /// Construct a fresh :class:`PhysicsPipeline`.
            #[new]
            fn new() -> Self { Self(rapier::pipeline::PhysicsPipeline::new()) }

            /// Read-only :class:`Counters` populated by the last :meth:`step`.
            ///
            /// Enable timing on the pipeline before stepping to get non-zero
            /// numbers.
            #[getter]
            fn counters(&self) -> Counters { Counters(self.0.counters) }

            /// Advance the simulation by one step.
            ///
            /// Releases the GIL via ``Python::allow_threads`` while the
            /// solver runs, so other Python threads can make progress.
            /// Hooks and event-handler callbacks re-acquire the GIL
            /// transparently before touching Python objects; any exception
            /// raised inside a callback is captured and re-raised after
            /// :meth:`step` returns.
            ///
            /// :param gravity: World-space gravity vector.
            /// :param integration_parameters: Solver tuning parameters.
            /// :param islands: Body island manager (mutated).
            /// :param broad_phase: Broad-phase BVH (mutated).
            /// :param narrow_phase: Narrow-phase store (mutated).
            /// :param bodies: Rigid-body set (mutated).
            /// :param colliders: Collider set (mutated).
            /// :param impulse_joints: Impulse-joint set (mutated).
            /// :param multibody_joints: Multibody-joint set (mutated).
            /// :param ccd_solver: CCD solver state (mutated).
            /// :param hooks: Optional ``PhysicsHooks``-protocol object.
            /// :param events: Optional ``EventHandler``-protocol object
            ///     (e.g. :class:`ChannelEventCollector`).
            /// :raises: Any exception raised from a Python hook / event
            ///     callback is re-raised after the step completes.
            #[pyo3(signature = (
                gravity,
                integration_parameters,
                islands,
                broad_phase,
                narrow_phase,
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                ccd_solver,
                hooks=None,
                events=None,
            ))]
            #[allow(clippy::too_many_arguments)]
            fn step(
                &mut self,
                py: Python<'_>,
                gravity: PyVector,
                integration_parameters: &IntegrationParameters,
                islands: &mut IslandManager,
                broad_phase: &mut BroadPhaseBvh,
                narrow_phase: &mut NarrowPhase,
                bodies: &mut RigidBodySet,
                colliders: &mut ColliderSet,
                impulse_joints: &mut ImpulseJointSet,
                multibody_joints: &mut MultibodyJointSet,
                ccd_solver: &mut CCDSolver,
                hooks: Option<&Bound<'_, PyAny>>,
                events: Option<&Bound<'_, PyAny>>,
            ) -> PyResult<()> {
                let hooks_obj: Option<Py<PyAny>> =
                    hooks.and_then(|h| if h.is_none() { None } else { Some(h.clone().unbind()) });
                let events_obj: Option<Py<PyAny>> =
                    events.and_then(|e| if e.is_none() { None } else { Some(e.clone().unbind()) });
                let err_slot = std::sync::Arc::new(std::sync::Mutex::new(DeferredError::default()));
                let hooks_box = build_physics_hooks(py, hooks_obj.as_ref(), err_slot.clone());
                let events_box = build_event_handler(py, events_obj.as_ref(), err_slot.clone());

                let g: rapier::math::Vector = gravity.0.into();
                let pp_inner = &mut self.0;
                let ip_inner = &integration_parameters.0;
                let islands_inner = &mut islands.0;
                let bp_inner = &mut broad_phase.0;
                let np_inner = &mut narrow_phase.0;
                let bodies_inner = &mut bodies.0;
                let colliders_inner = &mut colliders.0;
                let ij_inner = &mut impulse_joints.0;
                let mj_inner = &mut multibody_joints.0;
                let ccd_inner = &mut ccd_solver.0;
                let hooks_ref: &dyn rapier::pipeline::PhysicsHooks = match hooks_box.as_deref() {
                    Some(h) => h,
                    None => &(),
                };
                let events_ref: &dyn rapier::pipeline::EventHandler = match events_box.as_deref() {
                    Some(e) => e,
                    None => &(),
                };
                py.allow_threads(|| {
                    pp_inner.step(
                        g,
                        ip_inner,
                        islands_inner,
                        bp_inner,
                        np_inner,
                        bodies_inner,
                        colliders_inner,
                        ij_inner,
                        mj_inner,
                        ccd_inner,
                        hooks_ref,
                        events_ref,
                    );
                });
                drop(hooks_box);
                drop(events_box);
                let mut slot = err_slot.lock().unwrap();
                if let Some(e) = slot.err.take() {
                    return Err(e);
                }
                Ok(())
            }
        }

        // =====================================================================
        // CollisionPipeline
        // =====================================================================

        /// Collision-only step driver (no dynamics).
        ///
        /// A reduced :class:`PhysicsPipeline` that runs the broad / narrow
        /// phases and emits collision / contact-force events but **skips**
        /// joint and constraint solving. Useful for static scene queries,
        /// trigger evaluation, or driving custom controllers.
        #[pyclass(name = "CollisionPipeline", module = "rapier", unsendable)]
        pub struct CollisionPipeline(pub rapier::pipeline::CollisionPipeline);

        #[pymethods]
        impl CollisionPipeline {
            /// Construct a fresh :class:`CollisionPipeline`.
            #[new]
            fn new() -> Self { Self(rapier::pipeline::CollisionPipeline::new()) }

            /// Run a single collision-detection pass.
            ///
            /// Releases the GIL while running; hook / event callbacks
            /// re-acquire it on demand. See :meth:`PhysicsPipeline.step` for
            /// the threading and error-propagation contract.
            ///
            /// :param prediction_distance: Distance used to inflate AABBs
            ///     when looking for new contact pairs.
            /// :param islands: Body island manager.
            /// :param broad_phase: Broad-phase BVH (mutated).
            /// :param narrow_phase: Narrow-phase store (mutated).
            /// :param bodies: Rigid-body set (mutated).
            /// :param colliders: Collider set (mutated).
            /// :param hooks: Optional ``PhysicsHooks``-protocol object.
            /// :param events: Optional ``EventHandler``-protocol object.
            #[pyo3(signature = (
                prediction_distance,
                islands,
                broad_phase,
                narrow_phase,
                bodies,
                colliders,
                hooks=None,
                events=None,
            ))]
            #[allow(clippy::too_many_arguments)]
            fn step(
                &mut self,
                py: Python<'_>,
                prediction_distance: Real,
                islands: &mut IslandManager,
                broad_phase: &mut BroadPhaseBvh,
                narrow_phase: &mut NarrowPhase,
                bodies: &mut RigidBodySet,
                colliders: &mut ColliderSet,
                hooks: Option<&Bound<'_, PyAny>>,
                events: Option<&Bound<'_, PyAny>>,
            ) -> PyResult<()> {
                let hooks_obj: Option<Py<PyAny>> =
                    hooks.and_then(|h| if h.is_none() { None } else { Some(h.clone().unbind()) });
                let events_obj: Option<Py<PyAny>> =
                    events.and_then(|e| if e.is_none() { None } else { Some(e.clone().unbind()) });
                let err_slot = std::sync::Arc::new(std::sync::Mutex::new(DeferredError::default()));
                let hooks_box = build_physics_hooks(py, hooks_obj.as_ref(), err_slot.clone());
                let events_box = build_event_handler(py, events_obj.as_ref(), err_slot.clone());

                let cp_inner = &mut self.0;
                let islands_inner = &mut islands.0;
                let bp_inner = &mut broad_phase.0;
                let np_inner = &mut narrow_phase.0;
                let bodies_inner = &mut bodies.0;
                let colliders_inner = &mut colliders.0;
                let hooks_ref: &dyn rapier::pipeline::PhysicsHooks = match hooks_box.as_deref() {
                    Some(h) => h,
                    None => &(),
                };
                let events_ref: &dyn rapier::pipeline::EventHandler = match events_box.as_deref() {
                    Some(e) => e,
                    None => &(),
                };
                py.allow_threads(|| {
                    cp_inner.step(
                        prediction_distance,
                        islands_inner,
                        bp_inner,
                        np_inner,
                        bodies_inner,
                        colliders_inner,
                        hooks_ref,
                        events_ref,
                    );
                });
                drop(hooks_box);
                drop(events_box);
                let mut slot = err_slot.lock().unwrap();
                if let Some(e) = slot.err.take() {
                    return Err(e);
                }
                Ok(())
            }
        }

        // =====================================================================
        // PhysicsWorld — convenience aggregate.
        //
        // All sub-sets are stored as `Py<...>` so that property reads return
        // the **same** Python object on every call (mutations persist).
        // =====================================================================

        /// Recommended entry point — aggregates every rapier sub-state in one object.
        ///
        /// A :class:`PhysicsWorld` owns the body / collider / joint sets,
        /// broad / narrow phase, island manager, CCD solver, integration
        /// parameters, and physics + query pipelines. It exposes them as
        /// stable properties (the same Python object is returned on every
        /// access, so mutations persist)::
        ///
        ///     world = PhysicsWorld(gravity=Vec3(0.0, -9.81, 0.0),
        ///                          auto_update_query=True)
        ///     ground = world.add_collider(ColliderBuilder.cuboid(50, 0.1, 50))
        ///     ball = world.add_body(
        ///         RigidBodyBuilder.dynamic().translation(Vec3(0, 5, 0)),
        ///         colliders=[ColliderBuilder.ball(0.5)],
        ///     )
        ///     for _ in range(60):
        ///         world.step()
        ///
        ///     assert world.rigid_bodies is world.rigid_bodies  # stable
        ///
        /// Configure scene queries via :attr:`auto_update_query` (refresh the
        /// :class:`QueryPipeline` after every step) or call
        /// :meth:`update_query_pipeline` manually for finer control. Attach a
        /// :class:`ChannelEventCollector` to :attr:`event_handler` to consume
        /// collision / contact-force events.
        #[pyclass(name = "PhysicsWorld", module = "rapier", unsendable)]
        pub struct PhysicsWorld {
            pub bodies: Py<RigidBodySet>,
            pub colliders: Py<ColliderSet>,
            pub impulse_joints: Py<ImpulseJointSet>,
            pub multibody_joints: Py<MultibodyJointSet>,
            pub broad_phase: Py<BroadPhaseBvh>,
            pub narrow_phase: Py<NarrowPhase>,
            pub islands: Py<IslandManager>,
            pub ccd_solver: Py<CCDSolver>,
            pub integration_parameters: Py<IntegrationParameters>,
            pub physics_pipeline: Py<PhysicsPipeline>,
            pub query_pipeline: Py<QueryPipeline>,
            pub gravity: $Vec,
            pub event_handler: Option<Py<PyAny>>,
            pub physics_hooks: Option<Py<PyAny>>,
            pub auto_update_query: bool,
            /// Either `"defer"` (best-effort: stash exceptions, re-raise after
            /// `step()`) or `"strict"` (also short-circuit further hook calls
            /// within the same step).
            pub event_error_policy: String,
        }

        #[pymethods]
        impl PhysicsWorld {
            /// Construct a new world with default sub-states.
            ///
            /// :param gravity: World-space gravity vector. Defaults to zero
            ///     (no gravity).
            /// :param auto_update_query: When ``True``, :meth:`step` calls
            ///     :meth:`update_query_pipeline` automatically; otherwise the
            ///     :attr:`query_pipeline` reflects the previous
            ///     :meth:`update` call.
            #[new]
            #[pyo3(signature = (gravity=None, auto_update_query=false))]
            fn new(
                py: Python<'_>,
                gravity: Option<PyVector>,
                auto_update_query: bool,
            ) -> PyResult<Self> {
                let g = gravity.map(|v| v.0).unwrap_or_else($crate::na::SVector::<Real, $dim>::zeros);

                let bodies = Py::new(py, RigidBodySet(rapier::dynamics::RigidBodySet::new()))?;
                let colliders = Py::new(py, ColliderSet(rapier::geometry::ColliderSet::new()))?;
                let impulse_joints = Py::new(py, ImpulseJointSet(rapier::dynamics::ImpulseJointSet::new()))?;
                let multibody_joints = Py::new(py, MultibodyJointSet(rapier::dynamics::MultibodyJointSet::new()))?;
                let broad_phase = Py::new(py, BroadPhaseBvh(rapier::geometry::BroadPhaseBvh::new()))?;
                let narrow_phase = Py::new(py, NarrowPhase(rapier::geometry::NarrowPhase::new()))?;
                let islands = Py::new(py, IslandManager(rapier::dynamics::IslandManager::new()))?;
                let ccd_solver = Py::new(py, CCDSolver(rapier::dynamics::CCDSolver::new()))?;
                let integration_parameters = Py::new(
                    py,
                    IntegrationParameters(rapier::dynamics::IntegrationParameters::default()),
                )?;
                let physics_pipeline = Py::new(py, PhysicsPipeline(rapier::pipeline::PhysicsPipeline::new()))?;

                let query_pipeline = Py::new(py, QueryPipeline {
                    broad_phase: broad_phase.clone_ref(py),
                    narrow_phase: narrow_phase.clone_ref(py),
                    bodies: bodies.clone_ref(py),
                    colliders: colliders.clone_ref(py),
                })?;

                Ok(Self {
                    bodies, colliders, impulse_joints, multibody_joints,
                    broad_phase, narrow_phase, islands, ccd_solver,
                    integration_parameters, physics_pipeline, query_pipeline,
                    gravity: $Vec(g),
                    event_handler: None,
                    physics_hooks: None,
                    auto_update_query,
                    event_error_policy: "defer".to_string(),
                })
            }

            // ---- shared sub-set accessors (return the same Python objects) ----

            /// The world's :class:`RigidBodySet`. Returns the same Python
            /// object on every call, so ``world.rigid_bodies is world.rigid_bodies``.
            #[getter]
            fn rigid_bodies(&self, py: Python<'_>) -> Py<RigidBodySet> {
                self.bodies.clone_ref(py)
            }
            /// The world's :class:`ColliderSet` (stable across calls).
            #[getter]
            fn colliders(&self, py: Python<'_>) -> Py<ColliderSet> {
                self.colliders.clone_ref(py)
            }
            /// The world's :class:`ImpulseJointSet` (stable across calls).
            #[getter]
            fn impulse_joints(&self, py: Python<'_>) -> Py<ImpulseJointSet> {
                self.impulse_joints.clone_ref(py)
            }
            /// The world's :class:`MultibodyJointSet` (stable across calls).
            #[getter]
            fn multibody_joints(&self, py: Python<'_>) -> Py<MultibodyJointSet> {
                self.multibody_joints.clone_ref(py)
            }
            /// The world's :class:`BroadPhaseBvh` (stable across calls).
            #[getter]
            fn broad_phase(&self, py: Python<'_>) -> Py<BroadPhaseBvh> {
                self.broad_phase.clone_ref(py)
            }
            /// The world's :class:`NarrowPhase` (stable across calls).
            #[getter]
            fn narrow_phase(&self, py: Python<'_>) -> Py<NarrowPhase> {
                self.narrow_phase.clone_ref(py)
            }
            /// The world's :class:`IslandManager` (stable across calls).
            #[getter]
            fn islands(&self, py: Python<'_>) -> Py<IslandManager> {
                self.islands.clone_ref(py)
            }
            /// The world's :class:`CCDSolver` (stable across calls).
            #[getter]
            fn ccd_solver(&self, py: Python<'_>) -> Py<CCDSolver> {
                self.ccd_solver.clone_ref(py)
            }
            /// Current :class:`IntegrationParameters`. Stable across calls.
            #[getter]
            fn integration_parameters(&self, py: Python<'_>) -> Py<IntegrationParameters> {
                self.integration_parameters.clone_ref(py)
            }
            /// Replace the stored integration parameters in-place.
            #[setter]
            fn set_integration_parameters(
                &mut self,
                py: Python<'_>,
                ip: &IntegrationParameters,
            ) -> PyResult<()> {
                let mut cur = self.integration_parameters.borrow_mut(py);
                cur.0 = ip.0;
                Ok(())
            }
            /// The world's underlying :class:`PhysicsPipeline` (stable across calls).
            #[getter]
            fn physics_pipeline(&self, py: Python<'_>) -> Py<PhysicsPipeline> {
                self.physics_pipeline.clone_ref(py)
            }
            /// The world's :class:`QueryPipeline` (stable across calls).
            #[getter]
            fn query_pipeline(&self, py: Python<'_>) -> Py<QueryPipeline> {
                self.query_pipeline.clone_ref(py)
            }

            /// World-space gravity vector applied to dynamic bodies.
            #[getter]
            fn gravity(&self) -> $Vec { self.gravity }
            /// Set the gravity vector.
            #[setter]
            fn set_gravity(&mut self, v: PyVector) {
                self.gravity = $Vec(v.0);
            }

            /// Currently installed ``EventHandler``-protocol object, or ``None``.
            ///
            /// Usually a :class:`ChannelEventCollector`.
            #[getter]
            fn event_handler(&self, py: Python<'_>) -> Option<PyObject> {
                self.event_handler.as_ref().map(|p| p.clone_ref(py))
            }
            /// Install or remove the event handler.
            ///
            /// Pass ``None`` to detach. The handler is invoked from
            /// :meth:`step` (with the GIL re-acquired).
            #[setter]
            fn set_event_handler(&mut self, py: Python<'_>, v: Option<Py<PyAny>>) {
                self.event_handler = v.map(|p| p.clone_ref(py));
            }

            /// Currently installed ``PhysicsHooks``-protocol object, or ``None``.
            #[getter]
            fn physics_hooks(&self, py: Python<'_>) -> Option<PyObject> {
                self.physics_hooks.as_ref().map(|p| p.clone_ref(py))
            }
            /// Install or remove the physics-hooks object.
            ///
            /// The hooks object's ``filter_contact_pair`` /
            /// ``filter_intersection_pair`` / ``modify_solver_contacts``
            /// methods are invoked from :meth:`step` (with the GIL
            /// re-acquired).
            #[setter]
            fn set_physics_hooks(&mut self, py: Python<'_>, v: Option<Py<PyAny>>) {
                self.physics_hooks = v.map(|p| p.clone_ref(py));
            }

            /// When ``True``, :meth:`step` refreshes the query pipeline after each call.
            ///
            /// Trade-off: setting this to ``True`` keeps :attr:`query_pipeline`
            /// always fresh but adds a BVH rebuild after every step. Setting
            /// it to ``False`` avoids the redundant rebuild — useful when you
            /// only query infrequently or only after several steps; call
            /// :meth:`update_query_pipeline` manually before issuing queries.
            #[getter] fn auto_update_query(&self) -> bool { self.auto_update_query }
            /// Set :attr:`auto_update_query`.
            #[setter] fn set_auto_update_query(&mut self, v: bool) { self.auto_update_query = v; }

            /// Error-propagation policy for hook / event-handler exceptions.
            ///
            /// ``"defer"`` (default): catch every callback exception, stash the
            /// first one, finish the step, and re-raise it at the end.
            /// ``"strict"``: same, but also flip an abort flag so subsequent
            /// callbacks within the same step short-circuit.
            #[getter]
            fn event_error_policy(&self) -> String { self.event_error_policy.clone() }
            /// Set :attr:`event_error_policy` — must be ``"defer"`` or ``"strict"``.
            ///
            /// :raises ValueError: If ``v`` is neither ``"defer"`` nor ``"strict"``.
            #[setter]
            fn set_event_error_policy(&mut self, v: String) -> PyResult<()> {
                match v.as_str() {
                    "defer" | "strict" => {
                        self.event_error_policy = v;
                        Ok(())
                    }
                    _ => Err($crate::pyo3::exceptions::PyValueError::new_err(
                        "event_error_policy must be 'defer' or 'strict'",
                    )),
                }
            }

            // ---- step ----

            /// Advance the simulation by one step using the world's configuration.
            ///
            /// Uses the stored :attr:`integration_parameters`, :attr:`gravity`,
            /// :attr:`physics_hooks`, and :attr:`event_handler`. Releases the
            /// GIL via ``Python::allow_threads`` while the solver runs;
            /// callbacks re-acquire the GIL before invoking Python code.
            /// When :attr:`auto_update_query` is ``True``, the query pipeline
            /// is refreshed after the step. Exceptions raised inside Python
            /// callbacks are deferred per :attr:`event_error_policy` and
            /// re-raised after the step completes.
            fn step(&self, py: Python<'_>) -> PyResult<()> {
                let g = self.gravity.0;
                // Build the (boxed) hook/event adapters out-of-band so that
                // they can be passed as `&dyn Trait` into `allow_threads`.
                let err_slot = std::sync::Arc::new(std::sync::Mutex::new(DeferredError {
                    err: None,
                    aborted: false,
                    policy_strict: self.event_error_policy == "strict",
                }));
                let hooks_box = build_physics_hooks(py, self.physics_hooks.as_ref(), err_slot.clone());
                let events_box = build_event_handler(py, self.event_handler.as_ref(), err_slot.clone());
                {
                    let mut pp = self.physics_pipeline.borrow_mut(py);
                    let ip = self.integration_parameters.borrow(py);
                    let mut islands = self.islands.borrow_mut(py);
                    let mut bp = self.broad_phase.borrow_mut(py);
                    let mut np = self.narrow_phase.borrow_mut(py);
                    let mut bodies = self.bodies.borrow_mut(py);
                    let mut colliders = self.colliders.borrow_mut(py);
                    let mut ij = self.impulse_joints.borrow_mut(py);
                    let mut mj = self.multibody_joints.borrow_mut(py);
                    let mut ccd = self.ccd_solver.borrow_mut(py);
                    let g_engine: rapier::math::Vector = g.into();
                    // Borrow the inner rapier values explicitly so the
                    // `Ungil` closure doesn't capture any `Python<'_>` token
                    // (which is `!Send`).
                    let pp_inner = &mut pp.0;
                    let ip_inner = &ip.0;
                    let islands_inner = &mut islands.0;
                    let bp_inner = &mut bp.0;
                    let np_inner = &mut np.0;
                    let bodies_inner = &mut bodies.0;
                    let colliders_inner = &mut colliders.0;
                    let ij_inner = &mut ij.0;
                    let mj_inner = &mut mj.0;
                    let ccd_inner = &mut ccd.0;
                    let hooks_ref: &dyn rapier::pipeline::PhysicsHooks =
                        match hooks_box.as_deref() {
                            Some(h) => h,
                            None => &(),
                        };
                    let events_ref: &dyn rapier::pipeline::EventHandler =
                        match events_box.as_deref() {
                            Some(e) => e,
                            None => &(),
                        };
                    py.allow_threads(|| {
                        pp_inner.step(
                            g_engine,
                            ip_inner,
                            islands_inner,
                            bp_inner,
                            np_inner,
                            bodies_inner,
                            colliders_inner,
                            ij_inner,
                            mj_inner,
                            ccd_inner,
                            hooks_ref,
                            events_ref,
                        );
                    });
                }
                if self.auto_update_query {
                    self.update_query_pipeline(py)?;
                }
                drop(hooks_box);
                drop(events_box);
                let mut slot = err_slot.lock().unwrap();
                if let Some(e) = slot.err.take() {
                    return Err(e);
                }
                Ok(())
            }

            /// Refresh the :attr:`query_pipeline`'s broad-phase BVH.
            ///
            /// Drains the collider set's ``modified`` / ``removed`` change
            /// sets and updates the BVH. Call this manually before issuing
            /// scene queries when :attr:`auto_update_query` is ``False``, or
            /// after directly mutating colliders / bodies and wanting an
            /// immediate refresh.
            fn update_query_pipeline(&self, py: Python<'_>) -> PyResult<()> {
                let ip = self.integration_parameters.borrow(py);
                let bodies = self.bodies.borrow(py);
                let mut colliders = self.colliders.borrow_mut(py);
                let mut bp = self.broad_phase.borrow_mut(py);
                // Drain the colliders' "modified" / "removed" change sets so
                // the broad-phase BVH catches up without double-handling them
                // on the next `step()`.
                let modified = colliders.0.take_modified();
                let removed = colliders.0.take_removed();
                let mut events: Vec<rapier::geometry::BroadPhasePairEvent> = Vec::new();
                bp.0.update(&ip.0, &colliders.0, &bodies.0, &modified, &removed, &mut events);
                Ok(())
            }

            /// Wake a sleeping body, forcing it back into the active simulation.
            ///
            /// Useful after manually moving a body, applying forces, or
            /// otherwise wanting to make sure it gets simulated on the next
            /// step. No-op for already-awake bodies and for fixed bodies.
            ///
            /// :param handle: Handle of the body to wake.
            /// :param strong: If ``True``, the body is guaranteed to stay
            ///     awake for several frames; if ``False`` it may sleep again
            ///     immediately once sleep conditions are met.
            #[pyo3(signature = (handle, strong=true))]
            fn wake_up(&self, py: Python<'_>, handle: &RigidBodyHandle, strong: bool) -> PyResult<()> {
                let mut islands = self.islands.borrow_mut(py);
                let mut bodies = self.bodies.borrow_mut(py);
                islands.0.wake_up(&mut bodies.0, handle.0, strong);
                Ok(())
            }

            /// Wake every sleeping body in the world.
            ///
            /// :param strong: See :meth:`wake_up`.
            #[pyo3(signature = (strong=true))]
            fn wake_up_all(&self, py: Python<'_>, strong: bool) -> PyResult<()> {
                let mut islands = self.islands.borrow_mut(py);
                let mut bodies = self.bodies.borrow_mut(py);
                let handles: Vec<_> = bodies.0.iter().map(|(h, _)| h).collect();
                for handle in handles {
                    islands.0.wake_up(&mut bodies.0, handle, strong);
                }
                Ok(())
            }

            /// Return the handles of the currently active (awake) bodies.
            ///
            /// Sleeping bodies are skipped, as are bodies that never sleep but
            /// aren't part of any active island (e.g. unattached fixed
            /// bodies). This is the list to use when rendering or syncing
            /// transforms, since the poses of sleeping bodies haven't moved
            /// since the last step. Look each handle up via
            /// :attr:`rigid_bodies`.
            fn active_bodies(&self, py: Python<'_>) -> Vec<RigidBodyHandle> {
                let islands = self.islands.borrow(py);
                islands.0.active_bodies().map(RigidBodyHandle).collect()
            }

            /// Drop every body, collider, joint, and pipeline state.
            ///
            /// Leaves :attr:`gravity`, :attr:`integration_parameters`, and
            /// :attr:`auto_update_query` untouched. Equivalent to constructing
            /// a fresh world while reusing the same Python wrapper objects.
            fn clear(&self, py: Python<'_>) -> PyResult<()> {
                let mut bodies = self.bodies.borrow_mut(py);
                let mut colliders = self.colliders.borrow_mut(py);
                let mut ij = self.impulse_joints.borrow_mut(py);
                let mut mj = self.multibody_joints.borrow_mut(py);
                let mut bp = self.broad_phase.borrow_mut(py);
                let mut np = self.narrow_phase.borrow_mut(py);
                let mut islands = self.islands.borrow_mut(py);
                let mut ccd = self.ccd_solver.borrow_mut(py);
                bodies.0 = rapier::dynamics::RigidBodySet::new();
                colliders.0 = rapier::geometry::ColliderSet::new();
                ij.0 = rapier::dynamics::ImpulseJointSet::new();
                mj.0 = rapier::dynamics::MultibodyJointSet::new();
                bp.0 = rapier::geometry::BroadPhaseBvh::new();
                np.0 = rapier::geometry::NarrowPhase::new();
                islands.0 = rapier::dynamics::IslandManager::new();
                ccd.0 = rapier::dynamics::CCDSolver::new();
                Ok(())
            }

            // ---- pythonic add helpers ----

            /// Insert a rigid body, optionally attaching child colliders in one call.
            ///
            /// :param builder: A :class:`RigidBody` or :class:`RigidBodyBuilder`.
            /// :param colliders: Optional list of :class:`Collider` or
            ///     :class:`ColliderBuilder` instances attached to the new
            ///     body.
            /// :returns: Handle of the newly inserted body.
            /// :raises TypeError: If ``builder`` or any entry in ``colliders``
            ///     is not a recognized rapier type.
            #[pyo3(signature = (builder, colliders=None))]
            fn add_body(
                &self,
                py: Python<'_>,
                builder: &Bound<'_, PyAny>,
                colliders: Option<&Bound<'_, _PyList>>,
            ) -> PyResult<RigidBodyHandle> {
                let body = if let Ok(b) = builder.extract::<PyRef<'_, RigidBodyBuilder>>() {
                    b.builder.clone().build()
                } else if let Ok(rb) = builder.extract::<PyRef<'_, RigidBody>>() {
                    rb.to_owned_body()
                } else {
                    return Err(PyTypeError::new_err(
                        "PhysicsWorld.add_body expects a RigidBody or RigidBodyBuilder",
                    ));
                };
                let parent_handle = {
                    let mut bset = self.bodies.borrow_mut(py);
                    RigidBodyHandle(bset.0.insert(body))
                };
                if let Some(coll_list) = colliders {
                    for item in coll_list.iter() {
                        let coll = if let Ok(b) = item.extract::<PyRef<'_, ColliderBuilder>>() {
                            b.builder.clone().build()
                        } else if let Ok(c) = item.extract::<PyRef<'_, Collider>>() {
                            c.to_owned_collider()
                        } else {
                            return Err(PyTypeError::new_err(
                                "PhysicsWorld.add_body colliders must be Collider or ColliderBuilder instances",
                            ));
                        };
                        let mut cset = self.colliders.borrow_mut(py);
                        let mut bset = self.bodies.borrow_mut(py);
                        cset.0.insert_with_parent(coll, parent_handle.0, &mut bset.0);
                    }
                }
                Ok(parent_handle)
            }

            /// Insert a collider, optionally attaching it to a parent rigid body.
            ///
            /// :param builder: A :class:`Collider` or :class:`ColliderBuilder`.
            /// :param parent: Optional rigid-body handle; when ``None`` the
            ///     collider is added as standalone (no parent body).
            /// :returns: Handle of the newly inserted collider.
            /// :raises TypeError: If ``builder`` is not a recognized rapier
            ///     type.
            #[pyo3(signature = (builder, parent=None))]
            fn add_collider(
                &self,
                py: Python<'_>,
                builder: &Bound<'_, PyAny>,
                parent: Option<&RigidBodyHandle>,
            ) -> PyResult<ColliderHandle> {
                let coll = if let Ok(b) = builder.extract::<PyRef<'_, ColliderBuilder>>() {
                    b.builder.clone().build()
                } else if let Ok(c) = builder.extract::<PyRef<'_, Collider>>() {
                    c.to_owned_collider()
                } else {
                    return Err(PyTypeError::new_err(
                        "PhysicsWorld.add_collider expects a Collider or ColliderBuilder",
                    ));
                };
                let mut cset = self.colliders.borrow_mut(py);
                let handle = match parent {
                    None => cset.0.insert(coll),
                    Some(h) => {
                        let mut bset = self.bodies.borrow_mut(py);
                        cset.0.insert_with_parent(coll, h.0, &mut bset.0)
                    }
                };
                Ok(ColliderHandle(handle))
            }

            /// Remove a rigid body, its colliders, and any attached joints.
            ///
            /// :param handle: Handle returned from :meth:`add_body`.
            /// :returns: The removed :class:`RigidBody`, or ``None`` if the
            ///     handle was already invalid.
            fn remove_body(
                &self,
                py: Python<'_>,
                handle: &RigidBodyHandle,
            ) -> PyResult<Option<RigidBody>> {
                let mut bodies = self.bodies.borrow_mut(py);
                let mut islands = self.islands.borrow_mut(py);
                let mut colliders = self.colliders.borrow_mut(py);
                let mut ij = self.impulse_joints.borrow_mut(py);
                let mut mj = self.multibody_joints.borrow_mut(py);
                Ok(bodies.0.remove(
                    handle.0,
                    &mut islands.0,
                    &mut colliders.0,
                    &mut ij.0,
                    &mut mj.0,
                    true,
                ).map(RigidBody::new_owned))
            }

            /// Remove a collider, detaching it from its parent body if any.
            ///
            /// :param handle: Handle returned from :meth:`add_collider`.
            /// :returns: The removed :class:`Collider`, or ``None`` if the
            ///     handle was already invalid.
            fn remove_collider(
                &self,
                py: Python<'_>,
                handle: &ColliderHandle,
            ) -> PyResult<Option<Collider>> {
                let mut cset = self.colliders.borrow_mut(py);
                let mut bset = self.bodies.borrow_mut(py);
                let mut islands = self.islands.borrow_mut(py);
                Ok(cset.0.remove(handle.0, &mut islands.0, &mut bset.0, true)
                    .map(Collider::new_owned))
            }

            /// Debug repr — shows body and collider counts.
            fn __repr__(&self, py: Python<'_>) -> String {
                let nb = self.bodies.borrow(py).0.len();
                let nc = self.colliders.borrow(py).0.len();
                format!("PhysicsWorld(bodies={}, colliders={})", nb, nc)
            }
        }

        // touch the imported _PyList so unused warnings don't trip.
        #[allow(dead_code)]
        type _PipelinePyListAlias = _PyList;
    };
}

// ============================================================================
// Helpers: dim-specific bits used by the macro.
// ============================================================================

#[doc(hidden)]
#[macro_export]
macro_rules! __feature_id_edge_dim_specific {
    (3) => {
        #[staticmethod]
        #[pyo3(name = "Edge")]
        fn edge(id: u32) -> Self {
            Self { kind: "edge", id }
        }
    };
    (2) => {
        // FeatureId::Edge is 3D-only in parry; we still let users construct
        // it on 2D for portability, but it's never returned in 2D queries.
        #[staticmethod]
        #[pyo3(name = "Edge")]
        fn edge(id: u32) -> Self {
            Self { kind: "edge", id }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __feature_id_from_parry {
    (3, $f:expr) => {{
        use rapier::parry::shape::FeatureId as F;
        match $f {
            F::Vertex(i) => Self {
                kind: "vertex",
                id: i,
            },
            F::Edge(i) => Self {
                kind: "edge",
                id: i,
            },
            F::Face(i) => Self {
                kind: "face",
                id: i,
            },
            F::Unknown => Self {
                kind: "unknown",
                id: 0,
            },
        }
    }};
    (2, $f:expr) => {{
        use rapier::parry::shape::FeatureId as F;
        match $f {
            F::Vertex(i) => Self {
                kind: "vertex",
                id: i,
            },
            F::Face(i) => Self {
                kind: "face",
                id: i,
            },
            F::Unknown => Self {
                kind: "unknown",
                id: 0,
            },
        }
    }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! __nonlinear_angvel_extract {
    (3, $angvel:expr) => {{
        let av: rapier::math::Vector = match $angvel {
            None => rapier::math::Vector::ZERO,
            Some(v) => {
                let pv: PyVector = v.extract()?;
                pv.0.into()
            }
        };
        Ok::<rapier::math::Vector, $crate::pyo3::PyErr>(av)
    }};
    (2, $angvel:expr) => {{
        let av: Real = match $angvel {
            None => 0.0 as Real,
            Some(v) => {
                let f: Real = v.extract()?;
                f
            }
        };
        Ok::<Real, $crate::pyo3::PyErr>(av)
    }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! __nonlinear_angvel_getter {
    (3, $Vec:ident) => {
        #[pymethods]
        impl NonlinearRigidMotion {
            /// Angular velocity (3D: ``Vec3``, world-space, radians per unit time).
            #[getter]
            fn angvel(&self) -> $Vec {
                let v: $crate::na::Vector3<Real> = self.0.angvel.into();
                $Vec(v)
            }
        }
    };
    (2, $Vec:ident) => {
        #[pymethods]
        impl NonlinearRigidMotion {
            /// Angular velocity (2D: scalar, radians per unit time).
            #[getter]
            fn angvel(&self) -> Real {
                self.0.angvel
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __qp_query {
    ($self:expr, $py:expr, $filter:expr, $body:expr) => {{
        // Borrow the world's sub-sets immutably (queries are read-only).
        let bp = $self.broad_phase.borrow($py);
        let np = $self.narrow_phase.borrow($py);
        let bodies = $self.bodies.borrow($py);
        let colliders = $self.colliders.borrow($py);

        // Build the raw QueryFilter, handling the (optional) Python predicate.
        // The predicate is captured in a stack-pinned closure so the upstream
        // borrow (`&'a dyn Fn(...)`) stays valid for the call.
        let py_pred_obj: Option<Py<PyAny>> =
            $filter.and_then(|f| f.predicate.as_ref().map(|p| p.clone_ref($py)));
        let pred_err = std::cell::RefCell::new(None::<$crate::pyo3::PyErr>);
        let pred_closure =
            |h: rapier::geometry::ColliderHandle, co: &rapier::geometry::Collider| -> bool {
                let _ = co;
                if let Some(ref obj) = py_pred_obj {
                    // Hand the predicate a live view into the set (no copy of the
                    // collider). The set is already borrowed immutably for the
                    // query, so the view's reads re-borrow shared — fine; the
                    // predicate must not mutate the collider mid-query.
                    let coll_py = Collider {
                        backing: ColliderBacking::InSet {
                            set: $self.colliders.clone_ref($py),
                            handle: h,
                        },
                    };
                    let h_py = ColliderHandle(h);
                    match obj.call1($py, (h_py, coll_py)) {
                        Ok(r) => r.extract::<bool>($py).unwrap_or(true),
                        Err(e) => {
                            *pred_err.borrow_mut() = Some(e);
                            true
                        }
                    }
                } else {
                    true
                }
            };
        let pred_dyn: Option<
            &dyn Fn(rapier::geometry::ColliderHandle, &rapier::geometry::Collider) -> bool,
        > = if py_pred_obj.is_some() {
            Some(&pred_closure)
        } else {
            None
        };

        let qf = $filter.map(|f| f.as_rapier(pred_dyn)).unwrap_or_else(|| {
            let mut base = rapier::pipeline::QueryFilter::default();
            base.predicate = pred_dyn;
            base
        });

        let qp =
            bp.0.as_query_pipeline(np.0.query_dispatcher(), &bodies.0, &colliders.0, qf);

        let result = ($body)(qp);
        if let Some(e) = pred_err.into_inner() {
            return Err(e);
        }
        result
    }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_pipeline_register {
    () => {
        pub fn register_pipeline(
            _py: $crate::pyo3::Python<'_>,
            m: &$crate::pyo3::Bound<'_, $crate::pyo3::types::PyModule>,
        ) -> $crate::pyo3::PyResult<()> {
            use $crate::pyo3::prelude::*;
            m.add_class::<FeatureId>()?;
            m.add_class::<Ray>()?;
            m.add_class::<RayIntersection>()?;
            m.add_class::<PointProjection>()?;
            m.add_class::<ShapeCastStatus>()?;
            m.add_class::<ShapeCastOptions>()?;
            m.add_class::<ShapeCastHit>()?;
            m.add_class::<NonlinearRigidMotion>()?;
            m.add_class::<QueryFilterFlags>()?;
            m.add_class::<QueryFilter>()?;
            m.add_class::<QueryPipeline>()?;
            m.add_class::<StagesCounters>()?;
            m.add_class::<CollisionDetectionCounters>()?;
            m.add_class::<SolverCounters>()?;
            m.add_class::<CCDCounters>()?;
            m.add_class::<Counters>()?;
            m.add_class::<PhysicsPipeline>()?;
            m.add_class::<CollisionPipeline>()?;
            m.add_class::<PhysicsWorld>()?;
            Ok(())
        }
    };
}
