//! Macro that emits the user-facing geometry / collider / shape `#[pyclass]`-es
//! per cdylib.
//!
//! Invoke `define_geometry_types!` once per cdylib **before**
//! `define_dynamics_types!` so the dynamics macro can reference
//! `ColliderHandle` / `ColliderSet` from the enclosing scope.
//!
//! Expects the surrounding cdylib to already have:
//!   - The `Real`/`DIM` aliases and `Py{Vector,Point,Rotation,Isometry,AngVector}`
//!     adapter newtypes (from `define_conv_types!`).
//!   - The user-facing math `#[pyclass]`-es (`Vec*`/`Point*`/`Rotation*`/`Isometry*`)
//!     from `define_math_types!`.
//!   - `use rapier{2,3}d{,_f64} as rapier;` aliasing.

use numpy::{PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// Extract an `Mx3` `u32` matrix → `Vec<[u32; 3]>`.
pub fn extract_indices(obj: &Bound<'_, PyAny>) -> PyResult<Vec<[u32; 3]>> {
    if let Ok(arr) = obj.extract::<PyReadonlyArray2<u32>>() {
        let (nrows, ncols) = (arr.shape()[0], arr.shape()[1]);
        if ncols != 3 {
            return Err(PyValueError::new_err(format!(
                "expected ndarray with shape (M, 3); got (M, {ncols})"
            )));
        }
        let slice = arr
            .as_slice()
            .map_err(|_| PyValueError::new_err("ndarray must be contiguous"))?;
        let mut out = Vec::with_capacity(nrows);
        for chunk in slice.chunks_exact(3) {
            out.push([chunk[0], chunk[1], chunk[2]]);
        }
        return Ok(out);
    }
    if let Ok(arr) = obj.extract::<PyReadonlyArray2<i64>>() {
        let (nrows, ncols) = (arr.shape()[0], arr.shape()[1]);
        if ncols != 3 {
            return Err(PyValueError::new_err(format!(
                "expected ndarray with shape (M, 3); got (M, {ncols})"
            )));
        }
        let slice = arr
            .as_slice()
            .map_err(|_| PyValueError::new_err("ndarray must be contiguous"))?;
        let mut out = Vec::with_capacity(nrows);
        for chunk in slice.chunks_exact(3) {
            out.push([chunk[0] as u32, chunk[1] as u32, chunk[2] as u32]);
        }
        return Ok(out);
    }
    let seq: Vec<Vec<u32>> = obj.extract()?;
    seq.iter()
        .map(|c| {
            if c.len() != 3 {
                return Err(PyValueError::new_err(format!(
                    "expected inner sequence of length 3; got {}",
                    c.len(),
                )));
            }
            Ok([c[0], c[1], c[2]])
        })
        .collect()
}

/// Extract an `Mx2` `u32` matrix → `Vec<[u32; 2]>`.
pub fn extract_indices_2(obj: &Bound<'_, PyAny>) -> PyResult<Vec<[u32; 2]>> {
    if let Ok(arr) = obj.extract::<PyReadonlyArray2<u32>>() {
        let (nrows, ncols) = (arr.shape()[0], arr.shape()[1]);
        if ncols != 2 {
            return Err(PyValueError::new_err(format!(
                "expected ndarray with shape (M, 2); got (M, {ncols})"
            )));
        }
        let slice = arr
            .as_slice()
            .map_err(|_| PyValueError::new_err("ndarray must be contiguous"))?;
        let mut out = Vec::with_capacity(nrows);
        for chunk in slice.chunks_exact(2) {
            out.push([chunk[0], chunk[1]]);
        }
        return Ok(out);
    }
    let seq: Vec<Vec<u32>> = obj.extract()?;
    seq.iter()
        .map(|c| {
            if c.len() != 2 {
                return Err(PyValueError::new_err(format!(
                    "expected inner sequence of length 2; got {}",
                    c.len(),
                )));
            }
            Ok([c[0], c[1]])
        })
        .collect()
}

/// Materialize the geometry `#[pyclass]` types for a given `(Real, DIM)` pair.
///
/// Produces `register_geometry(py, m) -> PyResult<()>` for `#[pymodule]`.
#[macro_export]
macro_rules! define_geometry_types {
    (DIM = 2) => {
        $crate::__define_geometry_shared!(2);
        $crate::__define_geometry_2d!();
        $crate::__define_geometry_collider!(2, Vec2, Point2, Rotation2, Isometry2);
        $crate::__define_geometry_register!(2);
    };
    (DIM = 3) => {
        $crate::__define_geometry_shared!(3);
        $crate::__define_geometry_3d!();
        $crate::__define_geometry_collider!(3, Vec3, Point3, Rotation3, Isometry3);
        $crate::__define_geometry_register!(3);
    };
}

// ============================================================
// Shared dim-agnostic geometry types
// ============================================================

#[doc(hidden)]
#[macro_export]
macro_rules! __define_geometry_shared {
    ($dim:tt) => {
        use $crate::pyo3::exceptions::PyIndexError;

        // ============================================================
        // ColliderHandle (moved from dynamics)
        // ============================================================

        /// Opaque, generational handle identifying a collider in a `ColliderSet`.
        ///
        /// Handles remain valid until the collider is removed. After removal,
        /// the slot may be reused with a bumped generation, so old handles
        /// pointing to a freed slot compare unequal to the new one. Cheap to
        /// copy and hash.
        #[pyclass(name = "ColliderHandle", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct ColliderHandle(pub rapier::geometry::ColliderHandle);

        #[pymethods]
        impl ColliderHandle {
            /// Construct a handle from raw `(index, generation)` parts.
            ///
            /// Mostly useful for tests and serialization round-trips; in
            /// normal use, handles come from `ColliderSet.insert(...)`.
            #[new]
            #[pyo3(signature = (index=0, generation=0))]
            fn new(index: u32, generation: u32) -> Self {
                Self(rapier::geometry::ColliderHandle::from_raw_parts(
                    index, generation,
                ))
            }
            /// Build a handle from raw `(index, generation)` parts.
            #[staticmethod]
            fn from_raw_parts(index: u32, generation: u32) -> Self {
                Self(rapier::geometry::ColliderHandle::from_raw_parts(
                    index, generation,
                ))
            }
            /// Return the sentinel "invalid" handle.
            ///
            /// Never refers to a real collider; useful as a default value.
            #[staticmethod]
            fn invalid() -> Self {
                Self(rapier::geometry::ColliderSet::invalid_handle())
            }
            /// Slot index inside the underlying `ColliderSet`.
            #[getter]
            fn index(&self) -> u32 {
                self.0.into_raw_parts().0
            }
            /// Generation counter for the slot. Bumped on each reuse.
            #[getter]
            fn generation(&self) -> u32 {
                self.0.into_raw_parts().1
            }
            fn __hash__(&self) -> u64 {
                let (i, g) = self.0.into_raw_parts();
                ((i as u64) << 32) | (g as u64)
            }
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err(
                        "ColliderHandle supports only == and !=",
                    )),
                }
            }
            fn __repr__(&self) -> String {
                let (i, g) = self.0.into_raw_parts();
                format!("ColliderHandle(index={}, generation={})", i, g)
            }
        }

        // ============================================================
        // ShapeType (enum)
        // ============================================================

        /// Discriminant returned by `SharedShape.shape_type`.
        ///
        /// Use this to decide which `as_*()` downcast accessor to call on a
        /// `SharedShape`. `CUSTOM` covers user-defined shapes that do not
        /// match any built-in variant.
        #[pyclass(name = "ShapeType", module = "rapier", eq, eq_int)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum ShapeType {
            BALL,
            CUBOID,
            CAPSULE,
            SEGMENT,
            TRIANGLE,
            VOXELS,
            TRIMESH,
            POLYLINE,
            HALFSPACE,
            HEIGHTFIELD,
            COMPOUND,
            CONVEX_POLYGON,
            CONVEX_POLYHEDRON,
            CYLINDER,
            CONE,
            ROUND_CUBOID,
            ROUND_TRIANGLE,
            ROUND_CYLINDER,
            ROUND_CONE,
            ROUND_CONVEX_POLYHEDRON,
            ROUND_CONVEX_POLYGON,
            CUSTOM,
        }

        impl ShapeType {
            fn from_rapier(t: rapier::parry::shape::ShapeType) -> Self {
                use rapier::parry::shape::ShapeType as PT;
                // The dim-specific variants are gated by parry's `dim2`/`dim3`
                // features, but the cdylib already knows its own dim, so we
                // express the mapping using the dim-specific helper macro.
                $crate::__shape_type_from_rapier!($dim, t)
            }
        }

        // ============================================================
        // ColliderType
        // ============================================================

        /// Distinguishes a `SOLID` collider from a `SENSOR`.
        ///
        /// Solid colliders generate contact forces; sensor colliders only
        /// fire intersection events and do not produce contact responses.
        #[pyclass(name = "ColliderType", module = "rapier", eq, eq_int)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum ColliderType {
            SOLID,
            SENSOR,
        }

        impl ColliderType {
            #[allow(dead_code)]
            fn to_rapier(self) -> rapier::geometry::ColliderType {
                match self {
                    Self::SOLID => rapier::geometry::ColliderType::Solid,
                    Self::SENSOR => rapier::geometry::ColliderType::Sensor,
                }
            }
            #[allow(dead_code)]
            fn from_rapier(t: rapier::geometry::ColliderType) -> Self {
                match t {
                    rapier::geometry::ColliderType::Solid => Self::SOLID,
                    rapier::geometry::ColliderType::Sensor => Self::SENSOR,
                }
            }
        }

        // ============================================================
        // ColliderEnabled
        // ============================================================

        /// Enabled-state of a collider.
        ///
        /// `DISABLED_BY_PARENT` means the parent rigid-body is disabled, so
        /// the collider is effectively off without being explicitly disabled
        /// by the user.
        #[pyclass(name = "ColliderEnabled", module = "rapier", eq, eq_int)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum ColliderEnabled {
            ENABLED,
            DISABLED_BY_PARENT,
            DISABLED,
        }

        impl ColliderEnabled {
            #[allow(dead_code)]
            fn from_rapier(t: rapier::geometry::ColliderEnabled) -> Self {
                match t {
                    rapier::geometry::ColliderEnabled::Enabled => Self::ENABLED,
                    rapier::geometry::ColliderEnabled::DisabledByParent => Self::DISABLED_BY_PARENT,
                    rapier::geometry::ColliderEnabled::Disabled => Self::DISABLED,
                }
            }
        }

        // ============================================================
        // ActiveEvents (bitflags)
        // ============================================================

        /// Per-collider opt-in flags selecting which events to emit.
        ///
        /// A collider only emits an event kind if it has the corresponding
        /// flag set. Use `COLLISION_EVENTS` to receive `CollisionEvent`
        /// (started/stopped) and `CONTACT_FORCE_EVENTS` to receive
        /// `ContactForceEvent`. Combine flags via `|`.
        #[pyclass(name = "ActiveEvents", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct ActiveEvents(pub rapier::pipeline::ActiveEvents);

        #[pymethods]
        impl ActiveEvents {
            /// Construct from a raw bit pattern. Unknown bits are dropped.
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u32) -> Self {
                Self(rapier::pipeline::ActiveEvents::from_bits_truncate(bits))
            }
            /// Return the empty flag set (no events).
            #[staticmethod]
            fn empty() -> Self {
                Self(rapier::pipeline::ActiveEvents::empty())
            }
            /// Return the flag set with every known event kind enabled.
            #[staticmethod]
            fn all_events() -> Self {
                Self(rapier::pipeline::ActiveEvents::all())
            }
            #[classattr]
            const COLLISION_EVENTS: ActiveEvents =
                ActiveEvents(rapier::pipeline::ActiveEvents::COLLISION_EVENTS);
            #[classattr]
            const CONTACT_FORCE_EVENTS: ActiveEvents =
                ActiveEvents(rapier::pipeline::ActiveEvents::CONTACT_FORCE_EVENTS);
            #[classattr]
            const NONE: ActiveEvents = ActiveEvents(rapier::pipeline::ActiveEvents::empty());

            /// Raw bit pattern of the flag set.
            #[getter]
            fn bits(&self) -> u32 {
                self.0.bits()
            }
            /// True iff all flags in `other` are also set on `self`.
            fn contains(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// True iff no flags are set.
            fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
            fn __contains__(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            fn __or__(&self, other: &Self) -> Self {
                Self(self.0 | other.0)
            }
            fn __and__(&self, other: &Self) -> Self {
                Self(self.0 & other.0)
            }
            fn __xor__(&self, other: &Self) -> Self {
                Self(self.0 ^ other.0)
            }
            fn __invert__(&self) -> Self {
                Self(!self.0)
            }
            fn __sub__(&self, other: &Self) -> Self {
                Self(self.0 - other.0)
            }
            fn __bool__(&self) -> bool {
                !self.0.is_empty()
            }
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err("ActiveEvents supports only == and !=")),
                }
            }
            fn __hash__(&self) -> u64 {
                self.0.bits() as u64
            }
            fn __repr__(&self) -> String {
                format!("ActiveEvents(bits={:#010b})", self.0.bits())
            }
        }

        // ============================================================
        // ActiveHooks (bitflags)
        // ============================================================

        /// Per-collider opt-in flags selecting which `PhysicsHooks` callbacks
        /// to invoke.
        ///
        /// A `PhysicsHooks` method only fires for a contact/intersection pair
        /// if at least one of the involved colliders has the matching flag
        /// set. Combine flags via `|`.
        #[pyclass(name = "ActiveHooks", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct ActiveHooks(pub rapier::pipeline::ActiveHooks);

        #[pymethods]
        impl ActiveHooks {
            /// Construct from a raw bit pattern. Unknown bits are dropped.
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u32) -> Self {
                Self(rapier::pipeline::ActiveHooks::from_bits_truncate(bits))
            }
            /// Return the empty flag set (no hooks active).
            #[staticmethod]
            fn empty() -> Self {
                Self(rapier::pipeline::ActiveHooks::empty())
            }
            /// Return the flag set with every known hook enabled.
            #[staticmethod]
            fn all_hooks() -> Self {
                Self(rapier::pipeline::ActiveHooks::all())
            }
            #[classattr]
            const FILTER_CONTACT_PAIR: ActiveHooks =
                ActiveHooks(rapier::pipeline::ActiveHooks::FILTER_CONTACT_PAIRS);
            #[classattr]
            const FILTER_CONTACT_PAIRS: ActiveHooks =
                ActiveHooks(rapier::pipeline::ActiveHooks::FILTER_CONTACT_PAIRS);
            #[classattr]
            const FILTER_INTERSECTION_PAIR: ActiveHooks =
                ActiveHooks(rapier::pipeline::ActiveHooks::FILTER_INTERSECTION_PAIR);
            #[classattr]
            const MODIFY_SOLVER_CONTACTS: ActiveHooks =
                ActiveHooks(rapier::pipeline::ActiveHooks::MODIFY_SOLVER_CONTACTS);
            #[classattr]
            const NONE: ActiveHooks = ActiveHooks(rapier::pipeline::ActiveHooks::empty());

            /// Raw bit pattern of the flag set.
            #[getter]
            fn bits(&self) -> u32 {
                self.0.bits()
            }
            /// True iff all flags in `other` are also set on `self`.
            fn contains(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// True iff no flags are set.
            fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
            fn __contains__(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            fn __or__(&self, other: &Self) -> Self {
                Self(self.0 | other.0)
            }
            fn __and__(&self, other: &Self) -> Self {
                Self(self.0 & other.0)
            }
            fn __xor__(&self, other: &Self) -> Self {
                Self(self.0 ^ other.0)
            }
            fn __invert__(&self) -> Self {
                Self(!self.0)
            }
            fn __sub__(&self, other: &Self) -> Self {
                Self(self.0 - other.0)
            }
            fn __bool__(&self) -> bool {
                !self.0.is_empty()
            }
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err("ActiveHooks supports only == and !=")),
                }
            }
            fn __hash__(&self) -> u64 {
                self.0.bits() as u64
            }
            fn __repr__(&self) -> String {
                format!("ActiveHooks(bits={:#010b})", self.0.bits())
            }
        }

        // ============================================================
        // ActiveCollisionTypes (bitflags)
        // ============================================================

        /// Per-collider flags selecting which *rigid-body type* combinations
        /// generate contacts.
        ///
        /// By default only pairs involving at least one dynamic body collide
        /// (`DYNAMIC_DYNAMIC | DYNAMIC_KINEMATIC | DYNAMIC_FIXED`). Enable e.g.
        /// `KINEMATIC_FIXED` or `KINEMATIC_KINEMATIC` to detect contacts that
        /// would otherwise be skipped — useful for kinematic character or
        /// platform queries. Combine flags via `|`.
        #[pyclass(name = "ActiveCollisionTypes", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct ActiveCollisionTypes(pub rapier::geometry::ActiveCollisionTypes);

        #[pymethods]
        impl ActiveCollisionTypes {
            /// Construct from a raw bit pattern. Unknown bits are dropped.
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u16) -> Self {
                Self(rapier::geometry::ActiveCollisionTypes::from_bits_truncate(
                    bits,
                ))
            }
            /// Return the empty flag set (no collision types active).
            #[staticmethod]
            fn empty() -> Self {
                Self(rapier::geometry::ActiveCollisionTypes::empty())
            }
            /// Return the flag set with every collision type enabled.
            #[staticmethod]
            fn all() -> Self {
                Self(rapier::geometry::ActiveCollisionTypes::all())
            }
            /// Return the default flag set: every pair involving at least one
            /// dynamic body (`DYNAMIC_DYNAMIC | DYNAMIC_KINEMATIC | DYNAMIC_FIXED`).
            #[staticmethod]
            fn default_types() -> Self {
                Self(rapier::geometry::ActiveCollisionTypes::default())
            }
            #[classattr]
            const DYNAMIC_DYNAMIC: ActiveCollisionTypes =
                ActiveCollisionTypes(rapier::geometry::ActiveCollisionTypes::DYNAMIC_DYNAMIC);
            #[classattr]
            const DYNAMIC_KINEMATIC: ActiveCollisionTypes =
                ActiveCollisionTypes(rapier::geometry::ActiveCollisionTypes::DYNAMIC_KINEMATIC);
            #[classattr]
            const DYNAMIC_FIXED: ActiveCollisionTypes =
                ActiveCollisionTypes(rapier::geometry::ActiveCollisionTypes::DYNAMIC_FIXED);
            #[classattr]
            const KINEMATIC_KINEMATIC: ActiveCollisionTypes =
                ActiveCollisionTypes(rapier::geometry::ActiveCollisionTypes::KINEMATIC_KINEMATIC);
            #[classattr]
            const KINEMATIC_FIXED: ActiveCollisionTypes =
                ActiveCollisionTypes(rapier::geometry::ActiveCollisionTypes::KINEMATIC_FIXED);
            #[classattr]
            const FIXED_FIXED: ActiveCollisionTypes =
                ActiveCollisionTypes(rapier::geometry::ActiveCollisionTypes::FIXED_FIXED);
            #[classattr]
            const NONE: ActiveCollisionTypes =
                ActiveCollisionTypes(rapier::geometry::ActiveCollisionTypes::empty());

            /// Raw bit pattern of the flag set.
            #[getter]
            fn bits(&self) -> u16 {
                self.0.bits()
            }
            /// True iff all flags in `other` are also set on `self`.
            fn contains(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// True iff no flags are set.
            fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
            fn __contains__(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            fn __or__(&self, other: &Self) -> Self {
                Self(self.0 | other.0)
            }
            fn __and__(&self, other: &Self) -> Self {
                Self(self.0 & other.0)
            }
            fn __xor__(&self, other: &Self) -> Self {
                Self(self.0 ^ other.0)
            }
            fn __invert__(&self) -> Self {
                Self(!self.0)
            }
            fn __sub__(&self, other: &Self) -> Self {
                Self(self.0 - other.0)
            }
            fn __bool__(&self) -> bool {
                !self.0.is_empty()
            }
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err(
                        "ActiveCollisionTypes supports only == and !=",
                    )),
                }
            }
            fn __hash__(&self) -> u64 {
                self.0.bits() as u64
            }
            fn __repr__(&self) -> String {
                format!("ActiveCollisionTypes(bits={:#018b})", self.0.bits())
            }
        }

        // ============================================================
        // Group (bitflags)
        // ============================================================

        /// 32-bit bitmask identifying one or more collision groups.
        ///
        /// Used as both the *membership* and *filter* side of an
        /// `InteractionGroups`: two colliders interact iff each one's
        /// memberships intersect the other's filter. Combine groups via
        /// `|`, take complements via `~`, and use `ALL` / `NONE` for the
        /// universe / empty set.
        #[pyclass(name = "Group", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct Group(pub rapier::geometry::Group);

        #[pymethods]
        impl Group {
            /// Construct from a raw bit pattern. All 32 bits are retained.
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u32) -> Self {
                Self(rapier::geometry::Group::from_bits_retain(bits))
            }

            /// Construct from a raw bit pattern. All 32 bits are retained.
            #[staticmethod]
            fn from_bits(bits: u32) -> Self {
                Self(rapier::geometry::Group::from_bits_retain(bits))
            }

            // Rapier's Group constants are 1-indexed (GROUP_1..GROUP_32).
            // The spec asks for `GROUP_0..GROUP_31`. Expose both spellings to
            // bridge both worlds.
            #[classattr]
            const GROUP_0: Group = Group(rapier::geometry::Group::GROUP_1);
            #[classattr]
            const GROUP_1: Group = Group(rapier::geometry::Group::GROUP_2);
            #[classattr]
            const GROUP_2: Group = Group(rapier::geometry::Group::GROUP_3);
            #[classattr]
            const GROUP_3: Group = Group(rapier::geometry::Group::GROUP_4);
            #[classattr]
            const GROUP_4: Group = Group(rapier::geometry::Group::GROUP_5);
            #[classattr]
            const GROUP_5: Group = Group(rapier::geometry::Group::GROUP_6);
            #[classattr]
            const GROUP_6: Group = Group(rapier::geometry::Group::GROUP_7);
            #[classattr]
            const GROUP_7: Group = Group(rapier::geometry::Group::GROUP_8);
            #[classattr]
            const GROUP_8: Group = Group(rapier::geometry::Group::GROUP_9);
            #[classattr]
            const GROUP_9: Group = Group(rapier::geometry::Group::GROUP_10);
            #[classattr]
            const GROUP_10: Group = Group(rapier::geometry::Group::GROUP_11);
            #[classattr]
            const GROUP_11: Group = Group(rapier::geometry::Group::GROUP_12);
            #[classattr]
            const GROUP_12: Group = Group(rapier::geometry::Group::GROUP_13);
            #[classattr]
            const GROUP_13: Group = Group(rapier::geometry::Group::GROUP_14);
            #[classattr]
            const GROUP_14: Group = Group(rapier::geometry::Group::GROUP_15);
            #[classattr]
            const GROUP_15: Group = Group(rapier::geometry::Group::GROUP_16);
            #[classattr]
            const GROUP_16: Group = Group(rapier::geometry::Group::GROUP_17);
            #[classattr]
            const GROUP_17: Group = Group(rapier::geometry::Group::GROUP_18);
            #[classattr]
            const GROUP_18: Group = Group(rapier::geometry::Group::GROUP_19);
            #[classattr]
            const GROUP_19: Group = Group(rapier::geometry::Group::GROUP_20);
            #[classattr]
            const GROUP_20: Group = Group(rapier::geometry::Group::GROUP_21);
            #[classattr]
            const GROUP_21: Group = Group(rapier::geometry::Group::GROUP_22);
            #[classattr]
            const GROUP_22: Group = Group(rapier::geometry::Group::GROUP_23);
            #[classattr]
            const GROUP_23: Group = Group(rapier::geometry::Group::GROUP_24);
            #[classattr]
            const GROUP_24: Group = Group(rapier::geometry::Group::GROUP_25);
            #[classattr]
            const GROUP_25: Group = Group(rapier::geometry::Group::GROUP_26);
            #[classattr]
            const GROUP_26: Group = Group(rapier::geometry::Group::GROUP_27);
            #[classattr]
            const GROUP_27: Group = Group(rapier::geometry::Group::GROUP_28);
            #[classattr]
            const GROUP_28: Group = Group(rapier::geometry::Group::GROUP_29);
            #[classattr]
            const GROUP_29: Group = Group(rapier::geometry::Group::GROUP_30);
            #[classattr]
            const GROUP_30: Group = Group(rapier::geometry::Group::GROUP_31);
            #[classattr]
            const GROUP_31: Group = Group(rapier::geometry::Group::GROUP_32);

            #[classattr]
            const ALL: Group = Group(rapier::geometry::Group::ALL);
            #[classattr]
            const NONE: Group = Group(rapier::geometry::Group::NONE);

            /// Raw 32-bit pattern of the group set.
            #[getter]
            fn bits(&self) -> u32 {
                self.0.bits()
            }
            /// True iff all groups in `other` are also set on `self`.
            fn contains(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// True iff no group bit is set.
            fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
            fn __contains__(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            fn __or__(&self, other: &Self) -> Self {
                Self(self.0 | other.0)
            }
            fn __and__(&self, other: &Self) -> Self {
                Self(self.0 & other.0)
            }
            fn __xor__(&self, other: &Self) -> Self {
                Self(self.0 ^ other.0)
            }
            fn __invert__(&self) -> Self {
                Self(!self.0)
            }
            fn __sub__(&self, other: &Self) -> Self {
                Self(self.0 - other.0)
            }
            fn __bool__(&self) -> bool {
                !self.0.is_empty()
            }
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err("Group supports only == and !=")),
                }
            }
            fn __hash__(&self) -> u64 {
                self.0.bits() as u64
            }
            fn __repr__(&self) -> String {
                format!("Group(bits={:#034b})", self.0.bits())
            }
        }

        // ============================================================
        // InteractionTestMode
        // ============================================================

        /// How `InteractionGroups.test` combines the two pair-wise checks.
        ///
        /// `AND` (the default) requires each collider to be in the other's
        /// filter set. `OR` only requires one side to pass. `DEFAULT` and
        /// `ONLY_DYNAMIC` are kept for compatibility and behave like `AND`.
        #[pyclass(name = "InteractionTestMode", module = "rapier", eq, eq_int)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum InteractionTestMode {
            DEFAULT,
            ONLY_DYNAMIC,
            AND,
            OR,
        }

        impl InteractionTestMode {
            fn to_rapier(self) -> rapier::geometry::InteractionTestMode {
                match self {
                    Self::DEFAULT | Self::AND | Self::ONLY_DYNAMIC => {
                        rapier::geometry::InteractionTestMode::And
                    }
                    Self::OR => rapier::geometry::InteractionTestMode::Or,
                }
            }
            #[allow(dead_code)]
            fn from_rapier(t: rapier::geometry::InteractionTestMode) -> Self {
                match t {
                    rapier::geometry::InteractionTestMode::And => Self::DEFAULT,
                    rapier::geometry::InteractionTestMode::Or => Self::OR,
                }
            }
        }

        // ============================================================
        // InteractionGroups
        // ============================================================

        /// Pair (memberships, filter) used to selectively enable contacts.
        ///
        /// A collider with memberships M and filter F interacts with another
        /// collider (memberships M', filter F') iff
        /// `(M & F') != 0 and (M' & F) != 0` (the default `AND` mode).
        /// In short: a pair interacts iff each one passes the other's filter.
        /// Used for both `collision_groups` and `solver_groups` on a collider.
        #[pyclass(name = "InteractionGroups", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct InteractionGroups(pub rapier::geometry::InteractionGroups);

        #[pymethods]
        impl InteractionGroups {
            /// Build a new interaction-groups value.
            ///
            /// :param memberships: Which groups this collider belongs to.
            ///     Defaults to `Group.ALL`.
            /// :param filter: Which groups this collider is willing to
            ///     interact with. Defaults to `Group.ALL`.
            /// :param test_mode: How both sides are combined. Defaults to
            ///     `InteractionTestMode.AND`.
            #[new]
            #[pyo3(signature = (memberships=None, filter=None, test_mode=None))]
            fn new(
                memberships: Option<Group>,
                filter: Option<Group>,
                test_mode: Option<InteractionTestMode>,
            ) -> Self {
                let m = memberships
                    .map(|g| g.0)
                    .unwrap_or(rapier::geometry::Group::ALL);
                let f = filter.map(|g| g.0).unwrap_or(rapier::geometry::Group::ALL);
                let mode = test_mode
                    .map(|m| m.to_rapier())
                    .unwrap_or(rapier::geometry::InteractionTestMode::And);
                Self(rapier::geometry::InteractionGroups::new(m, f, mode))
            }

            /// Interaction groups that match everything (memberships = filter = ALL).
            #[staticmethod]
            fn all() -> Self {
                Self(rapier::geometry::InteractionGroups::all())
            }
            /// Interaction groups that match nothing (memberships = filter = NONE).
            #[staticmethod]
            fn none() -> Self {
                Self(rapier::geometry::InteractionGroups::none())
            }

            /// Return a copy with `memberships` replaced. Alias of `with_memberships`.
            fn with_membership(&self, g: Group) -> Self {
                Self(self.0.with_memberships(g.0))
            }
            /// Return a copy with `memberships` replaced.
            fn with_memberships(&self, g: Group) -> Self {
                Self(self.0.with_memberships(g.0))
            }
            /// Return a copy with `filter` replaced.
            fn with_filter(&self, g: Group) -> Self {
                Self(self.0.with_filter(g.0))
            }

            /// Groups this collider is a member of.
            #[getter]
            fn memberships(&self) -> Group {
                Group(self.0.memberships)
            }
            #[setter]
            fn set_memberships(&mut self, g: Group) {
                self.0.memberships = g.0;
            }
            /// Groups this collider is willing to interact with.
            #[getter]
            fn filter(&self) -> Group {
                Group(self.0.filter)
            }
            #[setter]
            fn set_filter(&mut self, g: Group) {
                self.0.filter = g.0;
            }
            /// How the two pair-wise group checks are combined.
            #[getter]
            fn test_mode(&self) -> InteractionTestMode {
                InteractionTestMode::from_rapier(self.0.test_mode)
            }
            #[setter]
            fn set_test_mode(&mut self, m: InteractionTestMode) {
                self.0.test_mode = m.to_rapier();
            }

            /// Test whether `self` and `other` should interact.
            ///
            /// :returns: True iff both sides pass the other's filter under
            ///     the active `test_mode`.
            fn test(&self, other: &Self) -> bool {
                self.0.test(other.0)
            }

            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err(
                        "InteractionGroups supports only == and !=",
                    )),
                }
            }
            fn __repr__(&self) -> String {
                format!(
                    "InteractionGroups(memberships={:#010x}, filter={:#010x})",
                    self.0.memberships.bits(),
                    self.0.filter.bits(),
                )
            }
        }

        // ============================================================
        // CollisionEventFlags (bitflags)
        // ============================================================

        /// Side-flags attached to each `CollisionEvent`.
        ///
        /// `SENSOR` indicates the event involves a sensor collider;
        /// `REMOVED` indicates the event was emitted because one of the
        /// colliders was removed mid-frame.
        #[pyclass(name = "CollisionEventFlags", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct CollisionEventFlags(pub rapier::geometry::CollisionEventFlags);

        #[pymethods]
        impl CollisionEventFlags {
            /// Construct from a raw bit pattern. Unknown bits are dropped.
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u32) -> Self {
                Self(rapier::geometry::CollisionEventFlags::from_bits_truncate(
                    bits,
                ))
            }
            /// Return the empty flag set.
            #[staticmethod]
            fn empty() -> Self {
                Self(rapier::geometry::CollisionEventFlags::empty())
            }
            #[classattr]
            const SENSOR: CollisionEventFlags =
                CollisionEventFlags(rapier::geometry::CollisionEventFlags::SENSOR);
            #[classattr]
            const REMOVED: CollisionEventFlags =
                CollisionEventFlags(rapier::geometry::CollisionEventFlags::REMOVED);

            /// Raw bit pattern of the flag set.
            #[getter]
            fn bits(&self) -> u32 {
                self.0.bits()
            }
            /// True iff all flags in `other` are also set on `self`.
            fn contains(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// True iff no flags are set.
            fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
            fn __contains__(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            fn __or__(&self, other: &Self) -> Self {
                Self(self.0 | other.0)
            }
            fn __and__(&self, other: &Self) -> Self {
                Self(self.0 & other.0)
            }
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err(
                        "CollisionEventFlags supports only == and !=",
                    )),
                }
            }
            fn __repr__(&self) -> String {
                format!("CollisionEventFlags(bits={:#010b})", self.0.bits())
            }
        }

        // ============================================================
        // TriMeshFlags (bitflags)
        // ============================================================

        /// Build-time flags controlling triangle-mesh preprocessing.
        ///
        /// Passed to `SharedShape.trimesh(..., flags=...)` or
        /// `MeshConverter.trimesh_with_flags(...)`. Toggle topology
        /// validation, duplicate cleanup, edge-normal fixups, etc.
        #[pyclass(name = "TriMeshFlags", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct TriMeshFlags(pub rapier::parry::shape::TriMeshFlags);

        #[pymethods]
        impl TriMeshFlags {
            /// Construct from a raw bit pattern. Unknown bits are dropped.
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u16) -> Self {
                Self(rapier::parry::shape::TriMeshFlags::from_bits_truncate(bits))
            }
            /// Return the empty flag set.
            #[staticmethod]
            fn empty() -> Self {
                Self(rapier::parry::shape::TriMeshFlags::empty())
            }

            #[classattr]
            const HALF_EDGE_TOPOLOGY: TriMeshFlags =
                TriMeshFlags(rapier::parry::shape::TriMeshFlags::HALF_EDGE_TOPOLOGY);
            #[classattr]
            const CONNECTED_COMPONENTS: TriMeshFlags =
                TriMeshFlags(rapier::parry::shape::TriMeshFlags::CONNECTED_COMPONENTS);
            #[classattr]
            const DELETE_BAD_TOPOLOGY_TRIANGLES: TriMeshFlags =
                TriMeshFlags(rapier::parry::shape::TriMeshFlags::DELETE_BAD_TOPOLOGY_TRIANGLES);
            #[classattr]
            const ORIENTED: TriMeshFlags =
                TriMeshFlags(rapier::parry::shape::TriMeshFlags::ORIENTED);
            #[classattr]
            const MERGE_DUPLICATE_VERTICES: TriMeshFlags =
                TriMeshFlags(rapier::parry::shape::TriMeshFlags::MERGE_DUPLICATE_VERTICES);
            #[classattr]
            const DELETE_DEGENERATE_TRIANGLES: TriMeshFlags =
                TriMeshFlags(rapier::parry::shape::TriMeshFlags::DELETE_DEGENERATE_TRIANGLES);
            #[classattr]
            const DELETE_DUPLICATE_TRIANGLES: TriMeshFlags =
                TriMeshFlags(rapier::parry::shape::TriMeshFlags::DELETE_DUPLICATE_TRIANGLES);
            #[classattr]
            const FIX_INTERNAL_EDGES: TriMeshFlags =
                TriMeshFlags(rapier::parry::shape::TriMeshFlags::FIX_INTERNAL_EDGES);

            /// Raw bit pattern of the flag set.
            #[getter]
            fn bits(&self) -> u16 {
                self.0.bits()
            }
            /// True iff all flags in `other` are also set on `self`.
            fn contains(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// True iff no flags are set.
            fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
            fn __contains__(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            fn __or__(&self, other: &Self) -> Self {
                Self(self.0 | other.0)
            }
            fn __and__(&self, other: &Self) -> Self {
                Self(self.0 & other.0)
            }
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err("TriMeshFlags supports only == and !=")),
                }
            }
            fn __repr__(&self) -> String {
                format!("TriMeshFlags(bits={:#018b})", self.0.bits())
            }
        }

        // ============================================================
        // ColliderMaterial
        // ============================================================

        /// Surface material: friction, restitution, and their combine rules.
        ///
        /// When two colliders touch, the engine combines their material
        /// values using the configured `CoefficientCombineRule` (e.g.
        /// `MAX`, `MIN`, `AVERAGE`, `MULTIPLY`).
        #[pyclass(name = "ColliderMaterial", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct ColliderMaterial(pub rapier::geometry::ColliderMaterial);

        #[pymethods]
        impl ColliderMaterial {
            /// :param friction: Friction coefficient. Defaults to 1.0.
            /// :param restitution: Restitution (bounciness). Defaults to 0.0.
            #[new]
            #[pyo3(signature = (friction=1.0 as Real, restitution=0.0 as Real))]
            fn new(friction: Real, restitution: Real) -> Self {
                Self(rapier::geometry::ColliderMaterial::new(
                    friction,
                    restitution,
                ))
            }

            /// Friction coefficient.
            #[getter]
            fn friction(&self) -> Real {
                self.0.friction
            }
            #[setter]
            fn set_friction(&mut self, v: Real) {
                self.0.friction = v;
            }
            /// Restitution (bounciness) in `[0, 1]`.
            #[getter]
            fn restitution(&self) -> Real {
                self.0.restitution
            }
            #[setter]
            fn set_restitution(&mut self, v: Real) {
                self.0.restitution = v;
            }

            /// Rule used to combine friction coefficients with another collider.
            #[getter]
            fn friction_combine_rule(&self) -> CoefficientCombineRule {
                CoefficientCombineRule::from_rapier(self.0.friction_combine_rule)
            }
            #[setter]
            fn set_friction_combine_rule(&mut self, v: CoefficientCombineRule) {
                self.0.friction_combine_rule = v.to_rapier();
            }
            /// Rule used to combine restitution with another collider.
            #[getter]
            fn restitution_combine_rule(&self) -> CoefficientCombineRule {
                CoefficientCombineRule::from_rapier(self.0.restitution_combine_rule)
            }
            #[setter]
            fn set_restitution_combine_rule(&mut self, v: CoefficientCombineRule) {
                self.0.restitution_combine_rule = v.to_rapier();
            }

            fn __repr__(&self) -> String {
                format!(
                    "ColliderMaterial(friction={}, restitution={})",
                    self.0.friction, self.0.restitution,
                )
            }
        }

        // ============================================================
        // BvhOptimizationStrategy (enum)
        // ============================================================

        /// Strategy used by `BroadPhaseBvh` to keep its bounding hierarchy tidy.
        ///
        /// `AUTO` lets the engine pick a reasonable default; the other
        /// variants force a specific behavior.
        #[pyclass(name = "BvhOptimizationStrategy", module = "rapier", eq, eq_int)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum BvhOptimizationStrategy {
            AUTO,
            NO_OPTIMIZATION,
            OPTIMIZE_FOR_INTERACTIONS,
            OPTIMIZE_FOR_SPATIAL_QUERIES,
            SUBTREE_OPTIMIZER,
            NONE,
        }

        impl BvhOptimizationStrategy {
            fn to_rapier(self) -> rapier::geometry::BvhOptimizationStrategy {
                match self {
                    Self::AUTO
                    | Self::OPTIMIZE_FOR_INTERACTIONS
                    | Self::OPTIMIZE_FOR_SPATIAL_QUERIES
                    | Self::SUBTREE_OPTIMIZER => {
                        rapier::geometry::BvhOptimizationStrategy::SubtreeOptimizer
                    }
                    Self::NO_OPTIMIZATION | Self::NONE => {
                        rapier::geometry::BvhOptimizationStrategy::None
                    }
                }
            }
        }

        // ============================================================
        // BroadPhaseBvh
        // ============================================================

        /// BVH-based broad-phase: emits candidate collider pairs each step.
        ///
        /// Usually accessed via `world.broad_phase`. Create one explicitly
        /// only if you are driving the pipeline yourself.
        #[pyclass(name = "BroadPhaseBvh", module = "rapier", unsendable)]
        pub struct BroadPhaseBvh(pub rapier::geometry::BroadPhaseBvh);

        #[pymethods]
        impl BroadPhaseBvh {
            /// Construct a broad-phase with the default optimization strategy.
            #[new]
            fn new() -> Self {
                Self(rapier::geometry::BroadPhaseBvh::new())
            }

            /// Construct a broad-phase tuned for `strategy`.
            #[staticmethod]
            fn optimized_for(strategy: BvhOptimizationStrategy) -> Self {
                Self(rapier::geometry::BroadPhaseBvh::with_optimization_strategy(
                    strategy.to_rapier(),
                ))
            }

            /// Reset the broad-phase to an empty state. Equivalent to constructing a new one.
            fn clear(&mut self) {
                self.0 = rapier::geometry::BroadPhaseBvh::new();
            }
        }

        // ============================================================
        // NarrowPhase
        // ============================================================

        /// Narrow-phase: computes per-pair contact manifolds and intersections.
        ///
        /// Usually accessed via `world.narrow_phase`. Use it to query
        /// existing contact pairs and intersections between specific
        /// colliders.
        #[pyclass(name = "NarrowPhase", module = "rapier", unsendable)]
        pub struct NarrowPhase(pub rapier::geometry::NarrowPhase);

        #[pymethods]
        impl NarrowPhase {
            /// Construct an empty narrow-phase.
            #[new]
            fn new() -> Self {
                Self(rapier::geometry::NarrowPhase::new())
            }

            /// Return the contact pair between two colliders, if any.
            ///
            /// :returns: A snapshot `ContactPair` or `None` if the pair is
            ///     not currently tracked by the narrow-phase.
            fn contact_pair(
                &self,
                h1: &ColliderHandle,
                h2: &ColliderHandle,
            ) -> Option<ContactPair> {
                self.0.contact_pair(h1.0, h2.0).cloned().map(ContactPair)
            }

            /// Return whether two sensor/intersection colliders intersect.
            ///
            /// :returns: `True`/`False` if the pair is tracked, or `None`
            ///     when no intersection pair is being maintained.
            fn intersection_pair(&self, h1: &ColliderHandle, h2: &ColliderHandle) -> Option<bool> {
                self.0.intersection_pair(h1.0, h2.0)
            }

            /// Snapshot every active contact pair as a list.
            fn contact_pairs(&self) -> Vec<ContactPair> {
                self.0.contact_pairs().cloned().map(ContactPair).collect()
            }

            /// Snapshot every tracked sensor/intersection pair.
            ///
            /// :returns: List of `(collider1, collider2, intersecting)` tuples.
            fn intersection_pairs(&self) -> Vec<(ColliderHandle, ColliderHandle, bool)> {
                self.0
                    .intersection_pairs()
                    .map(|(h1, h2, i)| (ColliderHandle(h1), ColliderHandle(h2), i))
                    .collect()
            }

            /// Reset the narrow-phase to an empty state. Equivalent to
            /// constructing a new one — the existing `NarrowPhase` doesn't
            /// expose `clear()` directly.
            fn clear(&mut self) {
                self.0 = rapier::geometry::NarrowPhase::new();
            }
        }

        // ============================================================
        // ColliderPair (named-tuple-like)
        // ============================================================

        /// Ordered pair of ``ColliderHandle`` values, indexable like a tuple.
        ///
        /// Used by ``BroadPhasePairEvent`` and various filter callbacks.
        /// ``pair[0]`` returns ``collider1``, ``pair[1]`` returns ``collider2``.
        #[pyclass(name = "ColliderPair", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct ColliderPair {
            #[pyo3(get)]
            pub collider1: ColliderHandle,
            #[pyo3(get)]
            pub collider2: ColliderHandle,
        }

        #[pymethods]
        impl ColliderPair {
            /// Build a pair from two collider handles.
            #[new]
            fn new(collider1: ColliderHandle, collider2: ColliderHandle) -> Self {
                Self {
                    collider1,
                    collider2,
                }
            }
            fn __repr__(&self) -> String {
                format!(
                    "ColliderPair(collider1={:?}, collider2={:?})",
                    self.collider1.0.into_raw_parts(),
                    self.collider2.0.into_raw_parts()
                )
            }
            fn __getitem__(&self, idx: isize) -> PyResult<ColliderHandle> {
                match idx {
                    0 | -2 => Ok(self.collider1),
                    1 | -1 => Ok(self.collider2),
                    _ => Err(PyIndexError::new_err("ColliderPair index out of range")),
                }
            }
            fn __len__(&self) -> usize {
                2
            }
        }

        // ============================================================
        // BroadPhasePairEvent (tagged enum)
        // ============================================================

        /// Pair-event emitted by the broad-phase when an AABB overlap
        /// starts or stops.
        ///
        /// `added=True` corresponds to "Added"; `added=False` to "Removed".
        /// These are coarse, AABB-level events — for actual contacts use
        /// `CollisionEvent` and `ContactPair`.
        #[pyclass(name = "BroadPhasePairEvent", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct BroadPhasePairEvent {
            /// True iff this is an "Added" event; False for "Removed".
            #[pyo3(get)]
            pub added: bool,
            #[pyo3(get)]
            pub pair: ColliderPair,
        }

        #[pymethods]
        impl BroadPhasePairEvent {
            /// Build an "Added" pair event.
            #[staticmethod]
            fn added(pair: ColliderPair) -> Self {
                Self { added: true, pair }
            }
            /// Build a "Removed" pair event.
            #[staticmethod]
            fn removed(pair: ColliderPair) -> Self {
                Self { added: false, pair }
            }
            /// True iff this is a "Removed" event (the inverse of `added`).
            #[getter]
            fn removed_(&self) -> bool {
                !self.added
            }
            fn __repr__(&self) -> String {
                if self.added {
                    format!(
                        "BroadPhasePairEvent.Added({:?})",
                        self.pair.collider1.0.into_raw_parts()
                    )
                } else {
                    format!(
                        "BroadPhasePairEvent.Removed({:?})",
                        self.pair.collider1.0.into_raw_parts()
                    )
                }
            }
        }

        // ============================================================
        // CollisionEvent (tagged union)
        // ============================================================

        /// Contact-level event flagging a new or ended collision pair.
        ///
        /// Only emitted for pairs where at least one collider has
        /// `ActiveEvents.COLLISION_EVENTS` set. Inspect `started` to
        /// distinguish "started" from "stopped", and `flags` for
        /// extra context (`SENSOR`, `REMOVED`).
        #[pyclass(name = "CollisionEvent", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct CollisionEvent {
            #[pyo3(get)]
            pub started: bool,
            #[pyo3(get)]
            pub collider1: ColliderHandle,
            #[pyo3(get)]
            pub collider2: ColliderHandle,
            #[pyo3(get)]
            pub flags: CollisionEventFlags,
        }

        #[pymethods]
        impl CollisionEvent {
            /// Construct a "Started" event.
            ///
            /// Fired when two colliders begin overlapping at the contact
            /// level. Requires `ActiveEvents.COLLISION_EVENTS` on at least
            /// one of the colliders.
            #[staticmethod]
            #[pyo3(name = "STARTED")]
            fn started_evt(
                c1: ColliderHandle,
                c2: ColliderHandle,
                flags: CollisionEventFlags,
            ) -> Self {
                Self {
                    started: true,
                    collider1: c1,
                    collider2: c2,
                    flags,
                }
            }
            /// Construct a "Stopped" event.
            ///
            /// Fired when two colliders stop overlapping (or one is
            /// removed). Requires `ActiveEvents.COLLISION_EVENTS` on at
            /// least one of the colliders.
            #[staticmethod]
            #[pyo3(name = "STOPPED")]
            fn stopped_evt(
                c1: ColliderHandle,
                c2: ColliderHandle,
                flags: CollisionEventFlags,
            ) -> Self {
                Self {
                    started: false,
                    collider1: c1,
                    collider2: c2,
                    flags,
                }
            }

            /// True iff this is a "Stopped" event (the inverse of `started`).
            #[getter]
            fn stopped(&self) -> bool {
                !self.started
            }
            /// True iff the event involves a sensor collider.
            #[getter]
            fn sensor(&self) -> bool {
                self.flags
                    .0
                    .contains(rapier::geometry::CollisionEventFlags::SENSOR)
            }
            /// True iff the event was triggered by a collider removal.
            #[getter]
            fn removed(&self) -> bool {
                self.flags
                    .0
                    .contains(rapier::geometry::CollisionEventFlags::REMOVED)
            }

            fn __repr__(&self) -> String {
                let kind = if self.started { "Started" } else { "Stopped" };
                format!(
                    "CollisionEvent.{kind}(c1={:?}, c2={:?})",
                    self.collider1.0.into_raw_parts(),
                    self.collider2.0.into_raw_parts(),
                )
            }
        }

        impl CollisionEvent {
            #[allow(dead_code)]
            pub(crate) fn from_rapier(e: rapier::geometry::CollisionEvent) -> Self {
                match e {
                    rapier::geometry::CollisionEvent::Started(c1, c2, f) => Self {
                        started: true,
                        collider1: ColliderHandle(c1),
                        collider2: ColliderHandle(c2),
                        flags: CollisionEventFlags(f),
                    },
                    rapier::geometry::CollisionEvent::Stopped(c1, c2, f) => Self {
                        started: false,
                        collider1: ColliderHandle(c1),
                        collider2: ColliderHandle(c2),
                        flags: CollisionEventFlags(f),
                    },
                }
            }
        }

        // ============================================================
        // ColliderFlags (struct-of-fields, not a bitflags wrapper —
        // mirrors rapier's `ColliderFlags` struct exactly).
        // ============================================================

        /// Bundle of the configurable flags on a `Collider`.
        ///
        /// Aggregates `collision_groups`, `solver_groups`, `active_hooks`,
        /// `active_events`, and the read-only `enabled` state. This is a
        /// value type — mutating it does not retroactively affect the
        /// collider it was read from.
        #[pyclass(name = "ColliderFlags", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct ColliderFlags(pub rapier::geometry::ColliderFlags);

        #[pymethods]
        impl ColliderFlags {
            /// Construct with default values (all groups allowed, no hooks/events).
            #[new]
            fn new() -> Self {
                Self(rapier::geometry::ColliderFlags::default())
            }

            /// Groups controlling which colliders generate contact pairs.
            #[getter]
            fn collision_groups(&self) -> InteractionGroups {
                InteractionGroups(self.0.collision_groups)
            }
            #[setter]
            fn set_collision_groups(&mut self, v: InteractionGroups) {
                self.0.collision_groups = v.0;
            }
            /// Groups controlling which contact pairs reach the solver.
            #[getter]
            fn solver_groups(&self) -> InteractionGroups {
                InteractionGroups(self.0.solver_groups)
            }
            #[setter]
            fn set_solver_groups(&mut self, v: InteractionGroups) {
                self.0.solver_groups = v.0;
            }
            /// Hooks this collider opts into (see `ActiveHooks`).
            #[getter]
            fn active_hooks(&self) -> ActiveHooks {
                ActiveHooks(self.0.active_hooks)
            }
            #[setter]
            fn set_active_hooks(&mut self, v: ActiveHooks) {
                self.0.active_hooks = v.0;
            }
            /// Rigid-body type combinations enabled (see `ActiveCollisionTypes`).
            #[getter]
            fn active_collision_types(&self) -> ActiveCollisionTypes {
                ActiveCollisionTypes(self.0.active_collision_types)
            }
            #[setter]
            fn set_active_collision_types(&mut self, v: ActiveCollisionTypes) {
                self.0.active_collision_types = v.0;
            }
            /// Events this collider opts into (see `ActiveEvents`).
            #[getter]
            fn active_events(&self) -> ActiveEvents {
                ActiveEvents(self.0.active_events)
            }
            #[setter]
            fn set_active_events(&mut self, v: ActiveEvents) {
                self.0.active_events = v.0;
            }
            /// Current enabled state. Read-only mirror of `Collider.is_enabled`.
            #[getter]
            fn enabled(&self) -> ColliderEnabled {
                ColliderEnabled::from_rapier(self.0.enabled)
            }
        }
    };
}

// ============================================================
// 3D-specific geometry types
// ============================================================

#[doc(hidden)]
#[macro_export]
macro_rules! __define_geometry_3d {
    () => {
        $crate::__define_geometry_shapes_common!(3, Vec3, Point3, Isometry3);

        // ---- 3D-only shape views ----

        /// Cylinder shape view, axis-aligned to Y.
        ///
        /// Inspection-only — typically obtained via `shape.as_cylinder()`.
        #[pyclass(name = "Cylinder", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct Cylinder(pub rapier::parry::shape::Cylinder);

        #[pymethods]
        impl Cylinder {
            /// Construct a cylinder of given half-height (along Y) and radius.
            #[new]
            fn new(half_height: Real, radius: Real) -> Self {
                Self(rapier::parry::shape::Cylinder::new(half_height, radius))
            }
            /// Half-height of the cylinder along the Y axis.
            #[getter]
            fn half_height(&self) -> Real {
                self.0.half_height
            }
            /// Cap radius.
            #[getter]
            fn radius(&self) -> Real {
                self.0.radius
            }
            fn __repr__(&self) -> String {
                format!(
                    "Cylinder(half_height={}, radius={})",
                    self.0.half_height, self.0.radius
                )
            }
        }

        /// Cone shape view, apex along +Y.
        ///
        /// Inspection-only — typically obtained via `shape.as_cone()`.
        #[pyclass(name = "Cone", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct Cone(pub rapier::parry::shape::Cone);

        #[pymethods]
        impl Cone {
            /// Construct a cone of given half-height (along Y) and base radius.
            #[new]
            fn new(half_height: Real, radius: Real) -> Self {
                Self(rapier::parry::shape::Cone::new(half_height, radius))
            }
            /// Half-height along the Y axis (apex at +half_height, base at -half_height).
            #[getter]
            fn half_height(&self) -> Real {
                self.0.half_height
            }
            /// Base radius.
            #[getter]
            fn radius(&self) -> Real {
                self.0.radius
            }
            fn __repr__(&self) -> String {
                format!(
                    "Cone(half_height={}, radius={})",
                    self.0.half_height, self.0.radius
                )
            }
        }

        /// Convex polyhedron shape view.
        ///
        /// Inspection-only — typically obtained via
        /// `shape.as_convex_polyhedron()`.
        #[pyclass(name = "ConvexPolyhedron", module = "rapier")]
        #[derive(Clone)]
        pub struct ConvexPolyhedron(pub rapier::parry::shape::ConvexPolyhedron);

        #[pymethods]
        impl ConvexPolyhedron {
            /// Vertices of the polyhedron as an `(N, 3)` ndarray.
            #[getter]
            fn points<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<Real>> {
                let pts = self.0.points();
                let n = pts.len();
                let mut flat = Vec::with_capacity(n * 3);
                for p in pts {
                    flat.extend_from_slice(&[p.x, p.y, p.z]);
                }
                PyArray2::from_vec2_bound(
                    py,
                    &flat.chunks(3).map(|c| c.to_vec()).collect::<Vec<_>>(),
                )
                .expect("contiguous 2D ndarray")
            }
            /// Number of vertices.
            fn num_points(&self) -> usize {
                self.0.points().len()
            }

            /// Triangulated face indices as an `(M, 3)` ndarray of `u32`.
            ///
            /// Each (convex) face is fan-triangulated; the indices reference
            /// rows of `points` and tile the polyhedron's surface. This lets a
            /// renderer build the actual mesh straight from parry's topology
            /// instead of recomputing a convex hull.
            #[getter]
            fn indices<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
                let adj = self.0.vertices_adj_to_face();
                let mut tris: Vec<Vec<u32>> = Vec::new();
                for face in self.0.faces() {
                    let first = face.first_vertex_or_edge as usize;
                    let n = face.num_vertices_or_edges as usize;
                    if n < 3 {
                        continue;
                    }
                    let v0 = adj[first];
                    for i in 1..n - 1 {
                        tris.push(vec![v0, adj[first + i], adj[first + i + 1]]);
                    }
                }
                PyArray2::from_vec2_bound(py, &tris).expect("contiguous 2D ndarray")
            }
        }

        // ---- 3D HeightField additional accessor: heights as (nrows, ncols) ndarray ----

        #[pymethods]
        impl HeightField {
            /// Height samples as an `(nrows, ncols)` float ndarray.
            ///
            /// World-space vertex positions are
            /// `(x * scale.x, height[i,j] * scale.y, z * scale.z)`
            /// where `x`/`z` are evenly-spaced in `[-0.5, 0.5]`.
            #[getter]
            fn heights<'py>(&self, py: Python<'py>) -> Bound<'py, $crate::numpy::PyArray2<Real>> {
                let h = self.0.heights();
                let nrows = h.nrows();
                let ncols = h.ncols();
                let rows: Vec<Vec<Real>> = (0..nrows)
                    .map(|i| (0..ncols).map(|j| h[(i, j)]).collect())
                    .collect();
                $crate::numpy::PyArray2::from_vec2_bound(py, &rows).expect("contiguous 2D ndarray")
            }
            /// Number of rows in the height grid (along the X axis).
            #[getter]
            fn nrows(&self) -> usize {
                self.0.heights().nrows()
            }
            /// Number of columns in the height grid (along the Z axis).
            #[getter]
            fn ncols(&self) -> usize {
                self.0.heights().ncols()
            }
        }

        // ---- ContactData (3D — tangent_impulse is 2 floats) ----

        /// Single contact point inside a `ContactManifold`.
        ///
        /// Read-only snapshot. Points are expressed in each collider's
        /// local frame; `dist` is the separation (negative if penetrating);
        /// `impulse` is the normal impulse computed by the solver and
        /// `tangent_impulse` is the friction impulse along the two
        /// tangent directions of the manifold's frame.
        #[pyclass(name = "ContactData", module = "rapier", frozen)]
        #[derive(Debug, Clone)]
        pub struct ContactData {
            #[pyo3(get)]
            pub local_p1: Point3,
            #[pyo3(get)]
            pub local_p2: Point3,
            #[pyo3(get)]
            pub dist: Real,
            #[pyo3(get)]
            pub fid1: u32,
            #[pyo3(get)]
            pub fid2: u32,
            #[pyo3(get)]
            pub impulse: Real,
            #[pyo3(get)]
            pub tangent_impulse: (Real, Real),
            #[pyo3(get)]
            pub contact_id: u32,
        }

        #[pymethods]
        impl ContactData {
            fn __repr__(&self) -> String {
                format!("ContactData(dist={}, impulse={})", self.dist, self.impulse)
            }
        }

        // ---- SolverContact view ----

        /// One contact prepared for the constraints solver.
        ///
        /// Read-only snapshot. `point` is in world coordinates;
        /// `friction`/`restitution` are the per-contact combined material
        /// values; `is_new` is true when the contact is freshly generated
        /// this step.
        #[pyclass(name = "SolverContact", module = "rapier", frozen)]
        #[derive(Debug, Clone)]
        pub struct SolverContact {
            #[pyo3(get)]
            pub point: Point3,
            #[pyo3(get)]
            pub dist: Real,
            #[pyo3(get)]
            pub friction: Real,
            #[pyo3(get)]
            pub restitution: Real,
            #[pyo3(get)]
            pub contact_id: u32,
            #[pyo3(get)]
            pub is_new: bool,
        }
    };
}

// ============================================================
// 2D-specific geometry types
// ============================================================

#[doc(hidden)]
#[macro_export]
macro_rules! __define_geometry_2d {
    () => {
        $crate::__define_geometry_shapes_common!(2, Vec2, Point2, Isometry2);

        // ---- 2D HeightField additional accessor: heights as 1-D ndarray ----

        #[pymethods]
        impl HeightField {
            /// Height samples as a 1-D float ndarray of length `nheights`.
            #[getter]
            fn heights<'py>(&self, py: Python<'py>) -> Bound<'py, $crate::numpy::PyArray1<Real>> {
                let h = self.0.heights();
                $crate::numpy::PyArray1::from_iter_bound(py, h.iter().copied())
            }
            /// Number of height samples.
            #[getter]
            fn nheights(&self) -> usize {
                self.0.heights().len()
            }
        }

        // ---- 2D-only shape views ----

        /// 2D segment between two endpoints.
        ///
        /// Inspection-only — typically obtained via `shape.as_segment()`.
        #[pyclass(name = "Segment", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct Segment(pub rapier::parry::shape::Segment);

        #[pymethods]
        impl Segment {
            /// Build a segment from two endpoints.
            #[new]
            fn new(a: PyVector, b: PyVector) -> Self {
                Self(rapier::parry::shape::Segment::new(a.0.into(), b.0.into()))
            }
            /// First endpoint.
            #[getter]
            fn a(&self) -> Point2 {
                let v: $crate::na::Vector2<Real> = self.0.a.into();
                Point2($crate::na::Point2::from(v))
            }
            /// Second endpoint.
            #[getter]
            fn b(&self) -> Point2 {
                let v: $crate::na::Vector2<Real> = self.0.b.into();
                Point2($crate::na::Point2::from(v))
            }
            fn __repr__(&self) -> String {
                format!("Segment(a={:?}, b={:?})", self.0.a, self.0.b)
            }
        }

        /// 2D polyline shape view.
        ///
        /// Inspection-only — typically obtained via `shape.as_polyline()`.
        #[pyclass(name = "Polyline", module = "rapier")]
        #[derive(Clone)]
        pub struct Polyline(pub rapier::parry::shape::Polyline);

        #[pymethods]
        impl Polyline {
            /// Vertices as an `(N, 2)` ndarray.
            #[getter]
            fn vertices<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<Real>> {
                let pts = self.0.vertices();
                let v: Vec<Vec<Real>> = pts.iter().map(|p| vec![p.x, p.y]).collect();
                PyArray2::from_vec2_bound(py, &v).expect("contiguous ndarray")
            }

            /// Segment indices as an `(M, 2)` ndarray of `u32`.
            #[getter]
            fn indices<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
                let idx = self.0.indices();
                let v: Vec<Vec<u32>> = idx.iter().map(|c| c.to_vec()).collect();
                PyArray2::from_vec2_bound(py, &v).expect("contiguous ndarray")
            }

            /// Number of segments.
            fn num_segments(&self) -> usize {
                self.0.num_segments()
            }
        }

        /// Convex polygon shape view.
        ///
        /// Inspection-only — typically obtained via
        /// `shape.as_convex_polygon()`.
        #[pyclass(name = "ConvexPolygon", module = "rapier")]
        #[derive(Clone)]
        pub struct ConvexPolygon(pub rapier::parry::shape::ConvexPolygon);

        #[pymethods]
        impl ConvexPolygon {
            /// Polygon vertices as an `(N, 2)` ndarray.
            #[getter]
            fn points<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<Real>> {
                let pts = self.0.points();
                let v: Vec<Vec<Real>> = pts.iter().map(|p| vec![p.x, p.y]).collect();
                PyArray2::from_vec2_bound(py, &v).expect("contiguous ndarray")
            }
            /// Number of vertices.
            fn num_points(&self) -> usize {
                self.0.points().len()
            }
        }

        // ---- ContactData (2D — tangent_impulse is 1 float) ----

        /// Single contact point inside a 2D `ContactManifold`.
        ///
        /// Read-only snapshot. Points are expressed in each collider's
        /// local frame; `dist` is the separation (negative if penetrating);
        /// `impulse` is the normal impulse computed by the solver and
        /// `tangent_impulse` is the (scalar) friction impulse along the
        /// tangent direction.
        #[pyclass(name = "ContactData", module = "rapier", frozen)]
        #[derive(Debug, Clone)]
        pub struct ContactData {
            #[pyo3(get)]
            pub local_p1: Point2,
            #[pyo3(get)]
            pub local_p2: Point2,
            #[pyo3(get)]
            pub dist: Real,
            #[pyo3(get)]
            pub fid1: u32,
            #[pyo3(get)]
            pub fid2: u32,
            #[pyo3(get)]
            pub impulse: Real,
            #[pyo3(get)]
            pub tangent_impulse: Real,
            #[pyo3(get)]
            pub contact_id: u32,
        }

        #[pymethods]
        impl ContactData {
            fn __repr__(&self) -> String {
                format!("ContactData(dist={}, impulse={})", self.dist, self.impulse)
            }
        }

        /// One contact prepared for the 2D constraints solver.
        ///
        /// Read-only snapshot. `point` is in world coordinates;
        /// `friction`/`restitution` are the per-contact combined material
        /// values; `is_new` is true when the contact is freshly generated
        /// this step.
        #[pyclass(name = "SolverContact", module = "rapier", frozen)]
        #[derive(Debug, Clone)]
        pub struct SolverContact {
            #[pyo3(get)]
            pub point: Point2,
            #[pyo3(get)]
            pub dist: Real,
            #[pyo3(get)]
            pub friction: Real,
            #[pyo3(get)]
            pub restitution: Real,
            #[pyo3(get)]
            pub contact_id: u32,
            #[pyo3(get)]
            pub is_new: bool,
        }
    };
}

// ============================================================
// Dim-agnostic shape views (Ball, Cuboid, Capsule, Triangle,
// TriMesh, HeightField, Compound) + MeshConverter +
// ContactManifold + ContactManifoldData + ContactPair + IntersectionPair
// ============================================================

#[doc(hidden)]
#[macro_export]
macro_rules! __define_geometry_shapes_common {
    ($dim:tt, $Vec:ident, $Point:ident, $Iso:ident) => {
        // `PyArrayMethods` is already brought in by `define_math_types!` so we
        // only need the new traits here.
        use $crate::numpy::{PyArray2, PyUntypedArrayMethods};

        // Dim+scalar-specific helpers that pick the right ndarray extractor
        // and convert into the engine's `Vector` type.
        $crate::__define_geometry_extract_verts_helper!($dim);

        // ============================================================
        // Ball
        // ============================================================

        /// Sphere (3D) / disk (2D) shape view.
        ///
        /// Inspection-only — typically obtained via `shape.as_ball()`.
        #[pyclass(name = "Ball", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct Ball(pub rapier::parry::shape::Ball);

        #[pymethods]
        impl Ball {
            /// Build a ball of the given radius.
            #[new]
            fn new(radius: Real) -> Self {
                Self(rapier::parry::shape::Ball::new(radius))
            }
            /// Radius.
            #[getter]
            fn radius(&self) -> Real {
                self.0.radius
            }
            fn __repr__(&self) -> String {
                format!("Ball(radius={})", self.0.radius)
            }
        }

        // ============================================================
        // Cuboid
        // ============================================================

        /// Axis-aligned box (3D) / rectangle (2D) shape view.
        ///
        /// Inspection-only — typically obtained via `shape.as_cuboid()`.
        /// The cuboid spans `[-half_extents, +half_extents]` in its local frame.
        #[pyclass(name = "Cuboid", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct Cuboid(pub rapier::parry::shape::Cuboid);

        #[pymethods]
        impl Cuboid {
            /// Build a cuboid from its half-extents vector.
            #[new]
            fn new(half_extents: PyVector) -> Self {
                Self(rapier::parry::shape::Cuboid::new(half_extents.0.into()))
            }
            /// Half-extents along each local axis.
            #[getter]
            fn half_extents(&self) -> $Vec {
                let v: $crate::na::SVector<Real, $dim> = self.0.half_extents.into();
                $Vec(v)
            }
            fn __repr__(&self) -> String {
                format!("Cuboid(half_extents={:?})", self.0.half_extents)
            }
        }

        // ============================================================
        // Capsule
        // ============================================================

        /// Capsule shape view (a swept sphere between two endpoints).
        ///
        /// Inspection-only — typically obtained via `shape.as_capsule()`.
        /// Construct via the explicit `(a, b, radius)` form or via the
        /// `SharedShape.capsule_x` / `capsule_y` / `capsule_z` factories
        /// to align it with a specific axis.
        #[pyclass(name = "Capsule", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct Capsule(pub rapier::parry::shape::Capsule);

        #[pymethods]
        impl Capsule {
            /// Build a capsule from its two endpoints and a radius.
            #[new]
            fn new(a: PyVector, b: PyVector, radius: Real) -> Self {
                Self(rapier::parry::shape::Capsule::new(
                    a.0.into(),
                    b.0.into(),
                    radius,
                ))
            }
            /// Radius of the spherical caps.
            #[getter]
            fn radius(&self) -> Real {
                self.0.radius
            }
            /// Half the length of the inner segment.
            #[getter]
            fn half_height(&self) -> Real {
                self.0.half_height()
            }
            /// Length of the inner segment.
            #[getter]
            fn height(&self) -> Real {
                self.0.height()
            }
            /// First endpoint of the inner segment.
            #[getter]
            fn a(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.segment.a.into();
                $Point($crate::na::Point::from(v))
            }
            /// Second endpoint of the inner segment.
            #[getter]
            fn b(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.segment.b.into();
                $Point($crate::na::Point::from(v))
            }
            fn __repr__(&self) -> String {
                format!(
                    "Capsule(half_height={}, radius={})",
                    self.0.half_height(),
                    self.0.radius
                )
            }
        }

        // ============================================================
        // Triangle
        // ============================================================

        /// Triangle shape view.
        ///
        /// Inspection-only — typically obtained via `shape.as_triangle()`.
        #[pyclass(name = "Triangle", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct Triangle(pub rapier::parry::shape::Triangle);

        #[pymethods]
        impl Triangle {
            /// Build a triangle from its three vertices.
            #[new]
            fn new(a: PyVector, b: PyVector, c: PyVector) -> Self {
                Self(rapier::parry::shape::Triangle::new(
                    a.0.into(),
                    b.0.into(),
                    c.0.into(),
                ))
            }
            /// First vertex.
            #[getter]
            fn a(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.a.into();
                $Point($crate::na::Point::from(v))
            }
            /// Second vertex.
            #[getter]
            fn b(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.b.into();
                $Point($crate::na::Point::from(v))
            }
            /// Third vertex.
            #[getter]
            fn c(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.c.into();
                $Point($crate::na::Point::from(v))
            }
        }

        // ============================================================
        // TriMesh view
        // ============================================================

        /// Triangle mesh shape view.
        ///
        /// Inspection-only — typically obtained via `shape.as_trimesh()`.
        /// Construct new tri-meshes via `SharedShape.trimesh(...)`.
        #[pyclass(name = "TriMesh", module = "rapier")]
        #[derive(Clone)]
        pub struct TriMesh(pub rapier::parry::shape::TriMesh);

        #[pymethods]
        impl TriMesh {
            /// Vertices as an `(N, D)` ndarray.
            #[getter]
            fn vertices<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<Real>> {
                let vs = self.0.vertices();
                let v: Vec<Vec<Real>> = vs
                    .iter()
                    .map(|p| {
                        let arr: [Real; $dim] = (*p).into();
                        arr.to_vec()
                    })
                    .collect();
                PyArray2::from_vec2_bound(py, &v).expect("contiguous ndarray")
            }
            /// Triangle indices as an `(M, 3)` ndarray of `u32`.
            #[getter]
            fn indices<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<u32>> {
                let idx = self.0.indices();
                let v: Vec<Vec<u32>> = idx.iter().map(|c| c.to_vec()).collect();
                PyArray2::from_vec2_bound(py, &v).expect("contiguous ndarray")
            }
            /// Build-time preprocessing flags this mesh was created with.
            #[getter]
            fn flags(&self) -> TriMeshFlags {
                TriMeshFlags(self.0.flags())
            }
            /// Number of triangles.
            fn num_triangles(&self) -> usize {
                self.0.indices().len()
            }
            /// Number of vertices.
            fn num_vertices(&self) -> usize {
                self.0.vertices().len()
            }
        }

        // ============================================================
        // Voxels (voxel-grid shape)
        // ============================================================

        /// Voxel-grid shape view.
        ///
        /// Inspection-only — typically obtained via `shape.as_voxels()`.
        /// Construct one with `SharedShape.voxels(...)` /
        /// `SharedShape.voxels_from_points(...)` (or the matching
        /// `Collider.*` builders).
        #[pyclass(name = "Voxels", module = "rapier")]
        #[derive(Clone)]
        pub struct Voxels(pub rapier::parry::shape::Voxels);

        #[pymethods]
        impl Voxels {
            /// Per-axis size of one voxel cell, as a `(D,)` ndarray.
            #[getter]
            fn voxel_size<'py>(
                &self,
                py: Python<'py>,
            ) -> Bound<'py, $crate::numpy::PyArray1<Real>> {
                let s = self.0.voxel_size();
                let arr: [Real; $dim] = s.into();
                $crate::numpy::PyArray1::from_iter_bound(py, arr.into_iter())
            }

            /// Centers of all **filled** (non-empty) voxels, as an `(M, D)`
            /// ndarray. Building one cuboid (of size `voxel_size`) per center
            /// reconstructs the solid; empty cells are skipped.
            #[getter]
            fn centers<'py>(&self, py: Python<'py>) -> Bound<'py, $crate::numpy::PyArray2<Real>> {
                let v: Vec<Vec<Real>> = self
                    .0
                    .voxels()
                    .filter(|vox| !vox.state.is_empty())
                    .map(|vox| {
                        let arr: [Real; $dim] = vox.center.into();
                        arr.to_vec()
                    })
                    .collect();
                $crate::numpy::PyArray2::from_vec2_bound(py, &v).expect("contiguous ndarray")
            }

            /// Number of filled (non-empty) voxels.
            fn num_voxels(&self) -> usize {
                self.0.voxels().filter(|vox| !vox.state.is_empty()).count()
            }
        }

        // ============================================================
        // HeightField (2D: 1-D heights; 3D: 2-D heights)
        // ============================================================

        /// Heightfield shape view.
        ///
        /// Inspection-only — typically obtained via `shape.as_heightfield()`.
        /// In 2D the heights are a 1-D array; in 3D they are 2-D. Use
        /// `SharedShape.heightfield(...)` to construct one.
        #[pyclass(name = "HeightField", module = "rapier")]
        #[derive(Clone)]
        pub struct HeightField(pub rapier::parry::shape::HeightField);

        #[pymethods]
        impl HeightField {
            /// Per-axis scale applied to the height samples.
            #[getter]
            fn scale(&self) -> $Vec {
                let v: $crate::na::SVector<Real, $dim> = self.0.scale().into();
                $Vec(v)
            }
        }

        // ============================================================
        // Compound
        // ============================================================

        /// Compound shape view — a collection of sub-shapes with local poses.
        ///
        /// Inspection-only — typically obtained via `shape.as_compound()`.
        /// Build one via `SharedShape.compound(...)`.
        #[pyclass(name = "Compound", module = "rapier")]
        #[derive(Clone)]
        pub struct Compound(pub rapier::parry::shape::Compound);

        #[pymethods]
        impl Compound {
            /// Number of sub-shapes.
            fn num_shapes(&self) -> usize {
                self.0.shapes().len()
            }

            /// Returns the list of `(pose, sub_shape)` parts.
            fn shapes(&self) -> Vec<($Iso, SharedShape)> {
                self.0
                    .shapes()
                    .iter()
                    .map(|(pose, shape)| {
                        let iso: $crate::na::Isometry<Real, _, $dim> = (*pose).into();
                        ($Iso(iso), SharedShape(shape.clone()))
                    })
                    .collect()
            }
        }

        // ============================================================
        // SharedShape
        // ============================================================

        /// Reference-counted handle to any collision shape.
        ///
        /// `SharedShape` is the polymorphic type carried by every `Collider`.
        /// Use the `ball`, `cuboid`, `capsule`, `trimesh`, ... static
        /// constructors to build one, then downcast with `as_ball`,
        /// `as_cuboid`, ... to inspect the concrete geometry. Cheap to
        /// clone (it's an `Arc`).
        #[pyclass(name = "SharedShape", module = "rapier")]
        #[derive(Clone)]
        pub struct SharedShape(pub rapier::parry::shape::SharedShape);

        #[pymethods]
        impl SharedShape {
            /// Discriminant identifying the concrete shape type.
            #[getter]
            fn shape_type(&self) -> ShapeType {
                ShapeType::from_rapier(self.0.shape_type())
            }

            /// Build a ball / sphere shape.
            #[staticmethod]
            fn ball(radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::ball(radius))
            }

            /// Build a capsule from two endpoints and a radius.
            ///
            /// For axis-aligned capsules, prefer `capsule_x`, `capsule_y`,
            /// or `capsule_z`.
            #[staticmethod]
            fn capsule(a: PyVector, b: PyVector, radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::capsule(
                    a.0.into(),
                    b.0.into(),
                    radius,
                ))
            }

            /// Build a triangle from its three vertices.
            #[staticmethod]
            fn triangle(a: PyVector, b: PyVector, c: PyVector) -> Self {
                Self(rapier::parry::shape::SharedShape::triangle(
                    a.0.into(),
                    b.0.into(),
                    c.0.into(),
                ))
            }

            /// Build a half-space (infinite plane) shape from its outward normal.
            ///
            /// The half-space is the unbounded region lying on the negative
            /// side of the plane through the origin with the given outward
            /// normal — i.e. the set of points ``p`` with
            /// ``p · outward_normal <= 0``. Commonly used as an immovable
            /// ground or wall. The normal is normalized automatically.
            #[staticmethod]
            fn halfspace(outward_normal: PyVector) -> Self {
                let n = outward_normal.0.normalize();
                Self(rapier::parry::shape::SharedShape::halfspace(n.into()))
            }

            /// Build a triangle with rounded edges of radius `border_radius`.
            #[staticmethod]
            fn round_triangle(a: PyVector, b: PyVector, c: PyVector, border_radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::round_triangle(
                    a.0.into(),
                    b.0.into(),
                    c.0.into(),
                    border_radius,
                ))
            }

            /// Build the convex hull of a point cloud, dilated by
            /// `border_radius` (rounded corners/edges).
            ///
            /// :raises MeshConversionError: If the hull cannot be built.
            #[staticmethod]
            fn round_convex_hull(points: &Bound<'_, PyAny>, border_radius: Real) -> PyResult<Self> {
                let pts = extract_verts_for_dim(points)?;
                rapier::parry::shape::SharedShape::round_convex_hull(&pts, border_radius)
                    .map(Self)
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "round convex hull computation failed",
                        )
                    })
            }

            /// Build a voxel-grid shape by voxelizing a point cloud.
            ///
            /// :param voxel_size: Per-axis size of one voxel cell.
            /// :param points: `(N, D)` ndarray of floats; each point marks the
            ///     voxel that contains it as filled.
            #[staticmethod]
            fn voxels_from_points(
                voxel_size: PyVector,
                points: &Bound<'_, PyAny>,
            ) -> PyResult<Self> {
                let pts = extract_verts_for_dim(points)?;
                Ok(Self(rapier::parry::shape::SharedShape::voxels_from_points(
                    voxel_size.0.into(),
                    &pts,
                )))
            }

            /// Build a compound shape from a list of `(pose, sub_shape)` pairs.
            #[staticmethod]
            fn compound(parts: Vec<(PyIsometry, SharedShape)>) -> Self {
                let parts: Vec<(rapier::math::Pose, rapier::parry::shape::SharedShape)> =
                    parts.into_iter().map(|(p, s)| (p.0.into(), s.0)).collect();
                Self(rapier::parry::shape::SharedShape::compound(parts))
            }

            /// Compute the world-space AABB enclosing this shape when
            /// placed at `pose`.
            fn compute_aabb(&self, pose: PyIsometry) -> Aabb {
                let p: rapier::math::Pose = pose.0.into();
                Aabb(self.0.compute_aabb(&p))
            }

            /// Compute the world-space bounding sphere enclosing this shape
            /// when placed at `pose`.
            fn compute_bounding_sphere(&self, pose: PyIsometry) -> BoundingSphere {
                let p: rapier::math::Pose = pose.0.into();
                BoundingSphere(self.0.compute_bounding_sphere(&p))
            }

            /// Compute mass properties (mass, center of mass, inertia
            /// tensor) for this shape at the given uniform density.
            fn compute_mass_properties(&self, density: Real) -> MassProperties {
                MassProperties(self.0.mass_properties(density))
            }
            /// Alias of `compute_mass_properties`.
            fn mass_properties(&self, density: Real) -> MassProperties {
                MassProperties(self.0.mass_properties(density))
            }

            /// Downcast to `Ball` if this is a ball shape.
            fn as_ball(&self) -> Option<Ball> {
                self.0.as_ball().map(|s| Ball(*s))
            }
            /// Downcast to `Cuboid` if this is a cuboid shape.
            fn as_cuboid(&self) -> Option<Cuboid> {
                self.0.as_cuboid().map(|s| Cuboid(*s))
            }
            /// Downcast to `Capsule` if this is a capsule shape.
            fn as_capsule(&self) -> Option<Capsule> {
                self.0.as_capsule().map(|s| Capsule(*s))
            }
            /// Downcast to `Triangle` if this is a triangle shape.
            fn as_triangle(&self) -> Option<Triangle> {
                self.0.as_triangle().map(|s| Triangle(*s))
            }
            /// Downcast to `TriMesh` if this is a triangle mesh shape.
            fn as_trimesh(&self) -> Option<TriMesh> {
                self.0.as_trimesh().map(|s| TriMesh(s.clone()))
            }
            /// Downcast to `HeightField` if this is a heightfield shape.
            fn as_heightfield(&self) -> Option<HeightField> {
                self.0.as_heightfield().map(|s| HeightField(s.clone()))
            }
            /// Downcast to `Compound` if this is a compound shape.
            fn as_compound(&self) -> Option<Compound> {
                self.0.as_compound().map(|s| Compound(s.clone()))
            }
            /// Downcast to `Voxels` if this is a voxel-grid shape.
            fn as_voxels(&self) -> Option<Voxels> {
                self.0.as_voxels().map(|s| Voxels(s.clone()))
            }

            /// Downcast to `Cuboid` if this is a *rounded* cuboid shape.
            ///
            /// Returns the inner (unrounded) `Cuboid`; the border radius is
            /// discarded. Use `shape_type` to detect the round variant.
            fn as_round_cuboid(&self) -> Option<Cuboid> {
                self.0.as_round_cuboid().map(|s| Cuboid(s.inner_shape))
            }
            /// Downcast to `Triangle` if this is a *rounded* triangle shape.
            ///
            /// Returns the inner (unrounded) `Triangle`; the border radius
            /// is discarded.
            fn as_round_triangle(&self) -> Option<Triangle> {
                self.0.as_round_triangle().map(|s| Triangle(s.inner_shape))
            }

            fn __repr__(&self) -> String {
                format!("SharedShape(type={:?})", self.0.shape_type())
            }
        }

        // Dim-specific SharedShape statics as a separate pymethods block.
        $crate::__define_geometry_sharedshape_dim_specific!($dim);

        // ============================================================
        // Aabb (small wrapper over parry::bounding_volume::Aabb)
        // ============================================================

        /// Axis-aligned bounding box.
        ///
        /// Stored as a `(mins, maxs)` pair in world coordinates. Returned
        /// by `Collider.compute_aabb()` and `SharedShape.compute_aabb()`.
        #[pyclass(name = "Aabb", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct Aabb(pub rapier::parry::bounding_volume::Aabb);

        #[pymethods]
        impl Aabb {
            /// Build an AABB from its `mins` and `maxs` corners.
            #[new]
            fn new(mins: PyVector, maxs: PyVector) -> Self {
                let mins: rapier::math::Vector = mins.0.into();
                let maxs: rapier::math::Vector = maxs.0.into();
                Self(rapier::parry::bounding_volume::Aabb::new(
                    mins.into(),
                    maxs.into(),
                ))
            }
            /// Lower corner.
            #[getter]
            fn mins(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.mins.into();
                $Point($crate::na::Point::from(v))
            }
            /// Upper corner.
            #[getter]
            fn maxs(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.maxs.into();
                $Point($crate::na::Point::from(v))
            }
            /// Geometric center of the AABB.
            #[getter]
            fn center(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.center().into();
                $Point($crate::na::Point::from(v))
            }
            /// Half-extents along each axis.
            #[getter]
            fn half_extents(&self) -> $Vec {
                let v: $crate::na::SVector<Real, $dim> = self.0.half_extents().into();
                $Vec(v)
            }
            /// Volume (3D) or area (2D) of the AABB.
            #[getter]
            fn volume(&self) -> Real {
                self.0.volume()
            }
        }

        // ============================================================
        // BoundingSphere
        // ============================================================

        /// Bounding sphere defined by a center and radius.
        ///
        /// Returned by `SharedShape.compute_bounding_sphere()`.
        #[pyclass(name = "BoundingSphere", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct BoundingSphere(pub rapier::parry::bounding_volume::BoundingSphere);

        #[pymethods]
        impl BoundingSphere {
            /// Build a bounding sphere from its `center` and `radius`.
            #[new]
            fn new(center: PyVector, radius: Real) -> Self {
                let c: rapier::math::Vector = center.0.into();
                Self(rapier::parry::bounding_volume::BoundingSphere { center: c, radius })
            }
            /// Center of the sphere.
            #[getter]
            fn center(&self) -> $Point {
                let v: $crate::na::SVector<Real, $dim> = self.0.center.into();
                $Point($crate::na::Point::from(v))
            }
            /// Radius of the sphere.
            #[getter]
            fn radius(&self) -> Real {
                self.0.radius
            }
        }

        // ============================================================
        // MeshConverter (tagged-union-ish; one variant constructor at a time)
        // ============================================================

        /// Strategy for turning a `(vertices, indices)` buffer pair into a
        /// `SharedShape`.
        ///
        /// Picks the conversion variant via the class attributes
        /// (`TRIMESH`, `OBB`, `BOUNDING_BOX`, `CONVEX_HULL`, `COMPOUND`) or
        /// the `trimesh_with_flags(...)` factory, then call `build()`.
        #[pyclass(name = "MeshConverter", module = "rapier")]
        #[derive(Clone)]
        pub struct MeshConverter(pub rapier::geometry::MeshConverter);

        #[pymethods]
        impl MeshConverter {
            /// Convert vertices/indices into a `TriMesh` shape.
            #[classattr]
            fn TRIMESH() -> Self {
                Self(rapier::geometry::MeshConverter::TriMesh)
            }
            /// Convert into a tightly-fitting oriented bounding box (`Cuboid`).
            #[classattr]
            fn OBB() -> Self {
                Self(rapier::geometry::MeshConverter::Obb)
            }
            /// Convert into an axis-aligned bounding box (`Cuboid`).
            #[classattr]
            fn BOUNDING_BOX() -> Self {
                Self(rapier::geometry::MeshConverter::Aabb)
            }
            /// Convert into a convex hull of the input vertices.
            #[classattr]
            fn CONVEX_HULL() -> Self {
                Self(rapier::geometry::MeshConverter::ConvexHull)
            }

            /// Convert into a `TriMesh` using the provided preprocessing
            /// `flags`.
            #[staticmethod]
            fn trimesh_with_flags(flags: TriMeshFlags) -> Self {
                Self(rapier::geometry::MeshConverter::TriMeshWithFlags(flags.0))
            }

            /// Build a `SharedShape` from a (vertices, indices) buffer pair.
            ///
            /// `vertices` is a (N, D) ndarray of floats; `indices` is a (M, 3)
            /// ndarray of u32. Raises `MeshConversionError` on failure.
            fn build(
                &self,
                py: Python<'_>,
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
            ) -> PyResult<(SharedShape, $Iso)> {
                let _ = py;
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                let result = self
                    .0
                    .convert(verts, idx)
                    .map_err(|e| $crate::errors::MeshConversionError::new_err(format!("{e}")))?;
                let iso: $crate::na::Isometry<Real, _, $dim> = result.1.into();
                Ok((SharedShape(result.0), $Iso(iso)))
            }
        }

        $crate::__define_meshconverter_dim_specific!($dim);

        // ============================================================
        // ContactManifoldData
        // ============================================================

        /// Per-manifold metadata accompanying a `ContactManifold`.
        ///
        /// Read-only view. Holds the parent rigid-body handles (if any),
        /// the world-space contact normal, the number of solver-active
        /// contacts, the relative dominance used by the solver, and a
        /// user-data payload.
        #[pyclass(name = "ContactManifoldData", module = "rapier", frozen)]
        #[derive(Debug, Clone)]
        pub struct ContactManifoldData {
            #[pyo3(get)]
            pub rigid_body1: Option<RigidBodyHandle>,
            #[pyo3(get)]
            pub rigid_body2: Option<RigidBodyHandle>,
            #[pyo3(get)]
            pub normal: $Vec,
            #[pyo3(get)]
            pub num_active_contacts: usize,
            #[pyo3(get)]
            pub relative_dominance: i16,
            #[pyo3(get)]
            pub user_data: u32,
        }

        // ============================================================
        // ContactManifold
        // ============================================================

        /// Set of contact points sharing a common normal between two colliders.
        ///
        /// Read-only view. `points` are the individual `ContactData` entries;
        /// `local_n1` / `local_n2` are the contact normal expressed in each
        /// collider's local frame; `subshape1` / `subshape2` identify the
        /// sub-shapes inside compound shapes (0 for non-compounds).
        #[pyclass(name = "ContactManifold", module = "rapier", frozen)]
        #[derive(Debug, Clone)]
        pub struct ContactManifold {
            #[pyo3(get)]
            pub data: ContactManifoldData,
            #[pyo3(get)]
            pub points: Vec<ContactData>,
            #[pyo3(get)]
            pub local_n1: $Vec,
            #[pyo3(get)]
            pub local_n2: $Vec,
            #[pyo3(get)]
            pub subshape1: u32,
            #[pyo3(get)]
            pub subshape2: u32,
        }

        // ============================================================
        // ContactPair (clone of rapier::geometry::ContactPair, but
        // converted into Python-friendly shape views)
        // ============================================================

        /// Read-only view of the contacts between two colliders.
        ///
        /// Holds one or more `ContactManifold`s plus helpers to query the
        /// deepest contact and the total impulse. Snapshots are returned
        /// by `NarrowPhase.contact_pair(...)` and
        /// `NarrowPhase.contact_pairs()`; mutating them does not affect
        /// the simulation.
        #[pyclass(name = "ContactPair", module = "rapier")]
        #[derive(Clone)]
        pub struct ContactPair(pub rapier::geometry::ContactPair);

        #[pymethods]
        impl ContactPair {
            /// Handle of the first collider.
            #[getter]
            fn collider1(&self) -> ColliderHandle {
                ColliderHandle(self.0.collider1)
            }
            /// Handle of the second collider.
            #[getter]
            fn collider2(&self) -> ColliderHandle {
                ColliderHandle(self.0.collider2)
            }

            /// List of contact manifolds for this pair.
            #[getter]
            fn manifolds(&self) -> Vec<ContactManifold> {
                self.0
                    .manifolds
                    .iter()
                    .map(|m| {
                        let normal_v: $crate::na::SVector<Real, $dim> = m.data.normal.into();
                        let n1: $crate::na::SVector<Real, $dim> = m.local_n1.into();
                        let n2: $crate::na::SVector<Real, $dim> = m.local_n2.into();

                        let points: Vec<ContactData> = m
                            .points
                            .iter()
                            .map(|c| $crate::__define_geometry_contact_point_to_py!($dim, c))
                            .collect();

                        ContactManifold {
                            data: ContactManifoldData {
                                rigid_body1: m.data.rigid_body1.map(RigidBodyHandle),
                                rigid_body2: m.data.rigid_body2.map(RigidBodyHandle),
                                normal: $Vec(normal_v),
                                num_active_contacts: m.data.solver_contacts.len(),
                                relative_dominance: m.data.relative_dominance,
                                user_data: m.data.user_data,
                            },
                            points,
                            local_n1: $Vec(n1),
                            local_n2: $Vec(n2),
                            subshape1: m.subshape1,
                            subshape2: m.subshape2,
                        }
                    })
                    .collect()
            }

            /// True iff any manifold has at least one solver-active contact.
            #[getter]
            fn has_any_active_contact(&self) -> bool {
                self.0.has_any_active_contact()
            }

            /// Return the deepest (most penetrating) `ContactData`, if any.
            fn find_deepest_contact(&self) -> Option<ContactData> {
                self.0
                    .find_deepest_contact()
                    .map(|(_, c)| $crate::__define_geometry_contact_point_to_py!($dim, c))
            }

            /// Sum of contact impulses across all manifolds, as a world-space vector.
            fn total_impulse(&self) -> $Vec {
                let v = self.0.total_impulse();
                let nav: $crate::na::SVector<Real, $dim> = v.into();
                $Vec(nav)
            }

            /// Magnitude of `total_impulse()`.
            fn total_impulse_magnitude(&self) -> Real {
                self.0.total_impulse_magnitude()
            }
        }

        // ============================================================
        // IntersectionPair (small view)
        // ============================================================

        /// Snapshot of one sensor/intersection pair tracked by the narrow-phase.
        ///
        /// Read-only view. `intersecting` is true while the pair currently
        /// overlaps.
        #[pyclass(name = "IntersectionPair", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct IntersectionPair {
            #[pyo3(get)]
            pub collider1: ColliderHandle,
            #[pyo3(get)]
            pub collider2: ColliderHandle,
            #[pyo3(get)]
            pub intersecting: bool,
        }

        // ============================================================
        // ContactForceEvent (plain dataclass-style)
        // ============================================================

        /// Event fired when the contact force magnitude between two
        /// colliders exceeds the configured threshold.
        ///
        /// Requires `ActiveEvents.CONTACT_FORCE_EVENTS` on at least one
        /// collider; the threshold is set via
        /// `Collider.contact_force_event_threshold` (or its builder).
        /// `total_force` is the sum of contact impulses divided by the
        /// time step; `max_force_direction` and `max_force_magnitude`
        /// summarize the strongest single contact.
        #[pyclass(name = "ContactForceEvent", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct ContactForceEvent {
            #[pyo3(get)]
            pub collider1: ColliderHandle,
            #[pyo3(get)]
            pub collider2: ColliderHandle,
            #[pyo3(get)]
            pub total_force: $Vec,
            #[pyo3(get)]
            pub total_force_magnitude: Real,
            #[pyo3(get)]
            pub max_force_direction: $Vec,
            #[pyo3(get)]
            pub max_force_magnitude: Real,
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_geometry_sharedshape_dim_specific {
    (3) => {
        #[pymethods]
        impl SharedShape {
            /// Build a 3D axis-aligned box of given half-extents.
            #[staticmethod]
            fn cuboid(hx: Real, hy: Real, hz: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::cuboid(hx, hy, hz))
            }
            /// Build a 3D cuboid with rounded edges of radius `border_radius`.
            #[staticmethod]
            fn round_cuboid(hx: Real, hy: Real, hz: Real, border_radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::round_cuboid(
                    hx,
                    hy,
                    hz,
                    border_radius,
                ))
            }
            /// Build a Y-axis cylinder.
            #[staticmethod]
            fn cylinder(half_height: Real, radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::cylinder(
                    half_height,
                    radius,
                ))
            }
            /// Build a Y-axis cone (apex at +Y).
            #[staticmethod]
            fn cone(half_height: Real, radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::cone(half_height, radius))
            }
            /// Build a Y-axis cylinder with rounded edges.
            #[staticmethod]
            fn round_cylinder(half_height: Real, radius: Real, border_radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::round_cylinder(
                    half_height,
                    radius,
                    border_radius,
                ))
            }
            /// Build a Y-axis cone with a rounded base/apex.
            #[staticmethod]
            fn round_cone(half_height: Real, radius: Real, border_radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::round_cone(
                    half_height,
                    radius,
                    border_radius,
                ))
            }
            /// Build a capsule aligned with the X axis.
            #[staticmethod]
            fn capsule_x(half_height: Real, radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::capsule_x(
                    half_height,
                    radius,
                ))
            }
            /// Build a capsule aligned with the Y axis (the default).
            #[staticmethod]
            fn capsule_y(half_height: Real, radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::capsule_y(
                    half_height,
                    radius,
                ))
            }
            /// Build a capsule aligned with the Z axis.
            #[staticmethod]
            fn capsule_z(half_height: Real, radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::capsule_z(
                    half_height,
                    radius,
                ))
            }
            /// Build a triangle mesh from `(vertices, indices)` buffers.
            ///
            /// :param vertices: `(N, 3)` ndarray of floats.
            /// :param indices: `(M, 3)` ndarray of `u32`.
            /// :param flags: Optional preprocessing flags (`TriMeshFlags`).
            /// :raises MeshConversionError: If the mesh cannot be built.
            #[staticmethod]
            #[pyo3(signature = (vertices, indices, flags=None))]
            fn trimesh(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
                flags: Option<TriMeshFlags>,
            ) -> PyResult<Self> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                let shape = match flags {
                    None => rapier::parry::shape::SharedShape::trimesh(verts, idx),
                    Some(f) => {
                        rapier::parry::shape::SharedShape::trimesh_with_flags(verts, idx, f.0)
                    }
                }
                .map_err(|e| $crate::errors::MeshConversionError::new_err(format!("{e:?}")))?;
                Ok(Self(shape))
            }

            /// Build a convex hull around the given 3D point cloud.
            ///
            /// :raises MeshConversionError: If fewer than 4 non-coplanar
            ///     points were supplied.
            #[staticmethod]
            fn convex_hull(points: &Bound<'_, PyAny>) -> PyResult<Self> {
                let pts = extract_verts_for_dim(points)?;
                rapier::parry::shape::SharedShape::convex_hull(&pts)
                    .map(Self)
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "convex hull computation failed (need ≥ 4 non-coplanar points)",
                        )
                    })
            }

            /// Alias of `convex_hull` (3D).
            #[staticmethod]
            fn convex_polyhedron(points: &Bound<'_, PyAny>) -> PyResult<Self> {
                // Alias for convex_hull in 3D since parry's convex_polyhedron
                // takes already-prepared points.
                Self::convex_hull(points)
            }

            /// Build a convex mesh from vertices that are *already* convex.
            ///
            /// Unlike `convex_hull`, no hull is recomputed; the caller
            /// guarantees convexity via the `(vertices, indices)` buffers.
            ///
            /// :param vertices: `(N, 3)` ndarray of floats.
            /// :param indices: `(M, 3)` ndarray of `u32`.
            /// :raises MeshConversionError: If the mesh is not a valid convex mesh.
            #[staticmethod]
            fn convex_mesh(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
            ) -> PyResult<Self> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                rapier::parry::shape::SharedShape::convex_mesh(verts, &idx)
                    .map(Self)
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "convex mesh construction failed (invalid convex mesh)",
                        )
                    })
            }

            /// Build a convex mesh dilated by `border_radius` (rounded edges).
            ///
            /// :raises MeshConversionError: If the mesh is not a valid convex mesh.
            #[staticmethod]
            fn round_convex_mesh(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
                border_radius: Real,
            ) -> PyResult<Self> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                rapier::parry::shape::SharedShape::round_convex_mesh(verts, &idx, border_radius)
                    .map(Self)
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "round convex mesh construction failed (invalid convex mesh)",
                        )
                    })
            }

            /// Build a voxel-grid shape from explicit integer grid coordinates.
            ///
            /// :param voxel_size: Per-axis size of one voxel cell.
            /// :param grid_coords: Sequence of `(i, j, k)` integer cell
            ///     coordinates marking the filled voxels.
            #[staticmethod]
            fn voxels(voxel_size: PyVector, grid_coords: &Bound<'_, PyAny>) -> PyResult<Self> {
                let raw: Vec<(i64, i64, i64)> = grid_coords.extract()?;
                let coords: Vec<rapier::math::IVector> = raw
                    .iter()
                    .map(|&(x, y, z)| rapier::math::IVector::new(x as _, y as _, z as _))
                    .collect();
                Ok(Self(rapier::parry::shape::SharedShape::voxels(
                    voxel_size.0.into(),
                    &coords,
                )))
            }

            /// Decompose a triangle soup into a compound of convex pieces.
            ///
            /// :param vertices: `(N, 3)` ndarray of floats.
            /// :param indices: `(M, 3)` ndarray of `u32`.
            #[staticmethod]
            #[pyo3(signature = (vertices, indices))]
            fn convex_decomposition(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
            ) -> PyResult<Self> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                Ok(Self(
                    rapier::parry::shape::SharedShape::convex_decomposition(&verts, &idx),
                ))
            }

            /// Build a 3D heightfield from a 2-D `heights` ndarray.
            ///
            /// :param heights: `(rows, cols)` ndarray of `Real`.
            /// :param scale: Per-axis scaling vector.
            #[staticmethod]
            fn heightfield(heights: &Bound<'_, PyAny>, scale: PyVector) -> PyResult<Self> {
                let arr: $crate::numpy::PyReadonlyArray2<Real> = heights.extract()?;
                let slice = arr.as_slice().map_err(|_| {
                    $crate::pyo3::exceptions::PyValueError::new_err(
                        "heights ndarray must be contiguous",
                    )
                })?;
                let nrows = arr.shape()[0];
                let ncols = arr.shape()[1];
                let arr2 = rapier::parry::utils::Array2::new(nrows, ncols, slice.to_vec());
                Ok(Self(rapier::parry::shape::SharedShape::heightfield(
                    arr2,
                    scale.0.into(),
                )))
            }

            /// Downcast to `Cylinder` if this is a cylinder shape.
            fn as_cylinder(&self) -> Option<Cylinder> {
                self.0.as_cylinder().map(|s| Cylinder(*s))
            }
            /// Downcast to `Cone` if this is a cone shape.
            fn as_cone(&self) -> Option<Cone> {
                self.0.as_cone().map(|s| Cone(*s))
            }
            /// Downcast to `ConvexPolyhedron` if this is a convex polyhedron.
            fn as_convex_polyhedron(&self) -> Option<ConvexPolyhedron> {
                self.0
                    .as_convex_polyhedron()
                    .map(|s| ConvexPolyhedron(s.clone()))
            }
            /// Downcast to `Cylinder` if this is a *rounded* cylinder.
            ///
            /// Returns the inner (unrounded) `Cylinder`; the border radius
            /// is discarded.
            fn as_round_cylinder(&self) -> Option<Cylinder> {
                self.0.as_round_cylinder().map(|s| Cylinder(s.inner_shape))
            }
            /// Downcast to `Cone` if this is a *rounded* cone.
            ///
            /// Returns the inner (unrounded) `Cone`; the border radius is
            /// discarded.
            fn as_round_cone(&self) -> Option<Cone> {
                self.0.as_round_cone().map(|s| Cone(s.inner_shape))
            }
            /// Downcast to `ConvexPolyhedron` if this is a *rounded* convex
            /// polyhedron. Returns the inner (unrounded) polyhedron.
            fn as_round_convex_polyhedron(&self) -> Option<ConvexPolyhedron> {
                self.0
                    .as_round_convex_polyhedron()
                    .map(|s| ConvexPolyhedron(s.inner_shape.clone()))
            }
        } // end impl SharedShape
    };

    (2) => {
        #[pymethods]
        impl SharedShape {
            /// Build a 2D axis-aligned rectangle of given half-extents.
            #[staticmethod]
            fn cuboid(hx: Real, hy: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::cuboid(hx, hy))
            }
            /// Build a 2D rectangle with rounded corners of radius `border_radius`.
            #[staticmethod]
            fn round_cuboid(hx: Real, hy: Real, border_radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::round_cuboid(
                    hx,
                    hy,
                    border_radius,
                ))
            }
            /// Build a 2D capsule aligned with the X axis.
            #[staticmethod]
            fn capsule_x(half_height: Real, radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::capsule_x(
                    half_height,
                    radius,
                ))
            }
            /// Build a 2D capsule aligned with the Y axis (the default).
            #[staticmethod]
            fn capsule_y(half_height: Real, radius: Real) -> Self {
                Self(rapier::parry::shape::SharedShape::capsule_y(
                    half_height,
                    radius,
                ))
            }
            /// Build a 2D segment shape from two endpoints.
            #[staticmethod]
            fn segment(a: PyVector, b: PyVector) -> Self {
                Self(rapier::parry::shape::SharedShape::segment(
                    a.0.into(),
                    b.0.into(),
                ))
            }
            /// Build a 2D polyline from `(vertices, indices)`.
            ///
            /// :param vertices: `(N, 2)` ndarray of floats.
            /// :param indices: Optional `(M, 2)` ndarray of `u32`. When
            ///     omitted, consecutive vertices form the segments.
            #[staticmethod]
            #[pyo3(signature = (vertices, indices=None))]
            fn polyline(
                vertices: &Bound<'_, PyAny>,
                indices: Option<&Bound<'_, PyAny>>,
            ) -> PyResult<Self> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = match indices {
                    Some(i) => Some($crate::geometry::extract_indices_2(i)?),
                    None => None,
                };
                Ok(Self(rapier::parry::shape::SharedShape::polyline(
                    verts, idx,
                )))
            }
            /// Build a 2D convex polygon from already-convex points.
            ///
            /// :raises MeshConversionError: If fewer than 3 non-collinear
            ///     points were supplied.
            #[staticmethod]
            fn convex_polygon(points: &Bound<'_, PyAny>) -> PyResult<Self> {
                let pts = extract_verts_for_dim(points)?;
                rapier::parry::shape::SharedShape::convex_polyline(pts)
                    .map(Self)
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "convex polygon construction failed (need ≥ 3 non-collinear points)",
                        )
                    })
            }
            /// Build a 2D convex hull around the given point cloud.
            #[staticmethod]
            fn convex_hull(points: &Bound<'_, PyAny>) -> PyResult<Self> {
                let pts = extract_verts_for_dim(points)?;
                rapier::parry::shape::SharedShape::convex_hull(&pts)
                    .map(Self)
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "convex hull computation failed",
                        )
                    })
            }
            /// Build a triangle mesh from `(vertices, indices)` buffers.
            ///
            /// :param vertices: `(N, 2)` ndarray of floats.
            /// :param indices: `(M, 3)` ndarray of `u32`.
            /// :param flags: Optional preprocessing flags (`TriMeshFlags`).
            #[staticmethod]
            #[pyo3(signature = (vertices, indices, flags=None))]
            fn trimesh(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
                flags: Option<TriMeshFlags>,
            ) -> PyResult<Self> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                let shape = match flags {
                    None => rapier::parry::shape::SharedShape::trimesh(verts, idx),
                    Some(f) => {
                        rapier::parry::shape::SharedShape::trimesh_with_flags(verts, idx, f.0)
                    }
                }
                .map_err(|e| $crate::errors::MeshConversionError::new_err(format!("{e:?}")))?;
                Ok(Self(shape))
            }
            /// Build a 2D heightfield from a 1-D `heights` vector and a scale.
            #[staticmethod]
            fn heightfield(heights: Vec<Real>, scale: PyVector) -> Self {
                Self(rapier::parry::shape::SharedShape::heightfield(
                    heights,
                    scale.0.into(),
                ))
            }

            /// Build a 2D convex polyline from points assumed to form a convex
            /// polygon (the points are *not* re-hulled, only sanitized).
            ///
            /// :raises MeshConversionError: If the polyline is degenerate.
            #[staticmethod]
            fn convex_polyline(points: &Bound<'_, PyAny>) -> PyResult<Self> {
                let pts = extract_verts_for_dim(points)?;
                rapier::parry::shape::SharedShape::convex_polyline(pts)
                    .map(Self)
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "convex polyline construction failed",
                        )
                    })
            }

            /// Build a 2D convex polyline dilated by `border_radius`
            /// (rounded corners).
            ///
            /// :raises MeshConversionError: If the polyline is degenerate.
            #[staticmethod]
            fn round_convex_polyline(
                points: &Bound<'_, PyAny>,
                border_radius: Real,
            ) -> PyResult<Self> {
                let pts = extract_verts_for_dim(points)?;
                rapier::parry::shape::SharedShape::round_convex_polyline(pts, border_radius)
                    .map(Self)
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "round convex polyline construction failed",
                        )
                    })
            }

            /// Build a voxel-grid shape from explicit integer grid coordinates.
            ///
            /// :param voxel_size: Per-axis size of one voxel cell.
            /// :param grid_coords: Sequence of `(i, j)` integer cell
            ///     coordinates marking the filled voxels.
            #[staticmethod]
            fn voxels(voxel_size: PyVector, grid_coords: &Bound<'_, PyAny>) -> PyResult<Self> {
                let raw: Vec<(i64, i64)> = grid_coords.extract()?;
                let coords: Vec<rapier::math::IVector> = raw
                    .iter()
                    .map(|&(x, y)| rapier::math::IVector::new(x as _, y as _))
                    .collect();
                Ok(Self(rapier::parry::shape::SharedShape::voxels(
                    voxel_size.0.into(),
                    &coords,
                )))
            }

            /// Downcast to `Segment` if this is a segment shape.
            fn as_segment(&self) -> Option<Segment> {
                self.0.as_segment().map(|s| Segment(*s))
            }
            /// Downcast to `Polyline` if this is a polyline shape.
            fn as_polyline(&self) -> Option<Polyline> {
                self.0.as_polyline().map(|s| Polyline(s.clone()))
            }
            /// Downcast to `ConvexPolygon` if this is a convex polygon.
            fn as_convex_polygon(&self) -> Option<ConvexPolygon> {
                self.0.as_convex_polygon().map(|s| ConvexPolygon(s.clone()))
            }
            /// Downcast to `ConvexPolygon` if this is a *rounded* convex
            /// polygon. Returns the inner (unrounded) polygon.
            fn as_round_convex_polygon(&self) -> Option<ConvexPolygon> {
                self.0
                    .as_round_convex_polygon()
                    .map(|s| ConvexPolygon(s.inner_shape.clone()))
            }
        } // end impl SharedShape
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_meshconverter_dim_specific {
    (3) => {
        #[pymethods]
        impl MeshConverter {
            /// Convert into a compound shape via convex decomposition.
            #[classattr]
            fn COMPOUND() -> Self {
                Self(rapier::geometry::MeshConverter::ConvexDecomposition)
            }
            /// Convert into a compound of convex pieces via convex
            /// decomposition (V-HACD, default parameters). 3D only.
            #[classattr]
            fn CONVEX_DECOMPOSITION() -> Self {
                Self(rapier::geometry::MeshConverter::ConvexDecomposition)
            }
        }
    };
    (2) => {
        #[pymethods]
        impl MeshConverter {
            /// Convert into a compound shape. Currently falls back to OBB
            /// in 2D since convex decomposition is not yet available.
            // No ConvexDecomposition in 2D in parry; reuse OBB as a sensible
            // fallback so the COMPOUND constant exists. May be revisited.
            #[classattr]
            fn COMPOUND() -> Self {
                Self(rapier::geometry::MeshConverter::Obb)
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_geometry_extract_verts_helper {
    (3) => {
        /// Extract vertex array (N, 3) and convert into `Vec<rapier::math::Vector>`.
        ///
        /// Accepts both `float32` and `float64` ndarrays (cast to `Real`), as
        /// well as list/tuple of triples.
        #[allow(dead_code)]
        fn extract_verts_for_dim(
            obj: &$crate::pyo3::Bound<'_, $crate::pyo3::PyAny>,
        ) -> $crate::pyo3::PyResult<Vec<rapier::math::Vector>> {
            use $crate::numpy::{PyArrayMethods, PyReadonlyArray2, PyUntypedArrayMethods};
            // Try the cdylib's matching-precision ndarray first (zero-copy).
            if let Ok(arr) = obj.extract::<PyReadonlyArray2<Real>>() {
                let (nrows, ncols) = (arr.shape()[0], arr.shape()[1]);
                if ncols != 3 {
                    return Err($crate::pyo3::exceptions::PyValueError::new_err(format!(
                        "expected ndarray with shape (N, 3); got (N, {ncols})"
                    )));
                }
                let slice = arr.as_slice().map_err(|_| {
                    $crate::pyo3::exceptions::PyValueError::new_err("ndarray must be contiguous")
                })?;
                let mut out = Vec::with_capacity(nrows);
                for c in slice.chunks_exact(3) {
                    out.push(rapier::math::Vector::new(c[0], c[1], c[2]));
                }
                return Ok(out);
            }
            // Try the other precision (lossy cast).
            if let Ok(arr) = obj.extract::<PyReadonlyArray2<f32>>() {
                let (nrows, ncols) = (arr.shape()[0], arr.shape()[1]);
                if ncols != 3 {
                    return Err($crate::pyo3::exceptions::PyValueError::new_err(format!(
                        "expected ndarray with shape (N, 3); got (N, {ncols})"
                    )));
                }
                let slice = arr.as_slice().map_err(|_| {
                    $crate::pyo3::exceptions::PyValueError::new_err("ndarray must be contiguous")
                })?;
                let mut out = Vec::with_capacity(nrows);
                for c in slice.chunks_exact(3) {
                    out.push(rapier::math::Vector::new(
                        c[0] as Real,
                        c[1] as Real,
                        c[2] as Real,
                    ));
                }
                return Ok(out);
            }
            if let Ok(arr) = obj.extract::<PyReadonlyArray2<f64>>() {
                let (nrows, ncols) = (arr.shape()[0], arr.shape()[1]);
                if ncols != 3 {
                    return Err($crate::pyo3::exceptions::PyValueError::new_err(format!(
                        "expected ndarray with shape (N, 3); got (N, {ncols})"
                    )));
                }
                let slice = arr.as_slice().map_err(|_| {
                    $crate::pyo3::exceptions::PyValueError::new_err("ndarray must be contiguous")
                })?;
                let mut out = Vec::with_capacity(nrows);
                for c in slice.chunks_exact(3) {
                    out.push(rapier::math::Vector::new(
                        c[0] as Real,
                        c[1] as Real,
                        c[2] as Real,
                    ));
                }
                return Ok(out);
            }
            // Fall back to list/tuple of tuples / any sequence of length 3.
            let seq: Vec<(Real, Real, Real)> = obj.extract()?;
            Ok(seq
                .into_iter()
                .map(|(x, y, z)| rapier::math::Vector::new(x, y, z))
                .collect())
        }
    };
    (2) => {
        #[allow(dead_code)]
        fn extract_verts_for_dim(
            obj: &$crate::pyo3::Bound<'_, $crate::pyo3::PyAny>,
        ) -> $crate::pyo3::PyResult<Vec<rapier::math::Vector>> {
            use $crate::numpy::{PyArrayMethods, PyReadonlyArray2, PyUntypedArrayMethods};
            if let Ok(arr) = obj.extract::<PyReadonlyArray2<Real>>() {
                let (nrows, ncols) = (arr.shape()[0], arr.shape()[1]);
                if ncols != 2 {
                    return Err($crate::pyo3::exceptions::PyValueError::new_err(format!(
                        "expected ndarray with shape (N, 2); got (N, {ncols})"
                    )));
                }
                let slice = arr.as_slice().map_err(|_| {
                    $crate::pyo3::exceptions::PyValueError::new_err("ndarray must be contiguous")
                })?;
                let mut out = Vec::with_capacity(nrows);
                for c in slice.chunks_exact(2) {
                    out.push(rapier::math::Vector::new(c[0], c[1]));
                }
                return Ok(out);
            }
            if let Ok(arr) = obj.extract::<PyReadonlyArray2<f32>>() {
                let (nrows, ncols) = (arr.shape()[0], arr.shape()[1]);
                if ncols != 2 {
                    return Err($crate::pyo3::exceptions::PyValueError::new_err(format!(
                        "expected ndarray with shape (N, 2); got (N, {ncols})"
                    )));
                }
                let slice = arr.as_slice().map_err(|_| {
                    $crate::pyo3::exceptions::PyValueError::new_err("ndarray must be contiguous")
                })?;
                let mut out = Vec::with_capacity(nrows);
                for c in slice.chunks_exact(2) {
                    out.push(rapier::math::Vector::new(c[0] as Real, c[1] as Real));
                }
                return Ok(out);
            }
            if let Ok(arr) = obj.extract::<PyReadonlyArray2<f64>>() {
                let (nrows, ncols) = (arr.shape()[0], arr.shape()[1]);
                if ncols != 2 {
                    return Err($crate::pyo3::exceptions::PyValueError::new_err(format!(
                        "expected ndarray with shape (N, 2); got (N, {ncols})"
                    )));
                }
                let slice = arr.as_slice().map_err(|_| {
                    $crate::pyo3::exceptions::PyValueError::new_err("ndarray must be contiguous")
                })?;
                let mut out = Vec::with_capacity(nrows);
                for c in slice.chunks_exact(2) {
                    out.push(rapier::math::Vector::new(c[0] as Real, c[1] as Real));
                }
                return Ok(out);
            }
            let seq: Vec<(Real, Real)> = obj.extract()?;
            Ok(seq
                .into_iter()
                .map(|(x, y)| rapier::math::Vector::new(x, y))
                .collect())
        }
    };
}

// ============================================================
// Collider / ColliderBuilder / ColliderSet — dim-agnostic
// definitions (the Vec/Point/Iso identifiers are passed in).
// ============================================================

#[doc(hidden)]
#[macro_export]
macro_rules! __define_geometry_collider {
    ($dim:tt, $Vec:ident, $Point:ident, $Rot:ident, $Iso:ident) => {
        // ============================================================
        // Collider — owned standalone value (mirrors RigidBody pattern).
        // ============================================================

        /// Standalone collider: a shape + material + per-collider flags
        /// that participates in collision detection.
        ///
        /// A collider may be attached to a `RigidBody` via
        /// `ColliderSet.insert_with_parent(...)`. Use the static shape
        /// factories (`Collider.ball`, `Collider.cuboid`, etc.) to obtain
        /// a `ColliderBuilder` and chain configuration calls, then
        /// `.build()` to get a `Collider`. A collider fetched from
        /// `ColliderSet[handle]` is a live **view**: reads and writes go
        /// straight through to the set with no copy, so mutations persist
        /// immediately.
        #[pyclass(name = "Collider", module = "rapier")]
        pub struct Collider {
            pub backing: ColliderBacking,
        }

        /// Storage backing a `Collider`: a standalone owned collider
        /// (pre-insertion) or a handle-backed view into a `ColliderSet`.
        #[derive(Debug)]
        pub enum ColliderBacking {
            Owned(Box<rapier::geometry::Collider>),
            InSet {
                set: Py<ColliderSet>,
                handle: rapier::geometry::ColliderHandle,
            },
        }

        impl Clone for ColliderBacking {
            fn clone(&self) -> Self {
                match self {
                    ColliderBacking::Owned(c) => ColliderBacking::Owned(c.clone()),
                    ColliderBacking::InSet { set, handle } => {
                        Python::with_gil(|py| ColliderBacking::InSet {
                            set: set.clone_ref(py),
                            handle: *handle,
                        })
                    }
                }
            }
        }
        impl Clone for Collider {
            fn clone(&self) -> Self {
                Collider {
                    backing: self.backing.clone(),
                }
            }
        }

        impl Collider {
            fn new_owned(collider: rapier::geometry::Collider) -> Self {
                Collider {
                    backing: ColliderBacking::Owned(Box::new(collider)),
                }
            }

            /// Run `f` with a shared reference to the underlying collider.
            fn with_ref<R>(&self, f: impl FnOnce(&rapier::geometry::Collider) -> R) -> R {
                match &self.backing {
                    ColliderBacking::Owned(c) => f(c),
                    ColliderBacking::InSet { set, handle } => Python::with_gil(|py| {
                        let set = set.bind(py).borrow();
                        let c = set
                            .0
                            .get(*handle)
                            .expect("Collider refers to a collider that was removed from its set");
                        f(c)
                    }),
                }
            }

            /// Run `f` with a mutable reference to the underlying collider,
            /// writing straight through to the set for an `InSet` view.
            fn with_mut<R>(&mut self, f: impl FnOnce(&mut rapier::geometry::Collider) -> R) -> R {
                match &mut self.backing {
                    ColliderBacking::Owned(c) => f(c),
                    ColliderBacking::InSet { set, handle } => Python::with_gil(|py| {
                        let mut set = set.bind(py).borrow_mut();
                        let c = set
                            .0
                            .get_mut(*handle)
                            .expect("Collider refers to a collider that was removed from its set");
                        f(c)
                    }),
                }
            }

            /// Clone the underlying collider out (used by `insert` and callers
            /// needing an owned `&rapier::Collider`).
            pub fn to_owned_collider(&self) -> rapier::geometry::Collider {
                self.with_ref(|c| c.clone())
            }
        }

        #[pymethods]
        impl Collider {
            // ---- read-only handle/parent ----

            /// Handle of the parent rigid-body, if any.
            #[getter]
            fn parent(&self) -> Option<RigidBodyHandle> {
                self.with_ref(|c| c.parent()).map(RigidBodyHandle)
            }

            // ---- position / translation / rotation ----

            /// World-space pose of the collider.
            #[getter]
            fn position(&self) -> $Iso {
                self.with_ref(|c| {
                    let iso: $crate::na::Isometry<Real, _, $dim> = (*c.position()).into();
                    $Iso(iso)
                })
            }
            #[setter]
            fn set_position(&mut self, p: PyIsometry) {
                self.with_mut(|c| c.set_position(p.0.into()));
            }
            /// World-space translation component of the pose.
            #[getter]
            fn translation(&self) -> $Vec {
                self.with_ref(|c| {
                    let v: $crate::na::SVector<Real, $dim> = c.translation().into();
                    $Vec(v)
                })
            }
            #[setter]
            fn set_translation(&mut self, v: PyVector) {
                self.with_mut(|c| c.set_translation(v.0.into()));
            }
            /// World-space rotation component of the pose.
            #[getter]
            fn rotation(&self) -> $Rot {
                self.with_ref(|c| $Rot(c.rotation().into()))
            }
            #[setter]
            fn set_rotation(&mut self, r: PyRotation) {
                self.with_mut(|c| c.set_rotation(r.0.into()));
            }

            // ---- shape ----

            /// Underlying collision shape.
            #[getter]
            fn shape(&self) -> SharedShape {
                self.with_ref(|c| SharedShape(c.shared_shape().clone()))
            }
            #[setter]
            fn set_shape(&mut self, s: SharedShape) {
                self.with_mut(|c| c.set_shape(s.0));
            }

            // ---- mass / density / mass_properties ----

            /// Uniform density used to derive mass when no explicit mass is set.
            #[getter]
            fn density(&self) -> Real {
                self.with_ref(|c| c.density())
            }
            #[setter]
            fn set_density(&mut self, v: Real) {
                self.with_mut(|c| c.set_density(v));
            }
            /// Explicit mass of the collider.
            #[getter]
            fn mass(&self) -> Real {
                self.with_ref(|c| c.mass())
            }
            #[setter]
            fn set_mass(&mut self, v: Real) {
                self.with_mut(|c| c.set_mass(v));
            }
            /// Full mass properties (mass, center of mass, inertia tensor).
            #[getter]
            fn mass_properties(&self) -> MassProperties {
                MassProperties(self.with_ref(|c| c.mass_properties()))
            }
            #[setter]
            fn set_mass_properties(&mut self, mp: MassProperties) {
                self.with_mut(|c| c.set_mass_properties(mp.0));
            }
            /// Volume (3D) or area (2D) of the shape.
            #[getter]
            fn volume(&self) -> Real {
                self.with_ref(|c| c.volume())
            }

            // ---- material ----

            /// Friction coefficient.
            #[getter]
            fn friction(&self) -> Real {
                self.with_ref(|c| c.friction())
            }
            #[setter]
            fn set_friction(&mut self, v: Real) {
                self.with_mut(|c| c.set_friction(v));
            }
            /// Restitution coefficient (bounciness).
            #[getter]
            fn restitution(&self) -> Real {
                self.with_ref(|c| c.restitution())
            }
            #[setter]
            fn set_restitution(&mut self, v: Real) {
                self.with_mut(|c| c.set_restitution(v));
            }
            /// Rule used to combine friction with another collider's friction.
            #[getter]
            fn friction_combine_rule(&self) -> CoefficientCombineRule {
                CoefficientCombineRule::from_rapier(self.with_ref(|c| c.friction_combine_rule()))
            }
            #[setter]
            fn set_friction_combine_rule(&mut self, v: CoefficientCombineRule) {
                self.with_mut(|c| c.set_friction_combine_rule(v.to_rapier()));
            }
            /// Rule used to combine restitution with another collider.
            #[getter]
            fn restitution_combine_rule(&self) -> CoefficientCombineRule {
                CoefficientCombineRule::from_rapier(self.with_ref(|c| c.restitution_combine_rule()))
            }
            #[setter]
            fn set_restitution_combine_rule(&mut self, v: CoefficientCombineRule) {
                self.with_mut(|c| c.set_restitution_combine_rule(v.to_rapier()));
            }
            /// Material bundle (friction, restitution, and their combine rules).
            #[getter]
            fn material(&self) -> ColliderMaterial {
                self.with_ref(|c| ColliderMaterial(*c.material()))
            }

            // ---- sensor / enabled ----

            /// True iff this collider is a sensor (no contact response).
            #[getter]
            fn is_sensor(&self) -> bool {
                self.with_ref(|c| c.is_sensor())
            }
            #[setter]
            fn set_is_sensor(&mut self, v: bool) {
                self.with_mut(|c| c.set_sensor(v));
            }
            /// True iff the collider is currently enabled.
            #[getter]
            fn is_enabled(&self) -> bool {
                self.with_ref(|c| c.is_enabled())
            }
            #[setter]
            fn set_is_enabled(&mut self, v: bool) {
                self.with_mut(|c| c.set_enabled(v));
            }

            // ---- hooks / events ----

            /// Event flags opted into by this collider (see `ActiveEvents`).
            #[getter]
            fn active_events(&self) -> ActiveEvents {
                ActiveEvents(self.with_ref(|c| c.active_events()))
            }
            #[setter]
            fn set_active_events(&mut self, v: ActiveEvents) {
                self.with_mut(|c| c.set_active_events(v.0));
            }
            /// Hook flags opted into by this collider (see `ActiveHooks`).
            #[getter]
            fn active_hooks(&self) -> ActiveHooks {
                ActiveHooks(self.with_ref(|c| c.active_hooks()))
            }
            #[setter]
            fn set_active_hooks(&mut self, v: ActiveHooks) {
                self.with_mut(|c| c.set_active_hooks(v.0));
            }
            /// Rigid-body type combinations this collider collides with
            /// (see `ActiveCollisionTypes`).
            #[getter]
            fn active_collision_types(&self) -> ActiveCollisionTypes {
                ActiveCollisionTypes(self.with_ref(|c| c.active_collision_types()))
            }
            #[setter]
            fn set_active_collision_types(&mut self, v: ActiveCollisionTypes) {
                self.with_mut(|c| c.set_active_collision_types(v.0));
            }

            // ---- groups ----

            /// Groups controlling which colliders form contact pairs.
            #[getter]
            fn collision_groups(&self) -> InteractionGroups {
                InteractionGroups(self.with_ref(|c| c.collision_groups()))
            }
            #[setter]
            fn set_collision_groups(&mut self, v: InteractionGroups) {
                self.with_mut(|c| c.set_collision_groups(v.0));
            }
            /// Groups controlling which contact pairs reach the solver.
            #[getter]
            fn solver_groups(&self) -> InteractionGroups {
                InteractionGroups(self.with_ref(|c| c.solver_groups()))
            }
            #[setter]
            fn set_solver_groups(&mut self, v: InteractionGroups) {
                self.with_mut(|c| c.set_solver_groups(v.0));
            }

            // ---- contact skin & event threshold ----

            /// Thickness of the virtual "skin" around the collider, used to
            /// reduce jitter on resting contacts.
            #[getter]
            fn contact_skin(&self) -> Real {
                self.with_ref(|c| c.contact_skin())
            }
            #[setter]
            fn set_contact_skin(&mut self, v: Real) {
                self.with_mut(|c| c.set_contact_skin(v));
            }
            /// Force magnitude above which a `ContactForceEvent` is emitted.
            ///
            /// Requires `ActiveEvents.CONTACT_FORCE_EVENTS`.
            #[getter]
            fn contact_force_event_threshold(&self) -> Real {
                self.with_ref(|c| c.contact_force_event_threshold())
            }
            #[setter]
            fn set_contact_force_event_threshold(&mut self, v: Real) {
                self.with_mut(|c| c.set_contact_force_event_threshold(v));
            }

            // ---- user_data ----

            /// Free 128-bit user payload (opaque to the engine).
            #[getter]
            fn user_data(&self) -> u128 {
                self.with_ref(|c| c.user_data)
            }
            #[setter]
            fn set_user_data(&mut self, v: u128) {
                self.with_mut(|c| c.user_data = v);
            }

            // ---- compute helpers ----

            /// Compute the world-space AABB of this collider.
            fn compute_aabb(&self) -> Aabb {
                Aabb(self.with_ref(|c| c.compute_aabb()))
            }

            // ---- Builder forwards (mirror RigidBody.dynamic etc) ----

            /// Builder for a ball/sphere collider of the given radius.
            ///
            /// :param radius: Sphere radius.
            /// :param kwargs: Optional `ColliderBuilder` kwargs (e.g.
            ///     `density`, `friction`, `sensor`, `translation`,
            ///     `rotation`).
            #[staticmethod]
            #[pyo3(signature = (radius, **kwargs))]
            fn ball(
                radius: Real,
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<ColliderBuilder> {
                ColliderBuilder::from_kwargs(
                    rapier::geometry::ColliderBuilder::ball(radius),
                    kwargs,
                )
            }
            /// Builder for a triangle collider with the given vertices.
            #[staticmethod]
            fn triangle(a: PyVector, b: PyVector, c: PyVector) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::triangle(
                        a.0.into(),
                        b.0.into(),
                        c.0.into(),
                    ),
                }
            }
            /// Builder for a half-space (infinite plane) collider.
            ///
            /// :param outward_normal: Plane normal; the solid half-space lies
            ///     on its negative side. Normalized automatically.
            /// :param kwargs: Optional `ColliderBuilder` kwargs (e.g.
            ///     `friction`, `translation`, `rotation`). A half-space has no
            ///     finite volume, so leave `density`/`mass` unset on bodies
            ///     that use it.
            #[staticmethod]
            #[pyo3(signature = (outward_normal, **kwargs))]
            fn halfspace(
                outward_normal: PyVector,
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<ColliderBuilder> {
                let n = outward_normal.0.normalize();
                ColliderBuilder::from_kwargs(
                    rapier::geometry::ColliderBuilder::new(
                        rapier::parry::shape::SharedShape::halfspace(n.into()),
                    ),
                    kwargs,
                )
            }
            /// Builder wrapping an arbitrary `SharedShape`.
            #[staticmethod]
            #[pyo3(signature = (shape))]
            fn new(shape: SharedShape) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::new(shape.0),
                }
            }
            /// Builder for a compound collider built from `(pose, sub_shape)` parts.
            #[staticmethod]
            fn compound(parts: Vec<(PyIsometry, SharedShape)>) -> ColliderBuilder {
                let parts: Vec<(rapier::math::Pose, rapier::parry::shape::SharedShape)> =
                    parts.into_iter().map(|(p, s)| (p.0.into(), s.0)).collect();
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::compound(parts),
                }
            }
            /// Builder for a triangle-mesh collider.
            ///
            /// :param vertices: `(N, D)` ndarray of floats.
            /// :param indices: `(M, 3)` ndarray of `u32`.
            /// :param flags: Optional `TriMeshFlags` preprocessing.
            /// :raises MeshConversionError: If the mesh cannot be built.
            #[staticmethod]
            #[pyo3(signature = (vertices, indices, flags=None))]
            fn trimesh(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
                flags: Option<TriMeshFlags>,
            ) -> PyResult<ColliderBuilder> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                let b = match flags {
                    None => rapier::geometry::ColliderBuilder::trimesh(verts, idx),
                    Some(f) => {
                        rapier::geometry::ColliderBuilder::trimesh_with_flags(verts, idx, f.0)
                    }
                }
                .map_err(|e| $crate::errors::MeshConversionError::new_err(format!("{e:?}")))?;
                Ok(ColliderBuilder { builder: b })
            }

            /// Builder for a collider wrapping the convex hull of a point cloud.
            ///
            /// :raises MeshConversionError: If the convex hull cannot be built.
            #[staticmethod]
            fn convex_hull(points: &Bound<'_, PyAny>) -> PyResult<ColliderBuilder> {
                let pts = extract_verts_for_dim(points)?;
                rapier::geometry::ColliderBuilder::convex_hull(&pts)
                    .map(|b| ColliderBuilder { builder: b })
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "convex hull computation failed",
                        )
                    })
            }
            /// Builder for a triangle collider with rounded edges.
            #[staticmethod]
            fn round_triangle(
                a: PyVector,
                b: PyVector,
                c: PyVector,
                border_radius: Real,
            ) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::round_triangle(
                        a.0.into(),
                        b.0.into(),
                        c.0.into(),
                        border_radius,
                    ),
                }
            }
            /// Builder for a rounded convex-hull collider.
            ///
            /// :raises MeshConversionError: If the hull cannot be built.
            #[staticmethod]
            fn round_convex_hull(
                points: &Bound<'_, PyAny>,
                border_radius: Real,
            ) -> PyResult<ColliderBuilder> {
                let pts = extract_verts_for_dim(points)?;
                rapier::geometry::ColliderBuilder::round_convex_hull(&pts, border_radius)
                    .map(|b| ColliderBuilder { builder: b })
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "round convex hull computation failed",
                        )
                    })
            }
            /// Builder for a voxel-grid collider built by voxelizing points.
            ///
            /// :param voxel_size: Per-axis size of one voxel cell.
            /// :param points: `(N, D)` ndarray of floats.
            #[staticmethod]
            fn voxels_from_points(
                voxel_size: PyVector,
                points: &Bound<'_, PyAny>,
            ) -> PyResult<ColliderBuilder> {
                let pts = extract_verts_for_dim(points)?;
                Ok(ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::voxels_from_points(
                        voxel_size.0.into(),
                        &pts,
                    ),
                })
            }
            /// Builder for a collider whose shape is produced from a triangle
            /// buffer by a `MeshConverter` (trimesh, hull, OBB, AABB,
            /// convex decomposition, ...).
            ///
            /// :raises MeshConversionError: If the conversion fails.
            #[staticmethod]
            fn converted_trimesh(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
                converter: &MeshConverter,
            ) -> PyResult<ColliderBuilder> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                rapier::geometry::ColliderBuilder::converted_trimesh(
                    verts,
                    idx,
                    converter.0.clone(),
                )
                .map(|b| ColliderBuilder { builder: b })
                .map_err(|e| $crate::errors::MeshConversionError::new_err(format!("{e}")))
            }

            // Capsule constructors (parry uses `capsule_from_endpoints`).
            /// Builder for a Y-axis capsule (the default axis).
            #[staticmethod]
            fn capsule(half_height: Real, radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::capsule_y(half_height, radius),
                }
            }
            /// Builder for an X-axis capsule.
            #[staticmethod]
            fn capsule_x(half_height: Real, radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::capsule_x(half_height, radius),
                }
            }
            /// Builder for a Y-axis capsule.
            #[staticmethod]
            fn capsule_y(half_height: Real, radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::capsule_y(half_height, radius),
                }
            }
            /// Builder for a capsule defined by its two endpoints and a radius.
            #[staticmethod]
            fn capsule_from_endpoints(a: PyVector, b: PyVector, radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::capsule_from_endpoints(
                        a.0.into(),
                        b.0.into(),
                        radius,
                    ),
                }
            }

            fn __repr__(&self) -> String {
                format!(
                    "Collider(shape={:?}, sensor={}, parent={:?})",
                    self.with_ref(|c| c.shape().shape_type()),
                    self.with_ref(|c| c.is_sensor()),
                    self.with_ref(|c| c.parent()).map(|h| h.into_raw_parts()),
                )
            }
        }

        // Emit the dim-specific Collider statics as a separate pymethods
        // block (multiple-pymethods feature).
        $crate::__define_collider_dim_specific!($dim);

        // ============================================================
        // ColliderBuilder
        // ============================================================

        /// Fluent builder for `Collider` values.
        ///
        /// Build one via a shape factory on `Collider` (e.g.
        /// `Collider.ball(radius)`, `Collider.cuboid(hx, hy, hz)`), chain
        /// configuration calls (`density`, `friction`, `sensor`, ...),
        /// then call `build()` to obtain a `Collider`.
        #[pyclass(name = "ColliderBuilder", module = "rapier")]
        #[derive(Clone)]
        pub struct ColliderBuilder {
            pub builder: rapier::geometry::ColliderBuilder,
        }

        impl ColliderBuilder {
            pub fn from_kwargs(
                base: rapier::geometry::ColliderBuilder,
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
                    "density" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().density(f);
                    }
                    "mass" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().mass(f);
                    }
                    "friction" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().friction(f);
                    }
                    "restitution" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().restitution(f);
                    }
                    "sensor" => {
                        let b: bool = v.extract()?;
                        self.builder = self.builder.clone().sensor(b);
                    }
                    "translation" => {
                        let pv: PyVector = v.extract()?;
                        self.builder = self.builder.clone().translation(pv.0.into());
                    }
                    "rotation" => {
                        $crate::__collider_apply_rotation_kwarg!($dim, self.builder, v);
                    }
                    "position" => {
                        let pi: PyIsometry = v.extract()?;
                        self.builder = self.builder.clone().position(pi.0.into());
                    }
                    "user_data" => {
                        let d: u128 = v.extract()?;
                        self.builder = self.builder.clone().user_data(d);
                    }
                    "enabled" => {
                        let b: bool = v.extract()?;
                        self.builder = self.builder.clone().enabled(b);
                    }
                    "contact_skin" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().contact_skin(f);
                    }
                    "contact_force_event_threshold" => {
                        let f: Real = v.extract()?;
                        self.builder = self.builder.clone().contact_force_event_threshold(f);
                    }
                    "active_events" => {
                        let e: ActiveEvents = v.extract()?;
                        self.builder = self.builder.clone().active_events(e.0);
                    }
                    "active_hooks" => {
                        let h: ActiveHooks = v.extract()?;
                        self.builder = self.builder.clone().active_hooks(h.0);
                    }
                    "active_collision_types" => {
                        let t: ActiveCollisionTypes = v.extract()?;
                        self.builder = self.builder.clone().active_collision_types(t.0);
                    }
                    "collision_groups" => {
                        let g: InteractionGroups = v.extract()?;
                        self.builder = self.builder.clone().collision_groups(g.0);
                    }
                    "solver_groups" => {
                        let g: InteractionGroups = v.extract()?;
                        self.builder = self.builder.clone().solver_groups(g.0);
                    }
                    "friction_combine_rule" => {
                        let r: CoefficientCombineRule = v.extract()?;
                        self.builder = self.builder.clone().friction_combine_rule(r.to_rapier());
                    }
                    "restitution_combine_rule" => {
                        let r: CoefficientCombineRule = v.extract()?;
                        self.builder = self.builder.clone().restitution_combine_rule(r.to_rapier());
                    }
                    "mass_properties" => {
                        let mp: MassProperties = v.extract()?;
                        self.builder = self.builder.clone().mass_properties(mp.0);
                    }
                    _ => {
                        return Err(PyTypeError::new_err(format!(
                            "unknown ColliderBuilder kwarg: '{}'",
                            key
                        )));
                    }
                }
                Ok(())
            }
        }

        #[pymethods]
        impl ColliderBuilder {
            /// Set the world-space translation. Returns a new builder.
            fn translation(&self, v: PyVector) -> Self {
                Self {
                    builder: self.builder.clone().translation(v.0.into()),
                }
            }
            /// Set the world-space pose. Returns a new builder.
            fn position(&self, p: PyIsometry) -> Self {
                Self {
                    builder: self.builder.clone().position(p.0.into()),
                }
            }
            /// Set the uniform density. Returns a new builder.
            fn density(&self, d: Real) -> Self {
                Self {
                    builder: self.builder.clone().density(d),
                }
            }
            /// Set an explicit mass (overrides density). Returns a new builder.
            fn mass(&self, m: Real) -> Self {
                Self {
                    builder: self.builder.clone().mass(m),
                }
            }
            /// Set the full mass properties. Returns a new builder.
            fn mass_properties(&self, mp: &MassProperties) -> Self {
                Self {
                    builder: self.builder.clone().mass_properties(mp.0),
                }
            }
            /// Set the friction coefficient. Returns a new builder.
            fn friction(&self, f: Real) -> Self {
                Self {
                    builder: self.builder.clone().friction(f),
                }
            }
            /// Set the restitution coefficient. Returns a new builder.
            fn restitution(&self, r: Real) -> Self {
                Self {
                    builder: self.builder.clone().restitution(r),
                }
            }
            /// Set the friction combine rule. Returns a new builder.
            fn friction_combine_rule(&self, r: CoefficientCombineRule) -> Self {
                Self {
                    builder: self.builder.clone().friction_combine_rule(r.to_rapier()),
                }
            }
            /// Set the restitution combine rule. Returns a new builder.
            fn restitution_combine_rule(&self, r: CoefficientCombineRule) -> Self {
                Self {
                    builder: self.builder.clone().restitution_combine_rule(r.to_rapier()),
                }
            }
            /// Toggle sensor mode (no contact response). Returns a new builder.
            fn sensor(&self, b: bool) -> Self {
                Self {
                    builder: self.builder.clone().sensor(b),
                }
            }
            /// Set the opt-in event flags. Returns a new builder.
            fn active_events(&self, e: &ActiveEvents) -> Self {
                Self {
                    builder: self.builder.clone().active_events(e.0),
                }
            }
            /// Set the opt-in hook flags. Returns a new builder.
            fn active_hooks(&self, h: &ActiveHooks) -> Self {
                Self {
                    builder: self.builder.clone().active_hooks(h.0),
                }
            }
            /// Set the enabled rigid-body-type collision combinations
            /// (see `ActiveCollisionTypes`). Returns a new builder.
            fn active_collision_types(&self, t: &ActiveCollisionTypes) -> Self {
                Self {
                    builder: self.builder.clone().active_collision_types(t.0),
                }
            }
            /// Set the collision groups. Returns a new builder.
            fn collision_groups(&self, g: &InteractionGroups) -> Self {
                Self {
                    builder: self.builder.clone().collision_groups(g.0),
                }
            }
            /// Set the solver groups. Returns a new builder.
            fn solver_groups(&self, g: &InteractionGroups) -> Self {
                Self {
                    builder: self.builder.clone().solver_groups(g.0),
                }
            }
            /// Set the contact skin thickness. Returns a new builder.
            fn contact_skin(&self, f: Real) -> Self {
                Self {
                    builder: self.builder.clone().contact_skin(f),
                }
            }
            /// Set the contact-force event threshold. Returns a new builder.
            fn contact_force_event_threshold(&self, f: Real) -> Self {
                Self {
                    builder: self.builder.clone().contact_force_event_threshold(f),
                }
            }
            /// Set the user-data payload. Returns a new builder.
            fn user_data(&self, d: u128) -> Self {
                Self {
                    builder: self.builder.clone().user_data(d),
                }
            }
            /// Toggle the initial enabled state. Returns a new builder.
            fn enabled(&self, b: bool) -> Self {
                Self {
                    builder: self.builder.clone().enabled(b),
                }
            }

            /// Set the rotation. Returns a new builder.
            ///
            /// In 2D the argument is an angle in radians; in 3D it is a
            /// rotation vector (axis * angle).
            fn rotation(&self, v: &Bound<'_, PyAny>) -> PyResult<Self> {
                let mut b = self.builder.clone();
                $crate::__collider_apply_rotation_kwarg!($dim, b, v);
                Ok(Self { builder: b })
            }

            /// Finalize the builder into a `Collider`.
            fn build(&self) -> Collider {
                Collider::new_owned(self.builder.clone().build())
            }
        }

        // ============================================================
        // ColliderSet
        // ============================================================

        /// Owning container of `Collider`s identified by `ColliderHandle`.
        ///
        /// Acts like a dict keyed by handles: supports `len(set)`,
        /// `handle in set`, `set[handle]`, iteration, plus `insert` and
        /// `remove`. `set[handle]` returns a live view, so mutating it
        /// (e.g. `set[h].set_sensor(True)`) persists in place.
        #[pyclass(name = "ColliderSet", module = "rapier", unsendable)]
        pub struct ColliderSet(pub rapier::geometry::ColliderSet);

        #[pymethods]
        impl ColliderSet {
            /// Construct an empty set.
            #[new]
            fn new() -> Self {
                Self(rapier::geometry::ColliderSet::new())
            }

            /// Insert a `Collider` or `ColliderBuilder` standalone (no parent).
            ///
            /// :returns: The new `ColliderHandle`.
            /// :raises TypeError: If `builder` is neither a `Collider`
            ///     nor a `ColliderBuilder`.
            fn insert(&mut self, builder: &Bound<'_, PyAny>) -> PyResult<ColliderHandle> {
                if let Ok(b) = builder.extract::<PyRef<'_, ColliderBuilder>>() {
                    return Ok(ColliderHandle(self.0.insert(b.builder.clone().build())));
                }
                if let Ok(c) = builder.extract::<PyRef<'_, Collider>>() {
                    return Ok(ColliderHandle(self.0.insert(c.to_owned_collider())));
                }
                Err(PyTypeError::new_err(
                    "ColliderSet.insert expects a Collider or ColliderBuilder",
                ))
            }

            /// Insert a collider attached to a parent rigid-body.
            ///
            /// The collider's position is interpreted relative to the
            /// parent's pose.
            ///
            /// :param builder: A `Collider` or `ColliderBuilder`.
            /// :param parent: Handle of the parent rigid-body.
            /// :param bodies: The set containing `parent`.
            /// :returns: The new `ColliderHandle`.
            fn insert_with_parent(
                &mut self,
                builder: &Bound<'_, PyAny>,
                parent: &RigidBodyHandle,
                bodies: &mut RigidBodySet,
            ) -> PyResult<ColliderHandle> {
                let coll = if let Ok(b) = builder.extract::<PyRef<'_, ColliderBuilder>>() {
                    b.builder.clone().build()
                } else if let Ok(c) = builder.extract::<PyRef<'_, Collider>>() {
                    c.to_owned_collider()
                } else {
                    return Err(PyTypeError::new_err(
                        "ColliderSet.insert_with_parent expects a Collider or ColliderBuilder",
                    ));
                };
                Ok(ColliderHandle(self.0.insert_with_parent(
                    coll,
                    parent.0,
                    &mut bodies.0,
                )))
            }

            /// Remove a collider by handle and return it.
            ///
            /// :param wake_parent: If True, wakes the parent rigid-body so
            ///     islands re-evaluate. Defaults to True.
            /// :returns: The removed `Collider`, or `None` if `handle` is unknown.
            #[pyo3(signature = (handle, islands, bodies, wake_parent=true))]
            fn remove(
                &mut self,
                handle: &ColliderHandle,
                islands: &mut IslandManager,
                bodies: &mut RigidBodySet,
                wake_parent: bool,
            ) -> Option<Collider> {
                self.0
                    .remove(handle.0, &mut islands.0, &mut bodies.0, wake_parent)
                    .map(Collider::new_owned)
            }

            /// Return a live **view** of the collider for `handle`, or `None`
            /// if the handle is unknown. Reads and writes go straight through
            /// to the set with no copy.
            fn get(slf: &Bound<'_, Self>, handle: &ColliderHandle) -> Option<Collider> {
                slf.borrow().0.get(handle.0)?;
                Some(Collider {
                    backing: ColliderBacking::InSet {
                        set: slf.clone().unbind(),
                        handle: handle.0,
                    },
                })
            }

            fn __getitem__(slf: &Bound<'_, Self>, handle: &ColliderHandle) -> PyResult<Collider> {
                if slf.borrow().0.get(handle.0).is_none() {
                    return Err($crate::errors::InvalidHandle::new_err(format!(
                        "no collider for {:?}",
                        handle.0.into_raw_parts(),
                    )));
                }
                Ok(Collider {
                    backing: ColliderBacking::InSet {
                        set: slf.clone().unbind(),
                        handle: handle.0,
                    },
                })
            }

            fn __contains__(&self, handle: &ColliderHandle) -> bool {
                self.0.contains(handle.0)
            }
            fn __len__(&self) -> usize {
                self.0.len()
            }
            /// True iff the set contains no colliders.
            fn is_empty(&self) -> bool {
                self.0.is_empty()
            }

            /// Remove every collider from the set.
            fn clear(&mut self) {
                self.0 = rapier::geometry::ColliderSet::new();
            }

            fn __iter__(slf: &Bound<'_, Self>) -> PyResult<Py<ColliderSetIter>> {
                let handles: Vec<rapier::geometry::ColliderHandle> =
                    slf.borrow().0.iter().map(|(h, _)| h).collect();
                Py::new(
                    slf.py(),
                    ColliderSetIter {
                        set: slf.clone().unbind(),
                        handles,
                        i: 0,
                    },
                )
            }

            /// Return an iterator over the ``ColliderHandle`` values in the set.
            fn handles(slf: PyRef<'_, Self>) -> PyResult<Py<ColliderHandleIter>> {
                let h: Vec<ColliderHandle> = slf.0.iter().map(|(h, _)| ColliderHandle(h)).collect();
                Py::new(slf.py(), ColliderHandleIter { handles: h, i: 0 })
            }
        }

        /// Iterator yielding `(handle, collider)` pairs from a `ColliderSet`.
        #[pyclass]
        pub struct ColliderSetIter {
            set: Py<ColliderSet>,
            handles: Vec<rapier::geometry::ColliderHandle>,
            i: usize,
        }
        #[pymethods]
        impl ColliderSetIter {
            fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
                slf
            }
            fn __next__(mut slf: PyRefMut<'_, Self>) -> Option<(ColliderHandle, Collider)> {
                if slf.i >= slf.handles.len() {
                    return None;
                }
                let py = slf.py();
                let handle = slf.handles[slf.i];
                slf.i += 1;
                let set = slf.set.clone_ref(py);
                Some((
                    ColliderHandle(handle),
                    Collider {
                        backing: ColliderBacking::InSet { set, handle },
                    },
                ))
            }
        }

        /// Iterator yielding `ColliderHandle`s from a `ColliderSet`.
        #[pyclass]
        pub struct ColliderHandleIter {
            handles: Vec<ColliderHandle>,
            i: usize,
        }
        #[pymethods]
        impl ColliderHandleIter {
            fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
                slf
            }
            fn __next__(mut slf: PyRefMut<'_, Self>) -> Option<ColliderHandle> {
                if slf.i >= slf.handles.len() {
                    return None;
                }
                let h = slf.handles[slf.i];
                slf.i += 1;
                Some(h)
            }
        }

        // ============================================================
        // ColliderParent / ColliderPosition value types
        // ============================================================

        /// Description of a collider's parent rigid-body and local offset.
        ///
        /// Holds the parent `RigidBodyHandle` and the pose of the collider
        /// expressed in the parent's local frame.
        #[pyclass(name = "ColliderParent", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct ColliderParent {
            #[pyo3(get)]
            pub handle: RigidBodyHandle,
            #[pyo3(get)]
            pub pos_wrt_parent: $Iso,
        }

        #[pymethods]
        impl ColliderParent {
            /// Build a `ColliderParent` from a parent handle and local pose.
            #[new]
            fn new(handle: RigidBodyHandle, pos_wrt_parent: PyIsometry) -> Self {
                let iso: $crate::na::Isometry<Real, _, $dim> = pos_wrt_parent.0;
                Self {
                    handle,
                    pos_wrt_parent: $Iso(iso),
                }
            }
        }

        /// Standalone wrapper holding only a collider's world-space pose.
        ///
        /// Used in some lower-level APIs that decouple the position from
        /// the full `Collider` struct.
        #[pyclass(name = "ColliderPosition", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct ColliderPosition {
            #[pyo3(get)]
            pub position: $Iso,
        }

        #[pymethods]
        impl ColliderPosition {
            /// Build a `ColliderPosition` from a world-space pose.
            #[new]
            fn new(position: PyIsometry) -> Self {
                let iso: $crate::na::Isometry<Real, _, $dim> = position.0;
                Self {
                    position: $Iso(iso),
                }
            }
        }

        // ============================================================
        // ColliderMassProps tagged union
        // ============================================================

        /// Tagged union describing how a collider's mass is specified.
        ///
        /// Build one of the three variants (`Density`, `Mass`, or
        /// `MassProperties`) to override the default density-based mass
        /// derivation.
        #[pyclass(name = "ColliderMassProps", module = "rapier")]
        #[derive(Clone)]
        pub struct ColliderMassProps(pub rapier::geometry::ColliderMassProps);

        #[pymethods]
        impl ColliderMassProps {
            /// Build the `Density` variant from a uniform density value.
            #[staticmethod]
            #[pyo3(name = "Density")]
            fn density(d: Real) -> Self {
                Self(rapier::geometry::ColliderMassProps::Density(d))
            }
            /// Build the `Mass` variant from an explicit mass value.
            #[staticmethod]
            #[pyo3(name = "Mass")]
            fn mass(m: Real) -> Self {
                Self(rapier::geometry::ColliderMassProps::Mass(m))
            }
            /// Build the `MassProperties` variant from a full mass-properties value.
            #[staticmethod]
            #[pyo3(name = "MassProperties")]
            fn mass_properties(mp: MassProperties) -> Self {
                Self(rapier::geometry::ColliderMassProps::MassProperties(
                    Box::new(mp.0),
                ))
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __collider_apply_rotation_kwarg {
    (3, $b:expr, $v:expr) => {{
        let pv: PyVector = $v.extract()?;
        $b = $b.clone().rotation(pv.0.into());
    }};
    (2, $b:expr, $v:expr) => {{
        let f: Real = $v.extract()?;
        $b = $b.clone().rotation(f);
    }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_collider_dim_specific {
    (3) => {
        #[pymethods]
        impl Collider {
            /// Builder for a 3D axis-aligned box (cuboid) collider.
            ///
            /// :param hx: Half-extent along the local X axis.
            /// :param hy: Half-extent along the local Y axis.
            /// :param hz: Half-extent along the local Z axis.
            /// :param kwargs: Optional `ColliderBuilder` kwargs.
            #[staticmethod]
            #[pyo3(signature = (hx, hy, hz, **kwargs))]
            fn cuboid(
                hx: Real,
                hy: Real,
                hz: Real,
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<ColliderBuilder> {
                ColliderBuilder::from_kwargs(
                    rapier::geometry::ColliderBuilder::cuboid(hx, hy, hz),
                    kwargs,
                )
            }
            /// Builder for a Y-axis cylinder collider.
            #[staticmethod]
            fn cylinder(half_height: Real, radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::cylinder(half_height, radius),
                }
            }
            /// Builder for a Y-axis cone collider (apex at +Y).
            #[staticmethod]
            fn cone(half_height: Real, radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::cone(half_height, radius),
                }
            }
            /// Builder for a Z-axis capsule collider.
            #[staticmethod]
            fn capsule_z(half_height: Real, radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::capsule_z(half_height, radius),
                }
            }
            /// Builder for a 3D cuboid with rounded edges of `border_radius`.
            #[staticmethod]
            fn round_cuboid(hx: Real, hy: Real, hz: Real, border_radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::round_cuboid(
                        hx,
                        hy,
                        hz,
                        border_radius,
                    ),
                }
            }
            /// Builder for a Y-axis cylinder with rounded edges.
            #[staticmethod]
            fn round_cylinder(
                half_height: Real,
                radius: Real,
                border_radius: Real,
            ) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::round_cylinder(
                        half_height,
                        radius,
                        border_radius,
                    ),
                }
            }
            /// Builder for a Y-axis cone with rounded apex/base.
            #[staticmethod]
            fn round_cone(half_height: Real, radius: Real, border_radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::round_cone(
                        half_height,
                        radius,
                        border_radius,
                    ),
                }
            }
            /// Builder for a convex-polyhedron collider (3D convex hull of `points`).
            #[staticmethod]
            fn convex_polyhedron(points: &Bound<'_, PyAny>) -> PyResult<ColliderBuilder> {
                // 3D: convex_hull builds a ConvexPolyhedron internally; reuse it.
                Self::convex_hull(points)
            }
            /// Builder for a compound collider obtained by convex
            /// decomposition of a triangle mesh.
            #[staticmethod]
            #[pyo3(signature = (vertices, indices))]
            fn convex_decomposition(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
            ) -> PyResult<ColliderBuilder> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                Ok(ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::convex_decomposition(&verts, &idx),
                })
            }
            /// Builder for a convex-mesh collider (vertices assumed convex).
            ///
            /// :raises MeshConversionError: If the mesh is not a valid convex mesh.
            #[staticmethod]
            fn convex_mesh(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
            ) -> PyResult<ColliderBuilder> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                rapier::geometry::ColliderBuilder::convex_mesh(verts, &idx)
                    .map(|b| ColliderBuilder { builder: b })
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "convex mesh construction failed (invalid convex mesh)",
                        )
                    })
            }
            /// Builder for a rounded convex-mesh collider.
            ///
            /// :raises MeshConversionError: If the mesh is not a valid convex mesh.
            #[staticmethod]
            fn round_convex_mesh(
                vertices: &Bound<'_, PyAny>,
                indices: &Bound<'_, PyAny>,
                border_radius: Real,
            ) -> PyResult<ColliderBuilder> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = $crate::geometry::extract_indices(indices)?;
                rapier::geometry::ColliderBuilder::round_convex_mesh(verts, &idx, border_radius)
                    .map(|b| ColliderBuilder { builder: b })
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "round convex mesh construction failed (invalid convex mesh)",
                        )
                    })
            }
            /// Builder for a voxel-grid collider from integer grid coordinates.
            ///
            /// :param voxel_size: Per-axis size of one voxel cell.
            /// :param grid_coords: Sequence of `(i, j, k)` integer cells.
            #[staticmethod]
            fn voxels(
                voxel_size: PyVector,
                grid_coords: &Bound<'_, PyAny>,
            ) -> PyResult<ColliderBuilder> {
                let raw: Vec<(i64, i64, i64)> = grid_coords.extract()?;
                let coords: Vec<rapier::math::IVector> = raw
                    .iter()
                    .map(|&(x, y, z)| rapier::math::IVector::new(x as _, y as _, z as _))
                    .collect();
                Ok(ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::voxels(
                        voxel_size.0.into(),
                        &coords,
                    ),
                })
            }
            /// Builder for a 3D heightfield collider.
            ///
            /// :param heights: `(rows, cols)` ndarray of floats.
            /// :param scale: Per-axis scaling.
            #[staticmethod]
            fn heightfield(
                heights: &Bound<'_, PyAny>,
                scale: PyVector,
            ) -> PyResult<ColliderBuilder> {
                let arr: $crate::numpy::PyReadonlyArray2<Real> = heights.extract()?;
                let slice = arr.as_slice().map_err(|_| {
                    $crate::pyo3::exceptions::PyValueError::new_err(
                        "heights ndarray must be contiguous",
                    )
                })?;
                let nrows = arr.shape()[0];
                let ncols = arr.shape()[1];
                let arr2 = rapier::parry::utils::Array2::new(nrows, ncols, slice.to_vec());
                Ok(ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::heightfield(arr2, scale.0.into()),
                })
            }
        }
    };
    (2) => {
        #[pymethods]
        impl Collider {
            /// Builder for a 2D axis-aligned rectangle (cuboid) collider.
            ///
            /// :param hx: Half-extent along X.
            /// :param hy: Half-extent along Y.
            /// :param kwargs: Optional `ColliderBuilder` kwargs.
            #[staticmethod]
            #[pyo3(signature = (hx, hy, **kwargs))]
            fn cuboid(
                hx: Real,
                hy: Real,
                kwargs: Option<&Bound<'_, $crate::pyo3::types::PyDict>>,
            ) -> PyResult<ColliderBuilder> {
                ColliderBuilder::from_kwargs(
                    rapier::geometry::ColliderBuilder::cuboid(hx, hy),
                    kwargs,
                )
            }
            /// Builder for a 2D segment collider.
            #[staticmethod]
            fn segment(a: PyVector, b: PyVector) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::segment(a.0.into(), b.0.into()),
                }
            }
            /// Builder for a 2D polyline collider.
            ///
            /// :param vertices: `(N, 2)` ndarray of floats.
            /// :param indices: Optional `(M, 2)` segment indices. Defaults
            ///     to consecutive vertices.
            #[staticmethod]
            #[pyo3(signature = (vertices, indices=None))]
            fn polyline(
                vertices: &Bound<'_, PyAny>,
                indices: Option<&Bound<'_, PyAny>>,
            ) -> PyResult<ColliderBuilder> {
                let verts = extract_verts_for_dim(vertices)?;
                let idx = match indices {
                    Some(i) => Some($crate::geometry::extract_indices_2(i)?),
                    None => None,
                };
                Ok(ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::polyline(verts, idx),
                })
            }
            /// Builder for a 2D convex polygon collider.
            #[staticmethod]
            fn convex_polygon(points: &Bound<'_, PyAny>) -> PyResult<ColliderBuilder> {
                let pts = extract_verts_for_dim(points)?;
                rapier::geometry::ColliderBuilder::convex_polyline(pts)
                    .map(|b| ColliderBuilder { builder: b })
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "convex polygon construction failed",
                        )
                    })
            }
            /// Builder for a 2D convex polyline collider (alias of
            /// `convex_polygon`; points assumed convex).
            #[staticmethod]
            fn convex_polyline(points: &Bound<'_, PyAny>) -> PyResult<ColliderBuilder> {
                Self::convex_polygon(points)
            }
            /// Builder for a 2D rounded convex polyline collider.
            ///
            /// :raises MeshConversionError: If the polyline is degenerate.
            #[staticmethod]
            fn round_convex_polyline(
                points: &Bound<'_, PyAny>,
                border_radius: Real,
            ) -> PyResult<ColliderBuilder> {
                let pts = extract_verts_for_dim(points)?;
                rapier::geometry::ColliderBuilder::round_convex_polyline(pts, border_radius)
                    .map(|b| ColliderBuilder { builder: b })
                    .ok_or_else(|| {
                        $crate::errors::MeshConversionError::new_err(
                            "round convex polyline construction failed",
                        )
                    })
            }
            /// Builder for a voxel-grid collider from integer grid coordinates.
            ///
            /// :param voxel_size: Per-axis size of one voxel cell.
            /// :param grid_coords: Sequence of `(i, j)` integer cells.
            #[staticmethod]
            fn voxels(
                voxel_size: PyVector,
                grid_coords: &Bound<'_, PyAny>,
            ) -> PyResult<ColliderBuilder> {
                let raw: Vec<(i64, i64)> = grid_coords.extract()?;
                let coords: Vec<rapier::math::IVector> = raw
                    .iter()
                    .map(|&(x, y)| rapier::math::IVector::new(x as _, y as _))
                    .collect();
                Ok(ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::voxels(
                        voxel_size.0.into(),
                        &coords,
                    ),
                })
            }
            /// Builder for a 2D rectangle with rounded corners.
            #[staticmethod]
            fn round_cuboid(hx: Real, hy: Real, border_radius: Real) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::round_cuboid(hx, hy, border_radius),
                }
            }
            /// Builder for a 2D heightfield collider.
            #[staticmethod]
            fn heightfield(heights: Vec<Real>, scale: PyVector) -> ColliderBuilder {
                ColliderBuilder {
                    builder: rapier::geometry::ColliderBuilder::heightfield(
                        heights,
                        scale.0.into(),
                    ),
                }
            }
        }
    };
}

// ============================================================
// register_geometry function
// ============================================================

#[doc(hidden)]
#[macro_export]
macro_rules! __define_geometry_register {
    (3) => {
        pub fn register_geometry(
            _py: $crate::pyo3::Python<'_>,
            m: &$crate::pyo3::Bound<'_, $crate::pyo3::types::PyModule>,
        ) -> $crate::pyo3::PyResult<()> {
            use $crate::pyo3::prelude::*;
            m.add_class::<ColliderHandle>()?;
            m.add_class::<ColliderType>()?;
            m.add_class::<ColliderEnabled>()?;
            m.add_class::<ShapeType>()?;
            m.add_class::<ActiveEvents>()?;
            m.add_class::<ActiveHooks>()?;
            m.add_class::<ActiveCollisionTypes>()?;
            m.add_class::<Group>()?;
            m.add_class::<InteractionTestMode>()?;
            m.add_class::<InteractionGroups>()?;
            m.add_class::<CollisionEventFlags>()?;
            m.add_class::<TriMeshFlags>()?;
            m.add_class::<ColliderMaterial>()?;
            m.add_class::<ColliderFlags>()?;
            m.add_class::<BvhOptimizationStrategy>()?;
            m.add_class::<BroadPhaseBvh>()?;
            m.add_class::<NarrowPhase>()?;
            m.add_class::<ColliderPair>()?;
            m.add_class::<BroadPhasePairEvent>()?;
            m.add_class::<CollisionEvent>()?;
            m.add_class::<ContactForceEvent>()?;
            // Shape views
            m.add_class::<Ball>()?;
            m.add_class::<Cuboid>()?;
            m.add_class::<Capsule>()?;
            m.add_class::<Triangle>()?;
            m.add_class::<TriMesh>()?;
            m.add_class::<HeightField>()?;
            m.add_class::<Compound>()?;
            m.add_class::<Voxels>()?;
            m.add_class::<Cylinder>()?;
            m.add_class::<Cone>()?;
            m.add_class::<ConvexPolyhedron>()?;
            m.add_class::<SharedShape>()?;
            m.add_class::<Aabb>()?;
            m.add_class::<BoundingSphere>()?;
            m.add_class::<MeshConverter>()?;
            m.add_class::<ContactData>()?;
            m.add_class::<ContactManifoldData>()?;
            m.add_class::<ContactManifold>()?;
            m.add_class::<ContactPair>()?;
            m.add_class::<IntersectionPair>()?;
            m.add_class::<SolverContact>()?;
            m.add_class::<Collider>()?;
            m.add_class::<ColliderBuilder>()?;
            m.add_class::<ColliderSet>()?;
            m.add_class::<ColliderSetIter>()?;
            m.add_class::<ColliderHandleIter>()?;
            m.add_class::<ColliderParent>()?;
            m.add_class::<ColliderPosition>()?;
            m.add_class::<ColliderMassProps>()?;
            Ok(())
        }
    };
    (2) => {
        pub fn register_geometry(
            _py: $crate::pyo3::Python<'_>,
            m: &$crate::pyo3::Bound<'_, $crate::pyo3::types::PyModule>,
        ) -> $crate::pyo3::PyResult<()> {
            use $crate::pyo3::prelude::*;
            m.add_class::<ColliderHandle>()?;
            m.add_class::<ColliderType>()?;
            m.add_class::<ColliderEnabled>()?;
            m.add_class::<ShapeType>()?;
            m.add_class::<ActiveEvents>()?;
            m.add_class::<ActiveHooks>()?;
            m.add_class::<ActiveCollisionTypes>()?;
            m.add_class::<Group>()?;
            m.add_class::<InteractionTestMode>()?;
            m.add_class::<InteractionGroups>()?;
            m.add_class::<CollisionEventFlags>()?;
            m.add_class::<TriMeshFlags>()?;
            m.add_class::<ColliderMaterial>()?;
            m.add_class::<ColliderFlags>()?;
            m.add_class::<BvhOptimizationStrategy>()?;
            m.add_class::<BroadPhaseBvh>()?;
            m.add_class::<NarrowPhase>()?;
            m.add_class::<ColliderPair>()?;
            m.add_class::<BroadPhasePairEvent>()?;
            m.add_class::<CollisionEvent>()?;
            m.add_class::<ContactForceEvent>()?;
            // Shape views (2D)
            m.add_class::<Ball>()?;
            m.add_class::<Cuboid>()?;
            m.add_class::<Capsule>()?;
            m.add_class::<Triangle>()?;
            m.add_class::<TriMesh>()?;
            m.add_class::<HeightField>()?;
            m.add_class::<Compound>()?;
            m.add_class::<Voxels>()?;
            m.add_class::<Segment>()?;
            m.add_class::<Polyline>()?;
            m.add_class::<ConvexPolygon>()?;
            m.add_class::<SharedShape>()?;
            m.add_class::<Aabb>()?;
            m.add_class::<BoundingSphere>()?;
            m.add_class::<MeshConverter>()?;
            m.add_class::<ContactData>()?;
            m.add_class::<ContactManifoldData>()?;
            m.add_class::<ContactManifold>()?;
            m.add_class::<ContactPair>()?;
            m.add_class::<IntersectionPair>()?;
            m.add_class::<SolverContact>()?;
            m.add_class::<Collider>()?;
            m.add_class::<ColliderBuilder>()?;
            m.add_class::<ColliderSet>()?;
            m.add_class::<ColliderSetIter>()?;
            m.add_class::<ColliderHandleIter>()?;
            m.add_class::<ColliderParent>()?;
            m.add_class::<ColliderPosition>()?;
            m.add_class::<ColliderMassProps>()?;
            Ok(())
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __shape_type_from_rapier {
    (3, $t:expr) => {{
        use rapier::parry::shape::ShapeType as PT;
        match $t {
            PT::Ball => ShapeType::BALL,
            PT::Cuboid => ShapeType::CUBOID,
            PT::Capsule => ShapeType::CAPSULE,
            PT::Segment => ShapeType::SEGMENT,
            PT::Triangle => ShapeType::TRIANGLE,
            PT::Voxels => ShapeType::VOXELS,
            PT::TriMesh => ShapeType::TRIMESH,
            PT::Polyline => ShapeType::POLYLINE,
            PT::HalfSpace => ShapeType::HALFSPACE,
            PT::HeightField => ShapeType::HEIGHTFIELD,
            PT::Compound => ShapeType::COMPOUND,
            PT::ConvexPolyhedron => ShapeType::CONVEX_POLYHEDRON,
            PT::Cylinder => ShapeType::CYLINDER,
            PT::Cone => ShapeType::CONE,
            PT::RoundCuboid => ShapeType::ROUND_CUBOID,
            PT::RoundTriangle => ShapeType::ROUND_TRIANGLE,
            PT::RoundCylinder => ShapeType::ROUND_CYLINDER,
            PT::RoundCone => ShapeType::ROUND_CONE,
            PT::RoundConvexPolyhedron => ShapeType::ROUND_CONVEX_POLYHEDRON,
            _ => ShapeType::CUSTOM,
        }
    }};
    (2, $t:expr) => {{
        use rapier::parry::shape::ShapeType as PT;
        match $t {
            PT::Ball => ShapeType::BALL,
            PT::Cuboid => ShapeType::CUBOID,
            PT::Capsule => ShapeType::CAPSULE,
            PT::Segment => ShapeType::SEGMENT,
            PT::Triangle => ShapeType::TRIANGLE,
            PT::Voxels => ShapeType::VOXELS,
            PT::TriMesh => ShapeType::TRIMESH,
            PT::Polyline => ShapeType::POLYLINE,
            PT::HalfSpace => ShapeType::HALFSPACE,
            PT::HeightField => ShapeType::HEIGHTFIELD,
            PT::Compound => ShapeType::COMPOUND,
            PT::ConvexPolygon => ShapeType::CONVEX_POLYGON,
            PT::RoundCuboid => ShapeType::ROUND_CUBOID,
            PT::RoundTriangle => ShapeType::ROUND_TRIANGLE,
            PT::RoundConvexPolygon => ShapeType::ROUND_CONVEX_POLYGON,
            _ => ShapeType::CUSTOM,
        }
    }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_geometry_contact_point_to_py {
    (3, $c:ident) => {{
        let p1: $crate::na::Vector3<Real> = $c.local_p1.into();
        let p2: $crate::na::Vector3<Real> = $c.local_p2.into();
        ContactData {
            local_p1: Point3($crate::na::Point3::from(p1)),
            local_p2: Point3($crate::na::Point3::from(p2)),
            dist: $c.dist,
            fid1: $c.fid1.0,
            fid2: $c.fid2.0,
            impulse: $c.data.impulse,
            tangent_impulse: ($c.data.tangent_impulse[0], $c.data.tangent_impulse[1]),
            contact_id: 0,
        }
    }};
    (2, $c:ident) => {{
        let p1: $crate::na::Vector2<Real> = $c.local_p1.into();
        let p2: $crate::na::Vector2<Real> = $c.local_p2.into();
        ContactData {
            local_p1: Point2($crate::na::Point2::from(p1)),
            local_p2: Point2($crate::na::Point2::from(p2)),
            dist: $c.dist,
            fid1: $c.fid1.0,
            fid2: $c.fid2.0,
            impulse: $c.data.impulse,
            tangent_impulse: $c.data.tangent_impulse[0],
            contact_id: 0,
        }
    }};
}
