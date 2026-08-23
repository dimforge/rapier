//! The soft-body constraints of one step over the awake soft bodies (element, shape-matching,
//! volume-piece and contact constraints) with their per-pass prepare, solve and writeback; the
//! assembly lives in `soft_element_constraint_assembly` and `soft_contact_assembly`.

mod soft_constraint_plasticity;
mod soft_constraint_prepare;
mod soft_constraint_rigid_coupling;
mod soft_constraint_solve;
mod soft_constraint_writeback;
mod soft_constraints;
mod soft_constraints_set;

use super::{soft_attachment, soft_contact_assembly, soft_contact_chunks, soft_element_constraint_assembly};

#[cfg(feature = "fem")]
pub(crate) use self::soft_constraint_prepare::soft_fem_amplification_cap;
pub(crate) use self::soft_constraint_plasticity::*;
pub(crate) use self::soft_constraint_rigid_coupling::*;
pub(crate) use self::soft_constraints::*;
pub(crate) use self::soft_constraints_set::*;
use self::soft_constraint_writeback::*;
