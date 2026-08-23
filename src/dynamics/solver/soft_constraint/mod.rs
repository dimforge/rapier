//! Soft-body constraints solved by the staged island solver alongside joints and contacts.

pub(crate) use self::soft_constraints_set::SoftConstraintsSet;

mod soft_attachment;
mod soft_constraints_set;
mod soft_contact;
mod soft_contact_assembly;
mod soft_contact_chunks;
mod soft_element_constraint_assembly;
