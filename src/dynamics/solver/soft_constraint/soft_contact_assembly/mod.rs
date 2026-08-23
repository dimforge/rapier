//! Assembly of the soft contact constraints: the narrow-phase pairs of the awake soft bodies'
//! surface colliders and particle balls, the vertex-vs-surface and edge-vs-edge candidates between
//! surfaces (see `geometry::soft_contacts`), and the write-back of their impulses.

mod soft_contact_assembly;
mod soft_contact_assembly_body_contacts;
mod soft_contact_assembly_edge_contacts;
mod soft_contact_assembly_vertex_contacts;
mod soft_contact_assembly_volume_constraints;
mod soft_contact_assembly_workspace;
mod soft_contact_assembly_writeback;

pub(crate) use soft_contact_assembly_workspace::EdgeContactWorkspace;
use soft_contact_assembly_workspace::{
    AssemblyCtx, BodyContacts, MeshContacts, OverlapConstraintWorkspace, SOURCE_EDGE_CONTACT,
    SOURCE_VERTEX_CONTACT, material_pace, mesh_of, mesh_ref,
};
