//! Soft-body contact detection: the geometric passes the narrow phase runs on every updated pair
//! with a deformable collider (vertex-vs-surface and edge-vs-edge candidates, boundary crossings)
//! and per soft mesh for self contacts; results: [`SoftPairContacts`], [`SoftSelfContacts`].

mod soft_contacts_classify;
mod soft_contacts_driver;
mod soft_contacts_edge_pass;
mod soft_contacts_pairs;
mod soft_contacts_types;
mod soft_contacts_vertex_pass;
mod soft_contacts_volume;
mod soft_self_contacts;

pub(crate) use soft_contacts_edge_pass::detect_edges;
pub(crate) use soft_contacts_pairs::{update_pair_soft_rigid, update_pair_soft_soft};
pub(crate) use soft_self_contacts::SoftSelfContacts;
pub use soft_contacts_types::{
    SoftContactImpulse, SoftEdgeCandidate, SoftEdgePass, SoftPairContacts, SoftRigidPatch,
    SoftVertexCandidate, SoftVertexHits, SoftVertexPass, SoftVolumePatch,
};
pub(crate) use soft_contacts_types::{SelfTangles, SoftDetectionCtx, SoftRigidContacts, body_frozen};
use soft_contacts_types::Side;
pub(crate) use soft_contacts_vertex_pass::{detect_vertex_pass, element_plane, elements_cross};
pub use soft_contacts_volume::VolumeBin;
