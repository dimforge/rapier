//! The mesh a soft body meets the world through, as opposed to the computational mesh
//! ([`SoftBody`]'s particles, edges and cells) it is simulated on. It maps onto that mesh
//! ([`SoftMeshMapping`]) directly (vertex `i` is a particle) or by skinning (riding a cell).

mod collision_mesh;
mod collision_mesh_accessors;
mod collision_mesh_binding;
mod collision_mesh_geometry;
mod collision_mesh_topology;

pub use self::collision_mesh_binding::{SoftBindingError, SoftMeshBinding, SoftMeshBindingMode};
pub(crate) use self::collision_mesh_binding::bind_to_cells;
pub use self::collision_mesh::{
    SoftCollisionMesh, SoftMeshCellBinding, SoftMeshId, SoftMeshMapping, SoftMeshRef,
};
pub(crate) use self::collision_mesh::SoftTopologyRemap;
pub(super) use super::SoftBodyParticle;
pub(super) use super::soft_body_builder;
pub(super) use super::soft_body_builder::{
    surface_element_cells, surface_is_closed, surface_rings, surface_vertex_elements,
};
pub(super) use super::{SoftBody, SoftBodyCell};
