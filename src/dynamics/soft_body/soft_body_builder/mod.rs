//! The soft-body builder: its settings struct, the builder struct itself and the module wiring.

mod soft_body_build;
mod soft_body_builder;
mod soft_body_builder_mesh_helpers;
mod soft_body_builder_settings;
mod soft_body_builder_shapes;

pub use self::soft_body_builder::{SoftBodyBuilder, SoftBodyParticleSettings};
#[cfg(feature = "dim3")]
pub(crate) use self::soft_body_builder_mesh_helpers::surface_edge_table;
pub(crate) use self::soft_body_builder_mesh_helpers::{
    cell_faces, element_vertices, element_vertices_mut, surface_element_cells, surface_is_closed,
    surface_rings, surface_vertex_elements,
};
#[cfg(feature = "fem")]
pub(super) use super::SoftBodySolver;
