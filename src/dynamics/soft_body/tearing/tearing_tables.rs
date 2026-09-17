//! The boundary tables and the parallel coloring rebuilt after a topology change.

use super::super::soft_body_builder::{surface_element_cells, surface_is_closed};
use super::super::{SoftBody, soft_body_coloring};

impl SoftBody {
    /// Rebuilds the tables derived from the boundary and the cells (closedness, owning cells,
    /// volume pieces), and the collision meshes' own tables.
    pub(crate) fn rebuild_boundary_tables(&mut self) {
        self.boundary_closed = surface_is_closed(&self.boundary);
        self.boundary_element_cells = surface_element_cells(&self.boundary, &self.cells);
        self.update_surface_flags();
        self.rebuild_mesh_tables();
        self.rebuild_volume_pieces(&[]);
    }

    /// Recomputes the parallel colors of every element.
    pub(crate) fn recolor(&mut self) {
        let (num_colors, has_overflow) = soft_body_coloring::assign_colors(
            self.particles.len(),
            &mut self.edges,
            &mut self.cells,
            #[cfg(feature = "dim3")]
            &mut self.dihedrals,
        );
        self.num_colors = num_colors;
        self.has_overflow_color = has_overflow;
    }
}
