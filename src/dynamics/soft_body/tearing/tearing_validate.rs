//! Consistency checks of a soft body topology and its derived tables after a tear.

use crate::alloc_prelude::*;
#[cfg(feature = "dim3")]
use crate::math::DIM;
use alloc::format;
use parry::utils::hashmap::HashMap;

use super::super::SoftBody;
use super::super::soft_body::SOFT_BODY_OVERFLOW_COLOR;
use super::super::soft_body_builder::element_vertices;
#[cfg(feature = "dim3")]
use super::tearing::pair;

impl SoftBody {
    /// Checks the topology and its derived tables (element indices, every particle in a measure
    /// element (see [`Self::rest_measure`]), surface rings and incidence, owning cells, the 3D edge
    /// table, the coloring) for consistency; returns the first inconsistency found.
    pub fn validate_topology(&self) -> Result<(), String> {
        let n = self.particles.len() as u32;
        let check = |what: &str, vertices: &[u32]| -> Result<(), String> {
            for &v in vertices {
                if v >= n {
                    return Err(format!("{what} references particle {v} of {n}"));
                }
            }
            Ok(())
        };
        for (i, e) in self.edges.iter().enumerate() {
            check(&format!("edge {i}"), &e.vertices)?;
            if e.vertices[0] == e.vertices[1] {
                return Err(format!("edge {i} is degenerate"));
            }
        }
        for (i, c) in self.cells.iter().enumerate() {
            check(&format!("cell {i}"), &c.vertices)?;
        }
        #[cfg(feature = "dim3")]
        for (i, d) in self.dihedrals.iter().enumerate() {
            check(&format!("dihedral {i}"), &d.vertices)?;
        }
        for (i, s) in self.boundary.iter().enumerate() {
            check(&format!("surface element {i}"), s)?;
        }
        // Every particle belongs to some measure element (a tear never removes one), unless the
        // body has none.
        if let Some(kind) = self.measure_kind() {
            let mut supported = vec![false; n as usize];
            self.for_each_measure_element(kind, |_, vertices| {
                for &v in vertices {
                    if let Some(s) = supported.get_mut(v as usize) {
                        *s = true;
                    }
                }
            });
            if let Some(v) = supported.iter().position(|s| !s) {
                return Err(format!("particle {v} belongs to no measure element"));
            }
        }
        for (mi, mesh) in self.meshes().enumerate() {
            let nv = mesh.vertex_count();
            if let super::collision_mesh::SoftMeshMapping::Skinned { bindings } = mesh.binding() {
                if bindings.len() != nv {
                    return Err(format!(
                        "mesh {mi} has {} bindings for {nv} vertices",
                        bindings.len()
                    ));
                }
                // A body left without any cell has nothing to bind to.
                let dead = |b: &super::collision_mesh::SoftMeshCellBinding| {
                    !self.cells.is_empty() && b.cell as usize >= self.cells.len()
                };
                if let Some(v) = bindings.iter().position(dead) {
                    return Err(format!("mesh {mi} vertex {v} is bound to a dead cell"));
                }
            }
            for (i, s) in mesh.indices().iter().enumerate() {
                for &v in element_vertices(s) {
                    if v as usize >= nv {
                        return Err(format!("mesh {mi} element {i} has an out-of-range vertex"));
                    }
                }
            }
            if mesh.ring_offsets.len() != nv + 1 {
                return Err(format!("mesh {mi} ring offsets do not cover every vertex"));
            }
            if mesh.vertex_elements_offsets.len() != nv + 1 {
                return Err(format!(
                    "mesh {mi} vertex-element offsets do not cover every vertex"
                ));
            }
            for (i, s) in mesh.indices().iter().enumerate() {
                let s = element_vertices(s);
                for &v in s {
                    let (start, end) = (
                        mesh.vertex_elements_offsets[v as usize] as usize,
                        mesh.vertex_elements_offsets[v as usize + 1] as usize,
                    );
                    if !mesh.vertex_elements[start..end].contains(&(i as u32)) {
                        return Err(format!(
                            "mesh {mi} element {i} missing from vertex {v}'s list"
                        ));
                    }
                    let (start, end) = (
                        mesh.ring_offsets[v as usize] as usize,
                        mesh.ring_offsets[v as usize + 1] as usize,
                    );
                    let ring = &mesh.ring[start..end];
                    for &w in s.iter().filter(|&&w| w != v) {
                        if ring.binary_search(&w).is_err() {
                            return Err(format!("mesh {mi}: vertex {w} missing from {v}'s ring"));
                        }
                    }
                }
            }
            for (v, window) in mesh.vertex_elements_offsets.windows(2).enumerate() {
                for &e in &mesh.vertex_elements[window[0] as usize..window[1] as usize] {
                    let ok = mesh
                        .indices()
                        .get(e as usize)
                        .is_some_and(|s| s.contains(&(v as u32)));
                    if !ok {
                        return Err(format!(
                            "mesh {mi}: vertex {v} lists element {e} it is not in"
                        ));
                    }
                }
            }
            #[cfg(feature = "dim3")]
            {
                if mesh.element_edges.len() != mesh.indices().len()
                    || mesh.edge_owners.len() != mesh.edges.len()
                {
                    return Err(format!("mesh {mi} edge tables do not cover its elements"));
                }
                for (i, s) in mesh.indices().iter().enumerate() {
                    // A wire segment (padded with `u32::MAX`) is its own single edge.
                    if element_vertices(s).len() < DIM {
                        let id = mesh.element_edges[i][0] as usize;
                        if mesh.edges.get(id) != Some(&pair(s[0], s[1])) {
                            return Err(format!("mesh {mi} segment {i} edge mismatch"));
                        }
                        continue;
                    }
                    for k in 0..DIM {
                        let id = mesh.element_edges[i][k] as usize;
                        let expected = pair(s[(k + 1) % DIM], s[(k + 2) % DIM]);
                        if mesh.edges.get(id) != Some(&expected) {
                            return Err(format!("mesh {mi} element {i} edge {k} mismatch"));
                        }
                    }
                }
                for (id, &owner) in mesh.edge_owners.iter().enumerate() {
                    let ok = mesh
                        .element_edges
                        .get(owner as usize)
                        .is_some_and(|edges| edges.contains(&(id as u32)));
                    if !ok {
                        return Err(format!("mesh {mi} edge {id} owner mismatch"));
                    }
                }
            }
        }
        if !self.cells.is_empty() {
            if self.boundary_element_cells.len() != self.boundary.len() {
                return Err("boundary element owners do not cover the boundary".into());
            }
            for (i, &c) in self.boundary_element_cells.iter().enumerate() {
                if c == u32::MAX {
                    continue;
                }
                let ok = self
                    .cells
                    .get(c as usize)
                    .is_some_and(|cell| self.boundary[i].iter().all(|v| cell.vertices.contains(v)));
                if !ok {
                    return Err(format!(
                        "boundary element {i} owned by a cell not containing it"
                    ));
                }
            }
        }
        // Coloring: elements of one parallel color share no particle.
        let mut seen: HashMap<(u8, u32), ()> = HashMap::default();
        let mut check_color = |color: u8, vertices: &[u32]| -> Result<(), String> {
            if color == SOFT_BODY_OVERFLOW_COLOR {
                return Ok(());
            }
            if color >= self.num_colors {
                return Err(format!(
                    "color {color} beyond num_colors {}",
                    self.num_colors
                ));
            }
            for &v in vertices {
                if seen.insert((color, v), ()).is_some() {
                    return Err(format!(
                        "particle {v} shared by two elements of color {color}"
                    ));
                }
            }
            Ok(())
        };
        for e in &self.edges {
            check_color(e.color, &e.vertices)?;
        }
        for c in &self.cells {
            check_color(c.color, &c.vertices)?;
        }
        #[cfg(feature = "dim3")]
        for d in &self.dihedrals {
            check_color(d.color, &d.vertices)?;
        }
        Ok(())
    }
}
