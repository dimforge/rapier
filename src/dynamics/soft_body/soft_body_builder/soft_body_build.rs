//! The build path of the soft-body builder: the soft body itself and its collision meshes.

use crate::alloc_prelude::*;
use crate::dynamics::RigidBodyHandle;
use crate::math::{DIM, Matrix, Real, Rotation, Vector};
use parry::utils::hashmap::HashMap;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

#[cfg(feature = "dim3")]
use super::super::SoftBodyDihedral;
use super::super::{
    SoftBody, SoftBodyCell, SoftBodyCellModel, SoftBodyEdge, SoftBodyEdgeKind, SoftBodyParticle,
    SoftCollisionMesh, soft_body_coloring,
};
use super::SoftBodyBuilder;
#[cfg(feature = "dim3")]
use super::soft_body_builder_mesh_helpers::dihedral_angle;
use super::soft_body_builder_mesh_helpers::{
    cell_boundary, surface_element_cells, surface_is_closed,
};

impl SoftBodyBuilder {
    /*
     * Build.
     */

    /// Builds the soft body (not yet inserted in a set: its root rigid body and colliders are
    /// created by [`super::SoftBodySet::insert`]).
    pub(crate) fn build(&self) -> SoftBody {
        let num_particles = self.positions.len();
        let masses: Vec<Real> = if self.masses.len() == num_particles {
            self.masses.clone()
        } else {
            vec![self.particle_mass; num_particles]
        };
        let mut pinned = vec![false; num_particles];
        for &i in &self.pinned {
            if let Some(p) = pinned.get_mut(i as usize) {
                *p = true;
            }
        }

        // Rest center of mass and rest positions.
        let mut rest_com = Vector::ZERO;
        let mut total_mass = 0.0;
        for (p, m) in self.positions.iter().zip(masses.iter()) {
            rest_com += *p * *m;
            total_mass += *m;
        }
        if total_mass > 0.0 {
            rest_com /= total_mass;
        }

        let particles: Vec<SoftBodyParticle> = self
            .positions
            .iter()
            .zip(masses.iter())
            .zip(pinned.iter())
            .enumerate()
            .map(|(i, ((p, m), pinned))| SoftBodyParticle {
                position: *p,
                velocity: Vector::ZERO,
                rest_position: *p - rest_com,
                initial_rest_position: *p - rest_com,
                mass: *m,
                inv_mass: if *pinned { 0.0 } else { crate::utils::inv(*m) },
                force: Vector::ZERO,
                next_position: None,
                damaged: false,
                on_surface: false,
                split_root: i as u32,
            })
            .collect();
        let pos = |i: u32| self.positions[i as usize];

        // Cells: positive orientation, rest matrices.
        let mut cells = Vec::with_capacity(self.cells.len());
        for c in &self.cells {
            let mut vertices = *c;
            let mut x: [Vector; DIM + 1] = core::array::from_fn(|k| pos(vertices[k]));
            if SoftBody::cell_volume(x) < 0.0 {
                vertices.swap(1, 2);
                x = core::array::from_fn(|k| pos(vertices[k]));
            }
            let rest_volume = SoftBody::cell_volume(x);
            let edge_matrix = SoftBody::cell_edge_matrix(x);
            let inv_rest_matrix = if rest_volume.abs() > Real::EPSILON {
                edge_matrix.inverse()
            } else {
                Matrix::ZERO
            };
            cells.push(SoftBodyCell {
                vertices,
                rest_volume,
                inv_rest_matrix,
                plastic_stretch: Matrix::IDENTITY,
                impulses: [0.0; super::super::CELL_IMPULSES],
                rotation: Rotation::IDENTITY,
                stiffness_scale: 1.0,
                tear_resistance: 1.0,
                color: 0,
                torn: false,
                stress: 0.0,
            });
        }

        // Surface: given, or the boundary of the cells.
        let surface = if !self.surface.is_empty() {
            self.surface.clone()
        } else {
            cell_boundary(&cells)
        };

        // Edges: given, or the cell edges for the volume model (the corotational cells provide the
        // whole material response: springs on top would add a stiffness floor of their own,
        // independent of the Young modulus, and hold the original shape against plastic flow).
        let mut edge_list: Vec<([u32; 2], SoftBodyEdgeKind)> = Vec::new();
        if self.edges.is_empty()
            && !cells.is_empty()
            && self.cell_model == SoftBodyCellModel::Volume
        {
            let mut seen: HashMap<[u32; 2], ()> = HashMap::default();
            for c in &cells {
                for a in 0..DIM + 1 {
                    for b in a + 1..DIM + 1 {
                        let key = [
                            c.vertices[a].min(c.vertices[b]),
                            c.vertices[a].max(c.vertices[b]),
                        ];
                        if seen.insert(key, ()).is_none() {
                            edge_list.push((key, SoftBodyEdgeKind::Structural));
                        }
                    }
                }
            }
        } else {
            edge_list.extend(
                self.edges
                    .iter()
                    .map(|e| (*e, SoftBodyEdgeKind::Structural)),
            );
        }
        edge_list.extend(self.bend_edges.iter().map(|e| (*e, SoftBodyEdgeKind::Bend)));
        let mut edges: Vec<SoftBodyEdge> = edge_list
            .iter()
            .map(|(vertices, kind)| SoftBodyEdge {
                vertices: *vertices,
                rest_length: (pos(vertices[0]) - pos(vertices[1])).length(),
                kind: *kind,
                tension_only: false,
                softness: None,
                tear_resistance: 1.0,
                impulse: 0.0,
                plastic_strain: 0.0,
                color: 0,
                torn: false,
                stress: 0.0,
            })
            .collect();
        for &i in &self.tension_only_edges {
            if let Some(e) = edges.get_mut(i as usize) {
                e.tension_only = true;
            }
        }
        for &(i, resistance) in &self.edge_tear_resistance {
            if let Some(e) = edges.get_mut(i as usize) {
                e.tear_resistance = resistance.max(0.0);
            }
        }
        for &(i, softness) in &self.edge_softness {
            if let Some(e) = edges.get_mut(i as usize) {
                e.softness = Some(softness);
            }
        }

        #[cfg(feature = "dim3")]
        let mut dihedrals: Vec<SoftBodyDihedral> = self
            .dihedrals
            .iter()
            .map(|v| SoftBodyDihedral {
                vertices: *v,
                rest_angle: dihedral_angle(pos(v[0]), pos(v[1]), pos(v[2]), pos(v[3])),
                plastic_set: 0.0,
                impulse: 0.0,
                color: 0,
            })
            .collect();

        let (num_colors, has_overflow_color) = soft_body_coloring::assign_colors(
            num_particles,
            &mut edges,
            &mut cells,
            #[cfg(feature = "dim3")]
            &mut dihedrals,
        );

        let boundary_closed = surface_is_closed(&surface);
        let boundary_element_cells = surface_element_cells(&surface, &cells);
        let mut particles = particles;
        for element in &surface {
            for &v in element {
                if let Some(p) = particles.get_mut(v as usize) {
                    p.on_surface = true;
                }
            }
        }

        let mut body = SoftBody {
            particles,
            clusters: vec![],
            cluster_refs: Vec::new(),
            root_body: RigidBodyHandle::invalid(),
            attachments: Vec::new(),
            sleeping: false,
            enabled: true,
            positions_modified: false,
            attachments_modified: false,
            edges,
            #[cfg(feature = "dim3")]
            dihedrals,
            cells,
            volume_preservation: self.volume_preservation,
            boundary: surface,
            boundary_closed,
            boundary_element_cells,
            material: self.material,
            cell_model: self.cell_model,
            #[cfg(feature = "fem")]
            solver: self.solver,
            volume_pieces: Vec::new(),
            volume_factor: self.volume_factor,
            rest_com,
            particle_radius: self.particle_radius,
            particle_settings: self.particle_settings,
            num_colors,
            has_overflow_color,
            modified: false,
            plastic_flowing: false,
            rest_fit_pending: false,
            sleep_speed: 0.0,
            tearing_pending: false,
            topology_version: 0,
            contact_approach_speeds: [None; 2],
            contact_approach_step_open: false,
            contact_load: 0.0,
            load_extra_substeps: 0,
            origin: None,
            pieces: Vec::new(),
            user_data: self.user_data,
        };
        // From the builder's positions: the particles' rest positions are centered on `rest_com`.
        body.volume_pieces = body.compute_volume_pieces(&[], pos);
        body.volume_preservation &= !body.volume_pieces.is_empty();
        body
    }
}

impl SoftBodyBuilder {
    /// The meshes of `body`, built at insertion once it has its whole-body cluster: its skin,
    /// bound to the cells at the rest positions, and its boundary, kept only when the body collides
    /// through it (a body with a collision skin collides through the skin alone).
    pub(crate) fn build_meshes(&self, body: &SoftBody) -> Vec<SoftCollisionMesh> {
        let has_collider = self.collider_template.is_some();
        let skin = self.skin.as_ref().and_then(|(vertices, indices)| {
            SoftCollisionMesh::skinned(
                vertices,
                indices,
                &body.cells,
                &self.positions,
                self.self_contacts,
                has_collider && self.skin_collision,
            )
        });
        let skin_collides = skin.as_ref().is_some_and(|mesh| mesh.collision_enabled());
        let mut meshes = Vec::new();
        if let Some(skin) = skin {
            meshes.push(skin);
        }
        // A wire replaces the boundary mesh: a body made of segments has no surface.
        #[cfg(feature = "dim3")]
        let wire = !skin_collides && !self.wire.is_empty();
        #[cfg(feature = "dim2")]
        let wire = false;
        #[cfg(feature = "dim3")]
        if wire {
            meshes.push(SoftCollisionMesh::wire(
                body,
                &self.wire,
                self.self_contacts,
                has_collider,
            ));
        }
        if !skin_collides && !wire {
            meshes.push(SoftCollisionMesh::boundary(
                body,
                self.self_contacts,
                has_collider,
            ));
        }
        for mesh in &mut meshes {
            mesh.oriented = self.oriented.unwrap_or(mesh.closed);
        }
        meshes
    }
}
