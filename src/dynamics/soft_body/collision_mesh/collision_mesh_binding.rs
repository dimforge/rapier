//! Binding a collision mesh to a soft body's cells: the requested binding modes, the errors, and
//! the barycentric bindings themselves.

use crate::alloc_prelude::*;
use crate::math::{DIM, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::{SoftBody, SoftBodyCell, SoftMeshCellBinding};

/// The barycentric coordinates of `point` in the cell `[a, b, c(, d)]`, extrapolating outside it.
///
/// `None` for a degenerate cell, which has no frame to express the point in.
fn barycentric(point: Vector, cell: [Vector; DIM + 1]) -> Option<[Real; DIM + 1]> {
    let frame = SoftBody::cell_edge_matrix(cell);
    if SoftBody::cell_volume(cell).abs() <= Real::EPSILON {
        return None;
    }

    let coords = frame.inverse() * (point - cell[0]);
    let mut weights = [0.0; DIM + 1];
    weights[0] = 1.0;

    for k in 0..DIM {
        weights[k + 1] = coords[k];
        weights[0] -= coords[k];
    }

    Some(weights)
}

/// The squared distance from `point` to the cell, zero inside it.
fn distance_squared_to_cell(point: Vector, cell: [Vector; DIM + 1]) -> Real {
    use parry::query::PointQuery;

    #[cfg(feature = "dim2")]
    {
        let triangle = parry::shape::Triangle::new(cell[0], cell[1], cell[2]);
        (triangle.project_local_point(point, true).point - point).length_squared()
    }
    #[cfg(feature = "dim3")]
    {
        // The distance to the closest of the four faces, zero when the point is behind all of
        // them.
        // TODO: refactor this into `Tetrahedron`.
        let faces = [
            [cell[0], cell[2], cell[1]],
            [cell[0], cell[1], cell[3]],
            [cell[0], cell[3], cell[2]],
            [cell[1], cell[2], cell[3]],
        ];
        let mut inside = true;
        let mut closest = Real::MAX;

        for face in faces {
            let triangle = parry::shape::Triangle::new(face[0], face[1], face[2]);
            let normal = (face[1] - face[0]).cross(face[2] - face[0]);
            inside &= (point - face[0]).dot(normal) <= 0.0;
            closest = closest
                .min((triangle.project_local_point(point, true).point - point).length_squared());
        }

        if inside { 0.0 } else { closest }
    }
}

/// Binds a mesh to a soft body's cells: for every vertex, the closest cell and the vertex's
/// barycentric coordinates in it.
pub(crate) fn bind_to_cells(
    vertices: &[Vector],
    indices: &[[u32; DIM]],
    cells: &[SoftBodyCell],
    positions: &[Vector],
) -> Option<Vec<SoftMeshCellBinding>> {
    use parry::bounding_volume::Aabb;
    use parry::partitioning::{Bvh, BvhBuildStrategy};
    use parry::query::PointQuery;

    if vertices.is_empty() || indices.is_empty() || cells.is_empty() {
        return None;
    }

    let cell_points = |cell: &SoftBodyCell| cell.vertices.map(|vid| positions[vid as usize]);
    let leaves: Vec<Aabb> = cells
        .iter()
        .map(|cell| Aabb::from_points(cell_points(cell).iter().copied()))
        .collect();
    let bvh = Bvh::from_leaves(BvhBuildStrategy::Binned, &leaves);

    let mut bindings = Vec::with_capacity(vertices.len());

    for vertex in vertices {
        // The closest cell: exact at the leaves, bounded by the node Aabbs on the way down.
        let closest = bvh.find_best(
            Real::MAX,
            |node, _| node.aabb().distance_to_local_point(*vertex, true),
            |cell, _| {
                let points = cell_points(&cells[cell as usize]);
                Some(distance_squared_to_cell(*vertex, points).sqrt())
            },
        );

        let (cell, _) = closest?;
        let binding = match barycentric(*vertex, cell_points(&cells[cell as usize])) {
            Some(weights) => SoftMeshCellBinding { cell, weights },
            // The closest cell can be a sliver too flat to invert: the closest cell that is not
            // holds the vertex instead (this checks all the cells so it's slow, bet it should
            // almost never happen unless the input mesh is malformed).
            None => {
                let mut by_distance: Vec<(Real, u32)> = cells
                    .iter()
                    .enumerate()
                    .map(|(id, cell)| {
                        (
                            distance_squared_to_cell(*vertex, cell_points(cell)),
                            id as u32,
                        )
                    })
                    .collect();
                by_distance.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
                by_distance.iter().find_map(|(_, cell)| {
                    let weights = barycentric(*vertex, cell_points(&cells[*cell as usize]))?;
                    Some(SoftMeshCellBinding {
                        cell: *cell,
                        weights,
                    })
                })?
            }
        };
        bindings.push(binding);
    }

    Some(bindings)
}

/// How a collision mesh should be bound to its cluster, requested at insertion
/// ([`crate::geometry::ColliderSet::insert_deformable`]).
#[derive(Clone, Debug)]
pub enum SoftMeshBindingMode {
    /// Vertex `i` of the mesh is the particle `particles[i]` of the cluster.
    Direct {
        /// The particle backing each vertex.
        particles: Vec<u32>,
    },
    /// Each vertex is the particle of the cluster closest to it, within `eps`: the common case,
    /// where the collision mesh was authored from the same vertices as the computational mesh.
    DirectByPosition {
        /// How far a vertex may sit from its particle.
        eps: Real,
    },
    /// Each vertex rides the cell of the cluster holding it (cage simulation): the cells only
    /// have to contain the mesh, not resolve it.
    Skinned,
}

/// How a deformable collider follows its soft body, and the collision settings that are not the
/// collider's own.
#[derive(Clone, Debug)]
pub struct SoftMeshBinding {
    pub(crate) mode: SoftMeshBindingMode,
    pub(crate) self_contacts: bool,
}

impl SoftMeshBinding {
    /// Binds vertex `i` of the mesh to `particles[i]`, which must belong to the cluster.
    pub fn direct(particles: Vec<u32>) -> Self {
        Self::with_mode(SoftMeshBindingMode::Direct { particles })
    }

    /// Binds every vertex to the particle of the cluster closest to it, within `eps`.
    pub fn direct_by_position(eps: Real) -> Self {
        Self::with_mode(SoftMeshBindingMode::DirectByPosition { eps })
    }

    /// Binds every vertex to the cell of the cluster holding it (cage simulation).
    pub fn skinned() -> Self {
        Self::with_mode(SoftMeshBindingMode::Skinned)
    }

    fn with_mode(mode: SoftMeshBindingMode) -> Self {
        Self {
            mode,
            self_contacts: false,
        }
    }

    /// Enables collisions of this mesh with itself.
    pub fn self_contacts(mut self, enabled: bool) -> Self {
        self.self_contacts = enabled;
        self
    }
}

/// Why a mesh could not be bound to a cluster
/// ([`crate::geometry::ColliderSet::insert_deformable`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SoftBindingError {
    /// The parent is not a live cluster proxy ([`crate::dynamics::SoftBodyCluster::proxy`]).
    #[error("the parent body is not a soft-body cluster proxy")]
    NotAClusterProxy,
    /// The collider's shape is not a polyline (2D) or triangle mesh (3D).
    #[error("a deformable collider must be a polyline or a trimesh")]
    UnsupportedShape,
    /// The collider's shape does not have the deformable flag (`TriMeshFlags::DEFORMABLE`,
    /// `PolylineFlags::DEFORMABLE`), so its vertices cannot be moved in place.
    #[error("a deformable collider's shape must have the deformable flag")]
    NotDeformable,
    /// The mesh has no vertex, or no element.
    #[error("the mesh has no vertex or no element")]
    DegenerateMesh,
    /// A direct binding names a particle that does not belong to the cluster.
    #[error("vertex {vertex} is bound to particle {particle}, which is not in the cluster")]
    VertexOutsideCluster {
        /// The mesh vertex.
        vertex: u32,
        /// The particle it names.
        particle: u32,
    },
    /// A direct binding by position found no particle of the cluster close enough to a vertex.
    #[error("no particle of the cluster is close enough to vertex {vertex}")]
    UnmatchedVertex {
        /// The mesh vertex.
        vertex: u32,
    },
    /// A skinned binding found no cell of the cluster to hold a vertex.
    #[error("no cell of the cluster can hold vertex {vertex}")]
    UnboundVertex {
        /// The mesh vertex.
        vertex: u32,
    },
}

/// The binding of `position` to the closest cell of `body` accepted by `allowed` (`None` without
/// any such cell).
// TODO: optimize neighrest neighbor search with the BVH?
pub(super) fn closest_cell_binding(body: &SoftBody, position: Vector) -> Option<SoftMeshCellBinding> {
}
