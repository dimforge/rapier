//! Free mesh helpers of the soft-body builder: stray-piece dropping, surface tables, cell boundaries and the icosphere.

use crate::alloc_prelude::*;
use crate::math::{DIM, Real, Vector};
use parry::utils::hashmap::HashMap;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::super::SoftBodyCell;

/// Drops the pieces of a filled mesh that are negligible next to its largest one; pieces of a
/// comparable size are all kept, since dropping them would lose limbs rather than specks.
pub(super) fn drop_stray_pieces(
    mesh: &mut parry::transformation::VolumeMesh,
    components: &[u32],
    count: usize,
) {
    let mut sizes = vec![0; count];
    for id in components {
        sizes[*id as usize] += 1;
    }
    let largest = sizes.iter().max().copied().unwrap_or(0);
    let mut kept = components
        .iter()
        .map(|id| sizes[*id as usize] * 20 >= largest);
    mesh.cells.retain(|_| kept.next().unwrap_or(true));
    mesh.compact();
}

/// Mean length of `edges` over `positions` (`0.0` without edges).
pub(super) fn mean_edge_length(positions: &[Vector], edges: &[[u32; 2]]) -> Real {
    if edges.is_empty() {
        return 0.0;
    }
    edges
        .iter()
        .map(|e| (positions[e[0] as usize] - positions[e[1] as usize]).length())
        .sum::<Real>()
        / edges.len() as Real
}

    counts.values().all(|&c| c == 2)

/// The cell owning each surface element (`u32::MAX` when no cell contains its vertices).
pub(crate) fn surface_element_cells(surface: &[[u32; DIM]], cells: &[SoftBodyCell]) -> Vec<u32> {
    if cells.is_empty() {
        return Vec::new();
    }
    let mut owners: HashMap<[u32; DIM], u32> = HashMap::default();
    for (ci, cell) in cells.iter().enumerate() {
        for k in 0..=DIM {
            let mut facet = [0u32; DIM];
            let mut n = 0;
            for (j, &v) in cell.vertices.iter().enumerate() {
                if j != k {
                    facet[n] = v;
                    n += 1;
                }
            }
            facet.sort_unstable();
            owners.entry(facet).or_insert(ci as u32);
        }
    }
    surface
        .iter()
        .map(|element| {
            let mut key = *element;
            key.sort_unstable();
            owners.get(&key).copied().unwrap_or(u32::MAX)
        })
        .collect()
}

/// The unique edges of a triangle surface (`[a, b]`, `a < b`, in first-seen order), the edge ids
/// of every triangle (edge `k` opposite to vertex `k`), and the triangle owning each edge (the
/// first one containing it).
#[cfg(feature = "dim3")]
pub(crate) fn surface_edge_table(
    surface: &[[u32; DIM]],
) -> (Vec<[u32; 2]>, Vec<[u32; DIM]>, Vec<u32>) {
    let mut ids: HashMap<[u32; 2], u32> = HashMap::default();
    let mut edges = Vec::new();
    let mut owners = Vec::new();
    let mut element_edges = Vec::with_capacity(surface.len());
    for (ei, element) in surface.iter().enumerate() {
        let mut e = [u32::MAX; DIM];
        for k in 0..DIM {
            let a = element[(k + 1) % DIM];
            let b = element[(k + 2) % DIM];
            let key = [a.min(b), a.max(b)];
            let id = *ids.entry(key).or_insert_with(|| {
                edges.push(key);
                owners.push(ei as u32);
                edges.len() as u32 - 1
            });
            e[k] = id;
        }
        element_edges.push(e);
    }
    (edges, element_edges, owners)
}

/// The surface elements incident to every particle as a CSR (offsets, element ids).
pub(crate) fn surface_vertex_elements(
    num_particles: usize,
    surface: &[[u32; DIM]],
) -> (Vec<u32>, Vec<u32>) {
    let mut lists: Vec<Vec<u32>> = vec![Vec::new(); num_particles];
    for (i, element) in surface.iter().enumerate() {
        for &v in element_vertices(element) {
            if let Some(list) = lists.get_mut(v as usize) {
                list.push(i as u32);
            }
        }
    }
    let mut offsets = Vec::with_capacity(num_particles + 1);
    let mut flat = Vec::new();
    offsets.push(0);
    for list in &lists {
        flat.extend_from_slice(list);
        offsets.push(flat.len() as u32);
    }
    (offsets, flat)
}

/// The surface 1-ring of every particle as a CSR (offsets, sorted neighbor ids).
pub(crate) fn surface_rings(num_particles: usize, surface: &[[u32; DIM]]) -> (Vec<u32>, Vec<u32>) {
    let mut rings: Vec<Vec<u32>> = vec![Vec::new(); num_particles];
    for element in surface {
        for &a in element {
            for &b in element {
                if a != b {
                    if let Some(ring) = rings.get_mut(a as usize) {
                        ring.push(b);
                    }
                }
            }
        }
    }
    let mut offsets = Vec::with_capacity(num_particles + 1);
    let mut flat = Vec::new();
    offsets.push(0);
    for ring in &mut rings {
        ring.sort_unstable();
        ring.dedup();
        flat.extend_from_slice(ring);
        offsets.push(flat.len() as u32);
    }
    (offsets, flat)
}

/// The boundary of a set of positively-oriented cells: the faces (segments in 2D, triangles in
/// 3D) belonging to exactly one cell, oriented outward.
pub(super) fn cell_boundary(cells: &[SoftBodyCell]) -> Vec<[u32; DIM]> {
    let mut count: HashMap<[u32; DIM], u32> = HashMap::default();
    let mut faces: Vec<[u32; DIM]> = Vec::new();
    for c in cells {
        for face in cell_faces(c.vertices) {
            let mut key = face;
            key.sort_unstable();
            *count.entry(key).or_insert(0) += 1;
            faces.push(face);
        }
    }
    faces
        .into_iter()
        .filter(|face| {
            let mut key = *face;
            key.sort_unstable();
            count[&key] == 1
        })
        .collect()
}

/// The outward-oriented faces of a positively-oriented simplex cell.
pub(crate) fn cell_faces(v: [u32; DIM + 1]) -> [[u32; DIM]; DIM + 1] {
    #[cfg(feature = "dim2")]
    {
        [[v[0], v[1]], [v[1], v[2]], [v[2], v[0]]]
    }
    #[cfg(feature = "dim3")]
    {
        [
            [v[0], v[2], v[1]],
            [v[0], v[1], v[3]],
            [v[0], v[3], v[2]],
            [v[1], v[2], v[3]],
        ]
    }
}

/// The dihedral angle between the triangles `(p0, p1, p2)` and `(p0, p1, p3)`.
#[cfg(feature = "dim3")]
pub(crate) fn dihedral_angle(p0: Vector, p1: Vector, p2: Vector, p3: Vector) -> Real {
    let e = p1 - p0;
    let n1 = e.cross(p2 - p0);
    let n2 = e.cross(p3 - p0);
    let (l1, l2) = (n1.length(), n2.length());
    if l1 == 0.0 || l2 == 0.0 {
        return 0.0;
    }
    (n1.dot(n2) / (l1 * l2)).clamp(-1.0, 1.0).acos()
}

/// An icosphere: vertices on the sphere of the given center and radius, outward-oriented
/// triangles.
#[cfg(feature = "dim3")]
pub(super) fn icosphere(center: Vector, radius: Real, subdivisions: usize) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let t = (1.0 + Real::sqrt(5.0)) * 0.5;
    let mut vertices: Vec<Vector> = [
        (-1.0, t, 0.0),
        (1.0, t, 0.0),
        (-1.0, -t, 0.0),
        (1.0, -t, 0.0),
        (0.0, -1.0, t),
        (0.0, 1.0, t),
        (0.0, -1.0, -t),
        (0.0, 1.0, -t),
        (t, 0.0, -1.0),
        (t, 0.0, 1.0),
        (-t, 0.0, -1.0),
        (-t, 0.0, 1.0),
    ]
    .iter()
    .map(|(x, y, z)| Vector::new(*x, *y, *z).normalize())
    .collect();
    let mut triangles: Vec<[u32; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];

    for _ in 0..subdivisions {
        let mut midpoints: HashMap<[u32; 2], u32> = HashMap::default();
        let mut midpoint = |a: u32, b: u32, vertices: &mut Vec<Vector>| -> u32 {
            let key = [a.min(b), a.max(b)];
            *midpoints.entry(key).or_insert_with(|| {
                let m = ((vertices[a as usize] + vertices[b as usize]) * 0.5).normalize();
                vertices.push(m);
                vertices.len() as u32 - 1
            })
        };
        let mut next = Vec::with_capacity(triangles.len() * 4);
        for [a, b, c] in triangles {
            let ab = midpoint(a, b, &mut vertices);
            let bc = midpoint(b, c, &mut vertices);
            let ca = midpoint(c, a, &mut vertices);
            next.push([a, ab, ca]);
            next.push([b, bc, ab]);
            next.push([c, ca, bc]);
            next.push([ab, bc, ca]);
        }
        triangles = next;
    }

    // Outward orientation.
    for tri in &mut triangles {
        let (a, b, c) = (
            vertices[tri[0] as usize],
            vertices[tri[1] as usize],
            vertices[tri[2] as usize],
        );
        if (b - a).cross(c - a).dot(a + b + c) < 0.0 {
            tri.swap(1, 2);
        }
    }

    for v in &mut vertices {
        *v = center + *v * radius;
    }
    (vertices, triangles)
}
