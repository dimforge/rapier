use crate::geometry::{Triangle, WAABBHierarchy};
use crate::math::{Isometry, Point};
use na::Point3;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};

#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A triangle mesh.
pub struct Trimesh {
    waabb_tree: WAABBHierarchy,
    aabb: AABB<f32>,
    vertices: Vec<Point<f32>>,
    indices: Vec<Point3<u32>>,
}

impl Trimesh {
    /// Creates a new triangle mesh from a vertex buffer and an index buffer.
    pub fn new(vertices: Vec<Point<f32>>, indices: Vec<Point3<u32>>) -> Self {
        assert!(
            vertices.len() > 1,
            "A triangle mesh must contain at least one point."
        );
        assert!(
            indices.len() > 1,
            "A triangle mesh must contain at least one triangle."
        );

        // z-sort the indices.
        //        indices.sort_unstable_by(|idx, jdx| {
        //            let ti = Triangle::new(
        //                vertices[idx[0] as usize],
        //                vertices[idx[1] as usize],
        //                vertices[idx[2] as usize],
        //            );
        //            let tj = Triangle::new(
        //                vertices[jdx[0] as usize],
        //                vertices[jdx[1] as usize],
        //                vertices[jdx[2] as usize],
        //            );
        //            let center_i = (ti.a.coords + ti.b.coords + ti.c.coords) / 3.0;
        //            let center_j = (tj.a.coords + tj.b.coords + tj.c.coords) / 3.0;
        //            crate::geometry::z_cmp_floats(center_i.as_slice(), center_j.as_slice())
        //                .unwrap_or(std::cmp::Ordering::Equal)
        //        });
        let aabb = AABB::from_points(&vertices);

        let aabbs: Vec<_> = indices
            .iter()
            .map(|idx| {
                Triangle::new(
                    vertices[idx[0] as usize],
                    vertices[idx[1] as usize],
                    vertices[idx[2] as usize],
                )
                .local_bounding_volume()
            })
            .collect();

        let waabb_tree = WAABBHierarchy::new(&aabbs);

        Self {
            waabb_tree,
            aabb,
            vertices,
            indices,
        }
    }

    /// Compute the axis-aligned bounding box of this triangle mesh.
    pub fn aabb(&self, pos: &Isometry<f32>) -> AABB<f32> {
        self.aabb.transform_by(pos)
    }

    pub(crate) fn waabbs(&self) -> &WAABBHierarchy {
        &self.waabb_tree
    }

    /// The number of triangles forming this mesh.
    pub fn num_triangles(&self) -> usize {
        self.indices.len()
    }

    /// An iterator through all the triangles of this mesh.
    pub fn triangles(&self) -> impl Iterator<Item = Triangle> + '_ {
        self.indices.iter().map(move |ids| {
            Triangle::new(
                self.vertices[ids.x as usize],
                self.vertices[ids.y as usize],
                self.vertices[ids.z as usize],
            )
        })
    }

    /// Get the `i`-th triangle of this mesh.
    pub fn triangle(&self, i: usize) -> Triangle {
        let idx = self.indices[i];
        Triangle::new(
            self.vertices[idx.x as usize],
            self.vertices[idx.y as usize],
            self.vertices[idx.z as usize],
        )
    }

    /// The vertex buffer of this mesh.
    pub fn vertices(&self) -> &[Point<f32>] {
        &self.vertices[..]
    }

    /// The index buffer of this mesh.
    pub fn indices(&self) -> &[Point3<u32>] {
        &self.indices
    }

    /// A flat view of the index buffer of this mesh.
    pub fn flat_indices(&self) -> &[u32] {
        unsafe {
            let len = self.indices.len() * 3;
            let data = self.indices.as_ptr() as *const u32;
            std::slice::from_raw_parts(data, len)
        }
    }
}
