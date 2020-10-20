use crate::geometry::{PointProjection, Ray, RayIntersection, Triangle, WQuadtree};
use crate::math::{Isometry, Point};
use na::Point3;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use ncollide::query::{PointQuery, RayCast};
use ncollide::shape::FeatureId;

#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A triangle mesh.
pub struct Trimesh {
    wquadtree: WQuadtree<usize>,
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

        let aabb = AABB::from_points(&vertices);
        let data = indices.iter().enumerate().map(|(i, idx)| {
            let aabb = Triangle::new(
                vertices[idx[0] as usize],
                vertices[idx[1] as usize],
                vertices[idx[2] as usize],
            )
            .local_bounding_volume();
            (i, aabb)
        });

        let mut wquadtree = WQuadtree::new();
        // NOTE: we apply no dilation factor because we won't
        // update this tree dynamically.
        wquadtree.clear_and_rebuild(data, 0.0);

        Self {
            wquadtree,
            aabb,
            vertices,
            indices,
        }
    }

    /// Compute the axis-aligned bounding box of this triangle mesh.
    pub fn aabb(&self, pos: &Isometry<f32>) -> AABB<f32> {
        self.aabb.transform_by(pos)
    }

    pub(crate) fn waabbs(&self) -> &WQuadtree<usize> {
        &self.wquadtree
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

impl PointQuery<f32> for Trimesh {
    fn project_point(&self, _m: &Isometry<f32>, _pt: &Point<f32>, _solid: bool) -> PointProjection {
        // TODO
        unimplemented!()
    }

    fn project_point_with_feature(
        &self,
        _m: &Isometry<f32>,
        _pt: &Point<f32>,
    ) -> (PointProjection, FeatureId) {
        // TODO
        unimplemented!()
    }
}

#[cfg(feature = "dim2")]
impl RayCast<f32> for Trimesh {
    fn toi_and_normal_with_ray(
        &self,
        _m: &Isometry<f32>,
        _ray: &Ray,
        _max_toi: f32,
        _solid: bool,
    ) -> Option<RayIntersection> {
        // TODO
        None
    }

    fn intersects_ray(&self, _m: &Isometry<f32>, _ray: &Ray, _max_toi: f32) -> bool {
        // TODO
        false
    }
}

#[cfg(feature = "dim3")]
impl RayCast<f32> for Trimesh {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<f32>,
        ray: &Ray,
        max_toi: f32,
        solid: bool,
    ) -> Option<RayIntersection> {
        // FIXME: do a best-first search.
        let mut intersections = Vec::new();
        let ls_ray = ray.inverse_transform_by(m);
        self.wquadtree
            .cast_ray(&ls_ray, max_toi, &mut intersections);
        let mut best: Option<RayIntersection> = None;

        for inter in intersections {
            let tri = self.triangle(inter);
            if let Some(inter) = tri.toi_and_normal_with_ray(m, ray, max_toi, solid) {
                if let Some(curr) = &mut best {
                    if curr.toi > inter.toi {
                        *curr = inter;
                    }
                } else {
                    best = Some(inter);
                }
            }
        }

        best
    }

    fn intersects_ray(&self, m: &Isometry<f32>, ray: &Ray, max_toi: f32) -> bool {
        // FIXME: do a best-first search.
        let mut intersections = Vec::new();
        let ls_ray = ray.inverse_transform_by(m);
        self.wquadtree
            .cast_ray(&ls_ray, max_toi, &mut intersections);

        for inter in intersections {
            let tri = self.triangle(inter);
            if tri.intersects_ray(m, ray, max_toi) {
                return true;
            }
        }

        false
    }
}
