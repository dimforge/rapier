use crate::geometry::{RawPointProjection, RawRayIntersection, RawShapeCastHit, RawShapeContact};
use crate::math::{RawRotation, RawVector};
#[cfg(feature = "dim3")]
use na::DMatrix;
#[cfg(feature = "dim2")]
use na::DVector;
use na::Unit;
use rapier::geometry::{Shape, SharedShape, TriMeshFlags};
use rapier::math::{Isometry, Point, Real, Vector, DIM};
use rapier::parry::query;
use rapier::parry::query::{Ray, ShapeCastOptions};
use rapier::parry::transformation::vhacd::{VHACDParameters, VHACD};
use wasm_bindgen::prelude::*;

pub trait SharedShapeUtility {
    fn castShape(
        &self,
        shapePos1: &Isometry<Real>,
        shapeVel1: &Vector<Real>,
        shape2: &dyn Shape,
        shapePos2: &Isometry<Real>,
        shapeVel2: &Vector<Real>,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
    ) -> Option<RawShapeCastHit>;

    fn intersectsShape(
        &self,
        shapePos1: &Isometry<Real>,
        shape2: &dyn Shape,
        shapePos2: &Isometry<Real>,
    ) -> bool;

    fn contactShape(
        &self,
        shapePos1: &Isometry<Real>,
        shape2: &dyn Shape,
        shapePos2: &Isometry<Real>,
        prediction: f32,
    ) -> Option<RawShapeContact>;

    fn containsPoint(&self, shapePos: &Isometry<Real>, point: &Point<Real>) -> bool;

    fn projectPoint(
        &self,
        shapePos: &Isometry<Real>,
        point: &Point<Real>,
        solid: bool,
    ) -> RawPointProjection;

    fn intersectsRay(
        &self,
        shapePos: &Isometry<Real>,
        rayOrig: Point<Real>,
        rayDir: Vector<Real>,
        maxToi: f32,
    ) -> bool;

    fn castRay(
        &self,
        shapePos: &Isometry<Real>,
        rayOrig: Point<Real>,
        rayDir: Vector<Real>,
        maxToi: f32,
        solid: bool,
    ) -> f32;

    fn castRayAndGetNormal(
        &self,
        shapePos: &Isometry<Real>,
        rayOrig: Point<Real>,
        rayDir: Vector<Real>,
        maxToi: f32,
        solid: bool,
    ) -> Option<RawRayIntersection>;
}

#[cfg(feature = "dim3")]
pub(crate) fn normalized_convex_polyhedron_mesh(
    polyhedron: &rapier::parry::shape::ConvexPolyhedron,
) -> Option<(Vec<Point<Real>>, Vec<u32>)> {
    let (points, indices) =
        rapier::parry::transformation::try_convex_hull(polyhedron.points()).ok()?;
    let flat_indices = indices
        .iter()
        .flat_map(|triangle| triangle.iter())
        .copied()
        .collect();
    Some((points, flat_indices))
}

// for RawShape & Collider
impl SharedShapeUtility for SharedShape {
    fn castShape(
        &self,
        shapePos1: &Isometry<Real>,
        shapeVel1: &Vector<Real>,
        shape2: &dyn Shape,
        shapePos2: &Isometry<Real>,
        shapeVel2: &Vector<Real>,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
    ) -> Option<RawShapeCastHit> {
        query::cast_shapes(
            shapePos1,
            shapeVel1,
            &*self.0,
            shapePos2,
            &shapeVel2,
            shape2,
            ShapeCastOptions {
                max_time_of_impact: maxToi,
                target_distance,
                stop_at_penetration,
                compute_impact_geometry_on_penetration: true,
            },
        )
        .ok()
        .flatten()
        .map(|hit| RawShapeCastHit { hit })
    }

    fn intersectsShape(
        &self,
        shapePos1: &Isometry<Real>,
        shape2: &dyn Shape,
        shapePos2: &Isometry<Real>,
    ) -> bool {
        query::intersection_test(shapePos1, &*self.0, shapePos2, shape2).unwrap_or(false)
    }

    fn contactShape(
        &self,
        shapePos1: &Isometry<Real>,
        shape2: &dyn Shape,
        shapePos2: &Isometry<Real>,
        prediction: f32,
    ) -> Option<RawShapeContact> {
        query::contact(shapePos1, &*self.0, shapePos2, shape2, prediction)
            .ok()
            .flatten()
            .map(|contact| RawShapeContact { contact })
    }

    fn containsPoint(&self, shapePos: &Isometry<Real>, point: &Point<Real>) -> bool {
        self.as_ref().contains_point(shapePos, point)
    }

    fn projectPoint(
        &self,
        shapePos: &Isometry<Real>,
        point: &Point<Real>,
        solid: bool,
    ) -> RawPointProjection {
        RawPointProjection(self.as_ref().project_point(shapePos, point, solid))
    }

    fn intersectsRay(
        &self,
        shapePos: &Isometry<Real>,
        rayOrig: Point<Real>,
        rayDir: Vector<Real>,
        maxToi: f32,
    ) -> bool {
        self.as_ref()
            .intersects_ray(shapePos, &Ray::new(rayOrig, rayDir), maxToi)
    }

    fn castRay(
        &self,
        shapePos: &Isometry<Real>,
        rayOrig: Point<Real>,
        rayDir: Vector<Real>,
        maxToi: f32,
        solid: bool,
    ) -> f32 {
        self.as_ref()
            .cast_ray(shapePos, &Ray::new(rayOrig, rayDir), maxToi, solid)
            .unwrap_or(-1.0) // Negative value = no hit.
    }

    fn castRayAndGetNormal(
        &self,
        shapePos: &Isometry<Real>,
        rayOrig: Point<Real>,
        rayDir: Vector<Real>,
        maxToi: f32,
        solid: bool,
    ) -> Option<RawRayIntersection> {
        self.as_ref()
            .cast_ray_and_get_normal(shapePos, &Ray::new(rayOrig, rayDir), maxToi, solid)
            .map(|inter| RawRayIntersection(inter))
    }
}

#[wasm_bindgen]
#[cfg(feature = "dim2")]
pub enum RawShapeType {
    Ball = 0,
    Cuboid = 1,
    Capsule = 2,
    Segment = 3,
    Polyline = 4,
    Triangle = 5,
    TriMesh = 6,
    HeightField = 7,
    Compound = 8,
    ConvexPolygon = 9,
    RoundCuboid = 10,
    RoundTriangle = 11,
    RoundConvexPolygon = 12,
    HalfSpace = 13,
    Voxels = 14,
}

#[wasm_bindgen]
#[cfg(feature = "dim3")]
pub enum RawShapeType {
    Ball = 0,
    Cuboid = 1,
    Capsule = 2,
    Segment = 3,
    Polyline = 4,
    Triangle = 5,
    TriMesh = 6,
    HeightField = 7,
    Compound = 8,
    ConvexPolyhedron = 9,
    Cylinder = 10,
    Cone = 11,
    RoundCuboid = 12,
    RoundTriangle = 13,
    RoundCylinder = 14,
    RoundCone = 15,
    RoundConvexPolyhedron = 16,
    HalfSpace = 17,
    Voxels = 18,
}

/// Parameters for VHACD convex decomposition algorithm
#[wasm_bindgen]
pub struct RawVHACDParameters(pub(crate) VHACDParameters);

#[wasm_bindgen]
impl RawVHACDParameters {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawVHACDParameters(VHACDParameters::default())
    }

    #[wasm_bindgen(getter)]
    pub fn alpha(&self) -> f32 {
        self.0.alpha
    }
    #[wasm_bindgen(setter)]
    pub fn set_alpha(&mut self, val: f32) {
        self.0.alpha = val.clamp(0.0, 1.0);
    }

    #[wasm_bindgen(getter)]
    pub fn beta(&self) -> f32 {
        self.0.beta
    }
    #[wasm_bindgen(setter)]
    pub fn set_beta(&mut self, val: f32) {
        self.0.beta = val.clamp(0.0, 1.0);
    }

    #[wasm_bindgen(getter)]
    pub fn concavity(&self) -> f32 {
        self.0.concavity
    }
    #[wasm_bindgen(setter)]
    pub fn set_concavity(&mut self, val: f32) {
        self.0.concavity = val.clamp(0.0, 1.0);
    }

    #[wasm_bindgen(getter)]
    pub fn plane_downsampling(&self) -> u32 {
        self.0.plane_downsampling
    }
    #[wasm_bindgen(setter)]
    pub fn set_plane_downsampling(&mut self, val: u32) {
        self.0.plane_downsampling = val;
    }

    #[wasm_bindgen(getter)]
    pub fn convex_hull_downsampling(&self) -> u32 {
        self.0.convex_hull_downsampling
    }
    #[wasm_bindgen(setter)]
    pub fn set_convex_hull_downsampling(&mut self, val: u32) {
        self.0.convex_hull_downsampling = val;
    }

    #[wasm_bindgen(getter)]
    pub fn max_convex_hulls(&self) -> u32 {
        self.0.max_convex_hulls
    }
    #[wasm_bindgen(setter)]
    pub fn set_max_convex_hulls(&mut self, val: u32) {
        self.0.max_convex_hulls = val;
    }

    #[wasm_bindgen(getter)]
    pub fn resolution(&self) -> u32 {
        self.0.resolution
    }
    #[wasm_bindgen(setter)]
    pub fn set_resolution(&mut self, val: u32) {
        self.0.resolution = val;
    }

    #[wasm_bindgen(getter)]
    pub fn convex_hull_approximation(&self) -> bool {
        self.0.convex_hull_approximation
    }
    #[wasm_bindgen(setter)]
    pub fn set_convex_hull_approximation(&mut self, val: bool) {
        self.0.convex_hull_approximation = val;
    }
}

/// The vertex/index buffers of a convex polyhedron’s convex hull.
#[cfg(feature = "dim3")]
#[wasm_bindgen(getter_with_clone)]
pub struct RawConvexMeshData {
    pub vertices: Vec<f32>,
    pub indices: Vec<u32>,
}

#[wasm_bindgen]
pub struct RawShape(pub(crate) SharedShape);

#[wasm_bindgen]
impl RawShape {
    pub fn shapeType(&self) -> RawShapeType {
        match self.0.shape_type() {
            rapier::geometry::ShapeType::Ball => RawShapeType::Ball,
            rapier::geometry::ShapeType::Cuboid => RawShapeType::Cuboid,
            rapier::geometry::ShapeType::Capsule => RawShapeType::Capsule,
            rapier::geometry::ShapeType::Segment => RawShapeType::Segment,
            rapier::geometry::ShapeType::Polyline => RawShapeType::Polyline,
            rapier::geometry::ShapeType::Triangle => RawShapeType::Triangle,
            rapier::geometry::ShapeType::TriMesh => RawShapeType::TriMesh,
            rapier::geometry::ShapeType::HeightField => RawShapeType::HeightField,
            rapier::geometry::ShapeType::Compound => RawShapeType::Compound,
            rapier::geometry::ShapeType::HalfSpace => RawShapeType::HalfSpace,
            rapier::geometry::ShapeType::Voxels => RawShapeType::Voxels,
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::ConvexPolyhedron => RawShapeType::ConvexPolyhedron,
            #[cfg(feature = "dim2")]
            rapier::geometry::ShapeType::ConvexPolygon => RawShapeType::ConvexPolygon,
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::Cylinder => RawShapeType::Cylinder,
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::Cone => RawShapeType::Cone,
            rapier::geometry::ShapeType::RoundCuboid => RawShapeType::RoundCuboid,
            rapier::geometry::ShapeType::RoundTriangle => RawShapeType::RoundTriangle,
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundCylinder => RawShapeType::RoundCylinder,
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundCone => RawShapeType::RoundCone,
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundConvexPolyhedron => {
                RawShapeType::RoundConvexPolyhedron
            }
            #[cfg(feature = "dim2")]
            rapier::geometry::ShapeType::RoundConvexPolygon => RawShapeType::RoundConvexPolygon,
            rapier::geometry::ShapeType::Custom => panic!("Not yet implemented."),
        }
    }

    pub fn halfspaceNormal(&self) -> Option<RawVector> {
        self.0
            .as_halfspace()
            .map(|halfspace| halfspace.normal.into_inner().into())
    }

    pub fn halfExtents(&self) -> Option<RawVector> {
        self.0
            .as_cuboid()
            .map(|cuboid| cuboid.half_extents.into())
            .or_else(|| {
                self.0
                    .as_round_cuboid()
                    .map(|cuboid| cuboid.inner_shape.half_extents.into())
            })
    }

    pub fn radius(&self) -> Option<f32> {
        match self.0.shape_type() {
            rapier::geometry::ShapeType::Ball => self.0.as_ball().map(|ball| ball.radius),
            rapier::geometry::ShapeType::Capsule => {
                self.0.as_capsule().map(|capsule| capsule.radius)
            }
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::Cylinder => {
                self.0.as_cylinder().map(|cylinder| cylinder.radius)
            }
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundCylinder => self
                .0
                .as_round_cylinder()
                .map(|cylinder| cylinder.inner_shape.radius),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::Cone => self.0.as_cone().map(|cone| cone.radius),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundCone => {
                self.0.as_round_cone().map(|cone| cone.inner_shape.radius)
            }
            _ => None,
        }
    }

    pub fn halfHeight(&self) -> Option<f32> {
        match self.0.shape_type() {
            rapier::geometry::ShapeType::Capsule => {
                self.0.as_capsule().map(|capsule| capsule.half_height())
            }
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::Cylinder => {
                self.0.as_cylinder().map(|cylinder| cylinder.half_height)
            }
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundCylinder => self
                .0
                .as_round_cylinder()
                .map(|cylinder| cylinder.inner_shape.half_height),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::Cone => self.0.as_cone().map(|cone| cone.half_height),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundCone => self
                .0
                .as_round_cone()
                .map(|cone| cone.inner_shape.half_height),
            _ => None,
        }
    }

    pub fn roundRadius(&self) -> Option<f32> {
        match self.0.shape_type() {
            rapier::geometry::ShapeType::RoundCuboid => {
                self.0.as_round_cuboid().map(|cuboid| cuboid.border_radius)
            }
            rapier::geometry::ShapeType::RoundTriangle => self
                .0
                .as_round_triangle()
                .map(|triangle| triangle.border_radius),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundCylinder => self
                .0
                .as_round_cylinder()
                .map(|cylinder| cylinder.border_radius),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundCone => {
                self.0.as_round_cone().map(|cone| cone.border_radius)
            }
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundConvexPolyhedron => self
                .0
                .as_round_convex_polyhedron()
                .map(|polyhedron| polyhedron.border_radius),
            #[cfg(feature = "dim2")]
            rapier::geometry::ShapeType::RoundConvexPolygon => self
                .0
                .as_round_convex_polygon()
                .map(|polygon| polygon.border_radius),
            _ => None,
        }
    }

    pub fn voxelData(&self) -> Option<Vec<i32>> {
        let voxels = self.0.as_voxels()?;
        Some(
            voxels
                .voxels()
                .filter_map(|vox| (!vox.state.is_empty()).then_some(vox.grid_coords))
                .flat_map(|ids| ids.coords.data.0[0])
                .collect(),
        )
    }

    pub fn voxelSize(&self) -> Option<RawVector> {
        self.0
            .as_voxels()
            .map(|voxels| RawVector(voxels.voxel_size()))
    }

    /// The vertices of this shape, if it is vertex-based.
    ///
    /// For convex polyhedra, this returns the vertices of a convex hull recomputed with
    /// `try_convex_hull` (so they may differ in count and order from the points the shape
    /// was built from), ensuring the result can be fed back to `RawShape::convexMesh`.
    /// If both `vertices` and `indices` are needed, prefer `convexMeshData` which computes
    /// the convex hull only once.
    pub fn vertices(&self) -> Option<Vec<f32>> {
        let flatten = |vertices: &[Point<f32>]| {
            vertices
                .iter()
                .flat_map(|point| point.iter())
                .copied()
                .collect()
        };

        match self.0.shape_type() {
            rapier::geometry::ShapeType::TriMesh => {
                self.0.as_trimesh().map(|mesh| flatten(mesh.vertices()))
            }
            #[cfg(feature = "dim2")]
            rapier::geometry::ShapeType::Polyline => self
                .0
                .as_polyline()
                .map(|polyline| flatten(polyline.vertices())),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::ConvexPolyhedron => self
                .0
                .as_convex_polyhedron()
                .and_then(normalized_convex_polyhedron_mesh)
                .map(|(points, _)| flatten(&points)),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundConvexPolyhedron => self
                .0
                .as_round_convex_polyhedron()
                .and_then(|polyhedron| normalized_convex_polyhedron_mesh(&polyhedron.inner_shape))
                .map(|(points, _)| flatten(&points)),
            #[cfg(feature = "dim2")]
            rapier::geometry::ShapeType::ConvexPolygon => self
                .0
                .as_convex_polygon()
                .map(|polygon| flatten(polygon.points())),
            #[cfg(feature = "dim2")]
            rapier::geometry::ShapeType::RoundConvexPolygon => self
                .0
                .as_round_convex_polygon()
                .map(|polygon| flatten(polygon.inner_shape.points())),
            rapier::geometry::ShapeType::Segment => self
                .0
                .as_segment()
                .map(|segment| flatten(&[segment.a, segment.b])),
            rapier::geometry::ShapeType::RoundTriangle => {
                self.0.as_round_triangle().map(|triangle| {
                    flatten(&[
                        triangle.inner_shape.a,
                        triangle.inner_shape.b,
                        triangle.inner_shape.c,
                    ])
                })
            }
            rapier::geometry::ShapeType::Triangle => self
                .0
                .as_triangle()
                .map(|triangle| flatten(&[triangle.a, triangle.b, triangle.c])),
            _ => None,
        }
    }

    pub fn indices(&self) -> Option<Vec<u32>> {
        match self.0.shape_type() {
            rapier::geometry::ShapeType::TriMesh => self.0.as_trimesh().map(|mesh| {
                mesh.indices()
                    .iter()
                    .flat_map(|triangle| triangle.iter())
                    .copied()
                    .collect()
            }),
            rapier::geometry::ShapeType::Polyline => self.0.as_polyline().map(|polyline| {
                polyline
                    .indices()
                    .iter()
                    .flat_map(|segment| segment.iter())
                    .copied()
                    .collect()
            }),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::ConvexPolyhedron => self
                .0
                .as_convex_polyhedron()
                .and_then(normalized_convex_polyhedron_mesh)
                .map(|(_, indices)| indices),
            #[cfg(feature = "dim3")]
            rapier::geometry::ShapeType::RoundConvexPolyhedron => self
                .0
                .as_round_convex_polyhedron()
                .and_then(|polyhedron| normalized_convex_polyhedron_mesh(&polyhedron.inner_shape))
                .map(|(_, indices)| indices),
            _ => None,
        }
    }

    /// The vertices and indices of the convex hull of this convex polyhedron, recomputed
    /// with `try_convex_hull` so that the result can always be fed back to
    /// `RawShape::convexMesh`.
    ///
    /// This computes the convex hull only once, unlike calling both `vertices()` and
    /// `indices()`.
    #[cfg(feature = "dim3")]
    pub fn convexMeshData(&self) -> Option<RawConvexMeshData> {
        let polyhedron = match self.0.shape_type() {
            rapier::geometry::ShapeType::ConvexPolyhedron => self.0.as_convex_polyhedron(),
            rapier::geometry::ShapeType::RoundConvexPolyhedron => self
                .0
                .as_round_convex_polyhedron()
                .map(|polyhedron| &polyhedron.inner_shape),
            _ => None,
        }?;
        let (points, indices) = normalized_convex_polyhedron_mesh(polyhedron)?;
        Some(RawConvexMeshData {
            vertices: points
                .iter()
                .flat_map(|point| point.iter())
                .copied()
                .collect(),
            indices,
        })
    }

    pub fn triMeshFlags(&self) -> Option<u32> {
        self.0
            .as_trimesh()
            .map(|trimesh| trimesh.flags().bits() as u32)
    }

    #[cfg(feature = "dim3")]
    pub fn heightFieldFlags(&self) -> Option<u32> {
        self.0
            .as_heightfield()
            .map(|heightfield| heightfield.flags().bits() as u32)
    }

    pub fn heightfieldHeights(&self) -> Option<Vec<f32>> {
        match self.0.shape_type() {
            rapier::geometry::ShapeType::HeightField => self
                .0
                .as_heightfield()
                .map(|heightfield| heightfield.heights().as_slice().to_vec()),
            _ => None,
        }
    }

    pub fn heightfieldScale(&self) -> Option<RawVector> {
        match self.0.shape_type() {
            rapier::geometry::ShapeType::HeightField => self
                .0
                .as_heightfield()
                .map(|heightfield| RawVector(*heightfield.scale())),
            _ => None,
        }
    }

    #[cfg(feature = "dim3")]
    pub fn heightfieldNRows(&self) -> Option<usize> {
        match self.0.shape_type() {
            rapier::geometry::ShapeType::HeightField => self
                .0
                .as_heightfield()
                .map(|heightfield| heightfield.nrows()),
            _ => None,
        }
    }

    #[cfg(feature = "dim3")]
    pub fn heightfieldNCols(&self) -> Option<usize> {
        match self.0.shape_type() {
            rapier::geometry::ShapeType::HeightField => self
                .0
                .as_heightfield()
                .map(|heightfield| heightfield.ncols()),
            _ => None,
        }
    }

    pub fn compoundLen(&self) -> Option<usize> {
        self.0.as_compound().map(|compound| compound.shapes().len())
    }

    pub fn compoundShape(&self, index: usize) -> Option<RawShape> {
        self.0
            .as_compound()
            .and_then(|compound| compound.shapes().get(index))
            .map(|(_, shape)| RawShape(shape.clone()))
    }

    pub fn compoundTranslation(&self, index: usize) -> Option<RawVector> {
        self.0
            .as_compound()
            .and_then(|compound| compound.shapes().get(index))
            .map(|(position, _)| position.translation.vector.into())
    }

    pub fn compoundRotation(&self, index: usize) -> Option<RawRotation> {
        self.0
            .as_compound()
            .and_then(|compound| compound.shapes().get(index))
            .map(|(position, _)| position.rotation.into())
    }

    #[cfg(feature = "dim2")]
    pub fn cuboid(hx: f32, hy: f32) -> Self {
        Self(SharedShape::cuboid(hx, hy))
    }

    #[cfg(feature = "dim3")]
    pub fn cuboid(hx: f32, hy: f32, hz: f32) -> Self {
        Self(SharedShape::cuboid(hx, hy, hz))
    }

    #[cfg(feature = "dim2")]
    pub fn roundCuboid(hx: f32, hy: f32, borderRadius: f32) -> Self {
        Self(SharedShape::round_cuboid(hx, hy, borderRadius))
    }

    #[cfg(feature = "dim3")]
    pub fn roundCuboid(hx: f32, hy: f32, hz: f32, borderRadius: f32) -> Self {
        Self(SharedShape::round_cuboid(hx, hy, hz, borderRadius))
    }

    pub fn ball(radius: f32) -> Self {
        Self(SharedShape::ball(radius))
    }

    pub fn halfspace(normal: &RawVector) -> Self {
        Self(SharedShape::halfspace(Unit::new_normalize(normal.0)))
    }

    pub fn capsule(halfHeight: f32, radius: f32) -> Self {
        let p2 = Point::from(Vector::y() * halfHeight);
        let p1 = -p2;
        Self(SharedShape::capsule(p1, p2, radius))
    }

    #[cfg(feature = "dim3")]
    pub fn cylinder(halfHeight: f32, radius: f32) -> Self {
        Self(SharedShape::cylinder(halfHeight, radius))
    }

    #[cfg(feature = "dim3")]
    pub fn roundCylinder(halfHeight: f32, radius: f32, borderRadius: f32) -> Self {
        Self(SharedShape::round_cylinder(
            halfHeight,
            radius,
            borderRadius,
        ))
    }

    #[cfg(feature = "dim3")]
    pub fn cone(halfHeight: f32, radius: f32) -> Self {
        Self(SharedShape::cone(halfHeight, radius))
    }

    #[cfg(feature = "dim3")]
    pub fn roundCone(halfHeight: f32, radius: f32, borderRadius: f32) -> Self {
        Self(SharedShape::round_cone(halfHeight, radius, borderRadius))
    }

    pub fn voxels(voxel_size: &RawVector, grid_coords: Vec<i32>) -> Self {
        let grid_coords: Vec<_> = grid_coords
            .chunks_exact(DIM)
            .map(Point::from_slice)
            .collect();
        Self(SharedShape::voxels(voxel_size.0, &grid_coords))
    }

    pub fn voxelsFromPoints(voxel_size: &RawVector, points: Vec<f32>) -> Self {
        let points: Vec<_> = points.chunks_exact(DIM).map(Point::from_slice).collect();
        Self(SharedShape::voxels_from_points(voxel_size.0, &points))
    }

    pub fn polyline(vertices: Vec<f32>, indices: Vec<u32>) -> Self {
        let vertices = vertices.chunks(DIM).map(|v| Point::from_slice(v)).collect();
        let indices: Vec<_> = indices.chunks(2).map(|v| [v[0], v[1]]).collect();
        if indices.is_empty() {
            Self(SharedShape::polyline(vertices, None))
        } else {
            Self(SharedShape::polyline(vertices, Some(indices)))
        }
    }

    pub fn trimesh(vertices: Vec<f32>, indices: Vec<u32>, flags: u32) -> Option<RawShape> {
        let flags = TriMeshFlags::from_bits(flags as u16).unwrap_or_default();
        let vertices = vertices.chunks(DIM).map(|v| Point::from_slice(v)).collect();
        let indices = indices.chunks(3).map(|v| [v[0], v[1], v[2]]).collect();
        SharedShape::trimesh_with_flags(vertices, indices, flags)
            .ok()
            .map(Self)
    }

    #[cfg(feature = "dim2")]
    pub fn heightfield(heights: Vec<f32>, scale: &RawVector) -> Self {
        let heights = DVector::from_vec(heights);
        Self(SharedShape::heightfield(heights, scale.0))
    }

    #[cfg(feature = "dim3")]
    pub fn heightfield(
        nrows: u32,
        ncols: u32,
        heights: Vec<f32>,
        scale: &RawVector,
        flags: u32,
    ) -> Self {
        let flags =
            rapier::parry::shape::HeightFieldFlags::from_bits(flags as u8).unwrap_or_default();
        let heights = DMatrix::from_vec(nrows as usize + 1, ncols as usize + 1, heights);
        Self(SharedShape::heightfield_with_flags(heights, scale.0, flags))
    }

    pub fn segment(p1: &RawVector, p2: &RawVector) -> Self {
        Self(SharedShape::segment(p1.0.into(), p2.0.into()))
    }

    pub fn triangle(p1: &RawVector, p2: &RawVector, p3: &RawVector) -> Self {
        Self(SharedShape::triangle(p1.0.into(), p2.0.into(), p3.0.into()))
    }

    pub fn roundTriangle(
        p1: &RawVector,
        p2: &RawVector,
        p3: &RawVector,
        borderRadius: f32,
    ) -> Self {
        Self(SharedShape::round_triangle(
            p1.0.into(),
            p2.0.into(),
            p3.0.into(),
            borderRadius,
        ))
    }

    pub fn convexHull(points: Vec<f32>) -> Option<RawShape> {
        let points: Vec<_> = points.chunks(DIM).map(|v| Point::from_slice(v)).collect();
        SharedShape::convex_hull(&points).map(|s| Self(s))
    }

    pub fn roundConvexHull(points: Vec<f32>, borderRadius: f32) -> Option<RawShape> {
        let points: Vec<_> = points.chunks(DIM).map(|v| Point::from_slice(v)).collect();
        SharedShape::round_convex_hull(&points, borderRadius).map(|s| Self(s))
    }

    #[cfg(feature = "dim2")]
    pub fn convexPolyline(vertices: Vec<f32>) -> Option<RawShape> {
        let vertices = vertices.chunks(DIM).map(|v| Point::from_slice(v)).collect();
        SharedShape::convex_polyline(vertices).map(|s| Self(s))
    }

    #[cfg(feature = "dim2")]
    pub fn roundConvexPolyline(vertices: Vec<f32>, borderRadius: f32) -> Option<RawShape> {
        let vertices = vertices.chunks(DIM).map(|v| Point::from_slice(v)).collect();
        SharedShape::round_convex_polyline(vertices, borderRadius).map(|s| Self(s))
    }

    #[cfg(feature = "dim3")]
    pub fn convexMesh(vertices: Vec<f32>, indices: Vec<u32>) -> Option<RawShape> {
        let vertices = vertices.chunks(DIM).map(|v| Point::from_slice(v)).collect();
        let indices: Vec<_> = indices.chunks(3).map(|v| [v[0], v[1], v[2]]).collect();
        SharedShape::convex_mesh(vertices, &indices).map(|s| Self(s))
    }

    #[cfg(feature = "dim3")]
    pub fn roundConvexMesh(
        vertices: Vec<f32>,
        indices: Vec<u32>,
        borderRadius: f32,
    ) -> Option<RawShape> {
        let vertices = vertices.chunks(DIM).map(|v| Point::from_slice(v)).collect();
        let indices: Vec<_> = indices.chunks(3).map(|v| [v[0], v[1], v[2]]).collect();
        SharedShape::round_convex_mesh(vertices, &indices, borderRadius).map(|s| Self(s))
    }

    pub fn compound(shapes: Vec<RawShape>, positions: Vec<f32>, rotations: Vec<f32>) -> RawShape {
        let mut compound_parts = Vec::new();
        let num_shapes = shapes.len();

        assert_eq!(
            positions.len(),
            num_shapes * DIM,
            "The compound positions array must contain DIM entries per shape."
        );
        #[cfg(feature = "dim2")]
        assert_eq!(
            rotations.len(),
            num_shapes,
            "The compound rotations array must contain one angle per shape."
        );
        #[cfg(feature = "dim3")]
        assert_eq!(
            rotations.len(),
            num_shapes * 4,
            "The compound rotations array must contain one quaternion (4 entries) per shape."
        );

        for i in 0..num_shapes {
            let pos_offset = i * DIM;

            #[cfg(feature = "dim2")]
            let translation = Vector::new(positions[pos_offset], positions[pos_offset + 1]).into();

            #[cfg(feature = "dim3")]
            let translation = Vector::new(
                positions[pos_offset],
                positions[pos_offset + 1],
                positions[pos_offset + 2],
            )
            .into();

            #[cfg(feature = "dim2")]
            let rotation = na::UnitComplex::new(rotations[i]);

            #[cfg(feature = "dim3")]
            let rotation = {
                let rot_offset = i * 4;
                na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    rotations[rot_offset + 3], // w
                    rotations[rot_offset],     // x
                    rotations[rot_offset + 1], // y
                    rotations[rot_offset + 2], // z
                ))
            };

            let isometry = Isometry::from_parts(translation, rotation);
            compound_parts.push((isometry, shapes[i].0.clone()));
        }

        Self(SharedShape::compound(compound_parts))
    }

    pub fn convexDecomposition(vertices: Vec<f32>, indices: Vec<u32>) -> Option<RawShape> {
        Self::convexDecompositionWithParams(vertices, indices, &RawVHACDParameters::new())
    }

    pub fn convexDecompositionWithParams(
        vertices: Vec<f32>,
        indices: Vec<u32>,
        params: &RawVHACDParameters,
    ) -> Option<RawShape> {
        let vertices: Vec<_> = vertices.chunks(DIM).map(|v| Point::from_slice(v)).collect();
        #[cfg(feature = "dim2")]
        let indices: Vec<_> = indices.chunks(2).map(|v| [v[0], v[1]]).collect();
        #[cfg(feature = "dim3")]
        let indices: Vec<_> = indices.chunks(3).map(|v| [v[0], v[1], v[2]]).collect();

        // Same as `SharedShape::convex_decomposition_with_params`, except that a
        // decomposition yielding no convex part returns `None` instead of panicking
        // when building the empty compound.
        let decomp = VHACD::decompose(&params.0, &vertices, &indices, true);
        let mut parts = vec![];

        #[cfg(feature = "dim2")]
        for hull_vertices in decomp.compute_exact_convex_hulls(&vertices, &indices) {
            if let Some(convex) = SharedShape::convex_polyline(hull_vertices) {
                parts.push((Isometry::identity(), convex));
            }
        }

        #[cfg(feature = "dim3")]
        for (hull_vertices, hull_indices) in decomp.compute_exact_convex_hulls(&vertices, &indices)
        {
            if let Some(convex) = SharedShape::convex_mesh(hull_vertices, &hull_indices) {
                parts.push((Isometry::identity(), convex));
            }
        }

        if parts.is_empty() {
            return None;
        }

        Some(Self(SharedShape::compound(parts)))
    }

    pub fn castShape(
        &self,
        shapePos1: &RawVector,
        shapeRot1: &RawRotation,
        shapeVel1: &RawVector,
        shape2: &RawShape,
        shapePos2: &RawVector,
        shapeRot2: &RawRotation,
        shapeVel2: &RawVector,
        target_distance: f32,
        maxToi: f32,
        stop_at_penetration: bool,
    ) -> Option<RawShapeCastHit> {
        let pos1 = Isometry::from_parts(shapePos1.0.into(), shapeRot1.0);
        let pos2 = Isometry::from_parts(shapePos2.0.into(), shapeRot2.0);

        self.0.castShape(
            &pos1,
            &shapeVel1.0,
            &*shape2.0,
            &pos2,
            &shapeVel2.0,
            target_distance,
            maxToi,
            stop_at_penetration,
        )
    }

    pub fn intersectsShape(
        &self,
        shapePos1: &RawVector,
        shapeRot1: &RawRotation,
        shape2: &RawShape,
        shapePos2: &RawVector,
        shapeRot2: &RawRotation,
    ) -> bool {
        let pos1 = Isometry::from_parts(shapePos1.0.into(), shapeRot1.0);
        let pos2 = Isometry::from_parts(shapePos2.0.into(), shapeRot2.0);

        self.0.intersectsShape(&pos1, &*shape2.0, &pos2)
    }

    pub fn contactShape(
        &self,
        shapePos1: &RawVector,
        shapeRot1: &RawRotation,
        shape2: &RawShape,
        shapePos2: &RawVector,
        shapeRot2: &RawRotation,
        prediction: f32,
    ) -> Option<RawShapeContact> {
        let pos1 = Isometry::from_parts(shapePos1.0.into(), shapeRot1.0);
        let pos2 = Isometry::from_parts(shapePos2.0.into(), shapeRot2.0);

        self.0.contactShape(&pos1, &*shape2.0, &pos2, prediction)
    }

    pub fn containsPoint(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        point: &RawVector,
    ) -> bool {
        let pos = Isometry::from_parts(shapePos.0.into(), shapeRot.0);

        self.0.containsPoint(&pos, &point.0.into())
    }

    pub fn projectPoint(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        point: &RawVector,
        solid: bool,
    ) -> RawPointProjection {
        let pos = Isometry::from_parts(shapePos.0.into(), shapeRot.0);

        self.0.projectPoint(&pos, &point.0.into(), solid)
    }

    pub fn intersectsRay(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        rayOrig: &RawVector,
        rayDir: &RawVector,
        maxToi: f32,
    ) -> bool {
        let pos = Isometry::from_parts(shapePos.0.into(), shapeRot.0);

        self.0
            .intersectsRay(&pos, rayOrig.0.into(), rayDir.0.into(), maxToi)
    }

    pub fn castRay(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        rayOrig: &RawVector,
        rayDir: &RawVector,
        maxToi: f32,
        solid: bool,
    ) -> f32 {
        let pos = Isometry::from_parts(shapePos.0.into(), shapeRot.0);

        self.0
            .castRay(&pos, rayOrig.0.into(), rayDir.0.into(), maxToi, solid)
    }

    pub fn castRayAndGetNormal(
        &self,
        shapePos: &RawVector,
        shapeRot: &RawRotation,
        rayOrig: &RawVector,
        rayDir: &RawVector,
        maxToi: f32,
        solid: bool,
    ) -> Option<RawRayIntersection> {
        let pos = Isometry::from_parts(shapePos.0.into(), shapeRot.0);

        self.0
            .castRayAndGetNormal(&pos, rayOrig.0.into(), rayDir.0.into(), maxToi, solid)
    }
}

#[cfg(all(test, feature = "dim3"))]
mod tests {
    use super::RawShape;

    #[test]
    fn convex_decomposition_of_degenerate_mesh_returns_none() {
        // A single zero-area triangle can’t produce any convex part.
        let vertices = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0];
        let indices = vec![0, 1, 2];
        assert!(RawShape::convexDecomposition(vertices, indices).is_none());
    }

    #[test]
    fn raw_convex_mesh_accessors_round_trip_with_collinear_boundary_vertex() {
        let vertices = vec![
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0,
            1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.5,
        ];
        let indices = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 8],
            [8, 5, 4],
            [0, 8, 7],
            [8, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
        ];
        let flat_indices: Vec<u32> = indices
            .iter()
            .flat_map(|triangle| triangle.iter())
            .copied()
            .collect();

        let raw_shape = RawShape::convexMesh(vertices, flat_indices).unwrap();
        let roundtrip_vertices = raw_shape.vertices().unwrap();
        let roundtrip_indices = raw_shape.indices().unwrap();

        assert!(RawShape::convexMesh(roundtrip_vertices, roundtrip_indices).is_some());
    }

    #[test]
    fn captured_invalid_convex_mesh_export_is_not_reconstructable() {
        let vertices = vec![
            -3.2503600120544434,
            19.476539611816406,
            -2.0955400466918945,
            -3.3597400188446045,
            19.960920333862305,
            -2.0330400466918945,
            -3.1253600120544434,
            20.30466079711914,
            -2.126800060272217,
            -2.3614742755889893,
            20.145801544189453,
            -1.71620512008667,
            -3.3284800052642822,
            20.007780075073242,
            -1.8768000602722168,
            -2.3614742755889893,
            20.138565063476562,
            -1.8094041347503662,
            -3.1878600120544434,
            19.476539611816406,
            -1.8924200534820557,
            -2.3614742755889893,
            20.05278968811035,
            -1.4539750814437866,
            -2.3614742755889893,
            20.1295166015625,
            -1.4817370176315308,
            -3.0941200256347656,
            20.382780075073242,
            -1.8768000602722168,
            -2.3614742755889893,
            19.238826751708984,
            -1.366246223449707,
            -2.3614742755889893,
            19.214706420898438,
            -1.3726158142089844,
            -2.3614742755889893,
            19.113746643066406,
            -1.4277477264404297,
            -2.3614742755889893,
            19.131973266601562,
            -1.6892800331115723,
            -2.3614742755889893,
            19.141845703125,
            -1.792968511581421,
            -2.526392936706543,
            20.19354248046875,
            -1.9115056991577148,
            -2.448280096054077,
            19.144739151000977,
            -1.7378942966461182,
            -2.448280096054077,
            19.153003692626953,
            -1.824700117111206,
            -2.7034800052642822,
            19.195280075073242,
            -2.0174200534820557,
            -2.448280096054077,
            19.155406951904297,
            -1.8499374389648438,
            -2.406599998474121,
            20.17966079711914,
            -1.5018000602722168,
            -2.4691200256347656,
            20.085920333862305,
            -1.4705400466918945,
            -2.688455820083618,
            19.69875144958496,
            -2.0099079608917236,
            -2.6722400188446045,
            20.24216079711914,
            -2.001800060272217,
            -2.7034800052642822,
            19.185195922851562,
            -1.9115058183670044,
            -2.6722400188446045,
            20.320280075073242,
            -1.6580400466918945,
            -2.7034800052642822,
            19.164039611816406,
            -1.6892800331115723,
        ];
        let indices = vec![
            1, 4, 9, 1, 0, 4, 6, 4, 0, 6, 10, 4, 2, 1, 9, 2, 0, 1, 26, 6, 0, 26, 12, 6, 11, 6, 12,
            11, 10, 6, 21, 20, 25, 21, 25, 9, 21, 9, 4, 21, 4, 10, 8, 20, 21, 8, 21, 7, 26, 0, 18,
            13, 12, 26, 13, 26, 18, 13, 18, 14, 3, 5, 25, 3, 25, 20, 3, 20, 8, 7, 21, 10, 7, 10,
            11, 7, 11, 12, 7, 12, 13, 7, 13, 14, 7, 14, 5, 7, 5, 3, 7, 3, 8, 18, 0, 2, 18, 23, 5,
            25, 5, 23, 23, 15, 5, 23, 22, 18, 23, 18, 2, 23, 2, 9, 23, 9, 25, 18, 5, 14,
        ];

        assert!(RawShape::convexMesh(vertices, indices).is_none());
    }

    #[test]
    fn raw_convex_hull_accessors_round_trip_with_redundant_points() {
        let vertices = vec![
            -3.2503600120544434,
            19.476539611816406,
            -2.0955400466918945,
            -3.3597400188446045,
            19.960920333862305,
            -2.0330400466918945,
            -3.1253600120544434,
            20.30466079711914,
            -2.126800060272217,
            -2.3614742755889893,
            20.145801544189453,
            -1.71620512008667,
            -3.3284800052642822,
            20.007780075073242,
            -1.8768000602722168,
            -2.3614742755889893,
            20.138565063476562,
            -1.8094041347503662,
            -3.1878600120544434,
            19.476539611816406,
            -1.8924200534820557,
            -2.3614742755889893,
            20.05278968811035,
            -1.4539750814437866,
            -2.3614742755889893,
            20.1295166015625,
            -1.4817370176315308,
            -3.0941200256347656,
            20.382780075073242,
            -1.8768000602722168,
            -2.3614742755889893,
            19.238826751708984,
            -1.366246223449707,
            -2.3614742755889893,
            19.214706420898438,
            -1.3726158142089844,
            -2.3614742755889893,
            19.113746643066406,
            -1.4277477264404297,
            -2.3614742755889893,
            19.131973266601562,
            -1.6892800331115723,
            -2.3614742755889893,
            19.141845703125,
            -1.792968511581421,
            -2.526392936706543,
            20.19354248046875,
            -1.9115056991577148,
            -2.448280096054077,
            19.144739151000977,
            -1.7378942966461182,
            -2.448280096054077,
            19.153003692626953,
            -1.824700117111206,
            -2.7034800052642822,
            19.195280075073242,
            -2.0174200534820557,
            -2.448280096054077,
            19.155406951904297,
            -1.8499374389648438,
            -2.406599998474121,
            20.17966079711914,
            -1.5018000602722168,
            -2.4691200256347656,
            20.085920333862305,
            -1.4705400466918945,
            -2.688455820083618,
            19.69875144958496,
            -2.0099079608917236,
            -2.6722400188446045,
            20.24216079711914,
            -2.001800060272217,
            -2.7034800052642822,
            19.185195922851562,
            -1.9115058183670044,
            -2.6722400188446045,
            20.320280075073242,
            -1.6580400466918945,
            -2.7034800052642822,
            19.164039611816406,
            -1.6892800331115723,
        ];

        let raw_shape = RawShape::convexHull(vertices).unwrap();
        let roundtrip_vertices = raw_shape.vertices().unwrap();
        let roundtrip_indices = raw_shape.indices().unwrap();

        assert!(RawShape::convexMesh(roundtrip_vertices, roundtrip_indices).is_some());
    }
}
