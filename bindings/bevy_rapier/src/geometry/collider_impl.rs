#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {
    bevy::mesh::{Indices, VertexAttributeValues},
    bevy::prelude::*,
};

use rapier::{
    geometry::{MeshConverter, MeshConverterError},
    math::Pose,
    parry::shape::{Compound, Polyline},
    parry::transformation::voxelization::FillMode,
    prelude::{FeatureId, Ray, SharedShape, Vector, Voxels, DIM},
};

use super::{get_snapped_scale, shape_views::*};
#[cfg(all(feature = "dim3", feature = "async-collider"))]
use crate::geometry::ComputedColliderShape;
#[cfg(feature = "dim3")]
use crate::geometry::HeightFieldFlags;
use crate::math::{Real, Rot, Vect};
#[cfg(feature = "dim2")]
use bevy::shape::{Aabb2d as BevyAabb, BoundingCircle as BevyBoundingSphere};
#[cfg(feature = "dim3")]
use bevy::shape::{Aabb3d as BevyAabb, BoundingSphere as BevyBoundingSphere};
use rapier::parry::query;

use crate::dynamics::MassProperties;
use crate::{
    geometry::{
        Collider, CompoundFlags, NonlinearMotion, PointProjection, PolylineFlags, RayIntersection,
        ShapeCastHit, ShapeCastOptions, ShapeClosestPoints, ShapeContact, TriMeshFlags,
        Unsupported, VHACDParameters,
    },
    math::IVect,
};

impl Collider {
    /// The scaling factor that was applied to this collider.
    pub fn scale(&self) -> Vect {
        self.scale
    }

    /// This replaces the unscaled version of this collider by its scaled version,
    /// and resets `self.scale()` to `1.0`.
    pub fn promote_scaled_shape(&mut self) {
        self.unscaled = self.raw.clone();
        self.scale = Vect::ONE;
    }

    /// Initialize a new collider with a compound shape.
    pub fn compound(shapes: Vec<(Vect, Rot, Collider)>) -> Self {
        let shapes = shapes
            .into_iter()
            .map(|(t, r, s)| (crate::utils::pose_from(t, r), s.raw))
            .collect();
        SharedShape::compound(shapes).into()
    }

    /// Initialize a new collider with a compound shape and flags controlling its optional
    /// pre-processing.
    ///
    /// `weld_tolerance` is the distance (in ULPs) under which two corners of different parts
    /// are considered equal when detecting internal edges; `None` selects parry's default.
    pub fn compound_with_flags(
        shapes: Vec<(Vect, Rot, Collider)>,
        flags: CompoundFlags,
        weld_tolerance: Option<Real>,
    ) -> Self {
        let shapes = shapes
            .into_iter()
            .map(|(t, r, s)| (crate::utils::pose_from(t, r), s.raw))
            .collect();
        SharedShape::new(Compound::with_flags(shapes, flags, weld_tolerance)).into()
    }

    /// Initialize a new collider with a ball shape defined by its radius.
    pub fn ball(radius: Real) -> Self {
        SharedShape::ball(radius).into()
    }

    /// Initialize a new collider build with a half-space shape defined by the outward normal
    /// of its planar boundary.
    pub fn halfspace(outward_normal: Vect) -> Option<Self> {
        let normal = Vector::from(outward_normal);
        if normal.length_squared() < 1.0e-12 {
            return None;
        }
        Some(SharedShape::halfspace(normal.normalize()).into())
    }

    /// Initialize a new collider with a cylindrical shape defined by its half-height
    /// (along the y axis) and its radius.
    #[cfg(feature = "dim3")]
    pub fn cylinder(half_height: Real, radius: Real) -> Self {
        SharedShape::cylinder(half_height, radius).into()
    }

    /// Initialize a new collider with a rounded cylindrical shape defined by its half-height
    /// (along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cylinder(half_height: Real, radius: Real, border_radius: Real) -> Self {
        SharedShape::round_cylinder(half_height, radius, border_radius).into()
    }

    /// Initialize a new collider with a cone shape defined by its half-height
    /// (along the y axis) and its basis radius.
    #[cfg(feature = "dim3")]
    pub fn cone(half_height: Real, radius: Real) -> Self {
        SharedShape::cone(half_height, radius).into()
    }

    /// Initialize a new collider with a rounded cone shape defined by its half-height
    /// (along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cone(half_height: Real, radius: Real, border_radius: Real) -> Self {
        SharedShape::round_cone(half_height, radius, border_radius).into()
    }

    /// Initialize a new collider with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim2")]
    pub fn cuboid(half_x: Real, half_y: Real) -> Self {
        SharedShape::cuboid(half_x, half_y).into()
    }

    /// Initialize a new collider with a round cuboid shape defined by its half-extents
    /// and border radius.
    #[cfg(feature = "dim2")]
    pub fn round_cuboid(half_x: Real, half_y: Real, border_radius: Real) -> Self {
        SharedShape::round_cuboid(half_x, half_y, border_radius).into()
    }

    /// Initialize a new collider with a capsule shape.
    pub fn capsule(start: Vect, end: Vect, radius: Real) -> Self {
        SharedShape::capsule(start, end, radius).into()
    }

    /// Initialize a new collider with a capsule shape aligned with the `x` axis.
    pub fn capsule_x(half_height: Real, radius: Real) -> Self {
        let p = Vector::X * half_height;
        SharedShape::capsule(-p, p, radius).into()
    }

    /// Initialize a new collider with a capsule shape aligned with the `y` axis.
    pub fn capsule_y(half_height: Real, radius: Real) -> Self {
        let p = Vector::Y * half_height;
        SharedShape::capsule(-p, p, radius).into()
    }

    /// Initialize a new collider with a capsule shape aligned with the `z` axis.
    #[cfg(feature = "dim3")]
    pub fn capsule_z(half_height: Real, radius: Real) -> Self {
        let p = Vector::Z * half_height;
        SharedShape::capsule(-p, p, radius).into()
    }

    /// Initialize a new collider with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim3")]
    pub fn cuboid(hx: Real, hy: Real, hz: Real) -> Self {
        SharedShape::cuboid(hx, hy, hz).into()
    }

    /// Initialize a new collider with a round cuboid shape defined by its half-extents
    /// and border radius.
    #[cfg(feature = "dim3")]
    pub fn round_cuboid(half_x: Real, half_y: Real, half_z: Real, border_radius: Real) -> Self {
        SharedShape::round_cuboid(half_x, half_y, half_z, border_radius).into()
    }

    /// Initializes a collider with a segment shape.
    pub fn segment(a: Vect, b: Vect) -> Self {
        SharedShape::segment(a, b).into()
    }

    /// Initializes a collider with a triangle shape.
    pub fn triangle(a: Vect, b: Vect, c: Vect) -> Self {
        SharedShape::triangle(a, b, c).into()
    }

    /// Initializes a collider with a triangle shape with round corners.
    pub fn round_triangle(a: Vect, b: Vect, c: Vect, border_radius: Real) -> Self {
        SharedShape::round_triangle(a, b, c, border_radius).into()
    }

    fn ivec_array_from_point_int_array(points: &[IVect]) -> Vec<IVect> {
        points.to_vec()
    }

    fn vec_array_from_point_float_array(points: &[Vect]) -> Vec<Vect> {
        points.to_vec()
    }

    /// Initializes a shape made of voxels.
    ///
    /// Each voxel has the size `voxel_size` and grid coordinate given by `grid_coords`.
    /// The `primitive_geometry` controls the behavior of collision detection at voxels boundaries.
    ///
    /// For initializing a voxels shape from points in space, see [`Self::voxels_from_points`].
    /// For initializing a voxels shape from a mesh to voxelize, see [`Self::voxelized_mesh`].
    /// For initializing multiple voxels shape from the convex decomposition of a mesh, see
    /// [`Self::voxelized_convex_decomposition`].
    pub fn voxels(voxel_size: Vect, grid_coordinates: &[IVect]) -> Self {
        let shape = Voxels::new(
            voxel_size,
            &Self::ivec_array_from_point_int_array(grid_coordinates),
        );
        SharedShape::new(shape).into()
    }

    /// Initializes a shape made of voxels.
    ///
    /// Each voxel has the size `voxel_size` and contains at least one point from `centers`.
    /// The `primitive_geometry` controls the behavior of collision detection at voxels boundaries.
    pub fn voxels_from_points(voxel_size: Vect, points: &[Vect]) -> Self {
        SharedShape::voxels_from_points(voxel_size, &Self::vec_array_from_point_float_array(points))
            .into()
    }

    /// Initializes a voxels shape obtained from the decomposition of the given trimesh (in 3D)
    /// or polyline (in 2D) into voxelized convex parts.
    pub fn voxelized_mesh(
        vertices: &[Vect],
        indices: &[[u32; DIM]],
        voxel_size: Real,
        fill_mode: FillMode,
    ) -> Self {
        let vertices = Self::vec_array_from_point_float_array(vertices);
        SharedShape::voxelized_mesh(&vertices, indices, voxel_size, fill_mode).into()
    }

    /// Initializes a compound shape obtained from the decomposition of the given trimesh (in 3D)
    /// or polyline (in 2D) into voxelized convex parts.
    pub fn voxelized_convex_decomposition(vertices: &[Vect], indices: &[[u32; DIM]]) -> Vec<Self> {
        Self::voxelized_convex_decomposition_with_params(
            vertices,
            indices,
            &VHACDParameters::default(),
        )
    }

    /// Initializes a compound shape obtained from the decomposition of the given trimesh (in 3D)
    /// or polyline (in 2D) into voxelized convex parts.
    pub fn voxelized_convex_decomposition_with_params(
        vertices: &[Vect],
        indices: &[[u32; DIM]],
        params: &VHACDParameters,
    ) -> Vec<Self> {
        SharedShape::voxelized_convex_decomposition_with_params(
            &Self::vec_array_from_point_float_array(vertices),
            indices,
            params,
        )
        .into_iter()
        .map(|c| c.into())
        .collect()
    }

    /// Initializes a collider with a polyline shape defined by its vertex and index buffers.
    pub fn polyline(vertices: Vec<Vect>, indices: Option<Vec<[u32; 2]>>) -> Self {
        let vertices = vertices.into_iter().collect();
        SharedShape::polyline(vertices, indices).into()
    }

    /// Initializes a collider with a polyline shape defined by its vertex and index buffers, and
    /// flags controlling its optional associated data (orientation, deformability).
    pub fn polyline_with_flags(
        vertices: Vec<Vect>,
        indices: Option<Vec<[u32; 2]>>,
        flags: PolylineFlags,
    ) -> Self {
        SharedShape::new(Polyline::with_flags(vertices, indices, flags)).into()
    }

    /// Initializes a collider with a one-sided (oriented) polyline shape.
    ///
    /// The segments only collide from their outward side, which is on the right of each
    /// segment's direction: a counter-clockwise loop has a solid interior.
    #[cfg(feature = "dim2")]
    pub fn oriented_polyline(vertices: Vec<Vect>, indices: Option<Vec<[u32; 2]>>) -> Self {
        Self::polyline_with_flags(vertices, indices, PolylineFlags::ORIENTED)
    }

    /// Initializes a collider with a triangle mesh shape defined by its vertex and index buffers.
    pub fn trimesh(
        vertices: Vec<Vect>,
        indices: Vec<[u32; 3]>,
    ) -> Result<Self, crate::rapier::prelude::TriMeshBuilderError> {
        let vertices = vertices.into_iter().collect();
        Ok(SharedShape::trimesh(vertices, indices)?.into())
    }

    /// Initializes a collider with a triangle mesh shape defined by its vertex and index buffers, and flags
    /// controlling its pre-processing.
    pub fn trimesh_with_flags(
        vertices: Vec<Vect>,
        indices: Vec<[u32; 3]>,
        flags: TriMeshFlags,
    ) -> Result<Self, crate::rapier::prelude::TriMeshBuilderError> {
        let vertices = vertices.into_iter().collect();
        Ok(SharedShape::trimesh_with_flags(vertices, indices, flags)?.into())
    }

    /// Initializes a collider with a shape converted from the given triangle mesh vertex and
    /// index buffers, following the conversion rule described by `converter`.
    ///
    /// Conversions yielding a shape offset from the mesh origin ([`MeshConverter::Obb`] and
    /// [`MeshConverter::Aabb`]) are wrapped into a single-part compound shape to keep that offset.
    pub fn converted_trimesh(
        vertices: Vec<Vect>,
        indices: Vec<[u32; 3]>,
        converter: MeshConverter,
    ) -> Result<Self, MeshConverterError> {
        let (shape, pose) = converter.convert(vertices, indices)?;
        if pose == Pose::IDENTITY {
            Ok(shape.into())
        } else {
            Ok(SharedShape::compound(vec![(pose, shape)]).into())
        }
    }

    /// Initializes a collider with a Bevy Mesh.
    ///
    /// Returns `None` if the index buffer or vertex buffer of the mesh are in an incompatible format,
    /// or if the shape computation failed.
    #[cfg(all(feature = "dim3", feature = "async-collider"))]
    pub fn from_bevy_mesh(mesh: &Mesh, collider_shape: &ComputedColliderShape) -> Option<Self> {
        let (vtx, idx) = extract_mesh_vertices_indices(mesh)?;

        match collider_shape {
            ComputedColliderShape::TriMesh(flags) => Some(
                SharedShape::trimesh_with_flags(vtx, idx, *flags)
                    .ok()?
                    .into(),
            ),
            ComputedColliderShape::ConvexHull => {
                SharedShape::convex_hull(&vtx).map(|shape| shape.into())
            }
            ComputedColliderShape::ConvexDecomposition(params) => {
                Some(SharedShape::convex_decomposition_with_params(&vtx, &idx, params).into())
            }
            ComputedColliderShape::Voxels {
                voxel_size,
                fill_mode,
            } => Some(Self::voxelized_mesh(&vtx, &idx, *voxel_size, *fill_mode)),
            ComputedColliderShape::Converted(converter) => {
                Self::converted_trimesh(vtx, idx, *converter).ok()
            }
        }
    }

    /// Initializes a collider with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts.
    pub fn convex_decomposition(vertices: &[Vect], indices: &[[u32; DIM]]) -> Self {
        let vertices: Vec<_> = vertices.to_vec();
        SharedShape::convex_decomposition(&vertices, indices).into()
    }

    /// Initializes a collider with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts dilated with round corners.
    pub fn round_convex_decomposition(
        vertices: &[Vect],
        indices: &[[u32; DIM]],
        border_radius: Real,
    ) -> Self {
        let vertices: Vec<_> = vertices.to_vec();
        SharedShape::round_convex_decomposition(&vertices, indices, border_radius).into()
    }

    /// Initializes a collider with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts.
    pub fn convex_decomposition_with_params(
        vertices: &[Vect],
        indices: &[[u32; DIM]],
        params: &VHACDParameters,
    ) -> Self {
        let vertices: Vec<_> = vertices.to_vec();
        SharedShape::convex_decomposition_with_params(&vertices, indices, params).into()
    }

    /// Initializes a collider with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts dilated with round corners.
    pub fn round_convex_decomposition_with_params(
        vertices: &[Vect],
        indices: &[[u32; DIM]],
        params: &VHACDParameters,
        border_radius: Real,
    ) -> Self {
        let vertices: Vec<_> = vertices.to_vec();
        SharedShape::round_convex_decomposition_with_params(
            &vertices,
            indices,
            params,
            border_radius,
        )
        .into()
    }

    /// Initializes a new collider with a 2D convex polygon or 3D convex polyhedron
    /// obtained after computing the convex-hull of the given points.
    pub fn convex_hull(points: &[Vect]) -> Option<Self> {
        let points: Vec<_> = points.to_vec();
        SharedShape::convex_hull(&points).map(Into::into)
    }

    /// Initializes a new collider with a round 2D convex polygon or 3D convex polyhedron
    /// obtained after computing the convex-hull of the given points. The shape is dilated
    /// by a sphere of radius `border_radius`.
    pub fn round_convex_hull(points: &[Vect], border_radius: Real) -> Option<Self> {
        let points: Vec<_> = points.to_vec();
        SharedShape::round_convex_hull(&points, border_radius).map(Into::into)
    }

    /// Creates a new collider that is a convex polygon formed by the
    /// given polyline assumed to be convex (no convex-hull will be automatically
    /// computed).
    #[cfg(feature = "dim2")]
    pub fn convex_polyline(points: Vec<Vect>) -> Option<Self> {
        SharedShape::convex_polyline(points).map(Into::into)
    }

    /// Creates a new collider that is a convex polygon formed by the given polyline assumed
    /// to be convex and counter-clockwise.
    ///
    /// Unlike [`Self::convex_polyline`], no point is removed from the input even if some are
    /// collinear. Returns `None` if `points` contains less than three points.
    #[cfg(feature = "dim2")]
    pub fn convex_polyline_unmodified(points: Vec<Vect>) -> Option<Self> {
        SharedShape::convex_polyline_unmodified(points).map(Into::into)
    }

    /// Creates a new collider that is a round convex polygon formed by the
    /// given polyline assumed to be convex (no convex-hull will be automatically
    /// computed). The polygon shape is dilated by a sphere of radius `border_radius`.
    #[cfg(feature = "dim2")]
    pub fn round_convex_polyline(points: Vec<Vect>, border_radius: Real) -> Option<Self> {
        SharedShape::round_convex_polyline(points, border_radius).map(Into::into)
    }

    /// Creates a new collider that is a convex polyhedron formed by the
    /// given triangle-mesh assumed to be convex (no convex-hull will be automatically
    /// computed).
    #[cfg(feature = "dim3")]
    pub fn convex_mesh(points: Vec<Vect>, indices: &[[u32; 3]]) -> Option<Self> {
        let points = points.into_iter().collect();
        SharedShape::convex_mesh(points, indices).map(Into::into)
    }

    /// Creates a new collider that is a round convex polyhedron formed by the
    /// given triangle-mesh assumed to be convex (no convex-hull will be automatically
    /// computed). The triangle mesh shape is dilated by a sphere of radius `border_radius`.
    #[cfg(feature = "dim3")]
    pub fn round_convex_mesh(
        points: Vec<Vect>,
        indices: &[[u32; 3]],
        border_radius: Real,
    ) -> Option<Self> {
        let points = points.into_iter().collect();
        SharedShape::round_convex_mesh(points, indices, border_radius).map(Into::into)
    }

    /// Initializes a collider with a heightfield shape defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim2")]
    pub fn heightfield(heights: Vec<Real>, scale: Vect) -> Self {
        SharedShape::heightfield(heights, scale).into()
    }

    /// Initializes a collider with a heightfield shape defined by its set of height (in
    /// column-major format) and a scale factor along each coordinate axis.
    #[cfg(feature = "dim3")]
    pub fn heightfield(heights: Vec<Real>, num_rows: usize, num_cols: usize, scale: Vect) -> Self {
        assert_eq!(
            heights.len(),
            num_rows * num_cols,
            "Invalid number of heights provided."
        );
        let heights = rapier::parry::utils::Array2::new(num_rows, num_cols, heights);
        SharedShape::heightfield(heights, scale).into()
    }

    /// Initializes a collider with a heightfield shape defined by its set of height (in
    /// column-major format), a scale factor along each coordinate axis, and flags controlling
    /// its contact behavior.
    #[cfg(feature = "dim3")]
    pub fn heightfield_with_flags(
        heights: Vec<Real>,
        num_rows: usize,
        num_cols: usize,
        scale: Vect,
        flags: HeightFieldFlags,
    ) -> Self {
        assert_eq!(
            heights.len(),
            num_rows * num_cols,
            "Invalid number of heights provided."
        );
        let heights = rapier::parry::utils::Array2::new(num_rows, num_cols, heights);
        SharedShape::heightfield_with_flags(heights, scale, flags).into()
    }

    /// Takes a strongly typed reference of this collider.
    pub fn as_typed_shape(&self) -> ColliderView<'_> {
        self.raw.as_typed_shape().into()
    }

    /// Takes a strongly typed reference of the unscaled version of this collider.
    pub fn as_unscaled_typed_shape(&self) -> ColliderView<'_> {
        self.unscaled.as_typed_shape().into()
    }

    /// Downcast this collider to a ball, if it is one.
    pub fn as_ball(&self) -> Option<BallView<'_>> {
        self.raw.as_ball().map(|s| BallView { raw: s })
    }

    /// Downcast this collider to a cuboid, if it is one.
    pub fn as_cuboid(&self) -> Option<CuboidView<'_>> {
        self.raw.as_cuboid().map(|s| CuboidView { raw: s })
    }

    /// Downcast this collider to a capsule, if it is one.
    pub fn as_capsule(&self) -> Option<CapsuleView<'_>> {
        self.raw.as_capsule().map(|s| CapsuleView { raw: s })
    }

    /// Downcast this collider to a segment, if it is one.
    pub fn as_segment(&self) -> Option<SegmentView<'_>> {
        self.raw.as_segment().map(|s| SegmentView { raw: s })
    }

    /// Downcast this collider to a triangle, if it is one.
    pub fn as_triangle(&self) -> Option<TriangleView<'_>> {
        self.raw.as_triangle().map(|s| TriangleView { raw: s })
    }

    /// Downcast this collider to a voxels, if it is one.
    pub fn as_voxels(&self) -> Option<VoxelsView<'_>> {
        self.raw.as_voxels().map(|s| VoxelsView { raw: s })
    }

    /// Downcast this collider to a triangle mesh, if it is one.
    pub fn as_trimesh(&self) -> Option<TriMeshView<'_>> {
        self.raw.as_trimesh().map(|s| TriMeshView { raw: s })
    }

    /// Downcast this collider to a polyline, if it is one.
    pub fn as_polyline(&self) -> Option<PolylineView<'_>> {
        self.raw.as_polyline().map(|s| PolylineView { raw: s })
    }

    /// Downcast this collider to a half-space, if it is one.
    pub fn as_halfspace(&self) -> Option<HalfSpaceView<'_>> {
        self.raw.as_halfspace().map(|s| HalfSpaceView { raw: s })
    }

    /// Downcast this collider to a heightfield, if it is one.
    pub fn as_heightfield(&self) -> Option<HeightFieldView<'_>> {
        self.raw
            .as_heightfield()
            .map(|s| HeightFieldView { raw: s })
    }

    /// Downcast this collider to a compound shape, if it is one.
    pub fn as_compound(&self) -> Option<CompoundView<'_>> {
        self.raw.as_compound().map(|s| CompoundView { raw: s })
    }

    /// Downcast this collider to a convex polygon, if it is one.
    #[cfg(feature = "dim2")]
    pub fn as_convex_polygon(&self) -> Option<ConvexPolygonView<'_>> {
        self.raw
            .as_convex_polygon()
            .map(|s| ConvexPolygonView { raw: s })
    }

    /// Downcast this collider to a convex polyhedron, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_convex_polyhedron(&self) -> Option<ConvexPolyhedronView<'_>> {
        self.raw
            .as_convex_polyhedron()
            .map(|s| ConvexPolyhedronView { raw: s })
    }

    /// Downcast this collider to a cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cylinder(&self) -> Option<CylinderView<'_>> {
        self.raw.as_cylinder().map(|s| CylinderView { raw: s })
    }

    /// Downcast this collider to a cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cone(&self) -> Option<ConeView<'_>> {
        self.raw.as_cone().map(|s| ConeView { raw: s })
    }

    /// Downcast this collider to a mutable ball, if it is one.
    pub fn as_ball_mut(&mut self) -> Option<BallViewMut<'_>> {
        self.raw
            .make_mut()
            .as_ball_mut()
            .map(|s| BallViewMut { raw: s })
    }

    /// Downcast this collider to a mutable cuboid, if it is one.
    pub fn as_cuboid_mut(&mut self) -> Option<CuboidViewMut<'_>> {
        self.raw
            .make_mut()
            .as_cuboid_mut()
            .map(|s| CuboidViewMut { raw: s })
    }

    /// Downcast this collider to a mutable capsule, if it is one.
    pub fn as_capsule_mut(&mut self) -> Option<CapsuleViewMut<'_>> {
        self.raw
            .make_mut()
            .as_capsule_mut()
            .map(|s| CapsuleViewMut { raw: s })
    }

    /// Downcast this collider to a mutable segment, if it is one.
    pub fn as_segment_mut(&mut self) -> Option<SegmentViewMut<'_>> {
        self.raw
            .make_mut()
            .as_segment_mut()
            .map(|s| SegmentViewMut { raw: s })
    }

    /// Downcast this collider to a mutable triangle, if it is one.
    pub fn as_triangle_mut(&mut self) -> Option<TriangleViewMut<'_>> {
        self.raw
            .make_mut()
            .as_triangle_mut()
            .map(|s| TriangleViewMut { raw: s })
    }

    /// Downcast this collider to a mutable voxels, if it is one.
    pub fn as_voxels_mut(&mut self) -> Option<VoxelsViewMut<'_>> {
        self.raw
            .make_mut()
            .as_voxels_mut()
            .map(|s| VoxelsViewMut { raw: s })
    }

    /// Downcast this collider to a mutable triangle mesh, if it is one.
    pub fn as_trimesh_mut(&mut self) -> Option<TriMeshViewMut<'_>> {
        self.raw
            .make_mut()
            .as_trimesh_mut()
            .map(|s| TriMeshViewMut { raw: s })
    }

    /// Downcast this collider to a mutable polyline, if it is one.
    pub fn as_polyline_mut(&mut self) -> Option<PolylineViewMut<'_>> {
        self.raw
            .make_mut()
            .as_polyline_mut()
            .map(|s| PolylineViewMut { raw: s })
    }

    /// Downcast this collider to a mutable half-space, if it is one.
    pub fn as_halfspace_mut(&mut self) -> Option<HalfSpaceViewMut<'_>> {
        self.raw
            .make_mut()
            .as_halfspace_mut()
            .map(|s| HalfSpaceViewMut { raw: s })
    }

    /// Downcast this collider to a mutable heightfield, if it is one.
    pub fn as_heightfield_mut(&mut self) -> Option<HeightFieldViewMut<'_>> {
        self.raw
            .make_mut()
            .as_heightfield_mut()
            .map(|s| HeightFieldViewMut { raw: s })
    }

    /// Downcast this collider to a mutable compound shape, if it is one.
    ///
    /// Like the other mutable downcasts, this clones the shape first if it is shared with
    /// other colliders. The parts of the compound remain shared.
    pub fn as_compound_mut(&mut self) -> Option<CompoundViewMut<'_>> {
        self.raw
            .make_mut()
            .as_compound_mut()
            .map(|s| CompoundViewMut { raw: s })
    }

    // /// Downcast this collider to a mutable convex polygon, if it is one.
    // #[cfg(feature = "dim2")]
    // pub fn as_convex_polygon_mut(&mut self) -> Option<ConvexPolygonViewMut> {
    //     self.raw.make_mut()
    //         .as_convex_polygon_mut()
    //         .map(|s| ConvexPolygonViewMut { raw: s })
    // }

    // /// Downcast this collider to a mutable convex polyhedron, if it is one.
    // #[cfg(feature = "dim3")]
    // pub fn as_convex_polyhedron_mut(&mut self) -> Option<ConvexPolyhedronViewMut> {
    //     self.raw.make_mut()
    //         .as_convex_polyhedron_mut()
    //         .map(|s| ConvexPolyhedronViewMut { raw: s })
    // }

    /// Downcast this collider to a mutable cylinder, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cylinder_mut(&mut self) -> Option<CylinderViewMut<'_>> {
        self.raw
            .make_mut()
            .as_cylinder_mut()
            .map(|s| CylinderViewMut { raw: s })
    }

    /// Downcast this collider to a mutable cone, if it is one.
    #[cfg(feature = "dim3")]
    pub fn as_cone_mut(&mut self) -> Option<ConeViewMut<'_>> {
        self.raw
            .make_mut()
            .as_cone_mut()
            .map(|s| ConeViewMut { raw: s })
    }

    /// Set the scaling factor of this shape.
    ///
    /// If the scaling factor is non-uniform, and the scaled shape can’t be
    /// represented as a supported smooth shape (for example scalling a Ball
    /// with a non-uniform scale results in an ellipse which isn’t supported),
    /// the shape is approximated by a convex polygon/convex polyhedron using
    /// `num_subdivisions` subdivisions.
    pub fn set_scale(&mut self, scale: Vect, num_subdivisions: u32) {
        let scale = get_snapped_scale(scale);

        if scale == self.scale {
            // Nothing to do.
            return;
        }

        if scale == Vect::ONE {
            // Trivial case.
            self.raw = self.unscaled.clone();
            self.scale = Vect::ONE;
            return;
        }

        if let Some(scaled) = self
            .as_unscaled_typed_shape()
            .raw_scale_by(scale, num_subdivisions)
        {
            self.raw = scaled;
            self.scale = scale;
        } else {
            log::error!("Failed to create the scaled convex hull geometry.");
        }
    }

    /// Projects a point on `self`, unless the projection lies further than the given max distance.
    ///
    /// The point is assumed to be expressed in the local-space of `self`.
    pub fn project_local_point_with_max_dist(
        &self,
        point: Vect,
        solid: bool,
        max_dist: Real,
    ) -> Option<PointProjection> {
        self.raw
            .project_local_point_with_max_dist(point, solid, max_dist)
            .map(Into::into)
    }

    /// Projects a point on `self` transformed by `m`, unless the projection lies further than the given max distance.
    pub fn project_point_with_max_dist(
        &self,
        translation: Vect,
        rotation: Rot,
        point: Vect,
        solid: bool,
        max_dist: Real,
    ) -> Option<PointProjection> {
        let pos = crate::utils::pose_from(translation, rotation);
        self.raw
            .project_point_with_max_dist(&pos, point, solid, max_dist)
            .map(Into::into)
    }

    /// Projects a point on `self`.
    ///
    /// The point is assumed to be expressed in the local-space of `self`.
    pub fn project_local_point(&self, point: Vect, solid: bool) -> PointProjection {
        self.raw.project_local_point(point, solid).into()
    }

    /// Projects a point on the boundary of `self` and returns the id of the
    /// feature the point was projected on.
    pub fn project_local_point_and_get_feature(&self, point: Vect) -> (PointProjection, FeatureId) {
        let (proj, feat) = self.raw.project_local_point_and_get_feature(point);
        (proj.into(), feat)
    }

    /// Computes the minimal distance between a point and `self`.
    pub fn distance_to_local_point(&self, point: Vect, solid: bool) -> Real {
        self.raw.distance_to_local_point(point, solid)
    }

    /// Tests if the given point is inside of `self`.
    pub fn contains_local_point(&self, point: Vect) -> bool {
        self.raw.contains_local_point(point)
    }

    /// Projects a point on `self` transformed by `m`.
    pub fn project_point(
        &self,
        translation: Vect,
        rotation: Rot,
        point: Vect,
        solid: bool,
    ) -> PointProjection {
        let pos = crate::utils::pose_from(translation, rotation);
        self.raw.project_point(&pos, point, solid).into()
    }

    /// Computes the minimal distance between a point and `self` transformed by `m`.
    #[inline]
    pub fn distance_to_point(
        &self,
        translation: Vect,
        rotation: Rot,
        point: Vect,
        solid: bool,
    ) -> Real {
        let pos = crate::utils::pose_from(translation, rotation);
        self.raw.distance_to_point(&pos, point, solid)
    }

    /// Projects a point on the boundary of `self` transformed by `m` and returns the id of the
    /// feature the point was projected on.
    pub fn project_point_and_get_feature(
        &self,
        translation: Vect,
        rotation: Rot,
        point: Vect,
    ) -> (PointProjection, FeatureId) {
        let pos = crate::utils::pose_from(translation, rotation);
        let (proj, feat) = self.raw.project_point_and_get_feature(&pos, point);
        (proj.into(), feat)
    }

    /// Tests if the given point is inside of `self` transformed by `m`.
    pub fn contains_point(&self, translation: Vect, rotation: Rot, point: Vect) -> bool {
        let pos = crate::utils::pose_from(translation, rotation);
        self.raw.contains_point(&pos, point)
    }

    /// Computes the time of impact between this transform shape and a ray.
    pub fn cast_local_ray(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<Real> {
        let ray = Ray::new(ray_origin, ray_dir);
        self.raw.cast_local_ray(&ray, max_time_of_impact, solid)
    }

    /// Computes the time of impact, and normal between this transformed shape and a ray.
    pub fn cast_local_ray_and_get_normal(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<RayIntersection> {
        let ray = Ray::new(ray_origin, ray_dir);
        self.raw
            .cast_local_ray_and_get_normal(&ray, max_time_of_impact, solid)
            .map(|inter| RayIntersection::from_rapier(inter, ray_origin, ray_dir))
    }

    /// Tests whether a ray intersects this transformed shape.
    pub fn intersects_local_ray(
        &self,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
    ) -> bool {
        let ray = Ray::new(ray_origin, ray_dir);
        self.raw.intersects_local_ray(&ray, max_time_of_impact)
    }

    /// Computes the time of impact between this transform shape and a ray.
    pub fn cast_ray(
        &self,
        translation: Vect,
        rotation: Rot,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<Real> {
        let pos = crate::utils::pose_from(translation, rotation);
        let ray = Ray::new(ray_origin, ray_dir);
        self.raw.cast_ray(&pos, &ray, max_time_of_impact, solid)
    }

    /// Computes the time of impact, and normal between this transformed shape and a ray.
    pub fn cast_ray_and_get_normal(
        &self,
        translation: Vect,
        rotation: Rot,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<RayIntersection> {
        let pos = crate::utils::pose_from(translation, rotation);
        let ray = Ray::new(ray_origin, ray_dir);
        self.raw
            .cast_ray_and_get_normal(&pos, &ray, max_time_of_impact, solid)
            .map(|inter| RayIntersection::from_rapier(inter, ray_origin, ray_dir))
    }

    /// Tests whether a ray intersects this transformed shape.
    pub fn intersects_ray(
        &self,
        translation: Vect,
        rotation: Rot,
        ray_origin: Vect,
        ray_dir: Vect,
        max_time_of_impact: Real,
    ) -> bool {
        let pos = crate::utils::pose_from(translation, rotation);
        let ray = Ray::new(ray_origin, ray_dir);
        self.raw.intersects_ray(&pos, &ray, max_time_of_impact)
    }

    /// Computes the axis-aligned bounding box of `self` transformed by the given translation
    /// and rotation.
    pub fn aabb(&self, translation: Vect, rotation: Rot) -> BevyAabb {
        let pos = crate::utils::pose_from(translation, rotation);
        let aabb = self.raw.compute_aabb(&pos);
        BevyAabb::new(aabb.center(), aabb.half_extents())
    }

    /// Computes the axis-aligned bounding box of `self` in its local-space.
    pub fn local_aabb(&self) -> BevyAabb {
        let aabb = self.raw.compute_local_aabb();
        BevyAabb::new(aabb.center(), aabb.half_extents())
    }

    /// Computes a bounding sphere (or circle in 2D) of `self` transformed by the given
    /// translation and rotation.
    pub fn bounding_sphere(&self, translation: Vect, rotation: Rot) -> BevyBoundingSphere {
        let pos = crate::utils::pose_from(translation, rotation);
        let sphere = self.raw.compute_bounding_sphere(&pos);
        BevyBoundingSphere::new(sphere.center, sphere.radius)
    }

    /// Computes a bounding sphere (or circle in 2D) of `self` in its local-space.
    pub fn local_bounding_sphere(&self) -> BevyBoundingSphere {
        let sphere = self.raw.compute_local_bounding_sphere();
        BevyBoundingSphere::new(sphere.center, sphere.radius)
    }

    /// Computes the mass properties of `self`, assuming it has the given uniform density.
    pub fn mass_properties(&self, density: Real) -> MassProperties {
        MassProperties::from_rapier(self.raw.mass_properties(density))
    }

    /// Computes the contact between `self` and `other`, each transformed by their respective
    /// translation and rotation.
    ///
    /// Returns `Ok(None)` if the colliders are separated by more than `prediction`. The contact
    /// points and normals are expressed in world-space.
    #[allow(clippy::too_many_arguments)]
    pub fn contact(
        &self,
        translation: Vect,
        rotation: Rot,
        other: &Collider,
        other_translation: Vect,
        other_rotation: Rot,
        prediction: Real,
    ) -> Result<Option<ShapeContact>, Unsupported> {
        let pos1 = crate::utils::pose_from(translation, rotation);
        let pos2 = crate::utils::pose_from(other_translation, other_rotation);
        Ok(
            query::contact(&pos1, &*self.raw, &pos2, &*other.raw, prediction)?
                .map(ShapeContact::from_rapier),
        )
    }

    /// Computes the minimal distance between `self` and `other`, each transformed by their
    /// respective translation and rotation.
    ///
    /// Returns `0.0` if the colliders are intersecting.
    pub fn distance(
        &self,
        translation: Vect,
        rotation: Rot,
        other: &Collider,
        other_translation: Vect,
        other_rotation: Rot,
    ) -> Result<Real, Unsupported> {
        let pos1 = crate::utils::pose_from(translation, rotation);
        let pos2 = crate::utils::pose_from(other_translation, other_rotation);
        Ok(query::distance(&pos1, &*self.raw, &pos2, &*other.raw)?.distance)
    }

    /// Tests whether `self` and `other`, each transformed by their respective translation and
    /// rotation, are intersecting.
    pub fn intersection_test(
        &self,
        translation: Vect,
        rotation: Rot,
        other: &Collider,
        other_translation: Vect,
        other_rotation: Rot,
    ) -> Result<bool, Unsupported> {
        let pos1 = crate::utils::pose_from(translation, rotation);
        let pos2 = crate::utils::pose_from(other_translation, other_rotation);
        Ok(query::intersection_test(&pos1, &*self.raw, &pos2, &*other.raw)?.intersecting)
    }

    /// Computes the closest points between `self` and `other`, each transformed by their
    /// respective translation and rotation.
    ///
    /// Returns [`ShapeClosestPoints::Disjoint`] if the colliders are separated by more than
    /// `max_dist`. The closest points are expressed in world-space.
    #[allow(clippy::too_many_arguments)]
    pub fn closest_points(
        &self,
        translation: Vect,
        rotation: Rot,
        other: &Collider,
        other_translation: Vect,
        other_rotation: Rot,
        max_dist: Real,
    ) -> Result<ShapeClosestPoints, Unsupported> {
        let pos1 = crate::utils::pose_from(translation, rotation);
        let pos2 = crate::utils::pose_from(other_translation, other_rotation);
        Ok(ShapeClosestPoints::from_rapier(query::closest_points(
            &pos1,
            &*self.raw,
            &pos2,
            &*other.raw,
            max_dist,
        )?))
    }

    /// Computes the first time of impact between `self` and `other`, each starting at their
    /// respective translation and rotation, and moving with a constant linear velocity.
    ///
    /// Returns `Ok(None)` if they do not hit within [`ShapeCastOptions::max_time_of_impact`].
    /// The witness points and normals of the result are expressed in the local-space of
    /// each collider.
    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape(
        &self,
        translation: Vect,
        rotation: Rot,
        velocity: Vect,
        other: &Collider,
        other_translation: Vect,
        other_rotation: Rot,
        other_velocity: Vect,
        options: ShapeCastOptions,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        let pos1 = crate::utils::pose_from(translation, rotation);
        let pos2 = crate::utils::pose_from(other_translation, other_rotation);
        Ok(query::cast_shapes(
            &pos1,
            velocity,
            &*self.raw,
            &pos2,
            other_velocity,
            &*other.raw,
            options,
        )?
        .map(|hit| ShapeCastHit::from_rapier(hit, options.compute_impact_geometry_on_penetration)))
    }

    /// Computes the first time of impact between `self` and `other`, each following a
    /// nonlinear rigid motion (with both linear and angular velocities).
    ///
    /// Only the time interval `[start_time, end_time]` is considered. If
    /// `stop_at_penetration` is `true` and the colliders are initially penetrating, a hit
    /// at `start_time` is returned. The witness points and normals of the result are
    /// expressed in the local-space of each collider.
    pub fn cast_shape_nonlinear(
        &self,
        motion: &NonlinearMotion,
        other: &Collider,
        other_motion: &NonlinearMotion,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        Ok(query::cast_shapes_nonlinear(
            &motion.into_rapier(),
            &*self.raw,
            &other_motion.into_rapier(),
            &*other.raw,
            start_time,
            end_time,
            stop_at_penetration,
        )?
        .map(|hit| ShapeCastHit::from_rapier(hit, false)))
    }
}

impl Default for Collider {
    fn default() -> Self {
        Self::ball(0.5)
    }
}

#[cfg(all(feature = "dim3", feature = "async-collider"))]
#[allow(clippy::type_complexity)]
fn extract_mesh_vertices_indices(mesh: &Mesh) -> Option<(Vec<Vector>, Vec<[u32; 3]>)> {
    let vertices = mesh.attribute(Mesh::ATTRIBUTE_POSITION)?;
    let indices = mesh.indices()?;

    let vtx: Vec<Vector> = match vertices {
        VertexAttributeValues::Float32(vtx) => Some(
            vtx.chunks(3)
                .map(|v| Vector::new(v[0] as Real, v[1] as Real, v[2] as Real))
                .collect(),
        ),
        VertexAttributeValues::Float32x3(vtx) => Some(
            vtx.iter()
                .map(|v| Vector::new(v[0] as Real, v[1] as Real, v[2] as Real))
                .collect(),
        ),
        _ => None,
    }?;

    let idx = match indices {
        Indices::U16(idx) => idx
            .as_chunks::<3>()
            .0
            .iter()
            .map(|i| i.map(u32::from))
            .collect(),
        Indices::U32(idx) => idx.as_chunks::<3>().0.to_vec(),
    };

    Some((vtx, idx))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::ShapeCastStatus;

    #[cfg(feature = "dim2")]
    const ROT_ID: Rot = 0.0;
    #[cfg(feature = "dim3")]
    const ROT_ID: Rot = Rot::IDENTITY;

    fn assert_approx_eq(a: Real, b: Real) {
        assert!((a - b).abs() < 1.0e-4, "{a} != {b}");
    }

    fn line_vertices() -> Vec<Vect> {
        vec![Vect::ZERO, Vect::X, Vect::X * 2.0]
    }

    #[test]
    fn polyline_with_flags_constructor() {
        let collider =
            Collider::polyline_with_flags(line_vertices(), None, PolylineFlags::DEFORMABLE);
        let polyline = collider.as_polyline().unwrap();
        assert_eq!(polyline.flags(), PolylineFlags::DEFORMABLE);
        assert_eq!(polyline.num_segments(), 2);
    }

    #[test]
    #[cfg(feature = "dim2")]
    fn oriented_polyline_constructor() {
        let collider = Collider::oriented_polyline(line_vertices(), None);
        let polyline = collider.as_polyline().unwrap();
        assert!(polyline.flags().contains(PolylineFlags::ORIENTED));
        assert_eq!(polyline.pseudo_normals().unwrap().len(), 3);
    }

    #[test]
    #[cfg(feature = "dim2")]
    fn convex_polyline_unmodified_keeps_collinear_points() {
        let points = vec![
            Vect::new(0.0, 0.0),
            Vect::new(1.0, 0.0),
            Vect::new(2.0, 0.0),
            Vect::new(2.0, 2.0),
            Vect::new(0.0, 2.0),
        ];
        let unmodified = Collider::convex_polyline_unmodified(points.clone()).unwrap();
        assert_eq!(unmodified.as_convex_polygon().unwrap().points().len(), 5);
        let simplified = Collider::convex_polyline(points).unwrap();
        assert_eq!(simplified.as_convex_polygon().unwrap().points().len(), 4);
    }

    #[test]
    #[cfg(feature = "dim3")]
    fn heightfield_with_flags_constructor() {
        let collider = Collider::heightfield_with_flags(
            vec![0.0; 9],
            3,
            3,
            Vect::ONE,
            HeightFieldFlags::FIX_INTERNAL_EDGES,
        );
        let heightfield = collider.as_heightfield().unwrap();
        assert_eq!(heightfield.flags(), HeightFieldFlags::FIX_INTERNAL_EDGES);
    }

    #[test]
    fn compound_with_flags_constructor_and_mutation() {
        let parts = vec![
            (Vect::ZERO, ROT_ID, Collider::ball(0.5)),
            (Vect::X, ROT_ID, Collider::ball(0.5)),
        ];
        let mut collider =
            Collider::compound_with_flags(parts, CompoundFlags::FIX_INTERNAL_EDGES, None);
        assert_eq!(
            collider.as_compound().unwrap().flags(),
            CompoundFlags::FIX_INTERNAL_EDGES
        );
        assert_eq!(collider.as_compound().unwrap().shapes().len(), 2);

        let shared = collider.clone();
        collider
            .as_compound_mut()
            .unwrap()
            .set_flags(CompoundFlags::empty(), None);
        assert_eq!(
            collider.as_compound().unwrap().flags(),
            CompoundFlags::empty()
        );
        // The mutation must not leak into the clone sharing the same shape.
        assert_eq!(
            shared.as_compound().unwrap().flags(),
            CompoundFlags::FIX_INTERNAL_EDGES
        );
    }

    #[test]
    fn converted_trimesh_constructor() {
        let offset = Vect::ONE * 3.0;
        let (vertices, indices) = cuboid_mesh_for_tests(offset);

        let aabb =
            Collider::converted_trimesh(vertices.clone(), indices.clone(), MeshConverter::Aabb)
                .unwrap();
        // The offset of the AABB is preserved by a single-part compound.
        let compound = aabb.as_compound().unwrap();
        let (translation, _, part) = compound.shapes().next().unwrap();
        assert!((translation - offset).length() < 1.0e-5);
        assert!(matches!(part, ColliderView::Cuboid(_)));

        let hull = Collider::converted_trimesh(
            vertices.clone(),
            indices.clone(),
            MeshConverter::ConvexHull,
        )
        .unwrap();
        #[cfg(feature = "dim2")]
        assert!(hull.as_convex_polygon().is_some());
        #[cfg(feature = "dim3")]
        assert!(hull.as_convex_polyhedron().is_some());

        let trimesh =
            Collider::converted_trimesh(vertices, indices, MeshConverter::TriMesh).unwrap();
        assert!(trimesh.as_trimesh().is_some());
    }

    #[test]
    fn trimesh_view_set_vertices() {
        let (vertices, indices) = cuboid_mesh_for_tests(Vect::ZERO);
        let mut collider =
            Collider::trimesh_with_flags(vertices.clone(), indices, TriMeshFlags::DEFORMABLE)
                .unwrap();
        assert_eq!(
            collider.as_trimesh().unwrap().flags(),
            TriMeshFlags::DEFORMABLE
        );

        let shift = Vect::X * 10.0;
        let shifted: Vec<_> = vertices.iter().map(|v| *v + shift).collect();
        collider.as_trimesh_mut().unwrap().set_vertices(&shifted);
        let aabb = collider.local_aabb();
        assert_approx_eq(aabb.min.x, 9.5);
        assert_approx_eq(aabb.max.x, 10.5);

        collider
            .as_trimesh_mut()
            .unwrap()
            .update_vertices(|vtx| vtx.iter_mut().for_each(|v| *v -= shift));
        assert_approx_eq(collider.local_aabb().max.x, 0.5);
    }

    #[test]
    fn polyline_view_set_vertices() {
        let mut collider =
            Collider::polyline_with_flags(line_vertices(), None, PolylineFlags::DEFORMABLE);
        let lifted = vec![Vect::ZERO, Vect::X + Vect::Y, Vect::X * 2.0];
        collider.as_polyline_mut().unwrap().set_vertices(&lifted);
        assert_approx_eq(collider.local_aabb().max.y, 1.0);
    }

    #[test]
    fn voxels_view_set_voxel_size() {
        let mut collider = Collider::voxels(Vect::ONE, &[IVect::ZERO, IVect::X]);
        let voxels = collider.as_voxels().unwrap();
        assert_eq!(voxels.voxel_size(), Vect::ONE);
        assert_eq!(voxels.voxel_center(IVect::X), Vect::X + Vect::splat(0.5));
        assert_eq!(
            voxels
                .cropped(IVect::ZERO, IVect::ZERO)
                .unwrap()
                .voxels()
                .count(),
            1
        );

        collider
            .as_voxels_mut()
            .unwrap()
            .set_voxel_size(Vect::splat(2.0));
        let voxels = collider.as_voxels().unwrap();
        assert_eq!(voxels.voxel_size(), Vect::splat(2.0));
        assert_eq!(voxels.voxel_center(IVect::X), Vect::X * 2.0 + Vect::ONE);
    }

    #[test]
    #[cfg(feature = "dim3")]
    fn convex_polyhedron_topology() {
        let (vertices, _) = cuboid_mesh_for_tests(Vect::ZERO);
        let collider = Collider::convex_hull(&vertices).unwrap();
        let polyhedron = collider.as_convex_polyhedron().unwrap();
        assert_eq!(polyhedron.num_vertices(), 8);
        assert_eq!(polyhedron.edges().count(), 12);
        assert_eq!(polyhedron.num_faces(), 6);
        for i in 0..polyhedron.num_faces() {
            assert_eq!(polyhedron.face_vertices(i).len(), 4);
            assert_eq!(polyhedron.face_edges(i).len(), 4);
            assert_approx_eq(polyhedron.face_normal(i).length(), 1.0);
        }
        for i in 0..polyhedron.num_vertices() {
            assert_eq!(polyhedron.vertex_adjacent_faces(i).len(), 3);
        }
        let points: Vec<_> = polyhedron.points().collect();
        for edge in polyhedron.edges() {
            let [a, b] = polyhedron.edge_vertices(edge as usize);
            let dir = (points[b as usize] - points[a as usize]).normalize();
            assert!((dir - polyhedron.edge_direction(edge as usize)).length() < 1.0e-5);
            let [f1, f2] = polyhedron.edge_faces(edge as usize);
            assert!(polyhedron.face_edges(f1 as usize).contains(&edge));
            assert!(polyhedron.face_edges(f2 as usize).contains(&edge));
        }
    }

    #[test]
    fn custom_shape_view_does_not_panic() {
        let collider = Collider::ball(1.0);
        let view = ColliderView::Custom(&*collider.raw);
        assert!(matches!(
            view.as_typed_shape(),
            rapier::parry::shape::TypedShape::Custom(_)
        ));
        let shared = view.to_shared_shape();
        assert!(shared.as_ball().is_some());
        assert!(view.raw_scale_by(Vect::splat(2.0), 10).is_some());
    }

    #[test]
    fn bounding_volumes_and_mass_properties() {
        let collider = Collider::ball(1.0);
        let translation = Vect::X * 2.0;

        let aabb = collider.aabb(translation, ROT_ID);
        assert_approx_eq(aabb.min.x, 1.0);
        assert_approx_eq(aabb.max.x, 3.0);
        assert_approx_eq(aabb.max.y, 1.0);

        let sphere = collider.bounding_sphere(translation, ROT_ID);
        assert_approx_eq(sphere.center.x, 2.0);
        assert_approx_eq(sphere.radius(), 1.0);

        let mprops = collider.mass_properties(2.0);
        #[cfg(feature = "dim2")]
        assert_approx_eq(mprops.mass, 2.0 * core::f32::consts::PI);
        #[cfg(feature = "dim3")]
        assert_approx_eq(mprops.mass, 2.0 * 4.0 / 3.0 * core::f32::consts::PI);
    }

    #[test]
    fn pairwise_queries() {
        let ball = Collider::ball(1.0);
        let pos1 = Vect::ZERO;
        let pos2 = Vect::X * 3.0;

        let distance = ball.distance(pos1, ROT_ID, &ball, pos2, ROT_ID).unwrap();
        assert_approx_eq(distance, 1.0);

        assert!(!ball
            .intersection_test(pos1, ROT_ID, &ball, pos2, ROT_ID)
            .unwrap());
        assert!(ball
            .intersection_test(pos1, ROT_ID, &ball, Vect::X, ROT_ID)
            .unwrap());

        let contact = ball
            .contact(pos1, ROT_ID, &ball, pos2, ROT_ID, 2.0)
            .unwrap()
            .unwrap();
        assert_approx_eq(contact.distance, 1.0);
        assert!((contact.normal1 - Vect::X).length() < 1.0e-5);
        assert!((contact.point1 - Vect::X).length() < 1.0e-5);
        assert!((contact.point2 - Vect::X * 2.0).length() < 1.0e-5);
        assert!(ball
            .contact(pos1, ROT_ID, &ball, pos2, ROT_ID, 0.5)
            .unwrap()
            .is_none());

        match ball
            .closest_points(pos1, ROT_ID, &ball, pos2, ROT_ID, 2.0)
            .unwrap()
        {
            ShapeClosestPoints::WithinMargin(p1, p2) => {
                assert!((p1 - Vect::X).length() < 1.0e-5);
                assert!((p2 - Vect::X * 2.0).length() < 1.0e-5);
            }
            other => panic!("Unexpected closest points: {other:?}"),
        }
        assert_eq!(
            ball.closest_points(pos1, ROT_ID, &ball, pos2, ROT_ID, 0.5)
                .unwrap(),
            ShapeClosestPoints::Disjoint
        );
        assert_eq!(
            ball.closest_points(pos1, ROT_ID, &ball, Vect::X, ROT_ID, 0.5)
                .unwrap(),
            ShapeClosestPoints::Intersecting
        );

        let hit = ball
            .cast_shape(
                pos1,
                ROT_ID,
                Vect::X,
                &ball,
                pos2,
                ROT_ID,
                Vect::ZERO,
                ShapeCastOptions::with_max_time_of_impact(10.0),
            )
            .unwrap()
            .unwrap();
        assert_approx_eq(hit.time_of_impact, 1.0);
        assert_eq!(hit.status, ShapeCastStatus::Converged);
        let details = hit.details.unwrap();
        assert!((details.normal1 - Vect::X).length() < 1.0e-4);

        let mut motion1 = NonlinearMotion::constant_position(pos1, ROT_ID);
        motion1.linear_velocity = Vect::X;
        let motion2 = NonlinearMotion::constant_position(pos2, ROT_ID);
        let hit = ball
            .cast_shape_nonlinear(&motion1, &ball, &motion2, 0.0, 10.0, true)
            .unwrap()
            .unwrap();
        assert!((hit.time_of_impact - 1.0).abs() < 1.0e-2);
    }

    /// The vertex and index buffers of a unit cube (3D) or square (2D) mesh centered at `offset`.
    fn cuboid_mesh_for_tests(offset: Vect) -> (Vec<Vect>, Vec<[u32; 3]>) {
        let (vtx, idx) = rapier::prelude::Cuboid::new(Vect::splat(0.5)).to_trimesh();
        (vtx.into_iter().map(|v| v + offset).collect(), idx)
    }
}
