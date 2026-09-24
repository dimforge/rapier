//! Module for utilities to convert from a [`rapier::prelude::Shape`] to a [`bevy::prelude::Mesh`].

use super::Collider;
use crate::rapier::prelude::Ball;
#[cfg(feature = "dim3")]
use crate::rapier::prelude::{Cone, Cylinder};
#[cfg(feature = "dim2")]
use bevy::mesh::{Capsule2dMeshBuilder, CircleMeshBuilder};
#[cfg(feature = "dim3")]
use bevy::mesh::{
    Capsule3dMeshBuilder, ConeMeshBuilder, CylinderMeshBuilder, PlaneMeshBuilder, SphereMeshBuilder,
};
use bevy::{
    asset::RenderAssetUsages,
    mesh::{Indices, PrimitiveTopology},
    prelude::{Mesh, MeshBuilder},
};

use crate::math::{Real, Vect};
use rapier::prelude::{Capsule, TypedShape};

/// Half-size of the finite square (in 3D) or segment (in 2D) generated to represent the
/// boundary of an (infinite) half-space by [`typed_shape_to_mesh`].
pub const HALF_SPACE_MESH_HALF_SIZE: Real = 1000.0;

/// Base number of subdivisions of the circle (in 2D) or sphere (in 3D) approximating the
/// rounded border of round shapes in [`typed_shape_to_mesh`].
pub const ROUND_BORDER_SUBDIVISIONS: u32 = 8;

/// Number of subdivisions of the circular parts of round cylinders and round cones in
/// [`typed_shape_to_mesh`].
#[cfg(feature = "dim3")]
pub const ROUND_SHAPE_CIRCLE_SUBDIVISIONS: u32 = 16;

fn position_attribute(vertices: &[Vect]) -> Vec<[f32; 3]> {
    #[cfg(feature = "dim2")]
    return vertices.iter().map(|pt| [pt.x, pt.y, 0.0]).collect();
    #[cfg(feature = "dim3")]
    return vertices.iter().map(|pt| [pt.x, pt.y, pt.z]).collect();
}

/// Builds a [`PrimitiveTopology::TriangleList`] mesh from a vertex and index buffer.
fn triangle_list_mesh(vertices: &[Vect], indices: impl IntoIterator<Item = [u32; 3]>) -> Mesh {
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_indices(Indices::U32(indices.into_iter().flatten().collect()))
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, position_attribute(vertices))
}

/// Builds a [`PrimitiveTopology::LineList`] mesh from a vertex and index buffer.
fn line_list_mesh(vertices: &[Vect], indices: impl IntoIterator<Item = [u32; 2]>) -> Mesh {
    Mesh::new(PrimitiveTopology::LineList, RenderAssetUsages::default())
        .with_inserted_indices(Indices::U32(indices.into_iter().flatten().collect()))
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, position_attribute(vertices))
}

/// Builds a [`PrimitiveTopology::LineList`] mesh from a list of disjoint segments.
#[cfg(feature = "dim2")]
fn segments_mesh(segments: impl IntoIterator<Item = (Vect, Vect)>) -> Mesh {
    let mut vertices = vec![];
    for (a, b) in segments {
        vertices.push(a);
        vertices.push(b);
    }
    let indices = (0..vertices.len() as u32 / 2).map(|i| [i * 2, i * 2 + 1]);
    line_list_mesh(&vertices, indices)
}

/// Triangulates a convex polygon with a triangle fan.
#[cfg(feature = "dim2")]
fn convex_polygon_mesh(vertices: &[Vect]) -> Option<Mesh> {
    if vertices.len() < 3 {
        return None;
    }
    let indices = (1..vertices.len() as u32 - 1).map(|i| [0, i, i + 1]);
    Some(triangle_list_mesh(vertices, indices))
}

/// Meshes the Minkowski sum of the convex hull of `inner_points` with a ball of radius
/// `border_radius`.
///
/// The ball is approximated by a polygon (2D) or polyhedron (3D), so the result is the exact
/// convex hull of the pairwise sums of the inner points and the ball approximation vertices.
fn round_convex_mesh(inner_points: &[Vect], border_radius: Real) -> Option<Mesh> {
    let ball = Ball::new(border_radius);
    #[cfg(feature = "dim2")]
    let ball_points = ball.to_polyline(ROUND_BORDER_SUBDIVISIONS * 2);
    #[cfg(feature = "dim3")]
    let ball_points = ball
        .to_trimesh(ROUND_BORDER_SUBDIVISIONS, ROUND_BORDER_SUBDIVISIONS / 2)
        .0;

    let points: Vec<Vect> = inner_points
        .iter()
        .flat_map(|pt| ball_points.iter().map(move |b| *pt + *b))
        .collect();

    #[cfg(feature = "dim2")]
    {
        let hull = rapier::parry::transformation::convex_hull(&points);
        convex_polygon_mesh(&hull)
    }
    #[cfg(feature = "dim3")]
    {
        let (vertices, indices) = rapier::parry::transformation::try_convex_hull(&points).ok()?;
        Some(triangle_list_mesh(&vertices, indices))
    }
}

/// Converts a [`TypedShape`] to a [`Mesh`].
///
/// Shapes convertible to a bevy builtin [`bevy::prelude::Meshable`] use it. Other shapes are
/// converted into a new mesh with [`PrimitiveTopology::TriangleList`], or
/// [`PrimitiveTopology::LineList`] for one-dimensional shapes (segments, polylines, and 2D
/// heightfields). In 2D, meshes lie on the `z = 0` plane.
///
/// Some shapes are approximated:
/// - half-spaces are represented by a finite square (3D) or segment (2D) with a half-size of
///   [`HALF_SPACE_MESH_HALF_SIZE`], centered at the shape's origin.
/// - round shapes use a polygonal (2D) or polyhedral (3D) approximation of their rounded border
///   controlled by [`ROUND_BORDER_SUBDIVISIONS`].
///
/// Returns `None` for custom shapes, and for compound shapes with no convertible part.
pub fn typed_shape_to_mesh(typed_shape: &TypedShape) -> Option<Mesh> {
    Some(match typed_shape {
        TypedShape::Ball(ball) => ball.mesh_builder().build(),
        TypedShape::Cuboid(cuboid) => {
            // FIXME: bevy 0.16 will expose a builder for cuboids: https://github.com/bevyengine/bevy/pull/17454
            let half_extents = cuboid.half_extents;
            #[cfg(feature = "dim2")]
            let mesh = bevy::prelude::Rectangle::new(half_extents.x * 2.0, half_extents.y * 2.0);
            #[cfg(feature = "dim3")]
            let mesh = bevy::prelude::Cuboid::new(
                half_extents.x * 2.0,
                half_extents.y * 2.0,
                half_extents.z * 2.0,
            );
            Mesh::from(mesh)
        }
        TypedShape::Capsule(capsule) => capsule.mesh_builder().build(),
        TypedShape::Segment(segment) => line_list_mesh(&[segment.a, segment.b], [[0, 1]]),
        TypedShape::Triangle(triangle) => {
            // FIXME: bevy 0.16 will expose a builder for triangles: https://github.com/bevyengine/bevy/pull/17454
            let a = triangle.a;
            let b = triangle.b;
            let c = triangle.c;
            #[cfg(feature = "dim2")]
            let mesh = bevy::prelude::Triangle3d::new(
                bevy::prelude::Vec3::new(a.x, a.y, 0.0),
                bevy::prelude::Vec3::new(b.x, b.y, 0.0),
                bevy::prelude::Vec3::new(c.x, c.y, 0.0),
            );
            #[cfg(feature = "dim3")]
            let mesh = bevy::prelude::Triangle3d::new(a, b, c);

            mesh.into()
        }
        TypedShape::Voxels(voxels) => {
            #[cfg(feature = "dim3")]
            {
                let (vtx, idx) = voxels.to_trimesh();
                triangle_list_mesh(&vtx, idx)
            }

            #[cfg(feature = "dim2")]
            {
                // One quad per filled voxel.
                let half_size = voxels.voxel_size() / 2.0;
                let mut vtx = vec![];
                let mut idx = vec![];
                for vox in voxels.voxels().filter(|vox| !vox.state.is_empty()) {
                    let base = vtx.len() as u32;
                    vtx.push(vox.center - half_size);
                    vtx.push(vox.center + Vect::new(half_size.x, -half_size.y));
                    vtx.push(vox.center + half_size);
                    vtx.push(vox.center + Vect::new(-half_size.x, half_size.y));
                    idx.push([base, base + 1, base + 2]);
                    idx.push([base, base + 2, base + 3]);
                }
                triangle_list_mesh(&vtx, idx)
            }
        }
        TypedShape::TriMesh(tri_mesh) => {
            triangle_list_mesh(tri_mesh.vertices(), tri_mesh.indices().iter().copied())
        }
        TypedShape::Polyline(polyline) => {
            line_list_mesh(polyline.vertices(), polyline.indices().iter().copied())
        }
        TypedShape::HalfSpace(half_space) => {
            #[cfg(feature = "dim2")]
            {
                let tangent = half_space.normal.perp() * HALF_SPACE_MESH_HALF_SIZE;
                line_list_mesh(&[-tangent, tangent], [[0, 1]])
            }
            #[cfg(feature = "dim3")]
            {
                PlaneMeshBuilder::new(
                    bevy::prelude::Dir3::new(half_space.normal).unwrap_or(bevy::prelude::Dir3::Y),
                    bevy::prelude::Vec2::splat(HALF_SPACE_MESH_HALF_SIZE * 2.0),
                )
                .build()
            }
        }
        TypedShape::HeightField(height_field) => {
            #[cfg(feature = "dim2")]
            {
                segments_mesh(height_field.segments().map(|seg| (seg.a, seg.b)))
            }
            #[cfg(feature = "dim3")]
            {
                // FIXME: we could use TriMesh::From(height_field), but that would clone, we should fix that in parry.
                let (vtx, idx) = height_field.to_trimesh();
                triangle_list_mesh(&vtx, idx)
            }
        }
        TypedShape::Compound(compound) => {
            let meshes: Vec<Mesh> = compound
                .shapes()
                .iter()
                .filter_map(|(pose, shape)| {
                    let mesh = typed_shape_to_mesh(&shape.as_typed_shape());
                    if mesh.is_none() {
                        log::warn!("Skipping a compound part that cannot be converted to a mesh.");
                    }
                    Some(mesh?.transformed_by(crate::utils::iso_to_transform(pose)))
                })
                .collect();
            merge_meshes(meshes)?
        }
        #[cfg(feature = "dim2")]
        TypedShape::ConvexPolygon(convex_polygon) => convex_polygon_mesh(convex_polygon.points())?,
        #[cfg(feature = "dim3")]
        TypedShape::ConvexPolyhedron(convex_polyhedron) => {
            let (vtx, idx) = convex_polyhedron.to_trimesh();
            triangle_list_mesh(&vtx, idx)
        }
        #[cfg(feature = "dim3")]
        TypedShape::Cone(cone) => cone.mesh_builder().build(),
        #[cfg(feature = "dim3")]
        TypedShape::Cylinder(cylinder) => cylinder.mesh_builder().build(),
        #[cfg(feature = "dim3")]
        TypedShape::RoundCone(round_cone) => {
            let inner = round_cone
                .inner_shape
                .to_trimesh(ROUND_SHAPE_CIRCLE_SUBDIVISIONS)
                .0;
            round_convex_mesh(&inner, round_cone.border_radius)?
        }
        #[cfg(feature = "dim3")]
        TypedShape::RoundCylinder(round_cylinder) => {
            let inner = round_cylinder
                .inner_shape
                .to_trimesh(ROUND_SHAPE_CIRCLE_SUBDIVISIONS)
                .0;
            round_convex_mesh(&inner, round_cylinder.border_radius)?
        }
        #[cfg(feature = "dim2")]
        TypedShape::RoundConvexPolygon(round_shape) => {
            round_convex_mesh(round_shape.inner_shape.points(), round_shape.border_radius)?
        }
        #[cfg(feature = "dim3")]
        TypedShape::RoundConvexPolyhedron(round_shape) => {
            round_convex_mesh(round_shape.inner_shape.points(), round_shape.border_radius)?
        }
        TypedShape::RoundCuboid(round_shape) => {
            #[cfg(feature = "dim2")]
            let inner = round_shape.inner_shape.to_polyline();
            #[cfg(feature = "dim3")]
            let inner = round_shape.inner_shape.to_trimesh().0;
            round_convex_mesh(&inner, round_shape.border_radius)?
        }
        TypedShape::RoundTriangle(round_shape) => {
            let tri = &round_shape.inner_shape;
            round_convex_mesh(&[tri.a, tri.b, tri.c], round_shape.border_radius)?
        }
        TypedShape::Custom(_shape) => {
            log::warn!("Custom shapes cannot be converted to a mesh.");
            return None;
        }
    })
}

/// Merges meshes into a single one, keeping only the vertex attributes shared by all of them.
///
/// Meshes with a primitive topology different from the first one are skipped.
fn merge_meshes(mut meshes: Vec<Mesh>) -> Option<Mesh> {
    let first = meshes.first()?;
    let topology = first.primitive_topology();
    meshes.retain(|mesh| {
        let same_topology = mesh.primitive_topology() == topology;
        if !same_topology {
            log::warn!("Skipping a compound part with an incompatible mesh topology.");
        }
        same_topology
    });

    let common_attributes: Vec<_> = meshes[0]
        .attributes()
        .map(|(attribute, _)| attribute.id)
        .filter(|id| meshes.iter().all(|mesh| mesh.contains_attribute(*id)))
        .collect();

    let mut meshes = meshes.into_iter().map(|mut mesh| {
        let to_remove: Vec<_> = mesh
            .attributes()
            .map(|(attribute, _)| attribute.id)
            .filter(|id| !common_attributes.contains(id))
            .collect();
        for id in to_remove {
            mesh.remove_attribute(id);
        }
        mesh
    });

    let mut result = meshes.next()?;
    for mesh in meshes {
        // FIXME: Error is simply "peeked" and ignored. Handle it properly by returning a Result from this function.
        let _ = result.merge(&mesh).inspect_err(|e| {
            log::warn!("Error merging mesh data: {e}");
        });
    }
    Some(result)
}

impl TryFrom<&Collider> for Mesh {
    type Error = ();

    fn try_from(collider: &Collider) -> Result<Self, Self::Error> {
        let typed_shape = collider.raw.as_typed_shape();
        typed_shape_to_mesh(&typed_shape).ok_or(())
    }
}

/// Trait to convert a parry shape to a [`MeshBuilder`].
pub trait ToMeshBuilder {
    /// Specific [`MeshBuilder`] being returned.
    type MeshBuilder: bevy::mesh::prelude::MeshBuilder;
    /// Returns a dedicated [`MeshBuilder`].
    fn mesh_builder(&self) -> Self::MeshBuilder;
}

#[cfg(feature = "dim2")]
impl ToMeshBuilder for &Ball {
    type MeshBuilder = CircleMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        CircleMeshBuilder::new(self.radius, 16)
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for &Ball {
    type MeshBuilder = SphereMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        SphereMeshBuilder::new(self.radius, bevy::mesh::SphereKind::Ico { subdivisions: 1 })
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for &rapier3d::prelude::HalfSpace {
    type MeshBuilder = PlaneMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        PlaneMeshBuilder::new(
            bevy::prelude::Dir3::new(self.normal).unwrap_or(bevy::prelude::Dir3::Y),
            bevy::prelude::Vec2::ONE,
        )
    }
}

#[cfg(feature = "dim2")]
impl ToMeshBuilder for &Capsule {
    type MeshBuilder = Capsule2dMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::mesh::Capsule2dMeshBuilder::new(self.radius, self.height(), 10)
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for &Capsule {
    type MeshBuilder = Capsule3dMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::mesh::Capsule3dMeshBuilder::new(self.radius, self.height(), 10, 10)
    }
}
#[cfg(feature = "dim3")]
impl ToMeshBuilder for &Cone {
    type MeshBuilder = ConeMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::mesh::ConeMeshBuilder::new(self.radius, self.half_height * 2.0, 16)
    }
}

#[cfg(feature = "dim3")]
impl ToMeshBuilder for &Cylinder {
    type MeshBuilder = CylinderMeshBuilder;

    fn mesh_builder(&self) -> Self::MeshBuilder {
        bevy::mesh::CylinderMeshBuilder::new(self.radius, self.half_height * 2.0, 16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::mesh::VertexAttributeValues;

    #[cfg(feature = "dim2")]
    const ROT_ID: crate::math::Rot = 0.0;
    #[cfg(feature = "dim3")]
    const ROT_ID: crate::math::Rot = crate::math::Rot::IDENTITY;

    fn to_mesh(collider: &Collider) -> Mesh {
        Mesh::try_from(collider).expect("the collider should be convertible to a mesh")
    }

    fn num_indices(mesh: &Mesh) -> usize {
        mesh.indices().unwrap().len()
    }

    fn positions(mesh: &Mesh) -> Vec<[f32; 3]> {
        match mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap() {
            VertexAttributeValues::Float32x3(pos) => pos.clone(),
            _ => panic!("unexpected position format"),
        }
    }

    fn max_coord(mesh: &Mesh, axis: usize) -> f32 {
        positions(mesh)
            .iter()
            .map(|p| p[axis])
            .fold(f32::MIN, f32::max)
    }

    #[test]
    fn voxels_mesh() {
        let single = to_mesh(&Collider::voxels(Vect::ONE, &[crate::math::IVect::ZERO]));
        assert_eq!(single.primitive_topology(), PrimitiveTopology::TriangleList);
        let pair = to_mesh(&Collider::voxels(
            Vect::ONE,
            &[crate::math::IVect::ZERO, crate::math::IVect::X],
        ));

        #[cfg(feature = "dim3")]
        {
            // Six faces of two triangles, minus the two faces shared by adjacent voxels.
            assert_eq!(num_indices(&single), 12 * 3);
            assert_eq!(num_indices(&pair), 20 * 3);
        }
        #[cfg(feature = "dim2")]
        {
            // One quad per voxel.
            assert_eq!(num_indices(&single), 2 * 3);
            assert_eq!(num_indices(&pair), 4 * 3);
        }
        assert!((max_coord(&pair, 0) - 2.0).abs() < 1.0e-5);
    }

    #[test]
    fn line_meshes() {
        let segment = to_mesh(&Collider::segment(Vect::ZERO, Vect::X));
        assert_eq!(segment.primitive_topology(), PrimitiveTopology::LineList);
        assert_eq!(num_indices(&segment), 2);

        let polyline = to_mesh(&Collider::polyline(
            vec![Vect::ZERO, Vect::X, Vect::X + Vect::Y],
            None,
        ));
        assert_eq!(polyline.primitive_topology(), PrimitiveTopology::LineList);
        assert_eq!(num_indices(&polyline), 4);
    }

    #[test]
    #[cfg(feature = "dim2")]
    fn heightfield_2d_mesh() {
        let mesh = to_mesh(&Collider::heightfield(vec![0.0, 1.0, 0.0, 1.0], Vect::ONE));
        assert_eq!(mesh.primitive_topology(), PrimitiveTopology::LineList);
        assert_eq!(num_indices(&mesh), 3 * 2);
    }

    #[test]
    fn halfspace_mesh() {
        let mesh = to_mesh(&Collider::halfspace(Vect::Y).unwrap());
        assert!((max_coord(&mesh, 0) - HALF_SPACE_MESH_HALF_SIZE).abs() < 1.0e-3);
        assert!(max_coord(&mesh, 1).abs() < 1.0e-3);
    }

    #[test]
    fn round_shape_meshes() {
        #[cfg(feature = "dim2")]
        let round_cuboid = Collider::round_cuboid(1.0, 2.0, 0.5);
        #[cfg(feature = "dim3")]
        let round_cuboid = Collider::round_cuboid(1.0, 2.0, 3.0, 0.5);
        let mesh = to_mesh(&round_cuboid);
        assert_eq!(mesh.primitive_topology(), PrimitiveTopology::TriangleList);
        assert!((max_coord(&mesh, 0) - 1.5).abs() < 1.0e-3);
        assert!((max_coord(&mesh, 1) - 2.5).abs() < 1.0e-3);

        let round_triangle = Collider::round_triangle(Vect::ZERO, Vect::X, Vect::Y, 0.1);
        let mesh = to_mesh(&round_triangle);
        assert!((max_coord(&mesh, 0) - 1.1).abs() < 1.0e-3);

        #[cfg(feature = "dim3")]
        {
            let mesh = to_mesh(&Collider::round_cylinder(1.0, 0.5, 0.1));
            assert!((max_coord(&mesh, 1) - 1.1).abs() < 1.0e-3);
            let mesh = to_mesh(&Collider::round_cone(1.0, 0.5, 0.1));
            assert!((max_coord(&mesh, 1) - 1.1).abs() < 1.0e-3);
            let hull =
                Collider::round_convex_hull(&[Vect::ZERO, Vect::X, Vect::Y, Vect::Z], 0.1).unwrap();
            assert!(num_indices(&to_mesh(&hull)) > 0);
        }
        #[cfg(feature = "dim2")]
        {
            let hull = Collider::round_convex_hull(&[Vect::ZERO, Vect::X, Vect::Y], 0.1).unwrap();
            assert!(num_indices(&to_mesh(&hull)) > 0);
        }
    }

    #[test]
    fn compound_mesh_applies_part_poses() {
        let compound = Collider::compound(vec![
            (Vect::X * 10.0, ROT_ID, Collider::ball(1.0)),
            (Vect::ZERO, ROT_ID, Collider::cuboid_for_tests()),
        ]);
        let mesh = to_mesh(&compound);
        // The ball mesh is an approximation that may not reach its exact radius along `x`.
        let max_x = max_coord(&mesh, 0);
        assert!(max_x > 10.5 && max_x < 11.0 + 1.0e-3, "{max_x}");
        // The ball mesh has normals but not the convex hull mesh, so they must be dropped
        // for all the remaining attributes to have one value per vertex.
        assert!(!mesh.contains_attribute(Mesh::ATTRIBUTE_NORMAL));
        for (_, values) in mesh.attributes() {
            assert_eq!(values.len(), positions(&mesh).len());
        }
    }

    #[cfg(feature = "dim3")]
    #[test]
    fn convex_polyhedron_mesh() {
        let (vtx, _) = rapier::prelude::Cuboid::new(Vect::splat(0.5)).to_trimesh();
        let mesh = to_mesh(&Collider::convex_hull(&vtx).unwrap());
        assert_eq!(num_indices(&mesh), 12 * 3);
    }

    impl Collider {
        fn cuboid_for_tests() -> Collider {
            let (vtx, _) = rapier::prelude::Cuboid::new(Vect::splat(0.5)).to_trimesh();
            Collider::convex_hull(&vtx).unwrap()
        }
    }
}
