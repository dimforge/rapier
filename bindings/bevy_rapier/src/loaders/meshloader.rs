//! Building colliders from mesh files (STL, Collada and Wavefront OBJ) with `rapier3d-meshloader`.

use crate::geometry::{Collider, MeshConverter};
use crate::math::Vect;
use crate::utils::iso_to_transform;
use bevy::prelude::*;
use rapier::geometry::SharedShape;
use std::path::Path;

pub use rapier3d_meshloader::{self, mesh_loader, MeshLoaderError};

/// A collider built from one of the meshes of a mesh file.
#[derive(Clone, Debug)]
pub struct MeshFileCollider {
    /// The collider computed from the mesh by the [`MeshConverter`].
    pub collider: Collider,
    /// The pose of the collider relative to the frame of the mesh file.
    ///
    /// This is the identity except for converters like [`MeshConverter::Obb`] which produce a
    /// shape offset from the origin of the mesh.
    pub transform: Transform,
    /// The mesh read from the file, without any scaling.
    pub raw_mesh: mesh_loader::Mesh,
    /// The material of the mesh read from the file (default for STL files).
    pub material: mesh_loader::Material,
}

/// Loads every mesh of a file, and converts each of them into a collider.
///
/// The file format is deduced from its extension. The `scale` is applied to the vertices before
/// the `converter` computes the collider shape.
pub fn load_mesh_file_colliders(
    path: impl AsRef<Path>,
    converter: &MeshConverter,
    scale: Vect,
) -> Result<Vec<MeshFileCollider>, MeshLoaderError> {
    rapier3d_meshloader::load_from_path(path, converter, scale)?
        .into_iter()
        .map(|loaded| {
            let loaded = loaded?;
            Ok(MeshFileCollider {
                collider: loaded.shape.into(),
                transform: iso_to_transform(&loaded.pose),
                raw_mesh: loaded.raw_mesh,
                material: loaded.material,
            })
        })
        .collect()
}

impl Collider {
    /// Loads a collider from a mesh file (STL, Collada or Wavefront OBJ).
    ///
    /// The file format is deduced from its extension. The `scale` is applied to the vertices
    /// before the `converter` computes the collider shape. If the file contains several meshes,
    /// or if the converter offsets the shape from the mesh origin, the result is a compound
    /// shape. See [`load_mesh_file_colliders`] to get the individual meshes instead.
    pub fn from_mesh_file(
        path: impl AsRef<Path>,
        converter: &MeshConverter,
        scale: Vect,
    ) -> Result<Collider, MeshLoaderError> {
        let mut parts: Vec<_> = rapier3d_meshloader::load_from_path(path, converter, scale)?
            .into_iter()
            .map(|loaded| loaded.map(|loaded| (loaded.pose, loaded.shape)))
            .collect::<Result<_, _>>()?;

        match parts.len() {
            0 => Err(MeshLoaderError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "the mesh file doesn't contain any mesh",
            ))),
            1 if parts[0].0 == rapier::math::Pose::IDENTITY => Ok(parts.remove(0).1.into()),
            _ => Ok(SharedShape::compound(parts).into()),
        }
    }
}

/// Converts a mesh read by `mesh_loader` into a Bevy triangle mesh.
///
/// The `scale` is applied to the vertex positions. Normals are computed if the file doesn't
/// provide them, and the first set of texture coordinates is kept.
#[cfg(feature = "to-bevy-mesh")]
pub fn raw_mesh_to_bevy_mesh(mesh: &mesh_loader::Mesh, scale: Vect) -> Mesh {
    use bevy::asset::RenderAssetUsages;
    use bevy::mesh::{Indices, PrimitiveTopology};

    let positions: Vec<[f32; 3]> = mesh
        .vertices
        .iter()
        .map(|v| (Vect::from(*v) * scale).into())
        .collect();
    let mut result = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_indices(Indices::U32(mesh.faces.iter().flatten().copied().collect()))
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions);

    if mesh.texcoords[0].len() == mesh.vertices.len() {
        result.insert_attribute(Mesh::ATTRIBUTE_UV_0, mesh.texcoords[0].clone());
    }
    if mesh.normals.len() == mesh.vertices.len() && scale.x == scale.y && scale.y == scale.z {
        result.insert_attribute(Mesh::ATTRIBUTE_NORMAL, mesh.normals.clone());
    } else {
        result.compute_normals();
    }
    result
}
