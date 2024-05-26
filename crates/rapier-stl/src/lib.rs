use rapier3d::geometry::{
    Collider, ColliderBuilder, Cuboid, MeshConverter, MeshConverterError, SharedShape, TriMesh,
};
use rapier3d::math::{Isometry, Point, Real, Vector};
use rapier3d::parry::bounding_volume;
use std::fs::File;
use std::io::{BufReader, Read, Seek};
use std::path::Path;
use stl_io::IndexedMesh;

#[derive(thiserror::Error, Debug)]
pub enum StlLoaderError {
    #[error(transparent)]
    MeshConverter(#[from] MeshConverterError),
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

/// The result of loading a shape from an stl mesh.
pub struct StlShape {
    /// The shape loaded from the file and converted by the [`MeshConverter`].
    pub shape: SharedShape,
    /// The shape’s pose.
    pub pose: Isometry<Real>,
    /// The raw mesh read from the stl file.
    pub raw_mesh: IndexedMesh,
}

pub fn load_from_path(
    file_path: impl AsRef<Path>,
    converter: MeshConverter,
    scale: Vector<Real>,
) -> Result<StlShape, StlLoaderError> {
    let mut reader = BufReader::new(File::open(file_path)?);
    load_from_reader(&mut reader, converter, scale)
}

pub fn load_from_reader<R: Read + Seek>(
    read: &mut R,
    converter: MeshConverter,
    scale: Vector<Real>,
) -> Result<StlShape, StlLoaderError> {
    let stl_mesh = stl_io::read_stl(read)?;
    Ok(load_from_raw_mesh(stl_mesh, converter, scale)?)
}

pub fn load_from_raw_mesh(
    raw_mesh: IndexedMesh,
    converter: MeshConverter,
    scale: Vector<Real>,
) -> Result<StlShape, MeshConverterError> {
    let mut vertices: Vec<_> = raw_mesh
        .vertices
        .iter()
        .map(|xyz| Point::new(xyz[0] as Real, xyz[1] as Real, xyz[2] as Real))
        .collect();
    vertices
        .iter_mut()
        .for_each(|pt| pt.coords.component_mul_assign(&scale));
    let indices: Vec<_> = raw_mesh
        .faces
        .iter()
        .map(|f| f.vertices.map(|i| i as u32))
        .collect();
    let (shape, pose) = converter.convert(vertices, indices)?;

    Ok(StlShape {
        shape,
        pose,
        raw_mesh,
    })
}
