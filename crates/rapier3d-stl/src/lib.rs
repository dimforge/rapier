//! ## STL loader for the Rapier physics engine
//!
//! Rapier is a set of 2D and 3D physics engines for games, animation, and robotics. The `rapier3d-stl`
//! crate lets you create a shape compatible with `rapier3d` and `parry3d` (the underlying collision-detection
//! library) from an STL file.

#![warn(missing_docs)]

use rapier3d::geometry::{MeshConverter, MeshConverterError, SharedShape};
use rapier3d::math::{Isometry, Point, Real, Vector};
use std::fs::File;
use std::io::{BufReader, Read, Seek};
use std::path::Path;
use stl_io::IndexedMesh;

/// Error while loading an STL file.
#[derive(thiserror::Error, Debug)]
pub enum StlLoaderError {
    /// An error triggered by rapier’s [`MeshConverter`].
    #[error(transparent)]
    MeshConverter(#[from] MeshConverterError),
    /// A generic IO error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

/// The result of loading a shape from an stl mesh.
pub struct StlShape {
    /// The shape loaded from the file and converted by the [`MeshConverter`].
    pub shape: SharedShape,
    /// The shape’s pose.
    pub pose: Isometry<Real>,
    /// The raw mesh read from the stl file without any modification.
    pub raw_mesh: IndexedMesh,
}

/// Loads an STL file as a shape from a file.
///
/// # Parameters
/// - `file_path`: the STL file’s path.
/// - `converter`: controls how the shape is computed from the STL content. In particular, it lets
///                you specify if the computed [`StlShape::shape`] is a triangle mesh, its convex hull,
///                bounding box, etc.
/// - `scale`: the scaling factor applied to the geometry input to the `converter`. This scale will
///            affect at the geometric level the [`StlShape::shape`]. Note that raw mesh value stored
///            in [`StlShape::raw_mesh`] remains unscaled.
pub fn load_from_path(
    file_path: impl AsRef<Path>,
    converter: MeshConverter,
    scale: Vector<Real>,
) -> Result<StlShape, StlLoaderError> {
    let mut reader = BufReader::new(File::open(file_path)?);
    load_from_reader(&mut reader, converter, scale)
}

/// Loads an STL file as a shape from an arbitrary reader.
///
/// # Parameters
/// - `reader`: the reader.
/// - `converter`: controls how the shape is computed from the STL content. In particular, it lets
///                you specify if the computed [`StlShape::shape`] is a triangle mesh, its convex hull,
///                bounding box, etc.
/// - `scale`: the scaling factor applied to the geometry input to the `converter`. This scale will
///            affect at the geometric level the [`StlShape::shape`]. Note that raw mesh value stored
///            in [`StlShape::raw_mesh`] remains unscaled.
pub fn load_from_reader<R: Read + Seek>(
    read: &mut R,
    converter: MeshConverter,
    scale: Vector<Real>,
) -> Result<StlShape, StlLoaderError> {
    let stl_mesh = stl_io::read_stl(read)?;
    Ok(load_from_raw_mesh(stl_mesh, converter, scale)?)
}

/// Loads an STL file as a shape from a preloaded raw stl mesh.
///
/// # Parameters
/// - `raw_mesh`: the raw stl mesh.
/// - `converter`: controls how the shape is computed from the STL content. In particular, it lets
///                you specify if the computed [`StlShape::shape`] is a triangle mesh, its convex hull,
///                bounding box, etc.
/// - `scale`: the scaling factor applied to the geometry input to the `converter`. This scale will
///            affect at the geometric level the [`StlShape::shape`]. Note that raw mesh value stored
///            in [`StlShape::raw_mesh`] remains unscaled.
pub fn load_from_raw_mesh(
    raw_mesh: IndexedMesh,
    converter: MeshConverter,
    scale: Vector<Real>,
) -> Result<StlShape, MeshConverterError> {
    let mut vertices: Vec<_> = raw_mesh
        .vertices
        .iter()
        .map(|xyz| Point::new(xyz[0], xyz[1], xyz[2]))
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
