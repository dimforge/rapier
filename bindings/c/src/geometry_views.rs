//! Typed geometry input boundaries; view counts always count elements.
use crate::handle_access::forward;
use crate::*;
/// Create an owned compound shape by convex decomposition of the input surface. Release it with
/// rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_convex_decomposition_shared_shape(
    vertices: RprVectorView,
    indices: RprSurfaceElementView,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            crate::array_views::validate_view(indices.data, indices.count)?;
            forward(impl_rpr_shared_shape_convex_decomposition(
                vertices.data,
                vertices.count,
                indices.data.cast(),
                indices.count,
                out,
            ))
        })
    })
}
/// Create an owned voxel shape by quantizing points with the supplied per-axis voxel size. Release
/// it with rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_voxels_shared_shape_from_points(
    voxel_size: RprVector,
    points: RprVectorView,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(points.data, points.count)?;
            forward(impl_rpr_shared_shape_voxels_from_points(
                voxel_size,
                points.data,
                points.count,
                out,
            ))
        })
    })
}
/// Create an owned voxel shape from a surface mesh with the supplied uniform voxel size. Release it
/// with rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_voxelized_mesh_shared_shape(
    vertices: RprVectorView,
    indices: RprSurfaceElementView,
    voxel_size: RprReal,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            crate::array_views::validate_view(indices.data, indices.count)?;
            forward(impl_rpr_shared_shape_voxelized_mesh(
                vertices.data,
                vertices.count,
                indices.data.cast(),
                indices.count,
                voxel_size,
                out,
            ))
        })
    })
}
/// Create an owned convex hull of the supplied vertices. Release it with rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_convex_hull_shared_shape(
    vertices: RprVectorView,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            forward(impl_rpr_shared_shape_convex_hull(
                vertices.data,
                vertices.count,
                out,
            ))
        })
    })
}
/// Create an owned convex polyhedron from vertices and triangle indices assumed to form a convex
/// mesh (no convex hull is computed); fails on degenerate input. Release it with rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[cfg(feature = "dim3")]
#[rapier_export]
pub unsafe extern "C" fn rpr_convex_mesh_shared_shape(
    vertices: RprVectorView,
    indices: RprTriangleView,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            crate::array_views::validate_view(indices.data, indices.count)?;
            out_ptr(out)?;
            let points = input(vertices.data, vertices.count)?
                .iter()
                .copied()
                .map(RprVector::raw)
                .collect::<Result<Vec<_>>>()?;
            let indices = indices_array::<3>(indices.data.cast(), indices.count, points.len())?;
            ensure(!indices.is_empty(), "empty mesh")?;
            let shape = SharedShape::convex_mesh(points, &indices)
                .ok_or_else(|| invalid("degenerate convex mesh"))?;
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}
/// Create an owned triangle mesh from vertices and triangle indices. Release it with
/// rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_trimesh_shared_shape(
    vertices: RprVectorView,
    indices: RprTriangleView,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            crate::array_views::validate_view(indices.data, indices.count)?;
            forward(impl_rpr_shared_shape_trimesh(
                vertices.data,
                vertices.count,
                indices.data.cast(),
                indices.count,
                out,
            ))
        })
    })
}
/// Create an owned polyline from vertices and edge indices. Release it with rpr_free_shared_shape.
/// Empty indices connect the vertices in order (a line strip).
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_polyline_shared_shape(
    vertices: RprVectorView,
    indices: RprEdgeView,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            crate::array_views::validate_view(indices.data, indices.count)?;
            forward(impl_rpr_shared_shape_polyline(
                vertices.data,
                vertices.count,
                indices.data.cast(),
                indices.count,
                out,
            ))
        })
    })
}
/// Create an owned oriented 2D polyline from vertices and edge indices. Release it with
/// rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[cfg(feature = "dim2")]
#[rapier_export]
pub unsafe extern "C" fn rpr_oriented_polyline_shared_shape(
    vertices: RprVectorView,
    indices: RprEdgeView,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            crate::array_views::validate_view(indices.data, indices.count)?;
            forward(impl_rpr_shared_shape_oriented_polyline(
                vertices.data,
                vertices.count,
                indices.data.cast(),
                indices.count,
                out,
            ))
        })
    })
}
/// Create an owned convex polygon from vertices already ordered along its boundary. Release it with
/// rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[cfg(feature = "dim2")]
#[rapier_export]
pub unsafe extern "C" fn rpr_convex_polyline_shared_shape(
    vertices: RprVectorView,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            forward(impl_rpr_shared_shape_convex_polyline(
                vertices.data,
                vertices.count,
                out,
            ))
        })
    })
}
/// Create an owned round convex hull shape. Release it with rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_round_convex_hull_shared_shape(
    vertices: RprVectorView,
    border_radius: RprReal,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            forward(impl_rpr_shared_shape_round_convex_hull(
                vertices.data,
                vertices.count,
                border_radius,
                out,
            ))
        })
    })
}
/// Create an owned triangle mesh with the supplied TRIMESH_* processing flags. Release it with
/// rpr_free_shared_shape.
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_trimesh_shared_shape_with_flags(
    vertices: RprVectorView,
    indices: RprTriangleView,
    flags: u32,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            crate::array_views::validate_view(vertices.data, vertices.count)?;
            crate::array_views::validate_view(indices.data, indices.count)?;
            forward(impl_rpr_shared_shape_trimesh_with_flags(
                vertices.data,
                vertices.count,
                indices.data.cast(),
                indices.count,
                flags,
                out,
            ))
        })
    })
}
