//! Typed geometry input boundaries; view counts always count elements.
use crate::handle_access::forward;
use crate::*;
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
#[cfg(feature = "dim2")]
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
#[cfg(feature = "dim2")]
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
/// Copies typed input geometry into an owned shared shape; arrays may be released on return.
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
