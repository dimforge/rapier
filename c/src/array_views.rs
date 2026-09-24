//! Borrowed, typed array inputs. Setters do not allocate or retain ownership.
#![allow(non_snake_case)]
use crate::*;

/// Vertex indices for one edge; contiguous u32 fields with no padding.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprEdge {
    pub a: u32,
    pub b: u32,
}
const _: () = assert!(size_of::<RprEdge>() == 2 * size_of::<u32>());
/// Vertex indices for one triangle; contiguous u32 fields with no padding.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprTriangle {
    pub a: u32,
    pub b: u32,
    pub c: u32,
}
const _: () = assert!(size_of::<RprTriangle>() == 3 * size_of::<u32>());
/// Vertex indices for one tetrahedron; contiguous u32 fields with no padding.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprTetrahedron {
    pub a: u32,
    pub b: u32,
    pub c: u32,
    pub d: u32,
}
const _: () = assert!(size_of::<RprTetrahedron>() == 4 * size_of::<u32>());
/// Vertex indices for one dihedral; contiguous u32 fields with no padding.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprDihedral {
    pub a: u32,
    pub b: u32,
    pub c: u32,
    pub d: u32,
}
const _: () = assert!(size_of::<RprDihedral>() == 4 * size_of::<u32>());
/// Borrowed array of vector elements. count always counts elements, not scalars.
/// Copying this view does not copy its data or extend its lifetime. No Free is needed.
/// Data must remain live through the build/insert call that reads the description.
/// NULL is permitted only when count is zero.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprVectorView {
    pub data: *const RprVector,
    pub count: usize,
}
/// Borrowed array of real elements. count always counts elements, not scalars.
/// Copying this view does not copy its data or extend its lifetime. No Free is needed.
/// Data must remain live through the build/insert call that reads the description.
/// NULL is permitted only when count is zero.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprRealView {
    pub data: *const RprReal,
    pub count: usize,
}
/// Borrowed array of index elements. count always counts elements, not scalars.
/// Copying this view does not copy its data or extend its lifetime. No Free is needed.
/// Data must remain live through the build/insert call that reads the description.
/// NULL is permitted only when count is zero.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprIndexView {
    pub data: *const u32,
    pub count: usize,
}
/// Borrowed array of edge elements. count always counts elements, not scalars.
/// Copying this view does not copy its data or extend its lifetime. No Free is needed.
/// Data must remain live through the build/insert call that reads the description.
/// NULL is permitted only when count is zero.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprEdgeView {
    pub data: *const RprEdge,
    pub count: usize,
}
/// Borrowed array of triangle elements. count always counts elements, not scalars.
/// Copying this view does not copy its data or extend its lifetime. No Free is needed.
/// Data must remain live through the build/insert call that reads the description.
/// NULL is permitted only when count is zero.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprTriangleView {
    pub data: *const RprTriangle,
    pub count: usize,
}
/// Borrowed array of tetrahedron elements. count always counts elements, not scalars.
/// Copying this view does not copy its data or extend its lifetime. No Free is needed.
/// Data must remain live through the build/insert call that reads the description.
/// NULL is permitted only when count is zero.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprTetrahedronView {
    pub data: *const RprTetrahedron,
    pub count: usize,
}
/// Borrowed array of dihedral elements. count always counts elements, not scalars.
/// Copying this view does not copy its data or extend its lifetime. No Free is needed.
/// Data must remain live through the build/insert call that reads the description.
/// NULL is permitted only when count is zero.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprDihedralView {
    pub data: *const RprDihedral,
    pub count: usize,
}
#[cfg(feature = "dim2")]
pub type RprCellView = RprTriangleView;
#[cfg(feature = "dim3")]
pub type RprCellView = RprTetrahedronView;
#[cfg(feature = "dim2")]
pub type RprSurfaceElementView = RprEdgeView;
#[cfg(feature = "dim3")]
pub type RprSurfaceElementView = RprTriangleView;

// Validate metadata without reading elements or allocating. Numerical values and
// topology bounds are validated by the existing build/insert path.
pub(crate) fn validate_view<T>(data: *const T, count: usize) -> Result {
    if count == 0 {
        return Ok(());
    }
    ensure(
        count <= isize::MAX as usize / size_of::<T>(),
        "array view is too large",
    )?;
    if data.is_null() {
        return Err((RPR_NULL_POINTER, "nonempty array view has null data".into()));
    }
    ensure(data.is_aligned(), "misaligned array view")
}

/// Replace the shape geometry with a borrowed tri mesh. Counts are elements.
/// Copies no arrays. Invalid view metadata leaves the description unchanged.
/// Geometry and flags are validated when the description is built or inserted.
#[rapier_export(shape_desc)]
pub unsafe extern "C" fn rpr_shape_desc_set_trimesh(
    desc: *mut RprShapeDesc,
    vertices: RprVectorView,
    indices: RprTriangleView,
    flags: u32,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(vertices.data, vertices.count)?;
        validate_view(indices.data, indices.count)?;
        let shape = RprShapeDesc {
            kind: RPR_SHAPE_DESC_TRIMESH,
            vertices,
            triangles: indices,
            flags,
            ..RprShapeDesc::default()
        };
        output(desc, shape)
    })
}
/// Replace the shape geometry with a borrowed polyline. Counts are elements.
/// Copies no arrays. Invalid view metadata leaves the description unchanged.
/// Geometry and flags are validated when the description is built or inserted.
#[rapier_export(shape_desc)]
pub unsafe extern "C" fn rpr_shape_desc_set_polyline(
    desc: *mut RprShapeDesc,
    vertices: RprVectorView,
    indices: RprEdgeView,
    flags: u32,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(vertices.data, vertices.count)?;
        validate_view(indices.data, indices.count)?;
        let shape = RprShapeDesc {
            kind: RPR_SHAPE_DESC_POLYLINE,
            vertices,
            edges: indices,
            flags,
            ..RprShapeDesc::default()
        };
        output(desc, shape)
    })
}
/// Replace the shape geometry with a borrowed convex hull point cloud.
#[rapier_export(shape_desc)]
pub unsafe extern "C" fn rpr_shape_desc_set_convex_hull(
    desc: *mut RprShapeDesc,
    vertices: RprVectorView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(vertices.data, vertices.count)?;
        output(
            desc,
            RprShapeDesc {
                kind: RPR_SHAPE_DESC_CONVEX_HULL,
                vertices,
                ..RprShapeDesc::default()
            },
        )
    })
}
/// Select an explicit particle recipe and borrow its positions. Other fields are preserved.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_particles(
    desc: *mut RprSoftBodyDesc,
    positions: RprVectorView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(positions.data, positions.count)?;
        let desc = get_mut(desc)?;
        desc.kind = RPR_SOFT_DESC_PARTICLES;
        desc.positions = positions;
        Ok(())
    })
}
/// Select a surface recipe and borrow its vertices and elements. Other fields are preserved.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_surface_mesh(
    desc: *mut RprSoftBodyDesc,
    vertices: RprVectorView,
    elements: RprSurfaceElementView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(vertices.data, vertices.count)?;
        validate_view(elements.data, elements.count)?;
        let desc = get_mut(desc)?;
        desc.kind = RPR_SOFT_DESC_SURFACE;
        desc.positions = vertices;
        desc.surface = elements;
        Ok(())
    })
}
/// Borrow skin geometry. Other fields, including skinCollision, are preserved.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_skin(
    desc: *mut RprSoftBodyDesc,
    vertices: RprVectorView,
    elements: RprSurfaceElementView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(vertices.data, vertices.count)?;
        validate_view(elements.data, elements.count)?;
        let desc = get_mut(desc)?;
        desc.skinVertices = vertices;
        desc.skinIndices = elements;
        Ok(())
    })
}
/// Borrow masses; preserve all other fields. No allocation or element reads.
/// Zero counts retain the recipe's generated defaults at insertion, as with directly assigned views.
/// Invalid view metadata leaves the description unchanged.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_masses(
    desc: *mut RprSoftBodyDesc,
    view: RprRealView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(view.data, view.count)?;
        let desc = get_mut(desc)?;
        desc.masses = view;
        Ok(())
    })
}
/// Borrow pinned particles; preserve all other fields. No allocation or element reads.
/// Zero counts retain the recipe's generated defaults at insertion, as with directly assigned views.
/// Invalid view metadata leaves the description unchanged.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_pinned_particles(
    desc: *mut RprSoftBodyDesc,
    view: RprIndexView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(view.data, view.count)?;
        let desc = get_mut(desc)?;
        desc.pinned = view;
        Ok(())
    })
}
/// Borrow edges; preserve all other fields. No allocation or element reads.
/// Zero counts retain the recipe's generated defaults at insertion, as with directly assigned views.
/// Invalid view metadata leaves the description unchanged.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_edges(
    desc: *mut RprSoftBodyDesc,
    view: RprEdgeView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(view.data, view.count)?;
        let desc = get_mut(desc)?;
        desc.edges = view;
        Ok(())
    })
}
/// Borrow bend edges; preserve all other fields. No allocation or element reads.
/// Zero counts retain the recipe's generated defaults at insertion, as with directly assigned views.
/// Invalid view metadata leaves the description unchanged.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_bend_edges(
    desc: *mut RprSoftBodyDesc,
    view: RprEdgeView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(view.data, view.count)?;
        let desc = get_mut(desc)?;
        desc.bendEdges = view;
        Ok(())
    })
}
/// Borrow cells; preserve all other fields. No allocation or element reads.
/// Zero counts retain the recipe's generated defaults at insertion, as with directly assigned views.
/// Invalid view metadata leaves the description unchanged.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_cells(
    desc: *mut RprSoftBodyDesc,
    view: RprCellView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(view.data, view.count)?;
        let desc = get_mut(desc)?;
        desc.cells = view;
        Ok(())
    })
}
/// Borrow surface; preserve all other fields. No allocation or element reads.
/// Zero counts retain the recipe's generated defaults at insertion, as with directly assigned views.
/// Invalid view metadata leaves the description unchanged.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_surface(
    desc: *mut RprSoftBodyDesc,
    view: RprSurfaceElementView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(view.data, view.count)?;
        let desc = get_mut(desc)?;
        desc.surface = view;
        Ok(())
    })
}
/// Borrow tension only edges; preserve all other fields. No allocation or element reads.
/// Zero counts retain the recipe's generated defaults at insertion, as with directly assigned views.
/// Invalid view metadata leaves the description unchanged.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_tension_only_edges(
    desc: *mut RprSoftBodyDesc,
    view: RprIndexView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(view.data, view.count)?;
        let desc = get_mut(desc)?;
        desc.tensionOnlyEdges = view;
        Ok(())
    })
}
/// Borrow dihedrals; preserve all other fields. No allocation or element reads.
/// Zero counts retain the recipe's generated defaults at insertion, as with directly assigned views.
/// Invalid view metadata leaves the description unchanged.
#[cfg(feature = "dim3")]
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_dihedrals(
    desc: *mut RprSoftBodyDesc,
    view: RprDihedralView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(view.data, view.count)?;
        let desc = get_mut(desc)?;
        desc.dihedrals = view;
        Ok(())
    })
}
/// Borrow wire; preserve all other fields. No allocation or element reads.
/// Zero counts retain the recipe's generated defaults at insertion, as with directly assigned views.
/// Invalid view metadata leaves the description unchanged.
#[cfg(feature = "dim3")]
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_set_wire(
    desc: *mut RprSoftBodyDesc,
    view: RprEdgeView,
) -> RprStatus {
    ffi(|| unsafe {
        validate_view(view.data, view.count)?;
        let desc = get_mut(desc)?;
        desc.wire = view;
        Ok(())
    })
}

#[cfg(feature = "dim2")]
pub type RprCell = RprTriangle;
#[cfg(feature = "dim3")]
pub type RprCell = RprTetrahedron;
#[cfg(feature = "dim2")]
pub type RprSurfaceElement = RprEdge;
#[cfg(feature = "dim3")]
pub type RprSurfaceElement = RprTriangle;

/// Borrowed elements; count counts elements. Data must remain live through insertion.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftEdgeSoftnessView {
    pub data: *const RprSoftEdgeSoftness,
    pub count: usize,
}

/// Borrowed elements; count counts elements. Data must remain live through insertion.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprSoftEdgeTearView {
    pub data: *const RprSoftEdgeTear,
    pub count: usize,
}

/// Borrowed elements; count counts elements. Data must remain live through insertion.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprCompoundShapeView {
    pub data: *const RprCompoundShapeDesc,
    pub count: usize,
}
