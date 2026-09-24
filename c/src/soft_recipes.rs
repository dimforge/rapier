//! POD soft-body recipes and copied previews of procedural geometry.
use crate::*;
/// Initializes a recipe without allocating. Geometry is validated during preview/insertion.
#[rapier_export]
pub extern "C" fn rpr_rope_soft_body_desc(
    a: RprVector,
    b: RprVector,
    particles: usize,
) -> RprSoftBodyDesc {
    RprSoftBodyDesc {
        kind: RPR_SOFT_DESC_ROPE,
        a,
        b,
        nx: particles,
        ..RprSoftBodyDesc::default()
    }
}
#[cfg(feature = "dim2")]
/// Initializes a recipe without allocating. Geometry is validated during preview/insertion.
#[rapier_export]
pub extern "C" fn rpr_grid_soft_body_desc(
    center: RprVector,
    half_extents: RprVector,
    nx: usize,
    ny: usize,
) -> RprSoftBodyDesc {
    RprSoftBodyDesc {
        kind: RPR_SOFT_DESC_GRID,
        a: center,
        b: half_extents,
        nx,
        ny,
        ..RprSoftBodyDesc::default()
    }
}
#[cfg(feature = "dim3")]
/// Initializes a recipe without allocating. Geometry is validated during preview/insertion.
#[rapier_export]
pub extern "C" fn rpr_cuboid_soft_body_desc(
    center: RprVector,
    half_extents: RprVector,
    nx: usize,
    ny: usize,
    nz: usize,
) -> RprSoftBodyDesc {
    RprSoftBodyDesc {
        kind: RPR_SOFT_DESC_CUBOID,
        a: center,
        b: half_extents,
        nx,
        ny,
        nz,
        ..RprSoftBodyDesc::default()
    }
}
#[cfg(feature = "dim3")]
/// Initializes a recipe without allocating. Geometry is validated during preview/insertion.
#[rapier_export]
pub extern "C" fn rpr_cloth_soft_body_desc(
    origin: RprVector,
    du: RprVector,
    dv: RprVector,
    nu: usize,
    nv: usize,
) -> RprSoftBodyDesc {
    RprSoftBodyDesc {
        kind: RPR_SOFT_DESC_CLOTH,
        a: origin,
        du,
        dv,
        nx: nu,
        ny: nv,
        ..RprSoftBodyDesc::default()
    }
}
#[cfg(feature = "dim2")]
/// Initializes a recipe without allocating. Geometry is validated during preview/insertion.
#[rapier_export]
pub extern "C" fn rpr_disk_soft_body_desc(
    center: RprVector,
    radius: RprReal,
    particles: usize,
) -> RprSoftBodyDesc {
    RprSoftBodyDesc {
        kind: RPR_SOFT_DESC_DISK,
        volumePreservation: 1,
        a: center,
        radius,
        nx: particles,
        ..RprSoftBodyDesc::default()
    }
}
#[cfg(feature = "dim3")]
/// Initializes a recipe without allocating. Geometry is validated during preview/insertion.
#[rapier_export]
pub extern "C" fn rpr_sphere_soft_body_desc(
    center: RprVector,
    radius: RprReal,
    subdivisions: u32,
) -> RprSoftBodyDesc {
    RprSoftBodyDesc {
        kind: RPR_SOFT_DESC_SPHERE,
        a: center,
        radius,
        nx: subdivisions as usize,
        volumePreservation: 1,
        ..RprSoftBodyDesc::default()
    }
}
#[cfg(feature = "dim3")]
/// Initializes a recipe without allocating. Geometry is validated during preview/insertion.
#[rapier_export]
pub extern "C" fn rpr_cloth_tube_soft_body_desc(
    origin: RprVector,
    axis: RprVector,
    radius_start: RprReal,
    radius_end: RprReal,
    num_around: usize,
    num_along: usize,
) -> RprSoftBodyDesc {
    RprSoftBodyDesc {
        kind: RPR_SOFT_DESC_CLOTH_TUBE,
        a: origin,
        b: axis,
        radius: radius_start,
        radiusEnd: radius_end,
        nx: num_around,
        ny: num_along,
        ..RprSoftBodyDesc::default()
    }
}
/// Initializes a borrowed meshing recipe. Mesh generation happens on preview/insertion.
#[rapier_export]
pub extern "C" fn rpr_volumetric_soft_body_desc(
    vertices: RprVectorView,
    surface: RprSurfaceElementView,
    parameters: RprVolumeMeshParameters,
) -> RprSoftBodyDesc {
    RprSoftBodyDesc {
        kind: RPR_SOFT_DESC_VOLUMETRIC,
        positions: vertices,
        surface,
        meshing: parameters,
        ..RprSoftBodyDesc::default()
    }
}
/// Returns a material with the same softness for each constraint family.
#[rapier_export]
pub extern "C" fn rpr_uniform_soft_body_material(
    value: RprSpringCoefficients,
) -> RprSoftBodyMaterial {
    let mut material = RprSoftBodyMaterial::from(SoftBodyMaterial::default());
    material.edgeSoftness = value;
    material.bendSoftness = value;
    material.volumeSoftness = value;
    material.shapeMatchingSoftness = value;
    material
}
/// Copies generated particle positions into caller-owned storage; no persistent builder.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_particle_positions(
    desc: *const RprSoftBodyDesc,
    buffer: *mut RprVector,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let b = get(desc)?.raw()?;
            let v: Vec<RprVector> = b.positions.iter().copied().map(Into::into).collect();
            copy_out(&v, buffer, capacity, count)
        })
    })
}
/// Copies generated cell indices into caller-owned storage. Counts scalar indices.
#[rapier_export(soft_body_desc)]
pub unsafe extern "C" fn rpr_soft_body_desc_cell_indices(
    desc: *const RprSoftBodyDesc,
    buffer: *mut u32,
    capacity: usize,
) -> usize {
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            let b = get(desc)?.raw()?;
            let v: Vec<u32> = b.cells.iter().flatten().copied().collect();
            copy_out(&v, buffer, capacity, count)
        })
    })
}
