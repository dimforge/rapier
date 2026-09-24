//! POD shape constructors; input geometry is borrowed until insertion.
use crate::*;
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_round_cuboid_collider_desc(
    half_extents: RprVector,
    border_radius: RprReal,
) -> RprColliderDesc {
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_ROUND_CUBOID,
            a: half_extents,
            radius: border_radius,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_capsule_collider_desc(
    a: RprVector,
    b: RprVector,
    radius: RprReal,
) -> RprColliderDesc {
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_CAPSULE,
            a,
            b,
            radius,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_segment_collider_desc(a: RprVector, b: RprVector) -> RprColliderDesc {
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_SEGMENT,
            a,
            b,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_triangle_collider_desc(
    a: RprVector,
    b: RprVector,
    c: RprVector,
) -> RprColliderDesc {
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_TRIANGLE,
            a,
            b,
            c,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_halfspace_collider_desc(normal: RprVector) -> RprColliderDesc {
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_HALFSPACE,
            a: normal,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
#[cfg(feature = "dim3")]
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_cylinder_collider_desc(
    half_height: RprReal,
    radius: RprReal,
) -> RprColliderDesc {
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_CYLINDER,
            halfHeight: half_height,
            radius,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
#[cfg(feature = "dim3")]
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_cone_collider_desc(half_height: RprReal, radius: RprReal) -> RprColliderDesc {
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_CONE,
            halfHeight: half_height,
            radius,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
#[cfg(feature = "dim3")]
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_round_cylinder_collider_desc(
    half_height: RprReal,
    radius: RprReal,
    border_radius: RprReal,
) -> RprColliderDesc {
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_ROUND_CYLINDER,
            halfHeight: half_height,
            radius,
            borderRadius: border_radius,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_capsule_x_collider_desc(
    half_height: RprReal,
    radius: RprReal,
) -> RprColliderDesc {
    let axis = Vector::X * half_height;
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_CAPSULE,
            a: (-axis).into(),
            b: axis.into(),
            radius,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_capsule_y_collider_desc(
    half_height: RprReal,
    radius: RprReal,
) -> RprColliderDesc {
    let axis = Vector::Y * half_height;
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_CAPSULE,
            a: (-axis).into(),
            b: axis.into(),
            radius,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
#[cfg(feature = "dim3")]
/// Returns a description without allocating or validating. Build/insert validates its fields.
#[rapier_export]
pub extern "C" fn rpr_capsule_z_collider_desc(
    half_height: RprReal,
    radius: RprReal,
) -> RprColliderDesc {
    let axis = Vector::Z * half_height;
    RprColliderDesc {
        shape: RprShapeDesc {
            kind: RPR_SHAPE_DESC_CAPSULE,
            a: (-axis).into(),
            b: axis.into(),
            radius,
            ..RprShapeDesc::default()
        },
        ..RprColliderDesc::default()
    }
}
