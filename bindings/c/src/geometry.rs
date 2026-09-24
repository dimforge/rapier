use crate::*;
/// Create an owned ball shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_ball_shared_shape(radius: RprReal) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let shape = SharedShape::ball(positive(radius)?);
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

/// Create an owned cuboid shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_cuboid_shared_shape(half_extents: RprVector) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let v = half_extents.raw()?;
            ensure(v.min_element() > 0.0, "half extents must be positive")?;
            let shape = {
                #[cfg(feature = "dim2")]
                {
                    SharedShape::cuboid(v.x, v.y)
                }
                #[cfg(feature = "dim3")]
                {
                    SharedShape::cuboid(v.x, v.y, v.z)
                }
            };
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

/// Create an owned round cuboid shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_round_cuboid_shared_shape(
    half_extents: RprVector,
    border_radius: RprReal,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let v = half_extents.raw()?;
            ensure(v.min_element() > 0.0, "half extents must be positive")?;
            let r = nonnegative(border_radius)?;
            let shape = {
                #[cfg(feature = "dim2")]
                {
                    SharedShape::round_cuboid(v.x, v.y, r)
                }
                #[cfg(feature = "dim3")]
                {
                    SharedShape::round_cuboid(v.x, v.y, v.z, r)
                }
            };
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

/// Create an owned capsule shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_capsule_shared_shape(
    a: RprVector,
    b: RprVector,
    radius: RprReal,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let shape = SharedShape::capsule(a.raw()?, b.raw()?, positive(radius)?);
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

/// Create an owned segment shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_segment_shared_shape(
    a: RprVector,
    b: RprVector,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let shape = SharedShape::segment(a.raw()?, b.raw()?);
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

/// Create an owned triangle shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_triangle_shared_shape(
    a: RprVector,
    b: RprVector,
    c: RprVector,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let shape = SharedShape::triangle(a.raw()?, b.raw()?, c.raw()?);
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

/// Create an owned halfspace shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_halfspace_shared_shape(normal: RprVector) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let v = normal.raw()?;
            positive(v.length())?;
            let shape = SharedShape::halfspace(v / v.length());
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

/// Create an owned cylinder shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[cfg(feature = "dim3")]
#[rapier_export]
pub unsafe extern "C" fn rpr_cylinder_shared_shape(
    half_height: RprReal,
    radius: RprReal,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let shape = SharedShape::cylinder(positive(half_height)?, positive(radius)?);
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

/// Create an owned cone shape. Release it with rpr_free_shared_shape.
/// @ingroup shapes
#[cfg(feature = "dim3")]
#[rapier_export]
pub unsafe extern "C" fn rpr_cone_shared_shape(
    half_height: RprReal,
    radius: RprReal,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let shape = SharedShape::cone(positive(half_height)?, positive(radius)?);
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

pub(crate) unsafe fn impl_rpr_shared_shape_convex_hull(
    vertices: *const RprVector,
    count: usize,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let points = input(vertices, count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        ensure(points.len() > rapier::math::DIM, "not enough vertices")?;
        let shape =
            SharedShape::convex_hull(&points).ok_or_else(|| invalid("degenerate convex hull"))?;
        output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
    })
}

pub(crate) unsafe fn impl_rpr_shared_shape_trimesh(
    vertices: *const RprVector,
    vertex_count: usize,
    indices: *const u32,
    element_count: usize,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let points = input(vertices, vertex_count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        let indices = indices_array::<3>(indices, element_count, vertex_count)?;
        ensure(!indices.is_empty(), "empty mesh")?;
        let shape = SharedShape::trimesh(points, indices).map_err(|e| invalid(e.to_string()))?;
        output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
    })
}

pub(crate) unsafe fn impl_rpr_shared_shape_polyline(
    vertices: *const RprVector,
    vertex_count: usize,
    indices: *const u32,
    element_count: usize,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let points = input(vertices, vertex_count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        let indices = indices_array::<2>(indices, element_count, vertex_count)?;
        ensure(points.len() >= 2, "not enough vertices")?;
        // No edges connects the vertices in order, as a line strip.
        let shape = SharedShape::polyline(points, (!indices.is_empty()).then_some(indices));
        output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
    })
}

pub(crate) unsafe fn indices_array<const N: usize>(
    indices: *const u32,
    count: usize,
    vertices: usize,
) -> Result<Vec<[u32; N]>> {
    let len = count
        .checked_mul(N)
        .ok_or_else(|| invalid("index count overflow"))?;
    let values = unsafe { input(indices, len)? };
    ensure(
        values.iter().all(|&i| (i as usize) < vertices),
        "vertex index out of range",
    )?;
    Ok(values
        .chunks_exact(N)
        .map(|c| c.try_into().unwrap())
        .collect())
}
/// Create an owned compound shape; each child pose is relative to the compound. Child shapes are
/// shared, not consumed. Release with rpr_free_shared_shape.
/// @ingroup shapes
#[rapier_export]
pub unsafe extern "C" fn rpr_compound_shared_shape(
    children: RprCompoundShapeView,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let desc = RprShapeDesc {
                kind: RPR_SHAPE_DESC_COMPOUND,
                children,
                ..Default::default()
            };
            output(out, Box::into_raw(Box::new(RprSharedShape(desc.raw()?))))
        })
    })
}

pub(crate) unsafe fn native_collider_set_position(
    object: *mut RprCollider,
    value: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let object = get_mut(object)?;
        object.0.set_position(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_translation(
    object: *mut RprCollider,
    value: RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let object = get_mut(object)?;
        object.0.set_translation(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_friction(
    object: *mut RprCollider,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let object = get_mut(object)?;
        object.0.set_friction(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_restitution(
    object: *mut RprCollider,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let object = get_mut(object)?;
        object.0.set_restitution(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_density(
    object: *mut RprCollider,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let object = get_mut(object)?;
        object.0.set_density(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_mass(
    object: *mut RprCollider,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let object = get_mut(object)?;
        object.0.set_mass(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_sensor(
    object: *mut RprCollider,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let object = get_mut(object)?;
        object.0.set_sensor(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_enabled(
    object: *mut RprCollider,
    value: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let value = boolean(value)?;
        let object = get_mut(object)?;
        object.0.set_enabled(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_collision_groups(
    object: *mut RprCollider,
    value: RprInteractionGroups,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let object = get_mut(object)?;
        object.0.set_collision_groups(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_solver_groups(
    object: *mut RprCollider,
    value: RprInteractionGroups,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let object = get_mut(object)?;
        object.0.set_solver_groups(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_friction_combine_rule(
    object: *mut RprCollider,
    value: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let value = combine(value)?;
        let object = get_mut(object)?;
        object.0.set_friction_combine_rule(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_restitution_combine_rule(
    object: *mut RprCollider,
    value: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let value = combine(value)?;
        let object = get_mut(object)?;
        object.0.set_restitution_combine_rule(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_contact_skin(
    object: *mut RprCollider,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let object = get_mut(object)?;
        object.0.set_contact_skin(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_contact_force_event_threshold(
    object: *mut RprCollider,
    value: RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let value = nonnegative(value)?;
        let object = get_mut(object)?;
        object.0.set_contact_force_event_threshold(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_active_events(
    object: *mut RprCollider,
    value: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let value = ActiveEvents::from_bits(value).ok_or_else(|| invalid("unknown event flags"))?;
        let object = get_mut(object)?;
        object.0.set_active_events(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_active_hooks(
    object: *mut RprCollider,
    value: u32,
) -> RprStatus {
    ffi(|| unsafe {
        let value = ActiveHooks::from_bits(value).ok_or_else(|| invalid("unknown hook flags"))?;
        let object = get_mut(object)?;
        object.0.set_active_hooks(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_active_collision_types(
    object: *mut RprCollider,
    value: u16,
) -> RprStatus {
    ffi(|| unsafe {
        let value = ActiveCollisionTypes::from_bits(value)
            .ok_or_else(|| invalid("unknown collision type flags"))?;
        let object = get_mut(object)?;
        object.0.set_active_collision_types(value);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_user_data(
    object: *mut RprCollider,
    value: RprUserData,
) -> RprStatus {
    ffi(|| unsafe {
        get_mut(object)?.0.user_data = value.raw();
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_position(
    object: *const RprCollider,
    out: *mut RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, (*object.0.position()).into())
    })
}

pub(crate) unsafe fn native_collider_translation(
    object: *const RprCollider,
    out: *mut RprVector,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.translation().into())
    })
}

pub(crate) unsafe fn native_collider_rotation(
    object: *const RprCollider,
    out: *mut RprRotation,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.rotation().into())
    })
}

pub(crate) unsafe fn native_collider_collision_groups(
    object: *const RprCollider,
    out: *mut RprInteractionGroups,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.collision_groups().into())
    })
}

pub(crate) unsafe fn native_collider_solver_groups(
    object: *const RprCollider,
    out: *mut RprInteractionGroups,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.solver_groups().into())
    })
}

pub(crate) unsafe fn native_collider_parent(
    object: *const RprCollider,
    out: *mut RprRigidBodyHandle,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.parent().map(Into::into).unwrap_or_default())
    })
}

pub(crate) unsafe fn native_collider_user_data(
    object: *const RprCollider,
    out: *mut RprUserData,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.user_data.into())
    })
}

pub(crate) unsafe fn native_collider_active_events(
    object: *const RprCollider,
    out: *mut u32,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.active_events().bits())
    })
}

pub(crate) unsafe fn native_collider_friction(
    object: *const RprCollider,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.friction())
    })
}

pub(crate) unsafe fn native_collider_restitution(
    object: *const RprCollider,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.restitution())
    })
}

pub(crate) unsafe fn native_collider_mass(
    object: *const RprCollider,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.mass())
    })
}

pub(crate) unsafe fn native_collider_density(
    object: *const RprCollider,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.density())
    })
}

pub(crate) unsafe fn native_collider_volume(
    object: *const RprCollider,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.volume())
    })
}

pub(crate) unsafe fn native_collider_contact_skin(
    object: *const RprCollider,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.contact_skin())
    })
}

pub(crate) unsafe fn native_collider_contact_force_event_threshold(
    object: *const RprCollider,
    out: *mut RprReal,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.contact_force_event_threshold())
    })
}

pub(crate) unsafe fn native_collider_is_sensor(
    object: *const RprCollider,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_sensor() as u32)
    })
}

pub(crate) unsafe fn native_collider_is_enabled(
    object: *const RprCollider,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.is_enabled() as u32)
    })
}

pub(crate) unsafe fn native_collider_compute_aabb(
    object: *const RprCollider,
    out: *mut RprAabb,
) -> RprStatus {
    ffi(|| unsafe {
        let a = get(object)?.0.compute_aabb();
        output(
            out,
            RprAabb {
                mins: a.mins.into(),
                maxs: a.maxs.into(),
            },
        )
    })
}

/// Returns an owned shared reference, independently freeable.
pub(crate) unsafe fn native_collider_shared_shape(
    object: *const RprCollider,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let shape = get(object)?.0.shared_shape().clone();
        output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
    })
}

pub(crate) unsafe fn native_collider_set_shape(
    object: *mut RprCollider,
    shape: *const RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        let shape = get(shape)?.0.clone();
        get_mut(object)?.0.set_shape(shape);
        Ok(())
    })
}

pub(crate) unsafe fn native_collider_set_position_wrt_parent(
    object: *mut RprCollider,
    value: RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let object = get_mut(object)?;
        object.0.set_position_wrt_parent(value);
        Ok(())
    })
}

/// Remove the collider and update its parent body mass properties. wake_up wakes the parent.
/// @ingroup colliders
#[rapier_export]
pub unsafe extern "C" fn rpr_remove_collider(
    handle: RprColliderHandle,
    wake_up: RprBool,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprColliderSet = std::ptr::addr_of_mut!((*raw).0.colliders).cast();
        let islands: *mut RprIslandManager = std::ptr::addr_of_mut!((*raw).0.islands).cast();
        let bodies: *mut RprRigidBodySet = std::ptr::addr_of_mut!((*raw).0.bodies).cast();
        let soft_bodies: *mut RprSoftBodySet = std::ptr::addr_of_mut!((*raw).0.soft_bodies).cast();

        let wake_up = boolean(wake_up)?;
        get_mut(set)?
            .0
            .remove(
                handle.raw(),
                &mut get_mut(islands)?.0,
                &mut get_mut(bodies)?.0,
                &mut get_mut(soft_bodies)?.0,
                wake_up,
            )
            .ok_or_else(missing)?;
        Ok(())
    })
}

/// Build a one-sided 2D polyline. Indices are flat pairs, as for impl_rpr_shared_shape_polyline.
#[cfg(feature = "dim2")]
pub(crate) unsafe fn impl_rpr_shared_shape_oriented_polyline(
    vertices: *const RprVector,
    vertex_count: usize,
    indices: *const u32,
    element_count: usize,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let points = input(vertices, vertex_count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        let indices = indices_array::<2>(indices, element_count, vertex_count)?;
        ensure(!indices.is_empty(), "empty mesh")?;
        let shape = ColliderBuilder::oriented_polyline(points, Some(indices)).shape;
        output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
    })
}
/// Convex polygon from an already ordered convex boundary.
#[cfg(feature = "dim2")]
pub(crate) unsafe fn impl_rpr_shared_shape_convex_polyline(
    vertices: *const RprVector,
    count: usize,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let points = input(vertices, count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        ensure(points.len() >= 3, "not enough vertices")?;
        let shape = SharedShape::convex_polyline(points)
            .ok_or_else(|| invalid("invalid convex polygon"))?;
        output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
    })
}
/// Rounded convex hull of the supplied points.
pub(crate) unsafe fn impl_rpr_shared_shape_round_convex_hull(
    vertices: *const RprVector,
    count: usize,
    border_radius: RprReal,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let points = input(vertices, count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        ensure(points.len() > rapier::math::DIM, "not enough vertices")?;
        let shape = SharedShape::round_convex_hull(&points, nonnegative(border_radius)?)
            .ok_or_else(|| invalid("degenerate convex hull"))?;
        output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
    })
}

/// Triangle mesh with Parry's TriMeshFlags bits.
pub(crate) unsafe fn impl_rpr_shared_shape_trimesh_with_flags(
    vertices: *const RprVector,
    vertex_count: usize,
    indices: *const u32,
    element_count: usize,
    flags: u32,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let points = input(vertices, vertex_count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        let indices = indices_array::<3>(indices, element_count, vertex_count)?;
        ensure(!indices.is_empty(), "empty mesh")?;
        let flags = rapier::parry::shape::TriMeshFlags::from_bits(flags as u16)
            .filter(|_| flags <= u16::MAX as u32)
            .ok_or_else(|| invalid("unknown trimesh flags"))?;
        let shape = SharedShape::trimesh_with_flags(points, indices, flags)
            .map_err(|e| invalid(e.to_string()))?;
        output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
    })
}

pub(crate) unsafe fn native_collider_active_collision_types(
    object: *const RprCollider,
    out: *mut u16,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.active_collision_types().bits())
    })
}

pub(crate) unsafe fn native_collider_active_hooks(
    object: *const RprCollider,
    out: *mut u32,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, object.0.active_hooks().bits())
    })
}

pub(crate) unsafe fn native_collider_friction_combine_rule(
    object: *const RprCollider,
    out: *mut u32,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, combine_value(object.0.friction_combine_rule()))
    })
}

pub(crate) unsafe fn native_collider_restitution_combine_rule(
    object: *const RprCollider,
    out: *mut u32,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(out, combine_value(object.0.restitution_combine_rule()))
    })
}

pub(crate) unsafe fn native_collider_position_wrt_parent(
    object: *const RprCollider,
    out: *mut RprPose,
) -> RprStatus {
    ffi(|| unsafe {
        let object = get(object)?;
        output(
            out,
            object
                .0
                .position_wrt_parent()
                .copied()
                .unwrap_or(*object.0.position())
                .into(),
        )
    })
}

pub(crate) unsafe fn native_collider_set_rotation(
    object: *mut RprCollider,
    value: RprRotation,
) -> RprStatus {
    ffi(|| unsafe {
        let value = value.raw()?;
        let object = get_mut(object)?;
        object.0.set_rotation(value);
        Ok(())
    })
}
