use crate::*;
use rapier::geometry::ContactPair;
/// Explicit mass and principal inertia, matching MassProperties constructors. Zero mass/inertia means infinite.
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprMassProperties {
    pub local_com: RprVector,
    pub mass: RprReal,
    pub principal_inertia: RprAngVector,
    #[cfg(feature = "dim3")]
    pub principal_inertia_local_frame: RprRotation,
}
impl RprMassProperties {
    pub(crate) fn raw(self) -> Result<MassProperties> {
        let com = self.local_com.raw()?;
        nonnegative(self.mass)?;
        #[cfg(feature = "dim2")]
        {
            Ok(MassProperties::new(
                com,
                self.mass,
                nonnegative(self.principal_inertia)?,
            ))
        }
        #[cfg(feature = "dim3")]
        {
            let i = self.principal_inertia.raw()?;
            ensure(i.min_element() >= 0.0, "negative inertia")?;
            Ok(MassProperties::with_principal_inertia_frame(
                com,
                self.mass,
                i,
                self.principal_inertia_local_frame.raw()?,
            ))
        }
    }
}
impl From<MassProperties> for RprMassProperties {
    fn from(m: MassProperties) -> Self {
        Self {
            local_com: m.local_com.into(),
            mass: m.mass(),
            principal_inertia: angular_out(m.principal_inertia()),
            #[cfg(feature = "dim3")]
            principal_inertia_local_frame: m.principal_inertia_local_frame.into(),
        }
    }
}
pub(crate) unsafe fn native_rigid_body_set_additional_mass_properties(
    body: *mut RprRigidBody,
    properties: RprMassProperties,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let p = properties.raw()?;
        let wake_up = boolean(wake_up)?;
        get_mut(body)?.0.set_additional_mass_properties(p, wake_up);
        Ok(())
    })
}
pub(crate) unsafe fn native_rigid_body_recompute_mass_properties_from_colliders(
    body: *mut RprRigidBody,
    colliders: *const RprColliderSet,
) -> RprStatus {
    ffi(|| unsafe {
        get_mut(body)?
            .0
            .recompute_mass_properties_from_colliders(&get(colliders)?.0);
        Ok(())
    })
}
pub(crate) unsafe fn native_collider_set_mass_properties(
    collider: *mut RprCollider,
    properties: RprMassProperties,
) -> RprStatus {
    ffi(|| unsafe {
        let p = properties.raw()?;
        get_mut(collider)?.0.set_mass_properties(p);
        Ok(())
    })
}
pub(crate) unsafe fn native_collider_mass_properties(
    collider: *const RprCollider,
    out: *mut RprMassProperties,
) -> RprStatus {
    ffi(|| unsafe { output(out, get(collider)?.0.mass_properties().into()) })
}
pub(crate) unsafe fn native_rigid_body_set_locked_axes(
    body: *mut RprRigidBody,
    axes: u8,
    wake_up: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let a = LockedAxes::from_bits(axes).ok_or_else(|| invalid("unknown locked axes"))?;
        let w = boolean(wake_up)?;
        get_mut(body)?.0.set_locked_axes(a, w);
        Ok(())
    })
}
pub(crate) unsafe fn native_rigid_body_locked_axes(
    body: *const RprRigidBody,
    out: *mut u8,
) -> RprStatus {
    ffi(|| unsafe { output(out, get(body)?.0.locked_axes().bits()) })
}

#[rapier_export]
pub unsafe extern "C" fn rpr_heightfield_shared_shape(
    heights: RprRealView,
    rows: usize,
    columns: usize,
    scale: RprVector,
) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let n = rows
                .checked_mul(columns)
                .ok_or_else(|| invalid("heightfield dimensions overflow"))?;
            ensure(
                heights.count == n,
                "heightfield data length does not match dimensions",
            )?;
            let data = input(heights.data, heights.count)?;
            for &h in data {
                finite(h)?;
            }
            let scale = scale.raw()?;
            ensure(
                scale.min_element() > 0.0,
                "heightfield scale must be positive",
            )?;
            #[cfg(feature = "dim2")]
            let shape = {
                ensure(
                    rows >= 2 && columns == 1,
                    "2D heightfield requires rows>=2 and columns=1",
                )?;
                SharedShape::heightfield(data.to_vec(), scale)
            };
            #[cfg(feature = "dim3")]
            let shape = {
                ensure(
                    rows >= 2 && columns >= 2,
                    "heightfield dimensions must be >=2",
                )?;
                SharedShape::heightfield(
                    rapier::parry::utils::Array2::new(rows, columns, data.to_vec()),
                    scale,
                )
            };
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}
/// Vertex indices have DIM entries per element. Uses Rapier's default decomposition parameters.
pub(crate) unsafe fn impl_rpr_shared_shape_convex_decomposition(
    vertices: *const RprVector,
    vertex_count: usize,
    indices: *const u32,
    element_count: usize,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let p = input(vertices, vertex_count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        let i = crate::geometry::indices_array::<{ rapier::math::DIM }>(
            indices,
            element_count,
            vertex_count,
        )?;
        ensure(!i.is_empty(), "empty mesh")?;
        let shape = SharedShape::convex_decomposition(&p, &i);
        output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
    })
}
pub(crate) unsafe fn impl_rpr_shared_shape_voxels_from_points(
    voxel_size: RprVector,
    points: *const RprVector,
    count: usize,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        let size = voxel_size.raw()?;
        ensure(size.min_element() > 0.0, "voxel size must be positive")?;
        let p = input(points, count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        ensure(!p.is_empty(), "no voxel points")?;
        output(
            out,
            Box::into_raw(Box::new(RprSharedShape(SharedShape::voxels_from_points(
                size, &p,
            )))),
        )
    })
}
#[rapier_export(shared_shape)]
pub unsafe extern "C" fn rpr_shared_shape_compute_aabb(
    shape: *const RprSharedShape,
    pose: RprPose,
) -> RprAabb {
    ffi_value(|out: *mut RprAabb| {
        ffi(|| unsafe {
            let a = get(shape)?.0.compute_aabb(&pose.raw()?);
            output(
                out,
                RprAabb {
                    mins: a.mins.into(),
                    maxs: a.maxs.into(),
                },
            )
        })
    })
}
#[rapier_export(shared_shape)]
pub unsafe extern "C" fn rpr_shared_shape_mass_properties(
    shape: *const RprSharedShape,
    density: RprReal,
) -> RprMassProperties {
    ffi_value(|out: *mut RprMassProperties| {
        ffi(|| unsafe {
            nonnegative(density)?;
            output(out, get(shape)?.0.mass_properties(density).into())
        })
    })
}
#[rapier_export(shared_shape)]
pub unsafe extern "C" fn rpr_shared_shape_contains_point(
    shape: *const RprSharedShape,
    pose: RprPose,
    point: RprVector,
) -> RprBool {
    ffi_value(|out: *mut RprBool| {
        ffi(|| unsafe {
            let p = pose.raw()?;
            let point = point.raw()?;
            output(out, get(shape)?.0.contains_point(&p, point) as u32)
        })
    })
}
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprContactPair {
    pub collider1: RprColliderHandle,
    pub collider2: RprColliderHandle,
    pub has_any_active_contact: RprBool,
    pub total_impulse: RprVector,
    pub total_impulse_magnitude: RprReal,
    pub max_impulse: RprReal,
    pub max_impulse_direction: RprVector,
}
impl From<&ContactPair> for RprContactPair {
    fn from(p: &ContactPair) -> Self {
        let (max, dir) = p.max_impulse();
        Self {
            collider1: p.collider1.into(),
            collider2: p.collider2.into(),
            has_any_active_contact: p.has_any_active_contact() as u32,
            total_impulse: p.total_impulse().into(),
            total_impulse_magnitude: p.total_impulse_magnitude(),
            max_impulse: max,
            max_impulse_direction: dir.into(),
        }
    }
}
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprIntersectionPair {
    pub collider1: RprColliderHandle,
    pub collider2: RprColliderHandle,
    pub intersecting: RprBool,
}
#[rapier_export]
pub unsafe extern "C" fn rpr_contact_pairs(
    world: *const RprWorld,
    buffer: *mut RprContactPair,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();

                let narrow: *const RprNarrowPhase =
                    std::ptr::addr_of!((*raw).0.narrow_phase).cast();

                let v: Vec<_> = get(narrow)?.0.contact_pairs().map(Into::into).collect();
                copy_out(&v, buffer, capacity, count)
            })
        })
    }
}

#[rapier_export]
pub unsafe extern "C" fn rpr_contact_pair(
    collider1: RprColliderHandle,
    collider2: RprColliderHandle,
) -> RprContactPair {
    let world = collider1.world;
    ffi_world_value(world, |out: *mut RprContactPair| {
        ffi(|| unsafe {
            collider1.check_world(world)?;
            collider2.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let narrow: *const RprNarrowPhase = std::ptr::addr_of!((*raw).0.narrow_phase).cast();

            let p = get(narrow)?
                .0
                .contact_pair(collider1.raw(), collider2.raw())
                .ok_or((RPR_NOT_FOUND, "no contact pair".into()))?;
            output(out, p.into())
        })
    })
}

#[rapier_export]
pub unsafe extern "C" fn rpr_intersection_pairs(
    world: *const RprWorld,
    buffer: *mut RprIntersectionPair,
    capacity: usize,
) -> usize {
    unsafe {
        ffi_world_array(world, buffer, capacity, |count: *mut usize| {
            ffi(|| {
                let access = get(world)?.read()?;
                let raw = access.raw();

                let narrow: *const RprNarrowPhase =
                    std::ptr::addr_of!((*raw).0.narrow_phase).cast();

                let v: Vec<_> = get(narrow)?
                    .0
                    .intersection_pairs()
                    .map(|(a, b, hit)| RprIntersectionPair {
                        collider1: a.into(),
                        collider2: b.into(),
                        intersecting: hit as u32,
                    })
                    .collect();
                copy_out(&v, buffer, capacity, count)
            })
        })
    }
}

#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprContactPoint {
    pub manifold_index: usize,
    pub local_p1: RprVector,
    pub local_p2: RprVector,
    pub normal: RprVector,
    pub distance: RprReal,
    pub impulse: RprReal,
}
/// Contact points in collider-local space; normal in world space. Geometric manifolds may be recycled.
/// For clustered solver impulses use contact pair totals. Soft pairs have no rigid manifolds.
#[rapier_export]
pub unsafe extern "C" fn rpr_contact_points(
    collider1: RprColliderHandle,
    collider2: RprColliderHandle,
    buffer: *mut RprContactPoint,
    capacity: usize,
) -> usize {
    let world = collider1.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            collider1.check_world(world)?;
            collider2.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let narrow: *const RprNarrowPhase = std::ptr::addr_of!((*raw).0.narrow_phase).cast();

            let p = get(narrow)?
                .0
                .contact_pair(collider1.raw(), collider2.raw())
                .ok_or((RPR_NOT_FOUND, "no contact pair".into()))?;
            let v: Vec<_> = p
                .manifolds()
                .iter()
                .enumerate()
                .flat_map(|(i, m)| {
                    m.points.iter().map(move |p| RprContactPoint {
                        manifold_index: i,
                        local_p1: p.local_p1.into(),
                        local_p2: p.local_p2.into(),
                        normal: m.data.normal.into(),
                        distance: p.dist,
                        impulse: p.data.impulse,
                    })
                })
                .collect();
            copy_out(&v, buffer, capacity, count)
        })
    })
}

#[rapier_export(multibody_joint)]
pub unsafe extern "C" fn rpr_multibody_joint_generalized_velocity(
    handle: RprMultibodyJointHandle,
    buffer: *mut RprReal,
    capacity: usize,
) -> usize {
    let world = handle.world;
    ffi_value(|count: *mut usize| {
        ffi(|| unsafe {
            handle.check_world(world)?;
            let access = get(world)?.read()?;
            let raw = access.raw();

            let set: *const RprMultibodyJointSet =
                std::ptr::addr_of!((*raw).0.multibody_joints).cast();

            let (m, _) = get(set)?.0.get(handle.raw()).ok_or_else(missing)?;
            copy_out(m.generalized_velocity().as_slice(), buffer, capacity, count)
        })
    })
}

#[rapier_export(multibody_joint)]
pub unsafe extern "C" fn rpr_multibody_joint_set_generalized_velocity(
    handle: RprMultibodyJointHandle,
    values: *const RprReal,
    count: usize,
) -> RprStatus {
    let world = handle.world;
    ffi(|| unsafe {
        handle.check_world(world)?;
        let access = get(world)?.write()?;
        let raw = access.raw();

        let set: *mut RprMultibodyJointSet =
            std::ptr::addr_of_mut!((*raw).0.multibody_joints).cast();

        let v = input(values, count)?;
        for &x in v {
            finite(x)?;
        }
        let (m, _) = get_mut(set)?.0.get_mut(handle.raw()).ok_or_else(missing)?;
        ensure(
            m.ndofs() == count,
            "velocity count must equal articulation dofs",
        )?;
        m.generalized_velocity_mut()
            .as_mut_slice()
            .copy_from_slice(v);
        Ok(())
    })
}

/// Check this before passing any dimension/precision-dependent structs across the ABI.
#[rapier_export]
pub unsafe extern "C" fn rpr_check_abi(
    version: u32,
    dimension: u32,
    real_size: usize,
    vector_size: usize,
    pose_size: usize,
) -> RprStatus {
    ffi(|| {
        ensure(
            version == RPR_ABI_VERSION
                && dimension == rapier::math::DIM as u32
                && real_size == std::mem::size_of::<Real>()
                && vector_size == std::mem::size_of::<RprVector>()
                && pose_size == std::mem::size_of::<RprPose>(),
            "header/library ABI mismatch",
        )
    })
}

/// Voxelize a boundary mesh with the native default solid-fill mode.
/// Indices contain DIM entries per boundary element.
pub(crate) unsafe fn impl_rpr_shared_shape_voxelized_mesh(
    vertices: *const RprVector,
    vertex_count: usize,
    indices: *const u32,
    element_count: usize,
    voxel_size: RprReal,
    out: *mut *mut RprSharedShape,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(out)?;
        positive(voxel_size)?;
        let points = input(vertices, vertex_count)?
            .iter()
            .copied()
            .map(RprVector::raw)
            .collect::<Result<Vec<_>>>()?;
        ensure(!points.is_empty(), "empty voxelization mesh")?;
        let indices = crate::geometry::indices_array::<{ rapier::math::DIM }>(
            indices,
            element_count,
            points.len(),
        )?;
        output(
            out,
            Box::into_raw(Box::new(RprSharedShape(SharedShape::voxelized_mesh(
                &points,
                &indices,
                voxel_size,
                Default::default(),
            )))),
        )
    })
}

pub(crate) unsafe fn native_collider_is_voxels(
    collider: *const RprCollider,
    out: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        output(
            out,
            get(collider)?.0.shape().as_voxels().is_some() as RprBool,
        )
    })
}
/// Voxel coordinates have DIM signed integer components.
#[cfg(feature = "f32")]
pub type RprVoxelCoord = i32;
#[cfg(feature = "f64")]
pub type RprVoxelCoord = i64;
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprVoxelKey {
    pub x: RprVoxelCoord,
    pub y: RprVoxelCoord,
    #[cfg(feature = "dim3")]
    pub z: RprVoxelCoord,
}
impl RprVoxelKey {
    fn raw(self) -> rapier::math::IVector {
        #[cfg(feature = "dim2")]
        {
            rapier::math::IVector::new(self.x, self.y)
        }
        #[cfg(feature = "dim3")]
        {
            rapier::math::IVector::new(self.x, self.y, self.z)
        }
    }
}
pub(crate) unsafe fn native_collider_voxel_at_flat_id(
    collider: *const RprCollider,
    id: u32,
    key: *mut RprVoxelKey,
    center: *mut RprVector,
    size: *mut RprVector,
    found: *mut RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        out_ptr(key)?;
        out_ptr(center)?;
        out_ptr(size)?;
        out_ptr(found)?;
        let voxels = get(collider)?
            .0
            .shape()
            .as_voxels()
            .ok_or_else(|| invalid("not a voxel collider"))?;
        output(size, voxels.voxel_size().into())?;
        if let Some(k) = voxels.voxel_at_flat_id(id) {
            output(
                key,
                RprVoxelKey {
                    x: k.x,
                    y: k.y,
                    #[cfg(feature = "dim3")]
                    z: k.z,
                },
            )?;
            output(center, voxels.voxel_center(k).into())?;
            output(found, 1)
        } else {
            output(key, Default::default())?;
            output(center, Default::default())?;
            output(found, 0)
        }
    })
}
pub(crate) unsafe fn native_collider_set_voxel(
    collider: *mut RprCollider,
    key: RprVoxelKey,
    filled: RprBool,
) -> RprStatus {
    ffi(|| unsafe {
        let filled = boolean(filled)?;
        get_mut(collider)?
            .0
            .shape_mut()
            .as_voxels_mut()
            .ok_or_else(|| invalid("not a voxel collider"))?
            .set_voxel(key.raw(), filled);
        Ok(())
    })
}
