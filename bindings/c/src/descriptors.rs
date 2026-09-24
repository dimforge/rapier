//! Caller-owned construction descriptions. Pointers borrow input only until build/insert returns.
#![allow(non_snake_case)]
use crate::*;

/// Stack-allocated rigid-body construction data. Initialize with rpr_dynamic_rigid_body_desc,
/// rpr_fixed_rigid_body_desc, or a kinematic description constructor.
/// Copying this value is safe; it owns no resources and must never be freed by Rapier.
/// @ingroup rigid_bodies
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprRigidBodyDesc {
    /// World-space pose.
    pub position: RprPose,
    /// World-space linear velocity.
    pub linvel: RprVector,
    /// World-space angular velocity in radians per second.
    pub angvel: RprAngVector,
    /// RPR_DYNAMIC, RPR_FIXED, RPR_KINEMATIC_POSITION_BASED, or RPR_KINEMATIC_VELOCITY_BASED.
    pub bodyType: u32,
    /// Multiplier applied to world gravity.
    pub gravityScale: RprReal,
    /// Nonnegative linear damping coefficient.
    pub linearDamping: RprReal,
    /// Nonnegative angular damping coefficient.
    pub angularDamping: RprReal,
    /// Nonnegative mass added to attached collider contributions.
    pub additionalMass: RprReal,
    /// 1 uses additionalMassProperties; 0 uses additionalMass.
    pub useAdditionalMassProperties: RprBool,
    /// Additional body-local mass and inertia when enabled.
    pub additionalMassProperties: RprMassProperties,
    /// Locked-axis bitmask; translations precede rotations.
    pub lockedAxes: u8,
    /// Whether automatic sleeping is allowed.
    pub canSleep: RprBool,
    /// Whether the body starts/is asleep.
    pub sleeping: RprBool,
    /// Whether continuous collision detection is enabled.
    pub ccdEnabled: RprBool,
    /// Nonnegative prediction distance for soft CCD.
    pub softCcdPrediction: RprReal,
    /// Whether to allow fast rotations without the native angular-motion clamp.
    pub allowFastRotation: RprBool,
    /// Whether this setting/object is enabled (0 or 1).
    pub enabled: RprBool,
    /// Signed dominance group; larger groups dominate smaller groups.
    pub dominanceGroup: i8,
    /// Extra solver iterations for this body and connected bodies.
    pub additionalSolverIterations: usize,
    /// Extra PGS iterations for this body.
    pub additionalPgsIterations: usize,
    /// Whether to include gyroscopic forces (3D).
    pub gyroscopicForcesEnabled: RprBool,
    /// Application data; Rapier does not own pointers encoded in it.
    pub userData: RprUserData,
}
impl RprRigidBodyDesc {
    fn new(kind: u32) -> Result<Self> {
        let b = RigidBodyBuilder::new(body_type(kind)?);
        Ok(Self {
            position: b.position.into(),
            linvel: b.linvel.into(),
            angvel: angular_out(b.angvel),
            bodyType: kind,
            gravityScale: b.gravity_scale,
            linearDamping: b.linear_damping,
            angularDamping: b.angular_damping,
            additionalMass: 0.0,
            useAdditionalMassProperties: 0,
            additionalMassProperties: MassProperties::default().into(),
            lockedAxes: 0,
            canSleep: b.can_sleep as _,
            sleeping: b.sleeping as _,
            ccdEnabled: b.ccd_enabled as _,
            softCcdPrediction: b.soft_ccd_prediction,
            allowFastRotation: b.allow_fast_rotation as _,
            enabled: b.enabled as _,
            dominanceGroup: b.dominance_group,
            additionalSolverIterations: b.additional_solver_iterations,
            additionalPgsIterations: b.additional_pgs_iterations,
            gyroscopicForcesEnabled: b.gyroscopic_forces_enabled as _,
            userData: b.user_data.into(),
        })
    }
    pub(crate) fn raw(&self) -> Result<RigidBodyBuilder> {
        let axes =
            LockedAxes::from_bits(self.lockedAxes).ok_or_else(|| invalid("unknown locked axes"))?;
        let mut b = RigidBodyBuilder::new(body_type(self.bodyType)?)
            .pose(self.position.raw()?)
            .linvel(self.linvel.raw()?)
            .angvel(angular(self.angvel)?)
            .gravity_scale(finite(self.gravityScale)?)
            .linear_damping(nonnegative(self.linearDamping)?)
            .angular_damping(nonnegative(self.angularDamping)?)
            .locked_axes(axes)
            .can_sleep(boolean(self.canSleep)?)
            .sleeping(boolean(self.sleeping)?)
            .ccd_enabled(boolean(self.ccdEnabled)?)
            .soft_ccd_prediction(nonnegative(self.softCcdPrediction)?)
            .allow_fast_rotation(boolean(self.allowFastRotation)?)
            .enabled(boolean(self.enabled)?)
            .dominance_group(self.dominanceGroup)
            .additional_solver_iterations(self.additionalSolverIterations)
            .additional_pgs_iterations(self.additionalPgsIterations)
            .user_data(self.userData.raw());
        b = if boolean(self.useAdditionalMassProperties)? {
            b.additional_mass_properties(self.additionalMassProperties.raw()?)
        } else {
            b.additional_mass(nonnegative(self.additionalMass)?)
        };
        let gyro = boolean(self.gyroscopicForcesEnabled)?;
        #[cfg(feature = "dim3")]
        {
            b = b.gyroscopic_forces_enabled(gyro);
        }
        #[cfg(feature = "dim2")]
        let _ = gyro;
        Ok(b)
    }
}
/// Return a dynamic rigid-body description with native defaults; no allocation.
/// @ingroup rigid_bodies
#[rapier_export]
pub extern "C" fn rpr_dynamic_rigid_body_desc() -> RprRigidBodyDesc {
    RprRigidBodyDesc::new(RPR_DYNAMIC).expect("valid body kind")
}

/// Return a fixed rigid-body description with native defaults; no allocation.
/// @ingroup rigid_bodies
#[rapier_export]
pub extern "C" fn rpr_fixed_rigid_body_desc() -> RprRigidBodyDesc {
    RprRigidBodyDesc::new(RPR_FIXED).expect("valid body kind")
}

/// Return a kinematic position based rigid-body description with native defaults; no allocation.
/// @ingroup rigid_bodies
#[rapier_export]
pub extern "C" fn rpr_kinematic_position_based_rigid_body_desc() -> RprRigidBodyDesc {
    RprRigidBodyDesc::new(RPR_KINEMATIC_POSITION_BASED).expect("valid body kind")
}

/// Return a kinematic velocity based rigid-body description with native defaults; no allocation.
/// @ingroup rigid_bodies
#[rapier_export]
pub extern "C" fn rpr_kinematic_velocity_based_rigid_body_desc() -> RprRigidBodyDesc {
    RprRigidBodyDesc::new(RPR_KINEMATIC_VELOCITY_BASED).expect("valid body kind")
}

/// @ingroup shapes
/// Treat the 2D polyline as oriented when generating contact normals.
#[cfg(feature = "dim2")]
pub const RPR_POLYLINE_ORIENTED: u32 = 1;
/// @ingroup shapes
/// Prepare polyline acceleration data for deformation.
pub const RPR_POLYLINE_DEFORMABLE: u32 = 2;

/// @ingroup shapes
/// ShapeDesc kind selecting a ball.
pub const RPR_SHAPE_DESC_BALL: u32 = 0;
/// @ingroup shapes
/// ShapeDesc kind selecting a cuboid.
pub const RPR_SHAPE_DESC_CUBOID: u32 = 1;
/// @ingroup shapes
/// ShapeDesc kind selecting a round cuboid.
pub const RPR_SHAPE_DESC_ROUND_CUBOID: u32 = 2;
/// @ingroup shapes
/// ShapeDesc kind selecting a capsule.
pub const RPR_SHAPE_DESC_CAPSULE: u32 = 3;
/// @ingroup shapes
/// ShapeDesc kind selecting a segment.
pub const RPR_SHAPE_DESC_SEGMENT: u32 = 4;
/// @ingroup shapes
/// ShapeDesc kind selecting a triangle.
pub const RPR_SHAPE_DESC_TRIANGLE: u32 = 5;
/// @ingroup shapes
/// ShapeDesc kind selecting a half-space.
pub const RPR_SHAPE_DESC_HALFSPACE: u32 = 6;
/// @ingroup shapes
/// ShapeDesc kind selecting a convex hull.
pub const RPR_SHAPE_DESC_CONVEX_HULL: u32 = 7;
/// @ingroup shapes
/// ShapeDesc kind selecting a triangle mesh.
pub const RPR_SHAPE_DESC_TRIMESH: u32 = 8;
/// @ingroup shapes
/// ShapeDesc kind selecting a polyline.
pub const RPR_SHAPE_DESC_POLYLINE: u32 = 9;
/// @ingroup shapes
/// ShapeDesc kind selecting borrowed shared geometry.
pub const RPR_SHAPE_DESC_SHARED: u32 = 10;
/// @ingroup shapes
/// ShapeDesc kind selecting a heightfield.
pub const RPR_SHAPE_DESC_HEIGHTFIELD: u32 = 11;
/// @ingroup shapes
/// ShapeDesc kind selecting a cylinder.
pub const RPR_SHAPE_DESC_CYLINDER: u32 = 12;
/// @ingroup shapes
/// ShapeDesc kind selecting a cone.
pub const RPR_SHAPE_DESC_CONE: u32 = 13;
/// @ingroup shapes
/// ShapeDesc kind selecting compound child shapes.
pub const RPR_SHAPE_DESC_COMPOUND: u32 = 14;
/// @ingroup shapes
/// ShapeDesc kind selecting a round cylinder.
pub const RPR_SHAPE_DESC_ROUND_CYLINDER: u32 = 15;
/// @ingroup shapes
/// ShapeDesc kind selecting a round cone.
pub const RPR_SHAPE_DESC_ROUND_CONE: u32 = 16;

/// Non-owning shape description. Only fields selected by kind are read.
/// a = cuboid half extents, capsule/segment endpoint, triangle vertex, or halfspace normal.
/// b/c = remaining endpoints/vertices. radius is also the rounded-cuboid border radius.
/// Mesh views count edges or triangles; heightfields are column-major.
/// Arrays, compound children, and sharedShape remain borrowed until build/insert returns.
/// @ingroup shapes
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RprShapeDesc {
    /// Discriminant selecting which description fields are read.
    pub kind: u32,
    /// Half extents, endpoint/vertex, or halfspace normal, selected by kind.
    pub a: RprVector,
    /// Second endpoint or triangle vertex, selected by kind.
    pub b: RprVector,
    /// Third triangle vertex.
    pub c: RprVector,
    /// Radius for the selected primitive or recipe.
    pub radius: RprReal,
    /// Half the height of a cylinder or cone.
    pub halfHeight: RprReal,
    /// Rounding radius of a round cylinder or round cone (the round cuboid reads radius instead).
    pub borderRadius: RprReal,
    /// Borrowed vertex positions.
    pub vertices: RprVectorView,
    /// Borrowed triangle topology; count is triangles.
    pub triangles: RprTriangleView,
    /// Borrowed edge topology; count is edges.
    pub edges: RprEdgeView,
    /// TRIMESH_*, POLYLINE_*, or HEIGHTFIELD_* bitmask selected by kind.
    pub flags: u32,
    /// Borrowed height samples; 3D uses column-major rows * columns samples.
    pub heights: RprRealView,
    /// Number of heightfield rows.
    pub rows: usize,
    /// Number of heightfield columns.
    pub columns: usize,
    /// Shape scale along each axis.
    pub scale: RprVector,
    /// Borrowed shared geometry; keep its wrapper alive through build/insert.
    pub sharedShape: *const RprSharedShape,
    /// Borrowed compound children and their nested geometry views.
    pub children: RprCompoundShapeView,
}
/// One compound child with a local pose and borrowed geometry description.
/// @ingroup shapes
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprCompoundShapeDesc {
    /// Pose relative to the compound parent.
    pub pose: RprPose,
    /// Non-owning shape description; build/insert consumes its views synchronously.
    pub shape: RprShapeDesc,
}
impl Default for RprShapeDesc {
    fn default() -> Self {
        Self {
            kind: RPR_SHAPE_DESC_BALL,
            a: Vector::ZERO.into(),
            b: Vector::ZERO.into(),
            c: Vector::ZERO.into(),
            radius: 0.5,
            halfHeight: 0.5,
            borderRadius: 0.0,
            vertices: RprVectorView::default(),
            triangles: RprTriangleView::default(),
            edges: RprEdgeView::default(),
            flags: 0,
            heights: RprRealView::default(),
            rows: 0,
            columns: 0,
            scale: Vector::ONE.into(),
            sharedShape: std::ptr::null(),
            children: RprCompoundShapeView::default(),
        }
    }
}
impl RprShapeDesc {
    pub(crate) unsafe fn raw(&self) -> Result<SharedShape> {
        unsafe { self.raw_depth(0) }
    }
    unsafe fn raw_depth(&self, depth: usize) -> Result<SharedShape> {
        ensure(depth < 32, "compound shape nesting exceeds 32 levels")?;
        Ok(match self.kind {
            RPR_SHAPE_DESC_BALL => SharedShape::ball(positive(self.radius)?),
            RPR_SHAPE_DESC_CUBOID | RPR_SHAPE_DESC_ROUND_CUBOID => {
                let v = self.a.raw()?;
                ensure(v.min_element() > 0.0, "half extents must be positive")?;
                if self.kind == RPR_SHAPE_DESC_CUBOID {
                    #[cfg(feature = "dim2")]
                    {
                        SharedShape::cuboid(v.x, v.y)
                    }
                    #[cfg(feature = "dim3")]
                    {
                        SharedShape::cuboid(v.x, v.y, v.z)
                    }
                } else {
                    let r = nonnegative(self.radius)?;
                    #[cfg(feature = "dim2")]
                    {
                        SharedShape::round_cuboid(v.x, v.y, r)
                    }
                    #[cfg(feature = "dim3")]
                    {
                        SharedShape::round_cuboid(v.x, v.y, v.z, r)
                    }
                }
            }
            RPR_SHAPE_DESC_CAPSULE => {
                SharedShape::capsule(self.a.raw()?, self.b.raw()?, positive(self.radius)?)
            }
            RPR_SHAPE_DESC_SEGMENT => SharedShape::segment(self.a.raw()?, self.b.raw()?),
            RPR_SHAPE_DESC_TRIANGLE => {
                SharedShape::triangle(self.a.raw()?, self.b.raw()?, self.c.raw()?)
            }
            RPR_SHAPE_DESC_HALFSPACE => {
                let n = self.a.raw()?;
                positive(n.length())?;
                SharedShape::halfspace(n.normalize())
            }
            #[cfg(feature = "dim3")]
            RPR_SHAPE_DESC_ROUND_CYLINDER => SharedShape::round_cylinder(
                positive(self.halfHeight)?,
                positive(self.radius)?,
                nonnegative(self.borderRadius)?,
            ),
            #[cfg(feature = "dim3")]
            RPR_SHAPE_DESC_ROUND_CONE => SharedShape::round_cone(
                positive(self.halfHeight)?,
                positive(self.radius)?,
                nonnegative(self.borderRadius)?,
            ),
            RPR_SHAPE_DESC_SHARED => unsafe { get(self.sharedShape)?.0.clone() },
            RPR_SHAPE_DESC_COMPOUND => {
                let mut shapes = Vec::new();
                for child in unsafe { input(self.children.data, self.children.count)? } {
                    shapes.push((child.pose.raw()?, unsafe {
                        child.shape.raw_depth(depth + 1)?
                    }));
                }
                ensure(!shapes.is_empty(), "empty compound shape")?;
                SharedShape::compound(shapes)
            }
            RPR_SHAPE_DESC_CONVEX_HULL | RPR_SHAPE_DESC_TRIMESH | RPR_SHAPE_DESC_POLYLINE => {
                let vertices: Vec<_> = unsafe { input(self.vertices.data, self.vertices.count)? }
                    .iter()
                    .map(|v| v.raw())
                    .collect::<Result<_>>()?;
                ensure(!vertices.is_empty(), "empty vertices")?;
                if self.kind == RPR_SHAPE_DESC_CONVEX_HULL {
                    SharedShape::convex_hull(&vertices)
                        .ok_or_else(|| invalid("degenerate convex hull"))?
                } else {
                    let (data, count, arity) = if self.kind == RPR_SHAPE_DESC_TRIMESH {
                        (self.triangles.data.cast::<u32>(), self.triangles.count, 3)
                    } else {
                        (self.edges.data.cast::<u32>(), self.edges.count, 2)
                    };
                    let count = count
                        .checked_mul(arity)
                        .ok_or_else(|| invalid("index count overflow"))?;
                    let indices = unsafe { input(data, count)? };
                    ensure(
                        indices.iter().all(|i| (*i as usize) < vertices.len()),
                        "mesh index out of bounds",
                    )?;
                    if self.kind == RPR_SHAPE_DESC_TRIMESH {
                        let flags = rapier::parry::shape::TriMeshFlags::from_bits(
                            u16::try_from(self.flags)
                                .map_err(|_| invalid("unknown trimesh flags"))?,
                        )
                        .ok_or_else(|| invalid("unknown trimesh flags"))?;
                        SharedShape::trimesh_with_flags(
                            vertices,
                            indices
                                .chunks_exact(3)
                                .map(|v| [v[0], v[1], v[2]])
                                .collect(),
                            flags,
                        )
                        .map_err(|e| invalid(e.to_string()))?
                    } else {
                        let flags = rapier::parry::shape::PolylineFlags::from_bits(
                            u8::try_from(self.flags)
                                .map_err(|_| invalid("unknown polyline flags"))?,
                        )
                        .ok_or_else(|| invalid("unknown polyline flags"))?;
                        SharedShape::new(rapier::parry::shape::Polyline::with_flags(
                            vertices,
                            if indices.is_empty() {
                                None
                            } else {
                                Some(indices.chunks_exact(2).map(|v| [v[0], v[1]]).collect())
                            },
                            flags,
                        ))
                    }
                }
            }
            #[cfg(feature = "dim3")]
            RPR_SHAPE_DESC_CYLINDER => {
                SharedShape::cylinder(positive(self.halfHeight)?, positive(self.radius)?)
            }
            #[cfg(feature = "dim3")]
            RPR_SHAPE_DESC_CONE => {
                SharedShape::cone(positive(self.halfHeight)?, positive(self.radius)?)
            }
            RPR_SHAPE_DESC_HEIGHTFIELD => {
                let count = self
                    .rows
                    .checked_mul(self.columns)
                    .ok_or_else(|| invalid("heightfield size overflow"))?;
                ensure(
                    self.heights.count == count,
                    "heightfield data length does not match dimensions",
                )?;
                let heights = unsafe { input(self.heights.data, self.heights.count)? };
                for &value in heights {
                    finite(value)?;
                }
                let scale = self.scale.raw()?;
                ensure(
                    scale.min_element() > 0.0,
                    "heightfield scale must be positive",
                )?;
                #[cfg(feature = "dim2")]
                {
                    ensure(
                        self.rows >= 2 && self.columns == 1,
                        "invalid 2D heightfield dimensions",
                    )?;
                    SharedShape::heightfield(heights.to_vec(), scale)
                }
                #[cfg(feature = "dim3")]
                {
                    ensure(
                        self.rows >= 2 && self.columns >= 2,
                        "invalid 3D heightfield dimensions",
                    )?;
                    SharedShape::heightfield_with_flags(
                        rapier::parry::utils::Array2::new(
                            self.rows,
                            self.columns,
                            heights.to_vec(),
                        ),
                        scale,
                        rapier::parry::shape::HeightFieldFlags::from_bits(
                            u8::try_from(self.flags)
                                .map_err(|_| invalid("unknown heightfield flags"))?,
                        )
                        .ok_or_else(|| invalid("unknown heightfield flags"))?,
                    )
                }
            }
            _ => return Err(invalid("unknown or unsupported shape description")),
        })
    }
}
/// Return native default shape desc. This POD value owns no resources.
/// @ingroup shapes
#[rapier_export]
pub extern "C" fn rpr_default_shape_desc() -> RprShapeDesc {
    RprShapeDesc::default()
}
/// Build an owned shared shape from a description; release it with rpr_free_shared_shape. Borrowed
/// inputs may be released after this call.
/// @ingroup shapes
#[rapier_export(shape_desc)]
pub unsafe extern "C" fn rpr_shape_desc_build(desc: *const RprShapeDesc) -> *mut RprSharedShape {
    ffi_value(|out: *mut *mut RprSharedShape| {
        ffi(|| unsafe {
            out_ptr(out)?;
            let shape = get(desc)?.raw()?;
            output(out, Box::into_raw(Box::new(RprSharedShape(shape))))
        })
    })
}

/// @ingroup colliders
/// Mass density.
pub const RPR_MASS_DENSITY: u32 = 0;
/// @ingroup colliders
/// Mass total.
pub const RPR_MASS_TOTAL: u32 = 1;
/// @ingroup colliders
/// Mass properties.
pub const RPR_MASS_PROPERTIES: u32 = 2;
/// Copyable collider construction data. Shape inputs are borrowed, never owned.
/// @ingroup colliders
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RprColliderDesc {
    /// Non-owning shape description; build/insert consumes its views synchronously.
    pub shape: RprShapeDesc,
    /// Pose relative to the parent body; world-space for a collider without a parent.
    pub position: RprPose,
    /// RPR_MASS_DENSITY, RPR_MASS_TOTAL, or RPR_MASS_PROPERTIES.
    pub massMode: u32,
    /// Nonnegative mass per unit volume.
    pub density: RprReal,
    /// Mass; nonnegative when supplied as input.
    pub mass: RprReal,
    /// Explicit local mass and inertia when massMode selects them.
    pub massProperties: RprMassProperties,
    /// Nonnegative friction coefficient.
    pub friction: RprReal,
    /// Nonnegative restitution coefficient.
    pub restitution: RprReal,
    /// RPR_COMBINE_AVERAGE, MIN, MULTIPLY, MAX, CLAMPED_SUM, or GEOMETRIC_MEAN.
    pub frictionCombineRule: u32,
    /// RPR_COMBINE_AVERAGE, MIN, MULTIPLY, MAX, CLAMPED_SUM, or GEOMETRIC_MEAN.
    pub restitutionCombineRule: u32,
    /// 1 detects intersections without generating contact forces.
    pub isSensor: RprBool,
    /// Whether this setting/object is enabled (0 or 1).
    pub enabled: RprBool,
    /// Groups controlling collision detection.
    pub collisionGroups: RprInteractionGroups,
    /// Groups controlling contact-force solving.
    pub solverGroups: RprInteractionGroups,
    /// Bitmask of body-type pairs allowed to collide.
    pub activeCollisionTypes: u16,
    /// Hook flags enabling pair filtering/contact modification.
    pub activeHooks: u32,
    /// RPR_COLLISION_EVENTS and/or RPR_CONTACT_FORCE_EVENTS.
    pub activeEvents: u32,
    /// Nonnegative force threshold for force events.
    pub contactForceEventThreshold: RprReal,
    /// Nonnegative extra separation distance around the collider.
    pub contactSkin: RprReal,
    /// Application data; Rapier does not own pointers encoded in it.
    pub userData: RprUserData,
}
impl Default for RprColliderDesc {
    fn default() -> Self {
        Self {
            shape: RprShapeDesc::default(),
            position: Pose::IDENTITY.into(),
            massMode: RPR_MASS_DENSITY,
            density: 1.0,
            mass: 0.0,
            massProperties: MassProperties::default().into(),
            friction: ColliderBuilder::default_friction(),
            restitution: 0.0,
            frictionCombineRule: 0,
            restitutionCombineRule: 0,
            isSensor: 0,
            enabled: 1,
            collisionGroups: InteractionGroups::all().into(),
            solverGroups: InteractionGroups::all().into(),
            activeCollisionTypes: ActiveCollisionTypes::default().bits(),
            activeHooks: 0,
            activeEvents: 0,
            contactForceEventThreshold: 0.0,
            contactSkin: 0.0,
            userData: 0u128.into(),
        }
    }
}
impl RprColliderDesc {
    pub(crate) unsafe fn raw(&self) -> Result<ColliderBuilder> {
        let mut b = ColliderBuilder::new(unsafe { self.shape.raw()? })
            .position(self.position.raw()?)
            .friction(nonnegative(self.friction)?)
            .restitution(nonnegative(self.restitution)?)
            .sensor(boolean(self.isSensor)?)
            .enabled(boolean(self.enabled)?)
            .collision_groups(self.collisionGroups.raw()?)
            .solver_groups(self.solverGroups.raw()?)
            .user_data(self.userData.raw())
            .friction_combine_rule(combine(self.frictionCombineRule)?)
            .restitution_combine_rule(combine(self.restitutionCombineRule)?)
            .active_collision_types(
                ActiveCollisionTypes::from_bits(self.activeCollisionTypes)
                    .ok_or_else(|| invalid("unknown collision types"))?,
            )
            .active_hooks(
                ActiveHooks::from_bits(self.activeHooks).ok_or_else(|| invalid("unknown hooks"))?,
            )
            .active_events(
                ActiveEvents::from_bits(self.activeEvents)
                    .ok_or_else(|| invalid("unknown events"))?,
            )
            .contact_force_event_threshold(nonnegative(self.contactForceEventThreshold)?)
            .contact_skin(nonnegative(self.contactSkin)?);
        b = match self.massMode {
            RPR_MASS_DENSITY => b.density(nonnegative(self.density)?),
            RPR_MASS_TOTAL => b.mass(nonnegative(self.mass)?),
            RPR_MASS_PROPERTIES => b.mass_properties(self.massProperties.raw()?),
            _ => return Err(invalid("unknown mass mode")),
        };
        Ok(b)
    }
}
/// Return native default collider desc. This POD value owns no resources.
/// @ingroup colliders
#[rapier_export]
pub extern "C" fn rpr_default_collider_desc() -> RprColliderDesc {
    RprColliderDesc::default()
}
/// Return a ball description with the supplied radius.
/// Returns a description without allocating or validating. Build/insert validates its fields.
/// @ingroup colliders
#[rapier_export]
pub extern "C" fn rpr_ball_collider_desc(radius: RprReal) -> RprColliderDesc {
    let mut d = RprColliderDesc::default();
    d.shape.radius = radius;
    d
}
/// Return an axis-aligned box description with the supplied half-extents.
/// Returns a description without allocating or validating. Build/insert validates its fields.
/// @ingroup colliders
#[rapier_export]
pub extern "C" fn rpr_cuboid_collider_desc(half_extents: RprVector) -> RprColliderDesc {
    let mut d = RprColliderDesc::default();
    d.shape.kind = RPR_SHAPE_DESC_CUBOID;
    d.shape.a = half_extents;
    d
}
/// Create a body from the description and return its world-bound handle. The world owns the body.
/// @ingroup rigid_bodies
#[rapier_export]
pub unsafe extern "C" fn rpr_insert_rigid_body(
    world: *mut RprWorld,
    desc: *const RprRigidBodyDesc,
) -> RprRigidBodyHandle {
    ffi_world_value(world, |out: *mut RprRigidBodyHandle| {
        ffi(|| unsafe {
            let access = get(world)?.write()?;
            let raw = access.raw();

            let world: *mut RprPhysicsWorld = raw;

            if !out.is_null() {
                out_ptr(out)?;
            }
            let body = get(desc)?.raw()?.build();
            let h = get_mut(world)?.0.insert_body(body);
            if !out.is_null() {
                output(out, h.into())?;
            }
            Ok(())
        })
    })
}

/// Insert a collider attached to a rigid body, using the world stored in its handle.
/// The parent handle is copied by value. The description is borrowed through this call.
/// Invalid or removed parents fail without inserting a collider.
/// @ingroup colliders
#[rapier_export]
pub unsafe extern "C" fn rpr_insert_collider(
    parent: RprRigidBodyHandle,
    desc: *const RprColliderDesc,
) -> RprColliderHandle {
    let world = parent.world;
    ffi_world_value(world, |out| {
        ffi(|| unsafe {
            if world.is_null() {
                return Err(missing());
            }
            let access = get(world)?.write()?;
            let world = &mut (*access.raw()).0;
            let parent = parent.raw();
            world.bodies.get(parent).ok_or_else(missing)?;
            let collider = get(desc)?.raw()?.build();
            output(out, world.insert_collider(collider, Some(parent)).into())
        })
    })
}

/// Insert a collider without a rigid-body parent. The world owns the collider.
/// The description is borrowed through this call.
/// @ingroup colliders
#[rapier_export]
pub unsafe extern "C" fn rpr_insert_collider_without_parent(
    world: *mut RprWorld,
    desc: *const RprColliderDesc,
) -> RprColliderHandle {
    ffi_world_value(world, |out| {
        ffi(|| unsafe {
            let access = get(world)?.write()?;
            let world = &mut (*access.raw()).0;
            let collider = get(desc)?.raw()?.build();
            output(out, world.insert_collider(collider, None).into())
        })
    })
}

/// Sizes of the POD types in this library build, for foreign-language layout checks.
/// @ingroup math
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct RprPodLayout {
    /// Size in bytes of the rigidBodyDesc structure; zero when unavailable.
    pub rigidBodyDesc: usize,
    /// Size in bytes of the colliderDesc structure; zero when unavailable.
    pub colliderDesc: usize,
    /// Size in bytes of the shapeDesc structure; zero when unavailable.
    pub shapeDesc: usize,
    /// Size in bytes of the jointDesc structure; zero when unavailable.
    pub jointDesc: usize,
    /// Size in bytes of the softBodyMaterial structure; zero when unavailable.
    pub softBodyMaterial: usize,
    /// Size in bytes of the integrationParameters structure; zero when unavailable.
    pub integrationParameters: usize,
    /// Size in bytes of the softBodyDesc structure; zero when unavailable.
    pub softBodyDesc: usize,
    /// Size in bytes of the softMeshBindingDesc structure; zero when unavailable.
    pub softMeshBindingDesc: usize,
    /// Size in bytes of the queryOptions structure; zero when unavailable.
    pub queryOptions: usize,
    /// Zero unless 3D f32 robotics is enabled.
    pub urdfLoaderOptions: usize,
    /// Zero unless 3D f32 robotics is enabled.
    pub mjcfLoaderOptions: usize,
}
/// Return POD structure sizes for checking foreign-language layouts against this library.
/// @ingroup errors
#[rapier_export]
pub extern "C" fn rpr_pod_layout() -> RprPodLayout {
    RprPodLayout {
        rigidBodyDesc: size_of::<RprRigidBodyDesc>(),
        colliderDesc: size_of::<RprColliderDesc>(),
        shapeDesc: size_of::<RprShapeDesc>(),
        jointDesc: size_of::<RprJointDesc>(),
        softBodyMaterial: size_of::<RprSoftBodyMaterial>(),
        integrationParameters: size_of::<RprIntegrationParameters>(),
        softBodyDesc: size_of::<RprSoftBodyDesc>(),
        softMeshBindingDesc: size_of::<RprSoftMeshBindingDesc>(),
        queryOptions: size_of::<RprQueryOptions>(),
        #[cfg(all(feature = "robotics", feature = "dim3", feature = "f32"))]
        urdfLoaderOptions: size_of::<RprUrdfLoaderOptions>(),
        #[cfg(not(all(feature = "robotics", feature = "dim3", feature = "f32")))]
        urdfLoaderOptions: 0,
        #[cfg(all(feature = "robotics", feature = "dim3", feature = "f32"))]
        mjcfLoaderOptions: size_of::<RprMjcfLoaderOptions>(),
        #[cfg(not(all(feature = "robotics", feature = "dim3", feature = "f32")))]
        mjcfLoaderOptions: 0,
    }
}
