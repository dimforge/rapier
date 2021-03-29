#![allow(dead_code)]

use na::{
    Isometry3, Matrix3, Matrix4, Point3, Quaternion, Rotation3, Translation3, Unit, UnitQuaternion,
    Vector3,
};
use physx::cooking::{
    ConvexMeshCookingResult, PxConvexMeshDesc, PxCooking, PxCookingParams, PxHeightFieldDesc,
    PxTriangleMeshDesc, TriangleMeshCookingResult,
};
use physx::foundation::DefaultAllocator;
use physx::prelude::*;
use physx::scene::FrictionType;
use physx::traits::Class;
use physx_sys::{
    FilterShaderCallbackInfo, PxBitAndByte, PxConvexFlags, PxConvexMeshGeometryFlags,
    PxHeightFieldSample, PxMeshGeometryFlags, PxMeshScale_new, PxRigidActor,
};
use rapier::counters::Counters;
use rapier::dynamics::{
    IntegrationParameters, JointParams, JointSet, RigidBodyHandle, RigidBodySet,
};
use rapier::geometry::{Collider, ColliderSet};
use rapier::utils::WBasis;
use std::collections::HashMap;

trait IntoNa {
    type Output;
    fn into_na(self) -> Self::Output;
}

impl IntoNa for glam::Mat4 {
    type Output = Matrix4<f32>;
    fn into_na(self) -> Self::Output {
        self.to_cols_array_2d().into()
    }
}

impl IntoNa for PxVec3 {
    type Output = Vector3<f32>;
    fn into_na(self) -> Self::Output {
        Vector3::new(self.x(), self.y(), self.z())
    }
}

impl IntoNa for PxQuat {
    type Output = Quaternion<f32>;
    fn into_na(self) -> Self::Output {
        Quaternion::new(self.w(), self.x(), self.y(), self.z())
    }
}

impl IntoNa for PxTransform {
    type Output = Isometry3<f32>;
    fn into_na(self) -> Self::Output {
        let tra = self.translation().into_na();
        let quat = self.rotation().into_na();
        let unit_quat = Unit::new_unchecked(quat);
        Isometry3::from_parts(tra.into(), unit_quat)
    }
}

trait IntoPhysx {
    type Output;
    fn into_physx(self) -> Self::Output;
}

impl IntoPhysx for Vector3<f32> {
    type Output = PxVec3;
    fn into_physx(self) -> Self::Output {
        PxVec3::new(self.x, self.y, self.z)
    }
}

impl IntoPhysx for Point3<f32> {
    type Output = PxVec3;
    fn into_physx(self) -> Self::Output {
        PxVec3::new(self.x, self.y, self.z)
    }
}

impl IntoPhysx for Isometry3<f32> {
    type Output = PxTransform;
    fn into_physx(self) -> Self::Output {
        self.into_glam().into()
    }
}

trait IntoGlam {
    type Output;
    fn into_glam(self) -> Self::Output;
}

impl IntoGlam for Vector3<f32> {
    type Output = glam::Vec3;
    fn into_glam(self) -> Self::Output {
        glam::vec3(self.x, self.y, self.z)
    }
}

impl IntoGlam for Point3<f32> {
    type Output = glam::Vec3;
    fn into_glam(self) -> Self::Output {
        glam::vec3(self.x, self.y, self.z)
    }
}

impl IntoGlam for Matrix4<f32> {
    type Output = glam::Mat4;
    fn into_glam(self) -> Self::Output {
        glam::Mat4::from_cols_array_2d(&self.into())
    }
}

impl IntoGlam for Isometry3<f32> {
    type Output = glam::Mat4;
    fn into_glam(self) -> Self::Output {
        glam::Mat4::from_cols_array_2d(&self.to_homogeneous().into())
    }
}

thread_local! {
pub static FOUNDATION: std::cell::RefCell<PxPhysicsFoundation> = std::cell::RefCell::new(PhysicsFoundation::default());
}

pub struct PhysxWorld {
    // physics: Physics,
    // cooking: Cooking,
    materials: Vec<Owner<PxMaterial>>,
    shapes: Vec<Owner<PxShape>>,
    scene: Option<Owner<PxScene>>,
}

impl Drop for PhysxWorld {
    fn drop(&mut self) {
        let scene = self.scene.take();
        // FIXME: we get a segfault if we don't forget the scene.
        std::mem::forget(scene);
    }
}

impl PhysxWorld {
    pub fn from_rapier(
        gravity: Vector3<f32>,
        integration_parameters: &IntegrationParameters,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        joints: &JointSet,
        use_two_friction_directions: bool,
        num_threads: usize,
    ) -> Self {
        FOUNDATION.with(|physics| {
            let mut physics = physics.borrow_mut();
            let mut shapes = Vec::new();
            let mut materials = Vec::new();

            let friction_type = if use_two_friction_directions {
                FrictionType::TwoDirectional
            } else {
                FrictionType::Patch
            };

            let mut scene_desc = SceneDescriptor {
                gravity: gravity.into_physx(),
                thread_count: num_threads as u32,
                broad_phase_type: BroadPhaseType::AutomaticBoxPruning,
                solver_type: SolverType::PGS,
                friction_type,
                ccd_max_passes: integration_parameters.max_ccd_substeps as u32,
                ..SceneDescriptor::new(())
            };

            let ccd_enabled = bodies.iter().any(|(_, rb)| rb.is_ccd_enabled());

            if ccd_enabled {
                scene_desc.simulation_filter_shader =
                    FilterShaderDescriptor::CallDefaultFirst(ccd_filter_shader);
                scene_desc.flags.insert(SceneFlag::EnableCcd);
            }

            let mut scene: Owner<PxScene> = physics.create(scene_desc).unwrap();
            let mut rapier2dynamic = HashMap::new();
            let mut rapier2static = HashMap::new();
            let cooking_params =
                PxCookingParams::new(&*physics).expect("Failed to init PhysX cooking.");
            let mut cooking = PxCooking::new(physics.foundation_mut(), &cooking_params)
                .expect("Failed to init PhysX cooking");

            /*
             *
             * Rigid bodies
             *
             */
            for (rapier_handle, rb) in bodies.iter() {
                let pos = rb.position().into_physx();
                if rb.is_dynamic() {
                    let mut actor = physics.create_dynamic(&pos, rapier_handle).unwrap();
                    let linvel = rb.linvel().into_physx();
                    let angvel = rb.angvel().into_physx();
                    actor.set_linear_velocity(&linvel, true);
                    actor.set_angular_velocity(&angvel, true);
                    actor.set_solver_iteration_counts(
                        integration_parameters.max_position_iterations as u32,
                        integration_parameters.max_velocity_iterations as u32,
                    );

                    rapier2dynamic.insert(rapier_handle, actor);
                } else {
                    let actor = physics.create_static(pos, ()).unwrap();
                    rapier2static.insert(rapier_handle, actor);
                }
            }

            /*
             *
             * Colliders
             *
             */
            for (_, collider) in colliders.iter() {
                if let Some((mut px_shape, px_material, collider_pos)) =
                    physx_collider_from_rapier_collider(&mut *physics, &mut cooking, &collider)
                {
                    let parent_body = &bodies[collider.parent()];

                    if !parent_body.is_dynamic() {
                        let actor = rapier2static.get_mut(&collider.parent()).unwrap();
                        actor.attach_shape(&mut px_shape);
                    } else {
                        let actor = rapier2dynamic.get_mut(&collider.parent()).unwrap();
                        actor.attach_shape(&mut px_shape);
                    }

                    unsafe {
                        let pose = collider_pos.into_physx();
                        physx_sys::PxShape_setLocalPose_mut(px_shape.as_mut_ptr(), &pose.into());
                    }

                    shapes.push(px_shape);
                    materials.push(px_material);
                }
            }

            // Update mass properties and CCD flags.
            for (rapier_handle, actor) in rapier2dynamic.iter_mut() {
                let rb = &bodies[*rapier_handle];
                let densities: Vec<_> = rb
                    .colliders()
                    .iter()
                    .map(|h| colliders[*h].density().unwrap_or(0.0))
                    .collect();

                unsafe {
                    physx_sys::PxRigidBodyExt_updateMassAndInertia_mut(
                        std::mem::transmute(actor.as_mut()),
                        densities.as_ptr(),
                        densities.len() as u32,
                        std::ptr::null(),
                        false,
                    );

                    if rb.is_ccd_enabled() {
                        physx_sys::PxRigidBody_setRigidBodyFlag_mut(
                            std::mem::transmute(actor.as_mut()),
                            RigidBodyFlag::EnableCCD as u32,
                            true,
                        );
                        // physx_sys::PxRigidBody_setMinCCDAdvanceCoefficient_mut(
                        //     std::mem::transmute(actor.as_mut()),
                        //     0.0,
                        // );
                        // physx_sys::PxRigidBody_setRigidBodyFlag_mut(
                        //     std::mem::transmute(actor.as_mut()),
                        //     RigidBodyFlag::EnableCCDFriction as u32,
                        //     true,
                        // );
                    }
                }
            }

            /*
             *
             * Joints
             *
             */
            Self::setup_joints(
                &mut physics,
                joints,
                &mut rapier2static,
                &mut rapier2dynamic,
            );

            for (_, actor) in rapier2static {
                scene.add_static_actor(actor);
            }

            for (_, actor) in rapier2dynamic {
                scene.add_dynamic_actor(actor);
            }

            Self {
                scene: Some(scene),
                shapes,
                materials,
            }
        })
    }

    fn setup_joints(
        physics: &mut PxPhysicsFoundation,
        joints: &JointSet,
        rapier2static: &mut HashMap<RigidBodyHandle, Owner<PxRigidStatic>>,
        rapier2dynamic: &mut HashMap<RigidBodyHandle, Owner<PxRigidDynamic>>,
    ) {
        unsafe {
            for joint in joints.iter() {
                let actor1 = rapier2static
                    .get_mut(&joint.1.body1)
                    .map(|act| &mut **act as *mut PxRigidStatic as *mut PxRigidActor)
                    .or(rapier2dynamic
                        .get_mut(&joint.1.body1)
                        .map(|act| &mut **act as *mut PxRigidDynamic as *mut PxRigidActor))
                    .unwrap();
                let actor2 = rapier2static
                    .get_mut(&joint.1.body2)
                    .map(|act| &mut **act as *mut PxRigidStatic as *mut PxRigidActor)
                    .or(rapier2dynamic
                        .get_mut(&joint.1.body2)
                        .map(|act| &mut **act as *mut PxRigidDynamic as *mut PxRigidActor))
                    .unwrap();

                match &joint.1.params {
                    JointParams::BallJoint(params) => {
                        let frame1 = Isometry3::new(params.local_anchor1.coords, na::zero())
                            .into_physx()
                            .into();
                        let frame2 = Isometry3::new(params.local_anchor2.coords, na::zero())
                            .into_physx()
                            .into();

                        physx_sys::phys_PxSphericalJointCreate(
                            physics.as_mut_ptr(),
                            actor1,
                            &frame1 as *const _,
                            actor2,
                            &frame2 as *const _,
                        );
                    }
                    JointParams::RevoluteJoint(params) => {
                        // NOTE: orthonormal_basis() returns the two basis vectors.
                        // However we only use one and recompute the other just to
                        // make sure our basis is right-handed.
                        let basis1a = params.local_axis1.orthonormal_basis()[0];
                        let basis2a = params.local_axis2.orthonormal_basis()[0];
                        let basis1b = params.local_axis1.cross(&basis1a);
                        let basis2b = params.local_axis2.cross(&basis2a);

                        let rotmat1 = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[
                            params.local_axis1.into_inner(),
                            basis1a,
                            basis1b,
                        ]));
                        let rotmat2 = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[
                            params.local_axis2.into_inner(),
                            basis2a,
                            basis2b,
                        ]));
                        let axisangle1 = rotmat1.scaled_axis();
                        let axisangle2 = rotmat2.scaled_axis();

                        let frame1 = Isometry3::new(params.local_anchor1.coords, axisangle1)
                            .into_physx()
                            .into();
                        let frame2 = Isometry3::new(params.local_anchor2.coords, axisangle2)
                            .into_physx()
                            .into();

                        let revolute_joint = physx_sys::phys_PxRevoluteJointCreate(
                            physics.as_mut_ptr(),
                            actor1,
                            &frame1 as *const _,
                            actor2,
                            &frame2 as *const _,
                        );

                        physx_sys::PxRevoluteJoint_setDriveVelocity_mut(
                            revolute_joint,
                            params.motor_target_vel,
                            true,
                        );

                        if params.motor_damping != 0.0 {
                            physx_sys::PxRevoluteJoint_setRevoluteJointFlag_mut(
                                revolute_joint,
                                physx_sys::PxRevoluteJointFlag::eDRIVE_ENABLED,
                                true,
                            );
                        }
                    }

                    JointParams::PrismaticJoint(params) => {
                        // NOTE: orthonormal_basis() returns the two basis vectors.
                        // However we only use one and recompute the other just to
                        // make sure our basis is right-handed.
                        let basis1a = params.local_axis1().orthonormal_basis()[0];
                        let basis2a = params.local_axis2().orthonormal_basis()[0];
                        let basis1b = params.local_axis1().cross(&basis1a);
                        let basis2b = params.local_axis2().cross(&basis2a);

                        let rotmat1 = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[
                            params.local_axis1().into_inner(),
                            basis1a,
                            basis1b,
                        ]));
                        let rotmat2 = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[
                            params.local_axis2().into_inner(),
                            basis2a,
                            basis2b,
                        ]));

                        let axisangle1 = rotmat1.scaled_axis();
                        let axisangle2 = rotmat2.scaled_axis();

                        let frame1 = Isometry3::new(params.local_anchor1.coords, axisangle1)
                            .into_physx()
                            .into();
                        let frame2 = Isometry3::new(params.local_anchor2.coords, axisangle2)
                            .into_physx()
                            .into();

                        let joint = physx_sys::phys_PxPrismaticJointCreate(
                            physics.as_mut_ptr(),
                            actor1,
                            &frame1 as *const _,
                            actor2,
                            &frame2 as *const _,
                        );

                        if params.limits_enabled {
                            let limits = physx_sys::PxJointLinearLimitPair {
                                restitution: 0.0,
                                bounceThreshold: 0.0,
                                stiffness: 0.0,
                                damping: 0.0,
                                contactDistance: 0.01,
                                lower: params.limits[0],
                                upper: params.limits[1],
                            };
                            physx_sys::PxPrismaticJoint_setLimit_mut(joint, &limits);
                            physx_sys::PxPrismaticJoint_setPrismaticJointFlag_mut(
                                joint,
                                physx_sys::PxPrismaticJointFlag::eLIMIT_ENABLED,
                                true,
                            );
                        }
                    }
                    JointParams::FixedJoint(params) => {
                        let frame1 = params.local_anchor1.into_physx().into();
                        let frame2 = params.local_anchor2.into_physx().into();

                        physx_sys::phys_PxFixedJointCreate(
                            physics.as_mut_ptr(),
                            actor1,
                            &frame1 as *const _,
                            actor2,
                            &frame2 as *const _,
                        );
                    } // JointParams::GenericJoint(_) => {
                      //     eprintln!(
                      //         "Joint type currently unsupported by the PhysX backend: GenericJoint."
                      //     )
                      // }
                }
            }
        }
    }

    pub fn step(&mut self, counters: &mut Counters, params: &IntegrationParameters) {
        let mut scratch = unsafe { ScratchBuffer::new(4) };

        counters.step_started();
        self.scene
            .as_mut()
            .unwrap()
            .step(
                params.dt,
                None::<&mut physx_sys::PxBaseTask>,
                Some(&mut scratch),
                true,
            )
            .expect("error occurred during PhysX simulation");
        counters.step_completed();
    }

    pub fn sync(&mut self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
        for actor in self.scene.as_mut().unwrap().get_dynamic_actors() {
            let handle = actor.get_user_data();
            let pos = actor.get_global_pose().into_na();
            let rb = &mut bodies[*handle];
            rb.set_position(pos, false);

            for coll_handle in rb.colliders() {
                let collider = &mut colliders[*coll_handle];
                collider.set_position_debug(pos * collider.position_wrt_parent());
            }
        }
    }
}

fn physx_collider_from_rapier_collider(
    physics: &mut PxPhysicsFoundation,
    cooking: &PxCooking,
    collider: &Collider,
) -> Option<(Owner<PxShape>, Owner<PxMaterial>, Isometry3<f32>)> {
    let mut local_pose = *collider.position_wrt_parent();
    let shape = collider.shape();
    let shape_flags = if collider.is_sensor() {
        ShapeFlag::TriggerShape.into()
    } else {
        ShapeFlag::SimulationShape.into()
    };
    let mut material = physics
        .create_material(
            collider.friction,
            collider.friction,
            collider.restitution,
            (),
        )
        .unwrap();
    let materials = &mut [material.as_mut()][..];

    let shape = if let Some(cuboid) = shape.as_cuboid() {
        let geometry = PxBoxGeometry::new(
            cuboid.half_extents.x,
            cuboid.half_extents.y,
            cuboid.half_extents.z,
        );
        physics.create_shape(&geometry, materials, true, shape_flags, ())
    } else if let Some(ball) = shape.as_ball() {
        let geometry = PxSphereGeometry::new(ball.radius);
        physics.create_shape(&geometry, materials, true, shape_flags, ())
    } else if let Some(capsule) = shape.as_capsule() {
        let center = capsule.center();
        let mut dir = capsule.segment.b - capsule.segment.a;

        if dir.x < 0.0 {
            dir = -dir;
        }

        let rot = UnitQuaternion::rotation_between(&Vector3::x(), &dir);
        local_pose = local_pose
            * Translation3::from(center.coords)
            * rot.unwrap_or(UnitQuaternion::identity());
        let geometry = PxCapsuleGeometry::new(capsule.radius, capsule.half_height());
        physics.create_shape(&geometry, materials, true, shape_flags, ())
    } else if let Some(heightfield) = shape.as_heightfield() {
        let heights = heightfield.heights();
        let scale = heightfield.scale();
        local_pose = local_pose * Translation3::new(-scale.x / 2.0, 0.0, -scale.z / 2.0);
        const Y_FACTOR: f32 = 1_000f32;
        let mut heightfield_desc;
        unsafe {
            let samples: Vec<_> = heights
                .iter()
                .map(|h| PxHeightFieldSample {
                    height: (*h * Y_FACTOR) as i16,
                    materialIndex0: PxBitAndByte { mData: 0 },
                    materialIndex1: PxBitAndByte { mData: 0 },
                })
                .collect();
            heightfield_desc = physx_sys::PxHeightFieldDesc_new();
            heightfield_desc.nbRows = heights.nrows() as u32;
            heightfield_desc.nbColumns = heights.ncols() as u32;
            heightfield_desc.samples.stride = std::mem::size_of::<PxHeightFieldSample>() as u32;
            heightfield_desc.samples.data = samples.as_ptr() as *const std::ffi::c_void;
        }

        let heightfield_desc = PxHeightFieldDesc {
            obj: heightfield_desc,
        };
        let heightfield = cooking.create_height_field(physics, &heightfield_desc);

        if let Some(mut heightfield) = heightfield {
            let flags = PxMeshGeometryFlags {
                mBits: physx_sys::PxMeshGeometryFlag::eDOUBLE_SIDED as u8,
            };
            let geometry = PxHeightFieldGeometry::new(
                &mut *heightfield,
                flags,
                scale.y / Y_FACTOR,
                scale.x / (heights.nrows() as f32 - 1.0),
                scale.z / (heights.ncols() as f32 - 1.0),
            );
            physics.create_shape(&geometry, materials, true, shape_flags, ())
        } else {
            eprintln!("PhysX heightfield construction failed.");
            return None;
        }
    } else if let Some(convex) = shape
        .as_convex_polyhedron()
        .or(shape.as_round_convex_polyhedron().map(|c| &c.base_shape))
    {
        let vertices = convex.points();
        let mut convex_desc;
        unsafe {
            convex_desc = physx_sys::PxConvexMeshDesc_new();
            convex_desc.points.count = vertices.len() as u32;
            convex_desc.points.stride = (3 * std::mem::size_of::<f32>()) as u32;
            convex_desc.points.data = vertices.as_ptr() as *const std::ffi::c_void;
            convex_desc.flags = PxConvexFlags {
                mBits: physx_sys::PxConvexFlag::eCOMPUTE_CONVEX as u16,
            };
        }

        let convex_desc = PxConvexMeshDesc { obj: convex_desc };
        let convex = cooking.create_convex_mesh(physics, &convex_desc);

        if let ConvexMeshCookingResult::Success(mut convex) = convex {
            let flags = PxConvexMeshGeometryFlags { mBits: 0 };
            let scaling = unsafe { PxMeshScale_new() };
            let geometry = PxConvexMeshGeometry::new(&mut convex, &scaling, flags);
            physics.create_shape(&geometry, materials, true, shape_flags, ())
        } else {
            eprintln!("PhysX convex mesh construction failed.");
            return None;
        }
    } else if let Some(trimesh) = shape.as_trimesh() {
        let vertices = trimesh.vertices();
        let indices = trimesh.flat_indices();

        let mut mesh_desc;
        unsafe {
            mesh_desc = physx_sys::PxTriangleMeshDesc_new();

            mesh_desc.points.count = trimesh.vertices().len() as u32;
            mesh_desc.points.stride = (3 * std::mem::size_of::<f32>()) as u32;
            mesh_desc.points.data = vertices.as_ptr() as *const std::ffi::c_void;

            mesh_desc.triangles.count = (indices.len() as u32) / 3;
            mesh_desc.triangles.stride = (3 * std::mem::size_of::<u32>()) as u32;
            mesh_desc.triangles.data = indices.as_ptr() as *const std::ffi::c_void;
        }

        let mesh_desc = PxTriangleMeshDesc { obj: mesh_desc };
        let trimesh = cooking.create_triangle_mesh(physics, &mesh_desc);

        if let TriangleMeshCookingResult::Success(mut trimesh) = trimesh {
            let flags = PxMeshGeometryFlags {
                mBits: physx_sys::PxMeshGeometryFlag::eDOUBLE_SIDED as u8,
            };

            let scaling = unsafe { PxMeshScale_new() };
            let geometry = PxTriangleMeshGeometry::new(&mut trimesh, &scaling, flags);
            physics.create_shape(&geometry, materials, true, shape_flags, ())
        } else {
            eprintln!("PhysX triangle mesh construction failed.");
            return None;
        }
    } else {
        eprintln!("Creating a shape unknown to the PhysX backend.");
        return None;
    };

    shape.map(|s| (s, material, local_pose))
}

type PxPhysicsFoundation = PhysicsFoundation<DefaultAllocator, PxShape>;
type PxMaterial = physx::material::PxMaterial<()>;
type PxShape = physx::shape::PxShape<(), PxMaterial>;
type PxArticulationLink = physx::articulation_link::PxArticulationLink<(), PxShape>;
type PxRigidStatic = physx::rigid_static::PxRigidStatic<(), PxShape>;
type PxRigidDynamic = physx::rigid_dynamic::PxRigidDynamic<RigidBodyHandle, PxShape>;
type PxArticulation = physx::articulation::PxArticulation<(), PxArticulationLink>;
type PxArticulationReducedCoordinate =
    physx::articulation_reduced_coordinate::PxArticulationReducedCoordinate<(), PxArticulationLink>;
type PxScene = physx::scene::PxScene<
    (),
    PxArticulationLink,
    PxRigidStatic,
    PxRigidDynamic,
    PxArticulation,
    PxArticulationReducedCoordinate,
    OnCollision,
    OnTrigger,
    OnConstraintBreak,
    OnWakeSleep,
    OnAdvance,
>;

/// Next up, the simulation event callbacks need to be defined, and possibly an
/// allocator callback as well.
struct OnCollision;
impl CollisionCallback for OnCollision {
    fn on_collision(
        &mut self,
        _header: &physx_sys::PxContactPairHeader,
        _pairs: &[physx_sys::PxContactPair],
    ) {
    }
}
struct OnTrigger;
impl TriggerCallback for OnTrigger {
    fn on_trigger(&mut self, _pairs: &[physx_sys::PxTriggerPair]) {}
}

struct OnConstraintBreak;
impl ConstraintBreakCallback for OnConstraintBreak {
    fn on_constraint_break(&mut self, _constraints: &[physx_sys::PxConstraintInfo]) {}
}
struct OnWakeSleep;
impl WakeSleepCallback<PxArticulationLink, PxRigidStatic, PxRigidDynamic> for OnWakeSleep {
    fn on_wake_sleep(
        &mut self,
        _actors: &[&physx::actor::ActorMap<PxArticulationLink, PxRigidStatic, PxRigidDynamic>],
        _is_waking: bool,
    ) {
    }
}

struct OnAdvance;
impl AdvanceCallback<PxArticulationLink, PxRigidDynamic> for OnAdvance {
    fn on_advance(
        &self,
        _actors: &[&physx::rigid_body::RigidBodyMap<PxArticulationLink, PxRigidDynamic>],
        _transforms: &[PxTransform],
    ) {
    }
}

unsafe extern "C" fn ccd_filter_shader(data: *mut FilterShaderCallbackInfo) -> u16 {
    (*(*data).pairFlags).mBits |= physx_sys::PxPairFlag::eDETECT_CCD_CONTACT as u16;
    0
}
