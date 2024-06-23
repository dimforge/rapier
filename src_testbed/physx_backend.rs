#![allow(dead_code)]

use na::{Isometry3, Matrix4, Point3, Quaternion, Translation3, Unit, UnitQuaternion, Vector3};
use physx::cooking::{
    ConvexMeshCookingResult, PxConvexMeshDesc, PxCookingParams, PxHeightFieldDesc,
    PxTriangleMeshDesc, TriangleMeshCookingResult,
};
use physx::foundation::DefaultAllocator;
use physx::prelude::*;
use physx::scene::{FrictionType, SceneFlags};
use physx::traits::Class;
use physx_sys::PxFilterFlags;
use physx_sys::{
    FilterShaderCallbackInfo, PxBitAndByte, PxConvexFlags, PxConvexMeshGeometryFlags,
    PxHeightFieldSample, PxMeshGeometryFlags, PxMeshScale_new, PxRigidActor,
};
use rapier::counters::Counters;
use rapier::dynamics::{
    ImpulseJointSet, IntegrationParameters, MultibodyJointSet, RigidBodyHandle, RigidBodySet,
};
use rapier::geometry::{Collider, ColliderSet};
use rapier::prelude::JointAxesMask;
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

impl IntoPhysx for UnitQuaternion<f32> {
    type Output = PxQuat;
    fn into_physx(self) -> Self::Output {
        PxQuat::new(self.i, self.j, self.k, self.w)
    }
}

impl IntoPhysx for Isometry3<f32> {
    type Output = PxTransform;
    fn into_physx(self) -> Self::Output {
        PxTransform::from_translation_rotation(
            &self.translation.vector.into_physx(),
            &self.rotation.into_physx(),
        )
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
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
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
                broad_phase_type: BroadPhaseType::Abp,
                solver_type: SolverType::Pgs,
                friction_type,
                ccd_max_passes: integration_parameters.max_ccd_substeps as u32,
                ..SceneDescriptor::new(())
            };

            let ccd_enabled = bodies.iter().any(|(_, rb)| rb.is_ccd_enabled());

            if ccd_enabled {
                scene_desc.simulation_filter_shader =
                    FilterShaderDescriptor::CallDefaultFirst(ccd_filter_shader);
                scene_desc.flags.insert(SceneFlags::EnableCcd);
            }

            let mut scene: Owner<PxScene> = physics.create(scene_desc).unwrap();
            let mut rapier2dynamic = HashMap::new();
            let mut rapier2static = HashMap::new();
            let mut rapier2link = HashMap::new();

            /*
             *
             * Rigid bodies
             *
             */
            for (rapier_handle, rb) in bodies.iter() {
                if multibody_joints.rigid_body_link(rapier_handle).is_some() {
                    continue;
                };

                let pos = rb.position().into_physx();
                if rb.is_dynamic() {
                    let mut actor = physics.create_dynamic(&pos, rapier_handle).unwrap();
                    let linvel = rb.linvel().into_physx();
                    let angvel = rb.angvel().into_physx();
                    actor.set_linear_velocity(&linvel, true);
                    actor.set_angular_velocity(&angvel, true);
                    actor.set_solver_iteration_counts(
                        // Use our number of solver iterations as their number of position iterations.
                        integration_parameters.num_solver_iterations.get() as u32,
                        1,
                    );

                    rapier2dynamic.insert(rapier_handle, actor);
                } else {
                    let actor = physics.create_static(pos, ()).unwrap();
                    rapier2static.insert(rapier_handle, actor);
                }
            }

            /*
             * Articulations.
             */
            /*
            for multibody in multibody_joints.multibodies() {
                let mut articulation: Owner<PxArticulationReducedCoordinate> =
                    physics.create_articulation_reduced_coordinate(()).unwrap();
                let mut parent = None;

                for link in multibody.links() {
                    let is_root = parent.is_none();
                    let rb_handle = link.rigid_body_handle();
                    let rb = bodies.get(rb_handle).unwrap();

                    if is_root && rb.is_fixed() {
                        articulation.set_articulation_flag(ArticulationFlag::FixBase, true);
                    }

                    let link_pose = rb.position().into_physx();
                    let px_link = articulation
                        .create_link(parent.take(), &link_pose, rb_handle)
                        .unwrap();

                    // TODO: there is no get_inbound_joint_mut?
                    if let Some(px_inbound_joint) =
                        unsafe { (PxArticulationLink_getInboundJoint(px_link.as_ptr())).as_mut() }
                    {
                        let frame1 = link.joint().data.local_frame1.into_physx();
                        let frame2 = link.joint().data.local_frame2.into_physx();

                        px_inbound_joint.set_parent_pose(&frame1);
                        px_inbound_joint.set_child_pose(&frame2);

                        /*

                        let px_joint = px_inbound_joint
                            .as_articulation_joint_reduced_coordinate()
                            .unwrap();

                        if let Some(_) = link
                            .articulation()
                            .downcast_ref::<SphericalMultibodyJoint>()
                        {
                            px_joint.set_joint_type(ArticulationJointType::Spherical);
                            px_joint.set_motion(ArticulationAxis::Swing1, ArticulationMotion::Free);
                            px_joint.set_motion(ArticulationAxis::Swing2, ArticulationMotion::Free);
                            px_joint.set_motion(ArticulationAxis::Twist, ArticulationMotion::Free);
                        } else if let Some(_) =
                            link.articulation().downcast_ref::<RevoluteMultibodyJoint>()
                        {
                            px_joint.set_joint_type(ArticulationJointType::Revolute);
                            px_joint.set_motion(ArticulationAxis::Swing1, ArticulationMotion::Free);
                            px_joint.set_motion(ArticulationAxis::Swing2, ArticulationMotion::Free);
                            px_joint.set_motion(ArticulationAxis::Twist, ArticulationMotion::Free);
                        }

                         */
                    }

                    // FIXME: we are using transmute here in order to erase the lifetime of
                    //        the &mut ref behind px_link (which is tied to the lifetime of the
                    //        multibody_joint). This looks necessary because we need
                    //        that mutable ref to create the next link. Yet, the link creation
                    //        methods also requires a mutable ref to the multibody_joint.
                    rapier2link.insert(rb_handle, px_link as *mut PxArticulationLink);
                    parent = Some(unsafe { std::mem::transmute(px_link as *mut _) });
                }

                scene.add_articulation(articulation);
            }

             */

            /*
             *
             * Colliders
             *
             */
            for (_, collider) in colliders.iter() {
                if let Some((mut px_shape, px_material, collider_pos)) =
                    physx_collider_from_rapier_collider(&mut *physics, &collider)
                {
                    if let Some(parent_handle) = collider.parent() {
                        let parent_body = &bodies[parent_handle];

                        if let Some(link) = rapier2link.get_mut(&parent_handle) {
                            unsafe {
                                physx_sys::PxRigidActor_attachShape_mut(
                                    *link as *mut PxRigidActor,
                                    px_shape.as_mut_ptr(),
                                );
                            }
                        } else if !parent_body.is_dynamic() {
                            let actor = rapier2static.get_mut(&parent_handle).unwrap();
                            actor.attach_shape(&mut px_shape);
                        } else {
                            let actor = rapier2dynamic.get_mut(&parent_handle).unwrap();
                            actor.attach_shape(&mut px_shape);
                        }

                        unsafe {
                            let pose = collider_pos.into_physx();
                            physx_sys::PxShape_setLocalPose_mut(
                                px_shape.as_mut_ptr(),
                                &pose.into(),
                            );
                        }

                        shapes.push(px_shape);
                        materials.push(px_material);
                    }
                }
            }

            // Update mass properties and CCD flags.
            for (rapier_handle, _rb) in bodies.iter() {
                let rb = &bodies[rapier_handle];
                let densities: Vec<_> = rb
                    .colliders()
                    .iter()
                    .map(|h| colliders[*h].density())
                    .collect();

                unsafe {
                    let actor = if let Some(actor) = rapier2dynamic.get_mut(&rapier_handle) {
                        std::mem::transmute(actor.as_mut())
                    } else if let Some(actor) = rapier2link.get_mut(&rapier_handle) {
                        *actor as *mut _
                    } else {
                        continue;
                    };

                    physx_sys::PxRigidBodyExt_updateMassAndInertia(
                        actor,
                        densities.as_ptr(),
                        densities.len() as u32,
                        std::ptr::null(),
                        false,
                    );

                    if rb.is_ccd_enabled() {
                        physx_sys::PxRigidBody_setRigidBodyFlag_mut(
                            actor,
                            RigidBodyFlag::EnableCcd,
                            true,
                        );
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
                impulse_joints,
                &mut rapier2static,
                &mut rapier2dynamic,
                &mut rapier2link,
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
        impulse_joints: &ImpulseJointSet,
        rapier2static: &mut HashMap<RigidBodyHandle, Owner<PxRigidStatic>>,
        rapier2dynamic: &mut HashMap<RigidBodyHandle, Owner<PxRigidDynamic>>,
        rapier2link: &mut HashMap<RigidBodyHandle, *mut PxArticulationLink>,
    ) {
        unsafe {
            for joint in impulse_joints.iter() {
                let actor1 = rapier2static
                    .get_mut(&joint.1.body1)
                    .map(|act| &mut **act as *mut PxRigidStatic as *mut PxRigidActor)
                    .or(rapier2dynamic
                        .get_mut(&joint.1.body1)
                        .map(|act| &mut **act as *mut PxRigidDynamic as *mut PxRigidActor))
                    .or(rapier2link
                        .get_mut(&joint.1.body1)
                        .map(|lnk| *lnk as *mut PxRigidActor))
                    .unwrap();
                let actor2 = rapier2static
                    .get_mut(&joint.1.body2)
                    .map(|act| &mut **act as *mut PxRigidStatic as *mut PxRigidActor)
                    .or(rapier2dynamic
                        .get_mut(&joint.1.body2)
                        .map(|act| &mut **act as *mut PxRigidDynamic as *mut PxRigidActor))
                    .or(rapier2link
                        .get_mut(&joint.1.body2)
                        .map(|lnk| *lnk as *mut PxRigidActor))
                    .unwrap();

                let px_frame1 = joint.1.data.local_frame1.into_physx();
                let px_frame2 = joint.1.data.local_frame2.into_physx();

                let px_joint = physx_sys::phys_PxD6JointCreate(
                    physics.as_mut_ptr(),
                    actor1,
                    px_frame1.as_ptr(),
                    actor2,
                    px_frame2.as_ptr(),
                );

                let motion_x = if joint.1.data.limit_axes.contains(JointAxesMask::LIN_X) {
                    physx_sys::PxD6Motion::Limited
                } else if !joint.1.data.locked_axes.contains(JointAxesMask::LIN_X) {
                    physx_sys::PxD6Motion::Free
                } else {
                    physx_sys::PxD6Motion::Locked
                };
                let motion_y = if joint.1.data.limit_axes.contains(JointAxesMask::LIN_Y) {
                    physx_sys::PxD6Motion::Limited
                } else if !joint.1.data.locked_axes.contains(JointAxesMask::LIN_Y) {
                    physx_sys::PxD6Motion::Free
                } else {
                    physx_sys::PxD6Motion::Locked
                };
                let motion_z = if joint.1.data.limit_axes.contains(JointAxesMask::LIN_Z) {
                    physx_sys::PxD6Motion::Limited
                } else if !joint.1.data.locked_axes.contains(JointAxesMask::LIN_Z) {
                    physx_sys::PxD6Motion::Free
                } else {
                    physx_sys::PxD6Motion::Locked
                };
                let motion_ax = if joint.1.data.limit_axes.contains(JointAxesMask::ANG_X) {
                    physx_sys::PxD6Motion::Limited
                } else if !joint.1.data.locked_axes.contains(JointAxesMask::ANG_X) {
                    physx_sys::PxD6Motion::Free
                } else {
                    physx_sys::PxD6Motion::Locked
                };
                let motion_ay = if joint.1.data.limit_axes.contains(JointAxesMask::ANG_Y) {
                    physx_sys::PxD6Motion::Limited
                } else if !joint.1.data.locked_axes.contains(JointAxesMask::ANG_Y) {
                    physx_sys::PxD6Motion::Free
                } else {
                    physx_sys::PxD6Motion::Locked
                };
                let motion_az = if joint.1.data.limit_axes.contains(JointAxesMask::ANG_Z) {
                    physx_sys::PxD6Motion::Limited
                } else if !joint.1.data.locked_axes.contains(JointAxesMask::ANG_Z) {
                    physx_sys::PxD6Motion::Free
                } else {
                    physx_sys::PxD6Motion::Locked
                };

                physx_sys::PxD6Joint_setMotion_mut(px_joint, physx_sys::PxD6Axis::X, motion_x);
                physx_sys::PxD6Joint_setMotion_mut(px_joint, physx_sys::PxD6Axis::Y, motion_y);
                physx_sys::PxD6Joint_setMotion_mut(px_joint, physx_sys::PxD6Axis::Z, motion_z);
                physx_sys::PxD6Joint_setMotion_mut(px_joint, physx_sys::PxD6Axis::Twist, motion_ax);
                physx_sys::PxD6Joint_setMotion_mut(
                    px_joint,
                    physx_sys::PxD6Axis::Swing1,
                    motion_ay,
                );
                physx_sys::PxD6Joint_setMotion_mut(
                    px_joint,
                    physx_sys::PxD6Axis::Swing2,
                    motion_az,
                );
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
        let mut sync_pos = |handle: &RigidBodyHandle, pos: Isometry3<f32>| {
            let rb = &mut bodies[*handle];
            rb.set_position(pos, false);

            for coll_handle in rb.colliders() {
                let collider = &mut colliders[*coll_handle];
                collider.set_position(
                    pos * collider.position_wrt_parent().copied().unwrap_or(na::one()),
                );
            }
        };

        for actor in self.scene.as_mut().unwrap().get_dynamic_actors() {
            let handle = actor.get_user_data();
            let pos = actor.get_global_pose().into_na();
            sync_pos(handle, pos);
        }

        /*
        for articulation in self.scene.as_mut().unwrap().get_articulations() {
            if let Some(articulation) = articulation.as_articulation_reduced_coordinate() {
                for link in articulation.get_links() {
                    let handle = link.get_user_data();
                    let pos = link.get_global_pose().into_na();
                    sync_pos(handle, pos);
                }
            }
        }

         */
    }
}

fn physx_collider_from_rapier_collider(
    physics: &mut PxPhysicsFoundation,
    collider: &Collider,
) -> Option<(Owner<PxShape>, Owner<PxMaterial>, Isometry3<f32>)> {
    let mut local_pose = collider.position_wrt_parent().copied().unwrap_or(na::one());
    let cooking_params = PxCookingParams::new(physics).unwrap();
    let shape = collider.shape();
    let shape_flags = if collider.is_sensor() {
        ShapeFlags::TriggerShape
    } else {
        ShapeFlags::SimulationShape
    };
    let mut material = physics
        .create_material(
            collider.material().friction,
            collider.material().friction,
            collider.material().restitution,
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
                    materialIndex0: PxBitAndByte {
                        structgen_pad0: [0; 1],
                    },
                    materialIndex1: PxBitAndByte {
                        structgen_pad0: [0; 1],
                    },
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
        let heightfield = physx::cooking::create_height_field(physics, &heightfield_desc);

        if let Some(mut heightfield) = heightfield {
            let flags = PxMeshGeometryFlags::DoubleSided;
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
        .or(shape.as_round_convex_polyhedron().map(|c| &c.inner_shape))
    {
        let vertices = convex.points();
        let mut convex_desc;
        unsafe {
            convex_desc = physx_sys::PxConvexMeshDesc_new();
            convex_desc.points.count = vertices.len() as u32;
            convex_desc.points.stride = (3 * std::mem::size_of::<f32>()) as u32;
            convex_desc.points.data = vertices.as_ptr() as *const std::ffi::c_void;
            convex_desc.flags = PxConvexFlags::ComputeConvex;
        }

        let convex_desc = PxConvexMeshDesc { obj: convex_desc };
        let convex = physx::cooking::create_convex_mesh(physics, &cooking_params, &convex_desc);

        if let ConvexMeshCookingResult::Success(mut convex) = convex {
            let flags = PxConvexMeshGeometryFlags::empty();
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
        let trimesh = physx::cooking::create_triangle_mesh(physics, &cooking_params, &mesh_desc);

        if let TriangleMeshCookingResult::Success(mut trimesh) = trimesh {
            let flags = PxMeshGeometryFlags::DoubleSided;
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
type PxArticulationLink = physx::articulation_link::PxArticulationLink<RigidBodyHandle, PxShape>;
type PxRigidStatic = physx::rigid_static::PxRigidStatic<(), PxShape>;
type PxRigidDynamic = physx::rigid_dynamic::PxRigidDynamic<RigidBodyHandle, PxShape>;
type PxArticulationReducedCoordinate =
    physx::articulation_reduced_coordinate::PxArticulationReducedCoordinate<(), PxArticulationLink>;
type PxScene = physx::scene::PxScene<
    (),
    PxArticulationLink,
    PxRigidStatic,
    PxRigidDynamic,
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

unsafe extern "C" fn ccd_filter_shader(data: *mut FilterShaderCallbackInfo) -> PxFilterFlags {
    (*(*data).pairFlags) |= physx_sys::PxPairFlags::DetectCcdContact;
    PxFilterFlags::empty()
}
