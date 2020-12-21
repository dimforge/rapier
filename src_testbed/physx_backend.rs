#![allow(dead_code)]

use na::{
    Isometry3, Matrix3, Matrix4, Point3, Quaternion, Rotation3, Translation3, Unit, UnitQuaternion,
    Vector3,
};
use physx::cooking::PxCooking;
use physx::foundation::DefaultAllocator;
use physx::prelude::*;
use physx::scene::FrictionType;
use physx::triangle_mesh::TriangleMesh;
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

            let scene_desc = SceneDescriptor {
                gravity: gravity.into_physx(),
                thread_count: num_threads as u32,
                broad_phase_type: BroadPhaseType::AutomaticBoxPruning,
                solver_type: SolverType::PGS,
                friction_type,
                ..SceneDescriptor::new(())
            };

            let mut scene: Owner<PxScene> = physics.create(scene_desc).unwrap();
            let mut rapier2dynamic = HashMap::new();
            let mut rapier2static = HashMap::new();

            for (rapier_handle, rb) in bodies.iter() {
                use physx::rigid_dynamic::RigidDynamic;
                use physx::rigid_static::RigidStatic;

                let pos = rb.position().into_physx();
                if rb.is_dynamic() {
                    let mut actor = physics.create_dynamic(&pos, rapier_handle).unwrap();
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

            for (_, collider) in colliders.iter() {
                if let Some((mut px_shape, px_material, collider_pos)) =
                    physx_collider_from_rapier_collider(&mut *physics, &collider)
                {
                    let parent_body = &bodies[collider.parent()];

                    if !parent_body.is_dynamic() {
                        let actor = rapier2static.get_mut(&collider.parent()).unwrap();
                        actor.attach_shape(&mut px_shape);
                    } else {
                        let actor = rapier2dynamic.get_mut(&collider.parent()).unwrap();
                        actor.attach_shape(&mut px_shape);
                    }
                    // physx_sys::PxShape_setLocalPose_mut(shape, &pose);

                    shapes.push(px_shape);
                    materials.push(px_material);
                }
            }

            // Update mass properties.
            for (rapier_handle, actor) in rapier2dynamic.iter_mut() {
                let rb = &bodies[*rapier_handle];
                let densities: Vec<_> = rb
                    .colliders()
                    .iter()
                    .map(|h| colliders[*h].density())
                    .collect();

                unsafe {
                    physx_sys::PxRigidBodyExt_updateMassAndInertia_mut(
                        std::mem::transmute(actor.as_mut()),
                        densities.as_ptr(),
                        densities.len() as u32,
                        std::ptr::null(),
                        false,
                    );
                }
            }

            /*
               res.setup_joints(joints);
               res
            */

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

    fn setup_joints(&mut self, joints: &JointSet) {
        /*
        unsafe {
            for joint in joints.iter() {
                let actor1 = self.rapier2physx[&joint.1.body1];
                let actor2 = self.rapier2physx[&joint.1.body2];

                match &joint.1.params {
                    JointParams::BallJoint(params) => {
                        let frame1 = physx::transform::gl_to_px_tf(
                            Isometry3::new(params.local_anchor1.coords, na::zero()).into_glam(),
                        );
                        let frame2 = physx::transform::gl_to_px_tf(
                            Isometry3::new(params.local_anchor2.coords, na::zero()).into_glam(),
                        );

                        physx_sys::phys_PxSphericalJointCreate(
                            self.physics.get_raw_mut(),
                            actor1.0 as *mut _,
                            &frame1 as *const _,
                            actor2.0 as *mut _,
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

                        let frame1 = physx::transform::gl_to_px_tf(
                            Isometry3::new(params.local_anchor1.coords, axisangle1).into_glam(),
                        );
                        let frame2 = physx::transform::gl_to_px_tf(
                            Isometry3::new(params.local_anchor2.coords, axisangle2).into_glam(),
                        );

                        physx_sys::phys_PxRevoluteJointCreate(
                            self.physics.get_raw_mut(),
                            actor1.0 as *mut _,
                            &frame1 as *const _,
                            actor2.0 as *mut _,
                            &frame2 as *const _,
                        );
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

                        let frame1 = physx::transform::gl_to_px_tf(
                            Isometry3::new(params.local_anchor1.coords, axisangle1).into_glam(),
                        );
                        let frame2 = physx::transform::gl_to_px_tf(
                            Isometry3::new(params.local_anchor2.coords, axisangle2).into_glam(),
                        );

                        let joint = physx_sys::phys_PxPrismaticJointCreate(
                            self.physics.get_raw_mut(),
                            actor1.0 as *mut _,
                            &frame1 as *const _,
                            actor2.0 as *mut _,
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
                        let frame1 =
                            physx::transform::gl_to_px_tf(params.local_anchor1.into_glam());
                        let frame2 =
                            physx::transform::gl_to_px_tf(params.local_anchor2.into_glam());

                        physx_sys::phys_PxFixedJointCreate(
                            self.physics.get_raw_mut(),
                            actor1.0 as *mut _,
                            &frame1 as *const _,
                            actor2.0 as *mut _,
                            &frame2 as *const _,
                        );
                    }
                }
            }
        }
         */
    }

    pub fn step(&mut self, counters: &mut Counters, params: &IntegrationParameters) {
        let mut scratch = unsafe { ScratchBuffer::new(4) };

        counters.step_started();
        self.scene
            .as_mut()
            .unwrap()
            .step(
                params.dt(),
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
    // cooking: &PxCooking,
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
        local_pose *= Translation3::from(center.coords) * rot.unwrap_or(UnitQuaternion::identity());
        let geometry = PxCapsuleGeometry::new(capsule.radius, capsule.half_height());
        physics.create_shape(&geometry, materials, true, shape_flags, ())
    } else if let Some(trimesh) = shape.as_trimesh() {
        return None;
        /*
        ColliderDesc::TriMesh {
            vertices: trimesh
                .vertices()
                .iter()
                .map(|pt| pt.into_physx())
                .collect(),
            indices: trimesh.flat_indices().to_vec(),
            mesh_scale: Vector3::repeat(1.0).into_glam(),
        };
        let desc = cooking.create_triangle_mesh(physics, desc);
        if let TriangleMeshCookingResult::Success(trimesh) = desc {
            Some(trimesh)
        } else {
            eprintln!("PhysX triangle mesh construction failed.");
            return None;
        }
         */
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
