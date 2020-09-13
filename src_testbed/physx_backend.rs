#![allow(dead_code)]

use na::{Isometry3, Matrix3, Matrix4, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use physx::prelude::*;
use rapier::counters::Counters;
use rapier::dynamics::{
    IntegrationParameters, JointParams, JointSet, RigidBodyHandle, RigidBodySet,
};
use rapier::geometry::{Collider, ColliderSet, Shape};
use rapier::utils::WBasis;
use std::collections::HashMap;

const PX_PHYSICS_VERSION: u32 = physx::version(4, 1, 1);

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

trait IntoPhysx {
    type Output;
    fn into_physx(self) -> Self::Output;
}

impl IntoPhysx for Vector3<f32> {
    type Output = physx_sys::PxVec3;
    fn into_physx(self) -> Self::Output {
        physx_sys::PxVec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

impl IntoPhysx for Point3<f32> {
    type Output = physx_sys::PxVec3;
    fn into_physx(self) -> Self::Output {
        physx_sys::PxVec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

impl IntoPhysx for Isometry3<f32> {
    type Output = physx_sys::PxTransform;
    fn into_physx(self) -> Self::Output {
        physx::transform::gl_to_px_tf(self.into_glam())
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
pub static FOUNDATION: std::cell::RefCell<Foundation> = std::cell::RefCell::new(Foundation::new(PX_PHYSICS_VERSION));
}

pub struct PhysxWorld {
    physics: Physics,
    cooking: Cooking,
    scene: Scene,
    rapier2physx: HashMap<RigidBodyHandle, BodyHandle>,
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
        let mut rapier2physx = HashMap::new();
        let mut physics = FOUNDATION.with(|f| {
            PhysicsBuilder::default()
                .load_extensions(false)
                .build(&mut *f.borrow_mut())
        });
        let mut cooking = FOUNDATION.with(|f| unsafe {
            let sc = physx_sys::PxTolerancesScale_new();
            let params = physx_sys::PxCookingParams_new(&sc);
            Cooking::new(PX_PHYSICS_VERSION, &mut *f.borrow_mut(), params)
        });

        let scene_desc = MySceneBuilder::default()
            .set_gravity(gravity.into_glam())
            .set_simulation_threading(SimulationThreadType::Dedicated(num_threads as u32))
            // .set_broad_phase_type(BroadPhaseType::SweepAndPrune)
            // .set_solver_type(physx_sys::PxSolverType::eTGS)
            .build_desc(&mut physics);

        let raw_scene =
            unsafe { physx_sys::PxPhysics_createScene_mut(physics.get_raw_mut(), &scene_desc) };

        // FIXME: we do this because we are also using two
        // friction directions. We should add to rapier the option to use
        // one friction direction too, and perhaps an equivalent of physX
        // ePATCH friction type.
        if use_two_friction_directions {
            unsafe {
                physx_sys::PxScene_setFrictionType_mut(
                    raw_scene,
                    physx_sys::PxFrictionType::eTWO_DIRECTIONAL,
                );
            }
        }

        let mut scene = Scene::new(raw_scene);

        for (rapier_handle, rb) in bodies.iter() {
            use physx::rigid_dynamic::RigidDynamic;
            use physx::rigid_static::RigidStatic;
            use physx::transform;

            let pos = transform::gl_to_px_tf(rb.position.to_homogeneous().into_glam());
            if rb.is_dynamic() {
                let actor = unsafe {
                    physx_sys::PxPhysics_createRigidDynamic_mut(physics.get_raw_mut(), &pos)
                };

                unsafe {
                    physx_sys::PxRigidDynamic_setSolverIterationCounts_mut(
                        actor,
                        integration_parameters.max_position_iterations as u32,
                        integration_parameters.max_velocity_iterations as u32,
                    );
                }

                let physx_handle = scene.add_dynamic(RigidDynamic::new(actor));
                rapier2physx.insert(rapier_handle, physx_handle);
            } else {
                let actor = unsafe {
                    physx_sys::PxPhysics_createRigidStatic_mut(physics.get_raw_mut(), &pos)
                };

                let physx_handle = scene.add_actor(RigidStatic::new(actor));
                rapier2physx.insert(rapier_handle, physx_handle);
            }
        }

        for (_, collider) in colliders.iter() {
            if let Some((px_collider, collider_pos)) =
                physx_collider_from_rapier_collider(&collider)
            {
                let material = physics.create_material(
                    collider.friction,
                    collider.friction,
                    collider.restitution,
                );
                let geometry = cooking.make_geometry(px_collider);
                let flags = if collider.is_sensor() {
                    physx_sys::PxShapeFlags {
                        mBits: physx_sys::PxShapeFlag::eTRIGGER_SHAPE as u8,
                    }
                } else {
                    physx_sys::PxShapeFlags {
                        mBits: physx_sys::PxShapeFlag::eSIMULATION_SHAPE as u8, // | physx_sys::PxShapeFlag::eSCENE_QUERY_SHAPE as u8,
                    }
                };

                let handle = rapier2physx[&collider.parent()];
                let parent_body = &bodies[collider.parent()];
                let parent = if !parent_body.is_dynamic() {
                    scene.get_static_mut(handle).unwrap().as_ptr_mut().ptr
                        as *mut physx_sys::PxRigidActor
                } else {
                    scene.get_dynamic_mut(handle).unwrap().as_ptr_mut().ptr
                        as *mut physx_sys::PxRigidActor
                };

                unsafe {
                    let shape = physx_sys::PxPhysics_createShape_mut(
                        physics.get_raw_mut(),
                        geometry.as_raw(),
                        material,
                        true,
                        flags.into(),
                    );
                    let pose = collider_pos.into_physx();
                    physx_sys::PxShape_setLocalPose_mut(shape, &pose);
                    physx_sys::PxRigidActor_attachShape_mut(parent, shape);
                };
            }
        }

        // Update mass properties.
        for (rapier_handle, physx_handle) in rapier2physx.iter() {
            let rb = &bodies[*rapier_handle];
            if let Some(rp) = scene.get_dynamic_mut(*physx_handle) {
                let densities: Vec<_> = rb
                    .colliders()
                    .iter()
                    .map(|h| colliders[*h].density())
                    .collect();

                unsafe {
                    physx_sys::PxRigidBodyExt_updateMassAndInertia_mut(
                        rp.as_ptr_mut().ptr as *mut physx_sys::PxRigidBody,
                        densities.as_ptr(),
                        densities.len() as u32,
                        std::ptr::null(),
                        false,
                    );
                }
            }
        }

        let mut res = Self {
            physics,
            cooking,
            scene,
            rapier2physx,
        };

        res.setup_joints(joints);
        res
    }

    fn setup_joints(&mut self, joints: &JointSet) {
        unsafe {
            for joint in joints.iter() {
                let actor1 = self.rapier2physx[&joint.body1];
                let actor2 = self.rapier2physx[&joint.body2];

                match &joint.params {
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
    }

    pub fn step(&mut self, counters: &mut Counters, params: &IntegrationParameters) {
        counters.step_started();
        self.scene.step(params.dt(), true);
        counters.step_completed();
    }

    pub fn sync(&self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
        for (rapier_handle, physx_handle) in self.rapier2physx.iter() {
            let mut rb = bodies.get_mut(*rapier_handle).unwrap();
            let ra = self.scene.get_rigid_actor(*physx_handle).unwrap();
            let pos = ra.get_global_pose().into_na();
            let iso = na::convert_unchecked(pos);
            rb.set_position(iso);

            if rb.is_kinematic() {}

            for coll_handle in rb.colliders() {
                let collider = &mut colliders[*coll_handle];
                collider.set_position_debug(iso * collider.position_wrt_parent());
            }
        }
    }
}

fn physx_collider_from_rapier_collider(
    collider: &Collider,
) -> Option<(ColliderDesc, Isometry3<f32>)> {
    let mut local_pose = *collider.position_wrt_parent();
    let desc = match collider.shape() {
        Shape::Cuboid(cuboid) => ColliderDesc::Box(
            cuboid.half_extents.x,
            cuboid.half_extents.y,
            cuboid.half_extents.z,
        ),
        Shape::Ball(ball) => ColliderDesc::Sphere(ball.radius),
        Shape::Capsule(capsule) => {
            let center = capsule.center();
            let mut dir = capsule.b - capsule.a;

            if dir.x < 0.0 {
                dir = -dir;
            }

            let rot = UnitQuaternion::rotation_between(&Vector3::x(), &dir);
            local_pose *=
                Translation3::from(center.coords) * rot.unwrap_or(UnitQuaternion::identity());
            ColliderDesc::Capsule(capsule.radius, capsule.height())
        }
        Shape::Trimesh(trimesh) => ColliderDesc::TriMesh {
            vertices: trimesh
                .vertices()
                .iter()
                .map(|pt| pt.into_physx())
                .collect(),
            indices: trimesh.flat_indices().to_vec(),
            mesh_scale: Vector3::repeat(1.0).into_glam(),
        },
        _ => {
            eprintln!("Creating a shape unknown to the PhysX backend.");
            return None;
        }
    };

    Some((desc, local_pose))
}

/*
 *
 * XXX: All the remaining code is a duplicate from physx-rs to allow more customizations.
 *
 */
use physx::scene::SimulationThreadType;

pub struct MySceneBuilder {
    gravity: glam::Vec3,
    simulation_filter_shader: Option<physx_sys::SimulationFilterShader>,
    simulation_threading: Option<SimulationThreadType>,
    broad_phase_type: BroadPhaseType,
    use_controller_manager: bool,
    controller_manager_locking: bool,
    call_default_filter_shader_first: bool,
    use_ccd: bool,
    enable_ccd_resweep: bool,
    solver_type: u32,
}

impl Default for MySceneBuilder {
    fn default() -> Self {
        Self {
            gravity: glam::Vec3::new(0.0, -9.80665, 0.0), // standard gravity value
            call_default_filter_shader_first: true,
            simulation_filter_shader: None,
            simulation_threading: None,
            broad_phase_type: BroadPhaseType::SweepAndPrune,
            use_controller_manager: false,
            controller_manager_locking: false,
            use_ccd: false,
            enable_ccd_resweep: false,
            solver_type: physx_sys::PxSolverType::ePGS,
        }
    }
}

impl MySceneBuilder {
    /// Set the gravity for the scene.
    ///
    /// Default: [0.0, -9.80665, 0.0] (standard gravity)
    pub fn set_gravity(&mut self, gravity: glam::Vec3) -> &mut Self {
        self.gravity = gravity;
        self
    }

    /// Set a callback to be invoked on various simulation events. Note:
    /// Currently only handles collision events
    ///
    /// Default: not set
    pub fn set_simulation_filter_shader(
        &mut self,
        simulation_filter_shader: physx_sys::SimulationFilterShader,
    ) -> &mut Self {
        self.simulation_filter_shader = Some(simulation_filter_shader);
        self
    }

    /// Enable the controller manager on the scene.
    ///
    /// Default: false, false
    pub fn use_controller_manager(
        &mut self,
        use_controller_manager: bool,
        locking_enabled: bool,
    ) -> &mut Self {
        self.use_controller_manager = use_controller_manager;
        self.controller_manager_locking = locking_enabled;
        self
    }

    pub fn set_solver_type(&mut self, solver_type: u32) -> &mut Self {
        self.solver_type = solver_type;
        self
    }

    /// Sets whether the filter should begin by calling the default filter shader
    /// PxDefaultSimulationFilterShader that emulates the PhysX 2.8 rules.
    ///
    /// Default: true
    pub fn set_call_default_filter_shader_first(
        &mut self,
        call_default_filter_shader_first: bool,
    ) -> &mut Self {
        self.call_default_filter_shader_first = call_default_filter_shader_first;
        self
    }

    /// Set the number of threads to use for simulation
    ///
    /// Default: not set
    pub fn set_simulation_threading(
        &mut self,
        simulation_threading: SimulationThreadType,
    ) -> &mut Self {
        self.simulation_threading = Some(simulation_threading);
        self
    }

    /// Set collision detection type
    ///
    /// Default: Sweep and prune
    pub fn set_broad_phase_type(&mut self, broad_phase_type: BroadPhaseType) -> &mut Self {
        self.broad_phase_type = broad_phase_type;
        self
    }

    /// Set if CCD (continuous collision detection) should be available for use in the scene.
    /// Doesn't automatically enable it for all rigid bodies, they still need to be flagged.
    ///
    /// If you don't set enable_ccd_resweep to true, eDISABLE_CCD_RESWEEP is set, which improves performance
    /// at the cost of accuracy right after bounces.
    ///
    /// Default: false, false
    pub fn set_use_ccd(&mut self, use_ccd: bool, enable_ccd_resweep: bool) -> &mut Self {
        self.use_ccd = use_ccd;
        self.enable_ccd_resweep = enable_ccd_resweep;
        self
    }

    pub(super) fn build_desc(&self, physics: &mut Physics) -> physx_sys::PxSceneDesc {
        unsafe {
            let tolerances = physics.get_tolerances_scale();
            let mut scene_desc = physx_sys::PxSceneDesc_new(tolerances);

            let dispatcher = match self.simulation_threading.as_ref().expect("foo") {
                SimulationThreadType::Default => {
                    physx_sys::phys_PxDefaultCpuDispatcherCreate(1, std::ptr::null_mut()) as *mut _
                }
                SimulationThreadType::Dedicated(count) => {
                    physx_sys::phys_PxDefaultCpuDispatcherCreate(*count, std::ptr::null_mut())
                        as *mut _
                }
                SimulationThreadType::Shared(dispatcher) => *dispatcher as *mut _,
            };

            scene_desc.cpuDispatcher = dispatcher;
            scene_desc.gravity = physx::transform::gl_to_px_v3(self.gravity);
            if self.use_ccd {
                scene_desc.flags.mBits |= physx_sys::PxSceneFlag::eENABLE_CCD;
                if !self.enable_ccd_resweep {
                    scene_desc.flags.mBits |= physx_sys::PxSceneFlag::eDISABLE_CCD_RESWEEP;
                }
            }
            if let Some(filter_shader) = self.simulation_filter_shader {
                physx_sys::enable_custom_filter_shader(
                    &mut scene_desc as *mut physx_sys::PxSceneDesc,
                    filter_shader,
                    if self.call_default_filter_shader_first {
                        1
                    } else {
                        0
                    },
                );
            } else {
                scene_desc.filterShader = physx_sys::get_default_simulation_filter_shader();
            }

            scene_desc.broadPhaseType = self.broad_phase_type.into();
            scene_desc.solverType = self.solver_type;
            scene_desc
        }
    }
}
