#[cfg(feature = "dim2")]
use ncollide::shape::ConvexPolygon;
use ncollide::shape::{Ball, Capsule, Cuboid, HeightField, ShapeHandle};
use nphysics::force_generator::DefaultForceGeneratorSet;
use nphysics::joint::{
    DefaultJointConstraintSet, FixedConstraint, PrismaticConstraint, RevoluteConstraint,
};
use nphysics::object::{
    BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodySet, DefaultColliderSet,
    RigidBodyDesc,
};
use nphysics::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use rapier::counters::Counters;
use rapier::dynamics::{
    IntegrationParameters, JointParams, JointSet, RigidBodyHandle, RigidBodySet,
};
use rapier::geometry::{Collider, ColliderSet};
use rapier::math::Vector;
use std::collections::HashMap;
#[cfg(feature = "dim3")]
use {ncollide::shape::TriMesh, nphysics::joint::BallConstraint};

pub struct NPhysicsWorld {
    rapier2nphysics: HashMap<RigidBodyHandle, DefaultBodyHandle>,
    mechanical_world: DefaultMechanicalWorld<f32>,
    geometrical_world: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    joints: DefaultJointConstraintSet<f32>,
    force_generators: DefaultForceGeneratorSet<f32>,
}

impl NPhysicsWorld {
    pub fn from_rapier(
        gravity: Vector<f32>,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        joints: &JointSet,
    ) -> Self {
        let mut rapier2nphysics = HashMap::new();

        let mechanical_world = DefaultMechanicalWorld::new(gravity);
        let geometrical_world = DefaultGeometricalWorld::new();
        let mut nphysics_bodies = DefaultBodySet::new();
        let mut nphysics_colliders = DefaultColliderSet::new();
        let mut nphysics_joints = DefaultJointConstraintSet::new();
        let force_generators = DefaultForceGeneratorSet::new();

        for (rapier_handle, rb) in bodies.iter() {
            // let material = physics.create_material(rb.collider.friction, rb.collider.friction, 0.0);
            let nphysics_rb = RigidBodyDesc::new().position(*rb.position()).build();
            let nphysics_rb_handle = nphysics_bodies.insert(nphysics_rb);

            rapier2nphysics.insert(rapier_handle, nphysics_rb_handle);
        }

        for (_, collider) in colliders.iter() {
            let parent = &bodies[collider.parent()];
            let nphysics_rb_handle = rapier2nphysics[&collider.parent()];
            if let Some(collider) =
                nphysics_collider_from_rapier_collider(&collider, parent.is_dynamic())
            {
                let nphysics_collider = collider.build(BodyPartHandle(nphysics_rb_handle, 0));
                nphysics_colliders.insert(nphysics_collider);
            } else {
                eprintln!("Creating shape unknown to the nphysics backend.")
            }
        }

        for joint in joints.iter() {
            let b1 = BodyPartHandle(rapier2nphysics[&joint.1.body1], 0);
            let b2 = BodyPartHandle(rapier2nphysics[&joint.1.body2], 0);

            match &joint.1.params {
                JointParams::FixedJoint(params) => {
                    let c = FixedConstraint::new(
                        b1,
                        b2,
                        params.local_anchor1.translation.vector.into(),
                        params.local_anchor1.rotation,
                        params.local_anchor2.translation.vector.into(),
                        params.local_anchor2.rotation,
                    );
                    nphysics_joints.insert(c);
                }
                #[cfg(feature = "dim3")]
                JointParams::BallJoint(params) => {
                    let c = BallConstraint::new(b1, b2, params.local_anchor1, params.local_anchor2);
                    nphysics_joints.insert(c);
                }
                #[cfg(feature = "dim2")]
                JointParams::BallJoint(params) => {
                    let c =
                        RevoluteConstraint::new(b1, b2, params.local_anchor1, params.local_anchor2);
                    nphysics_joints.insert(c);
                }
                #[cfg(feature = "dim3")]
                JointParams::RevoluteJoint(params) => {
                    let c = RevoluteConstraint::new(
                        b1,
                        b2,
                        params.local_anchor1,
                        params.local_axis1,
                        params.local_anchor2,
                        params.local_axis2,
                    );
                    nphysics_joints.insert(c);
                }
                JointParams::PrismaticJoint(params) => {
                    let mut c = PrismaticConstraint::new(
                        b1,
                        b2,
                        params.local_anchor1,
                        params.local_axis1(),
                        params.local_anchor2,
                    );

                    if params.limits_enabled {
                        c.enable_min_offset(params.limits[0]);
                        c.enable_max_offset(params.limits[1]);
                    }

                    nphysics_joints.insert(c);
                } // JointParams::GenericJoint(_) => {
                  //     eprintln!(
                  //         "Joint type currently unsupported by the nphysics backend: GenericJoint."
                  //     )
                  // }
            }
        }

        Self {
            rapier2nphysics,
            mechanical_world,
            geometrical_world,
            bodies: nphysics_bodies,
            colliders: nphysics_colliders,
            joints: nphysics_joints,
            force_generators,
        }
    }

    pub fn step(&mut self, counters: &mut Counters, params: &IntegrationParameters) {
        self.mechanical_world
            .integration_parameters
            .max_position_iterations = params.max_position_iterations;
        self.mechanical_world
            .integration_parameters
            .max_velocity_iterations = params.max_velocity_iterations;
        self.mechanical_world
            .integration_parameters
            .set_dt(params.dt);
        self.mechanical_world.integration_parameters.warmstart_coeff = params.warmstart_coeff;

        counters.step_started();
        self.mechanical_world.step(
            &mut self.geometrical_world,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joints,
            &mut self.force_generators,
        );
        counters.step_completed();
    }

    pub fn sync(&self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
        for (rapier_handle, nphysics_handle) in self.rapier2nphysics.iter() {
            let rb = bodies.get_mut(*rapier_handle).unwrap();
            let ra = self.bodies.rigid_body(*nphysics_handle).unwrap();
            let pos = *ra.position();
            rb.set_position(pos, false);

            for coll_handle in rb.colliders() {
                let collider = &mut colliders[*coll_handle];
                collider.set_position_debug(pos * collider.position_wrt_parent());
            }
        }
    }
}

fn nphysics_collider_from_rapier_collider(
    collider: &Collider,
    is_dynamic: bool,
) -> Option<ColliderDesc<f32>> {
    let mut margin = ColliderDesc::<f32>::default_margin();
    let mut pos = *collider.position_wrt_parent();
    let shape = collider.shape();

    let shape = if let Some(cuboid) = shape.as_cuboid() {
        ShapeHandle::new(Cuboid::new(cuboid.half_extents.map(|e| e - margin)))
    } else if let Some(cuboid) = shape.as_round_cuboid() {
        margin = cuboid.border_radius;
        ShapeHandle::new(Cuboid::new(cuboid.base_shape.half_extents))
    } else if let Some(ball) = shape.as_ball() {
        ShapeHandle::new(Ball::new(ball.radius - margin))
    } else if let Some(capsule) = shape.as_capsule() {
        pos *= capsule.transform_wrt_y();
        ShapeHandle::new(Capsule::new(capsule.half_height(), capsule.radius))
    } else if let Some(heightfield) = shape.as_heightfield() {
        let heights = heightfield.heights();
        let scale = heightfield.scale();
        let heightfield = HeightField::new(heights.clone(), *scale);
        ShapeHandle::new(heightfield)
    } else {
        #[cfg(feature = "dim3")]
        if let Some(trimesh) = shape.as_trimesh() {
            ShapeHandle::new(TriMesh::new(
                trimesh.vertices().to_vec(),
                trimesh
                    .indices()
                    .iter()
                    .map(|idx| na::Point3::new(idx[0] as usize, idx[1] as usize, idx[2] as usize))
                    .collect(),
                None,
            ))
        } else {
            return None;
        }

        #[cfg(feature = "dim2")]
        if let Some(polygon) = shape.as_round_convex_polygon() {
            margin = polygon.border_radius;
            ShapeHandle::new(ConvexPolygon::try_from_points(polygon.base_shape.points()).unwrap())
        } else if let Some(polygon) = shape.as_convex_polygon() {
            ShapeHandle::new(ConvexPolygon::try_from_points(polygon.points()).unwrap())
        } else {
            return None;
        }
    };

    let density = if is_dynamic {
        collider.density().unwrap_or(0.0)
    } else {
        0.0
    };

    Some(
        ColliderDesc::new(shape)
            .position(pos)
            .density(density)
            .sensor(collider.is_sensor())
            .margin(margin),
    )
}
