use std::collections::HashMap;

use na::{Isometry2, Vector2};
use rapier::counters::Counters;
use rapier::dynamics::{
    IntegrationParameters, JointParams, JointSet, RigidBodyHandle, RigidBodySet,
};
use rapier::geometry::{Collider, ColliderSet};
use std::f32;

use wrapped2d::b2;
use wrapped2d::dynamics::joints::{PrismaticJointDef, RevoluteJointDef, WeldJointDef};
use wrapped2d::user_data::NoUserData;

fn na_vec_to_b2_vec(v: Vector2<f32>) -> b2::Vec2 {
    b2::Vec2 { x: v.x, y: v.y }
}

fn b2_vec_to_na_vec(v: b2::Vec2) -> Vector2<f32> {
    Vector2::new(v.x, v.y)
}

fn b2_transform_to_na_isometry(v: b2::Transform) -> Isometry2<f32> {
    Isometry2::new(b2_vec_to_na_vec(v.pos), v.rot.angle())
}

pub struct Box2dWorld {
    world: b2::World<NoUserData>,
    rapier2box2d: HashMap<RigidBodyHandle, b2::BodyHandle>,
}

impl Box2dWorld {
    pub fn from_rapier(
        gravity: Vector2<f32>,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        joints: &JointSet,
    ) -> Self {
        let mut world = b2::World::new(&na_vec_to_b2_vec(gravity));
        world.set_continuous_physics(bodies.iter().any(|b| b.1.is_ccd_enabled()));

        let mut res = Box2dWorld {
            world,
            rapier2box2d: HashMap::new(),
        };

        res.insert_bodies(bodies);
        res.insert_colliders(colliders);
        res.insert_joints(joints);
        res
    }

    fn insert_bodies(&mut self, bodies: &RigidBodySet) {
        for (handle, body) in bodies.iter() {
            let body_type = if !body.is_dynamic() {
                b2::BodyType::Static
            } else {
                b2::BodyType::Dynamic
            };

            let linear_damping = 0.0;
            let angular_damping = 0.0;

            //            if let Some(rb) = body.downcast_ref::<RigidBody<f32>>() {
            //                linear_damping = rb.linear_damping();
            //                angular_damping = rb.angular_damping();
            //            } else {
            //                linear_damping = 0.0;
            //                angular_damping = 0.0;
            //            }

            let def = b2::BodyDef {
                body_type,
                position: na_vec_to_b2_vec(body.position().translation.vector),
                angle: body.position().rotation.angle(),
                linear_velocity: na_vec_to_b2_vec(*body.linvel()),
                angular_velocity: body.angvel(),
                linear_damping,
                angular_damping,
                bullet: body.is_ccd_enabled(),
                ..b2::BodyDef::new()
            };
            let b2_handle = self.world.create_body(&def);
            self.rapier2box2d.insert(handle, b2_handle);
        }
    }

    fn insert_colliders(&mut self, colliders: &ColliderSet) {
        for (_, collider) in colliders.iter() {
            let b2_body_handle = self.rapier2box2d[&collider.parent()];
            let mut b2_body = self.world.body_mut(b2_body_handle);
            Self::create_fixture(&collider, &mut *b2_body);
        }
    }

    fn insert_joints(&mut self, joints: &JointSet) {
        for joint in joints.iter() {
            let body_a = self.rapier2box2d[&joint.1.body1];
            let body_b = self.rapier2box2d[&joint.1.body2];

            match &joint.1.params {
                JointParams::BallJoint(params) => {
                    let def = RevoluteJointDef {
                        body_a,
                        body_b,
                        collide_connected: true,
                        local_anchor_a: na_vec_to_b2_vec(params.local_anchor1.coords),
                        local_anchor_b: na_vec_to_b2_vec(params.local_anchor2.coords),
                        reference_angle: 0.0,
                        enable_limit: false,
                        lower_angle: 0.0,
                        upper_angle: 0.0,
                        enable_motor: false,
                        motor_speed: 0.0,
                        max_motor_torque: 0.0,
                    };

                    self.world.create_joint(&def);
                }
                JointParams::FixedJoint(params) => {
                    let def = WeldJointDef {
                        body_a,
                        body_b,
                        collide_connected: true,
                        local_anchor_a: na_vec_to_b2_vec(params.local_anchor1.translation.vector),
                        local_anchor_b: na_vec_to_b2_vec(params.local_anchor2.translation.vector),
                        reference_angle: 0.0,
                        frequency: 0.0,
                        damping_ratio: 0.0,
                    };

                    self.world.create_joint(&def);
                }
                JointParams::PrismaticJoint(params) => {
                    let def = PrismaticJointDef {
                        body_a,
                        body_b,
                        collide_connected: true,
                        local_anchor_a: na_vec_to_b2_vec(params.local_anchor1.coords),
                        local_anchor_b: na_vec_to_b2_vec(params.local_anchor2.coords),
                        local_axis_a: na_vec_to_b2_vec(params.local_axis1().into_inner()),
                        reference_angle: 0.0,
                        enable_limit: params.limits_enabled,
                        lower_translation: params.limits[0],
                        upper_translation: params.limits[1],
                        enable_motor: false,
                        max_motor_force: 0.0,
                        motor_speed: 0.0,
                    };

                    self.world.create_joint(&def);
                }
            }
        }
    }

    fn create_fixture(collider: &Collider, body: &mut b2::MetaBody<NoUserData>) {
        let center = na_vec_to_b2_vec(collider.position_wrt_parent().translation.vector);
        let mut fixture_def = b2::FixtureDef::new();

        fixture_def.restitution = collider.restitution;
        fixture_def.friction = collider.friction;
        fixture_def.density = collider.density().unwrap_or(1.0);
        fixture_def.is_sensor = collider.is_sensor();
        fixture_def.filter = b2::Filter::new();

        let shape = collider.shape();

        if let Some(b) = shape.as_ball() {
            let mut b2_shape = b2::CircleShape::new();
            b2_shape.set_radius(b.radius);
            b2_shape.set_position(center);
            body.create_fixture(&b2_shape, &mut fixture_def);
        } else if let Some(p) = shape.as_convex_polygon() {
            let vertices: Vec<_> = p
                .points()
                .iter()
                .map(|p| na_vec_to_b2_vec(p.coords))
                .collect();
            let b2_shape = b2::PolygonShape::new_with(&vertices);
            body.create_fixture(&b2_shape, &mut fixture_def);
        } else if let Some(c) = shape.as_cuboid() {
            let b2_shape = b2::PolygonShape::new_box(c.half_extents.x, c.half_extents.y);
            body.create_fixture(&b2_shape, &mut fixture_def);
        // } else if let Some(polygon) = shape.as_polygon() {
        //     let points: Vec<_> = poly
        //         .vertices()
        //         .iter()
        //         .map(|p| collider.position_wrt_parent() * p)
        //         .map(|p| na_vec_to_b2_vec(p.coords))
        //         .collect();
        //     let b2_shape = b2::PolygonShape::new_with(&points);
        //     body.create_fixture(&b2_shape, &mut fixture_def);
        } else if let Some(heightfield) = shape.as_heightfield() {
            let mut segments = heightfield.segments();
            let seg1 = segments.next().unwrap();
            let mut vertices = vec![
                na_vec_to_b2_vec(seg1.a.coords),
                na_vec_to_b2_vec(seg1.b.coords),
            ];

            // TODO: this will not handle holes properly.
            segments.for_each(|seg| {
                vertices.push(na_vec_to_b2_vec(seg.b.coords));
            });

            let b2_shape = b2::ChainShape::new_chain(&vertices);
            body.create_fixture(&b2_shape, &mut fixture_def);
        } else {
            eprintln!("Creating a shape unknown to the Box2d backend.");
        }
    }

    pub fn step(&mut self, counters: &mut Counters, params: &IntegrationParameters) {
        counters.step_started();
        self.world.step(
            params.dt,
            params.max_velocity_iterations as i32,
            params.max_position_iterations as i32,
        );
        counters.step_completed();
    }

    pub fn sync(&self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
        for (handle, body) in bodies.iter_mut() {
            if let Some(pb2_handle) = self.rapier2box2d.get(&handle) {
                let b2_body = self.world.body(*pb2_handle);
                let pos = b2_transform_to_na_isometry(b2_body.transform().clone());
                body.set_position(pos, false);

                for coll_handle in body.colliders() {
                    let collider = &mut colliders[*coll_handle];
                    collider.set_position_debug(pos * collider.position_wrt_parent());
                }
            }
        }
    }
}
