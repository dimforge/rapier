//! Joints must be solved BEFORE contacts in every solver pass: with few
//! iterations per substep, the constraint solved last on a body wins its
//! velocity residual, and a joint solved after the contacts of a much heavier
//! contacting body re-imposes the joint velocity on its light body — letting
//! the heavy body push straight through it.
//!
//! This reproduces the `Spring Joints` testbed demo: heavy cubes (200x the
//! ball mass) dropped onto light balls hanging from spring joints. When the
//! solve order flips to contacts-then-joints, every cube tunnels through its
//! ball. The pair count matters: with few pairs all constraints share the
//! staged solver's worker-0 overflow stage (whose internal order was correct),
//! so the bug only appeared once the contacts got colored stages of their own.

#[cfg(feature = "dim2")]
use rapier2d::prelude::*;
#[cfg(feature = "dim3")]
use rapier3d::prelude::*;

#[cfg(feature = "dim2")]
fn vect(x: f32, y: f32) -> Vector {
    Vec2::new(x, y)
}
#[cfg(feature = "dim3")]
fn vect(x: f32, y: f32) -> Vector {
    Vec3::new(x, y, 0.0)
}

#[test]
fn heavy_cubes_rest_on_spring_jointed_balls() {
    let mut world = PhysicsWorld::new();

    let ground_handle = world.insert_body(RigidBodyBuilder::fixed());

    let num = 30;
    let radius = 0.5;
    let mass = Ball::new(radius).mass_properties(1.0).mass();
    let stiffness = 1.0e3;
    let critical_damping = 2.0 * (stiffness * mass).sqrt();
    let mut pairs = vec![];
    for i in 0..=num {
        let ball_pos = vect(-6.0 + 1.5 * i as f32, 4.5);
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(ball_pos)
            .can_sleep(false);
        let (ball, _) = world.insert(rigid_body, ColliderBuilder::ball(radius));

        let damping_ratio = i as f32 / (num as f32 / 2.0);
        let damping = damping_ratio * critical_damping;
        let joint = SpringJointBuilder::new(0.0, stiffness, damping)
            .local_anchor1(ball_pos - Vector::Y * 3.0);
        world.insert_impulse_joint(ground_handle, ball, joint);

        let rigid_body = RigidBodyBuilder::dynamic().translation(ball_pos + Vector::Y * 5.0);
        #[cfg(feature = "dim2")]
        let collider = ColliderBuilder::cuboid(radius, radius).density(100.0);
        #[cfg(feature = "dim3")]
        let collider = ColliderBuilder::cuboid(radius, radius, radius).density(100.0);
        let (cube, _) = world.insert(rigid_body, collider);
        pairs.push((ball, cube));
    }

    for _ in 0..300 {
        world.step();
    }

    for (i, (ball, cube)) in pairs.iter().enumerate() {
        let ball_y = world.bodies[*ball].translation().y;
        let cube_y = world.bodies[*cube].translation().y;
        assert!(
            cube_y > ball_y,
            "cube {i} tunneled through its spring-jointed ball \
             (cube y = {cube_y:.3}, ball y = {ball_y:.3})"
        );
    }
}
