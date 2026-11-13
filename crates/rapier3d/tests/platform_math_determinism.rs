//! Goldens for the joint paths that call `sin`, `asin`... directly: with
//! `enhanced-determinism` they must give the same bits on every target.
#![cfg(feature = "enhanced-determinism")]

use rapier3d::prelude::*;

fn digest(values: impl IntoIterator<Item = f32>) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for v in values {
        for b in v.to_bits().to_le_bytes() {
            h ^= b as u64;
            h = h.wrapping_mul(0x100_0000_01b3);
        }
    }
    h
}

fn step_and_digest(world: &mut PhysicsWorld, handles: &[RigidBodyHandle], steps: usize) -> u64 {
    let mut values = Vec::new();
    for _ in 0..steps {
        world.step();
        for handle in handles {
            let body = &world.bodies[*handle];
            let (t, r) = (body.translation(), body.rotation());
            let (v, w) = (body.linvel(), body.angvel());
            values.extend([
                t.x, t.y, t.z, r.x, r.y, r.z, r.w, v.x, v.y, v.z, w.x, w.y, w.z,
            ]);
        }
    }
    digest(values)
}

fn pendulums(
    world: &mut PhysicsWorld,
    joints: impl IntoIterator<Item = RevoluteJointBuilder>,
    multibody: bool,
) -> Vec<RigidBodyHandle> {
    joints
        .into_iter()
        .enumerate()
        .map(|(i, joint)| {
            let x = i as f32 * 3.0;
            let base =
                world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(x, 0.0, 0.0)));
            let (arm, _) = world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(x + 1.0, 0.0, 0.0))
                    .can_sleep(false),
                ColliderBuilder::cuboid(0.5, 0.1, 0.1),
            );
            let joint = joint
                .local_anchor1(Vector::ZERO)
                .local_anchor2(Vector::new(-1.0, 0.0, 0.0));
            if multibody {
                world.insert_multibody_joint(base, arm, joint).unwrap();
            } else {
                world.insert_impulse_joint(base, arm, joint);
            }
            arm
        })
        .collect()
}

#[test]
fn angular_limits() {
    let mut world = PhysicsWorld::new();
    let joints = (1..=8).map(|i| {
        RevoluteJointBuilder::new(Vector::Z).limits([-0.05 * i as f32, 0.05 * (i + 15) as f32])
    });
    let arms = pendulums(&mut world, joints, false);
    let digest = step_and_digest(&mut world, &arms, 60);
    assert_eq!(digest, 0x59ae_253d_3521_41d8, "got {digest:#x}");
}

#[test]
fn motor_position_on_multibody_link() {
    let mut world = PhysicsWorld::new();
    let links = pendulums(
        &mut world,
        [(); 5].map(|_| RevoluteJointBuilder::new(Vector::Z)),
        true,
    );
    let arms: Vec<_> = [0.19, 0.57, 0.93, 0.99, 1.09]
        .into_iter()
        .zip(&links)
        .map(|(target, link)| {
            let translation = world.bodies[*link].translation() + Vector::new(1.0, 0.0, 0.0);
            let (arm, _) = world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(translation)
                    .can_sleep(false),
                ColliderBuilder::cuboid(0.4, 0.1, 0.1),
            );
            let joint = RevoluteJointBuilder::new(Vector::Z)
                .local_anchor1(Vector::new(0.5, 0.0, 0.0))
                .local_anchor2(Vector::new(-0.5, 0.0, 0.0))
                .motor_position(target, 50.0, 5.0);
            world.insert_impulse_joint(*link, arm, joint);
            arm
        })
        .collect();
    let digest = step_and_digest(&mut world, &arms, 60);
    assert_eq!(digest, 0xec52_1711_c406_c841, "got {digest:#x}");
}

#[test]
fn revolute_joint_angle() {
    let joint = RevoluteJoint::new(Vector::Z);
    let angles = (1..=16).map(|i| {
        let rot2 = Rotation::from_rotation_z(0.19 * i as f32);
        joint.angle(&Rotation::IDENTITY, &rot2)
    });
    let digest = digest(angles);
    assert_eq!(digest, 0x384e_f971_c0fe_5a21, "got {digest:#x}");
}
