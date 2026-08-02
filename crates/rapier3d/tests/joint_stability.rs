//! Long-run joint stability regressions (testbed `ImpulseJoint prismatic`/`ball`
//! scenes): bilateral joint structures are conservative, so any per-step solver
//! energy injection compounds into a blow-up over thousands of steps.

#[cfg(feature = "dim2")]
use rapier2d::prelude::*;
#[cfg(feature = "dim3")]
use rapier3d::prelude::*;

#[cfg(feature = "dim2")]
fn vect(x: f32, y: f32, _z: f32) -> Vector {
    Vec2::new(x, y)
}
#[cfg(feature = "dim3")]
fn vect(x: f32, y: f32, z: f32) -> Vector {
    Vec3::new(x, y, z)
}

fn assert_sane(world: &PhysicsWorld, scene: &str, bound: f32) {
    let mut max_vel: f32 = 0.0;
    for (_, body) in world.bodies.iter() {
        let pos = body.translation();
        assert!(
            pos.length() < bound,
            "{scene}: body at non-sane position {pos:?}"
        );
        max_vel = max_vel.max(body.linvel().length());
    }
    assert!(
        max_vel < 100.0,
        "{scene}: runaway velocity {max_vel} (energy is being injected)"
    );
}

/// Hanging chains of boxes on limited prismatic joints along alternating diagonal
/// rails (the `ImpulseJoint prismatic` stress scene): the chains hang from the
/// engaged limit rows, so any limit-row energy injection accumulates.
#[test]
fn prismatic_limit_chains_remain_stable() {
    let mut world = PhysicsWorld::new();
    let rad = 0.4;
    let shift = 1.0;
    let num = 10;

    for chain in 0..10 {
        let x = chain as f32 * 4.0;
        let ground = RigidBodyBuilder::fixed().translation(vect(x, 0.0, 0.0));
        let mut curr_parent = world.bodies.insert(ground);
        world.colliders.insert_with_parent(
            #[cfg(feature = "dim2")]
            ColliderBuilder::cuboid(rad, rad),
            #[cfg(feature = "dim3")]
            ColliderBuilder::cuboid(rad, rad, rad),
            curr_parent,
            &mut world.bodies,
        );

        for i in 0..num {
            let y = -(i + 1) as f32 * shift;
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vect(x, y, 0.0))
                .can_sleep(false);
            let curr_child = world.bodies.insert(rigid_body);
            world.colliders.insert_with_parent(
                #[cfg(feature = "dim2")]
                ColliderBuilder::cuboid(rad, rad),
                #[cfg(feature = "dim3")]
                ColliderBuilder::cuboid(rad, rad, rad),
                curr_child,
                &mut world.bodies,
            );

            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            #[cfg(feature = "dim2")]
            let axis = Vec2::new(sign, 1.0).normalize();
            #[cfg(feature = "dim3")]
            let axis = Vec3::new(sign, 1.0, 0.0).normalize();

            let prism = PrismaticJointBuilder::new(axis)
                .local_anchor2(vect(0.0, shift, 0.0))
                .limits([-1.5, 1.5]);
            world
                .impulse_joints
                .insert(curr_parent, curr_child, prism, true);

            curr_parent = curr_child;
        }
    }

    for k in 0..10_000 {
        world.step();
        if k % 1000 == 999 {
            assert_sane(&world, "prismatic_limit_chains", 200.0);
        }
    }
}

/// A pinned net of revolute (2D) / spherical (3D) joints (the `ImpulseJoint
/// ball` stress scene, reduced): a conservative swinging structure that heats
/// up and eventually breaks apart if the solver injects energy.
///
/// CI-scale: sized to run in a few seconds per feature config. The full-size,
/// longer-horizon validation is [`joint_net_long_run`] (`--ignored`).
#[test]
fn joint_net_remains_stable() {
    let mut world = PhysicsWorld::new();
    let n = 32;
    let rad = 0.4;
    let shift = 1.0;
    let mut handles = vec![RigidBodyHandle::invalid(); n * n];

    for i in 0..n {
        for j in 0..n {
            let pos = vect(j as f32 * shift, -(i as f32) * shift, 0.0);
            let body = if i == 0 && (j % 4 == 0 || j == n - 1) {
                RigidBodyBuilder::fixed().translation(pos)
            } else {
                RigidBodyBuilder::dynamic()
                    .translation(pos)
                    .can_sleep(false)
            };
            let handle = world.bodies.insert(body);
            world.colliders.insert_with_parent(
                ColliderBuilder::ball(rad),
                handle,
                &mut world.bodies,
            );
            handles[i * n + j] = handle;
        }
    }

    #[cfg(feature = "dim2")]
    let joint = |anchor1: Vector, anchor2: Vector| {
        RevoluteJointBuilder::new()
            .local_anchor1(anchor1)
            .local_anchor2(anchor2)
            .build()
            .data
    };
    #[cfg(feature = "dim3")]
    let joint = |anchor1: Vector, anchor2: Vector| {
        SphericalJointBuilder::new()
            .local_anchor1(anchor1)
            .local_anchor2(anchor2)
            .build()
            .data
    };

    for i in 0..n {
        for j in 0..n {
            if i > 0 {
                let a = handles[(i - 1) * n + j];
                let b = handles[i * n + j];
                world.impulse_joints.insert(
                    a,
                    b,
                    joint(vect(0.0, -shift / 2.0, 0.0), vect(0.0, shift / 2.0, 0.0)),
                    true,
                );
            }
            if j > 0 {
                let a = handles[i * n + j - 1];
                let b = handles[i * n + j];
                world.impulse_joints.insert(
                    a,
                    b,
                    joint(vect(shift / 2.0, 0.0, 0.0), vect(-shift / 2.0, 0.0, 0.0)),
                    true,
                );
            }
        }
    }

    for k in 0..4000 {
        world.step();
        if k % 1000 == 999 {
            assert_sane(&world, "joint_net", 500.0);
        }
    }
}

/// Extended manual variants: longer runs and bigger structures than CI allows.
/// Run: `cargo test --release --test joint_stability -- --ignored`
#[test]
#[ignore = "long manual stability validation"]
fn prismatic_limit_chains_long_run() {
    let mut world = PhysicsWorld::new();
    let rad = 0.4;
    let shift = 1.0;
    let num = 10;

    for chain in 0..10 {
        let x = chain as f32 * 4.0;
        let ground = RigidBodyBuilder::fixed().translation(vect(x, 0.0, 0.0));
        let mut curr_parent = world.bodies.insert(ground);
        world.colliders.insert_with_parent(
            #[cfg(feature = "dim2")]
            ColliderBuilder::cuboid(rad, rad),
            #[cfg(feature = "dim3")]
            ColliderBuilder::cuboid(rad, rad, rad),
            curr_parent,
            &mut world.bodies,
        );

        for i in 0..num {
            let y = -(i + 1) as f32 * shift;
            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(vect(x, y, 0.0))
                .can_sleep(false);
            let curr_child = world.bodies.insert(rigid_body);
            world.colliders.insert_with_parent(
                #[cfg(feature = "dim2")]
                ColliderBuilder::cuboid(rad, rad),
                #[cfg(feature = "dim3")]
                ColliderBuilder::cuboid(rad, rad, rad),
                curr_child,
                &mut world.bodies,
            );

            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            #[cfg(feature = "dim2")]
            let axis = Vec2::new(sign, 1.0).normalize();
            #[cfg(feature = "dim3")]
            let axis = Vec3::new(sign, 1.0, 0.0).normalize();

            let prism = PrismaticJointBuilder::new(axis)
                .local_anchor2(vect(0.0, shift, 0.0))
                .limits([-1.5, 1.5]);
            world
                .impulse_joints
                .insert(curr_parent, curr_child, prism, true);

            curr_parent = curr_child;
        }
    }

    for k in 0..50_000 {
        world.step();
        if k % 2000 == 1999 {
            assert_sane(&world, "prismatic_limit_chains_long_run", 200.0);
        }
    }
}

#[test]
#[ignore = "long manual stability validation"]
fn joint_net_long_run() {
    let mut world = PhysicsWorld::new();
    let n = 80;
    let rad = 0.4;
    let shift = 1.0;
    let mut handles = vec![RigidBodyHandle::invalid(); n * n];

    for i in 0..n {
        for j in 0..n {
            let pos = vect(j as f32 * shift, -(i as f32) * shift, 0.0);
            let body = if i == 0 && (j % 4 == 0 || j == n - 1) {
                RigidBodyBuilder::fixed().translation(pos)
            } else {
                RigidBodyBuilder::dynamic()
                    .translation(pos)
                    .can_sleep(false)
            };
            let handle = world.bodies.insert(body);
            world.colliders.insert_with_parent(
                ColliderBuilder::ball(rad),
                handle,
                &mut world.bodies,
            );
            handles[i * n + j] = handle;
        }
    }

    #[cfg(feature = "dim2")]
    let joint = |anchor1: Vector, anchor2: Vector| {
        RevoluteJointBuilder::new()
            .local_anchor1(anchor1)
            .local_anchor2(anchor2)
            .build()
            .data
    };
    #[cfg(feature = "dim3")]
    let joint = |anchor1: Vector, anchor2: Vector| {
        SphericalJointBuilder::new()
            .local_anchor1(anchor1)
            .local_anchor2(anchor2)
            .build()
            .data
    };

    for i in 0..n {
        for j in 0..n {
            if i > 0 {
                let a = handles[(i - 1) * n + j];
                let b = handles[i * n + j];
                world.impulse_joints.insert(
                    a,
                    b,
                    joint(vect(0.0, -shift / 2.0, 0.0), vect(0.0, shift / 2.0, 0.0)),
                    true,
                );
            }
            if j > 0 {
                let a = handles[i * n + j - 1];
                let b = handles[i * n + j];
                world.impulse_joints.insert(
                    a,
                    b,
                    joint(vect(shift / 2.0, 0.0, 0.0), vect(-shift / 2.0, 0.0, 0.0)),
                    true,
                );
            }
        }
    }

    for k in 0..20_000 {
        world.step();
        if k % 2000 == 1999 {
            assert_sane(&world, "joint_net_long_run", 800.0);
        }
    }
}
