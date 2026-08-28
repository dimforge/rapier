//! Stress test for cloth: a stack of sheets sliding down a slope against each other, a banner
//! stretched to twice its length and released periodically, and a strip twisted around its axis
//! by a rotating clamp (self contacts on) until it winds into a rope.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(30.0, 0.5, 30.0),
    );
    let cloth = |origin: Vector, du: Vector, dv: Vector, nu: usize, nv: usize| {
        SoftBodyBuilder::cloth(origin, du, dv, nu, nv)
            .softness(SpringCoefficients::new(30.0, 1.0))
            .material(SoftBodyMaterial {
                bend_softness: SpringCoefficients::new(3.0, 1.0),
                ..SoftBodyMaterial::uniform(SpringCoefficients::new(30.0, 1.0))
            })
            .particle_mass(0.02)
            .particle_radius(0.05)
            .surface_collider(ColliderBuilder::ball(0.05).friction(0.5))
    };

    /*
     * Sheets sliding down a slope, on top of each other, into a stopper.
     */
    let slope = 0.5;
    world.insert(
        RigidBodyBuilder::fixed()
            .translation(Vector::new(-6.0, 2.0, 0.0))
            .rotation(Vector::new(0.0, 0.0, -slope)),
        ColliderBuilder::cuboid(5.0, 0.2, 4.0).friction(0.4),
    );
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(-0.6, 0.6, 0.0)),
        ColliderBuilder::cuboid(0.2, 0.6, 4.0),
    );
    let rot = Rotation::from_axis_angle(Vector::Z, -slope);
    for k in 0..6 {
        let origin = Vector::new(-6.0, 2.0, 0.0)
            + rot * Vector::new(-3.5 + k as Real * 0.3, 0.3 + k as Real * 0.12, -1.5);
        world.insert_soft_body(cloth(
            origin,
            rot * Vector::new(0.15, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.15),
            21,
            21,
        ));
    }

    /*
     * A banner pinned along both ends; the right end is pulled away and brought back.
     */
    let (nu, nv) = (31, 13);
    let banner_origin = Vector::new(2.0, 4.0, -4.0);
    let left: Vec<u32> = (0..nv).map(|j| j as u32).collect();
    let right: Vec<u32> = (0..nv).map(|j| ((nu - 1) * nv + j) as u32).collect();
    let banner = world.insert_soft_body(
        cloth(
            banner_origin,
            Vector::new(0.15, 0.0, 0.0),
            Vector::new(0.0, -0.15, 0.0),
            nu,
            nv,
        )
        .pinned_particles(left.iter().chain(right.iter()).copied()),
    );
    let banner_right_rest: Vec<Vector> = right
        .iter()
        .map(|&i| world.soft_bodies[banner].particle_position(i as usize))
        .collect();

    /*
     * A strip pinned top and bottom; the bottom clamp rotates about the vertical axis.
     */
    let (su, sv) = (11, 41);
    let strip_origin = Vector::new(9.0, 6.5, -0.75);
    let top: Vec<u32> = (0..su).map(|i| (i * sv) as u32).collect();
    let bottom: Vec<u32> = (0..su).map(|i| (i * sv + sv - 1) as u32).collect();
    let strip = world.insert_soft_body(
        cloth(
            strip_origin,
            Vector::new(0.0, 0.0, 0.15),
            Vector::new(0.0, -0.15, 0.0),
            su,
            sv,
        )
        .pinned_particles(top.iter().chain(bottom.iter()).copied())
        .self_contacts(true),
    );
    let strip_axis = Vector::new(9.0, 0.0, 0.0);
    let strip_bottom_rest: Vec<Vector> = bottom
        .iter()
        .map(|&i| world.soft_bodies[strip].particle_position(i as usize))
        .collect();

    // Drives the pinned clamps: stretch/release the banner, twist the strip.
    let animate = move |world: &mut PhysicsWorld, t: Real| {
        let stretch = 1.5 * (1.0 - (0.5 * t).cos());
        let banner_sb = &mut world.soft_bodies[banner];
        for (&i, rest) in right.iter().zip(banner_right_rest.iter()) {
            banner_sb
                .set_particle_kinematic_target(i as usize, *rest + Vector::new(stretch, 0.0, 0.0));
        }
        let twist = Rotation::from_axis_angle(Vector::Y, 0.8 * t);
        let strip_sb = &mut world.soft_bodies[strip];
        for (&i, rest) in bottom.iter().zip(strip_bottom_rest.iter()) {
            let target = strip_axis + twist * (*rest - strip_axis);
            strip_sb.set_particle_kinematic_target(i as usize, target);
        }
    };

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(2.0, 8.0, 18.0), Vec3::new(1.5, 3.0, 0.0));

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            t += world.integration_parameters.dt;
            animate(&mut world, t);
            world.step();
        }
    }
    Ok(())
}
