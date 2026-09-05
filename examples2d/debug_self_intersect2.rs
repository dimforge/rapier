//! The self-intersection recovery scenarios of `crates/rapier2d/tests/self_intersect_probe.rs`
//! side by side (strip tangles, a fast blob stopped by the crossing guard, the 8-shaped blob
//! family): without recovery the self-contacts hold each tangle; the stand-down pulls them out.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * A captured strip: ends pinned, middle-top particle parked below the bottom boundary.
     */
    let strip = |center: Vector| {
        SoftBodyBuilder::grid(center, Vector::new(3.0, 0.15), 3, 2)
            .pinned_particles([0, 1, 4, 5])
            .softness(SpringCoefficients::new(3.0, 1.0))
            .self_contacts(true)
    };
    let captured = world.insert_soft_body(strip(Vector::new(-16.0, 0.0)));
    world.soft_bodies[captured].set_particle_position(3, Vector::new(-15.0, -0.4));

    /*
     * The same capture loaded against its recovery by a heavy resting ball.
     */
    let loaded = world.insert_soft_body(strip(Vector::new(-6.0, 0.0)));
    world.soft_bodies[loaded].set_particle_position(3, Vector::new(-5.0, -0.4));
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(-5.7, 0.8)),
        ColliderBuilder::ball(0.45).density(20.0),
    );

    /*
     * A cell-less blob (edges + boundary volume) with its top vertex below the bottom wall, over
     * a floor; too soft to escape by tunneling, only the self-crossing stand-down frees it.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(2.0, -1.0)),
        ColliderBuilder::cuboid(2.5, 0.5),
    );
    let blob = world.insert_soft_body(
        SoftBodyBuilder::disk(Vector::new(2.0, 0.0), 0.5, 16)
            .softness(SpringCoefficients::new(4.0, 1.0))
            .self_contacts(true)
            .particle_mass(0.05),
    );
    // Vertex 4 sits at the top of the disk (angle 90°); park it below the bottom wall.
    world.soft_bodies[blob].set_particle_position(4, Vector::new(2.0, -0.8));

    /*
     * A strip whose top boundary is grabbed (like the testbed mouse) and ground into the interior
     * past the middle row: the cells invert while held, and the boundary must never cross itself.
     */
    let grab_origin = Vector::new(12.0, 0.0);
    world.insert(
        RigidBodyBuilder::fixed().translation(grab_origin + Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(2.5, 0.5),
    );
    let bottom_row = (0..7).map(|i| i * 3);
    let ground_strip = world.insert_soft_body(
        SoftBodyBuilder::grid(grab_origin + Vector::new(0.0, 0.15), Vector::new(1.5, 0.15), 7, 3)
            .pinned_particles(bottom_row)
            .softness(SpringCoefficients::new(5.0, 1.0))
            .self_contacts(true),
    );
    let grabbed = 11u32;
    let anchor = world.soft_bodies[ground_strip].particle_position(grabbed as usize);
    let cluster = world.add_soft_body_cluster(ground_strip, &[grabbed]).unwrap();
    let proxy = world.soft_bodies[ground_strip].cluster_proxy(cluster).unwrap();
    let mouse =
        world.insert_body(RigidBodyBuilder::kinematic_position_based().translation(anchor));
    let joint: GenericJoint = GenericJointBuilder::new(JointAxesMask::empty())
        .motor_position(JointAxis::LinX, 0.0, 1000.0, 50.0)
        .motor_position(JointAxis::LinY, 0.0, 1000.0, 50.0)
        .into();
    world.insert_impulse_joint(mouse, proxy, joint);

    /*
     * The crossing-guard cannon: a blob shot at a thin pinned strip faster than the contact
     * reach, re-teleported and re-shot every few seconds.
     */
    let cannon_origin = Vector::new(20.5, 0.0);
    let cannon_strip = SoftBodyBuilder::grid(cannon_origin, Vector::new(2.0, 0.1), 9, 2)
        .pinned_particles([0, 1, 16, 17])
        .softness(SpringCoefficients::new(10.0, 1.0))
        .self_contacts(true);
    world.insert_soft_body(cannon_strip);
    let bullet = world.insert_soft_body(
        SoftBodyBuilder::disk(cannon_origin + Vector::new(0.0, 2.5), 0.3, 12)
            .softness(SpringCoefficients::new(20.0, 1.0))
            .particle_mass(0.2)
            .self_contacts(true),
    );
    let reload = |world: &mut PhysicsWorld| {
        let n = world.soft_bodies[bullet].particles().len();
        for i in 0..n {
            let angle = core::f32::consts::TAU as Real * i as Real / n as Real;
            let p = cannon_origin
                + Vector::new(0.0, 2.5)
                + Vector::new(angle.cos(), angle.sin()) * 0.3;
            world.soft_bodies[bullet].set_particle_position(i, p);
            world.soft_bodies[bullet].set_particle_velocity(i, Vector::new(0.0, -150.0));
        }
    };
    reload(&mut world);

    /*
     * The 8-shaped blob family (bottom row): a disk blob reshaped into a figure-eight, one lobe
     * keeping the rest winding, the other wound backward.
     */
    let eight_blob = |world: &mut PhysicsWorld, center: Vector, radius: Real, n: usize| {
        world.insert_soft_body(
            SoftBodyBuilder::disk(center, radius, n)
                .softness(SpringCoefficients::new(4.0, 1.0))
                .self_contacts(true)
                .particle_mass(0.05),
        )
    };
    let gerono_eight = |world: &mut PhysicsWorld, h: SoftBodyHandle, center: Vector, r: Real| {
        let n = world.soft_bodies[h].particles().len();
        for i in 0..n {
            let t = core::f32::consts::TAU as Real * i as Real / n as Real;
            let p = center + Vector::new(r * t.cos(), r * t.sin() * t.cos());
            world.soft_bodies[h].set_particle_position(i, p);
        }
    };
    let asym_eight =
        |world: &mut PhysicsWorld, h: SoftBodyHandle, center: Vector, rb: Real, rs: Real| {
            let n = world.soft_bodies[h].particles().len();
            let n_big = (n as Real * rb / (rb + rs)) as usize;
            let tau = core::f32::consts::TAU as Real;
            for i in 0..n {
                let p = if i < n_big {
                    let t = tau * i as Real / n_big as Real;
                    center + Vector::new(-rb + rb * t.cos(), rb * t.sin())
                } else {
                    let t = tau * (i - n_big) as Real / (n - n_big) as Real;
                    center + Vector::new(rs - rs * t.cos(), -rs * t.sin())
                };
                world.soft_bodies[h].set_particle_position(i, p);
            }
        };
    // Wide enough for a de-overlapping pair to drift on (a 2.0 half-width let the upper
    // 8 of the penetrating pair slide off the edge).
    let eight_floor = |world: &mut PhysicsWorld, x: Real| {
        world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(x, -6.5)),
            ColliderBuilder::cuboid(3.0, 0.5),
        );
    };

    // An asymmetric 8 (small mirrored lobe): recovers to an O.
    eight_floor(&mut world, -14.0);
    let asym = eight_blob(&mut world, Vector::new(-14.0, -5.35), 0.6, 24);
    asym_eight(&mut world, asym, Vector::new(-13.9, -5.35), 0.55, 0.3);

    // A balanced 8: a second stable elastic equilibrium, resolved by the active untangler.
    eight_floor(&mut world, -7.0);
    let sym = eight_blob(&mut world, Vector::new(-7.0, -5.35), 0.6, 24);
    gerono_eight(&mut world, sym, Vector::new(-7.0, -5.35), 0.6);

    // Two 8s penetrating each other: unknots and de-overlaps.
    eight_floor(&mut world, 1.0);
    let e1 = eight_blob(&mut world, Vector::new(0.65, -5.35), 0.6, 24);
    gerono_eight(&mut world, e1, Vector::new(0.65, -5.35), 0.6);
    let e2 = eight_blob(&mut world, Vector::new(1.35, -5.05), 0.6, 24);
    gerono_eight(&mut world, e2, Vector::new(1.35, -5.05), 0.6);

    // An 8 whose mirrored lobe starts swallowed inside a bigger blob: recovers fully.
    eight_floor(&mut world, 8.0);
    let swallowed = eight_blob(&mut world, Vector::new(8.0, -5.35), 0.6, 24);
    asym_eight(&mut world, swallowed, Vector::new(7.8, -5.35), 0.55, 0.3);
    eight_blob(&mut world, Vector::new(8.65, -5.35), 0.55, 20);

    // An asymmetric 8 mutually penetrating a plain blob: unknots and de-overlaps.
    eight_floor(&mut world, 15.0);
    let overlapped = eight_blob(&mut world, Vector::new(15.0, -5.35), 0.6, 24);
    asym_eight(&mut world, overlapped, Vector::new(15.1, -5.35), 0.55, 0.3);
    eight_blob(&mut world, Vector::new(14.15, -5.35), 0.5, 20);

    // A small blob threaded through the 8's mirrored lobe: unknots and de-overlaps.
    eight_floor(&mut world, 22.0);
    let threaded = eight_blob(&mut world, Vector::new(22.0, -5.35), 0.6, 24);
    asym_eight(&mut world, threaded, Vector::new(21.8, -5.35), 0.55, 0.3);
    eight_blob(&mut world, Vector::new(22.1, -5.35), 0.22, 14);

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, -2.5), 38.0);

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            // The grab loops: pull the vertex down just above the bottom boundary, grind it
            // around while held, lift it back out, rest, repeat.
            let cycle = t % 6.0;
            let target = if cycle < 1.0 {
                Vector::new(0.0, 0.5 - 0.48 * cycle)
            } else if cycle < 4.0 {
                let w = core::f32::consts::TAU as Real * (cycle - 1.0);
                Vector::new(0.3 * w.sin(), 0.02 + 0.03 * (1.0 - (2.0 * w).cos()))
            } else if cycle < 5.0 {
                Vector::new(0.0, 0.02 + 0.48 * (cycle - 4.0))
            } else {
                Vector::new(0.0, 0.5)
            };
            world.bodies[mouse].set_next_kinematic_translation(grab_origin + target);
            world.bodies[proxy].wake_up(true);
            // Re-shoot the cannon blob every few seconds (a teleport resets the crossing
            // guard's trajectory baseline, so only the flight itself is guarded).
            let dt = world.integration_parameters.dt;
            if (t % 4.0) < dt && t > 0.0 {
                reload(&mut world);
            }
            world.step();
            t += dt;
        }
    }
    Ok(())
}
