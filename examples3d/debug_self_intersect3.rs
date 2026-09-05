//! The 3D self-intersection recovery scenarios of `crates/rapier3d/tests/self_intersect_probe3.rs`
//! (a pinned slab and a folded cloth with a vertex parked below the bottom face or layer) and the
//! re-firing crossing-guard scenes of `crates/rapier3d/tests/crossing_guard3.rs`, side by side.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * A ground plane for orientation (the tangles hang above it).
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.5, 0.0)),
        ColliderBuilder::cuboid(8.0, 0.5, 4.0),
    );

    /*
     * A volumetric slab pinned at both end faces, with its top-middle vertex parked below
     * the bottom face.
     */
    let slab_origin = Vector::new(-3.0, 0.0, 0.0);
    let builder = SoftBodyBuilder::cuboid(
        slab_origin + Vector::new(0.0, 0.15, 0.0),
        Vector::new(1.5, 0.15, 0.15),
        3,
        2,
        2,
    )
    .softness(SpringCoefficients::new(5.0, 1.0));
    let pinned: Vec<u32> = builder
        .particle_positions()
        .iter()
        .enumerate()
        .filter(|(_, p)| (p.x - slab_origin.x).abs() > 1.4)
        .map(|(i, _)| i as u32)
        .collect();
    let target = slab_origin + Vector::new(0.0, 0.3, 0.15);
    let captured = builder
        .particle_positions()
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            (**a - target)
                .length()
                .partial_cmp(&(**b - target).length())
                .unwrap()
        })
        .unwrap()
        .0;
    let slab =
        world.insert_soft_body(builder.pinned_particles(pinned).self_contacts(true));
    world.soft_bodies[slab]
        .set_particle_position(captured, slab_origin + Vector::new(0.3, -0.4, 0.0));

    /*
     * A cloth strip folded into two layers (cell-less), both end columns pinned, with one
     * top-layer vertex parked below the bottom layer so its triangles pierce it.
     */
    let cloth_origin = Vector::new(1.5, 0.0, -0.25);
    let (nu, nv) = (8usize, 3usize);
    let cloth = world.insert_soft_body(
        SoftBodyBuilder::cloth(
            cloth_origin,
            Vector::new(0.25, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.25),
            nu,
            nv,
        )
        .material(SoftBodyMaterial::uniform(SpringCoefficients::new(40.0, 1.0)))
        .softness(SpringCoefficients::new(40.0, 1.0))
        .particle_mass(0.02)
        .particle_radius(0.05)
        // Both ends of the strip pinned: the outer edge of the bottom layer at its rest
        // place, the free edge of the folded-back top layer above it (positions set below).
        .pinned_particles((0..nv as u32).chain((nu as u32 - 1) * nv as u32..(nu * nv) as u32))
        .self_contacts(true),
    );
    // Fold columns 4..8 back over the first four, one gap above them.
    let idx = |i: usize, j: usize| i * nv + j;
    for i in 4..nu {
        for j in 0..nv {
            let x = 0.25 * (7 - i) as Real;
            world.soft_bodies[cloth].set_particle_position(
                idx(i, j),
                cloth_origin + Vector::new(x, 0.15, 0.25 * j as Real),
            );
        }
    }
    // Park a middle vertex of the top layer below the bottom layer.
    world.soft_bodies[cloth]
        .set_particle_position(idx(5, 1), cloth_origin + Vector::new(0.5, -0.15, 0.25));

    /*
     * Guard scene: a folded cloth (clean start) whose top-layer middle vertex is flicked
     * straight down through the bottom layer.
     */
    let flick_origin = Vector::new(-6.5, 0.0, -0.25);
    let flick = world.insert_soft_body(
        SoftBodyBuilder::cloth(
            flick_origin,
            Vector::new(0.25, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.25),
            nu,
            nv,
        )
        .material(SoftBodyMaterial::uniform(SpringCoefficients::new(40.0, 1.0)))
        .softness(SpringCoefficients::new(40.0, 1.0))
        .particle_mass(0.02)
        .particle_radius(0.05)
        .pinned_particles((0..nv as u32).chain((nu as u32 - 1) * nv as u32..(nu * nv) as u32))
        .self_contacts(true),
    );
    for i in 4..nu {
        for j in 0..nv {
            let x = 0.25 * (7 - i) as Real;
            world.soft_bodies[flick].set_particle_position(
                idx(i, j),
                flick_origin + Vector::new(x, 0.2, 0.25 * j as Real),
            );
        }
    }
    let flick_rest: Vec<Vector> = world.soft_bodies[flick].particle_positions().collect();

    /*
     * Guard scene: a soft ball shot straight down at a pinned cloth faster than the
     * contact reach.
     */
    let cannon_origin = Vector::new(5.0, 1.2, 0.0);
    let n = 12;
    let extent = 0.12 * (n - 1) as Real / 2.0;
    let pinned_edge = |k: usize| {
        let (i, j) = (k / n, k % n);
        i == 0 || j == 0 || i == n - 1 || j == n - 1
    };
    world.insert_soft_body(
        SoftBodyBuilder::cloth(
            cannon_origin + Vector::new(-extent, 0.0, -extent),
            Vector::new(0.12, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.12),
            n,
            n,
        )
        .material(SoftBodyMaterial::uniform(SpringCoefficients::new(40.0, 1.0)))
        .softness(SpringCoefficients::new(40.0, 1.0))
        .particle_mass(0.02)
        .particle_radius(0.05)
        .pinned_particles((0..n * n).filter(|&k| pinned_edge(k)).map(|k| k as u32))
        .self_contacts(true),
    );
    let bullet = world.insert_soft_body(
        SoftBodyBuilder::sphere(cannon_origin + Vector::new(0.0, 1.4, 0.0), 0.22, 1)
            .softness(SpringCoefficients::new(20.0, 1.0))
            .particle_mass(0.05),
    );
    let bullet_rest: Vec<Vector> = world.soft_bodies[bullet].particle_positions().collect();

    /*
     * Guard scene: two perpendicular strips, the top one shot down across the bottom one
     * (the first touch is edge on edge).
     */
    let strips_origin = Vector::new(8.5, 0.0, 0.0);
    let strip = |origin: Vector, du: Vector, dv: Vector| {
        SoftBodyBuilder::cloth(origin, du, dv, 12, 2)
            .material(SoftBodyMaterial::uniform(SpringCoefficients::new(40.0, 1.0)))
            .softness(SpringCoefficients::new(40.0, 1.0))
            .particle_mass(0.02)
            .particle_radius(0.05)
            .self_contacts(true)
    };
    world.insert_soft_body(
        strip(
            strips_origin + Vector::new(-1.1, 0.3, -0.1),
            Vector::new(0.2, 0.0, 0.0),
            Vector::new(0.0, 0.0, 0.2),
        )
        .pinned_particles([0, 1, 22, 23]),
    );
    let top_strip = world.insert_soft_body(strip(
        strips_origin + Vector::new(-0.1, 1.3, -1.1),
        Vector::new(0.0, 0.0, 0.2),
        Vector::new(0.2, 0.0, 0.0),
    ));
    let top_rest: Vec<Vector> = world.soft_bodies[top_strip].particle_positions().collect();

    // Teleports every re-fire reset the guard's trajectory baseline by design.
    let reset = |world: &mut PhysicsWorld,
                 h: SoftBodyHandle,
                 rest: &[Vector],
                 vel: &dyn Fn(usize) -> Vector| {
        for (i, p) in rest.iter().enumerate() {
            world.soft_bodies[h].set_particle_position(i, *p);
            world.soft_bodies[h].set_particle_velocity(i, vel(i));
        }
    };
    let refire = |world: &mut PhysicsWorld| {
        reset(world, flick, &flick_rest, &|i| {
            if i == idx(5, 1) {
                Vector::new(0.0, -40.0, 0.0)
            } else {
                Vector::ZERO
            }
        });
        reset(world, bullet, &bullet_rest, &|_| Vector::new(0.0, -60.0, 0.0));
        reset(world, top_strip, &top_rest, &|_| Vector::new(0.0, -30.0, 0.0));
    };
    refire(&mut world);

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(1.0, 3.5, 11.0), Vec3::new(1.0, 0.5, 0.0));

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            let dt = world.integration_parameters.dt;
            t += dt;
            if (t % 4.0) < dt && t > 0.0 {
                refire(&mut world);
            }
            world.step();
        }
    }
    Ok(())
}
