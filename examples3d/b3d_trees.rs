//! Ports of box3d's `trees100`/`trees50`/`trees25` benchmarks
//! (`CreateTrees(1|2|4)`, `box3d/shared/benchmarks.c`): 50 "tree" bodies, each
//! a stack of 22 convex cylinders, spun onto a sinusoidal wave-mesh ground.
//! The scale selects the ground tessellation density.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;
use std::f32::consts::PI;

pub async fn run100(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    run(viewer, 1).await
}

pub async fn run50(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    run(viewer, 2).await
}

pub async fn run25(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    run(viewer, 4).await
}

async fn run(viewer: &mut TestbedViewer, scale: usize) -> anyhow::Result<()> {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0, 0.0);
    create_trees(&mut world, scale);

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(0.0, 30.0, 140.0), Vec3::new(0.0, 15.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}

/// Port of box3d's `CreateTrees` (`box3d/shared/benchmarks.c`). `scale` is
/// 1/2/4 for the trees100/50/25 variants. Builds a sinusoidal wave-mesh ground
/// and 50 "tree" bodies, each a stack of 22 convex cylinders, spun about Z with
/// alternating direction (release settings, `tilt = 0`).
fn create_trees(world: &mut PhysicsWorld, scale: usize) {
    let x_count = scale * 150;
    let z_count = scale * 200;
    let cell_width = 1.0 / scale as f32;
    let amplitude = 0.4;
    let row_hz = 0.05;
    let column_hz = 0.1;

    let (vertices, indices) =
        create_wave_mesh(x_count, z_count, cell_width, amplitude, row_hz, column_hz);
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    world.insert_collider(
        ColliderBuilder::trimesh(vertices, indices).unwrap(),
        Some(ground),
    );

    // The 22 stacked cylinders shared by every tree body. Building each hull
    // once (as a shared shape) lets all 50 trees reuse the geometry and lets
    // the renderer instance the repeated hulls.
    let hull_count = 22usize;
    let mut hulls = Vec::with_capacity(hull_count);
    let mut y = 1.0f32;
    let mut r = 0.75f32;
    let l = 1.5f32;
    for _ in 0..hull_count {
        let points = create_cylinder(l + 2.0 * r, r, y - r, 6);
        hulls.push(SharedShape::convex_hull(&points).unwrap());
        y += l + 2.0 * r;
        r *= 0.95;
    }

    let body_count = 50i32;
    let mut angular_velocity = -0.5f32;
    let mut z = -70.0f32;
    for body_index in 0..body_count {
        let position = Vector::new(0.0, 1.0, z);
        // box3d applies gyroscopic torque to every body every substep (see
        // `b3IntegrateVelocitiesTask`, "improves the simulation of long skinny
        // bodies"). rapier gates that term behind this flag, off by default, so
        // enable it here or the ~1300:1-inertia trees tumble along wrong paths.
        let handle = world.insert_body(RigidBodyBuilder::dynamic().translation(position));
        for hull in &hulls {
            world.insert_collider(
                ColliderBuilder::new(hull.clone())
                    .density(1.0)
                    .friction(0.9),
                Some(handle),
            );
        }

        let velocity_scale = 0.5 + (0.5 * body_index as f32) / body_count as f32;
        let center = world.bodies[handle].center_of_mass();
        let omega = Vector::new(0.0, 0.0, velocity_scale * angular_velocity);
        let v = omega.cross(center - position);
        let body = &mut world.bodies[handle];
        body.set_angvel(omega, true);
        body.set_linvel(v, true);

        z += 3.0;
        angular_velocity = -angular_velocity;
    }
}

/// box3d `b3CreateWaveMesh` (`src/mesh.c`): a grid whose height is a product of
/// two sinusoids, used as the "trees" benchmark ground.
fn create_wave_mesh(
    x_count: usize,
    z_count: usize,
    cell_width: f32,
    amplitude: f32,
    row_frequency: f32,
    column_frequency: f32,
) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let omega_z = 2.0 * PI * row_frequency * cell_width;
    let omega_x = 2.0 * PI * column_frequency * cell_width;
    grid_mesh(x_count, z_count, cell_width, |ix, iz| {
        amplitude * (omega_x * ix as f32).sin() * (omega_z * iz as f32).sin()
    })
}

/// Shared vertex/index grid builder for the box3d grid/wave meshes. `height`
/// yields the Y coordinate at grid cell `(ix, iz)`.
fn grid_mesh(
    x_count: usize,
    z_count: usize,
    cell_width: f32,
    height: impl Fn(usize, usize) -> f32,
) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let x_width = cell_width * x_count as f32;
    let z_width = cell_width * z_count as f32;

    let mut vertices = Vec::with_capacity((x_count + 1) * (z_count + 1));
    let mut x = -0.5 * x_width;
    for ix in 0..=x_count {
        let mut z = -0.5 * z_width;
        for iz in 0..=z_count {
            vertices.push(Vector::new(x, height(ix, iz), z));
            z += cell_width;
        }
        x += cell_width;
    }

    let mut indices = Vec::with_capacity(2 * x_count * z_count);
    for ix in 0..x_count {
        for iz in 0..z_count {
            let i1 = (iz + (z_count + 1) * ix) as u32;
            let i2 = i1 + 1;
            let i3 = i2 + (z_count as u32 + 1);
            let i4 = i3 - 1;
            indices.push([i1, i2, i3]);
            indices.push([i3, i4, i1]);
        }
    }
    (vertices, indices)
}

/// box3d `b3CreateGridMesh` (`src/mesh.c`): a flat `x_count * z_count` grid.
fn create_grid_mesh(
    x_count: usize,
    z_count: usize,
    cell_width: f32,
) -> (Vec<Vector>, Vec<[u32; 3]>) {
    grid_mesh(x_count, z_count, cell_width, |_, _| 0.0)
}

/// box3d `b3CreateCylinder` (`src/hull.c`): `2 * sides` points forming a
/// cylinder of the given `height`/`radius`, its base at `y_offset`, aligned
/// with the Y axis. Returned as a convex point cloud (rapier builds the hull).
fn create_cylinder(height: f32, radius: f32, y_offset: f32, sides: usize) -> Vec<Vector> {
    let mut points = Vec::with_capacity(2 * sides);
    let delta_alpha = 2.0 * PI / sides as f32;
    let mut alpha = 0.0f32;
    for _ in 0..sides {
        let (sin_a, cos_a) = alpha.sin_cos();
        points.push(Vector::new(radius * cos_a, y_offset, radius * sin_a));
        points.push(Vector::new(
            radius * cos_a,
            y_offset + height,
            radius * sin_a,
        ));
        alpha += delta_alpha;
    }
    points
}
