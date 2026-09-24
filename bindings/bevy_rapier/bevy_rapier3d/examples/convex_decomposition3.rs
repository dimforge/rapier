//! Concave dynamic bodies approximated with `Collider::convex_decomposition`, falling on a
//! wavy triangle mesh built with `TriMeshFlags::FIX_INTERNAL_EDGES`.
//!
//! The meshes are generated procedurally (a torus, a tube, and a star-shaped prism) instead of
//! being loaded from files.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use std::f32::consts::TAU;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(40.0, 30.0, 40.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    /*
     * Ground: a wavy triangle mesh.
     */
    let nsubdivs = 50;
    let (vertices, indices) = grid_mesh(nsubdivs, Vec3::new(100.0, 2.0, 100.0), |i, j| {
        let n = nsubdivs as f32;
        -(i as f32 * 40.0 / n / 2.0).cos() - (j as f32 * 40.0 / n / 2.0).cos()
    });
    commands.spawn(
        Collider::trimesh_with_flags(vertices, indices, TriMeshFlags::FIX_INTERNAL_EDGES).unwrap(),
    );

    /*
     * Create the convex decompositions.
     */
    let meshes = [
        torus_mesh(3.0, 1.0, 24, 12),
        tube_mesh(3.5, 2.5, 4.0, 24),
        star_prism_mesh(4.0, 1.8, 2.0, 5),
    ];
    let colors = [
        Hsla::hsl(220.0, 1.0, 0.3),
        Hsla::hsl(180.0, 1.0, 0.3),
        Hsla::hsl(30.0, 1.0, 0.5),
    ];
    let num_duplications = 4;
    let shift_xz = 10.0;

    for (igeom, ((vertices, indices), color)) in meshes.into_iter().zip(colors).enumerate() {
        info!("Decomposing mesh {igeom}.");
        // The decomposition is computed once and shared by all the duplicates.
        let collider = Collider::convex_decomposition(&vertices, &indices);

        for k in 0..num_duplications {
            let x = (igeom as f32 - 1.0) * shift_xz;
            let z = (k as f32 - (num_duplications - 1) as f32 / 2.0) * shift_xz;
            let rotation = Quat::from_rotation_x(0.3 * k as f32);

            commands.spawn((
                Transform::from_xyz(x, 10.0, z).with_rotation(rotation),
                RigidBody::Dynamic,
                collider.clone(),
                ContactSkin(0.1),
                ColliderDebugColor(color),
            ));
        }
    }
}

/// A regular grid centered at the origin, with `height(i, j)` giving the height of each vertex.
fn grid_mesh(
    nsubdivs: usize,
    scale: Vec3,
    height: impl Fn(usize, usize) -> f32,
) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let n = nsubdivs as f32;
    let vertices = (0..=nsubdivs)
        .flat_map(|i| (0..=nsubdivs).map(move |j| (i, j)))
        .map(|(i, j)| Vec3::new(i as f32 / n - 0.5, height(i, j), j as f32 / n - 0.5) * scale)
        .collect();
    let id = |i: usize, j: usize| (i * (nsubdivs + 1) + j) as u32;
    let indices = (0..nsubdivs)
        .flat_map(|i| (0..nsubdivs).map(move |j| (i, j)))
        .flat_map(|(i, j)| {
            [
                [id(i, j), id(i, j + 1), id(i + 1, j)],
                [id(i + 1, j), id(i, j + 1), id(i + 1, j + 1)],
            ]
        })
        .collect();
    (vertices, indices)
}

/// Quads connecting two closed loops of `n` vertices starting at `a` and `b`.
fn loop_strip(indices: &mut Vec<[u32; 3]>, a: u32, b: u32, n: u32) {
    for k in 0..n {
        let k1 = (k + 1) % n;
        indices.push([a + k, b + k, a + k1]);
        indices.push([a + k1, b + k, b + k1]);
    }
}

/// A torus lying on the XZ plane.
fn torus_mesh(
    major_radius: f32,
    minor_radius: f32,
    nu: u32,
    nv: u32,
) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut vertices = vec![];
    let mut indices = vec![];

    for iu in 0..nu {
        let (su, cu) = (iu as f32 / nu as f32 * TAU).sin_cos();
        for iv in 0..nv {
            let (sv, cv) = (iv as f32 / nv as f32 * TAU).sin_cos();
            let r = major_radius + minor_radius * cv;
            vertices.push(Vec3::new(r * cu, minor_radius * sv, r * su));
        }
    }

    for iu in 0..nu {
        loop_strip(&mut indices, iu * nv, ((iu + 1) % nu) * nv, nv);
    }

    (vertices, indices)
}

/// A vertical tube (a cylinder with a cylindrical hole) centered at the origin.
fn tube_mesh(
    outer_radius: f32,
    inner_radius: f32,
    height: f32,
    n: u32,
) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    // Four loops: outer bottom, outer top, inner top, inner bottom.
    let loops = [
        (outer_radius, -height / 2.0),
        (outer_radius, height / 2.0),
        (inner_radius, height / 2.0),
        (inner_radius, -height / 2.0),
    ];
    let vertices = loops
        .iter()
        .flat_map(|(radius, y)| {
            (0..n).map(move |k| {
                let (s, c) = (k as f32 / n as f32 * TAU).sin_cos();
                Vec3::new(radius * c, *y, radius * s)
            })
        })
        .collect();

    let mut indices = vec![];
    for l in 0..4 {
        loop_strip(&mut indices, l * n, ((l + 1) % 4) * n, n);
    }

    (vertices, indices)
}

/// A prism whose cross-section is a star with `num_branches` branches.
fn star_prism_mesh(
    outer_radius: f32,
    inner_radius: f32,
    height: f32,
    num_branches: u32,
) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let n = num_branches * 2;
    let mut vertices: Vec<Vec3> = [-height / 2.0, height / 2.0]
        .into_iter()
        .flat_map(|y| {
            (0..n).map(move |k| {
                let radius = if k % 2 == 0 {
                    outer_radius
                } else {
                    inner_radius
                };
                let (s, c) = (k as f32 / n as f32 * TAU).sin_cos();
                Vec3::new(radius * c, y, radius * s)
            })
        })
        .collect();
    let bottom_center = vertices.len() as u32;
    vertices.push(Vec3::new(0.0, -height / 2.0, 0.0));
    vertices.push(Vec3::new(0.0, height / 2.0, 0.0));
    let top_center = bottom_center + 1;

    let mut indices = vec![];
    loop_strip(&mut indices, 0, n, n);
    // The star is star-shaped with respect to its center, so its caps can be triangle fans.
    for k in 0..n {
        let k1 = (k + 1) % n;
        indices.push([bottom_center, k, k1]);
        indices.push([top_center, n + k1, n + k]);
    }

    (vertices, indices)
}
