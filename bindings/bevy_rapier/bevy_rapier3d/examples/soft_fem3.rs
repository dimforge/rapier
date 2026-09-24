//! The FEM soft-body solver side by side with the constraint solver: the same bodies twice,
//! simulated by `SoftBodySolver::Fem` at the back (blue) and by the constraint solver at the front
//! (orange). The FEM cantilever and plank hold their deflection; the constraint ones sag.
//!
//! Run with `--features fem`.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

/// Distance between the FEM column (at `-Z`) and the constraint-solver column (at `+Z`).
const COLUMN: f32 = 3.5;

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
        Transform::from_xyz(2.0, 8.0, 16.0).looking_at(Vec3::new(0.5, 2.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// A stiff, damped material.
fn stiff(young_modulus: f32) -> SoftBodyMaterial {
    SoftBodyMaterial(RapierSoftBodyMaterial {
        young_modulus,
        poisson_ratio: 0.3,
        elastic_damping_ratio: 1.0,
        ..default()
    })
}

/// The indices of the particles of `body` whose local position satisfies `pinned`.
fn pinned_particles(body: &SoftBody, pinned: impl Fn(Vec3) -> bool) -> Vec<u32> {
    (0..)
        .zip(body.builder.particle_positions())
        .filter(|(_, p)| pinned(**p))
        .map(|(i, _)| i)
        .collect()
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(20.0, 0.1, 20.0),
    ));

    let box_mesh = meshes.add(Cuboid::new(0.7, 0.7, 0.7));
    let box_color = materials.add(Color::srgb(0.4, 0.4, 0.45));
    let solvers = [
        (SoftBodySolver::Fem, -COLUMN, Color::srgb(0.3, 0.55, 0.9)),
        (
            SoftBodySolver::Constraints,
            COLUMN,
            Color::srgb(0.95, 0.6, 0.3),
        ),
    ];

    for (solver, z, color) in solvers {
        let color = materials.add(color);

        /*
         * A stiff cantilever bolted to a wall: the deflection under its own weight.
         */
        let (length, thickness) = (3.0, 0.3);
        let beam = SoftBody::cuboid(
            Vec3::new(length * 0.5, thickness * 0.5, thickness * 0.5),
            13,
            3,
            3,
        )
        .map(|b| b.cell_model(SoftBodyCellModel::Corotational).mass(12.0));
        let pinned = pinned_particles(&beam, |p| p.x < -length * 0.5 + 1.0e-4);
        commands.spawn((
            Transform::from_xyz(length * 0.5 - 5.0, 3.0, z),
            beam.map(|b| b.pinned_particles(pinned)),
            stiff(2.0e6),
            SoftBodyElasticitySolver(solver),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(color.clone()),
        ));

        /*
         * A plank pinned at both ends, loaded by a heavy box dropped in its middle.
         */
        let plank = SoftBody::cuboid(Vec3::new(2.0, 0.15, 0.6), 17, 3, 5)
            .map(|b| b.cell_model(SoftBodyCellModel::Corotational).mass(20.0));
        let pinned = pinned_particles(&plank, |p| p.x.abs() > 1.9);
        commands.spawn((
            Transform::from_xyz(1.0, 2.0, z),
            plank.map(|b| b.pinned_particles(pinned)),
            stiff(4.0e6),
            SoftBodyElasticitySolver(solver),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(color.clone()),
        ));
        commands.spawn((
            Transform::from_xyz(1.0, 4.5, z),
            RigidBody::Dynamic,
            Collider::cuboid(0.35, 0.35, 0.35),
            ColliderMassProperties::Density(30.0),
            Mesh3d(box_mesh.clone()),
            MeshMaterial3d(box_color.clone()),
        ));

        /*
         * A Neo-Hookean jelly cube dropped on the ground: the two solvers agree here.
         */
        commands.spawn((
            Transform::from_xyz(6.0, 2.0, z),
            SoftBody::cuboid(Vec3::splat(0.7), 5, 5, 5).map(|b| {
                b.cell_model(SoftBodyCellModel::NeoHookean)
                    .particle_mass(0.1)
            }),
            SoftBodyMaterial(RapierSoftBodyMaterial {
                young_modulus: 2.0e4,
                poisson_ratio: 0.4,
                elastic_damping_ratio: 0.5,
                ..default()
            }),
            SoftBodyElasticitySolver(solver),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(color),
        ));
    }
}
