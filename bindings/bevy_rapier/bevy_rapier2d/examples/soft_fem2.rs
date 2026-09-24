//! The FEM soft-body solver next to the constraint solver: the same bodies twice, simulated by
//! `SoftBodySolver::Fem` on the lower row (blue) and by the constraint solver on the upper one
//! (orange). The FEM cantilever and plank hold their static deflection; the others sag.
//!
//! Run with `--features fem`.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

/// Vertical distance between the FEM row and the constraint-solver row.
const ROW: f32 = 6.0;

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
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 1.0 / 45.0,
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(1.0, 5.0, 0.0),
    ));
    commands.spawn((
        Text::new("Lower row: FEM solver. Upper row: constraint solver."),
        TextColor(Color::BLACK),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));
}

/// A stiff, damped, corotational material.
fn stiff(young_modulus: f32) -> SoftBodyMaterial {
    SoftBodyMaterial(RapierSoftBodyMaterial {
        young_modulus,
        poisson_ratio: 0.3,
        elastic_damping_ratio: 1.0,
        ..default()
    })
}

/// The indices of the particles of `body` whose local position satisfies `pinned`.
fn pinned_particles(body: &SoftBody, pinned: impl Fn(Vec2) -> bool) -> Vec<u32> {
    (0..)
        .zip(body.builder.particle_positions())
        .filter(|(_, p)| pinned(**p))
        .map(|(i, _)| i)
        .collect()
}

fn setup_physics(mut commands: Commands, mut materials: ResMut<Assets<ColorMaterial>>) {
    commands.spawn((
        Transform::from_xyz(0.0, -0.5, 0.0),
        Collider::cuboid(30.0, 0.5),
    ));

    let solvers = [
        (SoftBodySolver::Fem, Color::srgb(0.3, 0.55, 0.9)),
        (SoftBodySolver::Constraints, Color::srgb(0.95, 0.6, 0.3)),
    ];

    for (row, (solver, color)) in solvers.into_iter().enumerate() {
        let y = 2.0 + row as f32 * ROW;
        let color = materials.add(color);

        /*
         * A stiff cantilever bolted to a wall.
         */
        let (length, thickness) = (4.0, 0.4);
        let beam = SoftBody::grid(Vec2::new(length * 0.5, thickness * 0.5), 17, 3)
            .map(|b| b.cell_model(SoftBodyCellModel::Corotational).mass(8.0));
        let pinned = pinned_particles(&beam, |p| p.x < -length * 0.5 + 1.0e-4);
        commands.spawn((
            Transform::from_xyz(length * 0.5 - 8.0, y, 0.0),
            beam.map(|b| b.pinned_particles(pinned)),
            stiff(2.0e6),
            SoftBodyElasticitySolver(solver),
            SoftBodyMeshSync::default(),
            MeshMaterial2d(color.clone()),
        ));

        /*
         * A plank pinned at both ends, loaded by a heavy box dropped in its middle.
         */
        let plank = SoftBody::grid(Vec2::new(3.0, 0.25), 21, 3)
            .map(|b| b.cell_model(SoftBodyCellModel::Corotational).mass(20.0));
        let pinned = pinned_particles(&plank, |p| p.x.abs() > 2.9);
        commands.spawn((
            Transform::from_xyz(2.0, y, 0.0),
            plank.map(|b| b.pinned_particles(pinned)),
            stiff(4.0e6),
            SoftBodyElasticitySolver(solver),
            SoftBodyMeshSync::default(),
            MeshMaterial2d(color.clone()),
        ));
        commands.spawn((
            Transform::from_xyz(2.0, y + 2.5, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(0.5, 0.5),
            ColliderMassProperties::Density(20.0),
        ));

        /*
         * A Neo-Hookean jelly square dropped next to the plank.
         */
        commands.spawn((
            Transform::from_xyz(9.0, y + 1.0, 0.0),
            SoftBody::grid(Vec2::splat(0.8), 5, 5).map(|b| {
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
            MeshMaterial2d(color),
        ));
    }
}
