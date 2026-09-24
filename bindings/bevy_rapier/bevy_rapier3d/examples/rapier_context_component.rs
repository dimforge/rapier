use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_physics, setup_graphics))
        .add_systems(
            PostUpdate,
            display_nb_colliders.after(PhysicsSet::SyncBackend),
        )
        .run();
}

/// Demonstrates how to access a more specific component of [`RapierContext`]
fn display_nb_colliders(
    query_context: Query<&RapierContextColliders, With<DefaultRapierContext>>,
    mut exit: MessageWriter<AppExit>,
) -> Result<()> {
    let nb_colliders = query_context.single()?.colliders.len();
    println!("There are {nb_colliders} colliders.");
    assert!(nb_colliders > 0);
    exit.write(AppExit::Success);

    Ok(())
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 5.1;
    let ground_height = 0.1;

    let starting_y = -0.5 - ground_height;

    commands.spawn((
        Transform::from_xyz(0.0, starting_y, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));

    for _ in 0..3 {
        /*
         * Create the cubes
         */

        commands.spawn((
            Transform::default(),
            RigidBody::Dynamic,
            Collider::cuboid(0.5, 0.5, 0.5),
        ));
    }
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, -10.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
    ));
}
