use bevy::{input::common_conditions::input_just_pressed, prelude::*};
use bevy_rapier3d::prelude::*;

const N_CONTEXTS: usize = 2;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default()
                .with_custom_initialization(RapierContextInitialization::NoAutomaticRapierContext),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(
            Startup,
            ((create_contexts, setup_physics).chain(), setup_graphics),
        )
        .add_systems(Update, move_platforms)
        .add_systems(
            Update,
            change_context.run_if(input_just_pressed(KeyCode::KeyC)),
        )
        .run();
}

fn create_contexts(mut commands: Commands) {
    for i in 0..N_CONTEXTS {
        let mut context = commands.spawn((RapierContextSimulation::default(), ContextId(i)));
        if i == 0 {
            context.insert((DefaultRapierContext, RapierContextSimulation::default()));
        }
    }
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, -10.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
    ));
}

#[derive(Component)]
pub struct ContextId(pub usize);

#[derive(Component)]
struct Platform {
    starting_y: f32,
}

fn move_platforms(time: Res<Time>, mut query: Query<(&mut Transform, &Platform)>) {
    for (mut transform, platform) in query.iter_mut() {
        transform.translation.y = platform.starting_y + -time.elapsed_secs().sin();
    }
}

/// Demonstrates how easy it is to move one entity to another context.
fn change_context(
    query_context: Query<Entity, With<DefaultRapierContext>>,
    mut query_links: Query<(Entity, &mut RapierContextEntityLink)>,
) -> Result<()> {
    let default_context = query_context.single()?;
    for (e, mut link) in query_links.iter_mut() {
        if link.0 == default_context {
            continue;
        }
        link.0 = default_context;
        println!("changing context of {} for context {}", e, link.0);
    }

    Ok(())
}

pub fn setup_physics(
    context: Query<(Entity, &ContextId), With<RapierContextSimulation>>,
    mut commands: Commands,
) {
    for (context_entity, id) in context.iter() {
        let id = id.0;

        let color = [
            Hsla::hsl(220.0, 1.0, 0.3),
            Hsla::hsl(180.0, 1.0, 0.3),
            Hsla::hsl(260.0, 1.0, 0.7),
        ][id % 3];

        /*
         * Ground
         */
        let ground_size = 5.1;
        let ground_height = 0.1;

        let starting_y = (id as f32) * -0.5 - ground_height;

        let mut platforms = commands.spawn((
            Transform::from_xyz(0.0, starting_y, 0.0),
            Collider::cuboid(ground_size, ground_height, ground_size),
            ColliderDebugColor(color),
            RapierContextEntityLink(context_entity),
        ));
        if id == 1 {
            platforms.insert(Platform { starting_y });
        }

        /*
         * Create the cube
         */

        commands.spawn((
            Transform::from_xyz(0.0, 1.0 + id as f32 * 5.0, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(0.5, 0.5, 0.5),
            ColliderDebugColor(color),
            RapierContextEntityLink(context_entity),
        ));
    }
}
