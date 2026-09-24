use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

const N_CONTEXTS: usize = 5;
const WORLD_CHANGE_DELAY_SEC: f32 = 3.0;

#[derive(Component)]
/// Denotes which object(s) to change the world of
struct ChangeWorld;

#[derive(Resource, Debug)]
struct Contexts(Vec<Entity>);

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::linear_rgb(
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
        .add_systems(Update, change_world)
        .run();
}

fn change_world(
    mut query: Query<&mut RapierContextEntityLink, With<ChangeWorld>>,
    time: Res<Time>,
    worlds: Res<Contexts>,
) {
    for mut context_entity_link in query.iter_mut() {
        let idx = (worlds.0.len() - 1).min((time.elapsed_secs() / WORLD_CHANGE_DELAY_SEC) as usize);

        // Prevent needless change detection
        if context_entity_link.0 != worlds.0[idx] {
            context_entity_link.0 = worlds.0[idx];
            info!(
                "Changing context to {:?} ({}/{N_CONTEXTS})",
                context_entity_link.0,
                idx + 1
            )
        }
    }
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, -10.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(
    mut commands: Commands,
    q_default_context: Query<Entity, With<DefaultRapierContext>>,
) {
    let default_context = q_default_context
        .single()
        .expect("This will automatically be created");

    let mut contexts = Contexts(vec![default_context]);

    for i in 1..N_CONTEXTS {
        // Each context can have its own rapier configuration
        let mut config = RapierConfiguration::new(1.0);
        if i + 1 == N_CONTEXTS {
            info!("The last context will have opposite gravity");
            config.gravity = -config.gravity;
        }
        let context_ent = commands
            .spawn((RapierContextSimulation::default(), config))
            .id();
        contexts.0.push(context_ent);
    }

    for (i, &world_ent) in contexts.0.iter().enumerate() {
        let color = [
            Color::hsl(220.0, 1.0, 0.3),
            Color::hsl(180.0, 1.0, 0.3),
            Color::hsl(260.0, 1.0, 0.7),
        ][i % 3];

        /*
         * Ground
         */
        let ground_size = 5.1;
        let ground_height = 0.1;

        commands.spawn((
            Transform::from_xyz(0.0, (i as f32) * -0.5 - ground_height, 0.0),
            Collider::cuboid(ground_size, ground_height, ground_size),
            ColliderDebugColor(color.into()),
            RigidBody::Fixed,
            RapierContextEntityLink(world_ent),
        ));
    }

    /*
     * Create the cube
     *
     * The child is just there to show that physics world changes will also change the children.
     */
    commands
        .spawn((
            // This will spawn in the default world, since no RapierContextEntityLink was added
            Transform::from_xyz(0.0, 3.0, 0.0),
            RigidBody::Dynamic,
            ChangeWorld,
        ))
        .with_children(|p| {
            p.spawn((
                Transform::from_xyz(0.0, 0.0, 0.0),
                Collider::cuboid(0.5, 0.5, 0.5),
                ColliderDebugColor(Color::hsl(260.0, 1.0, 0.7).into()),
            ));
        });

    info!("Spawning cube in default world (1/{N_CONTEXTS})");

    commands.insert_resource(contexts);
}
