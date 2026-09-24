use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

#[derive(Component, Default)]
pub struct Despawn;

#[derive(Resource, Default)]
pub struct DespawnResource {
    timer: Timer,
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(DespawnResource::default())
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, despawn)
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((Camera2d, Transform::from_xyz(0.0, -200.0, 0.0)));
}

pub fn setup_physics(mut commands: Commands, mut despawn: ResMut<DespawnResource>) {
    despawn.timer = Timer::from_seconds(4.0, TimerMode::Once);

    // Build the rigid body.
    let rad = 4.0;
    let numi = 40; // Num vertical nodes.
    let numk = 40; // Num horizontal nodes.
    let shift = 10.0;

    let mut body_entities = Vec::new();

    for k in 0..numk {
        for i in 0..numi {
            let fk = k as f32;
            let fi = i as f32;

            let rigid_body = if i == 0 && (k % 4 == 0 || k == numk - 1) {
                RigidBody::Fixed
            } else {
                RigidBody::Dynamic
            };

            let child_entity = commands
                .spawn((
                    Transform::from_xyz(fk * shift, -fi * shift, 0.0),
                    rigid_body,
                    Collider::cuboid(rad, rad),
                ))
                .id();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = RevoluteJointBuilder::new().local_anchor2(Vec2::new(0.0, shift));
                commands.entity(child_entity).with_children(|cmd| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    let mut entity = cmd.spawn(ImpulseJoint::new(parent_entity, joint));
                    if i == (numi / 2) || (k % 4 == 0 || k == numk - 1) {
                        entity.insert(Despawn);
                    }
                });
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - numi;
                let parent_entity = body_entities[parent_index];
                let joint = RevoluteJointBuilder::new().local_anchor2(Vec2::new(-shift, 0.0));
                commands.entity(child_entity).with_children(|cmd| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    let mut entity = cmd.spawn(ImpulseJoint::new(parent_entity, joint));
                    if i == (numi / 2) || (k % 4 == 0 || k == numk - 1) {
                        entity.insert(Despawn);
                    }
                });
            }

            body_entities.push(child_entity);
        }
    }
}

pub fn despawn(
    mut commands: Commands,
    time: Res<Time>,
    mut despawn: ResMut<DespawnResource>,
    query: Query<Entity, With<Despawn>>,
) {
    if despawn.timer.tick(time.delta()).just_finished() {
        for e in &query {
            println!("Despawning joint entity");
            commands.entity(e).despawn();
        }
    }
}
