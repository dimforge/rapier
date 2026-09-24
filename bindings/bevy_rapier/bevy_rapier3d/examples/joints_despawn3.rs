use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

#[derive(Component)]
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
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, despawn)
        .run();
}

pub fn setup_graphics(mut commands: Commands, mut res: ResMut<DespawnResource>) {
    res.timer = Timer::from_seconds(5.0, TimerMode::Once);

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(15.0, 5.0, 42.0).looking_at(Vec3::new(13.0, 1.0, 1.0), Vec3::Y),
    ));
}

fn create_prismatic_joints(commands: &mut Commands, origin: Vect, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let mut curr_parent = commands
        .spawn((
            Transform::from_xyz(origin.x, origin.y, origin.z),
            RigidBody::Fixed,
            Collider::cuboid(rad, rad, rad),
        ))
        .id();

    for i in 0..num {
        let dz = (i + 1) as f32 * shift;

        let axis = if i.is_multiple_of(2) {
            Vec3::new(1.0, 1.0, 0.0)
        } else {
            Vec3::new(-1.0, 1.0, 0.0)
        };

        let prism = PrismaticJointBuilder::new(axis)
            .local_anchor2(Vec3::new(0.0, 0.0, -shift))
            .limits([-2.0, 2.0]);
        let joint = ImpulseJoint::new(curr_parent, prism);

        let mut entity = commands.spawn((
            Transform::from_xyz(origin.x, origin.y, origin.z + dz),
            RigidBody::Dynamic,
            Collider::cuboid(rad, rad, rad),
            joint,
        ));

        curr_parent = entity.id();

        if i == 2 {
            entity.insert(Despawn);
        }
    }
}

fn create_revolute_joints(commands: &mut Commands, origin: Vec3, num: usize) {
    let rad = 0.4;
    let shift = 2.0;

    let mut curr_parent = commands
        .spawn((
            Transform::from_xyz(origin.x, origin.y, 0.0),
            RigidBody::Fixed,
            Collider::cuboid(rad, rad, rad),
        ))
        .id();

    for i in 0..num {
        // Create four bodies.
        let z = origin.z + i as f32 * shift * 2.0 + shift;
        let positions = [
            Vec3::new(origin.x, origin.y, z),
            Vec3::new(origin.x + shift, origin.y, z),
            Vec3::new(origin.x + shift, origin.y, z + shift),
            Vec3::new(origin.x, origin.y, z + shift),
        ];

        let mut handles = [curr_parent; 4];
        for k in 0..4 {
            handles[k] = commands
                .spawn((
                    Transform::from_translation(positions[k]),
                    RigidBody::Dynamic,
                    Collider::cuboid(rad, rad, rad),
                ))
                .id();
        }

        // Setup four joints.
        let x = Vec3::X;
        let z = Vec3::Z;

        let revs = [
            RevoluteJointBuilder::new(z).local_anchor2(Vec3::new(0.0, 0.0, -shift)),
            RevoluteJointBuilder::new(x).local_anchor2(Vec3::new(-shift, 0.0, 0.0)),
            RevoluteJointBuilder::new(z).local_anchor2(Vec3::new(0.0, 0.0, -shift)),
            RevoluteJointBuilder::new(x).local_anchor2(Vec3::new(shift, 0.0, 0.0)),
        ];

        commands
            .entity(handles[0])
            .insert(ImpulseJoint::new(curr_parent, revs[0]));
        commands
            .entity(handles[1])
            .insert(ImpulseJoint::new(handles[0], revs[1]));
        commands
            .entity(handles[2])
            .insert(ImpulseJoint::new(handles[1], revs[2]));
        commands
            .entity(handles[3])
            .insert(ImpulseJoint::new(handles[2], revs[3]));

        curr_parent = handles[3];

        if i == 1 {
            for e in handles {
                commands.entity(e).insert(Despawn);
            }
        }
    }
}

fn create_fixed_joints(commands: &mut Commands, origin: Vec3, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_entities = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            // NOTE: the num - 2 test is to avoid two consecutive
            // fixed bodies. Because physx will crash if we add
            // a joint between these.
            let rigid_body = if i == 0 && (k.is_multiple_of(4) && k != num - 2 || k == num - 1) {
                RigidBody::Fixed
            } else {
                RigidBody::Dynamic
            };

            let child_entity = commands
                .spawn((
                    Transform::from_xyz(origin.x + fk * shift, origin.y, origin.z + fi * shift),
                    rigid_body,
                    Collider::ball(rad),
                ))
                .id();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = FixedJointBuilder::new().local_anchor2(Vec3::new(0.0, 0.0, -shift));
                commands.entity(child_entity).with_children(|children| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    children.spawn(ImpulseJoint::new(parent_entity, joint));
                });
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = FixedJointBuilder::new().local_anchor2(Vec3::new(-shift, 0.0, 0.0));
                commands.entity(child_entity).with_children(|children| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    children.spawn((ImpulseJoint::new(parent_entity, joint), Despawn));
                });
            }

            body_entities.push(child_entity);
        }
    }
}

fn create_ball_joints(commands: &mut Commands, num: usize) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_entities = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            let rigid_body = if i == 0 && (k.is_multiple_of(4) || k == num - 1) {
                RigidBody::Fixed
            } else {
                RigidBody::Dynamic
            };

            let child_entity = commands
                .spawn((
                    Transform::from_xyz(fk * shift, 0.0, fi * shift),
                    rigid_body,
                    Collider::ball(rad),
                ))
                .id();

            // Vertical joint.
            if i > 0 {
                let parent_entity = *body_entities.last().unwrap();
                let joint = SphericalJointBuilder::new().local_anchor2(Vec3::new(0.0, 0.0, -shift));
                commands.entity(child_entity).with_children(|children| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    let mut entity = children.spawn(ImpulseJoint::new(parent_entity, joint));
                    if i == 2 {
                        entity.insert(Despawn);
                    }
                });
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_entities.len() - num;
                let parent_entity = body_entities[parent_index];
                let joint = SphericalJointBuilder::new().local_anchor2(Vec3::new(-shift, 0.0, 0.0));
                commands.entity(child_entity).with_children(|children| {
                    // NOTE: we want to attach multiple impulse joints to this entity, so
                    //       we need to add the components to children of the entity. Otherwise
                    //       the second joint component would just overwrite the first one.
                    children.spawn(ImpulseJoint::new(parent_entity, joint));
                });
            }

            body_entities.push(child_entity);
        }
    }
}

pub fn setup_physics(mut commands: Commands) {
    create_prismatic_joints(&mut commands, Vec3::new(20.0, 10.0, 0.0), 5);
    create_revolute_joints(&mut commands, Vec3::new(20.0, 0.0, 0.0), 3);
    create_fixed_joints(&mut commands, Vec3::new(0.0, 10.0, 0.0), 5);
    create_ball_joints(&mut commands, 15);
}

pub fn despawn(
    mut commands: Commands,
    time: Res<Time>,
    mut despawn: ResMut<DespawnResource>,
    query: Query<Entity, With<Despawn>>,
) {
    if despawn.timer.tick(time.delta()).is_finished() {
        for entity in &query {
            println!("Despawning joint entity");
            commands.entity(entity).despawn();
        }
    }
}
