use bevy::{ecs::system::SystemParam, prelude::*};
use bevy_rapier3d::prelude::*;

#[derive(PartialEq, Eq, Clone, Copy, Component)]
enum CustomFilterTag {
    GroupA,
    GroupB,
}

// A custom filter that allows contacts only between rigid-bodies with the
// same user_data value.
// Note that using collision groups would be a more efficient way of doing
// this, but we use custom filters instead for demonstration purpose.
#[derive(SystemParam)]
struct SameUserDataFilter<'w, 's> {
    tags: Query<'w, 's, &'static CustomFilterTag>,
}

impl BevyPhysicsHooks for SameUserDataFilter<'_, '_> {
    fn filter_contact_pair(&self, context: PairFilterContextView) -> Option<SolverFlags> {
        if self.tags.get(context.collider1()).ok().copied()
            == self.tags.get(context.collider2()).ok().copied()
        {
            Some(SolverFlags::COMPUTE_RIGID_IMPULSES)
        } else {
            None
        }
    }
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<SameUserDataFilter>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-30.0, 30.0, 100.0).looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 10.0;

    commands.spawn((
        Transform::from_xyz(0.0, -10.0, 0.0),
        Collider::cuboid(ground_size, 1.2, ground_size),
        CustomFilterTag::GroupA,
    ));

    commands.spawn((
        Transform::from_xyz(0.0, 0.0, 0.0),
        Collider::cuboid(ground_size, 1.2, ground_size),
        CustomFilterTag::GroupB,
    ));

    /*
     * Create the cubes
     */
    let num = 4;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let mut group_id = 0;
    let tags = [CustomFilterTag::GroupA, CustomFilterTag::GroupB];
    let colors = [Hsla::hsl(220.0, 1.0, 0.3), Hsla::hsl(260.0, 1.0, 0.7)];

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = (i as f32 + j as f32 * 0.2) * shift - centerx;
            let y = j as f32 * shift + centery + 2.0;
            group_id += 1;

            commands.spawn((
                Transform::from_xyz(x, y, 0.0),
                RigidBody::Dynamic,
                Collider::cuboid(rad, rad, rad),
                ActiveHooks::FILTER_CONTACT_PAIRS,
                tags[group_id % 2],
                ColliderDebugColor(colors[group_id % 2]),
            ));
        }
    }
}
