use bevy::prelude::*;
use bevy_rapier3d::dynamics::RigidBody;
use bevy_rapier3d::geometry::Collider;
use bevy_rapier3d::math::Vect;

pub fn create_pyramid(commands: &mut Commands, offset: Vect, stack_height: usize, rad: f32) {
    let shift = rad * 2.0;

    for i in 0usize..stack_height {
        for j in i..stack_height {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * shift;
            let y = fi * shift;

            // Build the rigid body.
            commands.spawn((
                RigidBody::Dynamic,
                Transform::from_translation(Vec3::new(x, y, 0.0) + offset),
                Collider::cuboid(1.0, 1.0, 1.0),
            ));
        }
    }
}

pub fn setup_pyramids(app: &mut App, pyramid_count: usize, stack_height: usize) {
    app.add_systems(Startup, move |mut commands: Commands| {
        let rad = 0.5;
        let spacing = 4.0;

        /*
         * Ground
         */
        let ground_size = 50.0;
        let ground_height = 0.1;

        commands.spawn((
            RigidBody::Fixed,
            Transform::from_translation(Vect::new(0.0, -ground_height, 0.0)),
            Collider::cuboid(
                ground_size,
                ground_height,
                pyramid_count as f32 * spacing / 2.0 + ground_size,
            ),
        ));

        /*
         * Create the pyramids
         */
        for pyramid_index in 0..pyramid_count {
            let bottomy = rad;
            create_pyramid(
                &mut commands,
                Vect::new(
                    0.0,
                    bottomy,
                    (pyramid_index as f32 - pyramid_count as f32 / 2.0) * spacing,
                ),
                stack_height,
                rad,
            );
        }
    });
}
