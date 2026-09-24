//! Scene queries drawn with gizmos: `cast_shape_nonlinear` (a tumbling stick),
//! `closest_points_to_shape` (an orbiting ball), and `contact_with_shape` (a cube moving between
//! the legs of a table), with the part of the compound table hit by each query highlighted from
//! its sub-shape id.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

const HIT_COLOR: Color = Color::srgb(0.9, 0.1, 0.1);
const MISS_COLOR: Color = Color::srgb(0.2, 0.6, 0.2);
const PATH_COLOR: Color = Color::srgb(0.6, 0.6, 0.6);
const PART_COLOR: Color = Color::srgb(1.0, 0.7, 0.0);

#[derive(Component)]
struct Ground;

#[derive(Component)]
struct ResultsText;

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
        .add_systems(Update, run_queries)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(8.0, 9.0, 14.0).looking_at(Vec3::new(0.0, 1.5, 0.0), Vec3::Y),
    ));
    commands.spawn((
        Text::new(""),
        TextColor(Color::BLACK),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        ResultsText,
    ));
}

fn setup_physics(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(20.0, 0.1, 20.0),
        Ground,
    ));

    // A table made of a single compound collider: part 0 is the top, parts 1 to 4 are the legs.
    let mut parts = vec![(
        Vec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
        Collider::cuboid(3.0, 0.2, 2.0),
    )];
    for (x, z) in [(-2.6, -1.6), (2.6, -1.6), (-2.6, 1.6), (2.6, 1.6)] {
        parts.push((
            Vec3::new(x, 0.9, z),
            Quat::IDENTITY,
            Collider::cuboid(0.2, 0.9, 0.2),
        ));
    }
    commands.spawn((Transform::default(), Collider::compound(parts)));

    // A few more obstacles.
    commands.spawn((Transform::from_xyz(-6.0, 1.0, 0.0), Collider::ball(1.0)));
    commands.spawn((
        Transform::from_xyz(6.0, 1.0, 0.0).with_rotation(Quat::from_rotation_y(0.5)),
        Collider::cuboid(1.0, 1.0, 1.0),
    ));
}

fn run_queries(
    time: Res<Time>,
    rapier_context: ReadRapierContext,
    ground: Single<Entity, With<Ground>>,
    colliders: Query<(&Collider, &GlobalTransform)>,
    mut text: Single<&mut Text, With<ResultsText>>,
    mut gizmos: Gizmos,
) -> Result<()> {
    let context = rapier_context.single()?;
    let t = time.elapsed_secs();
    let filter = QueryFilter::new().exclude_collider(*ground);
    let mut report = String::new();

    /*
     * Nonlinear shape-cast: a stick tumbling toward the table.
     */
    let stick = Collider::cuboid(0.8, 0.1, 0.1);
    let motion = NonlinearMotion {
        translation: Vec3::new(4.0 * (t * 0.4).sin(), 5.0, 8.0),
        rotation: Quat::IDENTITY,
        local_center: Vec3::ZERO,
        linear_velocity: Vec3::new(0.0, -3.0, -10.0),
        angular_velocity: Vec3::new(0.0, 2.0, 6.0),
    };
    let hit = context.cast_shape_nonlinear(&motion, &stick, 0.0, 1.0, true, filter);
    let end_time = hit.map_or(1.0, |(_, hit)| hit.time_of_impact);

    let raw_motion = motion.into_rapier();
    for k in 0..=10 {
        let pose = raw_motion.position_at_time(end_time * k as f32 / 10.0);
        let color = if k == 10 && hit.is_some() {
            HIT_COLOR
        } else {
            PATH_COLOR
        };
        draw_cuboid(
            &mut gizmos,
            pose.translation,
            pose.rotation,
            Vec3::new(0.8, 0.1, 0.1),
            color,
        );
    }

    match hit {
        Some((entity, hit)) => {
            if let Some(details) = hit.details {
                gizmos.sphere(
                    Isometry3d::from_translation(details.witness1),
                    0.08,
                    HIT_COLOR,
                );
            }
            highlight_part(&mut gizmos, &colliders, entity, hit.subshape1);
            report += &format!(
                "cast_shape_nonlinear: hit {entity} (part {}) at t = {:.2}\n",
                hit.subshape1, hit.time_of_impact
            );
        }
        None => report += "cast_shape_nonlinear: no hit\n",
    }

    /*
     * Closest points: a ball orbiting around the table.
     */
    let ball_radius = 0.4;
    let ball = Collider::ball(ball_radius);
    let ball_pos = Vec3::new(
        4.5 * (t * 0.5).cos(),
        1.5 + 1.2 * (t * 1.3).sin(),
        3.2 * (t * 0.5).sin(),
    );
    let max_dist = 2.0;

    match context.closest_points_to_shape(ball_pos, Quat::IDENTITY, &ball, max_dist, filter) {
        Some((entity, ShapeClosestPoints::WithinMargin(p1, p2))) => {
            gizmos.sphere(
                Isometry3d::from_translation(ball_pos),
                ball_radius,
                MISS_COLOR,
            );
            gizmos.line(p1, p2, MISS_COLOR);
            report += &format!(
                "closest_points_to_shape: {entity} is {:.2} away\n",
                p1.distance(p2)
            );
        }
        Some((entity, _)) => {
            gizmos.sphere(
                Isometry3d::from_translation(ball_pos),
                ball_radius,
                HIT_COLOR,
            );
            report += &format!("closest_points_to_shape: intersecting {entity}\n");
        }
        None => {
            gizmos.sphere(
                Isometry3d::from_translation(ball_pos),
                ball_radius,
                PATH_COLOR,
            );
            report += &format!("closest_points_to_shape: nothing within {max_dist}\n");
        }
    }

    /*
     * Contact: a cube moving under the table top, between its legs.
     */
    let cube_half_extents = Vec3::splat(0.4);
    let cube = Collider::cuboid(
        cube_half_extents.x,
        cube_half_extents.y,
        cube_half_extents.z,
    );
    let cube_pos = Vec3::new(2.8 * (t * 0.6).sin(), 1.0, 1.8 * (t * 0.6).cos());
    let cube_rot = Quat::from_rotation_y(t);
    let prediction = 0.5;

    match context.contact_with_shape(cube_pos, cube_rot, &cube, prediction, filter) {
        Some((entity, contact)) => {
            let color = if contact.distance < 0.0 {
                HIT_COLOR
            } else {
                MISS_COLOR
            };
            draw_cuboid(&mut gizmos, cube_pos, cube_rot, cube_half_extents, color);
            gizmos.sphere(Isometry3d::from_translation(contact.point1), 0.05, color);
            gizmos.sphere(Isometry3d::from_translation(contact.point2), 0.05, color);
            gizmos.arrow(contact.point1, contact.point1 + contact.normal1, color);
            highlight_part(&mut gizmos, &colliders, entity, contact.subshape1);
            report += &format!(
                "contact_with_shape: {entity} (part {}) at distance {:.2}\n",
                contact.subshape1, contact.distance
            );
        }
        None => {
            draw_cuboid(
                &mut gizmos,
                cube_pos,
                cube_rot,
                cube_half_extents,
                PATH_COLOR,
            );
            report += &format!("contact_with_shape: nothing within {prediction}\n");
        }
    }

    text.0 = report;
    Ok(())
}

fn draw_cuboid(gizmos: &mut Gizmos, pos: Vec3, rot: Quat, half_extents: Vec3, color: Color) {
    gizmos.cube(
        Transform::from_translation(pos)
            .with_rotation(rot)
            .with_scale(half_extents * 2.0),
        color,
    );
}

/// Draws the part `subshape` of the collider attached to `entity`, if it is a compound of cuboids.
fn highlight_part(
    gizmos: &mut Gizmos,
    colliders: &Query<(&Collider, &GlobalTransform)>,
    entity: Entity,
    subshape: u32,
) {
    let Ok((collider, transform)) = colliders.get(entity) else {
        return;
    };
    let Some(compound) = collider.as_compound() else {
        return;
    };
    let Some((pos, rot, ColliderView::Cuboid(cuboid))) = compound.shapes().nth(subshape as usize)
    else {
        return;
    };
    let (_, body_rot, body_pos) = transform.to_scale_rotation_translation();
    draw_cuboid(
        gizmos,
        body_pos + body_rot * pos,
        body_rot * rot,
        cuboid.half_extents() + Vec3::splat(0.02),
        PART_COLOR,
    );
}
