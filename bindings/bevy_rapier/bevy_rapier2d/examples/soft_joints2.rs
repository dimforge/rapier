//! Impulse joints attached to soft bodies, through the soft body entities (for the whole body) or
//! `SoftBodyCluster` entities: every 2D joint type on jellies, a motorized hinge between two
//! clusters of one soft bar, a jelly hanging from a two-link arm and tied to a crate, and a
//! kinematic cluster waving a rope.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

/// Marks the joint of the jelly shuttled along its rail.
#[derive(Component)]
struct RailJoint;

/// Marks the hinge between the two halves of the soft bar.
#[derive(Component)]
struct FlapJoint;

/// The cluster of the rope's top particles, waved kinematically from its initial position.
#[derive(Component)]
struct Grip {
    /// The initial position of the cluster's center.
    home: Vec2,
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
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, (animate_motors, wave_grip))
        .run();
}

fn setup_graphics(mut commands: Commands, mut debug_render: ResMut<DebugRenderContext>) {
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 1.0 / 32.0,
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(2.0, 3.0, 0.0),
    ));
    // Draw the elements of the soft bodies (e.g. the rope).
    debug_render.mode.soft_bodies = true;
}

/// The components of a jelly square of 5x5 particles centered at `center`.
fn jelly(
    center: Vec2,
    half_extents: Vec2,
    young_modulus: f32,
    color: &Handle<ColorMaterial>,
) -> impl Bundle {
    (
        Transform::from_translation(center.extend(0.0)),
        SoftBody::grid(half_extents, 5, 5).map(|b| {
            b.cell_model(SoftBodyCellModel::Corotational)
                .particle_mass(0.08)
                .particle_radius(0.06)
        }),
        SoftBodyMaterial(RapierSoftBodyMaterial {
            young_modulus,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 0.8,
            ..default()
        }),
        Friction::coefficient(0.6),
        SoftBodyMeshSync::default(),
        MeshMaterial2d(color.clone()),
    )
}

fn setup_physics(mut commands: Commands, mut materials: ResMut<Assets<ColorMaterial>>) {
    let color = materials.add(Color::srgb(0.55, 0.75, 0.95));

    /*
     * Ground.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.5, 0.0),
        Collider::cuboid(30.0, 0.5),
    ));

    /*
     * Revolute + velocity motor: a jelly spun about its center.
     */
    let center = Vec2::new(-12.0, 2.0);
    let pivot = commands
        .spawn((
            Transform::from_translation(center.extend(0.0)),
            RigidBody::Fixed,
        ))
        .id();
    // A joint on a soft body entity attaches the whole soft body.
    commands.spawn((
        jelly(center, Vec2::splat(0.6), 8.0e3, &color),
        ImpulseJoint::new(pivot, RevoluteJointBuilder::new().motor_velocity(1.5, 60.0)),
    ));

    /*
     * Fixed: a rigid plate welded onto a jelly's top-edge cluster; the full-rank edge cluster
     * holds the plate's orientation as the jelly wobbles.
     */
    let center = Vec2::new(-8.0, 0.61);
    let wobbler = jelly(center, Vec2::splat(0.6), 2.5e3, &color);
    let top_edge: Vec<u32> = (0..5).map(|i| i * 5 + 4).collect();
    let wobbler = commands.spawn(wobbler).id();
    let top_pos = center + Vec2::new(0.0, 0.6);
    let plate = commands
        .spawn((
            Transform::from_translation((top_pos + Vec2::new(0.0, 0.12)).extend(0.0)),
            RigidBody::Dynamic,
            Collider::cuboid(0.7, 0.06),
            ColliderMassProperties::Density(0.4),
        ))
        .id();
    commands.spawn((
        SoftBodyCluster::new(wobbler, top_edge),
        ImpulseJoint::new(
            plate,
            FixedJointBuilder::new().local_anchor1(Vec2::new(0.0, -0.12)),
        ),
    ));
    // A small crate dropped on the plate to make it wobble.
    commands.spawn((
        Transform::from_translation((top_pos + Vec2::new(0.3, 1.4)).extend(0.0)),
        RigidBody::Dynamic,
        Collider::cuboid(0.15, 0.15),
        ColliderMassProperties::Density(1.5),
    ));

    /*
     * Prismatic + limits + position motor: a jelly shuttled along a rail between two stops.
     */
    let center = Vec2::new(-3.0, 0.85);
    let rail = commands
        .spawn((
            Transform::from_translation(center.extend(0.0)),
            RigidBody::Fixed,
        ))
        .id();
    commands.spawn((
        RailJoint,
        jelly(center, Vec2::splat(0.4), 6.0e3, &color),
        ImpulseJoint::new(
            rail,
            PrismaticJointBuilder::new(Vec2::X)
                .limits([-1.8, 1.8])
                .motor_position(0.0, 40.0, 8.0),
        ),
    ));

    /*
     * Rope: two jellies chained over a ledge, one dragging the other.
     */
    commands.spawn((
        Transform::from_xyz(1.5, 1.0, 0.0),
        Collider::cuboid(1.2, 1.0),
    ));
    let anchor_jelly = commands
        .spawn(jelly(Vec2::new(1.5, 2.6), Vec2::splat(0.5), 1.2e4, &color))
        .id();
    commands.spawn((
        jelly(Vec2::new(3.8, 2.6), Vec2::splat(0.5), 1.2e4, &color),
        ImpulseJoint::new(anchor_jelly, RopeJointBuilder::new(2.2)),
    ));

    /*
     * Spring: a bungee jelly bouncing under a gantry.
     */
    let gantry = commands
        .spawn((Transform::from_xyz(6.5, 5.5, 0.0), RigidBody::Fixed))
        .id();
    commands.spawn((
        jelly(Vec2::new(6.5, 3.2), Vec2::splat(0.45), 6.0e3, &color),
        ImpulseJoint::new(gantry, SpringJointBuilder::new(1.2, 25.0, 1.5)),
    ));

    /*
     * Pin-slot: a jelly bead sliding and spinning along a vertical pole, caught by the slot
     * limits. The pole collides with nothing: the bead's particles would otherwise rest on it.
     */
    let pole_pos = Vec2::new(9.5, 2.5);
    let pole = commands
        .spawn((
            Transform::from_translation(pole_pos.extend(0.0)),
            RigidBody::Fixed,
            Collider::cuboid(0.05, 2.5),
            CollisionGroups::new(Group::NONE, Group::NONE),
        ))
        .id();
    commands.spawn((
        jelly(
            pole_pos + Vec2::new(0.0, 1.7),
            Vec2::splat(0.35),
            8.0e3,
            &color,
        ),
        ImpulseJoint::new(pole, PinSlotJointBuilder::new(Vec2::Y).limits([-1.8, 1.8])),
    ));

    /*
     * Same-soft-body joint: a motorized revolute hinges two disjoint half clusters of one soft
     * bar at its middle, so the body folds and flaps at its own hinge.
     */
    let bar_center = Vec2::new(13.5, 3.0);
    let bar = SoftBody::grid(Vec2::new(1.0, 0.22), 9, 3).map(|b| {
        b.cell_model(SoftBodyCellModel::Corotational)
            .particle_mass(0.08)
            .particle_radius(0.06)
    });
    let positions: Vec<Vec2> = bar
        .builder
        .particle_positions()
        .iter()
        .map(|p| bar_center + *p)
        .collect();
    let half = |left: bool| -> Vec<u32> {
        (0..positions.len() as u32)
            .filter(|i| {
                let dx = positions[*i as usize].x - bar_center.x;
                if left {
                    dx < -1.0e-3
                } else {
                    dx > 1.0e-3
                }
            })
            .collect()
    };
    let (left_half, right_half) = (half(true), half(false));
    let centroid = |particles: &[u32]| {
        particles
            .iter()
            .map(|i| positions[*i as usize])
            .sum::<Vec2>()
            / particles.len() as f32
    };
    let (left_pos, right_pos) = (centroid(&left_half), centroid(&right_half));
    let bar = commands
        .spawn((
            Transform::from_translation(bar_center.extend(0.0)),
            bar,
            SoftBodyMaterial(RapierSoftBodyMaterial {
                young_modulus: 2.0e4,
                poisson_ratio: 0.35,
                elastic_damping_ratio: 1.0,
                ..default()
            }),
            SoftBodyMeshSync::default(),
            MeshMaterial2d(color.clone()),
        ))
        .id();
    // Hold the bar's left half in the air, then flap the right half about the middle hinge.
    let bar_anchor = commands
        .spawn((
            Transform::from_translation(left_pos.extend(0.0)),
            RigidBody::Fixed,
        ))
        .id();
    let left_cluster = commands
        .spawn((
            SoftBodyCluster::new(bar, left_half),
            ImpulseJoint::new(bar_anchor, FixedJointBuilder::new()),
        ))
        .id();
    commands.spawn((
        FlapJoint,
        SoftBodyCluster::new(bar, right_half),
        ImpulseJoint::new(
            left_cluster,
            RevoluteJointBuilder::new()
                .local_anchor1(bar_center - left_pos)
                .local_anchor2(bar_center - right_pos)
                .motor_position(0.0, 80.0, 10.0),
        ),
    ));

    /*
     * A jelly hanging from a two-link arm, while a rope joint ties the same jelly to a rigid
     * crate on the ground.
     */
    let arm_root = commands
        .spawn((Transform::from_xyz(17.0, 6.0, 0.0), RigidBody::Fixed))
        .id();
    let link1 = commands
        .spawn((
            Transform::from_xyz(18.2, 6.0, 0.0),
            RigidBody::Dynamic,
            Collider::capsule_x(0.5, 0.08),
            ColliderMassProperties::Density(2.0),
            ImpulseJoint::new(
                arm_root,
                RevoluteJointBuilder::new().local_anchor2(Vec2::new(-1.2, 0.0)),
            ),
        ))
        .id();
    let link2 = commands
        .spawn((
            Transform::from_xyz(19.4, 6.0, 0.0),
            RigidBody::Dynamic,
            Collider::capsule_x(0.5, 0.08),
            ColliderMassProperties::Density(2.0),
            ImpulseJoint::new(
                link1,
                RevoluteJointBuilder::new()
                    .local_anchor1(Vec2::new(0.6, 0.0))
                    .local_anchor2(Vec2::new(-0.6, 0.0)),
            ),
        ))
        .id();
    let pendulum = commands
        .spawn((
            jelly(Vec2::new(20.2, 5.2), Vec2::splat(0.5), 5.0e3, &color),
            ImpulseJoint::new(
                link2,
                RevoluteJointBuilder::new()
                    .local_anchor1(Vec2::new(0.7, 0.0))
                    .local_anchor2(Vec2::new(0.0, 0.6))
                    .contacts_enabled(false),
            ),
        ))
        .id();
    commands.spawn((
        Transform::from_xyz(20.2, 0.3, 0.0),
        RigidBody::Dynamic,
        Collider::cuboid(0.3, 0.3),
        ColliderMassProperties::Density(0.5),
        ImpulseJoint::new(pendulum, RopeJointBuilder::new(4.2)),
    ));

    /*
     * Kinematic cluster (no joint): a soft rope whose pinned top cluster is waved rigidly.
     */
    let rope_top = Vec2::new(-16.0, 5.5);
    let rope = commands
        .spawn((
            Transform::from_translation(rope_top.extend(0.0)),
            SoftBody::rope(Vec2::ZERO, Vec2::new(0.0, -4.0), 20).map(|b| b.particle_mass(0.05)),
        ))
        .id();
    // The center of the first two particles, the rope's particles being 4 / 19 apart.
    let home = rope_top - Vec2::new(0.0, 2.0 / 19.0);
    commands.spawn((
        SoftBodyCluster::new(rope, [0, 1]),
        Grip { home },
        // Pins the cluster, and moves it to this pose.
        SoftBodyClusterKinematicTarget(Transform::from_translation(home.extend(0.0))),
    ));
}

/// Moves the targets of the prismatic and flapping motors.
fn animate_motors(
    time: Res<Time>,
    mut rail_joints: Query<&mut ImpulseJoint, (With<RailJoint>, Without<FlapJoint>)>,
    mut flap_joints: Query<&mut ImpulseJoint, With<FlapJoint>>,
) {
    let t = time.elapsed_secs();
    for mut joint in &mut rail_joints {
        joint
            .data
            .as_mut()
            .set_motor_position(JointAxis::LinX, 1.5 * (0.6 * t).sin(), 40.0, 8.0);
    }
    for mut joint in &mut flap_joints {
        joint
            .data
            .as_mut()
            .set_motor_position(JointAxis::AngX, 0.8 * (1.4 * t).sin(), 80.0, 10.0);
    }
}

/// Waves the grip cluster along a kinematic path.
fn wave_grip(time: Res<Time>, mut grips: Query<(&Grip, &mut SoftBodyClusterKinematicTarget)>) {
    let t = time.elapsed_secs();
    for (grip, mut target) in &mut grips {
        let position = grip.home + Vec2::new(1.2 * (0.7 * t).sin(), 0.15 * (1.9 * t).sin());
        target.0 = Transform::from_translation(position.extend(0.0))
            .with_rotation(Quat::from_rotation_z(0.5 * (1.1 * t).sin()));
    }
}
