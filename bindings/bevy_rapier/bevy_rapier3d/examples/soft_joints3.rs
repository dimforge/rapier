//! Impulse joints attached to soft bodies, through the soft body entities (for the whole body) or
//! `SoftBodyCluster` entities: revolute, spherical, fixed, prismatic, rope, spring and generic
//! joints between jellies, cloths and rigid or kinematic bodies, plus a hinge between two
//! clusters of one soft bar, a jelly hanging from a multibody arm, and a kinematic cluster waving
//! a banner.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

/// Marks the joint of the jelly shuttled along its rail.
#[derive(Component)]
struct RailJoint;

/// Marks the hinge between the two halves of the soft bar.
#[derive(Component)]
struct FlapJoint;

/// The kinematic body the cloth's corner is attached to, circling around `home`.
#[derive(Component)]
struct Mover {
    home: Vec3,
}

/// The cluster of the banner's top edge, waved kinematically from its initial position.
#[derive(Component)]
struct Grip {
    /// The initial position of the cluster's center.
    home: Vec3,
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
        .add_systems(Update, (animate_motors, move_mover, wave_grip))
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(13.0, 9.0, 15.0).looking_at(Vec3::new(0.0, 1.5, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// The components of a jelly cube of 4x4x4 particles centered at `center`.
fn jelly(
    center: Vec3,
    half_extents: Vec3,
    young_modulus: f32,
    color: &Handle<StandardMaterial>,
) -> impl Bundle {
    (
        Transform::from_translation(center),
        SoftBody::cuboid(half_extents, 4, 4, 4).map(|b| {
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
        MeshMaterial3d(color.clone()),
    )
}

/// The centroid of the given particle positions.
fn centroid(positions: &[Vec3], particles: &[u32]) -> Vec3 {
    particles
        .iter()
        .map(|i| positions[*i as usize])
        .sum::<Vec3>()
        / particles.len() as f32
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let color = materials.add(Color::srgb(0.55, 0.75, 0.95));
    let cloth_color = materials.add(StandardMaterial {
        base_color: Color::srgb(0.9, 0.4, 0.3),
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    let rigid_color = materials.add(Color::srgb(0.4, 0.4, 0.45));

    /*
     * Ground.
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(18.0, 0.1, 18.0),
    ));

    /*
     * Revolute + velocity motor: a jelly spun about Y at its center.
     */
    let center = Vec3::new(-6.0, 1.6, -4.0);
    let spinner = commands
        .spawn(jelly(center, Vec3::splat(0.5), 8.0e3, &color))
        .id();
    let pivot = commands
        .spawn((Transform::from_translation(center), RigidBody::Fixed))
        .id();
    // A joint on a soft body entity attaches the whole soft body.
    commands.entity(spinner).insert(ImpulseJoint::new(
        pivot,
        RevoluteJointBuilder::new(Vec3::Y).motor_velocity(1.5, 60.0),
    ));

    /*
     * Spherical: a cloth's corner cluster follows a circling kinematic mover (two-way: the joint
     * feels the cloth's weight). Particle `(i, j)` of a cloth is at `origin + i * du + j * dv`,
     * with the index `i * nv + j`.
     */
    let (du, dv) = (Vec3::X * 0.16, Vec3::Z * 0.16);
    let cloth_origin = Vec3::new(0.0, 2.6, 2.5);
    let cloth = commands
        .spawn((
            Transform::from_translation(cloth_origin),
            SoftBody::cloth(Vec3::ZERO, du, dv, 12, 12),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(cloth_color.clone()),
        ))
        .id();
    // The particles `(0, 0)`, `(0, 1)` and `(1, 0)`.
    let corner_pos = cloth_origin + (du + dv) / 3.0;
    let mover = commands
        .spawn((
            Mover { home: corner_pos },
            Transform::from_translation(corner_pos),
            RigidBody::KinematicPositionBased,
        ))
        .id();
    commands.spawn((
        SoftBodyCluster::new(cloth, [0, 1, 12]),
        ImpulseJoint::new(mover, SphericalJointBuilder::new()),
    ));

    /*
     * Fixed: a rigid plate welded flat onto a jelly's top-face cluster; the full-rank face
     * cluster lets the weld hold the plate's orientation as the jelly wobbles.
     */
    let center = Vec3::new(-2.5, 0.61, -4.0);
    let wobbler = SoftBody::cuboid(Vec3::splat(0.6), 4, 4, 4);
    let positions = wobbler.builder.particle_positions().to_vec();
    let top_face: Vec<u32> = (0..positions.len() as u32)
        .filter(|i| positions[*i as usize].y > 0.6 - 1.0e-3)
        .collect();
    let top_pos = center + centroid(&positions, &top_face);
    let wobbler = commands
        .spawn(jelly(center, Vec3::splat(0.6), 2.5e3, &color))
        .id();
    let plate = commands
        .spawn((
            Transform::from_translation(top_pos + Vec3::new(0.0, 0.12, 0.0)),
            RigidBody::Dynamic,
            Collider::cuboid(0.7, 0.06, 0.7),
            ColliderMassProperties::Density(0.4),
            Mesh3d(meshes.add(Cuboid::new(1.4, 0.12, 1.4))),
            MeshMaterial3d(rigid_color.clone()),
        ))
        .id();
    commands.spawn((
        SoftBodyCluster::new(wobbler, top_face),
        ImpulseJoint::new(
            plate,
            FixedJointBuilder::new().local_anchor1(Vec3::new(0.0, -0.12, 0.0)),
        ),
    ));
    // A small crate dropped on the plate to make it wobble.
    commands.spawn((
        Transform::from_translation(top_pos + Vec3::new(0.3, 1.4, 0.2)),
        RigidBody::Dynamic,
        Collider::cuboid(0.15, 0.15, 0.15),
        ColliderMassProperties::Density(1.5),
        Mesh3d(meshes.add(Cuboid::new(0.3, 0.3, 0.3))),
        MeshMaterial3d(rigid_color.clone()),
    ));

    /*
     * Prismatic + limits + position motor: a jelly shuttled along a rail between two stops.
     */
    let center = Vec3::new(2.0, 0.85, -4.0);
    let shuttle = commands
        .spawn(jelly(center, Vec3::splat(0.4), 6.0e3, &color))
        .id();
    let rail = commands
        .spawn((Transform::from_translation(center), RigidBody::Fixed))
        .id();
    commands.entity(shuttle).insert((
        RailJoint,
        ImpulseJoint::new(
            rail,
            PrismaticJointBuilder::new(Vec3::X)
                .limits([-1.8, 1.8])
                .motor_position(0.0, 40.0, 8.0),
        ),
    ));

    /*
     * Rope: two jellies chained over a ledge, one dragging the other.
     */
    commands.spawn((
        Transform::from_xyz(-1.0, 1.0, -9.0),
        Collider::cuboid(1.6, 1.0, 1.2),
        Mesh3d(meshes.add(Cuboid::new(3.2, 2.0, 2.4))),
        MeshMaterial3d(rigid_color.clone()),
    ));
    let anchor_jelly = commands
        .spawn(jelly(
            Vec3::new(-1.0, 2.6, -9.0),
            Vec3::splat(0.5),
            1.2e4,
            &color,
        ))
        .id();
    let hanging_jelly = commands
        .spawn(jelly(
            Vec3::new(1.6, 2.6, -9.0),
            Vec3::splat(0.5),
            1.2e4,
            &color,
        ))
        .id();
    commands
        .entity(hanging_jelly)
        .insert(ImpulseJoint::new(anchor_jelly, RopeJointBuilder::new(2.2)));

    /*
     * Spring: a bungee jelly bouncing under a gantry.
     */
    let bungee = commands
        .spawn(jelly(
            Vec3::new(5.5, 3.2, 2.5),
            Vec3::splat(0.45),
            6.0e3,
            &color,
        ))
        .id();
    let gantry = commands
        .spawn((Transform::from_xyz(5.5, 5.5, 2.5), RigidBody::Fixed))
        .id();
    commands.entity(bungee).insert(ImpulseJoint::new(
        gantry,
        SpringJointBuilder::new(1.2, 25.0, 1.5),
    ));

    /*
     * Generic: a jelly bead on a pole, translations locked to the pole's Y axis (limited along
     * it) and every rotation free: a 3D pin-slot joint spelled as a generic joint. The pole
     * collides with nothing: the bead's particles would otherwise rest on it.
     */
    let pole_base = Vec3::new(8.5, 0.0, -4.0);
    let pole = commands
        .spawn((
            Transform::from_translation(pole_base + Vec3::new(0.0, 2.5, 0.0)),
            RigidBody::Fixed,
            Collider::cylinder(2.5, 0.05),
            CollisionGroups::new(Group::NONE, Group::NONE),
        ))
        .id();
    let bead = commands
        .spawn(jelly(
            pole_base + Vec3::new(0.0, 4.2, 0.0),
            Vec3::splat(0.35),
            8.0e3,
            &color,
        ))
        .id();
    commands.entity(bead).insert(ImpulseJoint::new(
        pole,
        GenericJointBuilder::new(JointAxesMask::LIN_X | JointAxesMask::LIN_Z)
            .local_axis1(Vec3::Y)
            .local_axis2(Vec3::Y)
            .limits(JointAxis::LinY, [-1.8, 1.8]),
    ));

    /*
     * Same-soft-body joint: one soft bar whose two disjoint half clusters are hinged by a
     * motorized revolute at its middle, so the body folds and flaps at its own hinge.
     */
    let bar_center = Vec3::new(2.5, 2.2, 5.5);
    let bar = SoftBody::cuboid(Vec3::new(1.0, 0.22, 0.4), 7, 3, 3).map(|b| {
        b.cell_model(SoftBodyCellModel::Corotational)
            .particle_mass(0.08)
            .particle_radius(0.06)
    });
    let positions: Vec<Vec3> = bar
        .builder
        .particle_positions()
        .iter()
        .map(|p| bar_center + *p)
        .collect();
    let left_half: Vec<u32> = (0..positions.len() as u32)
        .filter(|i| positions[*i as usize].x < bar_center.x - 1.0e-3)
        .collect();
    let right_half: Vec<u32> = (0..positions.len() as u32)
        .filter(|i| positions[*i as usize].x > bar_center.x + 1.0e-3)
        .collect();
    let left_pos = centroid(&positions, &left_half);
    let right_pos = centroid(&positions, &right_half);
    let bar = commands
        .spawn((
            Transform::from_translation(bar_center),
            bar,
            SoftBodyMaterial(RapierSoftBodyMaterial {
                young_modulus: 2.0e4,
                poisson_ratio: 0.35,
                elastic_damping_ratio: 1.0,
                ..default()
            }),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(color.clone()),
        ))
        .id();
    // Hold the bar's left half in the air, then flap the right half about the middle hinge.
    let bar_anchor = commands
        .spawn((Transform::from_translation(left_pos), RigidBody::Fixed))
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
            RevoluteJointBuilder::new(Vec3::Z)
                .local_anchor1(bar_center - left_pos)
                .local_anchor2(bar_center - right_pos)
                .motor_position(0.0, 80.0, 10.0),
        ),
    ));

    /*
     * Multibody + rigid on one soft body: a jelly hanging from a two-link multibody arm while a
     * rope joint ties the same jelly to a rigid crate on the ground.
     */
    let arm_root = commands
        .spawn((Transform::from_xyz(7.0, 5.0, 6.0), RigidBody::Fixed))
        .id();
    let link1 = commands
        .spawn((
            Transform::from_xyz(8.2, 5.0, 6.0),
            RigidBody::Dynamic,
            Collider::capsule_x(0.5, 0.08),
            ColliderMassProperties::Density(2.0),
            MultibodyJoint::new(
                arm_root,
                RevoluteJointBuilder::new(Vec3::Z).local_anchor2(Vec3::new(-1.2, 0.0, 0.0)),
            ),
        ))
        .id();
    let link2 = commands
        .spawn((
            Transform::from_xyz(9.4, 5.0, 6.0),
            RigidBody::Dynamic,
            Collider::capsule_x(0.5, 0.08),
            ColliderMassProperties::Density(2.0),
            MultibodyJoint::new(
                link1,
                RevoluteJointBuilder::new(Vec3::Z)
                    .local_anchor1(Vec3::new(0.6, 0.0, 0.0))
                    .local_anchor2(Vec3::new(-0.6, 0.0, 0.0)),
            ),
        ))
        .id();
    let pendulum = commands
        .spawn(jelly(
            Vec3::new(10.2, 4.2, 6.0),
            Vec3::splat(0.5),
            5.0e3,
            &color,
        ))
        .id();
    commands.entity(pendulum).insert(ImpulseJoint::new(
        link2,
        SphericalJointBuilder::new()
            .local_anchor1(Vec3::new(0.7, 0.0, 0.0))
            .local_anchor2(Vec3::new(0.0, 0.6, 0.0)),
    ));
    commands.spawn((
        Transform::from_xyz(10.2, 0.3, 6.0),
        RigidBody::Dynamic,
        Collider::cuboid(0.3, 0.3, 0.3),
        ColliderMassProperties::Density(0.5),
        ImpulseJoint::new(pendulum, RopeJointBuilder::new(3.2)),
        Mesh3d(meshes.add(Cuboid::new(0.6, 0.6, 0.6))),
        MeshMaterial3d(rigid_color),
    ));

    /*
     * Kinematic cluster (no joint): a banner whose pinned top-edge cluster (the particles
     * `(i, 0)`) is waved rigidly.
     */
    let (du, dv) = (Vec3::X * 0.18, Vec3::NEG_Y * 0.18);
    let banner_origin = Vec3::new(-8.0, 3.2, 4.0);
    let banner = commands
        .spawn((
            Transform::from_translation(banner_origin),
            SoftBody::cloth(Vec3::ZERO, du, dv, 14, 10),
            SoftBodyMeshSync::default(),
            MeshMaterial3d(cloth_color),
        ))
        .id();
    let home = banner_origin + du * 6.5;
    commands.spawn((
        SoftBodyCluster::new(banner, (0..14).map(|i| i * 10)),
        Grip { home },
        // Pins the cluster, and moves it to this pose.
        SoftBodyClusterKinematicTarget(Transform::from_translation(home)),
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
    // The free axis of a revolute joint is its local `AngX`.
    for mut joint in &mut flap_joints {
        joint
            .data
            .as_mut()
            .set_motor_position(JointAxis::AngX, 0.8 * (1.4 * t).sin(), 80.0, 10.0);
    }
}

/// Circles the kinematic body the cloth's corner is attached to.
fn move_mover(time: Res<Time>, mut movers: Query<(&Mover, &mut Transform)>) {
    let t = time.elapsed_secs();
    for (mover, mut transform) in &mut movers {
        transform.translation = mover.home
            + Vec3::new(
                1.2 * (0.8 * t).sin(),
                0.4 * (1.3 * t).sin(),
                1.2 * (0.8 * t).cos() - 1.2,
            );
    }
}

/// Waves the grip cluster along a kinematic path.
fn wave_grip(time: Res<Time>, mut grips: Query<(&Grip, &mut SoftBodyClusterKinematicTarget)>) {
    let t = time.elapsed_secs();
    for (grip, mut target) in &mut grips {
        let position = grip.home + Vec3::new(1.5 * (0.7 * t).sin(), 0.2 * (1.9 * t).sin(), 0.0);
        target.0 = Transform::from_translation(position)
            .with_rotation(Quat::from_axis_angle(Vec3::X, 0.35 * (1.1 * t).sin()));
    }
}
