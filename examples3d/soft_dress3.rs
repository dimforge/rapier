//! A kinematic mannequin (capsules driven by a procedural dance) wearing a cloth skirt
//! (`SoftBodyBuilder::cloth_tube`) pinned at the waist and a sleeveless t-shirt tube pinned to
//! the torso; the cloth collides with the body and, for the skirt, with itself.

use kiss3d::color::Color;
use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

const UPPER_ARM: Real = 0.34;
const FOREARM: Real = 0.32;
const THIGH: Real = 0.5;
const SHIN: Real = 0.5;
const TORSO_RADIUS: Real = 0.15;

/// The body parts, in the order of `Frame::poses`.
#[derive(Clone, Copy)]
enum Part {
    Pelvis,
    Torso,
    Head,
    UpperArm(usize),
    Forearm(usize),
    Thigh(usize),
    Shin(usize),
}

impl Part {
    fn index(self) -> usize {
        match self {
            Part::Pelvis => 0,
            Part::Torso => 1,
            Part::Head => 2,
            Part::UpperArm(i) => 3 + i,
            Part::Forearm(i) => 5 + i,
            Part::Thigh(i) => 7 + i,
            Part::Shin(i) => 9 + i,
        }
    }
}

const NUM_PARTS: usize = 11;

/// The pose placing a capsule (along its local Y) from `joint` along `dir`.
fn limb_pose(joint: Vector, dir: Vector, half_length: Real) -> Pose {
    let dir = dir.normalize();
    Pose::from_parts(
        joint + dir * half_length,
        Rotation::from_rotation_arc(Vector::Y, dir),
    )
}

/// The mannequin's pose at time `t`: the dance makes the pelvis sway sideways and bounce on
/// the beat while the body slowly turns; the legs step, the arms swing.
fn frame_at(t: Real) -> [Pose; NUM_PARTS] {
    let beat = 2.0 * t;
    let yaw = 0.7 * (0.35 * t).sin() + 0.25 * t;
    let sway = 0.25 * beat.sin();
    let bounce = 0.04 * (2.0 * beat).sin().abs();
    let roll = 0.12 * beat.sin();
    let turn = Rotation::from_rotation_y(yaw);
    let hips_rot = turn * Rotation::from_rotation_z(roll);
    let hips = Vector::new(0.0, 0.95 + bounce, 0.0) + turn * Vector::new(sway, 0.0, 0.0);
    let up = hips_rot * Vector::Y;
    let side = hips_rot * Vector::X;
    let forward = hips_rot * Vector::Z;

    let mut poses = [Pose::IDENTITY; NUM_PARTS];
    // Pelvis and torso, leaning a little into the sway; the head on top.
    let lean = Rotation::from_axis_angle(forward, -0.5 * roll);
    let torso_rot = lean * hips_rot;
    let torso_center = hips + up * 0.42;
    poses[Part::Pelvis.index()] = Pose::from_parts(hips, hips_rot);
    poses[Part::Torso.index()] = Pose::from_parts(torso_center, torso_rot);
    let neck = torso_center + torso_rot * Vector::Y * 0.28;
    poses[Part::Head.index()] = Pose::from_parts(neck + up * 0.13, torso_rot);

    // Legs: thighs swing back and forth alternately, the knees bend on the forward swing.
    for (i, s) in [1.0, -1.0].iter().enumerate() {
        let hip = hips + side * (0.11 * s) - up * 0.05;
        let phase = if i == 0 { 0.0 } else { core::f32::consts::PI };
        let swing = 0.45 * (beat + phase).sin();
        let thigh_dir = (-up * swing.cos() + forward * swing.sin() + side * (0.05 * s)).normalize();
        let knee = hip + thigh_dir * THIGH;
        let bend = 0.9 * swing.max(0.0);
        let shin_dir = -up * (swing - bend).cos() + forward * (swing - bend).sin();
        poses[Part::Thigh(i).index()] = limb_pose(hip, thigh_dir, THIGH * 0.5);
        poses[Part::Shin(i).index()] = limb_pose(knee, shin_dir, SHIN * 0.5);
    }

    // Arms: raised sideways and swinging, the elbows bent.
    for (i, s) in [1.0, -1.0].iter().enumerate() {
        let shoulder = neck - up * 0.06 + torso_rot * Vector::X * (0.22 * s);
        let raise = 0.6 + 0.6 * (beat + if i == 0 { 0.0 } else { 1.5 }).sin();
        let arm_dir = (side * (s * raise.cos()) - up * raise.sin() * 0.6
            + forward * 0.2 * (0.7 * beat).sin())
        .normalize();
        let elbow = shoulder + arm_dir * UPPER_ARM;
        let fore_dir = (arm_dir + up * 0.9 + forward * 0.5).normalize();
        poses[Part::UpperArm(i).index()] = limb_pose(shoulder, arm_dir, UPPER_ARM * 0.5);
        poses[Part::Forearm(i).index()] = limb_pose(elbow, fore_dir, FOREARM * 0.5);
    }
    poses
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(10.0, 0.1, 10.0),
    );

    /*
     * The mannequin: kinematic capsules (a pelvis, a torso, a head, arms and legs in two
     * segments), placed at the dance's first frame and animated below.
     */
    let skin = Color::new(0.93, 0.8, 0.68, 1.0);
    let frame0 = frame_at(0.0);
    let shapes: [ColliderBuilder; NUM_PARTS] = [
        ColliderBuilder::capsule_x(0.08, 0.13),
        ColliderBuilder::capsule_y(0.18, TORSO_RADIUS),
        ColliderBuilder::ball(0.12),
        ColliderBuilder::capsule_y(UPPER_ARM * 0.5 - 0.03, 0.05),
        ColliderBuilder::capsule_y(UPPER_ARM * 0.5 - 0.03, 0.05),
        ColliderBuilder::capsule_y(FOREARM * 0.5 - 0.03, 0.045),
        ColliderBuilder::capsule_y(FOREARM * 0.5 - 0.03, 0.045),
        ColliderBuilder::capsule_y(THIGH * 0.5 - 0.05, 0.085),
        ColliderBuilder::capsule_y(THIGH * 0.5 - 0.05, 0.085),
        ColliderBuilder::capsule_y(SHIN * 0.5 - 0.04, 0.065),
        ColliderBuilder::capsule_y(SHIN * 0.5 - 0.04, 0.065),
    ];
    let mut parts = Vec::with_capacity(NUM_PARTS);
    for (i, shape) in shapes.into_iter().enumerate() {
        let (body, _) = world.insert(
            RigidBodyBuilder::kinematic_position_based().pose(frame0[i]),
            shape.friction(0.4),
        );
        viewer.set_initial_body_color(body, skin);
        parts.push(body);
    }

    /*
     * The clothes: tubes of cloth whose first ring is pinned and driven with a body part.
     */
    let mut pins: Vec<(SoftBodyHandle, usize, Part, Vector)> = Vec::new();
    let mut piece = |world: &mut PhysicsWorld,
                     viewer: &mut TestbedViewer,
                     tube: SoftBodyBuilder,
                     num_around: usize,
                     part: Part,
                     self_contacts: bool,
                     color: Color| {
        let handle = world.insert_soft_body(
            tube.pinned_particles(0..num_around as u32)
                .material(SoftBodyMaterial {
                    bend_softness: SpringCoefficients::new(3.0, 1.0),
                    ..SoftBodyMaterial::uniform(SpringCoefficients::new(80.0, 1.0))
                })
                .particle_mass(0.01)
                .particle_radius(0.02)
                .self_contacts(self_contacts)
                .surface_collider(ColliderBuilder::ball(0.02).friction(0.5)),
        );
        viewer.set_initial_soft_body_color(handle, color);
        // The pinned ring's offsets, in the frame of the part driving it.
        let part_pose = frame0[part.index()];
        let sb = &world.soft_bodies[handle];
        for k in 0..num_around {
            pins.push((
                handle,
                k,
                part,
                part_pose.inverse_transform_point(sb.particle_position(k)),
            ));
        }
    };
    let red = Color::new(0.75, 0.15, 0.3, 1.0);
    // The skirt: from a ring around the waist (pinned to the pelvis) flaring down to the hem.
    let pelvis = frame0[Part::Pelvis.index()];
    let waist = pelvis * Vector::new(0.0, 0.1, 0.0);
    let hem = Vector::new(waist.x, 0.25, waist.z);
    piece(
        &mut world,
        viewer,
        SoftBodyBuilder::cloth_tube(waist, hem - waist, 0.2, 0.62, 56, 18),
        56,
        Part::Pelvis,
        true,
        red,
    );
    // The t-shirt: a sleeveless tube from the shoulders down to just above the pelvis, hugging
    // the torso capsule at the top (its radius plus a small gap) and widening toward the hem,
    // its top ring pinned to the torso.
    let white = Color::new(0.95, 0.95, 0.9, 1.0);
    let torso = frame0[Part::Torso.index()];
    let shirt_top = torso * Vector::new(0.0, 0.26, 0.0);
    let shirt_bottom = torso * Vector::new(0.0, -0.26, 0.0);
    piece(
        &mut world,
        viewer,
        SoftBodyBuilder::cloth_tube(
            shirt_top,
            shirt_bottom - shirt_top,
            TORSO_RADIUS + 0.012,
            TORSO_RADIUS + 0.09,
            48,
            16,
        ),
        48,
        Part::Torso,
        false,
        white,
    );

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(3.0, 2.0, 4.0), Vec3::new(0.0, 0.9, 0.0));

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if !viewer.simulating() {
            continue;
        }
        t += world.integration_parameters.dt;
        let frame = frame_at(t);
        for (i, body) in parts.iter().enumerate() {
            world.bodies[*body].set_next_kinematic_position(frame[i]);
        }
        for (handle, i, part, offset) in &pins {
            world.soft_bodies[*handle]
                .set_particle_kinematic_target(*i, frame[part.index()] * *offset);
        }
        world.step();
    }
    Ok(())
}
