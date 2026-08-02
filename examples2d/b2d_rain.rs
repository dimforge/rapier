//! Port of box2d's `rain` benchmark (`CreateRain` + `StepRain`,
//! `box2d/shared/benchmarks.c`, using `box2d/shared/human.c`). Release: a bank
//! of static box "floors" onto which columns of 5-human ragdoll groups are
//! rained down over time and recycled once the grid is full.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;
use std::f32::consts::PI;

const ROW_COUNT: usize = 5;
const COLUMN_COUNT: usize = 40;
const GROUP_SIZE: usize = 5;
const GRID_SIZE: f32 = 0.5;
const GRID_COUNT: usize = 500;

struct RainState {
    groups: Vec<Vec<HumanHandles>>,
    column_count: usize,
    column_index: usize,
}

fn create_group(
    viewer: &mut TestbedViewer,
    world: &mut PhysicsWorld,
    state: &mut RainState,
    row: usize,
    col: usize,
) {
    let group_index = row * COLUMN_COUNT + col;
    let span = GRID_COUNT as f32 * GRID_SIZE;
    let group_distance = span / COLUMN_COUNT as f32;

    let mut x = -0.5 * span + group_distance * (col as f32 + 0.5);
    let y = 40.0 + 45.0 * row as f32;

    let mut humans = Vec::with_capacity(GROUP_SIZE);
    for i in 0..GROUP_SIZE {
        // box2d passes groupIndex = i + 1, so ragdolls in the same slot never
        // collide with each other; map that to the per-slot filter bit.
        let human = create_human(world, Vector::new(x, y), 1.0, 5.0, 0.5, (i + 1) as u32);
        for bone in human.bones {
            viewer.add_body(bone, world);
        }
        humans.push(human);
        x += 0.5;
    }
    state.groups[group_index] = humans;
}

fn destroy_group(
    viewer: &mut TestbedViewer,
    world: &mut PhysicsWorld,
    state: &mut RainState,
    row: usize,
    col: usize,
) {
    let group_index = row * COLUMN_COUNT + col;
    for human in state.groups[group_index].drain(..) {
        for bone in human.bones {
            world.remove_body(bone);
            viewer.remove_body(bone);
        }
    }
}

/// box2d `StepRain` (release: spawn/recycle one column every 8 steps).
fn step_rain(
    viewer: &mut TestbedViewer,
    world: &mut PhysicsWorld,
    state: &mut RainState,
    step_count: i32,
) {
    if step_count & 0x7 != 0 {
        return;
    }

    if state.column_count < COLUMN_COUNT {
        let col = state.column_count;
        for row in 0..ROW_COUNT {
            create_group(viewer, world, state, row, col);
        }
        state.column_count += 1;
    } else {
        let col = state.column_index;
        for row in 0..ROW_COUNT {
            destroy_group(viewer, world, state, row, col);
            create_group(viewer, world, state, row, col);
        }
        state.column_index = (state.column_index + 1) % COLUMN_COUNT;
    }
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box2d's `b2DefaultWorldDef`: gravity (0, -10). box2d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0);

    // Static box "floors": ROW_COUNT rows of a long strip of boxes.
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    let mut y = 0.0f32;
    for _ in 0..ROW_COUNT {
        let mut x = -0.5 * GRID_COUNT as f32 * GRID_SIZE;
        for _ in 0..=GRID_COUNT {
            world.insert_collider(
                ColliderBuilder::cuboid(0.5 * GRID_SIZE, 0.5 * GRID_SIZE)
                    .translation(Vector::new(x, y)),
                Some(ground),
            );
            x += GRID_SIZE;
        }
        y += 45.0;
    }

    let mut state = RainState {
        groups: (0..ROW_COUNT * COLUMN_COUNT).map(|_| Vec::new()).collect(),
        column_count: 0,
        column_index: 0,
    };

    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 110.0), 2.0);

    let mut step_count = 0i32;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            step_rain(viewer, &mut world, &mut state, step_count);
            step_count += 1;
            world.step();
        }
    }
    Ok(())
}

// ── Human ragdoll (box2d/shared/human.c) ─────────────────────────────────────

const BONE_COUNT: usize = 11;

const HIP: usize = 0;
const TORSO: usize = 1;
const HEAD: usize = 2;
const UPPER_LEFT_LEG: usize = 3;
const LOWER_LEFT_LEG: usize = 4;
const UPPER_RIGHT_LEG: usize = 5;
const LOWER_RIGHT_LEG: usize = 6;
const UPPER_LEFT_ARM: usize = 7;
const LOWER_LEFT_ARM: usize = 8;
const UPPER_RIGHT_ARM: usize = 9;
const LOWER_RIGHT_ARM: usize = 10;

/// One bone, transcribed from `human.c` (all lengths in units of `scale`).
struct BoneDef {
    parent: i32,
    /// Body position offset from the ragdoll origin (`bodyDef.position`).
    pos_y: f32,
    /// Capsule endpoints (x is always 0) + radius.
    cap_a: [f32; 2],
    cap_b: [f32; 2],
    cap_r: f32,
    /// Whether this bone carries the shared foot polygon (lower legs).
    has_foot: bool,
    /// Joint pivot Y (world offset from origin); joint to the parent.
    pivot_y: f32,
    /// Revolute limit `[lower, upper]`, radians.
    limits: [f32; 2],
    /// Extra rotation on the parent-side joint frame (lower arms use 0.25π).
    frame_a_angle: f32,
}

/// Handles to a spawned human's 11 bone bodies (index order matches `BoneId`).
struct HumanHandles {
    pub bones: [RigidBodyHandle; BONE_COUNT],
}

/// Spawn one box2d human ragdoll with its origin at `position`. `scale`,
/// `hertz`, `damping` mirror `CreateHuman`; `group_bit` selects the same-human
/// collision filter (box2d's negative `filter.groupIndex`, which disables all
/// self-collision within one ragdoll). Returns the bone handles so callers can
/// recycle the ragdoll (used by the `rain` benchmark).
fn create_human(
    world: &mut PhysicsWorld,
    position: Vector,
    scale: f32,
    hertz: f32,
    damping: f32,
    group_bit: u32,
) -> HumanHandles {
    let s = scale;
    let defs = human_bone_defs();
    let mut bones = [RigidBodyHandle::invalid(); BONE_COUNT];

    // Same-human collision filter: all bones share membership bit `group_bit`
    // and exclude it from their filter, so a ragdoll never self-collides while
    // still colliding with the ground and other ragdolls.
    let bit = Group::from_bits_truncate(1u32 << (group_bit % 24));
    let groups = InteractionGroups::new(bit, Group::ALL ^ bit, InteractionTestMode::And);

    // Shared foot polygon (box2d: rounded hull of 4 points).
    let foot_points: Vec<Vector> = [
        [-0.03, -0.185],
        [0.11, -0.185],
        [0.11, -0.16],
        [-0.03, -0.14],
    ]
    .iter()
    .map(|p| Vector::new(p[0] * s, p[1] * s))
    .collect();

    for (i, def) in defs.iter().enumerate() {
        let body =
            RigidBodyBuilder::dynamic().translation(position + Vector::new(0.0, def.pos_y * s));
        let handle = world.insert_body(body);

        let capsule = ColliderBuilder::capsule_from_endpoints(
            Vector::new(def.cap_a[0] * s, def.cap_a[1] * s),
            Vector::new(def.cap_b[0] * s, def.cap_b[1] * s),
            def.cap_r * s,
        )
        .friction(0.2)
        .collision_groups(groups);
        world.insert_collider(capsule, Some(handle));

        if def.has_foot {
            let foot = ColliderBuilder::round_convex_hull(&foot_points, 0.015 * s)
                .unwrap()
                .friction(0.05)
                .collision_groups(groups);
            world.insert_collider(foot, Some(handle));
        }

        bones[i] = handle;
    }

    // Soft angular spring (box2d hertz/damping) -> acceleration-based motor:
    // stiffness = w^2, damping = 2*zeta*w, w = 2*pi*hertz.
    let omega = 2.0 * PI * hertz;
    let stiffness = omega * omega;
    let motor_damping = 2.0 * damping * omega;

    for (i, def) in defs.iter().enumerate() {
        if def.parent < 0 {
            continue;
        }
        let parent = bones[def.parent as usize];
        let child = bones[i];
        let parent_y = defs[def.parent as usize].pos_y;

        // Anchors: pivot expressed in each body's local frame (bodies start
        // axis-aligned, so this is just the pivot minus the body position).
        let anchor_a = Vector::new(0.0, (def.pivot_y - parent_y) * s);
        let anchor_b = Vector::new(0.0, (def.pivot_y - def.pos_y) * s);
        let frame_a = Pose::from_parts(anchor_a, Rotation::new(def.frame_a_angle));
        let frame_b = Pose::from_translation(anchor_b);

        let joint = GenericJointBuilder::new(JointAxesMask::LIN_X | JointAxesMask::LIN_Y)
            .local_frame1(frame_a)
            .local_frame2(frame_b)
            .contacts_enabled(false)
            .limits(JointAxis::AngX, def.limits)
            .motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
            .motor_position(JointAxis::AngX, 0.0, stiffness, motor_damping);

        world.insert_impulse_joint(parent, child, joint);
    }

    HumanHandles { bones }
}

#[rustfmt::skip]
fn human_bone_defs() -> [BoneDef; BONE_COUNT] {
    let pi = PI;
    [
        // hip (root, no joint)
        BoneDef { parent: -1, pos_y: 0.95, cap_a: [0.0, -0.02], cap_b: [0.0, 0.02], cap_r: 0.095,
            has_foot: false, pivot_y: 0.0, limits: [0.0, 0.0], frame_a_angle: 0.0 },
        // torso
        BoneDef { parent: HIP as i32, pos_y: 1.2, cap_a: [0.0, -0.135], cap_b: [0.0, 0.135], cap_r: 0.09,
            has_foot: false, pivot_y: 1.0, limits: [-0.25 * pi, 0.0], frame_a_angle: 0.0 },
        // head
        BoneDef { parent: TORSO as i32, pos_y: 1.475, cap_a: [0.0, -0.038], cap_b: [0.0, 0.039], cap_r: 0.075,
            has_foot: false, pivot_y: 1.4, limits: [-0.3 * pi, 0.1 * pi], frame_a_angle: 0.0 },
        // upper left leg
        BoneDef { parent: HIP as i32, pos_y: 0.775, cap_a: [0.0, -0.125], cap_b: [0.0, 0.125], cap_r: 0.06,
            has_foot: false, pivot_y: 0.9, limits: [-0.05 * pi, 0.4 * pi], frame_a_angle: 0.0 },
        // lower left leg
        BoneDef { parent: UPPER_LEFT_LEG as i32, pos_y: 0.475, cap_a: [0.0, -0.155], cap_b: [0.0, 0.125], cap_r: 0.045,
            has_foot: true, pivot_y: 0.625, limits: [-0.5 * pi, -0.02 * pi], frame_a_angle: 0.0 },
        // upper right leg
        BoneDef { parent: HIP as i32, pos_y: 0.775, cap_a: [0.0, -0.125], cap_b: [0.0, 0.125], cap_r: 0.06,
            has_foot: false, pivot_y: 0.9, limits: [-0.05 * pi, 0.4 * pi], frame_a_angle: 0.0 },
        // lower right leg
        BoneDef { parent: UPPER_RIGHT_LEG as i32, pos_y: 0.475, cap_a: [0.0, -0.155], cap_b: [0.0, 0.125], cap_r: 0.045,
            has_foot: true, pivot_y: 0.625, limits: [-0.5 * pi, -0.02 * pi], frame_a_angle: 0.0 },
        // upper left arm
        BoneDef { parent: TORSO as i32, pos_y: 1.225, cap_a: [0.0, -0.125], cap_b: [0.0, 0.125], cap_r: 0.035,
            has_foot: false, pivot_y: 1.35, limits: [-0.1 * pi, 0.8 * pi], frame_a_angle: 0.0 },
        // lower left arm
        BoneDef { parent: UPPER_LEFT_ARM as i32, pos_y: 0.975, cap_a: [0.0, -0.125], cap_b: [0.0, 0.125], cap_r: 0.03,
            has_foot: false, pivot_y: 1.1, limits: [-0.2 * pi, 0.3 * pi], frame_a_angle: 0.25 * pi },
        // upper right arm
        BoneDef { parent: TORSO as i32, pos_y: 1.225, cap_a: [0.0, -0.125], cap_b: [0.0, 0.125], cap_r: 0.035,
            has_foot: false, pivot_y: 1.35, limits: [-0.1 * pi, 0.8 * pi], frame_a_angle: 0.0 },
        // lower right arm
        BoneDef { parent: UPPER_RIGHT_ARM as i32, pos_y: 0.975, cap_a: [0.0, -0.125], cap_b: [0.0, 0.125], cap_r: 0.03,
            has_foot: false, pivot_y: 1.1, limits: [-0.2 * pi, 0.3 * pi], frame_a_angle: 0.25 * pi },
    ]
}
