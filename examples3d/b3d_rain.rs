//! Port of box3d's `rain` benchmark (`CreateRain` + `StepRain`,
//! `box3d/shared/benchmarks.c`, using `box3d/shared/human.c`). Release settings:
//! a 10x10 grid of static cells (each a grid-mesh patch + torus obstacle) onto
//! which columns of 3-human ragdoll "groups" are rained down over time and
//! recycled once the grid is full.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;
use std::f32::consts::PI;

const GRID_COUNT: usize = 10;
const GROUP_SIZE: usize = 3;
const GRID_SIZE: f32 = 15.0;

struct RainState {
    /// Humans currently alive in each of the `GRID_COUNT * GRID_COUNT` cells.
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
    let group_index = row * GRID_COUNT + col;
    let span = GRID_COUNT as f32 * GRID_SIZE;
    let group_distance = span / GRID_COUNT as f32;

    let mut x = -0.5 * span + group_distance * (col as f32 + 0.5);
    let y = 20.0;
    let z = -0.5 * span + group_distance * (row as f32 + 0.5);

    let mut humans = Vec::with_capacity(GROUP_SIZE);
    for _ in 0..GROUP_SIZE {
        let human = create_human(
            world,
            Vector::new(x, y, z),
            5.0,
            1.0,
            0.7,
            group_index as u32,
        );
        // Register the newly-spawned bodies with the renderer (they're created
        // after `set_world`, so the viewer doesn't know about them yet).
        for bone in human.bones {
            viewer.add_body(bone, world);
        }
        humans.push(human);
        x += 0.75;
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
    let group_index = row * GRID_COUNT + col;
    for human in state.groups[group_index].drain(..) {
        for bone in human.bones {
            world.remove_body(bone);
            viewer.remove_body(bone);
        }
    }
}

/// box3d `StepRain` (release: spawn/recycle one column every 48 steps).
fn step_rain(
    viewer: &mut TestbedViewer,
    world: &mut PhysicsWorld,
    state: &mut RainState,
    step_count: i32,
) {
    if step_count & 0x2F != 0 {
        return;
    }

    if state.column_count < GRID_COUNT {
        let col = state.column_count;
        for row in 0..GRID_COUNT {
            create_group(viewer, world, state, row, col);
        }
        state.column_count += 1;
    } else {
        let col = state.column_index;
        for row in 0..GRID_COUNT {
            destroy_group(viewer, world, state, row, col);
            create_group(viewer, world, state, row, col);
        }
        state.column_index = (state.column_index + 1) % GRID_COUNT;
    }
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    // box3d's `b3DefaultWorldDef`: gravity (0, -10, 0). box3d steps at dt = 1/60
    // with 4 solver substeps, matching rapier's defaults.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -10.0, 0.0);

    // Static cells: a 10x10 grid, each cell one static body carrying a small
    // grid-mesh floor patch and a torus obstacle.
    let half_mesh_grid_rows = 4;
    let mesh_cell_width = GRID_SIZE / (2.0 * half_mesh_grid_rows as f32);
    let (grid_verts, grid_indices) = create_grid_mesh(
        2 * half_mesh_grid_rows,
        2 * half_mesh_grid_rows,
        mesh_cell_width,
    );
    let (torus_verts, torus_indices) = create_torus_mesh(16, 16, 0.25 * GRID_SIZE, 1.0);

    let span = GRID_SIZE * GRID_COUNT as f32;
    let mut x = -0.5 * span + 0.5 * GRID_SIZE;
    for _ in 0..GRID_COUNT {
        let mut z = -0.5 * span + 0.5 * GRID_SIZE;
        for _ in 0..GRID_COUNT {
            let cell =
                world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(x, 0.0, z)));
            world.insert_collider(
                ColliderBuilder::trimesh(grid_verts.clone(), grid_indices.clone()).unwrap(),
                Some(cell),
            );
            world.insert_collider(
                ColliderBuilder::trimesh(torus_verts.clone(), torus_indices.clone()).unwrap(),
                Some(cell),
            );
            z += GRID_SIZE;
        }
        x += GRID_SIZE;
    }

    let mut state = RainState {
        groups: (0..GRID_COUNT * GRID_COUNT).map(|_| Vec::new()).collect(),
        column_count: 0,
        column_index: 0,
    };

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(70.0, 30.0, 70.0), Vec3::new(0.0, 5.0, 0.0));

    // box3d calls the step function once with step 0 before the first world
    // step, then once per subsequent step.
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

/// Shared vertex/index grid builder for the box3d grid/wave meshes. `height`
/// yields the Y coordinate at grid cell `(ix, iz)`.
fn grid_mesh(
    x_count: usize,
    z_count: usize,
    cell_width: f32,
    height: impl Fn(usize, usize) -> f32,
) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let x_width = cell_width * x_count as f32;
    let z_width = cell_width * z_count as f32;

    let mut vertices = Vec::with_capacity((x_count + 1) * (z_count + 1));
    let mut x = -0.5 * x_width;
    for ix in 0..=x_count {
        let mut z = -0.5 * z_width;
        for iz in 0..=z_count {
            vertices.push(Vector::new(x, height(ix, iz), z));
            z += cell_width;
        }
        x += cell_width;
    }

    let mut indices = Vec::with_capacity(2 * x_count * z_count);
    for ix in 0..x_count {
        for iz in 0..z_count {
            let i1 = (iz + (z_count + 1) * ix) as u32;
            let i2 = i1 + 1;
            let i3 = i2 + (z_count as u32 + 1);
            let i4 = i3 - 1;
            indices.push([i1, i2, i3]);
            indices.push([i3, i4, i1]);
        }
    }
    (vertices, indices)
}

/// box3d `b3CreateGridMesh` (`src/mesh.c`): a flat `x_count * z_count` grid.
fn create_grid_mesh(
    x_count: usize,
    z_count: usize,
    cell_width: f32,
) -> (Vec<Vector>, Vec<[u32; 3]>) {
    grid_mesh(x_count, z_count, cell_width, |_, _| 0.0)
}

/// box3d `b3CreateTorusMesh` (`src/mesh.c`).
fn create_torus_mesh(
    radial_resolution: usize,
    tubular_resolution: usize,
    radius: f32,
    thickness: f32,
) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let two_pi = 2.0 * PI;
    let mut vertices = Vec::with_capacity(radial_resolution * tubular_resolution);
    for radial in 0..radial_resolution {
        for tubular in 0..tubular_resolution {
            let u = tubular as f32 / tubular_resolution as f32 * two_pi;
            let v = radial as f32 / radial_resolution as f32 * two_pi;
            let x = (radius + thickness * v.cos()) * u.cos();
            let y = (radius + thickness * v.cos()) * u.sin();
            let z = thickness * v.sin();
            vertices.push(Vector::new(x, y, z));
        }
    }

    let mut indices = Vec::with_capacity(2 * radial_resolution * tubular_resolution);
    for radial1 in 0..radial_resolution {
        let radial2 = (radial1 + 1) % radial_resolution;
        for tubular1 in 0..tubular_resolution {
            let tubular2 = (tubular1 + 1) % tubular_resolution;
            let i1 = (radial1 * tubular_resolution + tubular1) as u32;
            let i2 = (radial1 * tubular_resolution + tubular2) as u32;
            let i3 = (radial2 * tubular_resolution + tubular2) as u32;
            let i4 = (radial2 * tubular_resolution + tubular1) as u32;
            indices.push([i1, i2, i3]);
            indices.push([i3, i4, i1]);
        }
    }
    (vertices, indices)
}

// ── Human ragdoll (box3d/shared/human.c) ────────────────────────────────────

const BONE_COUNT: usize = 14;

/// Bone indices, matching box3d's `BoneId` enum order.
const PELVIS: usize = 0;
const SPINE_01: usize = 1;
const SPINE_02: usize = 2;
const SPINE_03: usize = 3;
const NECK: usize = 4;
const HEAD: usize = 5;
const THIGH_L: usize = 6;
const CALF_L: usize = 7;
const THIGH_R: usize = 8;
const CALF_R: usize = 9;
const UPPER_ARM_L: usize = 10;
const LOWER_ARM_L: usize = 11;
const UPPER_ARM_R: usize = 12;
const LOWER_ARM_R: usize = 13;

const DEG_TO_RAD: f32 = PI / 180.0;

#[derive(Clone, Copy)]
enum JointKind {
    Spherical,
    Revolute,
}

/// One bone's full description, transcribed from `human.c`.
struct BoneDef {
    parent: i32,
    /// Reference frame: body position offset + rotation (quaternion xyzw).
    ref_p: [f32; 3],
    ref_q: [f32; 4],
    /// Capsule endpoints + radius, in the bone's local frame.
    cap_a: [f32; 3],
    cap_b: [f32; 3],
    cap_r: f32,
    /// Joint to the parent (unused for the pelvis).
    kind: JointKind,
    frame_a_p: [f32; 3],
    frame_a_q: [f32; 4],
    frame_b_p: [f32; 3],
    frame_b_q: [f32; 4],
    /// Cone half-angle (spherical only), degrees.
    swing_deg: f32,
    /// Twist limit `[lo, hi]`, degrees.
    twist_deg: [f32; 2],
    /// Whether this bone's spine/thigh shape gets the same-human collision
    /// filter (box3d's negative `filter.groupIndex`).
    filtered: bool,
}

/// Handles to a spawned human's 14 bone bodies (index order matches `BoneId`).
struct HumanHandles {
    pub bones: [RigidBodyHandle; BONE_COUNT],
}

fn quat(q: [f32; 4]) -> Rotation {
    // box3d stores quaternions as {x, y, z, w}.
    Rotation::from_xyzw(q[0], q[1], q[2], q[3]).normalize()
}

/// Spawn one box3d human ragdoll with its pelvis reference position at
/// `position`. `friction_torque`/`hertz`/`damping` mirror the `CreateHuman`
/// parameters; `group_bit` selects the same-human collision filter bit (box3d's
/// per-human negative group index). Returns the bone body handles so callers can
/// recycle the ragdoll (used by the `rain` benchmark).
fn create_human(
    world: &mut PhysicsWorld,
    position: Vector,
    friction_torque: f32,
    hertz: f32,
    damping: f32,
    group_bit: u32,
) -> HumanHandles {
    let defs = human_bone_defs();
    let mut bones = [RigidBodyHandle::invalid(); BONE_COUNT];

    // Same-human collision filter: the three "filtered" shapes share membership
    // bit `group_bit` and exclude it from their filter, so they never collide
    // with each other (box3d's negative filter.groupIndex), while still
    // colliding with every other shape.
    let bit = Group::from_bits_truncate(1u32 << (group_bit % 24));
    let filtered_groups = InteractionGroups::new(bit, Group::ALL ^ bit, InteractionTestMode::And);

    for (i, def) in defs.iter().enumerate() {
        let pose = Pose::from_parts(position + Vector::from(def.ref_p), quat(def.ref_q));
        let body = RigidBodyBuilder::dynamic().pose(pose);
        let handle = world.insert_body(body);

        let mut collider = ColliderBuilder::capsule_from_endpoints(
            Vector::from(def.cap_a),
            Vector::from(def.cap_b),
            def.cap_r,
        )
        .density(1000.0);
        if def.filtered {
            collider = collider.collision_groups(filtered_groups);
        }
        world.insert_collider(collider, Some(handle));
        bones[i] = handle;
    }

    // Soft angular spring (box3d hertz/damping) mapped to an acceleration-based
    // motor: stiffness = w^2, damping = 2*zeta*w, w = 2*pi*hertz.
    let omega = 2.0 * PI * hertz;
    let stiffness = omega * omega;
    let motor_damping = 2.0 * damping * omega;
    let _ = friction_torque; // box3d clamps the motor by friction torque; omitted (approximate).

    for (i, def) in defs.iter().enumerate() {
        if def.parent < 0 {
            continue;
        }
        let parent = bones[def.parent as usize];
        let child = bones[i];

        let frame_a = Pose::from_parts(Vector::from(def.frame_a_p), quat(def.frame_a_q));
        let frame_b = Pose::from_parts(Vector::from(def.frame_b_p), quat(def.frame_b_q));
        let twist = [def.twist_deg[0] * DEG_TO_RAD, def.twist_deg[1] * DEG_TO_RAD];

        // Both joint types: lock the 3 linear axes (ball), twist about ANG_X,
        // swing about ANG_Y/ANG_Z (rapier convention), soft spring toward the
        // reference pose. Revolute additionally locks the two swing axes.
        let mut builder = GenericJointBuilder::new(
            JointAxesMask::LIN_X | JointAxesMask::LIN_Y | JointAxesMask::LIN_Z,
        )
        .local_frame1(frame_a)
        .local_frame2(frame_b)
        .contacts_enabled(false)
        .limits(JointAxis::AngX, twist)
        .motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
        .motor_position(JointAxis::AngX, 0.0, stiffness, motor_damping);

        match def.kind {
            JointKind::Spherical => {
                let swing = def.swing_deg * DEG_TO_RAD;
                builder = builder
                    .limits(JointAxis::AngY, [-swing, swing])
                    .limits(JointAxis::AngZ, [-swing, swing])
                    .motor_model(JointAxis::AngY, MotorModel::AccelerationBased)
                    .motor_position(JointAxis::AngY, 0.0, stiffness, motor_damping)
                    .motor_model(JointAxis::AngZ, MotorModel::AccelerationBased)
                    .motor_position(JointAxis::AngZ, 0.0, stiffness, motor_damping);
            }
            JointKind::Revolute => {
                builder = builder.locked_axes(
                    JointAxesMask::LIN_X
                        | JointAxesMask::LIN_Y
                        | JointAxesMask::LIN_Z
                        | JointAxesMask::ANG_Y
                        | JointAxesMask::ANG_Z,
                );
            }
        }

        world.insert_impulse_joint(parent, child, builder);
    }

    HumanHandles { bones }
}

#[rustfmt::skip]
fn human_bone_defs() -> [BoneDef; BONE_COUNT] {
    use JointKind::*;
    [
        // pelvis
        BoneDef { parent: -1,
            ref_p: [0.0, 0.932087, -0.051708], ref_q: [0.739169, 0.0, 0.0, 0.673520],
            cap_a: [0.07, 0.0, -0.08], cap_b: [-0.07, 0.0, -0.08], cap_r: 0.13,
            kind: Spherical, frame_a_p: [0.0; 3], frame_a_q: [0.0, 0.0, 0.0, 1.0],
            frame_b_p: [0.0; 3], frame_b_q: [0.0, 0.0, 0.0, 1.0], swing_deg: 0.0, twist_deg: [0.0, 0.0],
            filtered: false },
        // spine_01
        BoneDef { parent: PELVIS as i32,
            ref_p: [0.0, 1.113505, -0.03481], ref_q: [0.739973, 0.0, 0.0, 0.672637],
            cap_a: [0.06, 0.0, -0.052264], cap_b: [-0.06, 0.0, -0.052264], cap_r: 0.12,
            kind: Spherical,
            frame_a_p: [0.0, 0.0, -0.182204], frame_a_q: [-0.999999, 0.0, 0.0, 0.001194],
            frame_b_p: [0.0, 0.0, -0.007736], frame_b_q: [-1.0, 0.0, 0.0, 0.0],
            swing_deg: 25.0, twist_deg: [-15.0, 15.0], filtered: true },
        // spine_02
        BoneDef { parent: SPINE_01 as i32,
            ref_p: [0.0, 1.194336, -0.027087], ref_q: [0.703611, 0.0, 0.0, 0.710586],
            cap_a: [0.08, -0.015133, -0.091801], cap_b: [-0.08, -0.015133, -0.091801], cap_r: 0.10,
            kind: Spherical,
            frame_a_p: [0.0, 0.0, -0.088935], frame_a_q: [-0.998619, 0.0, 0.0, -0.052540],
            frame_b_p: [0.0, 0.0, -0.008199], frame_b_q: [-1.0, 0.0, 0.0, 0.0],
            swing_deg: 25.0, twist_deg: [-15.0, 15.0], filtered: false },
        // spine_03
        BoneDef { parent: SPINE_02 as i32,
            ref_p: [0.0, 1.31043, -0.028232], ref_q: [0.669856, 0.000001, -0.000001, 0.742491],
            cap_a: [0.11, -0.039753, -0.13], cap_b: [-0.11, -0.039753, -0.13], cap_r: 0.145,
            kind: Spherical,
            frame_a_p: [0.0, 0.0, -0.124298], frame_a_q: [-0.998921, 0.000001, -0.000001, -0.046434],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [-1.0, 0.0, -0.000001, 0.0],
            swing_deg: 15.0, twist_deg: [-10.0, 10.0], filtered: false },
        // neck
        BoneDef { parent: SPINE_03 as i32,
            ref_p: [0.0, 1.575582, -0.055837], ref_q: [0.879922, 0.0, 0.0, 0.475118],
            cap_a: [-0.000001, 0.0, -0.02], cap_b: [0.0, -0.005, -0.08], cap_r: 0.07,
            kind: Spherical,
            frame_a_p: [0.000001, -0.000259, -0.266585], frame_a_q: [-0.942192, -0.000001, 0.0, 0.335074],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [-1.0, 0.0, -0.000001, 0.0],
            swing_deg: 45.0, twist_deg: [-15.0, 15.0], filtered: false },
        // head
        BoneDef { parent: NECK as i32,
            ref_p: [0.0, 1.653348, -0.003241], ref_q: [0.750288, 0.0, 0.0, 0.661111],
            cap_a: [-0.000001, 0.016892, -0.05869], cap_b: [0.0, -0.003629, -0.115072], cap_r: 0.0975,
            kind: Spherical,
            frame_a_p: [0.0, 0.001321, -0.093873], frame_a_q: [-0.974301, 0.0, 0.0, -0.225251],
            frame_b_p: [0.0, 0.001268, -0.005104], frame_b_q: [-1.0, 0.0, 0.0, 0.0],
            swing_deg: 15.0, twist_deg: [-15.0, 15.0], filtered: false },
        // thigh_l
        BoneDef { parent: PELVIS as i32,
            ref_p: [0.090416, 0.986104, -0.035090], ref_q: [-0.703287, -0.070715, 0.053866, 0.705327],
            cap_a: [0.023719, 0.006008, -0.039068], cap_b: [-0.064492, -0.004664, -0.424718], cap_r: 0.09,
            kind: Spherical,
            frame_a_p: [0.05, 0.011537, -0.055325], frame_a_q: [-0.714896, -0.022305, -0.698361, -0.026790],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [-0.002064, 0.758987, 0.017046, 0.650880],
            swing_deg: 10.0, twist_deg: [-60.0, 40.0], filtered: true },
        // calf_l
        BoneDef { parent: THIGH_L as i32,
            ref_p: [0.101198, 0.527027, -0.037374], ref_q: [-0.653328, -0.066860, 0.058582, 0.751838],
            cap_a: [0.001778, 0.0, 0.009841], cap_b: [-0.078577, 0.014707, -0.41816], cap_r: 0.075,
            kind: Revolute,
            frame_a_p: [-0.069989, 0.000253, -0.453844], frame_a_q: [-0.000677, 0.760087, 0.105674, 0.641171],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [-0.044589, 0.765540, 0.053368, 0.639619],
            swing_deg: 0.0, twist_deg: [-5.0, 45.0], filtered: false },
        // thigh_r
        BoneDef { parent: PELVIS as i32,
            ref_p: [-0.090416, 0.986104, -0.03509], ref_q: [-0.703287, 0.070715, -0.053865, 0.705326],
            cap_a: [-0.023719, 0.006008, -0.039068], cap_b: [0.064492, -0.004664, -0.424718], cap_r: 0.09,
            kind: Spherical,
            frame_a_p: [-0.05, 0.011537, -0.055326], frame_a_q: [-0.039089, -0.714094, 0.043177, 0.697623],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [0.758805, -0.019886, -0.651012, -0.001759],
            swing_deg: 10.0, twist_deg: [-30.0, 60.0], filtered: true },
        // calf_r
        BoneDef { parent: THIGH_R as i32,
            ref_p: [-0.101198, 0.527027, -0.037373], ref_q: [-0.653327, 0.06686, -0.058582, 0.751839],
            cap_a: [-0.001820, 0.0, 0.010071], cap_b: [0.077883, 0.014825, -0.418047], cap_r: 0.075,
            kind: Revolute,
            frame_a_p: [0.069988, 0.000253, -0.453844], frame_a_q: [0.760086, -0.000675, -0.641171, -0.105676],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [0.765540, -0.044589, -0.639619, -0.053368],
            swing_deg: 0.0, twist_deg: [-45.0, 5.0], filtered: false },
        // upper_arm_l
        BoneDef { parent: SPINE_03 as i32,
            ref_p: [0.20378, 1.484275, -0.115897], ref_q: [0.143082, 0.695980, -0.690130, 0.13733],
            cap_a: [0.0, 0.0, 0.0], cap_b: [-0.091118, 0.037775, 0.229719], cap_r: 0.075,
            kind: Spherical,
            frame_a_p: [0.203780, -0.069369, -0.181921], frame_a_q: [-0.278486, 0.445600, -0.097014, 0.845266],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [-0.201396, -0.001586, 0.901850, 0.382234],
            swing_deg: 60.0, twist_deg: [-5.0, 5.0], filtered: false },
        // lower_arm_l
        BoneDef { parent: UPPER_ARM_L as i32,
            ref_p: [0.305614, 1.242908, -0.117599], ref_q: [0.165048, 0.563437, -0.802002, 0.109959],
            cap_a: [0.0, 0.0, 0.0], cap_b: [-0.142406, 0.039392, 0.261092], cap_r: 0.05,
            kind: Revolute,
            frame_a_p: [-0.095482, 0.039584, 0.240723], frame_a_q: [0.512487, -0.180629, 0.839474, 0.003742],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [0.503803, -0.029831, 0.858168, 0.094017],
            swing_deg: 0.0, twist_deg: [-5.0, 60.0], filtered: false },
        // upper_arm_r
        BoneDef { parent: SPINE_03 as i32,
            ref_p: [-0.20378, 1.484276, -0.115899], ref_q: [0.143083, -0.695978, 0.690132, 0.137329],
            cap_a: [0.0, 0.0, 0.0], cap_b: [0.091118, 0.037775, 0.229718], cap_r: 0.075,
            kind: Spherical,
            frame_a_p: [-0.203779, -0.069371, -0.181922], frame_a_q: [-0.253621, -0.414842, 0.106962, 0.867261],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [-0.201397, 0.001587, -0.901850, 0.382233],
            swing_deg: 60.0, twist_deg: [-5.0, 5.0], filtered: false },
        // lower_arm_r
        BoneDef { parent: UPPER_ARM_R as i32,
            ref_p: [-0.305614, 1.242907, -0.117599], ref_q: [0.165048, -0.563437, 0.802002, 0.109959],
            cap_a: [0.0, 0.0, 0.0], cap_b: [0.142406, 0.039392, 0.261092], cap_r: 0.05,
            kind: Revolute,
            frame_a_p: [0.095484, 0.039585, 0.240723], frame_a_q: [-0.180627, 0.512487, -0.003744, -0.839474],
            frame_b_p: [0.0, 0.0, 0.0], frame_b_q: [-0.029831, 0.503803, -0.094017, -0.858169],
            swing_deg: 0.0, twist_deg: [-60.0, 5.0], filtered: false },
    ]
}
