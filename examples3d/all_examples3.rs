#![allow(dead_code)]
#![allow(clippy::type_complexity)]

use rapier_testbed3d::{ExampleEntry, TestbedViewer};
use std::future::Future;
use std::pin::Pin;

mod utils;

mod ccd3;
mod character_controller3;
mod collision_groups3;
mod compound3;
mod convex_decomposition3;
mod convex_polyhedron3;
mod damping3;
mod debug_add_remove_collider3;
mod debug_articulations3;
mod debug_balls3;
mod debug_big_colliders3;
mod debug_boxes3;
mod debug_chain_high_mass_ratio3;
mod debug_cube_high_mass_ratio3;
mod debug_cylinder3;
mod debug_deserialize3;
mod debug_disabled3;
mod debug_dynamic_collider_add3;
mod debug_friction3;
mod debug_infinite_fall3;
mod debug_internal_edges3;
mod debug_long_chain3;
mod debug_multibody_ang_motor_pos3;
mod debug_pop3;
mod debug_prismatic3;
mod debug_rollback3;
mod debug_shape_modification3;
mod debug_sleeping_kinematic3;
mod debug_thin_cube_on_mesh3;
mod debug_triangle3;
mod debug_trimesh3;
mod debug_two_cubes3;
mod domino3;
mod dynamic_trimesh3;
mod fountain3;
mod gyroscopic3;
mod heightfield3;
mod inverse_kinematics3;
mod joint_motor_position3;
mod joints3;
mod keva3;
mod locked_rotations3;
mod mjcf3;
mod mujoco_menagerie3;
mod newton_cradle3;
mod one_way_platforms3;
mod platform3;
mod primitives3;
mod restitution3;
mod rope_joints3;
mod sensor3;
mod spring_joints3;
mod stress_tests;
mod trimesh3;
mod urdf3;
mod vehicle_controller3;
mod vehicle_joints3;
mod voxels3;

/// A registered example: a fn pointer running the example's owned loop.
/// (A non-capturing closure coerces to this higher-ranked fn pointer.)
type ExampleFn =
    for<'a> fn(&'a mut TestbedViewer) -> Pin<Box<dyn Future<Output = anyhow::Result<()>> + 'a>>;

/// `(group, name, run-fn)` -> `(ExampleEntry, ExampleFn)`.
macro_rules! examples {
    ($($group:expr, $name:expr, $run:path);* $(;)?) => {
        vec![ $( (ExampleEntry::new($group, $name), (|v| Box::pin($run(v))) as ExampleFn) ),* ]
    };
}

#[kiss3d::main]
pub async fn main() {
    const COLLISIONS: &str = "Collisions";
    const DYNAMICS: &str = "Dynamics";
    const JOINTS: &str = "Joints";
    const CONTROLS: &str = "Controls";
    const DEBUG: &str = "Debug";
    const ROBOTICS: &str = "Robotics";
    const STRESS: &str = "Stress tests";

    let examples: Vec<(ExampleEntry, ExampleFn)> = examples![
        // ── Collisions ──────────────────────────────────────────────────────
        COLLISIONS, "Fountain", fountain3::run;
        COLLISIONS, "Primitives", primitives3::run;
        COLLISIONS, "Keva tower", keva3::run;
        COLLISIONS, "Newton cradle", newton_cradle3::run;
        COLLISIONS, "Domino", domino3::run;
        COLLISIONS, "Platform", platform3::run;
        COLLISIONS, "Sensor", sensor3::run;
        COLLISIONS, "Compound", compound3::run;
        COLLISIONS, "Convex decomposition", convex_decomposition3::run;
        COLLISIONS, "Convex polyhedron", convex_polyhedron3::run;
        COLLISIONS, "TriMesh", trimesh3::run;
        COLLISIONS, "Dynamic trimeshes", dynamic_trimesh3::run;
        COLLISIONS, "Heightfield", heightfield3::run;
        COLLISIONS, "Voxels", voxels3::run;
        COLLISIONS, "Collision groups", collision_groups3::run;
        COLLISIONS, "One-way platforms", one_way_platforms3::run;
        // ── Dynamics ────────────────────────────────────────────────────────
        DYNAMICS, "Locked rotations", locked_rotations3::run;
        DYNAMICS, "Restitution", restitution3::run;
        DYNAMICS, "Damping", damping3::run;
        DYNAMICS, "Gyroscopic", gyroscopic3::run;
        DYNAMICS, "CCD", ccd3::run;
        // ── Joints ──────────────────────────────────────────────────────────
        JOINTS, "Impulse Joints", joints3::run_impulse_joints;
        JOINTS, "Multibody Joints", joints3::run_multibody_joints;
        JOINTS, "Rope Joints", rope_joints3::run;
        JOINTS, "Spring Joints", spring_joints3::run;
        JOINTS, "Joint Motor Position", joint_motor_position3::run;
        JOINTS, "Inverse kinematics", inverse_kinematics3::run;
        // ── Controls ────────────────────────────────────────────────────────
        CONTROLS, "Character controller", character_controller3::run;
        CONTROLS, "Vehicle controller", vehicle_controller3::run;
        CONTROLS, "Vehicle joints", vehicle_joints3::run;
        // ── Robotics ────────────────────────────────────────────────────────
        ROBOTICS, "URDF", urdf3::run;
        ROBOTICS, "MJCF", mjcf3::run;
        ROBOTICS, "Mujoco Menagerie", mujoco_menagerie3::run;
        // ── Debug ───────────────────────────────────────────────────────────
        DEBUG, "Multibody joints", debug_articulations3::run;
        DEBUG, "Add/rm collider", debug_add_remove_collider3::run;
        DEBUG, "Big colliders", debug_big_colliders3::run;
        DEBUG, "Boxes", debug_boxes3::run;
        DEBUG, "Balls", debug_balls3::run;
        DEBUG, "Disabled", debug_disabled3::run;
        DEBUG, "Two cubes", debug_two_cubes3::run;
        DEBUG, "Pop", debug_pop3::run;
        DEBUG, "Dyn. collider add", debug_dynamic_collider_add3::run;
        DEBUG, "Friction", debug_friction3::run;
        DEBUG, "Internal edges", debug_internal_edges3::run;
        DEBUG, "Long chain", debug_long_chain3::run;
        DEBUG, "High mass ratio: chain", debug_chain_high_mass_ratio3::run;
        DEBUG, "High mass ratio: cube", debug_cube_high_mass_ratio3::run;
        DEBUG, "Triangle", debug_triangle3::run;
        DEBUG, "Trimesh", debug_trimesh3::run;
        DEBUG, "Thin cube", debug_thin_cube_on_mesh3::run;
        DEBUG, "Cylinder", debug_cylinder3::run;
        DEBUG, "Infinite fall", debug_infinite_fall3::run;
        DEBUG, "Prismatic", debug_prismatic3::run;
        DEBUG, "Rollback", debug_rollback3::run;
        DEBUG, "Shape modification", debug_shape_modification3::run;
        DEBUG, "Sleeping kinematics", debug_sleeping_kinematic3::run;
        DEBUG, "Deserialize", debug_deserialize3::run;
        DEBUG, "Multibody ang. motor pos.", debug_multibody_ang_motor_pos3::run;
        // ── Stress tests ────────────────────────────────────────────────────
        STRESS, "Balls", stress_tests::balls3::run;
        STRESS, "Boxes", stress_tests::boxes3::run;
        STRESS, "Capsules", stress_tests::capsules3::run;
        STRESS, "CCD", stress_tests::ccd3::run;
        STRESS, "Compound", stress_tests::compound3::run;
        STRESS, "Convex polyhedron", stress_tests::convex_polyhedron3::run;
        STRESS, "Many kinematics", stress_tests::many_kinematics3::run;
        STRESS, "Many static", stress_tests::many_static3::run;
        STRESS, "Many sleep", stress_tests::many_sleep3::run;
        STRESS, "Heightfield", stress_tests::heightfield3::run;
        STRESS, "Stacks", stress_tests::stacks3::run;
        STRESS, "Pyramid", stress_tests::pyramid3::run;
        STRESS, "Trimesh", stress_tests::trimesh3::run;
        STRESS, "ImpulseJoint ball", stress_tests::joint_ball3::run;
        STRESS, "ImpulseJoint fixed", stress_tests::joint_fixed3::run;
        STRESS, "ImpulseJoint revolute", stress_tests::joint_revolute3::run;
        STRESS, "ImpulseJoint prismatic", stress_tests::joint_prismatic3::run;
        STRESS, "Many pyramids", stress_tests::many_pyramids3::run;
        STRESS, "Keva tower", stress_tests::keva3::run;
        STRESS, "Ray cast", stress_tests::ray_cast3::run;
    ];

    let (entries, run_fns): (Vec<_>, Vec<ExampleFn>) = examples.into_iter().unzip();
    let mut viewer = TestbedViewer::new(entries).await;

    // The example owns its physics state and render loop; this outer loop just
    // (re)dispatches the example the UI has selected, nexus-style.
    loop {
        viewer.clear_scene();
        let idx = viewer.selected();
        if let Err(e) = run_fns[idx](&mut viewer).await {
            eprintln!("example #{idx} failed: {e:?}");
        }
        if viewer.quitting() {
            break;
        }
    }
}
