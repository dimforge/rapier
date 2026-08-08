#![allow(dead_code)]
#![allow(clippy::type_complexity)]

use rapier_testbed2d::{ExampleEntry, TestbedViewer};
use std::future::Future;
use std::pin::Pin;

mod add_remove2;
mod b2d_compounds;
mod b2d_joint_grid;
mod b2d_junkyard;
mod b2d_large_pyramid;
mod b2d_many_pyramids;
mod b2d_rain;
mod b2d_smash;
mod b2d_spinner;
mod b2d_tumbler;
mod b2d_washer;
mod ccd2;
mod character_controller2;
mod collision_groups2;
mod convex_polygons2;
mod damping2;
mod debug_angular_limits2;
mod debug_box_ball2;
mod debug_compression2;
mod debug_intersection2;
mod debug_many_colliders2;
mod debug_total_overlap2;
mod debug_vertical_column2;
mod drum2;
mod heightfield2;
mod inv_pyramid2;
mod inverse_kinematics2;
mod joint_motor_position2;
mod joints2;
mod locked_rotations2;
mod multi_pendulum2;
mod one_way_platforms2;
mod pin_slot_joint2;
mod platform2;
mod polyline2;
mod pyramid2;
mod restitution2;
mod rope_joints2;
mod s2d_arch;
mod s2d_ball_and_chain;
mod s2d_bridge;
mod s2d_card_house;
mod s2d_confined;
mod s2d_far_pyramid;
mod s2d_high_mass_ratio_1;
mod s2d_high_mass_ratio_2;
mod s2d_high_mass_ratio_3;
mod s2d_joint_grid;
mod s2d_pyramid;
mod sensor2;
mod stress_tests;
mod trimesh2;
mod voxels2;

mod utils;

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
    const S2D: &str = "Inspired by Solver 2D";
    const STRESS: &str = "Stress tests";
    const B2D: &str = "Box2D benchmarks";

    let examples: Vec<(ExampleEntry, ExampleFn)> = examples![
        // ── Collisions ──────────────────────────────────────────────────────
        COLLISIONS, "Add remove", add_remove2::run;
        COLLISIONS, "Drum", drum2::run;
        COLLISIONS, "Inv pyramid", inv_pyramid2::run;
        COLLISIONS, "Platform", platform2::run;
        COLLISIONS, "Pyramid", pyramid2::run;
        COLLISIONS, "Sensor", sensor2::run;
        COLLISIONS, "Convex polygons", convex_polygons2::run;
        COLLISIONS, "Heightfield", heightfield2::run;
        COLLISIONS, "Polyline", polyline2::run;
        COLLISIONS, "Trimesh", trimesh2::run;
        COLLISIONS, "Voxels", voxels2::run;
        COLLISIONS, "Collision groups", collision_groups2::run;
        COLLISIONS, "One-way platforms", one_way_platforms2::run;
        // ── Dynamics ────────────────────────────────────────────────────────
        DYNAMICS, "Locked rotations", locked_rotations2::run;
        DYNAMICS, "Restitution", restitution2::run;
        DYNAMICS, "Damping", damping2::run;
        DYNAMICS, "CCD", ccd2::run;
        // ── Joints ──────────────────────────────────────────────────────────
        JOINTS, "Joints", joints2::run;
        JOINTS, "Rope Joints", rope_joints2::run;
        JOINTS, "Pin Slot Joint", pin_slot_joint2::run;
        JOINTS, "Joint motor position", joint_motor_position2::run;
        JOINTS, "Inverse kinematics", inverse_kinematics2::run;
        JOINTS, "Multi Pendulum", multi_pendulum2::run;
        // ── Controls ────────────────────────────────────────────────────────
        CONTROLS, "Character controller", character_controller2::run;
        // ── Debug ───────────────────────────────────────────────────────────
        DEBUG, "Angular limits", debug_angular_limits2::run;
        DEBUG, "Box ball", debug_box_ball2::run;
        DEBUG, "Compression", debug_compression2::run;
        DEBUG, "Intersection", debug_intersection2::run;
        DEBUG, "Many colliders", debug_many_colliders2::run;
        DEBUG, "Total overlap", debug_total_overlap2::run;
        DEBUG, "Vertical column", debug_vertical_column2::run;
        // ── Inspired by Solver2D ────────────────────────────────────────────
        S2D, "High mass ratio 1", s2d_high_mass_ratio_1::run;
        S2D, "High mass ratio 2", s2d_high_mass_ratio_2::run;
        S2D, "High mass ratio 3", s2d_high_mass_ratio_3::run;
        S2D, "Confined", s2d_confined::run;
        S2D, "Pyramid", s2d_pyramid::run;
        S2D, "Card house", s2d_card_house::run;
        S2D, "Arch", s2d_arch::run;
        S2D, "Bridge", s2d_bridge::run;
        S2D, "Ball and chain", s2d_ball_and_chain::run;
        S2D, "Joint grid", s2d_joint_grid::run;
        S2D, "Far pyramid", s2d_far_pyramid::run;
        // ── Box2D benchmarks (ports of box2d/benchmark) ─────────────────────
        B2D, "Compounds", b2d_compounds::run;
        B2D, "Joint grid", b2d_joint_grid::run;
        B2D, "Junkyard", b2d_junkyard::run;
        B2D, "Large pyramid", b2d_large_pyramid::run;
        B2D, "Many pyramids", b2d_many_pyramids::run;
        B2D, "Rain", b2d_rain::run;
        B2D, "Smash", b2d_smash::run;
        B2D, "Spinner", b2d_spinner::run;
        B2D, "Tumbler", b2d_tumbler::run;
        B2D, "Washer", b2d_washer::run;
        // ── Stress tests ────────────────────────────────────────────────────
        STRESS, "Balls", stress_tests::balls2::run;
        STRESS, "Boxes", stress_tests::boxes2::run;
        STRESS, "Capsules", stress_tests::capsules2::run;
        STRESS, "Convex polygons", stress_tests::convex_polygons2::run;
        STRESS, "Heightfield", stress_tests::heightfield2::run;
        STRESS, "Large pyramids", stress_tests::large_pyramids2::run;
        STRESS, "Many pyramids", stress_tests::many_pyramids2::run;
        STRESS, "Pyramid", stress_tests::pyramid2::run;
        STRESS, "Ragdoll piles", stress_tests::ragdolls2::run;
        STRESS, "Ropes", stress_tests::ropes2::run;
        STRESS, "Verticals stacks", stress_tests::vertical_stacks2::run;
        STRESS, "(Stress test) joint ball", stress_tests::joint_ball2::run;
        STRESS, "(Stress test) joint fixed", stress_tests::joint_fixed2::run;
        STRESS, "(Stress test) joint prismatic", stress_tests::joint_prismatic2::run;
    ];

    let (entries, run_fns): (Vec<_>, Vec<ExampleFn>) = examples.into_iter().unzip();
    let mut viewer = TestbedViewer::new(entries).await;

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
