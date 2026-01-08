#![allow(dead_code)]
#![allow(clippy::type_complexity)]

use rapier_testbed2d::{Example, TestbedApp};

mod add_remove2;
mod ccd2;
mod character_controller2;
mod collision_groups2;
mod convex_polygons2;
mod damping2;
mod debug_box_ball2;
mod debug_compression2;
mod debug_intersection2;
mod debug_total_overlap2;
mod debug_vertical_column2;
mod drum2;
mod heightfield2;
mod inv_pyramid2;
mod inverse_kinematics2;
mod joint_motor_position2;
mod joints2;
mod locked_rotations2;
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
#[cfg(not(target_arch = "wasm32"))]
mod trimesh2;
mod voxels2;

mod utils;

#[kiss3d::main]
pub async fn main() {
    const COLLISIONS: &str = "Collisions";
    const DYNAMICS: &str = "Dynamics";
    const COMPLEX: &str = "Complex Shapes";
    const JOINTS: &str = "Joints";
    const CONTROLS: &str = "Controls";
    const DEBUG: &str = "Debug";
    const S2D: &str = "Inspired by Solver 2D";

    let mut builders: Vec<Example> = vec![
        // ── Demos ──────────────────────────────────────────────────────────
        Example::new(COLLISIONS, "Add remove", add_remove2::init_world),
        Example::new(COLLISIONS, "Drum", drum2::init_world),
        Example::new(COLLISIONS, "Inv pyramid", inv_pyramid2::init_world),
        Example::new(COLLISIONS, "Platform", platform2::init_world),
        Example::new(COLLISIONS, "Pyramid", pyramid2::init_world),
        Example::new(COLLISIONS, "Sensor", sensor2::init_world),
        Example::new(COLLISIONS, "Convex polygons", convex_polygons2::init_world),
        Example::new(COLLISIONS, "Heightfield", heightfield2::init_world),
        Example::new(COLLISIONS, "Polyline", polyline2::init_world),
        #[cfg(not(target_arch = "wasm32"))]
        Example::new(COLLISIONS, "Trimesh", trimesh2::init_world),
        Example::new(COLLISIONS, "Voxels", voxels2::init_world),
        Example::new(
            COLLISIONS,
            "Collision groups",
            collision_groups2::init_world,
        ),
        Example::new(
            COLLISIONS,
            "One-way platforms",
            one_way_platforms2::init_world,
        ),
        // ── Dynamics ──────────────────────────────────────────────────────────
        Example::new(DYNAMICS, "Locked rotations", locked_rotations2::init_world),
        Example::new(DYNAMICS, "Restitution", restitution2::init_world),
        Example::new(DYNAMICS, "Damping", damping2::init_world),
        Example::new(DYNAMICS, "CCD", ccd2::init_world),
        // ── Joints ─────────────────────────────────────────────────────────
        Example::new(JOINTS, "Joints", joints2::init_world),
        Example::new(JOINTS, "Rope Joints", rope_joints2::init_world),
        Example::new(JOINTS, "Pin Slot Joint", pin_slot_joint2::init_world),
        Example::new(
            JOINTS,
            "Joint motor position",
            joint_motor_position2::init_world,
        ),
        Example::new(
            JOINTS,
            "Inverse kinematics",
            inverse_kinematics2::init_world,
        ),
        // ── Characters ─────────────────────────────────────────────────────
        Example::new(
            CONTROLS,
            "Character controller",
            character_controller2::init_world,
        ),
        // ── Debug ──────────────────────────────────────────────────────────
        Example::new(DEBUG, "Box ball", debug_box_ball2::init_world),
        Example::new(DEBUG, "Compression", debug_compression2::init_world),
        Example::new(DEBUG, "Intersection", debug_intersection2::init_world),
        Example::new(DEBUG, "Total overlap", debug_total_overlap2::init_world),
        Example::new(DEBUG, "Vertical column", debug_vertical_column2::init_world),
        // ── Demos inspired by Solver2D ───────────────────────────────────────────────────
        Example::new(S2D, "High mass ratio 1", s2d_high_mass_ratio_1::init_world),
        Example::new(S2D, "High mass ratio 2", s2d_high_mass_ratio_2::init_world),
        Example::new(S2D, "High mass ratio 3", s2d_high_mass_ratio_3::init_world),
        Example::new(S2D, "Confined", s2d_confined::init_world),
        Example::new(S2D, "Pyramid", s2d_pyramid::init_world),
        Example::new(S2D, "Card house", s2d_card_house::init_world),
        Example::new(S2D, "Arch", s2d_arch::init_world),
        Example::new(S2D, "Bridge", s2d_bridge::init_world),
        Example::new(S2D, "Ball and chain", s2d_ball_and_chain::init_world),
        Example::new(S2D, "Joint grid", s2d_joint_grid::init_world),
        Example::new(S2D, "Far pyramid", s2d_far_pyramid::init_world),
    ];
    let mut benches = stress_tests::builders();
    builders.append(&mut benches);

    let testbed = TestbedApp::from_builders(builders);
    testbed.run().await
}
