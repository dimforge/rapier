#![allow(dead_code)]
#![allow(clippy::type_complexity)]

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use rapier_testbed2d::{Testbed, TestbedApp};
use std::cmp::Ordering;

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
mod trimesh2;
mod voxels2;

mod utils;

#[cfg_attr(target_arch = "wasm32", wasm_bindgen(start))]
pub fn main() {
    let mut builders: Vec<(_, fn(&mut Testbed))> = vec![
        ("Add remove", add_remove2::init_world),
        ("CCD", ccd2::init_world),
        ("Character controller", character_controller2::init_world),
        ("Collision groups", collision_groups2::init_world),
        ("Convex polygons", convex_polygons2::init_world),
        ("Damping", damping2::init_world),
        ("Drum", drum2::init_world),
        ("Heightfield", heightfield2::init_world),
        ("Inverse kinematics", inverse_kinematics2::init_world),
        ("Inv pyramid", inv_pyramid2::init_world),
        ("Joints", joints2::init_world),
        ("Locked rotations", locked_rotations2::init_world),
        ("One-way platforms", one_way_platforms2::init_world),
        ("Platform", platform2::init_world),
        ("Polyline", polyline2::init_world),
        ("Pyramid", pyramid2::init_world),
        ("Restitution", restitution2::init_world),
        ("Rope Joints", rope_joints2::init_world),
        ("Sensor", sensor2::init_world),
        ("Trimesh", trimesh2::init_world),
        ("Voxels", voxels2::init_world),
        ("Joint motor position", joint_motor_position2::init_world),
        ("(Debug) box ball", debug_box_ball2::init_world),
        ("(Debug) compression", debug_compression2::init_world),
        ("(Debug) intersection", debug_intersection2::init_world),
        ("(Debug) total overlap", debug_total_overlap2::init_world),
        (
            "(Debug) vertical column",
            debug_vertical_column2::init_world,
        ),
        ("(s2d) high mass ratio 1", s2d_high_mass_ratio_1::init_world),
        ("(s2d) high mass ratio 2", s2d_high_mass_ratio_2::init_world),
        ("(s2d) high mass ratio 3", s2d_high_mass_ratio_3::init_world),
        ("(s2d) confined", s2d_confined::init_world),
        ("(s2d) pyramid", s2d_pyramid::init_world),
        ("(s2d) card house", s2d_card_house::init_world),
        ("(s2d) arch", s2d_arch::init_world),
        ("(s2d) bridge", s2d_bridge::init_world),
        ("(s2d) ball and chain", s2d_ball_and_chain::init_world),
        ("(s2d) joint grid", s2d_joint_grid::init_world),
        ("(s2d) far pyramid", s2d_far_pyramid::init_world),
    ];

    // Lexicographic sort, with stress tests moved at the end of the list.
    builders.sort_by(|a, b| match (a.0.starts_with('('), b.0.starts_with('(')) {
        (true, true) | (false, false) => a.0.cmp(b.0),
        (true, false) => Ordering::Greater,
        (false, true) => Ordering::Less,
    });

    let testbed = TestbedApp::from_builders(builders);

    testbed.run()
}
