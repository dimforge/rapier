#![allow(dead_code)]

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use rapier_testbed3d::{Testbed, TestbedApp};
use std::cmp::Ordering;

mod utils;

mod ccd3;
mod collision_groups3;
mod compound3;
mod convex_decomposition3;
mod convex_polyhedron3;
mod damping3;
mod debug_add_remove_collider3;
mod debug_articulations3;
mod debug_big_colliders3;
mod debug_boxes3;
mod debug_cylinder3;
mod debug_deserialize3;
mod debug_dynamic_collider_add3;
mod debug_friction3;
mod debug_infinite_fall3;
mod debug_pop3;
mod debug_prismatic3;
mod debug_rollback3;
mod debug_shape_modification3;
mod debug_thin_cube_on_mesh3;
mod debug_triangle3;
mod debug_trimesh3;
mod domino3;
mod dynamic_trimesh3;
mod fountain3;
mod heightfield3;
mod joints3;
// mod joints3;
mod character_controller3;
mod debug_chain_high_mass_ratio3;
mod debug_cube_high_mass_ratio3;
mod debug_internal_edges3;
mod debug_long_chain3;
mod debug_multibody_ang_motor_pos3;
mod inverse_kinematics3;
mod joint_motor_position3;
mod keva3;
mod locked_rotations3;
mod newton_cradle3;
mod one_way_platforms3;
mod platform3;
mod primitives3;
mod restitution3;
mod rope_joints3;
mod sensor3;
mod spring_joints3;
mod trimesh3;
mod urdf3;
mod vehicle_controller3;
mod vehicle_joints3;

#[cfg_attr(target_arch = "wasm32", wasm_bindgen(start))]
pub fn main() {
    let mut builders: Vec<(_, fn(&mut Testbed))> = vec![
        ("Character controller", character_controller3::init_world),
        ("Fountain", fountain3::init_world),
        ("Primitives", primitives3::init_world),
        ("Multibody joints", joints3::init_world_with_articulations),
        ("CCD", ccd3::init_world),
        ("Collision groups", collision_groups3::init_world),
        ("Compound", compound3::init_world),
        ("Convex decomposition", convex_decomposition3::init_world),
        ("Convex polyhedron", convex_polyhedron3::init_world),
        ("Damping", damping3::init_world),
        ("Domino", domino3::init_world),
        ("Dynamic trimeshes", dynamic_trimesh3::init_world),
        ("Heightfield", heightfield3::init_world),
        ("Impulse Joints", joints3::init_world_with_joints),
        ("Inverse kinematics", inverse_kinematics3::init_world),
        ("Joint Motor Position", joint_motor_position3::init_world),
        ("Locked rotations", locked_rotations3::init_world),
        ("One-way platforms", one_way_platforms3::init_world),
        ("Platform", platform3::init_world),
        ("Restitution", restitution3::init_world),
        ("Rope Joints", rope_joints3::init_world),
        ("Sensor", sensor3::init_world),
        ("Spring Joints", spring_joints3::init_world),
        ("TriMesh", trimesh3::init_world),
        ("Urdf", urdf3::init_world),
        ("Vehicle controller", vehicle_controller3::init_world),
        ("Vehicle joints", vehicle_joints3::init_world),
        ("Keva tower", keva3::init_world),
        ("Newton cradle", newton_cradle3::init_world),
        ("(Debug) multibody_joints", debug_articulations3::init_world),
        (
            "(Debug) add/rm collider",
            debug_add_remove_collider3::init_world,
        ),
        ("(Debug) big colliders", debug_big_colliders3::init_world),
        ("(Debug) boxes", debug_boxes3::init_world),
        ("(Debug) pop", debug_pop3::init_world),
        (
            "(Debug) dyn. coll. add",
            debug_dynamic_collider_add3::init_world,
        ),
        ("(Debug) friction", debug_friction3::init_world),
        ("(Debug) internal edges", debug_internal_edges3::init_world),
        ("(Debug) long chain", debug_long_chain3::init_world),
        (
            "(Debug) high mass ratio: chain",
            debug_chain_high_mass_ratio3::init_world,
        ),
        (
            "(Debug) high mass ratio: cube",
            debug_cube_high_mass_ratio3::init_world,
        ),
        ("(Debug) triangle", debug_triangle3::init_world),
        ("(Debug) trimesh", debug_trimesh3::init_world),
        ("(Debug) thin cube", debug_thin_cube_on_mesh3::init_world),
        ("(Debug) cylinder", debug_cylinder3::init_world),
        ("(Debug) infinite fall", debug_infinite_fall3::init_world),
        ("(Debug) prismatic", debug_prismatic3::init_world),
        ("(Debug) rollback", debug_rollback3::init_world),
        (
            "(Debug) shape modification",
            debug_shape_modification3::init_world,
        ),
        ("(Debug) deserialize", debug_deserialize3::init_world),
        (
            "(Debug) multibody ang. motor pos.",
            debug_multibody_ang_motor_pos3::init_world,
        ),
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
