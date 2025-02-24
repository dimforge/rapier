#![allow(dead_code)]

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use rapier_testbed3d::{Testbed, TestbedApp};
use std::cmp::Ordering;

mod ccd3;
mod collision_groups3;
mod compound3;
mod convex_decomposition3;
mod convex_polyhedron3;
mod damping3;
mod debug_add_remove_collider3;
mod debug_big_colliders3;
mod debug_boxes3;
mod debug_cylinder3;
mod debug_dynamic_collider_add3;
mod debug_friction3;
mod debug_infinite_fall3;
mod debug_prismatic3;
mod debug_rollback3;
mod debug_shape_modification3;
mod debug_triangle3;
mod debug_trimesh3;
mod domino3;
mod fountain3;
mod heightfield3;
mod joints3;
mod keva3;
mod locked_rotations3;
mod one_way_platforms3;
mod platform3;
mod primitives3;
mod restitution3;
mod sensor3;
mod trimesh3;

fn demo_name_from_command_line() -> Option<String> {
    let mut args = std::env::args();

    while let Some(arg) = args.next() {
        if &arg[..] == "--example" {
            return args.next();
        }
    }

    None
}

#[cfg(target_arch = "wasm32")]
fn demo_name_from_url() -> Option<String> {
    None
    //    let window = stdweb::web::window();
    //    let hash = window.location()?.search().ok()?;
    //    if hash.len() > 0 {
    //        Some(hash[1..].to_string())
    //    } else {
    //        None
    //    }
}

#[cfg(not(target_arch = "wasm32"))]
fn demo_name_from_url() -> Option<String> {
    None
}

#[cfg_attr(target_arch = "wasm32", wasm_bindgen(start))]
pub fn main() {
    let demo = demo_name_from_command_line()
        .or_else(|| demo_name_from_url())
        .unwrap_or(String::new())
        .to_camel_case();

    let mut builders: Vec<(_, fn(&mut Testbed))> = vec![
        ("Fountain", fountain3::init_world),
        ("Primitives", primitives3::init_world),
        ("CCD", ccd3::init_world),
        ("Collision groups", collision_groups3::init_world),
        ("Compound", compound3::init_world),
        ("Convex decomposition", convex_decomposition3::init_world),
        ("Convex polyhedron", convex_polyhedron3::init_world),
        ("Damping", damping3::init_world),
        ("Domino", domino3::init_world),
        ("Heightfield", heightfield3::init_world),
        ("Joints", joints3::init_world),
        ("Locked rotations", locked_rotations3::init_world),
        ("One-way platforms", one_way_platforms3::init_world),
        ("Platform", platform3::init_world),
        ("Restitution", restitution3::init_world),
        ("Sensor", sensor3::init_world),
        ("TriMesh", trimesh3::init_world),
        ("Keva tower", keva3::init_world),
        (
            "(Debug) add/rm collider",
            debug_add_remove_collider3::init_world,
        ),
        ("(Debug) big colliders", debug_big_colliders3::init_world),
        ("(Debug) boxes", debug_boxes3::init_world),
        (
            "(Debug) dyn. coll. add",
            debug_dynamic_collider_add3::init_world,
        ),
        ("(Debug) friction", debug_friction3::init_world),
        ("(Debug) triangle", debug_triangle3::init_world),
        ("(Debug) trimesh", debug_trimesh3::init_world),
        ("(Debug) cylinder", debug_cylinder3::init_world),
        ("(Debug) infinite fall", debug_infinite_fall3::init_world),
        ("(Debug) prismatic", debug_prismatic3::init_world),
        ("(Debug) rollback", debug_rollback3::init_world),
        (
            "(Debug) shape modification",
            debug_shape_modification3::init_world,
        ),
    ];

    // Lexicographic sort, with stress tests moved at the end of the list.
    builders.sort_by(|a, b| match (a.0.starts_with("("), b.0.starts_with("(")) {
        (true, true) | (false, false) => a.0.cmp(b.0),
        (true, false) => Ordering::Greater,
        (false, true) => Ordering::Less,
    });

    let testbed = TestbedApp::from_builders(builders);
    testbed.run()
}
