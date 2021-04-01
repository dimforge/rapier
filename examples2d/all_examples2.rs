#![allow(dead_code)]

extern crate nalgebra as na;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use inflector::Inflector;

use rapier_testbed2d::Testbed;
use std::cmp::Ordering;

mod add_remove2;
mod ccd2;
mod collision_groups2;
mod convex_polygons2;
mod damping2;
mod debug_box_ball2;
mod heightfield2;
mod joints2;
mod locked_rotations2;
mod one_way_platforms2;
mod platform2;
mod polyline2;
mod pyramid2;
mod restitution2;
mod sensor2;
mod trimesh2;

fn demo_name_from_command_line() -> Option<String> {
    let mut args = std::env::args();

    while let Some(arg) = args.next() {
        if &arg[..] == "--example" {
            return args.next();
        }
    }

    None
}

#[cfg(any(target_arch = "wasm32", target_arch = "asmjs"))]
fn demo_name_from_url() -> Option<String> {
    None
    //    let window = stdweb::web::window();
    //    let hash = window.location()?.search().ok()?;
    //    Some(hash[1..].to_string())
}

#[cfg(not(any(target_arch = "wasm32", target_arch = "asmjs")))]
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
        ("Add remove", add_remove2::init_world),
        ("CCD", ccd2::init_world),
        ("Collision groups", collision_groups2::init_world),
        ("Convex polygons", convex_polygons2::init_world),
        ("Damping", damping2::init_world),
        ("Heightfield", heightfield2::init_world),
        ("Joints", joints2::init_world),
        ("Locked rotations", locked_rotations2::init_world),
        ("One-way platforms", one_way_platforms2::init_world),
        ("Platform", platform2::init_world),
        ("Polyline", polyline2::init_world),
        ("Pyramid", pyramid2::init_world),
        ("Restitution", restitution2::init_world),
        ("Sensor", sensor2::init_world),
        ("Trimesh", trimesh2::init_world),
        ("(Debug) box ball", debug_box_ball2::init_world),
    ];

    // Lexicographic sort, with stress tests moved at the end of the list.
    builders.sort_by(|a, b| match (a.0.starts_with("("), b.0.starts_with("(")) {
        (true, true) | (false, false) => a.0.cmp(b.0),
        (true, false) => Ordering::Greater,
        (false, true) => Ordering::Less,
    });

    let i = builders
        .iter()
        .position(|builder| builder.0.to_camel_case().as_str() == demo.as_str())
        .unwrap_or(0);
    let testbed = Testbed::from_builders(i, builders);

    testbed.run()
}
