#![allow(dead_code)]

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use rapier_testbed2d::{Testbed, TestbedApp};
use std::cmp::Ordering;

mod balls2;
mod boxes2;
mod capsules2;
mod convex_polygons2;
mod heightfield2;
mod joint_ball2;
mod joint_fixed2;
mod joint_prismatic2;
mod pyramid2;
mod vertical_stacks2;

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
    //    Some(hash[1..].to_string())
}

#[cfg(not(target_arch = "wasm32"))]
fn demo_name_from_url() -> Option<String> {
    None
}

#[cfg_attr(target_arch = "wasm32", wasm_bindgen(start))]
pub fn main() {
    let mut builders: Vec<(_, fn(&mut Testbed))> = vec![
        ("Balls", balls2::init_world),
        ("Boxes", boxes2::init_world),
        ("Capsules", capsules2::init_world),
        ("Convex polygons", convex_polygons2::init_world),
        ("Heightfield", heightfield2::init_world),
        ("Pyramid", pyramid2::init_world),
        ("Verticals stacks", vertical_stacks2::init_world),
        ("(Stress test) joint ball", joint_ball2::init_world),
        ("(Stress test) joint fixed", joint_fixed2::init_world),
        (
            "(Stress test) joint prismatic",
            joint_prismatic2::init_world,
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
