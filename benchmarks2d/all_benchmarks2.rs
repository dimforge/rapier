#![allow(dead_code)]

extern crate nalgebra as na;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use inflector::Inflector;

use rapier_testbed2d::Testbed;
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
        ("Balls", balls2::init_world),
        ("Boxes", boxes2::init_world),
        ("Capsules", capsules2::init_world),
        ("Convex polygons", convex_polygons2::init_world),
        ("Heightfield", heightfield2::init_world),
        ("Pyramid", pyramid2::init_world),
        ("(Stress test) joint ball", joint_ball2::init_world),
        ("(Stress test) joint fixed", joint_fixed2::init_world),
        (
            "(Stress test) joint prismatic",
            joint_prismatic2::init_world,
        ),
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
