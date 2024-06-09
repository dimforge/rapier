#![allow(dead_code)]

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
extern crate rapier3d_f64 as rapier3d;

use inflector::Inflector;

use rapier_testbed3d::{Testbed, TestbedApp};
use std::cmp::Ordering;

mod debug_serialized3;

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
        .or_else(demo_name_from_url)
        .unwrap_or_default()
        .to_camel_case();

    let mut builders: Vec<(_, fn(&mut Testbed))> =
        vec![("(Debug) serialized", debug_serialized3::init_world)];

    // Lexicographic sort, with stress tests moved at the end of the list.
    builders.sort_by(|a, b| match (a.0.starts_with('('), b.0.starts_with('(')) {
        (true, true) | (false, false) => a.0.cmp(b.0),
        (true, false) => Ordering::Greater,
        (false, true) => Ordering::Less,
    });

    let i = builders
        .iter()
        .position(|builder| builder.0.to_camel_case().as_str() == demo.as_str())
        .unwrap_or(0);

    let testbed = TestbedApp::from_builders(i, builders);
    testbed.run()
}
