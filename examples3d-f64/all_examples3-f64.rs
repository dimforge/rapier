#![allow(dead_code)]

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
extern crate rapier3d_f64 as rapier3d;
extern crate rapier_testbed3d_f64 as rapier_testbed3d;

use rapier_testbed3d::{Example, TestbedApp};

mod debug_serialized3;
mod trimesh3_f64;

#[kiss3d::main]
pub async fn main() {
    let builders: Vec<_> = vec![
        Example::new("Demos f64", "Trimesh", trimesh3_f64::init_world),
        Example::new(
            "Demos f64",
            "(Debug) serialized",
            debug_serialized3::init_world,
        ),
    ];

    let testbed = TestbedApp::from_builders(builders);
    testbed.run().await
}
