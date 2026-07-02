#![allow(dead_code)]
#![allow(clippy::type_complexity)]

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
extern crate rapier3d_f64 as rapier3d;
extern crate rapier_testbed3d_f64 as rapier_testbed3d;

use rapier_testbed3d::{ExampleEntry, TestbedViewer};
use std::future::Future;
use std::pin::Pin;

mod debug_serialized3;
mod trimesh3_f64;

type ExampleFn =
    for<'a> fn(&'a mut TestbedViewer) -> Pin<Box<dyn Future<Output = anyhow::Result<()>> + 'a>>;

macro_rules! examples {
    ($($group:expr, $name:expr, $run:path);* $(;)?) => {
        vec![ $( (ExampleEntry::new($group, $name), (|v| Box::pin($run(v))) as ExampleFn) ),* ]
    };
}

#[kiss3d::main]
pub async fn main() {
    const DEMOS: &str = "Demos f64";

    let examples: Vec<(ExampleEntry, ExampleFn)> = examples![
        DEMOS, "Trimesh", trimesh3_f64::run;
        DEMOS, "(Debug) serialized", debug_serialized3::run;
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
