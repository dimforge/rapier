#![allow(dead_code)]

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use inflector::Inflector;

use rapier_testbed3d::{Testbed, TestbedApp};
use std::cmp::Ordering;

mod balls3;
mod boxes3;
mod capsules3;
mod ccd3;
mod compound3;
mod convex_polyhedron3;
mod heightfield3;
mod joint_ball3;
mod joint_fixed3;
mod joint_prismatic3;
mod joint_revolute3;
mod keva3;
mod many_kinematics3;
mod many_pyramids3;
mod many_sleep3;
mod many_static3;
mod pyramid3;
mod ray_cast3;
mod stacks3;
mod trimesh3;

enum Command {
    Run(String),
    List,
    RunAll,
}

fn parse_command_line() -> Command {
    let mut args = std::env::args();

    while let Some(arg) = args.next() {
        if &arg[..] == "--example" {
            return Command::Run(args.next().unwrap_or_default());
        } else if &arg[..] == "--list" {
            return Command::List;
        }
    }

    Command::RunAll
}

#[allow(clippy::type_complexity)]
pub fn demo_builders() -> Vec<(&'static str, fn(&mut Testbed))> {
    let mut builders: Vec<(_, fn(&mut Testbed))> = vec![
        ("Balls", balls3::init_world),
        ("Boxes", boxes3::init_world),
        ("Capsules", capsules3::init_world),
        ("CCD", ccd3::init_world),
        ("Compound", compound3::init_world),
        ("Convex polyhedron", convex_polyhedron3::init_world),
        ("Many kinematics", many_kinematics3::init_world),
        ("Many static", many_static3::init_world),
        ("Many sleep", many_sleep3::init_world),
        ("Heightfield", heightfield3::init_world),
        ("Stacks", stacks3::init_world),
        ("Pyramid", pyramid3::init_world),
        ("Trimesh", trimesh3::init_world),
        ("ImpulseJoint ball", joint_ball3::init_world),
        ("ImpulseJoint fixed", joint_fixed3::init_world),
        ("ImpulseJoint revolute", joint_revolute3::init_world),
        ("ImpulseJoint prismatic", joint_prismatic3::init_world),
        ("Many pyramids", many_pyramids3::init_world),
        ("Keva tower", keva3::init_world),
        ("Ray cast", ray_cast3::init_world),
    ];

    // Lexicographic sort, with stress tests moved at the end of the list.
    builders.sort_by(|a, b| match (a.0.starts_with('('), b.0.starts_with('(')) {
        (true, true) | (false, false) => a.0.cmp(b.0),
        (true, false) => Ordering::Greater,
        (false, true) => Ordering::Less,
    });
    builders
}

pub fn main() {
    let command = parse_command_line();
    let builders = demo_builders();

    match command {
        Command::Run(demo) => {
            if let Some(i) = builders
                .iter()
                .position(|builder| builder.0.to_camel_case().as_str() == demo.as_str())
            {
                TestbedApp::from_builders(vec![builders[i]]).run()
            } else {
                eprintln!("Invalid example to run provided: '{demo}'");
            }
        }
        Command::RunAll => TestbedApp::from_builders(builders).run(),
        Command::List => {
            for builder in &builders {
                println!("{}", builder.0.to_camel_case())
            }
        }
    }
}
