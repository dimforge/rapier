use rapier::counters::Counters;

use crate::testbed::{RunMode, TestbedActionFlags, TestbedState, TestbedStateFlags};
use rapier::harness::Harness;

use crate::PhysicsState;
use bevy_egui::egui::Slider;
use bevy_egui::{egui, EguiContext};

pub fn update_ui(ui_context: &EguiContext, state: &mut TestbedState, harness: &mut Harness) {
    egui::Window::new("Parameters").show(ui_context.ctx(), |ui| {
        if state.backend_names.len() > 1 && !state.example_names.is_empty() {
            #[cfg(all(feature = "dim3", feature = "other-backends"))]
            let prev_selected_backend = state.selected_backend;
            let mut changed = false;
            egui::ComboBox::from_label("backend")
                .width(150.0)
                .selected_text(state.backend_names[state.selected_backend])
                .show_ui(ui, |ui| {
                    for (id, name) in state.backend_names.iter().enumerate() {
                        changed = ui
                            .selectable_value(&mut state.selected_backend, id, *name)
                            .changed()
                            || changed;
                    }
                });

            if changed {
                state
                    .action_flags
                    .set(TestbedActionFlags::BACKEND_CHANGED, true);

                #[cfg(all(feature = "dim3", feature = "other-backends"))]
                fn is_physx(id: usize) -> bool {
                    id == crate::testbed::PHYSX_BACKEND_PATCH_FRICTION
                        || id == crate::testbed::PHYSX_BACKEND_TWO_FRICTION_DIR
                }

                #[cfg(all(feature = "dim3", feature = "other-backends"))]
                if (is_physx(state.selected_backend) && !is_physx(prev_selected_backend))
                    || (!is_physx(state.selected_backend) && is_physx(prev_selected_backend))
                {
                    // PhysX defaults (4 position iterations, 1 velocity) are the
                    // opposite of rapier's (4 velocity iterations, 1 position).
                    std::mem::swap(
                        &mut harness
                            .physics
                            .integration_parameters
                            .max_position_iterations,
                        &mut harness
                            .physics
                            .integration_parameters
                            .max_velocity_iterations,
                    );
                }
            }

            ui.separator();
        }

        ui.horizontal(|ui| {
            if ui.button("<").clicked() {
                if state.selected_example > 0 {
                    state.selected_example -= 1;
                    state
                        .action_flags
                        .set(TestbedActionFlags::EXAMPLE_CHANGED, true)
                }
            }

            if ui.button(">").clicked() {
                if state.selected_example + 1 < state.example_names.len() {
                    state.selected_example += 1;
                    state
                        .action_flags
                        .set(TestbedActionFlags::EXAMPLE_CHANGED, true)
                }
            }

            let mut changed = false;
            egui::ComboBox::from_label("example")
                .width(150.0)
                .selected_text(state.example_names[state.selected_example])
                .show_ui(ui, |ui| {
                    for (id, name) in state.example_names.iter().enumerate() {
                        changed = ui
                            .selectable_value(&mut state.selected_example, id, *name)
                            .changed()
                            || changed;
                    }
                });
            if changed {
                state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
            }
        });

        ui.separator();

        ui.collapsing("Profile infos", |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(profiling_string(&harness.physics.pipeline.counters))
            });
        });
        ui.collapsing("Serialization infos", |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(serialization_string(
                    harness.run_state.timestep_id,
                    &harness.physics,
                ))
            });
        });

        let integration_parameters = &mut harness.physics.integration_parameters;
        ui.add(
            Slider::new(&mut integration_parameters.max_velocity_iterations, 0..=200)
                .text("vels. iters."),
        );
        ui.add(
            Slider::new(&mut integration_parameters.max_position_iterations, 0..=200)
                .text("pos. iters."),
        );
        #[cfg(feature = "parallel")]
        {
            ui.add(
                Slider::new(
                    &mut harness.thread_state.num_threads,
                    1..=num_cpus::get_physical(),
                )
                .text("num. threads"),
            );
        }
        ui.add(
            Slider::new(&mut integration_parameters.max_ccd_substeps, 0..=10).text("CCD substeps"),
        );
        ui.add(
            Slider::new(&mut integration_parameters.min_island_size, 1..=10_000)
                .text("min island size"),
        );
        ui.add(
            Slider::new(&mut integration_parameters.warmstart_coeff, 0.0..=1.0)
                .text("warmstart coeff"),
        );
        let mut frequency = integration_parameters.inv_dt().round() as u32;
        ui.add(Slider::new(&mut frequency, 0..=240).text("frequency (Hz)"));
        integration_parameters.set_inv_dt(frequency as f32);

        let mut sleep = state.flags.contains(TestbedStateFlags::SLEEP);
        // let mut contact_points = state.flags.contains(TestbedStateFlags::CONTACT_POINTS);
        // let mut wireframe = state.flags.contains(TestbedStateFlags::WIREFRAME);
        ui.checkbox(&mut sleep, "sleep enabled");
        // ui.checkbox(&mut contact_points, "draw contacts");
        // ui.checkbox(&mut wireframe, "draw wireframes");

        state.flags.set(TestbedStateFlags::SLEEP, sleep);
        // state
        //     .flags
        //     .set(TestbedStateFlags::CONTACT_POINTS, contact_points);
        // state.flags.set(TestbedStateFlags::WIREFRAME, wireframe);
        ui.separator();

        let label = if state.running == RunMode::Stop {
            "Start (T)"
        } else {
            "Pause (T)"
        };

        if ui.button(label).clicked() {
            if state.running == RunMode::Stop {
                state.running = RunMode::Running
            } else {
                state.running = RunMode::Stop
            }
        }

        if ui.button("Single Step (S)").clicked() {
            state.running = RunMode::Step;
        }

        if ui.button("Take snapshot").clicked() {
            state
                .action_flags
                .set(TestbedActionFlags::TAKE_SNAPSHOT, true);
        }

        if ui.button("Restore snapshot").clicked() {
            state
                .action_flags
                .set(TestbedActionFlags::RESTORE_SNAPSHOT, true);
        }

        if ui.button("Restart (R)").clicked() {
            state.action_flags.set(TestbedActionFlags::RESTART, true);
        }
    });
}

fn profiling_string(counters: &Counters) -> String {
    format!(
        r#"Total: {:.2}ms
Collision detection: {:.2}ms
|_ Broad-phase: {:.2}ms
   Narrow-phase: {:.2}ms
Island computation: {:.2}ms
Solver: {:.2}ms
|_ Velocity assembly: {:.2}ms
   Velocity resolution: {:.2}ms
   Velocity integration: {:.2}ms
   Position assembly: {:.2}ms
   Position resolution: {:.2}ms
CCD: {:.2}ms
|_ # of substeps: {}
   TOI computation: {:.2}ms
   Broad-phase: {:.2}ms
   Narrow-phase: {:.2}ms
   Solver: {:.2}ms"#,
        counters.step_time(),
        counters.collision_detection_time(),
        counters.broad_phase_time(),
        counters.narrow_phase_time(),
        counters.island_construction_time(),
        counters.solver_time(),
        counters.solver.velocity_assembly_time.time(),
        counters.velocity_resolution_time(),
        counters.solver.velocity_update_time.time(),
        counters.solver.position_assembly_time.time(),
        counters.position_resolution_time(),
        counters.ccd_time(),
        counters.ccd.num_substeps,
        counters.ccd.toi_computation_time.time(),
        counters.ccd.broad_phase_time.time(),
        counters.ccd.narrow_phase_time.time(),
        counters.ccd.solver_time.time(),
    )
}

fn serialization_string(timestep_id: usize, physics: &PhysicsState) -> String {
    let t = instant::now();
    // let t = instant::now();
    let bf = bincode::serialize(&physics.broad_phase).unwrap();
    // println!("bf: {}", instant::now() - t);
    // let t = instant::now();
    let nf = bincode::serialize(&physics.narrow_phase).unwrap();
    // println!("nf: {}", instant::now() - t);
    // let t = instant::now();
    let bs = bincode::serialize(&physics.bodies).unwrap();
    // println!("bs: {}", instant::now() - t);
    // let t = instant::now();
    let cs = bincode::serialize(&physics.colliders).unwrap();
    // println!("cs: {}", instant::now() - t);
    // let t = instant::now();
    let js = bincode::serialize(&physics.joints).unwrap();
    // println!("js: {}", instant::now() - t);
    let serialization_time = instant::now() - t;
    let hash_bf = md5::compute(&bf);
    let hash_nf = md5::compute(&nf);
    let hash_bodies = md5::compute(&bs);
    let hash_colliders = md5::compute(&cs);
    let hash_joints = md5::compute(&js);
    format!(
        r#"Serialization time: {:.2}ms
Hashes at frame: {}
|_ Broad phase [{:.1}KB]: {}
|_ Narrow phase [{:.1}KB]: {}
|_ Bodies [{:.1}KB]: {}
|_ Colliders [{:.1}KB]: {}
|_ Joints [{:.1}KB]: {}"#,
        serialization_time,
        timestep_id,
        bf.len() as f32 / 1000.0,
        format!("{:?}", hash_bf).split_at(10).0,
        nf.len() as f32 / 1000.0,
        format!("{:?}", hash_nf).split_at(10).0,
        bs.len() as f32 / 1000.0,
        format!("{:?}", hash_bodies).split_at(10).0,
        cs.len() as f32 / 1000.0,
        format!("{:?}", hash_colliders).split_at(10).0,
        js.len() as f32 / 1000.0,
        format!("{:?}", hash_joints).split_at(10).0,
    )
}
