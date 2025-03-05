use rapier::counters::Counters;
use rapier::math::Real;
use std::num::NonZeroUsize;

use crate::debug_render::DebugRenderPipelineResource;
use crate::harness::Harness;
use crate::testbed::{
    RapierSolverType, RunMode, TestbedActionFlags, TestbedState, TestbedStateFlags,
    PHYSX_BACKEND_PATCH_FRICTION, PHYSX_BACKEND_TWO_FRICTION_DIR,
};

pub use bevy_egui::egui;

use crate::settings::SettingValue;
use crate::PhysicsState;
use bevy_egui::egui::{ComboBox, Slider, Ui, Window};
use bevy_egui::EguiContexts;
use rapier::dynamics::IntegrationParameters;
use web_time::Instant;

pub(crate) fn update_ui(
    ui_context: &mut EguiContexts,
    state: &mut TestbedState,
    harness: &mut Harness,
    debug_render: &mut DebugRenderPipelineResource,
) {
    #[cfg(feature = "profiler_ui")]
    {
        profiler_ui(ui_context);
    }

    example_settings_ui(ui_context, state);

    Window::new("Parameters").show(ui_context.ctx_mut(), |ui| {
        if state.backend_names.len() > 1 && !state.example_names.is_empty() {
            let mut changed = false;
            ComboBox::from_label("backend")
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
            }

            ui.separator();
        }

        ui.horizontal(|ui| {
            if ui.button("<").clicked() && state.selected_example > 0 {
                state.selected_example -= 1;
                state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, true)
            }

            if ui.button(">").clicked() && state.selected_example + 1 < state.example_names.len() {
                state.selected_example += 1;
                state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, true)
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

        ui.collapsing("Scene infos", |ui| {
            scene_infos_ui(ui, &harness.physics);
        });
        ui.collapsing("Profile infos", |ui| {
            ui.horizontal_wrapped(|ui| {
                profiling_ui(ui, &harness.physics.pipeline.counters);
            });
        });
        ui.collapsing("Serialization infos", |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(serialization_string(
                    harness.state.timestep_id,
                    &harness.physics,
                ))
            });
        });

        let integration_parameters = &mut harness.physics.integration_parameters;

        if state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
            || state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
        {
            let mut num_iterations = integration_parameters.num_solver_iterations.get();
            ui.add(Slider::new(&mut num_iterations, 1..=40).text("pos. iters."));
            integration_parameters.num_solver_iterations =
                NonZeroUsize::new(num_iterations).unwrap();
        } else {
            let mut changed = false;
            egui::ComboBox::from_label("solver type")
                .width(150.0)
                .selected_text(format!("{:?}", state.solver_type))
                .show_ui(ui, |ui| {
                    let solver_types = [
                        RapierSolverType::TgsSoft,
                        RapierSolverType::TgsSoftNoWarmstart,
                        RapierSolverType::PgsLegacy,
                    ];
                    for sty in solver_types {
                        changed = ui
                            .selectable_value(&mut state.solver_type, sty, format!("{sty:?}"))
                            .changed()
                            || changed;
                    }
                });

            if changed {
                match state.solver_type {
                    RapierSolverType::TgsSoft => {
                        *integration_parameters = IntegrationParameters::tgs_soft();
                    }
                    RapierSolverType::TgsSoftNoWarmstart => {
                        *integration_parameters =
                            IntegrationParameters::tgs_soft_without_warmstart();
                    }
                    RapierSolverType::PgsLegacy => {
                        *integration_parameters = IntegrationParameters::pgs_legacy();
                    }
                }
            }

            let mut num_iterations = integration_parameters.num_solver_iterations.get();
            ui.add(Slider::new(&mut num_iterations, 1..=40).text("num solver iters."));
            integration_parameters.num_solver_iterations =
                NonZeroUsize::new(num_iterations).unwrap();

            ui.add(
                Slider::new(
                    &mut integration_parameters.num_internal_pgs_iterations,
                    1..=40,
                )
                .text("num internal PGS iters."),
            );
            ui.add(
                Slider::new(
                    &mut integration_parameters.num_additional_friction_iterations,
                    0..=40,
                )
                .text("num additional frict. iters."),
            );
            ui.add(
                Slider::new(
                    &mut integration_parameters.num_internal_stabilization_iterations,
                    0..=100,
                )
                .text("max internal stabilization iters."),
            );
            ui.add(
                Slider::new(&mut integration_parameters.warmstart_coefficient, 0.0..=1.0)
                    .text("warmstart coefficient"),
            );

            let mut substep_params = *integration_parameters;
            substep_params.dt /= substep_params.num_solver_iterations.get() as Real;
            let curr_erp = substep_params.contact_erp();
            let curr_cfm_factor = substep_params.contact_cfm_factor();
            ui.add(
                Slider::new(
                    &mut integration_parameters.contact_natural_frequency,
                    0.01..=120.0,
                )
                .text(format!("contacts Hz (erp = {:.3})", curr_erp)),
            );
            ui.add(
                Slider::new(
                    &mut integration_parameters.contact_damping_ratio,
                    0.01..=20.0,
                )
                .text(format!(
                    "damping ratio (cfm-factor = {:.3})",
                    curr_cfm_factor
                )),
            );
            ui.add(
                Slider::new(
                    &mut integration_parameters.joint_natural_frequency,
                    0.0..=1200000.0,
                )
                .text("joint erp"),
            );
            ui.add(
                Slider::new(&mut integration_parameters.joint_damping_ratio, 0.0..=20.0)
                    .text("joint damping ratio"),
            );
        }

        #[cfg(feature = "parallel")]
        {
            let mut num_threads = harness.state.num_threads();
            ui.add(
                Slider::new(&mut num_threads, 1..=num_cpus::get_physical()).text("num. threads"),
            );
            harness.state.set_num_threads(num_threads);
        }
        ui.add(
            Slider::new(&mut integration_parameters.max_ccd_substeps, 0..=10).text("CCD substeps"),
        );
        ui.add(
            Slider::new(&mut integration_parameters.min_island_size, 1..=10_000)
                .text("min island size"),
        );
        ui.add(Slider::new(&mut state.nsteps, 1..=100).text("sims. per frame"));

        let mut frequency = integration_parameters.inv_dt().round() as u32;
        ui.add(Slider::new(&mut frequency, 0..=240).text("frequency (Hz)"));
        integration_parameters.set_inv_dt(frequency as Real);

        let mut sleep = state.flags.contains(TestbedStateFlags::SLEEP);
        let mut draw_surfaces = state.flags.contains(TestbedStateFlags::DRAW_SURFACES);
        // let mut contact_points = state.flags.contains(TestbedStateFlags::CONTACT_POINTS);
        // let mut wireframe = state.flags.contains(TestbedStateFlags::WIREFRAME);
        ui.checkbox(&mut sleep, "sleep enabled");
        // ui.checkbox(&mut contact_points, "draw contacts");
        // ui.checkbox(&mut wireframe, "draw wireframes");
        ui.checkbox(&mut draw_surfaces, "surface render enabled");
        ui.checkbox(&mut debug_render.enabled, "debug render enabled");

        state.flags.set(TestbedStateFlags::SLEEP, sleep);
        state
            .flags
            .set(TestbedStateFlags::DRAW_SURFACES, draw_surfaces);
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

fn scene_infos_ui(ui: &mut Ui, physics: &PhysicsState) {
    ui.label(format!("# rigid-bodies: {}", physics.bodies.len()));
    ui.label(format!("# colliders: {}", physics.colliders.len()));
    ui.label(format!("# impulse joint: {}", physics.impulse_joints.len()));
    // ui.label(format!(
    //     "# multibody joint: {}",
    //     physics.multibody_joints.len()
    // ));
}

fn profiling_ui(ui: &mut Ui, counters: &Counters) {
    egui::CollapsingHeader::new(format!(
        "Total: {:.2}ms - {} fps",
        counters.step_time_ms(),
        (1000.0 / counters.step_time_ms()).round()
    ))
    .id_salt("total")
    .show(ui, |ui| {
        egui::CollapsingHeader::new(format!(
            "Collision detection: {:.2}ms",
            counters.collision_detection_time_ms()
        ))
        .id_salt("collision detection")
        .show(ui, |ui| {
            ui.label(format!(
                "Broad-phase: {:.2}ms",
                counters.broad_phase_time_ms()
            ));
            ui.label(format!(
                "Narrow-phase: {:.2}ms",
                counters.narrow_phase_time_ms()
            ));
        });
        egui::CollapsingHeader::new(format!("Solver: {:.2}ms", counters.solver_time_ms()))
            .id_salt("solver")
            .show(ui, |ui| {
                ui.label(format!(
                    "Velocity assembly: {:.2}ms",
                    counters.solver.velocity_assembly_time.time_ms()
                ));
                ui.label(format!(
                    "Velocity resolution: {:.2}ms",
                    counters.velocity_resolution_time_ms()
                ));
                ui.label(format!(
                    "Velocity integration: {:.2}ms",
                    counters.solver.velocity_update_time.time_ms()
                ));
                ui.label(format!(
                    "Writeback: {:.2}ms",
                    counters.solver.velocity_writeback_time.time_ms()
                ));
            });
        egui::CollapsingHeader::new(format!("CCD: {:.2}ms", counters.ccd_time_ms()))
            .id_salt("ccd")
            .show(ui, |ui| {
                ui.label(format!("# of substeps: {}", counters.ccd.num_substeps));
                ui.label(format!(
                    "TOI computation: {:.2}ms",
                    counters.ccd.toi_computation_time.time_ms(),
                ));
                ui.label(format!(
                    "Broad-phase: {:.2}ms",
                    counters.ccd.broad_phase_time.time_ms()
                ));
                ui.label(format!(
                    "Narrow-phase: {:.2}ms",
                    counters.ccd.narrow_phase_time.time_ms(),
                ));
                ui.label(format!(
                    "Solver: {:.2}ms",
                    counters.ccd.solver_time.time_ms()
                ));
            });
        ui.label(format!(
            "Island computation: {:.2}ms",
            counters.island_construction_time_ms()
        ));
        ui.label(format!(
            "Query pipeline: {:.2}ms",
            counters.query_pipeline_update_time_ms()
        ));
        ui.label(format!(
            "User changes: {:.2}ms",
            counters.stages.user_changes.time_ms()
        ));
    });
}

fn serialization_string(timestep_id: usize, physics: &PhysicsState) -> String {
    let t = Instant::now();
    // let t = Instant::now();
    let bf = bincode::serialize(&physics.broad_phase).unwrap();
    // println!("bf: {}", Instant::now() - t);
    // let t = Instant::now();
    let nf = bincode::serialize(&physics.narrow_phase).unwrap();
    // println!("nf: {}", Instant::now() - t);
    // let t = Instant::now();
    let bs = bincode::serialize(&physics.bodies).unwrap();
    // println!("bs: {}", Instant::now() - t);
    // let t = Instant::now();
    let cs = bincode::serialize(&physics.colliders).unwrap();
    // println!("cs: {}", Instant::now() - t);
    // let t = Instant::now();
    let js = bincode::serialize(&physics.impulse_joints).unwrap();
    // println!("js: {}", Instant::now() - t);
    let serialization_time = Instant::now() - t;
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
|_ &RigidBodySet [{:.1}KB]: {}
|_ Colliders [{:.1}KB]: {}
|_ Joints [{:.1}KB]: {}"#,
        serialization_time.as_secs_f64() * 1000.0,
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

fn example_settings_ui(ui_context: &mut EguiContexts, state: &mut TestbedState) {
    if state.example_settings.is_empty() {
        // Don’t show any window if there is no settings for the
        // example.
        return;
    }

    Window::new("Example settings").show(ui_context.ctx_mut(), |ui| {
        let mut any_changed = false;
        for (name, value) in state.example_settings.iter_mut() {
            let prev_value = value.clone();
            match value {
                SettingValue::F32 { value, range } => {
                    ui.add(Slider::new(value, range.clone()).text(name));
                }
                SettingValue::U32 { value, range } => {
                    ui.horizontal(|ui| {
                        if ui.button("<").clicked() && *value > *range.start() {
                            *value -= 1;
                        }
                        if ui.button(">").clicked() && *value <= *range.end() {
                            *value += 1;
                        }

                        ui.add(Slider::new(value, range.clone()).text(name));
                    });
                }
            }

            any_changed = any_changed || *value != prev_value;
        }

        if any_changed {
            // The value changed, request a restart.
            state.action_flags.set(TestbedActionFlags::RESTART, true);
        }
    });
}

#[cfg(feature = "profiler_ui")]
fn profiler_ui(ui_context: &mut EguiContexts) {
    let window = egui::Window::new("Profiling");
    let window = window.default_open(false);

    #[cfg(feature = "unstable-puffin-pr-235")]
    {
        use std::sync::Once;
        static START: Once = Once::new();

        fn set_default_rapier_filter() {
            let mut profile_ui = puffin_egui::PROFILE_UI.lock();
            profile_ui
                .profiler_ui
                .flamegraph_options
                .scope_name_filter
                .set_filter("Harness::step_with_graphics".to_string());
        }
        START.call_once(|| {
            set_default_rapier_filter();
        });
        window.show(ui_context.ctx_mut(), |ui| {
            if ui.button("🔍 Rapier filter").clicked() {
                set_default_rapier_filter();
            }
            puffin_egui::profiler_ui(ui);
        });
    }

    #[cfg(not(feature = "unstable-puffin-pr-235"))]
    window.show(ui_context.ctx_mut(), |ui| {
        puffin_egui::profiler_ui(ui);
    });
}
