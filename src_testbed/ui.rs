use rapier::math::Real;

use crate::debug_render::DebugRenderPipelineResource;
use crate::harness::{Harness, RapierBroadPhaseType};
use crate::testbed::{
    PHYSX_BACKEND_PATCH_FRICTION, PHYSX_BACKEND_TWO_FRICTION_DIR, RunMode, TestbedActionFlags,
    TestbedState, TestbedStateFlags, UiTab,
};

pub use egui;

use crate::PhysicsState;
use crate::settings::SettingValue;
use egui::{ComboBox, RichText, Slider, Ui, Window};
use web_time::Instant;

#[cfg(feature = "dim3")]
use rapier::dynamics::FrictionModel;

/// Sets up a custom warm theme that complements the app's off-white background.
fn setup_custom_theme(ctx: &egui::Context) {
    use egui::{Color32, CornerRadius, Stroke};

    let bg_fill = Color32::from_rgb(250, 250, 245);
    let window_fill = Color32::from_rgb(252, 252, 248);
    let faint_bg = Color32::from_rgb(240, 240, 232);
    let extreme_bg = Color32::from_rgb(255, 255, 252);

    let text_color = Color32::from_rgb(60, 58, 52);

    let accent = Color32::from_rgb(82, 130, 150);
    let accent_active = Color32::from_rgb(70, 115, 135);

    let widget_bg = Color32::from_rgb(235, 235, 225);
    let widget_bg_hover = Color32::from_rgb(225, 225, 215);
    let widget_bg_active = Color32::from_rgb(215, 215, 205);

    let stroke_color = Color32::from_rgb(200, 198, 190);
    let stroke_hover = Color32::from_rgb(180, 178, 170);

    let rounding = CornerRadius::same(6);
    let small_rounding = CornerRadius::same(4);

    ctx.style_mut(|style| {
        let v = &mut style.visuals;
        v.dark_mode = false;

        v.widgets.noninteractive.bg_fill = faint_bg;
        v.widgets.noninteractive.weak_bg_fill = faint_bg;
        v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, stroke_color);
        v.widgets.noninteractive.corner_radius = rounding;
        v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, text_color);

        v.widgets.inactive.bg_fill = widget_bg;
        v.widgets.inactive.weak_bg_fill = widget_bg;
        v.widgets.inactive.bg_stroke = Stroke::new(1.0, stroke_color);
        v.widgets.inactive.corner_radius = small_rounding;
        v.widgets.inactive.fg_stroke = Stroke::new(1.0, text_color);

        v.widgets.hovered.bg_fill = widget_bg_hover;
        v.widgets.hovered.weak_bg_fill = widget_bg_hover;
        v.widgets.hovered.bg_stroke = Stroke::new(1.0, stroke_hover);
        v.widgets.hovered.corner_radius = small_rounding;
        v.widgets.hovered.fg_stroke = Stroke::new(1.5, text_color);

        v.widgets.active.bg_fill = widget_bg_active;
        v.widgets.active.weak_bg_fill = widget_bg_active;
        v.widgets.active.bg_stroke = Stroke::new(1.0, accent);
        v.widgets.active.corner_radius = small_rounding;
        v.widgets.active.fg_stroke = Stroke::new(2.0, accent_active);

        v.widgets.open.bg_fill = widget_bg;
        v.widgets.open.weak_bg_fill = widget_bg;
        v.widgets.open.bg_stroke = Stroke::new(1.0, stroke_color);
        v.widgets.open.corner_radius = small_rounding;
        v.widgets.open.fg_stroke = Stroke::new(1.0, text_color);

        v.selection.bg_fill = accent.gamma_multiply(0.25);
        v.selection.stroke = Stroke::new(1.0, accent);

        v.hyperlink_color = accent;
        v.faint_bg_color = faint_bg;
        v.extreme_bg_color = extreme_bg;
        v.code_bg_color = Color32::from_rgb(230, 230, 220);
        v.warn_fg_color = Color32::from_rgb(180, 120, 60);
        v.error_fg_color = Color32::from_rgb(180, 70, 70);

        v.window_corner_radius = CornerRadius::same(8);
        v.window_fill = window_fill;
        v.window_stroke = Stroke::new(1.0, stroke_color);

        v.panel_fill = bg_fill;

        v.slider_trailing_fill = true;
        v.handle_shape = egui::style::HandleShape::Circle;

        style.spacing.item_spacing = egui::vec2(6.0, 3.0);
        style.spacing.window_margin = egui::Margin::same(10);
        style.spacing.button_padding = egui::vec2(6.0, 3.0);
        style.spacing.slider_width = 130.0;
        style.spacing.indent = 14.0;
        style.spacing.interact_size = egui::vec2(32.0, 18.0);
        style.spacing.combo_width = 100.0;
    });
}

pub(crate) fn update_ui(
    ui_context: &egui::Context,
    state: &mut TestbedState,
    harness: &mut Harness,
    debug_render: &mut DebugRenderPipelineResource,
) {
    setup_custom_theme(ui_context);

    #[cfg(feature = "profiler_ui")]
    {
        profiler_ui(ui_context);
    }

    example_settings_ui(ui_context, state);

    Window::new("Rapier Testbed")
        .default_width(300.0)
        .show(ui_context, |ui| {
            // ═══════════════════════════════════════════════════════════════
            // TAB BAR
            // ═══════════════════════════════════════════════════════════════
            ui.horizontal(|ui| {
                ui.selectable_value(&mut state.selected_tab, UiTab::Examples, "Examples");
                ui.selectable_value(&mut state.selected_tab, UiTab::Settings, "Settings");
                ui.selectable_value(&mut state.selected_tab, UiTab::Performance, "Performance");
            });

            ui.separator();

            // ═══════════════════════════════════════════════════════════════
            // TAB CONTENT
            // ═══════════════════════════════════════════════════════════════
            egui::ScrollArea::vertical()
                .max_height(400.0)
                .show(ui, |ui| match state.selected_tab {
                    UiTab::Examples => {
                        examples_tab(ui, state);
                    }
                    UiTab::Settings => {
                        settings_tab(ui, state, harness, debug_render);
                    }
                    UiTab::Performance => {
                        performance_tab(ui, harness);
                    }
                });

            ui.separator();

            // ═══════════════════════════════════════════════════════════════
            // BOTTOM CONTROLS - Always visible
            // ═══════════════════════════════════════════════════════════════
            ui.horizontal(|ui| {
                // Play/Pause
                let (label, hover) = if state.running == RunMode::Stop {
                    ("Play", "Start simulation (T)")
                } else {
                    ("Pause", "Pause simulation (T)")
                };

                if ui.button(label).on_hover_text(hover).clicked() {
                    state.running = if state.running == RunMode::Stop {
                        RunMode::Running
                    } else {
                        RunMode::Stop
                    };
                }

                // Step
                if ui.button("Step").on_hover_text("Single step (S)").clicked() {
                    state.running = RunMode::Step;
                }

                // Restart
                if ui
                    .button("Restart")
                    .on_hover_text("Restart example (R)")
                    .clicked()
                {
                    state.action_flags.set(TestbedActionFlags::RESTART, true);
                }

                ui.separator();

                // Save/Restore
                if ui.button("Save").on_hover_text("Save snapshot").clicked() {
                    state
                        .action_flags
                        .set(TestbedActionFlags::TAKE_SNAPSHOT, true);
                }

                if ui
                    .button("Restore")
                    .on_hover_text("Restore snapshot")
                    .clicked()
                {
                    state
                        .action_flags
                        .set(TestbedActionFlags::RESTORE_SNAPSHOT, true);
                }
            });
        });
}

fn examples_tab(ui: &mut Ui, state: &mut TestbedState) {
    // Backend selector (if multiple backends available)
    if state.backend_names.len() > 1 {
        ui.horizontal(|ui| {
            ui.label("Backend:");
            let mut backend_changed = false;
            ComboBox::from_id_salt("backend_combo")
                .width(150.0)
                .selected_text(state.backend_names[state.selected_backend])
                .show_ui(ui, |ui| {
                    for (id, name) in state.backend_names.iter().enumerate() {
                        backend_changed = ui
                            .selectable_value(&mut state.selected_backend, id, *name)
                            .changed()
                            || backend_changed;
                    }
                });

            if backend_changed {
                state
                    .action_flags
                    .set(TestbedActionFlags::BACKEND_CHANGED, true);
            }
        });
    }

    // Navigation and backend selector row
    ui.horizontal(|ui| {
        // Previous/Next buttons (navigate in display order)
        if ui
            .add_enabled(state.selected_display_index > 0, egui::Button::new("<"))
            .on_hover_text("Previous example")
            .clicked()
        {
            state.selected_display_index -= 1;
            state
                .action_flags
                .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
        }

        if ui
            .add_enabled(
                state.selected_display_index + 1 < state.examples.len(),
                egui::Button::new(">"),
            )
            .on_hover_text("Next example")
            .clicked()
        {
            state.selected_display_index += 1;
            state
                .action_flags
                .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
        }

        // Current example name
        if let Some(current) = state.examples.get(state.selected_display_index) {
            ui.label(
                RichText::new(format!("[{}] {}", current.group, current.name))
                    .strong()
                    .italics(),
            );
        }
    });

    ui.add_space(4.0);
    ui.separator();
    ui.add_space(4.0);

    // Display examples grouped by their assigned groups
    for group in &state.example_groups.clone() {
        // Collect examples in this group with their display indices
        let examples_in_group: Vec<(usize, &str)> = state
            .examples
            .iter()
            .enumerate()
            .filter(|(_, e)| e.group == *group)
            .map(|(display_idx, e)| (display_idx, e.name))
            .collect();

        if examples_in_group.is_empty() {
            continue;
        }

        egui::CollapsingHeader::new(format!("{} ({})", group, examples_in_group.len()))
            .default_open(false)
            .show(ui, |ui| {
                for (display_idx, name) in examples_in_group {
                    let is_selected = state.selected_display_index == display_idx;
                    let text = if is_selected {
                        RichText::new(name).strong()
                    } else {
                        RichText::new(name)
                    };

                    if ui
                        .selectable_label(is_selected, text)
                        .on_hover_text("Click to run this example")
                        .clicked()
                        && !is_selected
                    {
                        state.selected_display_index = display_idx;
                        state
                            .action_flags
                            .set(TestbedActionFlags::EXAMPLE_CHANGED, true);
                    }
                }
            });
    }
}

fn settings_tab(
    ui: &mut Ui,
    state: &mut TestbedState,
    harness: &mut Harness,
    debug_render: &mut DebugRenderPipelineResource,
) {
    let integration_parameters = &mut harness.physics.integration_parameters;
    let is_physx = state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
        || state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR;

    // ─────────────────────────────────────────────────────────────────
    // RENDERING
    // ─────────────────────────────────────────────────────────────────
    ui.label(RichText::new("Rendering").strong());
    ui.add_space(2.0);

    let mut draw_surfaces = state.flags.contains(TestbedStateFlags::DRAW_SURFACES);
    if ui
        .checkbox(&mut draw_surfaces, "Surfaces")
        .on_hover_text("Render collider surfaces.")
        .changed()
    {
        state
            .flags
            .set(TestbedStateFlags::DRAW_SURFACES, draw_surfaces);
    }

    ui.checkbox(&mut debug_render.enabled, "Debug render")
        .on_hover_text("Show debug wireframes and contacts.");

    // ─────────────────────────────────────────────────────────────────
    // SIMULATION
    // ─────────────────────────────────────────────────────────────────
    ui.label(RichText::new("Simulation").strong());
    ui.add_space(2.0);

    // Frequency
    let mut frequency = integration_parameters.inv_dt().round() as u32;
    if ui
        .add(
            Slider::new(&mut frequency, 30..=240)
                .text("Hz")
                .clamping(egui::SliderClamping::Never),
        )
        .on_hover_text("Simulation frequency. Higher = more accurate but slower.")
        .changed()
    {
        integration_parameters.set_inv_dt(frequency as Real);
    }

    // Steps per frame
    ui.add(Slider::new(&mut state.nsteps, 1..=100).text("Steps/frame"))
        .on_hover_text("Physics steps per rendered frame.");

    // Gravity
    let mut gravity_y = harness.physics.gravity.y;
    if ui
        .add(Slider::new(&mut gravity_y, 0.0..=-200.0).text("Gravity"))
        .on_hover_text("Gravity (m/s^2). Default: -9.81")
        .changed()
    {
        harness.physics.gravity.y = gravity_y;
    }

    // Sleep
    let mut sleep = state.flags.contains(TestbedStateFlags::SLEEP);
    if ui
        .checkbox(&mut sleep, "Sleep enabled")
        .on_hover_text("Allow resting bodies to sleep for better performance.")
        .changed()
    {
        state.flags.set(TestbedStateFlags::SLEEP, sleep);
    }

    ui.add_space(8.0);

    // ─────────────────────────────────────────────────────────────────
    // SOLVER
    // ─────────────────────────────────────────────────────────────────
    ui.label(RichText::new("Solver").strong());
    ui.add_space(2.0);

    ui.add(Slider::new(&mut integration_parameters.num_solver_iterations, 1..=10).text("Substeps"))
        .on_hover_text("Main solver iterations. Higher = more stable stacking.");

    if !is_physx {
        ui.add(
            Slider::new(
                &mut integration_parameters.num_internal_pgs_iterations,
                1..=40,
            )
            .text("PGS iters"),
        )
        .on_hover_text("Internal Projected Gauss-Seidel iterations.");

        ui.add(
            Slider::new(
                &mut integration_parameters.num_internal_stabilization_iterations,
                0..=100,
            )
            .text("Relaxation"),
        )
        .on_hover_text("Position stabilization iterations.");

        ui.add(
            Slider::new(&mut integration_parameters.warmstart_coefficient, 0.0..=1.0)
                .text("Warmstart"),
        )
        .on_hover_text("Reuse previous impulses for faster convergence.");
    }

    ui.add_space(8.0);

    // ─────────────────────────────────────────────────────────────────
    // CONTACTS (Rapier only)
    // ─────────────────────────────────────────────────────────────────
    if !is_physx {
        ui.label(RichText::new("Contacts").strong());
        ui.add_space(2.0);

        let mut substep_params = *integration_parameters;
        substep_params.dt /= substep_params.num_solver_iterations as Real;
        let curr_erp = substep_params.contact_softness.erp(substep_params.dt);
        let curr_cfm = substep_params
            .contact_softness
            .cfm_factor(substep_params.dt);

        ui.add(
            Slider::new(
                &mut integration_parameters.contact_softness.natural_frequency,
                0.01..=120.0,
            )
            .text("Frequency"),
        )
        .on_hover_text(format!(
            "Contact stiffness (Hz). Higher = stiffer.\nERP = {curr_erp:.3}"
        ));

        ui.add(
            Slider::new(
                &mut integration_parameters.contact_softness.damping_ratio,
                0.01..=20.0,
            )
            .text("Damping"),
        )
        .on_hover_text(format!(
            "Contact damping. 1.0 = critical.\nCFM = {curr_cfm:.5}"
        ));

        #[cfg(feature = "dim3")]
        {
            ui.horizontal(|ui| {
                ui.label("Friction model:");
                egui::ComboBox::from_id_salt("friction_model")
                    .width(100.0)
                    .selected_text(format!("{:?}", integration_parameters.friction_model))
                    .show_ui(ui, |ui| {
                        for model in [FrictionModel::Simplified, FrictionModel::Coulomb] {
                            ui.selectable_value(
                                &mut integration_parameters.friction_model,
                                model,
                                format!("{model:?}"),
                            )
                            .on_hover_text(match model {
                                FrictionModel::Simplified => "Fast friction approximation",
                                FrictionModel::Coulomb => "Accurate Coulomb friction",
                            });
                        }
                    });
            });
        }

        ui.add_space(8.0);
    }

    // ─────────────────────────────────────────────────────────────────
    // ADVANCED (Rapier only)
    // ─────────────────────────────────────────────────────────────────
    if !is_physx {
        ui.label(RichText::new("Advanced").strong());
        ui.add_space(2.0);

        // Broad-phase
        ui.horizontal(|ui| {
            ui.label("Broad-phase:");
            let mut bp_changed = false;
            egui::ComboBox::from_id_salt("broad_phase")
                .width(120.0)
                .selected_text(match state.broad_phase_type {
                    RapierBroadPhaseType::BvhSubtreeOptimizer => "BVH (optimized)",
                    RapierBroadPhaseType::BvhWithoutOptimization => "BVH (basic)",
                })
                .show_ui(ui, |ui| {
                    for (bpt, label) in [
                        (RapierBroadPhaseType::BvhSubtreeOptimizer, "BVH (optimized)"),
                        (RapierBroadPhaseType::BvhWithoutOptimization, "BVH (basic)"),
                    ] {
                        bp_changed = ui
                            .selectable_value(&mut state.broad_phase_type, bpt, label)
                            .changed()
                            || bp_changed;
                    }
                });

            if bp_changed {
                harness.physics.broad_phase = state.broad_phase_type.init_broad_phase();
                state.action_flags.set(TestbedActionFlags::RESTART, true);
            }
        });

        ui.add(
            Slider::new(&mut integration_parameters.max_ccd_substeps, 0..=10).text("CCD substeps"),
        )
        .on_hover_text("Continuous collision detection substeps.");

        ui.add(
            Slider::new(&mut integration_parameters.min_island_size, 1..=10_000)
                .text("Min island sz."),
        )
        .on_hover_text("Minimum bodies per simulation island.");

        #[cfg(feature = "parallel")]
        {
            let mut num_threads = harness.state.num_threads();
            if ui
                .add(Slider::new(&mut num_threads, 1..=num_cpus::get_physical()).text("Threads"))
                .on_hover_text("Parallel solver threads.")
                .changed()
            {
                harness.state.set_num_threads(num_threads);
            }
        }

        ui.add_space(8.0);
    }
}

fn performance_tab(ui: &mut Ui, harness: &Harness) {
    // ─────────────────────────────────────────────────────────────────
    // SCENE INFO
    // ─────────────────────────────────────────────────────────────────
    ui.label(RichText::new("Scene").strong());
    ui.add_space(2.0);

    egui::Grid::new("scene_grid")
        .num_columns(2)
        .spacing([20.0, 2.0])
        .show(ui, |ui| {
            ui.label("Bodies:");
            ui.label(format!("{}", harness.physics.bodies.len()));
            ui.end_row();

            ui.label("Colliders:");
            ui.label(format!("{}", harness.physics.colliders.len()));
            ui.end_row();

            ui.label("Joints:");
            ui.label(format!("{}", harness.physics.impulse_joints.len()));
            ui.end_row();
        });

    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    // ─────────────────────────────────────────────────────────────────
    // TIMING - Full details
    // ─────────────────────────────────────────────────────────────────
    let counters = &harness.physics.pipeline.counters;
    let total_ms = counters.step_time_ms();
    let fps = if total_ms > 0.0 {
        (1000.0 / total_ms).round()
    } else {
        0.0
    };

    ui.label(RichText::new(format!("Total: {:.2}ms - {:.0} FPS", total_ms, fps)).strong());
    ui.add_space(4.0);

    // Collision detection
    egui::CollapsingHeader::new(format!(
        "Collision detection: {:.2}ms",
        counters.collision_detection_time_ms()
    ))
    .id_salt("collision_detection")
    .default_open(false)
    .show(ui, |ui| {
        ui.label(format!(
            "Broad-phase: {:.2}ms",
            counters.broad_phase_time_ms()
        ));
        ui.label(format!(
            "Final broad-phase: {:.2}ms",
            counters.cd.final_broad_phase_time.time_ms()
        ));
        ui.label(format!(
            "Narrow-phase: {:.2}ms",
            counters.narrow_phase_time_ms()
        ));
    });

    // Solver
    egui::CollapsingHeader::new(format!("Solver: {:.2}ms", counters.solver_time_ms()))
        .id_salt("solver")
        .default_open(false)
        .show(ui, |ui| {
            ui.label(format!(
                "Velocity assembly: {:.2}ms",
                counters.solver.velocity_assembly_time.time_ms()
            ));
            ui.label(format!(
                "  > Solver bodies: {:.2}ms",
                counters
                    .solver
                    .velocity_assembly_time_solver_bodies
                    .time_ms()
            ));
            ui.label(format!(
                "  > Constraints init: {:.2}ms",
                counters
                    .solver
                    .velocity_assembly_time_constraints_init
                    .time_ms()
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

    // CCD
    egui::CollapsingHeader::new(format!("CCD: {:.2}ms", counters.ccd_time_ms()))
        .id_salt("ccd")
        .default_open(false)
        .show(ui, |ui| {
            ui.label(format!("# of substeps: {}", counters.ccd.num_substeps));
            ui.label(format!(
                "TOI computation: {:.2}ms",
                counters.ccd.toi_computation_time.time_ms()
            ));
            ui.label(format!(
                "Broad-phase: {:.2}ms",
                counters.ccd.broad_phase_time.time_ms()
            ));
            ui.label(format!(
                "Narrow-phase: {:.2}ms",
                counters.ccd.narrow_phase_time.time_ms()
            ));
            ui.label(format!(
                "Solver: {:.2}ms",
                counters.ccd.solver_time.time_ms()
            ));
        });

    // Other timings
    ui.add_space(4.0);
    ui.label(format!(
        "Island computation: {:.2}ms",
        counters.island_construction_time_ms()
    ));
    ui.label(format!(
        "Active constraints collection: {:.2}ms",
        counters.stages.island_constraints_collection_time.time_ms()
    ));
    ui.label(format!(
        "Mass properties update: {:.2}ms",
        counters.update_time_ms()
    ));
    ui.label(format!(
        "User changes: {:.2}ms",
        counters.stages.user_changes.time_ms()
    ));
    ui.label(format!("Debug timer: {:.2}ms", counters.custom.time_ms()));

    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    // ─────────────────────────────────────────────────────────────────
    // SERIALIZATION INFO
    // ─────────────────────────────────────────────────────────────────
    egui::CollapsingHeader::new("Serialization Hashes")
        .default_open(false)
        .show(ui, |ui| {
            ui.label(
                egui::RichText::new(serialization_string(
                    harness.state.timestep_id,
                    &harness.physics,
                ))
                .small()
                .monospace(),
            );
        });
}

fn serialization_string(timestep_id: usize, physics: &PhysicsState) -> String {
    let t = Instant::now();
    let bf = bincode::serialize(&physics.broad_phase).unwrap();
    let nf = bincode::serialize(&physics.narrow_phase).unwrap();
    let bs = bincode::serialize(&physics.bodies).unwrap();
    let cs = bincode::serialize(&physics.colliders).unwrap();
    let js = bincode::serialize(&physics.impulse_joints).unwrap();
    let serialization_time = Instant::now() - t;
    let hash_bf = md5::compute(&bf);
    let hash_nf = md5::compute(&nf);
    let hash_bodies = md5::compute(&bs);
    let hash_colliders = md5::compute(&cs);
    let hash_joints = md5::compute(&js);
    format!(
        "Frame: {} ({:.1}ms)\n\
         Broad:     {:.1}KB {}\n\
         Narrow:    {:.1}KB {}\n\
         Bodies:    {:.1}KB {}\n\
         Colliders: {:.1}KB {}\n\
         Joints:    {:.1}KB {}",
        timestep_id,
        serialization_time.as_secs_f64() * 1000.0,
        bf.len() as f32 / 1000.0,
        format!("{hash_bf:?}").split_at(8).0,
        nf.len() as f32 / 1000.0,
        format!("{hash_nf:?}").split_at(8).0,
        bs.len() as f32 / 1000.0,
        format!("{hash_bodies:?}").split_at(8).0,
        cs.len() as f32 / 1000.0,
        format!("{hash_colliders:?}").split_at(8).0,
        js.len() as f32 / 1000.0,
        format!("{hash_joints:?}").split_at(8).0,
    )
}

fn example_settings_ui(ui_context: &egui::Context, state: &mut TestbedState) {
    if state.example_settings.is_empty() {
        return;
    }

    Window::new("Example Settings")
        .default_width(250.0)
        .show(ui_context, |ui| {
            let mut any_changed = false;

            for (name, value) in state.example_settings.iter_mut() {
                let prev_value = value.clone();
                match value {
                    SettingValue::Label(text) => {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(format!("{name}:")).strong());
                            ui.label(text.as_str());
                        });
                    }
                    SettingValue::F32 { value, range } => {
                        ui.add(Slider::new(value, range.clone()).text(name));
                    }
                    SettingValue::U32 { value, range } => {
                        ui.horizontal(|ui| {
                            if ui.small_button("-").on_hover_text("Decrease").clicked()
                                && *value > *range.start()
                            {
                                *value -= 1;
                            }
                            if ui.small_button("+").on_hover_text("Increase").clicked()
                                && *value < *range.end()
                            {
                                *value += 1;
                            }
                            ui.add(Slider::new(value, range.clone()).text(name));
                        });
                    }
                    SettingValue::Bool { value } => {
                        ui.checkbox(value, name);
                    }
                    SettingValue::String { value, range } => {
                        ComboBox::from_label(name)
                            .width(150.0)
                            .selected_text(&range[*value])
                            .show_ui(ui, |ui| {
                                for (id, option) in range.iter().enumerate() {
                                    ui.selectable_value(value, id, option);
                                }
                            });
                    }
                }

                any_changed = any_changed || *value != prev_value;
            }

            if any_changed {
                state.action_flags.set(TestbedActionFlags::RESTART, true);
            }
        });
}

#[cfg(feature = "profiler_ui")]
fn profiler_ui(_ui_context: &egui::Context) {
    #[cfg(feature = "unstable-puffin-pr-235")]
    {
        let window = egui::Window::new("Profiler");
        let window = window.default_open(false);
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
        window.show(_ui_context, |ui| {
            if ui.button("Rapier filter").clicked() {
                set_default_rapier_filter();
            }
            puffin_egui::profiler_ui(ui);
        });
    }
}
