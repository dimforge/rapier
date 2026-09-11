use rapier::math::Real;
use rapier::pipeline::PhysicsWorld;

use crate::debug_render::DebugRenderPipelineResource;
use crate::physics::RapierBroadPhaseType;
use crate::testbed::state::Transition;
use crate::testbed::{RunMode, TestbedActionFlags, TestbedState, TestbedStateFlags, UiTab};

pub use egui;

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

    ctx.global_style_mut(|style| {
        let v = &mut style.visuals;
        v.dark_mode = false;

        v.widgets.noninteractive.bg_fill = faint_bg;
        v.widgets.noninteractive.weak_bg_fill = faint_bg;
        v.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, stroke_color);
        v.widgets.noninteractive.corner_radius = rounding;
        v.widgets.noninteractive.fg_stroke = Stroke::new(1.0_f32, text_color);

        v.widgets.inactive.bg_fill = widget_bg;
        v.widgets.inactive.weak_bg_fill = widget_bg;
        v.widgets.inactive.bg_stroke = Stroke::new(1.0_f32, stroke_color);
        v.widgets.inactive.corner_radius = small_rounding;
        v.widgets.inactive.fg_stroke = Stroke::new(1.0_f32, text_color);

        v.widgets.hovered.bg_fill = widget_bg_hover;
        v.widgets.hovered.weak_bg_fill = widget_bg_hover;
        v.widgets.hovered.bg_stroke = Stroke::new(1.0_f32, stroke_hover);
        v.widgets.hovered.corner_radius = small_rounding;
        v.widgets.hovered.fg_stroke = Stroke::new(1.5_f32, text_color);

        v.widgets.active.bg_fill = widget_bg_active;
        v.widgets.active.weak_bg_fill = widget_bg_active;
        v.widgets.active.bg_stroke = Stroke::new(1.0_f32, accent);
        v.widgets.active.corner_radius = small_rounding;
        v.widgets.active.fg_stroke = Stroke::new(2.0_f32, accent_active);

        v.widgets.open.bg_fill = widget_bg;
        v.widgets.open.weak_bg_fill = widget_bg;
        v.widgets.open.bg_stroke = Stroke::new(1.0_f32, stroke_color);
        v.widgets.open.corner_radius = small_rounding;
        v.widgets.open.fg_stroke = Stroke::new(1.0_f32, text_color);

        v.selection.bg_fill = accent.gamma_multiply(0.25);
        v.selection.stroke = Stroke::new(1.0_f32, accent);

        v.hyperlink_color = accent;
        v.faint_bg_color = faint_bg;
        v.extreme_bg_color = extreme_bg;
        v.code_bg_color = Color32::from_rgb(230, 230, 220);
        v.warn_fg_color = Color32::from_rgb(180, 120, 60);
        v.error_fg_color = Color32::from_rgb(180, 70, 70);

        v.window_corner_radius = CornerRadius::same(8);
        v.window_fill = window_fill;
        v.window_stroke = Stroke::new(1.0_f32, stroke_color);

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
    world: &mut PhysicsWorld,
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
                ui.selectable_value(&mut state.selected_tab, UiTab::DebugRender, "Debug");
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
                        settings_tab(ui, state, world);
                    }
                    UiTab::Performance => {
                        performance_tab(ui, state, world);
                    }
                    UiTab::DebugRender => {
                        debug_render_tab(ui, debug_render);
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

                ui.label(RichText::new(format!("#{}", state.timestep_id)).monospace())
                    .on_hover_text("Steps run since the example started");

                // Restart
                if ui
                    .button("Restart")
                    .on_hover_text("Restart example (R)")
                    .clicked()
                {
                    state.preserve_settings_on_switch = true;
                    state.transition = Some(Transition::Switch);
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

                ui.separator();

                if ui
                    .button("Frame all")
                    .on_hover_text("Recenter the camera so the entire scene fills the view")
                    .clicked()
                {
                    state
                        .action_flags
                        .set(TestbedActionFlags::FRAME_SCENE, true);
                }
            });
        });
}

fn examples_tab(ui: &mut Ui, state: &mut TestbedState) {
    // Navigation row
    ui.horizontal(|ui| {
        // Previous/Next buttons (navigate in display order)
        if ui
            .add_enabled(state.selected_display_index > 0, egui::Button::new("<"))
            .on_hover_text("Previous example")
            .clicked()
        {
            state.selected_display_index -= 1;
            state.transition = Some(Transition::Switch);
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
            state.transition = Some(Transition::Switch);
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
                        state.transition = Some(Transition::Switch);
                    }
                }
            });
    }
}

fn settings_tab(ui: &mut Ui, state: &mut TestbedState, world: &mut PhysicsWorld) {
    let integration_parameters = &mut world.integration_parameters;

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

    #[cfg(feature = "dim3")]
    {
        let mut smooth = state
            .flags
            .contains(TestbedStateFlags::SMOOTH_MESH_COLLIDERS);
        if ui
            .checkbox(&mut smooth, "Smooth mesh colliders")
            .on_hover_text(
                "Shared vertices and per-vertex normals on mesh colliders, soft bodies included \
                 (flat shading otherwise).",
            )
            .changed()
        {
            state
                .flags
                .set(TestbedStateFlags::SMOOTH_MESH_COLLIDERS, smooth);
        }
    }

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

    // Gravity slider — operates along the current up-axis so the
    // slider's "down" matches the camera's "down". Scenes that switch
    // to Z-up (e.g. the MJCF demo) get gravity along -Z instead of the
    // hard-coded -Y. Any orthogonal components of gravity are
    // preserved when the slider moves.
    let up = state.up_axis;
    let mut gravity_along_up = world.gravity.dot(up);
    if ui
        .add(Slider::new(&mut gravity_along_up, 0.0..=-200.0).text("Gravity"))
        .on_hover_text("Gravity (m/s^2) along the up-axis. Default: -9.81")
        .changed()
    {
        let current = world.gravity.dot(up);
        world.gravity += up * (gravity_along_up - current);
    }

    // Sleeping toggle (whole-island sleep is the only strategy).
    let mut sleep = state.flags.contains(TestbedStateFlags::SLEEP);
    if ui
        .checkbox(&mut sleep, "Sleeping")
        .on_hover_text(
            "An island (touching-contact/joint connected component) falls \
             asleep once all of its bodies settled; it sleeps and wakes as a \
             unit.",
        )
        .changed()
    {
        state.flags.set(TestbedStateFlags::SLEEP, sleep);
        if sleep {
            // Wake everything so the scene re-settles from a clean state.
            // Field-borrow version of `PhysicsWorld::wake_up_all`
            // (`integration_parameters` is still borrowed from `world` here).
            let handles: Vec<_> = world.bodies.iter().map(|(h, _)| h).collect();
            for handle in handles {
                world.islands.wake_up(&mut world.bodies, handle, true);
            }
        }
        // Disabling is applied by `handle_sleep_settings` reacting to the
        // flag change (wakes everything and negates the sleep thresholds).
    }

    ui.add_space(8.0);

    // ─────────────────────────────────────────────────────────────────
    // SOLVER
    // ─────────────────────────────────────────────────────────────────
    ui.label(RichText::new("Solver").strong());
    ui.add_space(2.0);

    ui.add(Slider::new(&mut integration_parameters.num_solver_iterations, 1..=10).text("Substeps"))
        .on_hover_text("Main solver iterations. Higher = more stable stacking.");

    {
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
    {
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
                0.01..=100.0,
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
    {
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
                world.broad_phase = state.broad_phase_type.init_broad_phase();
                state.preserve_settings_on_switch = true;
                state.transition = Some(Transition::Switch);
            }
        });

        ui.add(
            Slider::new(&mut integration_parameters.max_ccd_substeps, 0..=10).text("CCD substeps"),
        )
        .on_hover_text("Continuous collision detection substeps.");

        if world.soft_bodies.iter().next().is_some() {
            // Edits the testbed-owned copy, stamped onto the world every frame: the
            // choices survive demo restarts and switches (and app relaunches, through the
            // saved testbed state).
            soft_recovery_section(ui, &mut state.soft_recovery);
        }

        #[cfg(feature = "parallel")]
        {
            let max_threads = num_cpus::get();
            let mut num_threads = state
                .physics_thread_pool
                .as_ref()
                .map(|pool| pool.current_num_threads())
                .unwrap_or_else(|| (max_threads.saturating_sub(1)).clamp(1, 8));

            if ui
                .add(Slider::new(&mut num_threads, 1..=max_threads).text("Solver threads"))
                .on_hover_text(
                    "Worker threads running the physics step. On heterogeneous CPUs,                      the performance-core count works best (efficiency cores stall                      the solver's barrier-paced stages).",
                )
                .changed()
            {
                if let Err(e) = world.configure_thread_pool(num_threads) {
                    eprintln!("Failed to build the physics thread pool: {e}");
                }
                if let Some(pool) = world.thread_pool()
                {
                    state.physics_thread_pool = Some(pool);
                }
            }
        }

        ui.add_space(8.0);
    }
}

/// The soft-body tangle detection and recovery toggles and knobs (see `SoftRecoverySettings`),
/// edited live on the running world; shown whenever the world contains soft bodies. Re-enabling
/// a mechanism does not wake bodies that fell asleep while it was off.
fn soft_recovery_section(ui: &mut Ui, r: &mut rapier::dynamics::SoftRecoverySettings) {
    use rapier::dynamics::SoftPatchConstraints;
    egui::CollapsingHeader::new("Soft recovery")
        .default_open(false)
        .show(ui, |ui| {
            ui.label("Prevention");
            ui.checkbox(&mut r.authored_velocity_margin, "Authored-velocity margin");
            ui.checkbox(&mut r.edge_speculation, "Edge speculation (3D closed pairs)");
            ui.separator();
            ui.label("Detection");
            ui.checkbox(&mut r.inverted_cell_detection, "Inverted cells");
            ui.checkbox(&mut r.self_crossing_detection, "Self-crossings");
            ui.checkbox(&mut r.detection_motion_gating, "Self-crossing sweep motion gating");
            ui.checkbox(&mut r.cross_body_detection, "Cross-body crossings");
            ui.separator();
            ui.label("Passive stand-down");
            ui.checkbox(&mut r.self_stand_down, "Self stand-down");
            ui.checkbox(&mut r.cross_body_expel_gate, "Cross expel-only gate");
            ui.checkbox(&mut r.edge_stand_down, "Edge-pass stand-down");
            ui.checkbox(&mut r.crossing_repulsion, "Crossing repulsion (repel, not drop)");
            ui.checkbox(&mut r.crossing_repulsion_guide, "Repulsion guided by the volume normal");
            ui.checkbox(
                &mut r.crossing_repulsion_self_guide,
                "Self-repulsion guided by the fold's volume normal",
            );
            ui.separator();
            ui.label("Intersection-volume constraints (closed surfaces)");
            ui.checkbox(&mut r.overlap_constraints, "Overlap constraints (master)");
            ui.checkbox(&mut r.overlap_skin_volume, "Skin volume");
            ui.add(
                Slider::new(&mut r.overlap_kept_depth, 0.0..=1.0)
                    .text("Kept skin overlap (fraction of skins)"),
            );
            ui.checkbox(&mut r.overlap_normal_push, "Push along the normal instead of the gradients");
            ui.checkbox(&mut r.overlap_self_regions, "Self-overlaps between distinct regions");
            ui.horizontal(|ui| {
                ui.label("Per-point constraints on the patch's features");
                for (policy, name) in [
                    (SoftPatchConstraints::Keep, "Keep"),
                    (SoftPatchConstraints::StandDown, "Stand down"),
                    (SoftPatchConstraints::AlongNormal, "Along the normal"),
                ] {
                    ui.selectable_value(&mut r.overlap_patch_constraints, policy, name);
                }
            });
            ui.checkbox(&mut r.overlap_multi_volume, "Multi-volume grid (section 5)");
            ui.add(Slider::new(&mut r.overlap_split, 1..=8).text("Grid cells per tangent axis"));
            ui.checkbox(&mut r.overlap_rigid, "Against rigid colliders");
            ui.checkbox(&mut r.overlap_skip_self_tangled, "Skip self-crossed meshes");
            ui.checkbox(&mut r.overlap_edge_stand_down, "Edge constraints stand down on owned pairs (3D)");
            ui.add(
                Slider::new(&mut r.recovery_pace, 0.05..=8.0)
                    .logarithmic(true)
                    .text("Recovery pace (length units / s)"),
            );
            ui.add(
                Slider::new(&mut r.overlap_constraint_pace, 0.05..=64.0)
                    .logarithmic(true)
                    .text("Constraint impulse bound (x recovery pace)"),
            );
            ui.add(Slider::new(&mut r.overlap_patience, 10..=2000).text("Stall patience (steps)"));
            ui.add(
                Slider::new(&mut r.overlap_progress_margin, 0.0..=0.5)
                    .text("Progress margin (fraction)"),
            );
            if ui.button("Reset to defaults").clicked() {
                *r = Default::default();
            }
        });
}

/// One of the debug renderer's colors, as a color picker. The style stores HSLA; the picker
/// works in sRGB, so the value round-trips through [`crate::debug_render::rgb_to_hsla`].
fn debug_color_picker(ui: &mut Ui, label: &str, color: &mut rapier::pipeline::DebugColor) {
    let rgba = crate::debug_render::hsla_to_rgb(color[0], color[1], color[2], color[3]);
    let mut srgba = rgba.map(|c| (c.clamp(0.0, 1.0) * 255.0).round() as u8);
    ui.horizontal(|ui| {
        if ui.color_edit_button_srgba_unmultiplied(&mut srgba).changed() {
            *color = crate::debug_render::rgb_to_hsla(srgba.map(|c| c as f32 / 255.0));
        }
        ui.label(label);
    });
}

/// One of the debug renderer's color multipliers (a per-HSLA-component scale, not a color).
fn debug_multiplier_row(ui: &mut Ui, label: &str, color: &mut rapier::pipeline::DebugColor) {
    ui.horizontal(|ui| {
        for (component, prefix) in color.iter_mut().zip(["h ", "s ", "l ", "a "]) {
            ui.add(
                egui::DragValue::new(component)
                    .speed(0.01)
                    .range(0.0..=1.0)
                    .prefix(prefix),
            );
        }
        ui.label(label);
    });
}

/// What the debug renderer draws, and how it draws it.
fn debug_render_tab(ui: &mut Ui, debug_render: &mut DebugRenderPipelineResource) {
    use rapier::pipeline::DebugRenderMode;

    ui.checkbox(&mut debug_render.enabled, "Debug render")
        .on_hover_text("Draw the physics state over the scene (wireframes, joints, contacts).");
    ui.add_space(8.0);

    // ─────────────────────────────────────────────────────────────────
    // WHAT TO DRAW
    // ─────────────────────────────────────────────────────────────────
    ui.label(RichText::new("What to draw").strong());
    ui.add_space(2.0);

    {
        // The composite `JOINTS` flag is left out: its two halves are here.
        const FLAGS: &[(DebugRenderMode, &str, &str)] = &[
            (
                DebugRenderMode::COLLIDER_SHAPES,
                "Collider shapes",
                "The colliders' outlines, colored by their body type.",
            ),
            (
                DebugRenderMode::COLLIDER_AABBS,
                "Collider AABBs",
                "The bounding boxes the broad phase sees.",
            ),
            (
                DebugRenderMode::RIGID_BODY_AXES,
                "Rigid-body axes",
                "The local frame of every rigid body, at its center of mass.",
            ),
            (
                DebugRenderMode::IMPULSE_JOINTS,
                "Impulse joints",
                "The anchors of the impulse joints, and their separation.",
            ),
            (
                DebugRenderMode::MULTIBODY_JOINTS,
                "Multibody joints",
                "The anchors of the multibody joints, and their separation.",
            ),
            (
                DebugRenderMode::CONTACTS,
                "Contacts",
                "The geometric contact points and their normals.",
            ),
            (
                DebugRenderMode::SOLVER_CONTACTS,
                "Solver contacts",
                "The contact points the solver actually used this step.",
            ),
            (
                DebugRenderMode::SOFT_BODIES,
                "Soft bodies",
                "The soft bodies' elements (structural and cell edges), their cluster frames, \
                 and their soft-vs-soft contacts.",
            ),
            (
                DebugRenderMode::PSEUDO_NORMALS,
                "Pseudo-normals",
                "The pseudo-normals of the oriented triangle-meshes and polylines, at their \
                 vertices and edge midpoints.",
            ),
            (
                DebugRenderMode::SOFT_VOLUME_CONTACTS,
                "Soft volume constraints",
                "The soft bodies' intersection-volume constraints: each one's normal at its \
                 patch center, and the volume gradient at every particle it acts on.",
            ),
            (
                DebugRenderMode::SOFT_BODY_STRESS,
                "Soft-body stress",
                "Colors the soft bodies' elements by their load, blue when slack to red at \
                 their tear threshold (by their stretch, full at 50%, for a body that never \
                 tears). Needs the soft bodies drawn.",
            ),
        ];

        for (flag, label, hover) in FLAGS {
            let mut on = debug_render.pipeline.mode.contains(*flag);
            if ui.checkbox(&mut on, *label).on_hover_text(*hover).changed() {
                debug_render.pipeline.mode.set(*flag, on);
            }
        }

        ui.horizontal(|ui| {
            if ui.button("All").clicked() {
                debug_render.pipeline.mode = DebugRenderMode::all();
            }
            if ui.button("None").clicked() {
                debug_render.pipeline.mode = DebugRenderMode::empty();
            }
            if ui.button("Default").clicked() {
                debug_render.pipeline.mode = DebugRenderMode::default();
            }
        });

        ui.add_space(8.0);

        // ─────────────────────────────────────────────────────────────
        // STYLE
        // ─────────────────────────────────────────────────────────────
        ui.label(RichText::new("Style").strong());
        ui.add_space(2.0);

        let style = &mut debug_render.pipeline.style;
        ui.add(Slider::new(&mut style.subdivisions, 2..=64).text("Subdivisions"))
            .on_hover_text("Segments approximating a curved shape (balls, capsules, cones).");
        ui.add(Slider::new(&mut style.border_subdivisions, 1..=32).text("Border subdivisions"))
            .on_hover_text("Segments approximating the rounded border of a round shape.");
        ui.add(
            Slider::new(&mut style.rigid_body_axes_length, 0.0..=2.0).text("Axes length"),
        )
        .on_hover_text("Length of the rigid-body axes.");
        ui.add(
            Slider::new(&mut style.contact_normal_length, 0.0..=1.0).text("Normal length"),
        )
        .on_hover_text("Length of the contact normals.");
        ui.add(
            Slider::new(&mut style.pseudo_normal_length, 0.0..=1.0).text("Pseudo-normal length"),
        )
        .on_hover_text("Length of the meshes' pseudo-normals.");

        ui.collapsing("Colors", |ui| {
            debug_color_picker(ui, "Dynamic colliders", &mut style.collider_dynamic_color);
            debug_color_picker(ui, "Fixed colliders", &mut style.collider_fixed_color);
            debug_color_picker(ui, "Kinematic colliders", &mut style.collider_kinematic_color);
            debug_color_picker(ui, "Parentless colliders", &mut style.collider_parentless_color);
            debug_color_picker(ui, "Collider AABBs", &mut style.collider_aabb_color);
            debug_color_picker(ui, "Impulse joint anchors", &mut style.impulse_joint_anchor_color);
            debug_color_picker(
                ui,
                "Impulse joint separation",
                &mut style.impulse_joint_separation_color,
            );
            debug_color_picker(
                ui,
                "Multibody joint anchors",
                &mut style.multibody_joint_anchor_color,
            );
            debug_color_picker(
                ui,
                "Multibody joint separation",
                &mut style.multibody_joint_separation_color,
            );
            debug_color_picker(ui, "Contact depth", &mut style.contact_depth_color);
            debug_color_picker(ui, "Contact normals", &mut style.contact_normal_color);
            debug_color_picker(ui, "Soft-body elements", &mut style.soft_body_element_color);
            debug_color_picker(ui, "Soft-body frames", &mut style.soft_body_frame_color);
            debug_color_picker(
                ui,
                "Vertex pseudo-normals",
                &mut style.vertex_pseudo_normal_color,
            );
            debug_color_picker(
                ui,
                "Edge pseudo-normals",
                &mut style.edge_pseudo_normal_color,
            );

            ui.add_space(4.0);
            ui.label("Multipliers (per HSLA component)");
            debug_multiplier_row(ui, "Sleeping", &mut style.sleep_color_multiplier);
            debug_multiplier_row(ui, "Sleep-ready", &mut style.sleep_eligible_color_multiplier);
            debug_multiplier_row(ui, "Disabled", &mut style.disabled_color_multiplier);
            if ui.button("Reset colors").clicked() {
                let default = rapier::pipeline::DebugRenderStyle::default();
                // The scalars above are edited separately: only the colors are reset.
                *style = rapier::pipeline::DebugRenderStyle {
                    subdivisions: style.subdivisions,
                    border_subdivisions: style.border_subdivisions,
                    rigid_body_axes_length: style.rigid_body_axes_length,
                    contact_normal_length: style.contact_normal_length,
                    ..default
                };
            }
        });
    }
}

fn performance_tab(ui: &mut Ui, state: &TestbedState, world: &PhysicsWorld) {
    // ─────────────────────────────────────────────────────────────────
    // SCENE INFO
    // ─────────────────────────────────────────────────────────────────
    ui.label(RichText::new("Scene").strong());
    ui.add_space(2.0);

    let num_contacts: usize = world
        .narrow_phase
        .contact_pairs()
        .map(|pair| pair.manifolds().iter().map(|m| m.points.len()).sum::<usize>())
        .sum();

    let num_sleeping = world
        .rigid_bodies()
        .filter(|(_, rb)| rb.is_sleeping())
        .count();

    egui::Grid::new("scene_grid")
        .num_columns(2)
        .spacing([20.0, 2.0])
        .show(ui, |ui| {
            ui.label("Step:");
            ui.label(format!("{}", state.timestep_id));
            ui.end_row();

            ui.label("Bodies:");
            ui.label(format!("{}", world.bodies.len()));
            ui.end_row();

            ui.label("Sleeping:");
            ui.label(format!("{}", num_sleeping));
            ui.end_row();

            ui.label("Colliders:");
            ui.label(format!("{}", world.colliders.len()));
            ui.end_row();

            ui.label("Joints:");
            ui.label(format!("{}", world.impulse_joints.len()));
            ui.end_row();

            ui.label("Contacts:");
            ui.label(format!("{}", num_contacts));
            ui.end_row();
        });

    // ─────────────────────────────────────────────────────────────────
    // SERIALIZATION INFO
    // ─────────────────────────────────────────────────────────────────
    egui::CollapsingHeader::new("Serialization Hashes")
        .default_open(false)
        .show(ui, |ui| {
            ui.label(
                egui::RichText::new(serialization_string(world))
                    .small()
                    .monospace(),
            );
        });

    ui.add_space(8.0);
    ui.separator();
    ui.add_space(4.0);

    // ─────────────────────────────────────────────────────────────────
    // TIMING - Full details
    // ─────────────────────────────────────────────────────────────────
    let counters = &world.physics_pipeline.counters;
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
}

fn serialization_string(world: &PhysicsWorld) -> String {
    let t = Instant::now();
    let bf = bincode::serialize(&world.broad_phase).unwrap();
    let nf = bincode::serialize(&world.narrow_phase).unwrap();
    let bs = bincode::serialize(&world.bodies).unwrap();
    let cs = bincode::serialize(&world.colliders).unwrap();
    let js = bincode::serialize(&world.impulse_joints).unwrap();
    let serialization_time = Instant::now() - t;
    let hash_bf = md5::compute(&bf);
    let hash_nf = md5::compute(&nf);
    let hash_bodies = md5::compute(&bs);
    let hash_colliders = md5::compute(&cs);
    let hash_joints = md5::compute(&js);
    format!(
        "Serialized ({:.1}ms)\n\
         Broad:     {:.1}KB {}\n\
         Narrow:    {:.1}KB {}\n\
         Bodies:    {:.1}KB {}\n\
         Colliders: {:.1}KB {}\n\
         Joints:    {:.1}KB {}",
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
            // Settings the example reads live (e.g. via a callback): changing
            // them updates the value but must not restart the simulation.
            let non_restart = state.example_settings.non_restart_keys().clone();

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
                    SettingValue::String {
                        value,
                        range,
                        display_mode,
                    } => match display_mode {
                        crate::settings::StringDisplayMode::ComboBox => {
                            let n = range.len();
                            ui.horizontal(|ui| {
                                // `<` / `>` step to the previous / next option
                                // without opening the dropdown (mirrors the `List`
                                // mode steppers and the `U32` `-` / `+` pair).
                                let can_prev = *value > 0;
                                let can_next = *value + 1 < n;
                                if ui
                                    .add_enabled(can_prev, egui::Button::new("<"))
                                    .on_hover_text("Previous")
                                    .clicked()
                                {
                                    *value -= 1;
                                }
                                if ui
                                    .add_enabled(can_next, egui::Button::new(">"))
                                    .on_hover_text("Next")
                                    .clicked()
                                {
                                    *value += 1;
                                }
                                ComboBox::from_label(name)
                                    .width(150.0)
                                    .selected_text(&range[*value])
                                    .show_ui(ui, |ui| {
                                        // Bound the popup height so a long option
                                        // list scrolls instead of being clipped by
                                        // the screen edge (which otherwise hides all
                                        // but the first couple of entries when the
                                        // settings window sits low on screen).
                                        egui::ScrollArea::vertical().max_height(1600.0).show(
                                            ui,
                                            |ui| {
                                                for (id, option) in range.iter().enumerate() {
                                                    ui.selectable_value(value, id, option);
                                                }
                                            },
                                        );
                                    });
                            });
                        }
                        crate::settings::StringDisplayMode::List => {
                            let current = range.get(*value).cloned().unwrap_or_default();
                            let n = range.len();
                            ui.horizontal(|ui| {
                                // `<` / `>` step through the list one
                                // entry at a time. Mirrors the `-` / `+`
                                // pair on `U32` settings.
                                let can_prev = n > 0 && *value > 0;
                                let can_next = n > 0 && *value + 1 < n;
                                if ui
                                    .add_enabled(can_prev, egui::Button::new("<"))
                                    .on_hover_text("Previous")
                                    .clicked()
                                {
                                    *value -= 1;
                                }
                                if ui
                                    .add_enabled(can_next, egui::Button::new(">"))
                                    .on_hover_text("Next")
                                    .clicked()
                                {
                                    *value += 1;
                                }
                                ui.label(RichText::new(format!("{name}:")).strong());
                                ui.label(current);
                            });
                            // Bound the height so a long option list
                            // doesn't crowd out the rest of the panel.
                            egui::ScrollArea::vertical()
                                .id_salt(name.as_str())
                                .max_height(300.0)
                                .auto_shrink([false, true])
                                .show(ui, |ui| {
                                    for (id, opt) in range.iter().enumerate() {
                                        let is_selected = *value == id;
                                        let text = if is_selected {
                                            RichText::new(opt).strong()
                                        } else {
                                            RichText::new(opt)
                                        };
                                        if ui.selectable_label(is_selected, text).clicked()
                                            && !is_selected
                                        {
                                            *value = id;
                                        }
                                    }
                                });
                        }
                    },
                }

                // A change to a non-restart setting still updates the value
                // (the widget mutated it above) but doesn't restart the sim.
                if *value != prev_value && !non_restart.contains(name.as_str()) {
                    any_changed = true;
                }
            }

            if any_changed {
                // A restart-triggering setting changed: re-run the example,
                // preserving the user's setting edits.
                state.preserve_settings_on_switch = true;
                state.transition = Some(Transition::Switch);
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
