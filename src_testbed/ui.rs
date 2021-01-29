use kiss3d::conrod::{self, Borderable, Colorable, Labelable, Positionable, Sizeable, Widget};
use kiss3d::window::Window;
use rapier::dynamics::IntegrationParameters;

use crate::harness::RunState;
use crate::testbed::{RunMode, TestbedActionFlags, TestbedState, TestbedStateFlags};

const SIDEBAR_W: f64 = 200.0;
const ELEMENT_W: f64 = SIDEBAR_W - 20.0;
const ELEMENT_H: f64 = 20.0;
const VSPACE: f64 = 4.0;
const TITLE_VSPACE: f64 = 4.0;
const LEFT_MARGIN: f64 = 10.0;
const ALPHA: f32 = 0.9;

widget_ids! {
    pub struct ConrodIds {
        canvas,
        title_backends_list,
        title_demos_list,
        title_slider_vel_iter,
        title_slider_pos_iter,
        title_slider_num_threads,
        title_slider_ccd_substeps,
        title_slider_min_island_size,
        title_warmstart_coeff,
        title_frequency,
        backends_list,
        demos_list,
        button_pause,
        button_single_step,
        button_restart,
        button_quit,
        button_prev_example,
        button_next_example,
        button_take_snapshot,
        button_restore_snapshot,
        slider_vel_iter,
        slider_pos_iter,
        slider_num_threads,
        slider_ccd_substeps,
        slider_min_island_size,
        slider_warmstart_coeff,
        slider_frequency,
        toggle_sleep,
        toggle_warm_starting,
        toggle_sub_stepping,
        toggle_shapes,
        toggle_joints,
        toggle_aabbs,
        toggle_contact_points,
        toggle_contact_normals,
        toggle_center_of_masses,
        toggle_statistics,
        toggle_profile,
        toggle_debug,
        toggle_wireframe,
        separator0,
        separator1,
        separator2,
    }
}

pub struct TestbedUi {
    ids: ConrodIds,
}

impl TestbedUi {
    pub fn new(window: &mut Window) -> Self {
        use conrod::position::{Align, Direction, Padding, Position, Relative};

        let mut ui = window.conrod_ui_mut();
        ui.theme = conrod::Theme {
            name: "Testbed theme".to_string(),
            padding: Padding::none(),
            x_position: Position::Relative(Relative::Align(Align::Start), None),
            y_position: Position::Relative(Relative::Direction(Direction::Backwards, 20.0), None),
            background_color: conrod::color::DARK_CHARCOAL.alpha(ALPHA),
            shape_color: conrod::color::LIGHT_CHARCOAL.alpha(ALPHA),
            border_color: conrod::color::BLACK.alpha(ALPHA),
            border_width: 0.0,
            label_color: conrod::color::WHITE.alpha(ALPHA),
            font_id: None,
            font_size_large: 15,
            font_size_medium: 11,
            font_size_small: 8,
            widget_styling: conrod::theme::StyleMap::default(),
            mouse_drag_threshold: 0.0,
            double_click_threshold: std::time::Duration::from_millis(500),
        };

        Self {
            ids: ConrodIds::new(ui.widget_id_generator()),
        }
    }

    pub fn update(
        &mut self,
        window: &mut Window,
        integration_parameters: &mut IntegrationParameters,
        state: &mut TestbedState,
        _run_state: &mut RunState,
    ) {
        let ui_root = window.conrod_ui().window;
        let mut ui = window.conrod_ui_mut().set_widgets();
        conrod::widget::Canvas::new()
            //            .title_bar("Demos")
            //            .title_bar_color(conrod::color::Color::Rgba(1.0, 0.0, 0.0, 1.0))
            //            .pad(100.0)
            //            .pad_left(MARGIN)
            //            .pad_right(MARGIN)
            .scroll_kids_vertically()
            .mid_right_with_margin(10.0)
            .w(SIDEBAR_W)
            .padded_h_of(ui_root, 10.0)
            .set(self.ids.canvas, &mut ui);

        // NOTE: If examples_names is empty, we can't change the backend because
        // we have no way to properly restart the simulation.
        if state.backend_names.len() > 1 && !state.example_names.is_empty() {
            /*
             * Backend drop-down.
             */
            conrod::widget::Text::new("Select backend:")
                .top_left_with_margins_on(self.ids.canvas, VSPACE, LEFT_MARGIN)
                .set(self.ids.title_backends_list, &mut ui);

            for selected in conrod::widget::DropDownList::new(
                &state.backend_names,
                Some(state.selected_backend),
            )
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_backends_list, TITLE_VSPACE)
            .left_justify_label()
            .w_h(ELEMENT_W, ELEMENT_H)
            .color(conrod::color::LIGHT_CHARCOAL) // No alpha.
            .set(self.ids.backends_list, &mut ui)
            {
                if selected != state.selected_backend {
                    #[cfg(all(feature = "dim3", feature = "other-backends"))]
                    fn is_physx(id: usize) -> bool {
                        id == crate::testbed::PHYSX_BACKEND_PATCH_FRICTION
                            || id == crate::testbed::PHYSX_BACKEND_TWO_FRICTION_DIR
                    }

                    #[cfg(all(feature = "dim3", feature = "other-backends"))]
                    if (is_physx(state.selected_backend) && !is_physx(selected))
                        || (!is_physx(state.selected_backend) && is_physx(selected))
                    {
                        // PhysX defaults (4 position iterations, 1 velocity) are the
                        // opposite of rapier's (4 velocity iterations, 1 position).
                        std::mem::swap(
                            &mut integration_parameters.max_position_iterations,
                            &mut integration_parameters.max_velocity_iterations,
                        );
                    }

                    state.selected_backend = selected;
                    state
                        .action_flags
                        .set(TestbedActionFlags::BACKEND_CHANGED, true)
                }
            }

            separator(
                self.ids.canvas,
                self.ids.backends_list,
                self.ids.separator0,
                &mut ui,
            );
        } else {
            conrod::widget::Text::new("")
                .top_left_with_margins_on(self.ids.canvas, 0.0, LEFT_MARGIN)
                .set(self.ids.separator0, &mut ui);
        }

        let display_ticks = state.example_names.len() > 1;
        let _select_example_title = if display_ticks {
            "Select example:"
        } else {
            "Current example:"
        };
        let tick_width = if display_ticks { 20.0 } else { 0.0 };

        /*
         * Examples drop-down.
         */
        conrod::widget::Text::new("Select example:")
            .down_from(self.ids.separator0, VSPACE)
            //                .top_left_with_margins_on(self.ids.canvas, VSPACE, LEFT_MARGIN)
            //            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.title_demos_list, &mut ui);

        for selected in
            conrod::widget::DropDownList::new(&state.example_names, Some(state.selected_example))
                //            .mid_top_with_margin_on(self.ids.canvas, 20.0)
                .align_middle_x_of(self.ids.canvas)
                .down_from(self.ids.title_demos_list, TITLE_VSPACE)
                //            .right_from(self.ids.button_prev_example, 0.0)
                .left_justify_label()
                .w_h(ELEMENT_W - tick_width, ELEMENT_H)
                .color(conrod::color::LIGHT_CHARCOAL) // No alpha.
                .set(self.ids.demos_list, &mut ui)
        {
            if selected != state.selected_example {
                state.selected_example = selected;
                state
                    .action_flags
                    .set(TestbedActionFlags::EXAMPLE_CHANGED, true)
            }
        }

        if display_ticks {
            for _click in conrod::widget::Button::new()
                .label("<")
                .align_middle_x_of(self.ids.canvas)
                .left_from(self.ids.demos_list, 0.0)
                .w_h(10.0, ELEMENT_H)
                .enabled(state.selected_example > 0)
                .color(conrod::color::LIGHT_CHARCOAL) // No alpha.
                .set(self.ids.button_prev_example, &mut ui)
            {
                if state.selected_example > 0 {
                    state.selected_example -= 1;
                    state
                        .action_flags
                        .set(TestbedActionFlags::EXAMPLE_CHANGED, true)
                }
            }

            for _click in conrod::widget::Button::new()
                .label(">")
                .align_middle_x_of(self.ids.canvas)
                .right_from(self.ids.demos_list, 0.0)
                .w_h(10.0, ELEMENT_H)
                .enabled(state.selected_example + 1 < state.example_names.len())
                .color(conrod::color::LIGHT_CHARCOAL) // No alpha.
                .set(self.ids.button_next_example, &mut ui)
            {
                if state.selected_example + 1 < state.example_names.len() {
                    state.selected_example += 1;
                    state
                        .action_flags
                        .set(TestbedActionFlags::EXAMPLE_CHANGED, true)
                }
            }
        }

        separator(
            self.ids.canvas,
            self.ids.demos_list,
            self.ids.separator1,
            &mut ui,
        );

        let curr_vel_iters = integration_parameters.max_velocity_iterations;
        let curr_pos_iters = integration_parameters.max_position_iterations;
        #[cfg(feature = "parallel")]
        let curr_num_threads = _run_state.num_threads;
        let curr_max_ccd_substeps = integration_parameters.max_ccd_substeps;
        let curr_min_island_size = integration_parameters.min_island_size;
        let curr_warmstart_coeff = integration_parameters.warmstart_coeff;
        let curr_frequency = integration_parameters.inv_dt().round() as usize;

        conrod::widget::Text::new("Vel. Iters.:")
            .down_from(self.ids.separator1, VSPACE)
            .set(self.ids.title_slider_vel_iter, &mut ui);

        for val in conrod::widget::Slider::new(curr_vel_iters as f32, 0.0, 200.0)
            .label(&curr_vel_iters.to_string())
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_slider_vel_iter, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_vel_iter, &mut ui)
        {
            integration_parameters.max_velocity_iterations = val as usize;
        }

        conrod::widget::Text::new("Pos. Iters.:")
            .down_from(self.ids.slider_vel_iter, VSPACE)
            .set(self.ids.title_slider_pos_iter, &mut ui);

        for val in conrod::widget::Slider::new(curr_pos_iters as f32, 0.0, 200.0)
            .label(&curr_pos_iters.to_string())
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_slider_pos_iter, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_pos_iter, &mut ui)
        {
            integration_parameters.max_position_iterations = val as usize;
        }

        #[cfg(feature = "parallel")]
        {
            conrod::widget::Text::new("Num. Threads.:")
                .down_from(self.ids.slider_pos_iter, VSPACE)
                .set(self.ids.title_slider_num_threads, &mut ui);

            for val in conrod::widget::Slider::new(
                curr_num_threads as f32,
                1.0,
                num_cpus::get_physical() as f32,
            )
            .label(&curr_num_threads.to_string())
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_slider_num_threads, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_num_threads, &mut ui)
            {
                if _run_state.num_threads != val as usize {
                    _run_state.num_threads = val as usize;
                    _run_state.thread_pool = rapier::rayon::ThreadPoolBuilder::new()
                        .num_threads(_run_state.num_threads)
                        .build()
                        .unwrap();
                }
            }
        }

        conrod::widget::Text::new("CCD substeps:")
            .down_from(
                if cfg!(feature = "parallel") {
                    self.ids.slider_num_threads
                } else {
                    self.ids.slider_pos_iter
                },
                VSPACE,
            )
            .set(self.ids.title_slider_ccd_substeps, &mut ui);

        for val in conrod::widget::Slider::new(curr_max_ccd_substeps as f32, 0.0, 10.0)
            .label(&curr_max_ccd_substeps.to_string())
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_slider_ccd_substeps, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_ccd_substeps, &mut ui)
        {
            integration_parameters.max_ccd_substeps = val as usize;
        }

        conrod::widget::Text::new("Min island size:")
            .down_from(self.ids.slider_ccd_substeps, VSPACE)
            .set(self.ids.title_slider_min_island_size, &mut ui);

        for val in conrod::widget::Slider::new(curr_min_island_size as f32, 1.0, 10000.0)
            .label(&curr_min_island_size.to_string())
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_slider_min_island_size, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_min_island_size, &mut ui)
        {
            integration_parameters.min_island_size = val as usize;
        }

        conrod::widget::Text::new("Warm-start coeff.:")
            .down_from(self.ids.slider_min_island_size, VSPACE)
            .set(self.ids.title_warmstart_coeff, &mut ui);

        for val in conrod::widget::Slider::new(curr_warmstart_coeff as f32, 0.0, 1.0)
            .label(&format!("{:.2}", curr_warmstart_coeff))
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_warmstart_coeff, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_warmstart_coeff, &mut ui)
        {
            integration_parameters.warmstart_coeff = val;
        }

        conrod::widget::Text::new("Frequency:")
            .down_from(self.ids.slider_warmstart_coeff, VSPACE)
            .set(self.ids.title_frequency, &mut ui);

        for val in conrod::widget::Slider::new(curr_frequency as f32, 0.0, 240.0)
            .label(&format!("{:.2}Hz", curr_frequency))
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.title_frequency, TITLE_VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.slider_frequency, &mut ui)
        {
            integration_parameters.set_inv_dt(val.round());
        }

        let toggle_list = [
            ("Sleep", self.ids.toggle_sleep, TestbedStateFlags::SLEEP),
            //            ("Warm Starting", self.ids.toggle_warm_starting, TestbedStateFlags::WARM_STARTING),
            (
                "Sub-Stepping",
                self.ids.toggle_sub_stepping,
                TestbedStateFlags::SUB_STEPPING,
            ),
            ("", self.ids.separator2, TestbedStateFlags::NONE),
            //            ("Shapes", self.ids.toggle_shapes, TestbedStateFlags::SHAPES),
            //            ("Joints", self.ids.toggle_joints, TestbedStateFlags::JOINTS),
            ("AABBs", self.ids.toggle_aabbs, TestbedStateFlags::AABBS),
            (
                "Contacts",
                self.ids.toggle_contact_points,
                TestbedStateFlags::CONTACT_POINTS,
            ),
            //            ("ContactManifold Normals", self.ids.toggle_contact_normals, TestbedStateFlags::CONTACT_NORMALS),
            (
                "Wireframe",
                self.ids.toggle_wireframe,
                TestbedStateFlags::WIREFRAME,
            ),
            //            ("Center of Masses", self.ids.toggle_center_of_masses, TestbedStateFlags::CENTER_OF_MASSES),
            //            ("Statistics", self.ids.toggle_statistics, TestbedStateFlags::STATISTICS),
            (
                "Profile",
                self.ids.toggle_profile,
                TestbedStateFlags::PROFILE,
            ),
            (
                "Debug infos",
                self.ids.toggle_debug,
                TestbedStateFlags::DEBUG,
            ),
        ];

        toggles(
            &toggle_list,
            self.ids.canvas,
            self.ids.slider_frequency,
            &mut ui,
            &mut state.flags,
        );

        let label = if state.running == RunMode::Stop {
            "Start (T)"
        } else {
            "Pause (T)"
        };
        for _press in conrod::widget::Button::new()
            .label(label)
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.toggle_debug, VSPACE)
            .w_h(ELEMENT_W, ELEMENT_H)
            .set(self.ids.button_pause, &mut ui)
        {
            if state.running == RunMode::Stop {
                state.running = RunMode::Running
            } else {
                state.running = RunMode::Stop
            }
        }

        for _press in conrod::widget::Button::new()
            .label("Single Step (S)")
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.button_pause, VSPACE)
            .set(self.ids.button_single_step, &mut ui)
        {
            state.running = RunMode::Step
        }

        for _press in conrod::widget::Button::new()
            .label("Take snapshot")
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.button_single_step, VSPACE)
            .set(self.ids.button_take_snapshot, &mut ui)
        {
            state
                .action_flags
                .set(TestbedActionFlags::TAKE_SNAPSHOT, true);
        }

        for _press in conrod::widget::Button::new()
            .label("Restore snapshot")
            .align_middle_x_of(self.ids.canvas)
            .down_from(self.ids.button_take_snapshot, VSPACE)
            .set(self.ids.button_restore_snapshot, &mut ui)
        {
            state
                .action_flags
                .set(TestbedActionFlags::RESTORE_SNAPSHOT, true);
        }

        let before_quit_button_id = if !state.example_names.is_empty() {
            for _press in conrod::widget::Button::new()
                .label("Restart (R)")
                .align_middle_x_of(self.ids.canvas)
                .down_from(self.ids.button_restore_snapshot, VSPACE)
                .set(self.ids.button_restart, &mut ui)
            {
                state.action_flags.set(TestbedActionFlags::RESTART, true);
            }

            self.ids.button_restart
        } else {
            self.ids.button_restore_snapshot
        };

        #[cfg(not(target_arch = "wasm32"))]
        for _press in conrod::widget::Button::new()
            .label("Quit (Esc)")
            .align_middle_x_of(self.ids.canvas)
            .down_from(before_quit_button_id, VSPACE)
            .set(self.ids.button_quit, &mut ui)
        {
            state.running = RunMode::Quit
        }
    }
}

fn toggles(
    toggles: &[(&str, conrod::widget::Id, TestbedStateFlags)],
    canvas: conrod::widget::Id,
    prev: conrod::widget::Id,
    ui: &mut conrod::UiCell,
    flags: &mut TestbedStateFlags,
) {
    toggle(
        toggles[0].0,
        toggles[0].2,
        canvas,
        prev,
        toggles[0].1,
        ui,
        flags,
    );

    for win in toggles.windows(2) {
        toggle(win[1].0, win[1].2, canvas, win[0].1, win[1].1, ui, flags)
    }
}

fn toggle(
    title: &str,
    flag: TestbedStateFlags,
    canvas: conrod::widget::Id,
    prev: conrod::widget::Id,
    curr: conrod::widget::Id,
    ui: &mut conrod::UiCell,
    flags: &mut TestbedStateFlags,
) {
    if title == "" {
        // This is a separator.
        separator(canvas, prev, curr, ui)
    } else {
        for _pressed in conrod::widget::Toggle::new(flags.contains(flag))
            .mid_left_with_margin_on(canvas, LEFT_MARGIN)
            .down_from(prev, VSPACE)
            .w_h(20.0 /*ELEMENT_W*/, ELEMENT_H)
            .label(title)
            .label_color(kiss3d::conrod::color::WHITE)
            .label_x(conrod::position::Relative::Direction(
                conrod::position::Direction::Forwards,
                5.0,
            ))
            .border(2.0)
            //            .border_color(kiss3d::conrod::color::WHITE)
            .set(curr, ui)
        {
            flags.toggle(flag)
        }
    }
}

fn separator(
    canvas: conrod::widget::Id,
    prev: conrod::widget::Id,
    curr: conrod::widget::Id,
    ui: &mut conrod::UiCell,
) {
    conrod::widget::Line::centred([-ELEMENT_W / 2.0, 0.0], [ELEMENT_W / 2.0, 0.0])
        .align_middle_x_of(canvas)
        .down_from(prev, VSPACE)
        .w(ELEMENT_W)
        .set(curr, ui);
}
