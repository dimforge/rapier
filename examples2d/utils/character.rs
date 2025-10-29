use rapier_testbed2d::ui::egui::Align2;
use rapier_testbed2d::{
    KeyCode, PhysicsState, TestbedGraphics,
    ui::egui::{ComboBox, Slider, Ui, Window},
};
use rapier2d::{
    control::{CharacterLength, KinematicCharacterController, PidController},
    prelude::*,
};

pub type CharacterSpeed = Real;

#[derive(PartialEq, Clone, Copy, Debug)]
pub enum CharacterControlMode {
    Kinematic(CharacterSpeed),
    Pid(CharacterSpeed),
}

pub fn update_character(
    graphics: &mut TestbedGraphics,
    physics: &mut PhysicsState,
    control_mode: &mut CharacterControlMode,
    controller: &mut KinematicCharacterController,
    pid: &mut PidController,
    character_handle: RigidBodyHandle,
) {
    let prev_control_mode = *control_mode;
    character_control_ui(graphics, controller, pid, control_mode);

    if *control_mode != prev_control_mode {
        match control_mode {
            CharacterControlMode::Kinematic(_) => physics.bodies[character_handle]
                .set_body_type(RigidBodyType::KinematicPositionBased, false),
            CharacterControlMode::Pid(_) => {
                physics.bodies[character_handle].set_body_type(RigidBodyType::Dynamic, true)
            }
        }
    }

    match *control_mode {
        CharacterControlMode::Kinematic(speed) => {
            update_kinematic_controller(graphics, physics, character_handle, controller, speed)
        }
        CharacterControlMode::Pid(speed) => {
            update_pid_controller(graphics, physics, character_handle, pid, speed)
        }
    }
}

fn character_movement_from_inputs(
    gfx: &TestbedGraphics,
    mut speed: Real,
    artificial_gravity: bool,
) -> Vector<Real> {
    let mut desired_movement = Vector::zeros();

    for key in gfx.keys().get_pressed() {
        match *key {
            KeyCode::ArrowRight => {
                desired_movement += Vector::x();
            }
            KeyCode::ArrowLeft => {
                desired_movement -= Vector::x();
            }
            KeyCode::Space => {
                desired_movement += Vector::y() * 2.0;
            }
            KeyCode::ControlRight => {
                desired_movement -= Vector::y();
            }
            KeyCode::ShiftRight => {
                speed /= 10.0;
            }
            _ => {}
        }
    }

    desired_movement *= speed;

    if artificial_gravity {
        desired_movement -= Vector::y() * speed;
    }

    desired_movement
}

fn update_pid_controller(
    gfx: &mut TestbedGraphics,
    phx: &mut PhysicsState,
    character_handle: RigidBodyHandle,
    pid: &mut PidController,
    speed: Real,
) {
    let desired_movement = character_movement_from_inputs(gfx, speed, false);

    let character_body = &mut phx.bodies[character_handle];

    // Adjust the controlled axis depending on the keys pressed by the user.
    // - If the user is jumping, enable control over Y.
    // - If the user isn’t pressing any key, disable all linear controls to let
    //   gravity/collision do their thing freely.
    let mut axes = AxesMask::ANG_Z;

    if desired_movement.norm() != 0.0 {
        axes |= if desired_movement.y == 0.0 {
            AxesMask::LIN_X
        } else {
            AxesMask::LIN_X | AxesMask::LIN_Y
        }
    };

    pid.set_axes(axes);

    let corrective_vel = pid.rigid_body_correction(
        phx.integration_parameters.dt,
        character_body,
        (character_body.translation() + desired_movement).into(),
        RigidBodyVelocity::zero(),
    );
    let new_vel = *character_body.vels() + corrective_vel;

    character_body.set_vels(new_vel, true);
}

fn update_kinematic_controller(
    gfx: &mut TestbedGraphics,
    phx: &mut PhysicsState,
    character_handle: RigidBodyHandle,
    controller: &KinematicCharacterController,
    speed: Real,
) {
    let desired_movement = character_movement_from_inputs(gfx, speed, true);

    let character_body = &phx.bodies[character_handle];
    let character_collider = &phx.colliders[character_body.colliders()[0]];
    let character_collider_pose = *character_collider.position();
    let character_shape = character_collider.shared_shape().clone();
    let character_mass = character_body.mass();

    let mut query_pipeline = phx.broad_phase.as_query_pipeline_mut(
        phx.narrow_phase.query_dispatcher(),
        &mut phx.bodies,
        &mut phx.colliders,
        QueryFilter::new().exclude_rigid_body(character_handle),
    );

    let mut collisions = vec![];
    let mvt = controller.move_shape(
        phx.integration_parameters.dt,
        &query_pipeline.as_ref(),
        &*character_shape,
        &character_collider_pose,
        desired_movement.cast::<Real>(),
        |c| collisions.push(c),
    );

    if mvt.grounded {
        gfx.set_body_color(character_handle, [0.1, 0.8, 0.1]);
    } else {
        gfx.set_body_color(character_handle, [0.8, 0.1, 0.1]);
    }

    controller.solve_character_collision_impulses(
        phx.integration_parameters.dt,
        &mut query_pipeline,
        &*character_shape,
        character_mass,
        &*collisions,
    );

    let character_body = &mut phx.bodies[character_handle];
    let pose = character_body.position();
    character_body.set_next_kinematic_translation(pose.translation.vector + mvt.translation);
}

fn character_control_ui(
    gfx: &mut TestbedGraphics,
    character_controller: &mut KinematicCharacterController,
    pid_controller: &mut PidController,
    control_mode: &mut CharacterControlMode,
) {
    Window::new("Character Control")
        .anchor(Align2::RIGHT_TOP, [-15.0, 15.0])
        .show(gfx.ui_context_mut().ctx_mut(), |ui| {
            ComboBox::from_label("control mode")
                .selected_text(format!("{:?}", *control_mode))
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        control_mode,
                        CharacterControlMode::Kinematic(0.1),
                        "Kinematic",
                    );
                    ui.selectable_value(control_mode, CharacterControlMode::Pid(0.1), "Pid");
                });

            match control_mode {
                CharacterControlMode::Kinematic(speed) => {
                    kinematic_control_ui(ui, character_controller, speed);
                }
                CharacterControlMode::Pid(speed) => {
                    pid_control_ui(ui, pid_controller, speed);
                }
            }
        });
}

fn pid_control_ui(ui: &mut Ui, pid_controller: &mut PidController, speed: &mut Real) {
    let mut lin_kp = pid_controller.pd.lin_kp.x;
    let mut lin_ki = pid_controller.lin_ki.x;
    let mut lin_kd = pid_controller.pd.lin_kd.x;
    let mut ang_kp = pid_controller.pd.ang_kp;
    let mut ang_ki = pid_controller.ang_ki;
    let mut ang_kd = pid_controller.pd.ang_kd;

    ui.add(Slider::new(speed, 0.0..=1.0).text("speed"));
    ui.add(Slider::new(&mut lin_kp, 0.0..=100.0).text("linear Kp"));
    ui.add(Slider::new(&mut lin_ki, 0.0..=10.0).text("linear Ki"));
    ui.add(Slider::new(&mut lin_kd, 0.0..=1.0).text("linear Kd"));
    ui.add(Slider::new(&mut ang_kp, 0.0..=100.0).text("angular Kp"));
    ui.add(Slider::new(&mut ang_ki, 0.0..=10.0).text("angular Ki"));
    ui.add(Slider::new(&mut ang_kd, 0.0..=1.0).text("angular Kd"));

    pid_controller.pd.lin_kp.fill(lin_kp);
    pid_controller.lin_ki.fill(lin_ki);
    pid_controller.pd.lin_kd.fill(lin_kd);
    pid_controller.pd.ang_kp = ang_kp;
    pid_controller.ang_ki = ang_ki;
    pid_controller.pd.ang_kd = ang_kd;
}

fn kinematic_control_ui(
    ui: &mut Ui,
    character_controller: &mut KinematicCharacterController,
    speed: &mut Real,
) {
    ui.add(Slider::new(speed, 0.0..=1.0).text("Speed"))
        .on_hover_text("The speed applied each simulation tick.");
    ui.checkbox(&mut character_controller.slide, "slide")
        .on_hover_text("Should the character try to slide against the floor if it hits it?");
    #[allow(clippy::useless_conversion)]
    {
        ui.add(Slider::new(&mut character_controller.max_slope_climb_angle, 0.0..=std::f32::consts::TAU.into()).text("max_slope_climb_angle"))
            .on_hover_text("The maximum angle (radians) between the floor’s normal and the `up` vector that the character is able to climb.");
        ui.add(Slider::new(&mut character_controller.min_slope_slide_angle, 0.0..=std::f32::consts::FRAC_PI_2.into()).text("min_slope_slide_angle"))
            .on_hover_text("The minimum angle (radians) between the floor’s normal and the `up` vector before the character starts to slide down automatically.");
    }
    let mut is_snapped = character_controller.snap_to_ground.is_some();
    if ui.checkbox(&mut is_snapped, "snap_to_ground").changed {
        match is_snapped {
            true => {
                character_controller.snap_to_ground = Some(CharacterLength::Relative(0.1));
            }
            false => {
                character_controller.snap_to_ground = None;
            }
        }
    }
    if let Some(snapped) = &mut character_controller.snap_to_ground {
        match snapped {
            CharacterLength::Relative(val) => {
                ui.add(Slider::new(val, 0.0..=10.0).text("Snapped Relative Character Length"));
            }
            CharacterLength::Absolute(val) => {
                ui.add(Slider::new(val, 0.0..=10.0).text("Snapped Absolute Character Length"));
            }
        }
    }
}
