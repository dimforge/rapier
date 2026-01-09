//! Testbed struct for building examples.

use kiss3d::color::Color;
use rapier::dynamics::{
    ImpulseJointSet, IntegrationParameters, MultibodyJointSet, RigidBodyHandle, RigidBodySet,
};
use rapier::geometry::{ColliderHandle, ColliderSet};
use rapier::pipeline::PhysicsHooks;

#[cfg(feature = "dim3")]
use {glamx::Vec3, rapier::control::DynamicRayCastVehicleController};

use crate::harness::Harness;
use crate::physics::PhysicsState;
use crate::settings::ExampleSettings;

use super::graphics_context::TestbedGraphics;
use super::state::{TestbedActionFlags, TestbedState};

#[cfg(all(feature = "dim3", feature = "other-backends"))]
use super::OtherBackends;

#[cfg(all(feature = "dim3", feature = "other-backends"))]
use super::state::{PHYSX_BACKEND_PATCH_FRICTION, PHYSX_BACKEND_TWO_FRICTION_DIR};

#[cfg(all(feature = "dim3", feature = "other-backends"))]
use crate::physx_backend::PhysxWorld;

use super::Plugins;

/// An example/demo that can be run in the testbed
#[derive(Clone)]
pub struct Example {
    /// Display name of the example
    pub name: &'static str,
    /// Group/category for organizing in the UI (e.g., "Demos", "Joints", "Debug")
    pub group: &'static str,
    /// The builder function that initializes the example
    pub builder: fn(&mut Testbed),
}

impl Example {
    /// Create a new example with a group
    pub fn new(group: &'static str, name: &'static str, builder: fn(&mut Testbed)) -> Self {
        Self {
            name,
            group,
            builder,
        }
    }

    /// Create a new example in the default "Demos" group
    pub fn demo(name: &'static str, builder: fn(&mut Testbed)) -> Self {
        Self::new("Demos", name, builder)
    }
}

/// Allow constructing Example from a tuple (group, name, builder) for convenience
impl From<(&'static str, &'static str, fn(&mut Testbed))> for Example {
    fn from((group, name, builder): (&'static str, &'static str, fn(&mut Testbed))) -> Self {
        Self::new(group, name, builder)
    }
}

/// Type alias for simulation builder functions
pub type SimulationBuilders = Vec<Example>;

/// The main testbed struct passed to example builders
pub struct Testbed<'a> {
    pub graphics: Option<TestbedGraphics<'a>>,
    pub harness: &'a mut Harness,
    pub state: &'a mut TestbedState,
    #[cfg(all(feature = "dim3", feature = "other-backends"))]
    pub other_backends: &'a mut OtherBackends,
    pub plugins: &'a mut Plugins,
}

impl Testbed<'_> {
    pub fn set_number_of_steps_per_frame(&mut self, nsteps: usize) {
        self.state.nsteps = nsteps;
    }

    #[cfg(feature = "dim3")]
    pub fn set_vehicle_controller(&mut self, controller: DynamicRayCastVehicleController) {
        self.state.vehicle_controller = Some(controller);
    }

    pub fn allow_grabbing_behind_ground(&mut self, allow: bool) {
        self.state.can_grab_behind_ground = allow;
    }

    pub fn integration_parameters_mut(&mut self) -> &mut IntegrationParameters {
        &mut self.harness.physics.integration_parameters
    }

    pub fn physics_state_mut(&mut self) -> &mut PhysicsState {
        &mut self.harness.physics
    }

    pub fn harness(&self) -> &Harness {
        self.harness
    }

    pub fn harness_mut(&mut self) -> &mut Harness {
        self.harness
    }

    pub fn example_settings_mut(&mut self) -> &mut ExampleSettings {
        &mut self.state.example_settings
    }

    pub fn set_world(
        &mut self,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulse_joints: ImpulseJointSet,
        multibody_joints: MultibodyJointSet,
    ) {
        self.set_world_with_params(
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            rapier::math::Vector::Y * -9.81,
            (),
        )
    }

    pub fn set_world_with_params(
        &mut self,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulse_joints: ImpulseJointSet,
        multibody_joints: MultibodyJointSet,
        gravity: rapier::math::Vector,
        hooks: impl PhysicsHooks + 'static,
    ) {
        self.harness.set_world_with_params(
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            self.state.broad_phase_type,
            gravity,
            hooks,
        );

        self.state
            .action_flags
            .set(TestbedActionFlags::RESET_WORLD_GRAPHICS, true);

        #[cfg(feature = "dim3")]
        {
            self.state.vehicle_controller = None;
        }

        #[cfg(all(feature = "dim3", feature = "other-backends"))]
        {
            if self.state.selected_backend == PHYSX_BACKEND_PATCH_FRICTION
                || self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR
            {
                self.other_backends.physx = Some(PhysxWorld::from_rapier(
                    self.harness.physics.gravity,
                    &self.harness.physics.integration_parameters,
                    &self.harness.physics.bodies,
                    &self.harness.physics.colliders,
                    &self.harness.physics.impulse_joints,
                    &self.harness.physics.multibody_joints,
                    self.state.selected_backend == PHYSX_BACKEND_TWO_FRICTION_DIR,
                    self.harness.state.num_threads(),
                ));
            }
        }
    }

    pub fn set_graphics_shift(&mut self, shift: rapier::math::Vector) {
        if !self.state.camera_locked
            && let Some(graphics) = &mut self.graphics
        {
            graphics.graphics.gfx_shift = shift;
        }
    }

    #[cfg(feature = "dim2")]
    pub fn look_at(&mut self, at: glamx::Vec2, zoom: f32) {
        if !self.state.camera_locked
            && let Some(graphics) = &mut self.graphics
        {
            graphics.camera.set_at(at);
            graphics.camera.set_zoom(zoom);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn look_at(&mut self, eye: Vec3, at: Vec3) {
        if !self.state.camera_locked
            && let Some(graphics) = &mut self.graphics
        {
            graphics.camera.look_at(eye, at);
        }
    }

    pub fn set_initial_body_color(&mut self, body: RigidBodyHandle, color: Color) {
        if let Some(graphics) = &mut self.graphics {
            graphics.graphics.set_initial_body_color(body, color);
        }
    }

    pub fn set_initial_collider_color(&mut self, collider: ColliderHandle, color: Color) {
        if let Some(graphics) = &mut self.graphics {
            graphics
                .graphics
                .set_initial_collider_color(collider, color);
        }
    }

    pub fn set_body_wireframe(&mut self, body: RigidBodyHandle, wireframe_enabled: bool) {
        if let Some(graphics) = &mut self.graphics {
            graphics
                .graphics
                .set_body_wireframe(body, wireframe_enabled);
        }
    }

    pub fn add_callback<
        F: FnMut(
                Option<&mut TestbedGraphics>,
                &mut PhysicsState,
                &crate::physics::PhysicsEvents,
                &crate::harness::RunState,
            ) + 'static,
    >(
        &mut self,
        callback: F,
    ) {
        self.harness.add_callback(callback);
    }

    pub fn add_plugin(&mut self, mut plugin: impl crate::plugin::TestbedPlugin + 'static) {
        plugin.init_plugin();
        self.plugins.0.push(Box::new(plugin));
    }
}
