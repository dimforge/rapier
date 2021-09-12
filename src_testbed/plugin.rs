use crate::harness::Harness;
use crate::physics::PhysicsState;
use crate::GraphicsManager;
use bevy::prelude::*;
use bevy::render::pipeline::PipelineDescriptor;
use bevy_egui::EguiContext;

pub trait TestbedPlugin {
    fn init_plugin(
        &mut self,
        pipelines: &mut Assets<PipelineDescriptor>,
        shaders: &mut Assets<Shader>,
    );
    fn init_graphics(
        &mut self,
        graphics: &mut GraphicsManager,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
        components: &mut Query<(&mut Transform,)>,
        harness: &mut Harness,
    );
    fn clear_graphics(&mut self, graphics: &mut GraphicsManager, commands: &mut Commands);
    fn run_callbacks(&mut self, harness: &mut Harness);
    fn step(&mut self, physics: &mut PhysicsState);
    fn draw(
        &mut self,
        graphics: &mut GraphicsManager,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
        components: &mut Query<(&mut Transform,)>,
        harness: &mut Harness,
    );
    fn update_ui(
        &mut self,
        ui_context: &EguiContext,
        harness: &mut Harness,
        graphics: &mut GraphicsManager,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
        components: &mut Query<(&mut Transform,)>,
    );
    fn profiling_string(&self) -> String;
}
