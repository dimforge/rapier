use crate::GraphicsManager;
use crate::graphics::BevyMaterial;
use crate::harness::Harness;
use crate::physics::PhysicsState;
use bevy::prelude::*;
// use bevy::render::render_resource::RenderPipelineDescriptor;
use bevy_egui::EguiContexts;

pub trait TestbedPlugin {
    fn init_plugin(&mut self);
    fn init_graphics(
        &mut self,
        graphics: &mut GraphicsManager,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<BevyMaterial>,
        components: &mut Query<&mut Transform>,
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
        materials: &mut Assets<BevyMaterial>,
        components: &mut Query<&mut Transform>,
        harness: &mut Harness,
    );
    fn update_ui(
        &mut self,
        ui_context: &EguiContexts,
        harness: &mut Harness,
        graphics: &mut GraphicsManager,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<BevyMaterial>,
        components: &mut Query<&mut Transform>,
    );
    fn profiling_string(&self) -> String;
}
