use crate::GraphicsManager;
use crate::harness::Harness;
use crate::physics::PhysicsState;
use kiss3d::window::Window;

pub trait TestbedPlugin {
    fn init_plugin(&mut self);
    fn init_graphics(
        &mut self,
        graphics: &mut GraphicsManager,
        window: &mut Window,
        harness: &mut Harness,
    );
    fn clear_graphics(&mut self, graphics: &mut GraphicsManager, window: &mut Window);
    fn run_callbacks(&mut self, harness: &mut Harness);
    fn step(&mut self, physics: &mut PhysicsState);
    fn draw(&mut self, graphics: &mut GraphicsManager, window: &mut Window, harness: &mut Harness);
    fn update_ui(
        &mut self,
        ui_context: &egui::Context,
        harness: &mut Harness,
        graphics: &mut GraphicsManager,
        window: &mut Window,
    );
    fn profiling_string(&self) -> String;
}
