use crate::harness::Harness;
use crate::lines::DebugLines;
use bevy::prelude::*;
use rapier::math::{Point, Real};
use rapier::pipeline::{
    DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline,
};

#[derive(Resource)]
pub struct DebugRenderPipelineResource(pub DebugRenderPipeline);

pub struct RapierDebugRenderPlugin {
    depth_test: bool,
}

impl Default for RapierDebugRenderPlugin {
    fn default() -> Self {
        Self {
            depth_test: cfg!(feature = "dim3"),
        }
    }
}

impl Plugin for RapierDebugRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(crate::lines::DebugLinesPlugin::with_depth_test(
            self.depth_test,
        ))
        .insert_resource(DebugRenderPipelineResource(DebugRenderPipeline::new(
            Default::default(),
            !DebugRenderMode::RIGID_BODY_AXES & !DebugRenderMode::COLLIDER_AABBS,
        )))
        .add_system_to_stage(CoreStage::Update, debug_render_scene);
    }
}

struct BevyLinesRenderBackend<'a> {
    lines: &'a mut DebugLines,
}

impl<'a> DebugRenderBackend for BevyLinesRenderBackend<'a> {
    #[cfg(feature = "dim2")]
    fn draw_line(&mut self, _: DebugRenderObject, a: Point<Real>, b: Point<Real>, color: [f32; 4]) {
        self.lines.line_colored(
            [a.x as f32, a.y as f32, 1.0e-8 as f32].into(),
            [b.x as f32, b.y as f32, 1.0e-8 as f32].into(),
            0.0,
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }
    #[cfg(feature = "dim3")]
    fn draw_line(&mut self, _: DebugRenderObject, a: Point<Real>, b: Point<Real>, color: [f32; 4]) {
        self.lines.line_colored(
            [a.x as f32, a.y as f32, a.z as f32].into(),
            [b.x as f32, b.y as f32, b.z as f32].into(),
            0.0,
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }
}

fn debug_render_scene(
    mut pipeline: ResMut<DebugRenderPipelineResource>,
    harness: NonSend<Harness>,
    mut lines: ResMut<DebugLines>,
) {
    let mut backend = BevyLinesRenderBackend { lines: &mut *lines };
    pipeline.0.render(
        &mut backend,
        &harness.physics.bodies,
        &harness.physics.colliders,
        &harness.physics.impulse_joints,
        &harness.physics.multibody_joints,
        &harness.physics.narrow_phase,
    );
}
