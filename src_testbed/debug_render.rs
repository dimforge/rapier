#![allow(clippy::unnecessary_cast)] // Casts are needed for switching between f32/f64.

use crate::harness::Harness;
use bevy::gizmos::gizmos::Gizmos;
use bevy::prelude::*;
use rapier::math::{Point, Real};
use rapier::pipeline::{
    DebugColor, DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline,
};

#[derive(Resource)]
pub struct DebugRenderPipelineResource {
    pub pipeline: DebugRenderPipeline,
    pub enabled: bool,
}

#[derive(Default)]
pub struct RapierDebugRenderPlugin {}

impl Plugin for RapierDebugRenderPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(DebugRenderPipelineResource {
            pipeline: DebugRenderPipeline::new(
                Default::default(),
                !DebugRenderMode::RIGID_BODY_AXES & !DebugRenderMode::COLLIDER_AABBS,
            ),
            enabled: false,
        })
        .add_systems(Update, debug_render_scene);
    }
}

struct BevyLinesRenderBackend<'w, 's> {
    gizmos: Gizmos<'w, 's>,
}

impl<'w, 's> DebugRenderBackend for BevyLinesRenderBackend<'w, 's> {
    #[cfg(feature = "dim2")]
    fn draw_line(
        &mut self,
        _: DebugRenderObject,
        a: Point<Real>,
        b: Point<Real>,
        color: DebugColor,
    ) {
        self.gizmos.line(
            [a.x as f32, a.y as f32, 1.0e-8].into(),
            [b.x as f32, b.y as f32, 1.0e-8].into(),
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }
    #[cfg(feature = "dim3")]
    fn draw_line(
        &mut self,
        _: DebugRenderObject,
        a: Point<Real>,
        b: Point<Real>,
        color: DebugColor,
    ) {
        self.gizmos.line(
            [a.x as f32, a.y as f32, a.z as f32].into(),
            [b.x as f32, b.y as f32, b.z as f32].into(),
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }
}

fn debug_render_scene(
    mut debug_render: ResMut<DebugRenderPipelineResource>,
    harness: NonSend<Harness>,
    gizmos: Gizmos,
) {
    if debug_render.enabled {
        let mut backend = BevyLinesRenderBackend { gizmos };
        debug_render.pipeline.render(
            &mut backend,
            &harness.physics.bodies,
            &harness.physics.colliders,
            &harness.physics.impulse_joints,
            &harness.physics.multibody_joints,
            &harness.physics.narrow_phase,
        );
    }
}
