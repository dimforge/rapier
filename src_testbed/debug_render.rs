#![allow(clippy::unnecessary_cast)] // Casts are needed for switching between f32/f64.

use crate::harness::Harness;
use kiss3d::window::Window;
use rapier::math::Vector;
use rapier::pipeline::{
    DebugColor, DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline,
};

pub struct DebugRenderPipelineResource {
    pub pipeline: DebugRenderPipeline,
    pub enabled: bool,
}

impl Default for DebugRenderPipelineResource {
    fn default() -> Self {
        Self {
            pipeline: DebugRenderPipeline::new(
                Default::default(),
                !DebugRenderMode::RIGID_BODY_AXES & !DebugRenderMode::COLLIDER_AABBS,
            ),
            enabled: false,
        }
    }
}

/// Kiss3d-based debug render backend
pub struct Kiss3dLinesRenderBackend<'a> {
    pub window: &'a mut Window,
}

impl<'a> DebugRenderBackend for Kiss3dLinesRenderBackend<'a> {
    #[cfg(feature = "dim2")]
    fn draw_line(&mut self, _: DebugRenderObject, a: Vector, b: Vector, color: DebugColor) {
        // Convert HSLA to RGB
        let rgb = hsla_to_rgb(color[0], color[1], color[2], color[3]);
        self.window.draw_line_2d(
            glamx::Vec2::new(a.x as f32, a.y as f32),
            glamx::Vec2::new(b.x as f32, b.y as f32),
            rgb.into(),
            4.0,
        );
    }

    #[cfg(feature = "dim3")]
    fn draw_line(&mut self, _: DebugRenderObject, a: Vector, b: Vector, color: DebugColor) {
        // Convert HSLA to RGB
        let rgb = hsla_to_rgb(color[0], color[1], color[2], color[3]);
        self.window.draw_line(
            glamx::Vec3::new(a.x as f32, a.y as f32, a.z as f32),
            glamx::Vec3::new(b.x as f32, b.y as f32, b.z as f32),
            rgb.into(),
            4.0,
            false,
        );
    }
}

/// Render debug visualization using kiss3d
pub fn debug_render_scene(
    window: &mut Window,
    debug_render: &mut DebugRenderPipelineResource,
    harness: &Harness,
) {
    if debug_render.enabled {
        let mut backend = Kiss3dLinesRenderBackend { window };
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

/// Convert HSLA color to RGB
fn hsla_to_rgb(h: f32, s: f32, l: f32, a: f32) -> [f32; 4] {
    if s == 0.0 {
        return [l, l, l, a];
    }

    let q = if l < 0.5 {
        l * (1.0 + s)
    } else {
        l + s - l * s
    };
    let p = 2.0 * l - q;

    let r = hue_to_rgb(p, q, h / 360.0 + 1.0 / 3.0);
    let g = hue_to_rgb(p, q, h / 360.0);
    let b = hue_to_rgb(p, q, h / 360.0 - 1.0 / 3.0);

    [r, g, b, a]
}

fn hue_to_rgb(p: f32, q: f32, t: f32) -> f32 {
    let t = if t < 0.0 {
        t + 1.0
    } else if t > 1.0 {
        t - 1.0
    } else {
        t
    };

    if t < 1.0 / 6.0 {
        p + (q - p) * 6.0 * t
    } else if t < 1.0 / 2.0 {
        q
    } else if t < 2.0 / 3.0 {
        p + (q - p) * (2.0 / 3.0 - t) * 6.0
    } else {
        p
    }
}
