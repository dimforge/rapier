#![allow(clippy::unnecessary_cast)] // Casts are needed for switching between f32/f64.

#[cfg(feature = "dim3")]
use kiss3d::renderer::Polyline3d;
use kiss3d::window::Window;
use rapier::math::Vector;
use rapier::pipeline::{
    DebugColor, DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline,
    PhysicsWorld,
};

pub struct DebugRenderPipelineResource {
    pub pipeline: DebugRenderPipeline,
    pub enabled: bool,
}

impl Default for DebugRenderPipelineResource {
    fn default() -> Self {
        Self {
            // AABBs and pseudo-normals are opt-in (both bury the scene under lines), and so is
            // the soft-body stress coloring (it replaces the elements' color).
            pipeline: DebugRenderPipeline::new(
                Default::default(),
                !(DebugRenderMode::COLLIDER_AABBS
                    | DebugRenderMode::PSEUDO_NORMALS
                    | DebugRenderMode::SOFT_BODY_STRESS),
            ),
            enabled: false,
        }
    }
}

/// Slight depth bias to avoid z-fighting with other lines (like mesh geometry).
#[cfg(feature = "dim3")]
const SOFT_BODY_DEPTH_BIAS: f32 = 1.0e-6;

/// Kiss3d-based debug render backend
pub struct Kiss3dLinesRenderBackend<'a> {
    pub window: &'a mut Window,
    /// Reused from line to line: a depth-biased line goes through a polyline, and rebuilding one
    /// per line would allocate thousands of times per frame.
    #[cfg(feature = "dim3")]
    biased: Polyline3d,
}

impl<'a> Kiss3dLinesRenderBackend<'a> {
    pub fn new(window: &'a mut Window) -> Self {
        Self {
            window,
            #[cfg(feature = "dim3")]
            biased: Polyline3d::new(vec![glamx::Vec3::ZERO; 2])
                .with_width(4.0)
                .with_depth_bias(SOFT_BODY_DEPTH_BIAS),
        }
    }
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
    fn draw_line(&mut self, object: DebugRenderObject, a: Vector, b: Vector, color: DebugColor) {
        // Convert HSLA to RGB
        let rgb = hsla_to_rgb(color[0], color[1], color[2], color[3]);
        let a = glamx::Vec3::new(a.x as f32, a.y as f32, a.z as f32);
        let b = glamx::Vec3::new(b.x as f32, b.y as f32, b.z as f32);

        // A soft body's cage is inside the surface the body is drawn as, so it is drawn in front
        // of it instead of behind. Nothing else is biased: it would only hide what it overlaps.
        if matches!(object, DebugRenderObject::SoftBody(..)) {
            let Self { window, biased } = self;
            biased.vertices[0] = a;
            biased.vertices[1] = b;
            biased.color = rgb.into();
            window.draw_polyline(biased);
        } else {
            self.window.draw_line(a, b, rgb.into(), 4.0, false);
        }
    }
}

/// Render debug visualization using kiss3d
pub fn debug_render_scene(
    window: &mut Window,
    debug_render: &mut DebugRenderPipelineResource,
    world: &PhysicsWorld,
) {
    if debug_render.enabled {
        let mut backend = Kiss3dLinesRenderBackend::new(window);
        debug_render.pipeline.render(
            &mut backend,
            &world.bodies,
            &world.colliders,
            &world.impulse_joints,
            &world.multibody_joints,
            &world.narrow_phase,
            &world.soft_bodies,
        );
    }
}

/// Convert HSLA color to RGB
pub(crate) fn hsla_to_rgb(h: f32, s: f32, l: f32, a: f32) -> [f32; 4] {
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

/// The inverse of [`hsla_to_rgb`]: an RGBA color as the HSLA the debug-render style stores (hue
/// in degrees, the rest in `0..=1`).
pub(crate) fn rgb_to_hsla(rgba: [f32; 4]) -> DebugColor {
    let [r, g, b, a] = rgba;
    let max = r.max(g).max(b);
    let min = r.min(g).min(b);
    let l = (max + min) / 2.0;
    let delta = max - min;

    if delta <= f32::EPSILON {
        return [0.0, 0.0, l, a];
    }

    let s = if l < 0.5 {
        delta / (max + min)
    } else {
        delta / (2.0 - max - min)
    };
    let h = if max == r {
        ((g - b) / delta).rem_euclid(6.0)
    } else if max == g {
        (b - r) / delta + 2.0
    } else {
        (r - g) / delta + 4.0
    };

    // Hue is an angle: a value a rounding error short of a full turn is zero, not 359.999.
    let h = (h * 60.0).rem_euclid(360.0);
    let h = if h > 360.0 - 1.0e-3 { 0.0 } else { h };
    [h, s, l, a]
}

#[cfg(test)]
mod tests {
    use super::{hsla_to_rgb, rgb_to_hsla};

    /// The color picker shows a style color as RGB and writes it back as HSLA: the pair must
    /// round-trip, or editing one color would drift every time the tab is opened.
    #[test]
    fn hsla_round_trips_through_rgb() {
        for hsla in [
            [340.0, 1.0, 0.3, 1.0],
            [20.0, 1.0, 0.3, 1.0],
            [30.0, 1.0, 0.4, 0.5],
            [0.0, 1.0, 0.5, 1.0],
            [120.0, 0.5, 0.75, 1.0],
            [240.0, 0.25, 0.5, 0.25],
            [359.0, 0.9, 0.1, 1.0],
        ] {
            let rgba = hsla_to_rgb(hsla[0], hsla[1], hsla[2], hsla[3]);
            assert!(
                rgba.iter().all(|c| (0.0..=1.0).contains(c)),
                "{hsla:?} left the unit cube: {rgba:?}"
            );
            let back = rgb_to_hsla(rgba);
            assert!(
                (back[0] - hsla[0]).abs() < 0.1
                    && (back[1] - hsla[1]).abs() < 1.0e-3
                    && (back[2] - hsla[2]).abs() < 1.0e-3
                    && (back[3] - hsla[3]).abs() < 1.0e-6,
                "{hsla:?} came back as {back:?}"
            );
        }
    }
}
