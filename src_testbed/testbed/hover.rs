#![allow(clippy::useless_conversion)] // Conversions are needed for switching between f32/f64.

use crate::mouse::SceneMouse;
use crate::{GraphicsManager, PhysicsState};
use kiss3d::prelude::*;
use rapier::prelude::QueryFilter;

#[cfg(feature = "dim3")]
use rapier::prelude::{Ray, Real};

#[cfg(feature = "dim2")]
pub fn highlight_hovered_body(
    graphics_manager: &mut GraphicsManager,
    mouse: &SceneMouse,
    physics: &PhysicsState,
) {
    use rapier::math::Vector;

    if let Some(pt) = mouse.point {
        // Convert from kiss3d Vec2 (f32) to rapier Vector (may be f64)
        let pt = Vector::new(pt.x as _, pt.y as _);

        let query_pipeline = physics.broad_phase.as_query_pipeline(
            physics.narrow_phase.query_dispatcher(),
            &physics.bodies,
            &physics.colliders,
            QueryFilter::only_dynamic(),
        );

        for (handle, _) in query_pipeline.intersect_point(pt) {
            let collider = &physics.colliders[handle];
            if let Some(parent_handle) = collider.parent() {
                graphics_manager.set_body_color(parent_handle, RED, true);
            }
        }
    }
}

#[cfg(feature = "dim3")]
pub fn highlight_hovered_body(
    graphics_manager: &mut GraphicsManager,
    mouse: &SceneMouse,
    physics: &PhysicsState,
) {
    if let Some((ray_origin, ray_dir)) = mouse.ray {
        let ray = Ray::new(ray_origin.into(), ray_dir.into());
        let query_pipeline = physics.broad_phase.as_query_pipeline(
            physics.narrow_phase.query_dispatcher(),
            &physics.bodies,
            &physics.colliders,
            QueryFilter::only_dynamic(),
        );

        let hit = query_pipeline.cast_ray(&ray, Real::MAX, true);

        if let Some((handle, _)) = hit {
            let collider = &physics.colliders[handle];

            if let Some(parent_handle) = collider.parent() {
                graphics_manager.set_body_color(parent_handle, RED, true);
            }
        }
    }
}
