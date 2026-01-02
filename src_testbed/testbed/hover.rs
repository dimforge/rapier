use kiss3d::prelude::*;
use rapier::prelude::{QueryFilter, Ray, Real};
use crate::{Camera, GraphicsManager, PhysicsState, TestbedState};
use crate::mouse::SceneMouse;

#[cfg(feature = "dim2")]
pub fn highlight_hovered_body(
    graphics_manager: &mut GraphicsManager,
    testbed_state: &mut TestbedState,
    mouse: &SceneMouse,
    physics: &PhysicsState,
) {
    if let Some(pt) = mouse.point {
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
    testbed_state: &mut TestbedState,
    mouse: &SceneMouse,
    physics: &PhysicsState,
) {
    if let Some((ray_origin, ray_dir)) = mouse.ray {
        let ray = Ray::new(ray_origin, ray_dir);
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
