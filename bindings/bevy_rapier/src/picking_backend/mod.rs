//! A picking backend for Rapier physics entities.
//!
//! By default, all colliders are pickable. Picking can be disabled for individual entities
//! by adding [`Pickable::IGNORE`](bevy::picking::Pickable::IGNORE).
//!
//! To make rapier picking entirely opt-in, set [`RapierPickingSettings::require_markers`]
//! to `true` and add a [`RapierPickable`] component to the desired camera and target entities.

use bevy::app::prelude::*;
use bevy::camera::visibility::{InheritedVisibility, RenderLayers};
use bevy::ecs::prelude::*;
use bevy::picking::backend::ray::RayMap;
use bevy::picking::backend::HitData;
use bevy::picking::backend::PointerHits;
use bevy::picking::{PickingPlugin, PickingSystems};
use bevy::prelude::*;

/// How a ray cast should handle [`Visibility`].
#[derive(Clone, Copy, Reflect)]
pub enum RapierCastVisibility {
    /// Completely ignore visibility checks. Hidden items can still be ray casted against.
    Any,
    /// Only cast rays against entities that are visible in the hierarchy. See [`Visibility`].
    Visible,
}
/// Runtime settings for the [`RapierPickingPlugin`].
#[derive(Resource, Reflect)]
#[reflect(Resource, Default)]
pub struct RapierPickingSettings {
    /// When set to `true` ray casting will only happen between cameras and entities marked with
    /// [`RapierPickable`]. `false` by default.
    ///
    /// This setting is provided to give you fine-grained control over which cameras and entities
    /// should be used by the rapier picking backend at runtime.
    pub require_markers: bool,

    /// Determines how rapier picking should consider [`Visibility`]. When set to [`RapierCastVisibility::Any`],
    /// ray casts can be performed against both visible and hidden entities.
    ///
    /// Defaults to [`RapierCastVisibility::Visible`], only performing picking against entities with [`InheritedVisibility`] set to `true`.
    pub ray_cast_visibility: RapierCastVisibility,
}

impl Default for RapierPickingSettings {
    fn default() -> Self {
        Self {
            require_markers: false,
            ray_cast_visibility: RapierCastVisibility::Visible,
        }
    }
}

/// An optional component that marks cameras and target entities that should be used in the [`RapierPickingPlugin`].
///
/// Only needed if [`RapierPickingSettings::require_markers`] is set to `true`, and ignored otherwise.
#[derive(Debug, Clone, Default, Component, Reflect)]
#[reflect(Component, Default)]
pub struct RapierPickable;

/// Adds the rapier picking backend to your app.
#[derive(Clone, Default)]
pub struct RapierPickingPlugin;

impl Plugin for RapierPickingPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<(RapierPickable, RapierPickingSettings)>()
            .init_resource::<RapierPickingSettings>()
            .add_systems(PreUpdate, update_hits.in_set(PickingSystems::Backend));
        if !app.is_plugin_added::<PickingPlugin>() {
            app.add_plugins(PickingPlugin);
        }
    }
}

/// System which casts rays into the scene using [`RapierPickingSettings`] and sends [`PointerHits`] events.
#[allow(clippy::too_many_arguments)]
pub fn update_hits(
    backend_settings: Res<RapierPickingSettings>,
    ray_map: Res<RayMap>,
    picking_cameras: Query<(&Camera, Option<&RapierPickable>, Option<&RenderLayers>)>,
    marked_targets: Query<&RapierPickable>,
    culling_query: Query<(Option<&InheritedVisibility>, Option<&ViewVisibility>)>,
    layers: Query<&RenderLayers>,
    rapier_context: Query<(
        &crate::prelude::RapierContextColliders,
        &crate::prelude::RapierRigidBodySet,
        &crate::prelude::RapierContextSimulation,
    )>,
    mut output: MessageWriter<PointerHits>,
) {
    for (&ray_id, &ray) in ray_map.map.iter() {
        let Ok((camera, cam_pickable, cam_layers)) = picking_cameras.get(ray_id.camera) else {
            continue;
        };
        if backend_settings.require_markers && cam_pickable.is_none() {
            continue;
        }
        let order = camera.order as f32;
        for (colliders, bodies, simulation) in rapier_context.iter() {
            let predicate = |entity, _: &rapier::geometry::Collider| {
                let marker_requirement =
                    !backend_settings.require_markers || marked_targets.get(entity).is_ok();
                if !marker_requirement {
                    return false;
                }

                let visibility_requirement = match backend_settings.ray_cast_visibility {
                    RapierCastVisibility::Any => true,
                    RapierCastVisibility::Visible => {
                        let is_visible = culling_query
                            .get(entity)
                            .map(|(inherited_visibility, _)| {
                                inherited_visibility.map(|v| v.get()).unwrap_or(false)
                            })
                            .unwrap_or(false);
                        is_visible
                    }
                };
                if !visibility_requirement {
                    return false;
                }
                // Other entities missing render layers are on the default layer 0
                let entity_layers = layers.get(entity).to_owned().unwrap_or_default();
                let cam_layers = cam_layers.to_owned().unwrap_or_default();
                let render_layers_match = cam_layers.intersects(entity_layers);
                if !render_layers_match {
                    return false;
                }

                true
            };
            let filter = crate::prelude::QueryFilter::default().predicate(&predicate);
            let mut picks = Vec::new();
            crate::prelude::RapierQueryPipeline::new_scoped(
                &simulation.broad_phase,
                colliders,
                bodies,
                &filter,
                simulation.query_dispatcher(),
                |query_pipeline| {
                    #[cfg(feature = "dim2")]
                    for (entity, _) in query_pipeline
                        .intersect_point(bevy::math::Vec2::new(ray.origin.x, ray.origin.y))
                    {
                        let hit_data = HitData {
                            camera: ray_id.camera,
                            position: Some(bevy::math::Vec3::new(ray.origin.x, ray.origin.y, 0.0)),
                            normal: None,
                            depth: 0.0,
                            extra: None,
                        };
                        picks.push((entity, hit_data));
                    }
                    #[cfg(feature = "dim3")]
                    for (entity, _, intersection) in query_pipeline.intersect_ray(
                        ray.origin,
                        ray.direction.into(),
                        f32::MAX,
                        true,
                    ) {
                        let hit_data = HitData {
                            camera: ray_id.camera,
                            position: Some(intersection.point),
                            normal: Some(intersection.normal),
                            depth: intersection.time_of_impact,
                            extra: None,
                        };
                        picks.push((entity, hit_data));
                    }
                },
            );

            if !picks.is_empty() {
                output.write(PointerHits::new(ray_id.pointer, picks, order));
            }
        }
    }
}
