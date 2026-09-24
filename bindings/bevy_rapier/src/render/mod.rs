use crate::plugin::context::{
    RapierContextColliders, RapierContextJoints, RapierContextSimulation, RapierRigidBodySet,
};
use bevy::prelude::*;
use bevy::transform::TransformSystems;
use rapier::math::Vector;
use rapier::pipeline::{DebugRenderBackend, DebugRenderObject, DebugRenderPipeline};
pub use rapier::pipeline::{DebugRenderMode, DebugRenderStyle};
use std::fmt::Debug;

#[cfg(doc)]
use crate::prelude::{Collider, ImpulseJoint, MultibodyJoint, SoftBody};

/// The color of a collider when using the debug-renderer.
///
/// Insert this component alongside the [`Collider`] component to
/// force to a specific value the color used to render the
/// collider and its AABB. It takes precedence over [`DebugRenderColor`] for these objects.
#[derive(Copy, Clone, Component, PartialEq, Debug, Reflect)]
pub struct ColliderDebugColor(pub Hsla);

/// Overrides the global [`DebugRenderContext`] for a single collider.
///
/// This applies to the collider’s shape, its AABB, and the contact pairs it is involved in
/// (which are hidden if either collider is [`ColliderDebug::NeverRender`]). It takes precedence
/// over [`DebugRenderVisibility`] for these objects.
#[derive(Copy, Clone, Reflect, Component, Eq, PartialEq, Default, Debug)]
pub enum ColliderDebug {
    /// Always render the debug gizmos for this collider, regardless of global config.
    #[default]
    AlwaysRender,
    /// Never render the debug gizmos for this collider, regardless of global config.
    NeverRender,
}

/// The color used by the debug-renderer for every object attached to this entity.
///
/// This applies to the entity’s collider and collider AABB (unless it has a
/// [`ColliderDebugColor`]), to its rigid-body’s local axes, to its [`ImpulseJoint`] or
/// [`MultibodyJoint`], and to the elements of its [`SoftBody`] (overriding the stress coloring of
/// [`DebugRenderMode::SOFT_BODY_STRESS`]).
#[derive(Copy, Clone, Component, PartialEq, Debug, Reflect)]
pub struct DebugRenderColor(pub Hsla);

/// Controls whether the debug-renderer draws the objects attached to this entity.
///
/// This applies to the entity’s collider shape (unless it has a [`ColliderDebug`], and
/// regardless of [`DebugRenderContext::default_collider_debug`]), collider AABB, the contact
/// pairs involving its collider, its rigid-body’s local axes, its [`ImpulseJoint`] or
/// [`MultibodyJoint`], and the elements of its [`SoftBody`].
#[derive(Copy, Clone, Reflect, Component, Eq, PartialEq, Default, Debug)]
pub enum DebugRenderVisibility {
    /// Render the objects attached to this entity (if enabled by the [`DebugRenderMode`]).
    #[default]
    Visible,
    /// Never render the objects attached to this entity.
    Hidden,
}

/// Reflectable mirror of [`DebugRenderMode`], with one boolean per flag.
///
/// Convert from and to [`DebugRenderMode`] with [`From`]/[`Into`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, Reflect)]
pub struct DebugRenderModeFlags {
    /// See [`DebugRenderMode::COLLIDER_SHAPES`].
    pub collider_shapes: bool,
    /// See [`DebugRenderMode::RIGID_BODY_AXES`].
    pub rigid_body_axes: bool,
    /// See [`DebugRenderMode::MULTIBODY_JOINTS`].
    pub multibody_joints: bool,
    /// See [`DebugRenderMode::IMPULSE_JOINTS`].
    pub impulse_joints: bool,
    /// See [`DebugRenderMode::SOLVER_CONTACTS`].
    pub solver_contacts: bool,
    /// See [`DebugRenderMode::CONTACTS`].
    pub contacts: bool,
    /// See [`DebugRenderMode::COLLIDER_AABBS`].
    pub collider_aabbs: bool,
    /// See [`DebugRenderMode::SOFT_BODIES`].
    pub soft_bodies: bool,
    /// See [`DebugRenderMode::PSEUDO_NORMALS`].
    pub pseudo_normals: bool,
    /// See [`DebugRenderMode::SOFT_VOLUME_CONTACTS`].
    pub soft_volume_contacts: bool,
    /// See [`DebugRenderMode::SOFT_BODY_STRESS`].
    pub soft_body_stress: bool,
}

impl Default for DebugRenderModeFlags {
    fn default() -> Self {
        DebugRenderMode::default().into()
    }
}

impl From<DebugRenderMode> for DebugRenderModeFlags {
    fn from(mode: DebugRenderMode) -> Self {
        Self {
            collider_shapes: mode.contains(DebugRenderMode::COLLIDER_SHAPES),
            rigid_body_axes: mode.contains(DebugRenderMode::RIGID_BODY_AXES),
            multibody_joints: mode.contains(DebugRenderMode::MULTIBODY_JOINTS),
            impulse_joints: mode.contains(DebugRenderMode::IMPULSE_JOINTS),
            solver_contacts: mode.contains(DebugRenderMode::SOLVER_CONTACTS),
            contacts: mode.contains(DebugRenderMode::CONTACTS),
            collider_aabbs: mode.contains(DebugRenderMode::COLLIDER_AABBS),
            soft_bodies: mode.contains(DebugRenderMode::SOFT_BODIES),
            pseudo_normals: mode.contains(DebugRenderMode::PSEUDO_NORMALS),
            soft_volume_contacts: mode.contains(DebugRenderMode::SOFT_VOLUME_CONTACTS),
            soft_body_stress: mode.contains(DebugRenderMode::SOFT_BODY_STRESS),
        }
    }
}

impl From<DebugRenderModeFlags> for DebugRenderMode {
    fn from(flags: DebugRenderModeFlags) -> Self {
        let mut mode = DebugRenderMode::empty();
        mode.set(DebugRenderMode::COLLIDER_SHAPES, flags.collider_shapes);
        mode.set(DebugRenderMode::RIGID_BODY_AXES, flags.rigid_body_axes);
        mode.set(DebugRenderMode::MULTIBODY_JOINTS, flags.multibody_joints);
        mode.set(DebugRenderMode::IMPULSE_JOINTS, flags.impulse_joints);
        mode.set(DebugRenderMode::SOLVER_CONTACTS, flags.solver_contacts);
        mode.set(DebugRenderMode::CONTACTS, flags.contacts);
        mode.set(DebugRenderMode::COLLIDER_AABBS, flags.collider_aabbs);
        mode.set(DebugRenderMode::SOFT_BODIES, flags.soft_bodies);
        mode.set(DebugRenderMode::PSEUDO_NORMALS, flags.pseudo_normals);
        mode.set(
            DebugRenderMode::SOFT_VOLUME_CONTACTS,
            flags.soft_volume_contacts,
        );
        mode.set(DebugRenderMode::SOFT_BODY_STRESS, flags.soft_body_stress);
        mode
    }
}

/// Plugin rensponsible for rendering (using lines) what Rapier "sees" when performing
/// its physics simulation. This is typically useful to check proper
/// alignment between colliders and your own visual assets.
pub struct RapierDebugRenderPlugin {
    /// Whether to show debug gizmos for all colliders.
    ///
    /// Can be overridden for individual colliders by adding a [`ColliderDebug`] component.
    pub default_collider_debug: ColliderDebug,
    /// Is the debug-rendering enabled?
    pub enabled: bool,
    /// Control some aspects of the render coloring.
    ///
    /// Its lengths are expressed in meters: they are multiplied by the length unit of each
    /// context (see [`DebugRenderContext::scale_lengths_by_length_unit`]).
    pub style: DebugRenderStyle,
    /// Flags to select what part of physics scene is rendered (by default
    /// everything is rendered).
    pub mode: DebugRenderMode,
}

impl Default for RapierDebugRenderPlugin {
    fn default() -> Self {
        Self {
            enabled: true,
            default_collider_debug: ColliderDebug::AlwaysRender,
            style: DebugRenderStyle::default(),
            mode: DebugRenderMode::default(),
        }
    }
}

impl RapierDebugRenderPlugin {
    /// Initialize the render plugin such that it is initially disabled.
    pub fn disabled(mut self) -> Self {
        self.enabled = false;
        self
    }
}

/// Context to control some aspect of the debug-renderer after initialization.
#[derive(Resource, Reflect)]
#[reflect(Resource)]
pub struct DebugRenderContext {
    /// Is the debug-rendering currently enabled?
    pub enabled: bool,
    /// Whether to show debug gizmos for all colliders.
    ///
    /// Can be overridden for individual colliders by adding a [`ColliderDebug`] or
    /// [`DebugRenderVisibility`] component.
    pub default_collider_debug: ColliderDebug,
    /// The style used to compute the colors and lengths of the rendered lines.
    ///
    /// Its lengths ([`DebugRenderStyle::rigid_body_axes_length`],
    /// [`DebugRenderStyle::contact_normal_length`] and [`DebugRenderStyle::pseudo_normal_length`])
    /// are multiplied by the length unit of each context if
    /// [`Self::scale_lengths_by_length_unit`] is set.
    #[reflect(remote = crate::reflect::DebugRenderStyleWrapper)]
    pub style: DebugRenderStyle,
    /// Flags selecting the parts of the physics scene that are rendered.
    pub mode: DebugRenderModeFlags,
    /// If `true` (the default), the lengths of [`Self::style`] are expressed in meters and
    /// multiplied by the [`IntegrationParameters::length_unit`] (e.g. the pixels-per-meter in
    /// 2D) of the rendered context.
    ///
    /// [`IntegrationParameters::length_unit`]: rapier::dynamics::IntegrationParameters::length_unit
    pub scale_lengths_by_length_unit: bool,
    /// Pipeline responsible for rendering.
    ///
    /// Its `style` and `mode` are synchronized with [`Self::style`] and [`Self::mode`] before
    /// each render. Modifying `pipeline.style` or `pipeline.mode` directly is still supported:
    /// such changes are copied back into [`Self::style`] and [`Self::mode`].
    #[reflect(ignore)]
    pub pipeline: DebugRenderPipeline,
}

impl Default for DebugRenderContext {
    fn default() -> Self {
        Self {
            enabled: true,
            default_collider_debug: ColliderDebug::AlwaysRender,
            style: DebugRenderStyle::default(),
            mode: DebugRenderModeFlags::default(),
            scale_lengths_by_length_unit: true,
            pipeline: DebugRenderPipeline::default(),
        }
    }
}

impl DebugRenderContext {
    /// Copies the style and mode into the pipeline (or the other way round if the pipeline was
    /// modified directly since the last synchronization).
    fn sync_pipeline(&mut self, state: &mut DebugRenderSyncState) {
        let mut mode = DebugRenderMode::from(self.mode);
        match state.synced {
            Some((style, prev_mode)) => {
                if self.pipeline.style != style {
                    self.style = self.pipeline.style;
                }
                if self.pipeline.mode != prev_mode {
                    mode = self.pipeline.mode;
                    self.mode = mode.into();
                }
            }
            None => state.built_subdivisions = self.pipeline.style.subdivisions,
        }

        if self.style.subdivisions != state.built_subdivisions {
            // The shape outlines depend on the subdivisions, so the pipeline must be rebuilt.
            self.pipeline = DebugRenderPipeline::new(self.style, mode);
            state.built_subdivisions = self.style.subdivisions;
        } else {
            self.pipeline.style = self.style;
            self.pipeline.mode = mode;
        }
        state.synced = Some((self.style, mode));
    }
}

/// The style and mode last copied into [`DebugRenderContext::pipeline`].
#[derive(Resource, Default)]
struct DebugRenderSyncState {
    synced: Option<(DebugRenderStyle, DebugRenderMode)>,
    built_subdivisions: u32,
}

/// Multiplies the length-like fields of `style` by `length_unit`.
fn scale_style(mut style: DebugRenderStyle, length_unit: f32) -> DebugRenderStyle {
    style.rigid_body_axes_length *= length_unit;
    style.contact_normal_length *= length_unit;
    style.pseudo_normal_length *= length_unit;
    style
}

impl Plugin for RapierDebugRenderPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<DebugRenderContext>();
        app.register_type::<ColliderDebug>();
        app.register_type::<ColliderDebugColor>();
        app.register_type::<DebugRenderColor>();
        app.register_type::<DebugRenderVisibility>();
        app.register_type::<DebugRenderModeFlags>();
        app.register_type::<crate::reflect::DebugRenderStyleWrapper>();

        app.init_resource::<DebugRenderSyncState>();
        app.insert_resource(DebugRenderContext {
            enabled: self.enabled,
            default_collider_debug: self.default_collider_debug,
            style: self.style,
            mode: self.mode.into(),
            scale_lengths_by_length_unit: true,
            pipeline: DebugRenderPipeline::new(self.style, self.mode),
        })
        .add_systems(
            PostUpdate,
            debug_render_scene.after(TransformSystems::Propagate),
        );
    }
}

/// The per-entity debug-render overrides.
type DebugOverrides = (
    Option<&'static ColliderDebugColor>,
    Option<&'static ColliderDebug>,
    Option<&'static DebugRenderColor>,
    Option<&'static DebugRenderVisibility>,
);

struct BevyLinesRenderBackend<'world, 'state, 'world2, 'state2, 'q, 'p> {
    overrides: &'q Query<'world, 'state, DebugOverrides>,
    default_collider_debug: ColliderDebug,
    gizmos: &'p mut Gizmos<'world2, 'state2>,
}

impl<'world, 'state, 'world2, 'state2, 'q, 'p>
    BevyLinesRenderBackend<'world, 'state, 'world2, 'state2, 'q, 'p>
{
    /// The entity whose components can override the rendering of `object`.
    fn object_entity(object: &DebugRenderObject) -> Option<Entity> {
        let user_data = match object {
            DebugRenderObject::Collider(_, co) | DebugRenderObject::ColliderAabb(_, co, _) => {
                co.user_data
            }
            DebugRenderObject::RigidBody(_, rb) => rb.user_data,
            DebugRenderObject::ImpulseJoint(_, joint) => joint.data.user_data,
            DebugRenderObject::MultibodyJoint(_, _, link) => link.joint().data.user_data,
            DebugRenderObject::SoftBody(_, sb) => sb.user_data,
            DebugRenderObject::ContactPair(..) => return None,
        };
        Entity::try_from_bits(user_data as u64)
    }

    /// The explicit visibility of the collider objects of `collider`, if any.
    fn collider_visibility(&self, collider: &rapier::geometry::Collider) -> Option<bool> {
        let entity = Entity::try_from_bits(collider.user_data as u64)?;
        let (_, collider_debug, _, visibility) = self.overrides.get(entity).ok()?;
        collider_debug
            .map(|d| *d == ColliderDebug::AlwaysRender)
            .or(visibility.map(|v| *v == DebugRenderVisibility::Visible))
    }

    fn object_color(&self, object: DebugRenderObject, default: [f32; 4]) -> [f32; 4] {
        let Some(entity) = Self::object_entity(&object) else {
            return default;
        };
        let Ok((collider_color, _, color, _)) = self.overrides.get(entity) else {
            return default;
        };
        let color = match object {
            DebugRenderObject::Collider(..) | DebugRenderObject::ColliderAabb(..) => {
                collider_color.map(|c| c.0).or(color.map(|c| c.0))
            }
            _ => color.map(|c| c.0),
        };

        color.map(|co: Hsla| co.to_f32_array()).unwrap_or(default)
    }
}

impl<'world, 'state, 'world2, 'state2, 'q, 'p> DebugRenderBackend
    for BevyLinesRenderBackend<'world, 'state, 'world2, 'state2, 'q, 'p>
{
    fn filter_object(&self, object: DebugRenderObject) -> bool {
        match object {
            DebugRenderObject::Collider(_, co) => self
                .collider_visibility(co)
                .unwrap_or(self.default_collider_debug == ColliderDebug::AlwaysRender),
            DebugRenderObject::ColliderAabb(_, co, _) => {
                self.collider_visibility(co).unwrap_or(true)
            }
            DebugRenderObject::ContactPair(_, co1, co2) => {
                self.collider_visibility(co1) != Some(false)
                    && self.collider_visibility(co2) != Some(false)
            }
            DebugRenderObject::RigidBody(..)
            | DebugRenderObject::ImpulseJoint(..)
            | DebugRenderObject::MultibodyJoint(..)
            | DebugRenderObject::SoftBody(..) => Self::object_entity(&object)
                .and_then(|entity| self.overrides.get(entity).ok())
                .and_then(|(_, _, _, visibility)| visibility)
                .is_none_or(|v| *v == DebugRenderVisibility::Visible),
        }
    }

    #[cfg(feature = "dim2")]
    fn draw_line(&mut self, object: DebugRenderObject, a: Vector, b: Vector, color: [f32; 4]) {
        let color = self.object_color(object, color);
        self.gizmos.line(
            [a.x, a.y, 0.0].into(),
            [b.x, b.y, 0.0].into(),
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }

    #[cfg(feature = "dim3")]
    fn draw_line(&mut self, object: DebugRenderObject, a: Vector, b: Vector, color: [f32; 4]) {
        let color = self.object_color(object, color);
        self.gizmos.line(
            [a.x, a.y, a.z].into(),
            [b.x, b.y, b.z].into(),
            Color::hsla(color[0], color[1], color[2], color[3]),
        )
    }
}

fn debug_render_scene(
    rapier_context: Query<(
        &RapierContextSimulation,
        &RapierContextColliders,
        &RapierContextJoints,
        &RapierRigidBodySet,
    )>,
    mut render_context: ResMut<DebugRenderContext>,
    mut sync_state: ResMut<DebugRenderSyncState>,
    mut gizmos: Gizmos,
    overrides: Query<DebugOverrides>,
) {
    if !render_context.enabled {
        return;
    }

    let render_context = &mut *render_context;
    render_context.sync_pipeline(&mut sync_state);

    for (rapier_context, rapier_context_colliders, joints, rigidbody_set) in rapier_context.iter() {
        let mut backend = BevyLinesRenderBackend {
            overrides: &overrides,
            default_collider_debug: render_context.default_collider_debug,
            gizmos: &mut gizmos,
        };

        if render_context.scale_lengths_by_length_unit {
            render_context.pipeline.style = scale_style(
                render_context.style,
                rapier_context.integration_parameters.length_unit,
            );
        }
        render_context.pipeline.render(
            &mut backend,
            &rigidbody_set.bodies,
            &rapier_context_colliders.colliders,
            &joints.impulse_joints,
            &joints.multibody_joints,
            &rapier_context.narrow_phase,
            &rigidbody_set.soft_bodies,
        );
        render_context.pipeline.style = render_context.style;
    }
}
