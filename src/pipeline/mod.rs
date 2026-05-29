//! Structure for combining the various physics components to perform an actual simulation.

#[cfg(feature = "alloc")]
pub use collision_pipeline::CollisionPipeline;
pub use event_handler::ActiveEvents;
#[cfg(all(feature = "std", feature = "alloc"))]
pub use event_handler::ChannelEventCollector;
#[cfg(feature = "alloc")]
pub use event_handler::EventHandler;
pub use physics_hooks::ActiveHooks;
#[cfg(feature = "alloc")]
pub use physics_hooks::{ContactModificationContext, PairFilterContext, PhysicsHooks};
#[cfg(feature = "alloc")]
pub use physics_pipeline::PhysicsPipeline;
#[cfg(feature = "alloc")]
pub use physics_world::PhysicsWorld;
#[cfg(feature = "alloc")]
pub use query_pipeline::{QueryFilter, QueryFilterFlags, QueryPipeline, QueryPipelineMut};

#[cfg(all(feature = "debug-render", feature = "alloc"))]
pub use self::debug_render_pipeline::{
    DebugColor, DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline,
    DebugRenderStyle,
};

#[cfg(feature = "alloc")]
mod collision_pipeline;
mod event_handler;
mod physics_hooks;
#[cfg(feature = "alloc")]
mod physics_pipeline;
#[cfg(feature = "alloc")]
mod physics_world;
#[cfg(feature = "alloc")]
mod query_pipeline;
#[cfg(feature = "alloc")]
mod user_changes;

#[cfg(all(feature = "debug-render", feature = "alloc"))]
mod debug_render_pipeline;
