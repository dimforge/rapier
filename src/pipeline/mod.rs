//! Structure for combining the various physics components to perform an actual simulation.

pub use collision_pipeline::CollisionPipeline;
pub use event_handler::{ChannelEventCollector, EventHandler};
#[cfg(feature = "fluids")]
pub use fluids_pipeline::FluidsPipeline;
pub use physics_pipeline::PhysicsPipeline;
pub use query_pipeline::QueryPipeline;

mod collision_pipeline;
mod event_handler;
#[cfg(feature = "fluids")]
mod fluids_pipeline;
mod physics_pipeline;
mod query_pipeline;
