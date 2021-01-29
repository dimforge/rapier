#[macro_use]
extern crate kiss3d;
extern crate nalgebra as na;
#[cfg(feature = "dim2")]
extern crate ncollide2d as ncollide;
#[cfg(feature = "dim3")]
extern crate ncollide3d as ncollide;
#[cfg(all(feature = "dim2", feature = "other-backends"))]
extern crate nphysics2d as nphysics;
#[cfg(all(feature = "dim3", feature = "other-backends"))]
extern crate nphysics3d as nphysics;
#[cfg(feature = "dim2")]
extern crate parry2d as parry;
#[cfg(feature = "dim3")]
extern crate parry3d as parry;
#[cfg(feature = "dim2")]
extern crate rapier2d as rapier;
#[cfg(feature = "dim3")]
extern crate rapier3d as rapier;

#[macro_use]
extern crate bitflags;

#[cfg(feature = "log")]
#[macro_use]
extern crate log;

pub use crate::engine::GraphicsManager;
pub use crate::harness::plugin::HarnessPlugin;
pub use crate::physics::PhysicsState;
pub use crate::plugin::TestbedPlugin;
pub use crate::testbed::Testbed;

#[cfg(all(feature = "dim2", feature = "other-backends"))]
mod box2d_backend;
mod engine;
pub mod harness;
#[cfg(feature = "other-backends")]
mod nphysics_backend;
pub mod objects;
pub mod physics;
#[cfg(all(feature = "dim3", feature = "other-backends"))]
mod physx_backend;
mod plugin;
mod testbed;
mod ui;

#[cfg(feature = "dim2")]
pub mod math {
    pub type Isometry<N> = na::Isometry2<N>;
    pub type Vector<N> = na::Vector2<N>;
    pub type Point<N> = na::Point2<N>;
    pub type Translation<N> = na::Translation2<N>;
}

#[cfg(feature = "dim3")]
pub mod math {
    pub type Isometry<N> = na::Isometry3<N>;
    pub type Vector<N> = na::Vector3<N>;
    pub type Point<N> = na::Point3<N>;
    pub type Translation<N> = na::Translation3<N>;
}
