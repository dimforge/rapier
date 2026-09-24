pub use self::ball::*;
pub use self::capsule::*;
pub use self::collider_view::*;
pub use self::compound::*;
pub use self::cuboid::*;
pub use self::halfspace::*;
pub use self::heightfield::*;
pub use self::polyline::*;
pub use self::round_shape::*;
pub use self::segment::*;
pub use self::triangle::*;
pub use self::trimesh::*;
pub use self::voxels::*;

#[cfg(feature = "dim2")]
pub use self::convex_polygon::*;

#[cfg(feature = "dim3")]
pub use self::{cone::*, convex_polyhedron::*, cylinder::*};

mod ball;
mod capsule;
mod collider_view;
mod compound;
mod cuboid;
mod halfspace;
mod heightfield;
mod polyline;
mod round_shape;
mod segment;
mod triangle;
mod trimesh;
mod voxels;

#[cfg(feature = "dim2")]
mod convex_polygon;

#[cfg(feature = "dim3")]
mod cone;
#[cfg(feature = "dim3")]
mod convex_polyhedron;
#[cfg(feature = "dim3")]
mod cylinder;
