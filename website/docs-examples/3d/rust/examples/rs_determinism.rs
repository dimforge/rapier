use nalgebra::{vector, ComplexField};
use rapier3d::geometry::ColliderBuilder;

fn main() {
    // DOCUSAURUS: Determinism start
    // WRONG version:
    // The following will not work cross-platform-deterministically because the values
    // given to `ColliderBuilder::translation` won't be cross-platform deterministic.
    let collider = ColliderBuilder::ball(0.5)
        .translation(vector![1.0.sqrt(), 2.0.sin(), 3.0.cos()])
        .build();

    // CORRECT version:
    // The following will work cross-platform-deterministically because we use the
    // functions from nalgebra.
    let collider = ColliderBuilder::ball(0.5)
        .translation(vector![
            ComplexField::sqrt(1.0),
            ComplexField::sin(2.0),
            ComplexField::cos(3.0),
        ])
        .build();
    // DOCUSAURUS: Determinism stop
}
