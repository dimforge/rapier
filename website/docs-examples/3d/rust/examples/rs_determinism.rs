use rapier3d::na::ComplexField;
use rapier3d::prelude::*;

fn main() {
    // DOCUSAURUS: Determinism start
    // WRONG version:
    // The following will not work cross-platform-deterministically because the values
    // given to `ColliderBuilder::translation` won't be cross-platform deterministic.
    let collider = ColliderBuilder::ball(0.5)
        .translation(Vector::new(1.0f32.sqrt(), 2.0f32.sin(), 3.0f32.cos()))
        .build();

    // CORRECT version:
    // The following will work cross-platform-deterministically because we use the
    // functions from nalgebra.
    let collider = ColliderBuilder::ball(0.5)
        .translation(Vector::new(
            ComplexField::sqrt(1.0),
            ComplexField::sin(2.0),
            ComplexField::cos(3.0),
        ))
        .build();
    // DOCUSAURUS: Determinism stop
}
