//! Reproduction of issue #970: a single always-awake dynamic body carrying 2,210 convex
//! colliders (130 copies of a 17-part convex decomposition, all at the same pose), in a
//! zero-gravity world with nothing else. The body spins forever, so the step cost is
//! entirely spent processing the moving colliders of that one body.
//!
//! Set [`WITH_COMPOUND_COMPARISON`] to `true` to also spawn a second body carrying the
//! very same parts as one compound-shape collider per copy: the broad phase then sees
//! 130 colliders for it instead of 2,210.

use rapier_testbed2d::TestbedViewer;
use rapier2d::prelude::*;

/// The poster's 17 convex polygons (a convex decomposition of a complex outline),
/// in the original 0.01-scaled coordinates.
// The vertex data is verbatim from the issue's JS reproduction, where two vertices carry
// f64 precision.
#[allow(clippy::excessive_precision)]
fn decomposition_parts() -> Vec<Vec<Vector>> {
    let parts: &[&[[f32; 2]]] = &[
        &[[525.0, 104.0], [540.0, 104.0], [419.0, 119.0]],
        &[[419.0, 119.0], [449.0, 74.0], [510.0, 59.0], [525.0, 104.0]],
        &[
            [299.0, 134.0],
            [419.0, 119.0],
            [540.0, 104.0],
            [540.0, 134.0],
        ],
        &[
            [315.0, 450.0],
            [179.0, 284.0],
            [179.0, 254.0],
            [224.0, 224.0],
        ],
        &[
            [224.0, 224.0],
            [224.0, 223.0],
            [299.0, 134.0],
            [540.0, 134.0],
            [555.0, 134.0],
            [555.0, 209.0],
            [359.0, 465.0],
            [315.0, 450.0],
        ],
        &[
            [119.0, 359.0],
            [134.0, 314.0],
            [179.0, 284.0],
            [315.0, 450.0],
            [315.0, 465.0],
            [300.0, 465.0],
        ],
        &[[164.0, 510.0], [134.0, 495.0], [240.0, 510.0]],
        &[
            [240.0, 510.0],
            [240.0, 525.0],
            [164.0, 525.0],
            [164.0, 510.0],
        ],
        &[
            [134.0, 495.0],
            [104.0, 359.0],
            [119.0, 359.0],
            [300.0, 465.0],
            [270.0, 510.0],
            [240.0, 510.0],
        ],
        &[[615.0, 269.0], [660.0, 284.0], [660.0, 359.0]],
        &[
            [673.6813186813187, 390.010989010989],
            [660.0, 359.0],
            [675.0, 359.0],
            [675.0, 389.0],
        ],
        &[
            [675.0, 389.0],
            [735.0, 434.0],
            [735.0, 495.0],
            [720.0, 495.0],
            [673.6813186813187, 390.010989010989],
        ],
        &[
            [645.0, 540.0],
            [645.0, 555.0],
            [494.0, 555.0],
            [494.0, 540.0],
        ],
        &[
            [705.0, 525.0],
            [645.0, 540.0],
            [494.0, 540.0],
            [464.0, 540.0],
            [464.0, 525.0],
        ],
        &[
            [660.0, 359.0],
            [720.0, 495.0],
            [705.0, 525.0],
            [464.0, 525.0],
            [434.0, 525.0],
            [434.0, 510.0],
        ],
        &[
            [660.0, 359.0],
            [434.0, 510.0],
            [404.0, 510.0],
            [404.0, 495.0],
        ],
        &[
            [404.0, 495.0],
            [359.0, 495.0],
            [359.0, 465.0],
            [555.0, 209.0],
            [570.0, 209.0],
            [615.0, 269.0],
            [660.0, 359.0],
        ],
    ];

    const SCALING: f32 = 0.01;
    parts
        .iter()
        .map(|part| {
            part.iter()
                .map(|[x, y]| Vector::new(x * SCALING, y * SCALING))
                .collect()
        })
        .collect()
}

const NUM_COPIES: usize = 130;

/// Spawn a second body using compound shapes instead of individual colliders, for
/// an A/B comparison of the two approaches.
const WITH_COMPOUND_COMPARISON: bool = false;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World: no gravity, and the body is never allowed to sleep — the whole step cost
     * comes from the pipeline processing the colliders of a single body.
     */
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    let parts = decomposition_parts();

    /*
     * One dynamic body with 130 × 17 = 2,210 individual convex colliders, all at the
     * same pose (the poster's scene, with `wakeUp()` before every step replaced by
     * disabling sleep). The constant spin never decays (no gravity, damping, or
     * contacts), so the colliders keep moving indefinitely and the scene exercises the
     * broad phase's moving-leaves path on every step, not just the awake-body one.
     */
    let body = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().can_sleep(false).angvel(1.0));
    for _ in 0..NUM_COPIES {
        for part in &parts {
            let collider = ColliderBuilder::convex_polyline(part.clone())
                .expect("the decomposition parts are valid convex polygons")
                .density(1.0);
            world.insert_collider(collider, Some(body));
        }
    }

    /*
     * The optional comparison body: the same 130 copies of the decomposition, but each
     * copy is a single compound-shape collider (17 parts each), so the broad phase sees
     * 130 colliders instead of 2,210.
     */
    if WITH_COMPOUND_COMPARISON {
        let compound_parts: Vec<_> = parts
            .iter()
            .map(|part| {
                (
                    Pose::IDENTITY,
                    SharedShape::convex_polyline(part.clone())
                        .expect("the decomposition parts are valid convex polygons"),
                )
            })
            .collect();
        let compound_body = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(10.0, 0.0))
                .can_sleep(false)
                .angvel(1.0),
        );
        for _ in 0..NUM_COPIES {
            let collider = ColliderBuilder::compound(compound_parts.clone()).density(1.0);
            world.insert_collider(collider, Some(compound_body));
        }
    }

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(5.0, 3.0), 40.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
