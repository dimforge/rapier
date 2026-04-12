use rapier_testbed2d::Testbed;
use rapier2d::na::ComplexField;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let nsubdivs = 2000;
    let step_size = ground_size / (nsubdivs as f32);
    let mut points = Vec::new();

    points.push(Vector::new(-ground_size / 2.0, 40.0));
    for i in 1..nsubdivs - 1 {
        let x = -ground_size / 2.0 + i as f32 * step_size;
        let y = ComplexField::cos(i as f32 * step_size) * 2.0;
        points.push(Vector::new(x, y));
    }
    points.push(Vector::new(ground_size / 2.0, 40.0));

    let rigid_body = RigidBodyBuilder::fixed();
    let collider = ColliderBuilder::polyline(points, None);
    let _ = world.insert(rigid_body, collider);

    /*
     * Create the cubes
     */
    let num = 20;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 3.0;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(x, y));

            if j % 2 == 0 {
                let collider = ColliderBuilder::cuboid(rad, rad);
                let _ = world.insert(rigid_body, collider);
            } else {
                let collider = ColliderBuilder::ball(rad);
                let _ = world.insert(rigid_body, collider);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::ZERO, 10.0);
}
