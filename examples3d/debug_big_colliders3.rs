use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::fixed();
    let halfspace = SharedShape::new(HalfSpace::new(Vector::Y));
    let collider = ColliderBuilder::new(halfspace);
    let (_handle, _) = world.insert(rigid_body, collider);

    let mut curr_y = 0.0;
    let mut curr_width = 10_000.0;

    for _ in 0..12 {
        let curr_height = 0.1f32.min(curr_width);
        curr_y += curr_height * 4.0;

        let rigid_body = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, curr_y, 0.0));
        let collider = ColliderBuilder::cuboid(curr_width, curr_height, curr_width);
        let (_handle, _) = world.insert(rigid_body, collider);

        curr_width /= 5.0;
    }

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
