use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground.
     */
    let ground_size = 20.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, 0.4)
        .friction(0.15)
        // .restitution(0.5)
        ;
    let (_ground_handle, _) = world.insert(rigid_body, collider);

    /*
     * Rolling ball
     */
    let ball_rad = 0.1;
    let rb = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.2, 0.0))
        .linvel(Vector::new(10.0, 0.0, 0.0));
    let collider = ColliderBuilder::ball(ball_rad).density(100.0);
    let (ball_handle, _) = world.insert(rb, collider);

    let mut linvel = Vector::ZERO;
    let mut angvel = AngVector::ZERO;
    let mut pos = Pose::IDENTITY;
    let mut step = 0;
    let snapped_frame = 51;

    testbed.add_callback(move |_, physics, _, _| {
        step += 1;

        // Snap the ball velocity or restore it.
        let ball = physics.bodies.get_mut(ball_handle).unwrap();

        if step == snapped_frame {
            linvel = ball.linvel();
            angvel = ball.angvel();
            pos = *ball.position();
        }

        if step == 100 {
            ball.set_linvel(linvel, true);
            ball.set_angvel(angvel, true);
            ball.set_position(pos, true);
            step = snapped_frame;
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
