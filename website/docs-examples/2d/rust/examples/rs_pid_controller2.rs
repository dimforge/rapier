use rapier2d::control::PidController;
use rapier2d::prelude::*;

fn main() {
    let mut world = PhysicsWorld::new();
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1), None);
    let (body_handle, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.0)),
        ColliderBuilder::ball(0.5),
    );

    // DOCUSAURUS: Pid start
    // The proportional, integral, and derivative gains of the controller, acting on the linear
    // axes only: the body is pushed toward its target without its rotation being controlled.
    let axes = AxesMask::LIN_X | AxesMask::LIN_Y;
    let mut pid = PidController::new(60.0, 0.0, 0.8, axes);
    let target = Vector::new(3.0, 2.0);

    for _ in 0..200 {
        let dt = world.integration_parameters.dt;
        let body = &mut world.bodies[body_handle];
        // The correction is the velocity change bringing the body closer to its target pose.
        let correction = pid.rigid_body_correction(
            dt,
            body,
            Pose::from_translation(target),
            RigidBodyVelocity::zero(),
        );
        let new_velocities = *body.vels() + correction;
        body.set_vels(new_velocities, true);

        world.step();
    }
    // DOCUSAURUS: Pid stop
}
