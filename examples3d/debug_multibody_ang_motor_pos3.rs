use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    let ground = RigidBodyBuilder::fixed().translation(Vector::new(0.0, 0.0, 0.0));
    let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0);
    let (body, _) = world.insert(ground, collider);

    let rigid_body =
        RigidBodyBuilder::dynamic().pose(Pose::from_translation(Vector::new(0.0, 1.0, 0.0)));
    let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0).density(1.0);
    let (body_part, _) = world.insert(rigid_body, collider);

    let joint = SphericalJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 4.0, 0.0))
        .local_anchor2(Vector::new(0.0, 0.0, 0.0))
        .motor_position(JointAxis::AngX, 1.0, 1000.0, 200.0)
        .motor_position(JointAxis::AngY, 0.0, 1000.0, 200.0)
        .motor_position(JointAxis::AngZ, 0.0, 1000.0, 200.0);

    world.insert_multibody_joint(body, body_part, joint);

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(20.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}
