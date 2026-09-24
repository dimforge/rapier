import rapier3d as rp

world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
world.colliders.insert(rp.Collider.cuboid(100.0, 0.1, 100.0).build())
body_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 1.0, 0.0)),
    colliders=[rp.Collider.ball(0.5)],
)

# DOCUSAURUS: Pid start
# The proportional, integral, and derivative gains of the controller, acting on the linear
# axes only: the body is pushed toward its target without its rotation being controlled.
axes = rp.AxesMask.LIN_X | rp.AxesMask.LIN_Y | rp.AxesMask.LIN_Z
pid = rp.PidController(axes=axes, Kp=60.0, Ki=0.0, Kd=0.8)
target = rp.Isometry3.from_translation(3.0, 2.0, 0.0)

for _ in range(200):
    dt = world.integration_parameters.dt
    body = world.rigid_bodies[body_handle]
    # The correction is the velocity change bringing the body closer to its target pose
    # (and to its target velocities, zero here).
    correction = pid.rigid_body_correction(dt, body, target, target_vels=rp.RigidBodyVelocity())
    body.linvel = body.linvel + correction.linear
    body.angvel = body.angvel + correction.angular

    world.step()
# DOCUSAURUS: Pid stop

final_translation = world.rigid_bodies[body_handle].translation
print("Body translation:", final_translation)
assert (final_translation - target.translation).norm() < 0.05
assert pid.lin_kp == rp.Vec3(60.0, 60.0, 60.0)
assert pid.ang_kd == rp.Vec3(0.8, 0.8, 0.8)
