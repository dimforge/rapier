import math

import rapier3d as rp

world = rp.PhysicsWorld()

body_handle1 = world.add_body(rp.RigidBody.dynamic(), colliders=[rp.Collider.ball(0.5)])
body_handle2 = world.add_body(rp.RigidBody.dynamic(), colliders=[rp.Collider.ball(0.5)])

# DOCUSAURUS: FixedJoint start
# NOTE: setting the local anchors sets the translation part of the local frames.
joint = (
    rp.FixedJoint.builder()
    .local_anchor1((0.0, 1.0, 0.0))
    .local_anchor2((0.0, -3.0, 0.0))
)
world.impulse_joints.insert(body_handle1, body_handle2, joint)
# DOCUSAURUS: FixedJoint stop

# DOCUSAURUS: SphericalJoint start
joint = (
    rp.SphericalJoint.builder()
    .local_anchor1((0.0, 0.0, 1.0))
    .local_anchor2((0.0, 0.0, -3.0))
)
world.impulse_joints.insert(body_handle1, body_handle2, joint)
# DOCUSAURUS: SphericalJoint stop

# DOCUSAURUS: RevoluteJoint start
x = (1.0, 0.0, 0.0)
joint = (
    rp.RevoluteJoint.builder(axis=x)
    .local_anchor1((0.0, 0.0, 1.0))
    .local_anchor2((0.0, 0.0, -3.0))
)
world.impulse_joints.insert(body_handle1, body_handle2, joint)
# DOCUSAURUS: RevoluteJoint stop

# DOCUSAURUS: PrismaticJoint start
x = (1.0, 0.0, 0.0)
joint = (
    rp.PrismaticJoint.builder(axis=x)
    .local_anchor1((0.0, 0.0, 1.0))
    .local_anchor2((0.0, 0.0, -3.0))
    .limits(-2.0, 5.0)
)
world.impulse_joints.insert(body_handle1, body_handle2, joint)
# DOCUSAURUS: PrismaticJoint stop

# DOCUSAURUS: Motor start
x = (1.0, 0.0, 0.0)
joint = (
    rp.PrismaticJoint.builder(axis=x)
    .local_anchor1((0.0, 0.0, 1.0))
    .local_anchor2((0.0, 0.0, -3.0))
    .motor_velocity(1.0, 0.5)
)
joint_handle = world.impulse_joints.insert(body_handle1, body_handle2, joint)
# DOCUSAURUS: Motor stop
motor = world.impulse_joints[joint_handle].data.motor(rp.JointAxis.LIN_X)
assert motor is not None and motor.target_vel == 1.0 and motor.damping == 0.5

# DOCUSAURUS: GenericJoint start
# A cylindrical joint: only the translation along, and the rotation around, the X axis are free.
locked_axes = rp.JointAxesMask.LIN_Y | rp.JointAxesMask.LIN_Z | rp.JointAxesMask.ANG_Y | rp.JointAxesMask.ANG_Z
# Make the locked axes springy instead of rigid.
# Default: a natural frequency of 1.0e6 Hz and a damping ratio of 1.0.
softness = rp.SpringCoefficients(natural_frequency=10.0, damping_ratio=1.0)
joint = (
    rp.GenericJoint.builder(locked_axes=locked_axes)
    .local_anchor1((0.0, 0.0, 1.0))
    .local_anchor2((0.0, 0.0, -3.0))
    # Limit the relative translation along the X axis.
    .limits(rp.JointAxis.LIN_X, -2.0, 5.0)
    # Allow contacts between the colliders of the two rigid-bodies.
    # Default: True
    .contacts_enabled(True)
    .softness(softness)
    # An integer (up to 128 bits) freely available to the application.
    # Default: 0
    .user_data(42)
)
world.impulse_joints.insert(body_handle1, body_handle2, joint)
# DOCUSAURUS: GenericJoint stop

# Check that the defaults documented above are accurate.
defaults = rp.GenericJoint()
assert defaults.contacts_enabled and defaults.user_data == 0
assert defaults.softness.natural_frequency == 1.0e6 and defaults.softness.damping_ratio == 1.0

# DOCUSAURUS: CoupledAxes start
# The relative translation along all the axes is free, but its length is limited to 2.0 (like a rope).
joint = (
    rp.GenericJoint.builder()
    .coupled_axes(rp.JointAxesMask.LIN_AXES)
    # Only the limits of the first coupled axis are used.
    .limits(rp.JointAxis.LIN_X, 0.0, 2.0)
)
world.impulse_joints.insert(body_handle1, body_handle2, joint)
# DOCUSAURUS: CoupledAxes stop

# DOCUSAURUS: RopeSpringJoint start
# The distance between the anchors can't exceed 2.0.
rope = rp.RopeJoint.builder(max_distance=2.0)
world.impulse_joints.insert(body_handle1, body_handle2, rope)
# A spring with a rest length of 2.0, a stiffness of 10.0, and a damping of 0.5.
spring = rp.SpringJoint.builder(rest_length=2.0, stiffness=10.0, damping=0.5)
world.impulse_joints.insert(body_handle1, body_handle2, spring)
# DOCUSAURUS: RopeSpringJoint stop

world.step()

# DOCUSAURUS: JointImpulses start
max_impulse = 10.0
world.step()
# The impulses applied by the joint during the last step: 3 linear components, then 3 angular components.
impulses = world.impulse_joints[joint_handle].impulses
# Break the joint if it has to pull its rigid-bodies too strongly to keep them together.
if math.hypot(*impulses[:3]) > max_impulse:
    world.impulse_joints.remove(joint_handle)
# DOCUSAURUS: JointImpulses stop
assert joint_handle in world.impulse_joints

# DOCUSAURUS: ModifyJoint start
# A live view of the joint: modifying it modifies the joint inserted into the world (and wakes up its rigid-bodies).
joint = world.impulse_joints[joint_handle]
# Change the motor of the joint.
joint.data.set_motor_velocity(rp.JointAxis.LIN_X, 2.0, 0.5)
# Forbid contacts between the colliders of its rigid-bodies.
joint.data.contacts_enabled = False
# Disable the joint: it stays attached to its rigid-bodies but is ignored by the solver.
joint.data.set_enabled(False)
# The rigid-bodies attached by the joint.
print("The joint attaches", joint.body1, "and", joint.body2)
# Attach the joint to other rigid-bodies (its handle doesn't change).
world.impulse_joints.set_bodies(joint_handle, body_handle2, body_handle1)
# Iterate through all the impulse joints of the world.
for handle, joint in world.impulse_joints:
    print(handle, "attaches", joint.body1, "and", joint.body2)
# The handles of all the impulse joints attached to a rigid-body.
attached_joints = list(world.impulse_joints.attached_joints(body_handle1))
# Remove the joint, waking up its rigid-bodies.
world.impulse_joints.remove(joint_handle)
# DOCUSAURUS: ModifyJoint stop
assert joint_handle not in world.impulse_joints

# DOCUSAURUS: Multibody start
# The root of the multibody: a fixed rigid-body.
root = world.add_body(rp.RigidBody.fixed())
# Three links, each attached to the previous one by a revolute multibody joint.
parent = root
for i in range(1, 4):
    link = world.add_body(
        rp.RigidBody.dynamic(translation=(0.0, 0.0, -2.0 * i)),
        colliders=[rp.Collider.ball(0.5)],
    )
    joint = rp.RevoluteJoint.builder(axis=(1.0, 0.0, 0.0)).local_anchor2((0.0, 0.0, 2.0))
    # `insert` returns None if the joint would make the multibody invalid.
    last_joint = world.multibody_joints.insert(parent, link, joint)
    parent = link
# DOCUSAURUS: Multibody stop
assert last_joint is not None
# Inserting a joint that would close a loop fails.
assert world.multibody_joints.insert(root, parent, rp.SphericalJoint.builder()) is None

world.step()

# DOCUSAURUS: GeneralizedVelocity start
# A live view of the multibody the joint belongs to, and the index of the link attached by the joint.
multibody, link_id = world.multibody_joints.get(last_joint)
# The number of degrees of freedom of the whole multibody (3 revolute joints: 3 DOF).
print("Degrees of freedom:", multibody.ndofs)
# The coordinates of the joint of the link, one per axis: the angle of a revolute joint is the one of its ANG_X axis.
angle = multibody.get_link(link_id).coords[3]
# Read the generalized velocities of the multibody, i.e., the relative angular velocity of each joint.
velocities = multibody.generalized_velocity()
# Stop every joint of the multibody.
multibody.set_generalized_velocity([0.0] * multibody.ndofs)
# DOCUSAURUS: GeneralizedVelocity stop
assert multibody.ndofs == 3 and len(velocities) == 3

# DOCUSAURUS: ModifyMultibodyJoint start
# A live view of the multibody joint, like for impulse joints.
joint = world.multibody_joints[last_joint]
# Change the motor of the joint (its rigid-bodies are woken up at the next step).
joint.data.set_motor_velocity(rp.JointAxis.ANG_X, 1.0, 0.5)
# Changing its locked axes would change the degrees of freedom of the multibody: this raises a ValueError.
# DOCUSAURUS: ModifyMultibodyJoint stop
assert world.multibody_joints[last_joint].data.motor(rp.JointAxis.ANG_X).target_vel == 1.0
try:
    joint.data.locked_axes = rp.JointAxesMask.LOCKED_FIXED_AXES
    raise AssertionError("changing the locked axes of a multibody joint should fail")
except ValueError:
    pass

root = world.add_body(rp.RigidBody.fixed())
parent = root
for i in range(1, 4):
    link = world.add_body(
        rp.RigidBody.dynamic(translation=(0.0, i - 0.5, 0.0)),
        colliders=[rp.Collider.ball(0.5)],
    )
    joint = (
        rp.SphericalJoint.builder()
        .local_anchor1((0.0, 0.5 * (i != 1), 0.0))
        .local_anchor2((0.0, -0.5, 0.0))
    )
    end_effector = world.multibody_joints.insert(parent, link, joint)
    parent = link
# Let the first step finalize the multibody (its fixed root then has no degree of freedom).
world.step()

# DOCUSAURUS: InverseKinematics start
# Only try to reach the target translation, whatever the orientation of the last link.
options = rp.InverseKinematicsOption(constrained_axes=rp.JointAxesMask.LIN_AXES)
target = rp.Isometry3(translation=(0.5, 1.5, 0.0))
# Compute the displacements (one per degree of freedom of the multibody) moving the link attached by
# `end_effector` toward the target. Here, every link is allowed to move.
displacements = world.multibody_joints.inverse_kinematics_for_link(
    world.rigid_bodies, end_effector, target, options, joint_can_move=lambda link: True
)
# Apply them to the generalized coordinates of the multibody.
multibody = world.multibody_joints.multibody(end_effector)
multibody.apply_displacements(displacements)
# The poses of the links are updated by the next step, or right away with:
multibody.forward_kinematics(world.rigid_bodies)
multibody.update_rigid_bodies(world.rigid_bodies, False)
# DOCUSAURUS: InverseKinematics stop
assert len(displacements) == multibody.ndofs
tip = world.rigid_bodies[world.multibody_joints[end_effector].multibody.get_link(3).rigid_body].translation
print("End effector after inverse kinematics:", tip)

# DOCUSAURUS: LoopClosing start
# Five pearls forming a necklace.
pearls = []
for i in range(5):
    angle = 2.0 * math.pi * i / 5.0
    pearl = rp.RigidBody.dynamic(translation=(2.0 * math.cos(angle), 10.0, 2.0 * math.sin(angle)))
    pearls.append(world.add_body(pearl, colliders=[rp.Collider.ball(0.5)]))


def delta(a, b):
    """The translation from the center of the pearl `a` to the center of the pearl `b`."""
    return world.rigid_bodies[b].translation - world.rigid_bodies[a].translation


# Each spherical joint links the centers of two consecutive pearls.
# The first four joints form a multibody (a tree).
for i in range(4):
    joint = rp.SphericalJoint.builder().local_anchor1(delta(pearls[i], pearls[i + 1]))
    world.multibody_joints.insert(pearls[i], pearls[i + 1], joint)
# The fifth joint closes the loop: it has to be an impulse joint.
joint = rp.SphericalJoint.builder().local_anchor1(delta(pearls[4], pearls[0]))
world.impulse_joints.insert(pearls[4], pearls[0], joint)
# DOCUSAURUS: LoopClosing stop

for _ in range(10):
    world.step()
