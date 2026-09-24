import numpy as np

vertices = np.array([[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [1.0, 1.0, 0.0]])
indices = np.array([[0, 2, 1]], dtype=np.uint32)
heights = np.array([[0.0, 0.5], [1.0, 0.0]], dtype=np.float32)
scale = (1.0, 1.0, 1.0)

# DOCUSAURUS: Creation start
import math

import rapier3d as rp

# The world that will contain our colliders.
world = rp.PhysicsWorld()

# Builder for a ball-shaped collider.
_ = rp.Collider.ball(0.5)
# Builder for a cuboid-shaped collider.
_ = rp.Collider.cuboid(0.5, 0.2, 0.1)
# Builder for a capsule-shaped collider. The capsule principal axis is the `x` coordinate axis.
_ = rp.Collider.capsule_x(0.5, 0.2)
# Builder for a capsule-shaped collider. The capsule principal axis is the `y` coordinate axis.
_ = rp.Collider.capsule_y(0.5, 0.2)
# Builder for a capsule-shaped collider. The capsule principal axis is the `z` coordinate axis.
_ = rp.Collider.capsule_z(0.5, 0.2)
# Builder for a triangle-mesh-shaped collider.
_ = rp.Collider.trimesh(vertices, indices)
# Builder for a heightfield-shaped collider.
_ = rp.Collider.heightfield(heights, scale)
# Builder for a collider with the given shape.
collider = (
    rp.Collider.new(rp.SharedShape.ball(0.5))
    # The collider translation wrt. the body it is attached to.
    # Default: the zero vector.
    .translation((1.0, 2.0, 3.0))
    # The collider rotation wrt. the body it is attached to, as a rotation vector (axis * angle).
    # Default: the identity rotation.
    .rotation((0.0, math.pi, 0.0))
    # The collider position wrt. the body it is attached to.
    # Default: the identity isometry.
    .position(rp.Isometry3((1.0, 2.0, 3.0), rp.Rotation3.from_scaled_axis((0.0, math.pi, 0.0))))
    # The collider density. If non-zero the collider's mass and angular inertia will be added
    # to the inertial properties of the body it is attached to.
    # Default: 1.0
    .density(1.3)
    # The friction coefficient of this collider.
    # Default: 0.5
    .friction(0.8)
    # Whether this collider is a sensor.
    # Default: False
    .sensor(True)
    # All done, actually build the collider.
    .build()
)

# Insert the collider into the world, without attaching it to a rigid-body.
collider_handle = world.add_collider(collider)

rigid_body_handle = world.add_body(rp.RigidBody.dynamic())
# Or insert the collider into the world and attach it to a rigid-body.
handle = world.add_collider(collider, parent=rigid_body_handle)
# DOCUSAURUS: Creation stop

# DOCUSAURUS: ColliderType1 start
# Set the collider type when the collider is created.
collider = rp.Collider.ball(0.5).sensor(True).build()
# DOCUSAURUS: ColliderType1 stop

# DOCUSAURUS: ColliderType2 start
# Set the collider type after the collider creation.
collider = world.colliders[collider_handle]
collider.is_sensor = True
assert collider.is_sensor
# DOCUSAURUS: ColliderType2 stop

# DOCUSAURUS: VoxelsPoints start
# A voxels shape from arbitrary points.
shape = rp.Collider.voxels_from_points(
    (1.0, 1.0, 1.0),
    np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]),
)
# DOCUSAURUS: VoxelsPoints stop

# The triangle mesh of a unit cube.
mesh_vertices = np.array(
    [
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0],
    ]
)
mesh_indices = np.array(
    [
        [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
        [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [0, 4, 7], [0, 7, 3],
    ],
    dtype=np.uint32,
)
# DOCUSAURUS: VoxelsMesh start
shape = rp.SharedShape.voxelized_mesh(mesh_vertices, mesh_indices, 0.2, rp.FillMode.flood_fill())
# DOCUSAURUS: VoxelsMesh stop
assert shape.as_voxels().num_voxels() > 0

shape = rp.SharedShape.ball(0.5)
pos1 = rp.Isometry3.from_translation(0.0, 1.0, 0.0)
pos2 = rp.Isometry3.from_translation(0.0, 1.0, 0.0)
# DOCUSAURUS: Compound start
_ = rp.Collider.compound([(pos1, shape), (pos2, shape)])
# DOCUSAURUS: Compound stop

# DOCUSAURUS: Mass start
rigid_body_handle = world.add_body(rp.RigidBody.dynamic())
# First option: by setting the density of the collider (or we could just leave
#               its default value 1.0).
collider = rp.Collider.cuboid(1.0, 2.0, 3.0).density(2.0).build()
# Second option: by setting the mass of the collider.
collider = rp.Collider.cuboid(1.0, 2.0, 3.0).mass(0.8).build()
# Third option: by setting the mass-properties explicitly.
collider = (
    rp.Collider.cuboid(1.0, 2.0, 3.0)
    .mass_properties(
        rp.MassProperties(
            local_com=(0.0, 1.0, 0.0),
            mass=0.5,
            principal_inertia=(0.3, 0.2, 0.1),
        )
    )
    .build()
)
# When the collider is attached, the rigid-body's mass and angular
# inertia is automatically updated to take the collider into account.
world.add_collider(collider, parent=rigid_body_handle)
# DOCUSAURUS: Mass stop

# DOCUSAURUS: Position1 start
# Set the collider position when the collider is created.
collider = (
    rp.Collider.ball(0.5)
    .translation((1.0, 2.0, 3.0))
    .rotation((0.1, 0.2, 0.4))
    # Set both translation and rotation at once.
    .position(rp.Isometry3((1.0, 2.0, 3.0), rp.Rotation3.from_scaled_axis((0.1, 0.2, 0.4))))
    .build()
)
# DOCUSAURUS: Position1 stop

# DOCUSAURUS: Position2 start
# Set the collider position after the collider creation.
collider = world.colliders[collider_handle]
collider.translation = (1.0, 2.0, 3.0)
collider.rotation = rp.Rotation3.from_scaled_axis((0.1, 0.2, 0.4))
# Set both the translation and rotation at once.
collider.position = rp.Isometry3((1.0, 2.0, 3.0), rp.Rotation3.from_scaled_axis((0.1, 0.2, 0.4)))
assert collider.translation == (1.0, 2.0, 3.0)
assert (collider.rotation.scaled_axis - (0.1, 0.2, 0.4)).norm() < 1.0e-6
# DOCUSAURUS: Position2 stop

# DOCUSAURUS: Position3 start
rigid_body_handle = world.add_body(rp.RigidBody.dynamic())
collider = rp.Collider.ball(0.5).translation((1.0, 2.0, 3.0)).build()
# Attach the collider to the rigid-body. The collider's position wrt. the rigid-body
# is automatically set to the collider current position when this method is called.
attached_collider_handle = world.add_collider(collider, parent=rigid_body_handle)
# DOCUSAURUS: Position3 stop

# DOCUSAURUS: Position4 start
# Set the collider position wrt. its parent after the collider creation.
collider = world.colliders[attached_collider_handle]
collider.position_wrt_parent = rp.Isometry3.from_translation(1.0, 2.0, 3.0)
assert collider.position_wrt_parent.translation == (1.0, 2.0, 3.0)
# DOCUSAURUS: Position4 stop

# DOCUSAURUS: Friction1 start
# Set the friction coefficient and friction combine rule
# when the collider is created.
collider = (
    rp.Collider.ball(0.5)
    .friction(0.7)
    .friction_combine_rule(rp.CoefficientCombineRule.MIN)
    .build()
)
# DOCUSAURUS: Friction1 stop

# DOCUSAURUS: Friction2 start
# Set the friction coefficient and friction combine rule
# after the collider creation.
collider = world.colliders[collider_handle]
collider.friction = 0.7
collider.friction_combine_rule = rp.CoefficientCombineRule.MIN
assert math.isclose(collider.friction, 0.7, rel_tol=1.0e-6)
assert collider.friction_combine_rule == rp.CoefficientCombineRule.MIN
# DOCUSAURUS: Friction2 stop

# DOCUSAURUS: Restitution1 start
# Set the restitution coefficient and restitution combine rule
# when the collider is created.
collider = (
    rp.Collider.ball(0.5)
    .restitution(0.7)
    .restitution_combine_rule(rp.CoefficientCombineRule.MIN)
    .build()
)
# DOCUSAURUS: Restitution1 stop

# DOCUSAURUS: Restitution2 start
# Set the restitution coefficient and restitution combine rule
# after the collider creation.
collider = world.colliders[collider_handle]
collider.restitution = 0.7
collider.restitution_combine_rule = rp.CoefficientCombineRule.MIN
assert math.isclose(collider.restitution, 0.7, rel_tol=1.0e-6)
assert collider.restitution_combine_rule == rp.CoefficientCombineRule.MIN
# DOCUSAURUS: Restitution2 stop

# DOCUSAURUS: Groups1 start
# Set the collision groups and solver groups when the collider is created.
collider = (
    rp.Collider.ball(0.5)
    .collision_groups(
        rp.InteractionGroups(
            memberships=rp.Group.GROUP_1 | rp.Group.GROUP_3 | rp.Group.GROUP_4,
            filter=rp.Group.GROUP_3,
            test_mode=rp.InteractionTestMode.AND,
        )
    )
    .solver_groups(
        rp.InteractionGroups(
            memberships=rp.Group.GROUP_1 | rp.Group.GROUP_2,
            filter=rp.Group.GROUP_1 | rp.Group.GROUP_2 | rp.Group.GROUP_4,
            test_mode=rp.InteractionTestMode.AND,
        )
    )
    .build()
)
# DOCUSAURUS: Groups1 stop

# DOCUSAURUS: Groups2 start
# Set the collision groups and solver groups after the collider creation.
collider = world.colliders[collider_handle]
collision_groups = rp.InteractionGroups(
    memberships=rp.Group.GROUP_1 | rp.Group.GROUP_3 | rp.Group.GROUP_4,
    filter=rp.Group.GROUP_3,
    test_mode=rp.InteractionTestMode.AND,
)
solver_groups = rp.InteractionGroups(
    memberships=rp.Group.GROUP_1 | rp.Group.GROUP_2,
    filter=rp.Group.GROUP_1 | rp.Group.GROUP_2 | rp.Group.GROUP_4,
    test_mode=rp.InteractionTestMode.AND,
)
collider.collision_groups = collision_groups
collider.solver_groups = solver_groups
assert collider.collision_groups == collision_groups
assert collider.solver_groups == solver_groups
# DOCUSAURUS: Groups2 stop

# DOCUSAURUS: ActiveCollisionTypes1 start
# Set the active collision types when the collider is created.
collider = (
    rp.Collider.ball(0.5)
    .active_collision_types(
        rp.ActiveCollisionTypes.default_types() | rp.ActiveCollisionTypes.KINEMATIC_FIXED
    )
    .build()
)
# DOCUSAURUS: ActiveCollisionTypes1 stop

# DOCUSAURUS: ActiveCollisionTypes2 start
# Set the active collision types after the collider creation.
collider = world.colliders[collider_handle]
collider.active_collision_types = (
    rp.ActiveCollisionTypes.default_types() | rp.ActiveCollisionTypes.KINEMATIC_FIXED
)
assert collider.active_collision_types.contains(rp.ActiveCollisionTypes.DYNAMIC_KINEMATIC)
assert collider.active_collision_types.contains(rp.ActiveCollisionTypes.KINEMATIC_FIXED)
# DOCUSAURUS: ActiveCollisionTypes2 stop

# DOCUSAURUS: ActiveEvents1 start
# Set the active events when the collider is created.
collider = rp.Collider.ball(0.5).active_events(rp.ActiveEvents.COLLISION_EVENTS).build()
# DOCUSAURUS: ActiveEvents1 stop

# DOCUSAURUS: ActiveEvents2 start
# Set the active events after the collider creation.
collider = world.colliders[collider_handle]
collider.active_events = rp.ActiveEvents.COLLISION_EVENTS
assert collider.active_events.contains(rp.ActiveEvents.COLLISION_EVENTS)
# DOCUSAURUS: ActiveEvents2 stop

# DOCUSAURUS: ActiveHooks1 start
# Set the active hooks when the collider is created.
collider = (
    rp.Collider.ball(0.5)
    .active_hooks(rp.ActiveHooks.FILTER_CONTACT_PAIRS | rp.ActiveHooks.MODIFY_SOLVER_CONTACTS)
    .build()
)
# DOCUSAURUS: ActiveHooks1 stop

# DOCUSAURUS: ActiveHooks2 start
# Set the active hooks after the collider creation.
collider = world.colliders[collider_handle]
collider.active_hooks = rp.ActiveHooks.FILTER_CONTACT_PAIRS | rp.ActiveHooks.MODIFY_SOLVER_CONTACTS
assert collider.active_hooks.contains(rp.ActiveHooks.FILTER_CONTACT_PAIRS)
assert collider.active_hooks.contains(rp.ActiveHooks.MODIFY_SOLVER_CONTACTS)
# DOCUSAURUS: ActiveHooks2 stop

# DOCUSAURUS: UserData1 start
# Set the user-data when the collider is created.
collider = rp.Collider.ball(0.5).user_data(42).build()
# DOCUSAURUS: UserData1 stop

# DOCUSAURUS: UserData2 start
# Set the user-data after the collider creation.
collider = world.colliders[collider_handle]
collider.user_data = 42
assert collider.user_data == 42
# DOCUSAURUS: UserData2 stop

# DOCUSAURUS: ContactSkin start
# Set the contact skin when the collider is created.
collider = rp.Collider.ball(0.5).contact_skin(0.01).build()
# Set the contact skin after the collider creation.
collider = world.colliders[collider_handle]
collider.contact_skin = 0.01
assert math.isclose(collider.contact_skin, 0.01, rel_tol=1.0e-6)
# DOCUSAURUS: ContactSkin stop
