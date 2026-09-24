import rapier3d as rp

world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))

# Create the ground.
world.add_collider(rp.Collider.cuboid(100.0, 0.1, 100.0))

# Create the bouncing ball.
ball_body_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 10.0, 0.0)),
    colliders=[rp.Collider.ball(0.5).restitution(0.7)],
)

# Create the player, a sensor, and a collider with a user-data, for the query filters.
player_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(1.0, 3.0, 3.0)),
    colliders=[rp.Collider.ball(0.5)],
)
world.add_collider(rp.Collider.ball(0.5).translation((1.0, 4.0, 3.0)).sensor(True))
tagged_handle = world.add_collider(rp.Collider.cuboid(1.0, 0.1, 1.0).translation((1.0, 5.0, 3.0)).user_data(10))

# DOCUSAURUS: QueryPipeline start
# Game loop.
for _ in range(10):
    # Stepping the simulation updates the broad-phase the scene queries rely on.
    world.step()

    # The scene queries take into account the positions of the colliders at the end of
    # the last timestep.
    query_pipeline = world.query_pipeline
    # Run the scene queries with `query_pipeline` here.
# DOCUSAURUS: QueryPipeline stop

# DOCUSAURUS: Raycast start
ray = rp.Ray(origin=(1.0, 2.0, 3.0), dir=(0.0, 1.0, 0.0))
max_toi = 4.0
solid = True
query_filter = rp.QueryFilter()

query_pipeline = world.query_pipeline

hit = query_pipeline.cast_ray(ray, max_toi, solid, filter=query_filter)
if hit is not None:
    handle, toi = hit
    # The first collider hit has the handle `handle` and it hit after
    # the ray travelled a distance equal to `ray.dir * toi`.
    hit_point = ray.point_at(toi)  # Same as: `ray.origin + ray.dir * toi`
    print(f"Collider {handle} hit at point {hit_point}")

hit = query_pipeline.cast_ray_and_get_normal(ray, max_toi, solid, filter=query_filter)
if hit is not None:
    handle, intersection = hit
    # This is similar to `QueryPipeline.cast_ray` illustrated above except
    # that it also returns the normal of the collider shape at the hit point.
    hit_point = ray.point_at(intersection.time_of_impact)
    hit_normal = intersection.normal
    print(f"Collider {handle} hit at point {hit_point} with normal {hit_normal}")


def on_ray_hit(handle, intersection):
    # Callback called on each collider hit by the ray.
    hit_point = ray.point_at(intersection.time_of_impact)
    hit_normal = intersection.normal
    print(f"Collider {handle} hit at point {hit_point} with normal {hit_normal}")
    return True  # Return `False` to stop the search.


query_pipeline.intersect_ray(ray, max_toi, solid, on_ray_hit, filter=query_filter)
# DOCUSAURUS: Raycast stop
assert query_pipeline.cast_ray(ray, max_toi, solid, filter=query_filter) is not None

# DOCUSAURUS: Shapecast start
shape = rp.SharedShape.cuboid(1.0, 2.0, 3.0)
shape_pos = rp.Isometry3(translation=(0.0, 1.0, 0.0), rotation=rp.rotation_from_angle((0.2, 0.7, 0.1)))
shape_vel = (0.1, 0.4, 0.2)
query_filter = rp.QueryFilter()
options = rp.ShapeCastOptions(
    max_time_of_impact=4.0,
    target_distance=0.0,
    stop_at_penetration=False,
    compute_impact_geometry_on_penetration=False,
)

query_pipeline = world.query_pipeline

hit = query_pipeline.cast_shape(shape_pos, shape_vel, shape, options, filter=query_filter)
if hit is not None:
    handle, hit = hit
    # The first collider hit has the handle `handle`. The `hit` is a
    # structure containing details about the hit configuration.
    print(f"Hit the collider {handle} with the configuration: {hit}")
# DOCUSAURUS: Shapecast stop

# DOCUSAURUS: ShapecastNonlinear start
# The shape rotates around its center (in its local-space) while it translates.
motion = rp.NonlinearRigidMotion(
    start=rp.Isometry3(translation=(5.0, 8.0, 0.0)),
    local_center=(0.0, 0.0, 0.0),
    linvel=(0.0, -4.0, 0.0),
    angvel=(0.0, 0.0, 3.0),
)
# Only `stop_at_penetration` is taken into account by the nonlinear shape-casting.
options = rp.ShapeCastOptions(stop_at_penetration=True)
start_time = 0.0
end_time = 2.0

hit = query_pipeline.cast_shape_nonlinear(motion, shape, options, start_time, end_time, filter=query_filter)
if hit is not None:
    handle, hit = hit
    # The pose of the cast shape at the time of impact gives the world-space
    # coordinates of its witness point.
    shape_pos_at_impact = motion.position_at_time(hit.time_of_impact)
    witness2 = shape_pos_at_impact.transform_point(hit.witness2)
    print(f"Hit the collider {handle} at time {hit.time_of_impact}, at point {witness2}")
# DOCUSAURUS: ShapecastNonlinear stop
assert hit is not None

# DOCUSAURUS: PointProjection start
point = (1.0, 2.0, 3.0)
solid = True
max_dist = 12.0
query_filter = rp.QueryFilter()

query_pipeline = world.query_pipeline

projection = query_pipeline.project_point(point, solid, filter=query_filter, max_dist=max_dist)
if projection is not None:
    handle, projection = projection
    # The collider closest to the point has this `handle`.
    print(f"Projected point on collider {handle}. Point projection: {projection.point}")
    print(f"Point was inside of the collider shape: {projection.is_inside}")

def on_point_intersection(handle):
    # Callback called on each collider with a shape containing the point.
    print(f"The collider {handle} contains the point.")
    return True  # Return `False` to stop the search.

query_pipeline.intersect_point(point, on_point_intersection, filter=query_filter)
# DOCUSAURUS: PointProjection stop

# DOCUSAURUS: IntersectionTest start
shape = rp.SharedShape.cuboid(1.0, 2.0, 3.0)
shape_pos = rp.Isometry3(translation=(0.0, 1.0, 0.0), rotation=rp.rotation_from_angle((0.2, 0.7, 0.1)))
query_filter = rp.QueryFilter()

query_pipeline = world.query_pipeline


def on_shape_intersection(handle):
    print(f"The collider {handle} intersects our shape.")
    return True  # Return `False` to stop the search.


query_pipeline.intersect_shape(shape_pos, shape, on_shape_intersection, filter=query_filter)

aabb = rp.Aabb(mins=(-1.0, -2.0, -3.0), maxs=(1.0, 2.0, 3.0))


def on_aabb_intersection(handle):
    print(f"The collider {handle} has an AABB intersecting our test AABB.")
    return True  # Return `False` to stop the search.


query_pipeline.intersect_aabb_conservative(aabb, on_aabb_intersection, filter=query_filter)
# DOCUSAURUS: IntersectionTest stop

# DOCUSAURUS: QueryFilter start
ray = rp.Ray(origin=(1.0, 2.0, 3.0), dir=(0.0, 1.0, 0.0))
max_toi = 4.0
solid = True
query_filter = (
    rp.QueryFilter.exclude_dynamic()
    .exclude_sensors()
    .exclude_rigid_body(player_handle)
    .groups(
        rp.InteractionGroups(
            memberships=rp.Group.GROUP_1 | rp.Group.GROUP_2,
            filter=rp.Group.GROUP_1,
            test_mode=rp.InteractionTestMode.AND,
        )
    )
    .predicate(lambda handle, collider: collider.user_data == 10)
)
query_pipeline = world.query_pipeline

hit = query_pipeline.cast_ray(ray, max_toi, solid, filter=query_filter)
if hit is not None:
    handle, toi = hit
    # Handle the hit.
# DOCUSAURUS: QueryFilter stop
    assert handle == tagged_handle
assert hit is not None
