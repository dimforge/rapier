#include "snippets.h"

static void raycast_section(const R3World *world);
static void shapecast_section(const R3World *world);
static void point_projection_section(const R3World *world);
static void intersection_section(const R3World *world);

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R3World *world = r3NewWorld();

    /* Create the ground. */
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(100.0, 0.1, 100.0));
    r3InsertColliderWithoutParent(world, &ground);

    /* Create the bouncing ball. */
    R3RigidBodyDesc ball_body = r3DynamicRigidBodyDesc();
    ball_body.position.translation = r3Vector(0.0, 10.0, 0.0);
    R3RigidBodyHandle ball_body_handle = r3InsertRigidBody(world, &ball_body);
    R3ColliderDesc ball = r3BallColliderDesc(0.5);
    ball.restitution = 0.7;
    r3InsertCollider(ball_body_handle, &ball);

    R3ColliderDesc cuboid = r3CuboidColliderDesc(r3Vector(1.0, 1.0, 1.0));
    cuboid.position.translation = r3Vector(0.0, 10.0, 0.0);
    R3ColliderHandle handle1 = r3InsertColliderWithoutParent(world, &cuboid);
    cuboid.position.translation = r3Vector(0.0, 15.0, 0.0);
    R3ColliderHandle handle_to_remove = r3InsertColliderWithoutParent(world, &cuboid);
    r3RemoveCollider(handle_to_remove, 1);
    r3Collider_SetTranslation(handle1, r3Vector(0.0, 12.0, 0.0));

    /* Obstacles on the path of the ray and of the cast shape of the snippets below. */
    R3ColliderDesc obstacle = r3CuboidColliderDesc(r3Vector(1.0, 0.5, 1.0));
    obstacle.position.translation = r3Vector(1.0, 4.0, 3.0);
    r3InsertColliderWithoutParent(world, &obstacle);
    obstacle = r3BallColliderDesc(0.5);
    obstacle.position.translation = r3Vector(1.0, 5.5, 3.0);
    r3InsertColliderWithoutParent(world, &obstacle);

    // DOCUSAURUS: QueryPipeline start
    // Game loop.
    for (int i = 0; i < 10; i++) {
        // Stepping the simulation updates the broad-phase the scene queries rely on.
        r3Step(world, NULL, NULL);

        // The scene queries take into account the positions of the colliders at the end of
        // the last timestep. Run the scene queries on `world` here.
    }
    // DOCUSAURUS: QueryPipeline stop

    raycast_section(world);
    shapecast_section(world);
    point_projection_section(world);
    intersection_section(world);

    r3FreeWorld(world);
    return EXIT_SUCCESS;
}

static void raycast_section(const R3World *world) {
    // DOCUSAURUS: Raycast start
    R3Vector ray_origin = r3Vector(1.0, 2.0, 3.0);
    R3Vector ray_dir = r3Vector(0.0, 1.0, 0.0);
    R3Real max_toi = 4.0;
    R3Bool solid = 1;
    R3QueryOptions options = r3DefaultQueryOptions();

    R3RayToi toi = r3CastRayToi(world, &options, ray_origin, ray_dir, max_toi, solid);
    if (toi.found) {
        // The first collider hit has the handle `toi.collider` and it hit after
        // the ray travelled a distance equal to `ray_dir * toi.toi`.
        R3Vector hit_point = r3VectorAdd(ray_origin, r3VectorScale(ray_dir, toi.toi));
        printf("Collider %u hit at point (%f, %f, %f)\n", toi.collider.index, (double)hit_point.x,
               (double)hit_point.y, (double)hit_point.z);
    }

    R3OptionalRayHit result = r3TryCastRay(world, &options, ray_origin, ray_dir, max_toi, solid);
    if (result.found) {
        R3RayHit hit = result.hit;
        // This is similar to `r3CastRayToi` illustrated above except
        // that it also returns the normal of the collider shape at the hit point.
        R3Vector hit_point = r3VectorAdd(ray_origin, r3VectorScale(ray_dir, hit.time_of_impact));
        R3Vector hit_normal = hit.normal;
        printf("Collider %u hit at point (%f, %f, %f) with normal (%f, %f, %f)\n",
               hit.collider.index, (double)hit_point.x, (double)hit_point.y,
               (double)hit_point.z, (double)hit_normal.x, (double)hit_normal.y,
               (double)hit_normal.z);
    }
    // DOCUSAURUS: Raycast stop

    // DOCUSAURUS: RaycastAll start
    // Get the number of colliders hit by the ray, then copy all their hits.
    size_t count = r3IntersectRay(world, &options, ray_origin, ray_dir, max_toi, solid, NULL, 0);
    R3RayHit *hits = malloc(count * sizeof(*hits));
    count = r3IntersectRay(world, &options, ray_origin, ray_dir, max_toi, solid, hits, count);

    for (size_t i = 0; i < count; i++) {
        // Loop on each collider hit by the ray.
        R3Vector hit_point = r3VectorAdd(ray_origin, r3VectorScale(ray_dir, hits[i].time_of_impact));
        R3Vector hit_normal = hits[i].normal;
        printf("Collider %u hit at point (%f, %f, %f) with normal (%f, %f, %f)\n",
               hits[i].collider.index, (double)hit_point.x, (double)hit_point.y,
               (double)hit_point.z, (double)hit_normal.x, (double)hit_normal.y,
               (double)hit_normal.z);
    }
    free(hits);
    // DOCUSAURUS: RaycastAll stop
}

static void shapecast_section(const R3World *world) {
    // DOCUSAURUS: Shapecast start
    R3SharedShape *shape = r3CuboidSharedShape(r3Vector(1.0, 2.0, 3.0));
    // The rotation is given as a scaled axis, i.e., an axis multiplied by the angle.
    R3Vector scaled_axis = r3Vector(0.2, 0.7, 0.1);
    R3Pose shape_pos = r3Pose(r3Vector(0.0, 1.0, 0.0),
                              r3RotationFromAxisAngle(scaled_axis, r3VectorLength(scaled_axis)));
    R3Vector shape_vel = r3Vector(0.1, 0.4, 0.2);
    R3QueryOptions options = r3DefaultQueryOptions();
    R3ShapeCastOptions cast_options = r3DefaultShapeCastOptions();
    cast_options.max_time_of_impact = 4.0;
    cast_options.target_distance = 0.0;
    cast_options.stop_at_penetration = 0;
    cast_options.compute_impact_geometry_on_penetration = 0;

    R3OptionalShapeCastHit result = r3TryCastShape(world, &options, shape_pos, shape_vel, shape, cast_options);
    if (result.found) {
        R3ShapeCastHit hit = result.hit;
        // The first collider hit has the handle `hit.collider`. The `hit` is a
        // structure containing details about the hit configuration.
        printf("Hit the collider %u with the time of impact %f\n", hit.collider.index,
               (double)hit.time_of_impact);
    }

    // The shape is owned by the application.
    r3FreeSharedShape(shape);
    // DOCUSAURUS: Shapecast stop
}

static void point_projection_section(const R3World *world) {
    // DOCUSAURUS: PointProjection start
    R3Vector point = r3Vector(1.0, 2.0, 3.0);
    R3Bool solid = 1;
    R3Real max_dist = 12.0;
    R3QueryOptions options = r3DefaultQueryOptions();

    R3OptionalPointProjection result = r3TryProjectPoint(world, &options, point, max_dist, solid);
    if (result.found) {
        R3PointProjection projection = result.projection;
        // The collider closest to the point has the handle `projection.collider`.
        printf("Projected point on collider %u. Point projection: (%f, %f, %f)\n", projection.collider.index,
               (double)projection.point.x, (double)projection.point.y,
               (double)projection.point.z);
        printf("Point was inside of the collider shape: %u\n", projection.is_inside);
    }

    // Get the number of colliders containing the point, then copy their handles.
    size_t count = r3IntersectPoint(world, &options, point, NULL, 0);
    R3ColliderHandle *handles = malloc(count * sizeof(*handles));
    count = r3IntersectPoint(world, &options, point, handles, count);
    for (size_t i = 0; i < count; i++) {
        // Loop on each collider with a shape containing the point.
        printf("The collider %u contains the point.\n", handles[i].index);
    }
    free(handles);
    // DOCUSAURUS: PointProjection stop
}

static void intersection_section(const R3World *world) {
    // DOCUSAURUS: IntersectionTest start
    R3SharedShape *shape = r3CuboidSharedShape(r3Vector(1.0, 2.0, 3.0));
    // The rotation is given as a scaled axis, i.e., an axis multiplied by the angle.
    R3Vector scaled_axis = r3Vector(0.2, 0.7, 0.1);
    R3Pose shape_pos = r3Pose(r3Vector(0.0, 1.0, 0.0),
                              r3RotationFromAxisAngle(scaled_axis, r3VectorLength(scaled_axis)));
    R3QueryOptions options = r3DefaultQueryOptions();

    // Get the number of colliders intersecting the shape, then copy their handles.
    size_t count = r3IntersectShape(world, &options, shape_pos, shape, NULL, 0);
    R3ColliderHandle *handles = malloc(count * sizeof(*handles));
    count = r3IntersectShape(world, &options, shape_pos, shape, handles, count);
    for (size_t i = 0; i < count; i++) {
        printf("The collider %u intersects our shape.\n", handles[i].index);
    }
    free(handles);
    r3FreeSharedShape(shape);

    R3Aabb aabb = {r3Vector(-1.0, -2.0, -3.0), r3Vector(1.0, 2.0, 3.0)};
    count = r3IntersectAabbConservative(world, &options, aabb, NULL, 0);
    handles = malloc(count * sizeof(*handles));
    count = r3IntersectAabbConservative(world, &options, aabb, handles, count);
    for (size_t i = 0; i < count; i++) {
        printf("The collider %u has an AABB intersecting our test AABB.\n", handles[i].index);
    }
    free(handles);
    // DOCUSAURUS: IntersectionTest stop
}
