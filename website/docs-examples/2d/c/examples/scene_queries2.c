#include "snippets.h"

static void raycast_section(const R2World *world);
static void shapecast_section(const R2World *world);
static void point_projection_section(const R2World *world);
static void intersection_section(const R2World *world);
static void query_filter_section(const R2World *world, R2RigidBodyHandle player_handle);

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R2World *world = r2NewWorld();

    /* Create the ground. */
    R2ColliderDesc ground = r2CuboidColliderDesc(r2Vector(100.0, 0.1));
    r2InsertColliderWithoutParent(world, &ground);

    /* Create the bouncing ball. */
    R2RigidBodyDesc ball_body = r2DynamicRigidBodyDesc();
    ball_body.position.translation = r2Vector(0.0, 10.0);
    R2RigidBodyHandle ball_body_handle = r2InsertRigidBody(world, &ball_body);
    R2ColliderDesc ball = r2BallColliderDesc(0.5);
    ball.restitution = 0.7;
    r2InsertCollider(ball_body_handle, &ball);

    R2ColliderDesc cuboid = r2CuboidColliderDesc(r2Vector(1.0, 1.0));
    cuboid.position.translation = r2Vector(0.0, 10.0);
    R2ColliderHandle handle1 = r2InsertColliderWithoutParent(world, &cuboid);
    cuboid.position.translation = r2Vector(0.0, 15.0);
    R2ColliderHandle handle_to_remove = r2InsertColliderWithoutParent(world, &cuboid);
    r2RemoveCollider(handle_to_remove, 1);
    r2Collider_SetTranslation(handle1, r2Vector(0.0, 12.0));

    /* Obstacles on the path of the ray and of the cast shape of the snippets below. */
    R2ColliderDesc obstacle = r2CuboidColliderDesc(r2Vector(1.0, 0.5));
    obstacle.position.translation = r2Vector(1.0, 4.0);
    r2InsertColliderWithoutParent(world, &obstacle);
    obstacle = r2BallColliderDesc(0.5);
    obstacle.position.translation = r2Vector(1.0, 5.5);
    r2InsertColliderWithoutParent(world, &obstacle);

    // DOCUSAURUS: QueryPipeline start
    // Game loop.
    for (int i = 0; i < 10; i++) {
        // Stepping the simulation updates the broad-phase the scene queries rely on.
        r2Step(world, NULL, NULL);

        // The scene queries take into account the positions of the colliders at the end of
        // the last timestep. Run the scene queries on `world` here.
    }
    // DOCUSAURUS: QueryPipeline stop

    raycast_section(world);
    shapecast_section(world);
    point_projection_section(world);
    intersection_section(world);

    R2RigidBodyHandle player_handle = ball_body_handle;
    query_filter_section(world, player_handle);

    r2FreeWorld(world);
    return EXIT_SUCCESS;
}

static void raycast_section(const R2World *world) {
    // DOCUSAURUS: Raycast start
    R2Vector ray_origin = r2Vector(1.0, 2.0);
    R2Vector ray_dir = r2Vector(0.0, 1.0);
    R2Real max_toi = 4.0;
    R2Bool solid = 1;
    R2QueryOptions options = r2DefaultQueryOptions();

    R2RayToi toi = r2CastRayToi(world, &options, ray_origin, ray_dir, max_toi, solid);
    if (toi.found) {
        // The first collider hit has the handle `toi.collider` and it hit after
        // the ray travelled a distance equal to `ray_dir * toi.toi`.
        R2Vector hit_point = r2VectorAdd(ray_origin, r2VectorScale(ray_dir, toi.toi));
        printf("Collider %u hit at point (%f, %f)\n", toi.collider.index, (double)hit_point.x,
               (double)hit_point.y);
    }

    R2OptionalRayHit result = r2TryCastRay(world, &options, ray_origin, ray_dir, max_toi, solid);
    if (result.found) {
        R2RayHit hit = result.hit;
        // This is similar to `r2CastRayToi` illustrated above except
        // that it also returns the normal of the collider shape at the hit point.
        R2Vector hit_point = r2VectorAdd(ray_origin, r2VectorScale(ray_dir, hit.time_of_impact));
        R2Vector hit_normal = hit.normal;
        printf("Collider %u hit at point (%f, %f) with normal (%f, %f)\n",
               hit.collider.index, (double)hit_point.x, (double)hit_point.y,
               (double)hit_normal.x, (double)hit_normal.y);
    }
    // DOCUSAURUS: Raycast stop

    // DOCUSAURUS: RaycastAll start
    // Get the number of colliders hit by the ray, then copy all their hits.
    size_t count = r2IntersectRay(world, &options, ray_origin, ray_dir, max_toi, solid, NULL, 0);
    R2RayHit *hits = malloc(count * sizeof(*hits));
    count = r2IntersectRay(world, &options, ray_origin, ray_dir, max_toi, solid, hits, count);

    for (size_t i = 0; i < count; i++) {
        // Loop on each collider hit by the ray.
        R2Vector hit_point = r2VectorAdd(ray_origin, r2VectorScale(ray_dir, hits[i].time_of_impact));
        R2Vector hit_normal = hits[i].normal;
        printf("Collider %u hit at point (%f, %f) with normal (%f, %f)\n",
               hits[i].collider.index, (double)hit_point.x, (double)hit_point.y,
               (double)hit_normal.x, (double)hit_normal.y);
    }
    free(hits);
    // DOCUSAURUS: RaycastAll stop
}

static void shapecast_section(const R2World *world) {
    // DOCUSAURUS: Shapecast start
    R2SharedShape *shape = r2CuboidSharedShape(r2Vector(1.0, 2.0));
    R2Pose shape_pos = r2Pose(r2Vector(0.0, 1.0), r2Rotation(0.2));
    R2Vector shape_vel = r2Vector(0.1, 0.4);
    R2QueryOptions options = r2DefaultQueryOptions();
    R2ShapeCastOptions cast_options = r2DefaultShapeCastOptions();
    cast_options.max_time_of_impact = 4.0;
    cast_options.target_distance = 0.0;
    cast_options.stop_at_penetration = 0;
    cast_options.compute_impact_geometry_on_penetration = 0;

    R2OptionalShapeCastHit result = r2TryCastShape(world, &options, shape_pos, shape_vel, shape, cast_options);
    if (result.found) {
        R2ShapeCastHit hit = result.hit;
        // The first collider hit has the handle `hit.collider`. The `hit` is a
        // structure containing details about the hit configuration.
        printf("Hit the collider %u with the time of impact %f\n", hit.collider.index,
               (double)hit.time_of_impact);
    }

    // The shape is owned by the application.
    r2FreeSharedShape(shape);
    // DOCUSAURUS: Shapecast stop
}

static void point_projection_section(const R2World *world) {
    // DOCUSAURUS: PointProjection start
    R2Vector point = r2Vector(1.0, 2.0);
    R2Bool solid = 1;
    R2Real max_dist = 12.0;
    R2QueryOptions options = r2DefaultQueryOptions();

    R2OptionalPointProjection result = r2TryProjectPoint(world, &options, point, max_dist, solid);
    if (result.found) {
        R2PointProjection projection = result.projection;
        // The collider closest to the point has the handle `projection.collider`.
        printf("Projected point on collider %u. Point projection: (%f, %f)\n", projection.collider.index,
               (double)projection.point.x, (double)projection.point.y);
        printf("Point was inside of the collider shape: %u\n", projection.is_inside);
    }

    // Get the number of colliders containing the point, then copy their handles.
    size_t count = r2IntersectPoint(world, &options, point, NULL, 0);
    R2ColliderHandle *handles = malloc(count * sizeof(*handles));
    count = r2IntersectPoint(world, &options, point, handles, count);
    for (size_t i = 0; i < count; i++) {
        // Loop on each collider with a shape containing the point.
        printf("The collider %u contains the point.\n", handles[i].index);
    }
    free(handles);
    // DOCUSAURUS: PointProjection stop
}

static void intersection_section(const R2World *world) {
    // DOCUSAURUS: IntersectionTest start
    R2SharedShape *shape = r2CuboidSharedShape(r2Vector(1.0, 2.0));
    R2Pose shape_pos = r2Pose(r2Vector(0.0, 1.0), r2Rotation(0.2));
    R2QueryOptions options = r2DefaultQueryOptions();

    // Get the number of colliders intersecting the shape, then copy their handles.
    size_t count = r2IntersectShape(world, &options, shape_pos, shape, NULL, 0);
    R2ColliderHandle *handles = malloc(count * sizeof(*handles));
    count = r2IntersectShape(world, &options, shape_pos, shape, handles, count);
    for (size_t i = 0; i < count; i++) {
        printf("The collider %u intersects our shape.\n", handles[i].index);
    }
    free(handles);
    r2FreeSharedShape(shape);

    R2Aabb aabb = {r2Vector(-1.0, -2.0), r2Vector(1.0, 2.0)};
    count = r2IntersectAabbConservative(world, &options, aabb, NULL, 0);
    handles = malloc(count * sizeof(*handles));
    count = r2IntersectAabbConservative(world, &options, aabb, handles, count);
    for (size_t i = 0; i < count; i++) {
        printf("The collider %u has an AABB intersecting our test AABB.\n", handles[i].index);
    }
    free(handles);
    // DOCUSAURUS: IntersectionTest stop
}

// DOCUSAURUS: QueryFilter start
// The predicate is called for each collider that passed the other filtering rules.
// Returning 0 excludes the collider from the scene query.
static R2Bool RAPIER_CALL user_data_predicate(void *user_data, const R2ReadContext *read,
                                              R2ColliderHandle handle) {
    (void)user_data;
    return r2ReadCollider_UserData(read, handle).low == 10;
}

static void query_filter_section(const R2World *world, R2RigidBodyHandle player_handle) {
    R2Vector ray_origin = r2Vector(1.0, 2.0);
    R2Vector ray_dir = r2Vector(0.0, 1.0);
    R2Real max_toi = 4.0;
    R2Bool solid = 1;
    R2QueryOptions options = r2DefaultQueryOptions();
    options.filter.flags = R2_QUERY_EXCLUDE_DYNAMIC | R2_QUERY_EXCLUDE_SENSORS;
    options.filter.exclude_rigid_body = player_handle;
    options.filter.use_groups = 1;
    options.filter.groups.memberships = 0x0001 | 0x0002; // Groups 1 and 2.
    options.filter.groups.filter = 0x0001;               // Group 1.
    options.filter.groups.test_mode = R2_GROUPS_AND;
    options.predicate = user_data_predicate;
    options.userData = NULL; // Given to the predicate as its first argument.

    R2RayToi toi = r2CastRayToi(world, &options, ray_origin, ray_dir, max_toi, solid);
    if (toi.found) {
        // Handle the hit.
    }
}
// DOCUSAURUS: QueryFilter stop
