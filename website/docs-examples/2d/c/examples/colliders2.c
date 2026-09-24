#include "snippets.h"

#include <assert.h>
#include <math.h>

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */

    R2Vector vertices[] = {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}};
    R2Triangle indices[] = {{0, 2, 1}};
    R2Real heights[] = {0.0, 1.0, 0.5, 0.9};
    R2Vector scale = r2Vector(1.0, 1.0);

    // DOCUSAURUS: Creation start
    // The world that will contain our colliders.
    R2World *world = r2NewWorld();

    // Description of a ball-shaped collider.
    R2ColliderDesc ball = r2BallColliderDesc(0.5);
    // Description of a cuboid-shaped collider.
    R2ColliderDesc cuboid = r2CuboidColliderDesc(r2Vector(0.5, 0.2));
    // Description of a capsule-shaped collider. The capsule principal axis is the `x` coordinate axis.
    R2ColliderDesc capsule_x = r2CapsuleXColliderDesc(0.5, 0.2);
    // Description of a capsule-shaped collider. The capsule principal axis is the `y` coordinate axis.
    R2ColliderDesc capsule_y = r2CapsuleYColliderDesc(0.5, 0.2);
    // Description of a triangle-mesh-shaped collider.
    R2ColliderDesc trimesh = r2DefaultColliderDesc();
    r2ShapeDesc_SetTrimesh(&trimesh.shape, (R2VectorView){vertices, 3}, (R2TriangleView){indices, 1}, 0);
    // Description of a heightfield-shaped collider.
    R2ColliderDesc heightfield = r2DefaultColliderDesc();
    heightfield.shape.kind = R2_SHAPE_DESC_HEIGHTFIELD;
    heightfield.shape.heights = (R2RealView){heights, 4};
    heightfield.shape.rows = 4;
    heightfield.shape.columns = 1;
    heightfield.shape.scale = scale;
    // Description of a collider with the given shared shape.
    R2SharedShape *shape = r2BallSharedShape(0.5);
    R2ColliderDesc collider = r2DefaultColliderDesc();
    collider.shape.kind = R2_SHAPE_DESC_SHARED;
    collider.shape.sharedShape = shape;
    // The collider translation wrt. the body it is attached to.
    // Default: the zero vector.
    collider.position.translation = r2Vector(1.0, 2.0);
    // The collider rotation wrt. the body it is attached to.
    // Default: the identity rotation.
    collider.position.rotation = r2Rotation(R2_PI);
    // The collider position wrt. the body it is attached to.
    // Default: the identity pose.
    collider.position = r2Pose(r2Vector(1.0, 2.0), r2Rotation(R2_PI));
    // The collider density. If non-zero the collider's mass and angular inertia will be added
    // to the inertial properties of the body it is attached to.
    // Default: 1.0
    collider.density = 1.3;
    // The friction coefficient of this collider.
    // Default: 0.5
    collider.friction = 0.8;
    // Whether this collider is a sensor.
    // Default: 0
    collider.isSensor = 1;

    // Insert the collider into the world, without attaching it to a rigid-body.
    R2ColliderHandle collider_handle = r2InsertColliderWithoutParent(world, &collider);

    R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
    R2RigidBodyHandle rigid_body_handle = r2InsertRigidBody(world, &rigid_body);
    // Or insert the collider into the world and attach it to a rigid-body.
    R2ColliderHandle handle = r2InsertCollider(rigid_body_handle, &collider);
    // The descriptions only borrow the shared shape: free it once it is no longer needed.
    r2FreeSharedShape(shape);
    // DOCUSAURUS: Creation stop

    r2InsertColliderWithoutParent(world, &ball);
    r2InsertColliderWithoutParent(world, &cuboid);
    r2InsertColliderWithoutParent(world, &capsule_x);
    r2InsertColliderWithoutParent(world, &capsule_y);
    r2InsertColliderWithoutParent(world, &trimesh);
    r2InsertColliderWithoutParent(world, &heightfield);
    (void)handle;

    {
        // DOCUSAURUS: ColliderType1 start
        /* Set the collider type when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.isSensor = 1;
        // DOCUSAURUS: ColliderType1 stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: ColliderType2 start
        /* Set the collider type after the collider creation. */
        r2Collider_SetSensor(collider_handle, 1);
        assert(r2Collider_IsSensor(collider_handle));
        // DOCUSAURUS: ColliderType2 stop
    }

    {
        // DOCUSAURUS: VoxelsPoints start
        // A voxels shape from arbitrary points.
        R2Vector points[] = {{0.0, 0.0}, {1.0, 1.0}, {-1.0, 1.0}};
        R2SharedShape *shape = r2VoxelsSharedShapeFromPoints(r2Vector(1.0, 1.0), (R2VectorView){points, 3});
        R2ColliderDesc collider = r2DefaultColliderDesc();
        collider.shape.kind = R2_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        // DOCUSAURUS: VoxelsPoints stop
        r2InsertColliderWithoutParent(world, &collider);
        r2FreeSharedShape(shape);
    }

    {
        R2Vector mesh[] = {{0.0, 0.0}, {0.0, 10.0}};
        R2Edge indices[] = {{0, 1}, {1, 0}};
        // DOCUSAURUS: VoxelsMesh start
        R2SharedShape *shape =
            r2VoxelizedMeshSharedShape((R2VectorView){mesh, 2}, (R2SurfaceElementView){indices, 2}, 0.2);
        // DOCUSAURUS: VoxelsMesh stop
        r2FreeSharedShape(shape);
    }

    {
        R2ShapeDesc shape = r2BallColliderDesc(0.5).shape;
        R2Pose pos1 = r2TranslationPose(r2Vector(0.0, 1.0));
        R2Pose pos2 = r2TranslationPose(r2Vector(0.0, 1.0));
        // DOCUSAURUS: Compound start
        R2CompoundShapeDesc parts[] = {{pos1, shape}, {pos2, shape}};
        R2ColliderDesc collider = r2DefaultColliderDesc();
        collider.shape.kind = R2_SHAPE_DESC_COMPOUND;
        collider.shape.children = (R2CompoundShapeView){parts, 2};
        // DOCUSAURUS: Compound stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: Mass start
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        R2RigidBodyHandle rigid_body_handle = r2InsertRigidBody(world, &rigid_body);
        // First option: by setting the density of the collider (or we could just leave
        //               its default value 1.0).
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1.0, 2.0));
        collider.density = 2.0;
        // Second option: by setting the mass of the collider.
        collider = r2CuboidColliderDesc(r2Vector(1.0, 2.0));
        collider.massMode = R2_MASS_TOTAL;
        collider.mass = 0.8;
        // Third option: by setting the mass-properties explicitly.
        collider = r2CuboidColliderDesc(r2Vector(1.0, 2.0));
        collider.massMode = R2_MASS_PROPERTIES;
        collider.massProperties = (R2MassProperties){
            .local_com = r2Vector(0.0, 1.0),
            .mass = 0.5,
            .principal_inertia = 0.3,
        };
        // When the collider is attached, the rigid-body's mass and angular
        // inertia is automatically updated to take the collider into account.
        r2InsertCollider(rigid_body_handle, &collider);
        // DOCUSAURUS: Mass stop
    }

    {
        // DOCUSAURUS: Position1 start
        /* Set the collider position when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.position.translation = r2Vector(1.0, 2.0);
        collider.position.rotation = r2Rotation(0.4);
        // Set both translation and rotation at once.
        collider.position = r2Pose(r2Vector(1.0, 2.0), r2Rotation(0.4));
        // DOCUSAURUS: Position1 stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: Position2 start
        /* Set the collider position after the collider creation. */
        r2Collider_SetTranslation(collider_handle, r2Vector(1.0, 2.0));
        r2Collider_SetRotation(collider_handle, r2Rotation(0.4));
        // Set both the translation and rotation at once.
        r2Collider_SetPosition(collider_handle, r2Pose(r2Vector(1.0, 2.0), r2Rotation(0.4)));
        R2Vector translation = r2Collider_Translation(collider_handle);
        assert(translation.x == 1.0 && translation.y == 2.0);
        assert(fabs(r2Collider_Rotation(collider_handle).angle - 0.4) < 1.0e-6);
        // DOCUSAURUS: Position2 stop
    }

    {
        // DOCUSAURUS: Position3 start
        R2RigidBodyDesc rigid_body = r2DynamicRigidBodyDesc();
        R2RigidBodyHandle rigid_body_handle = r2InsertRigidBody(world, &rigid_body);
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.position.translation = r2Vector(1.0, 2.0);
        // Attach the collider to the rigid-body. The description's position is
        // the collider's position wrt. the rigid-body.
        R2ColliderHandle collider_handle = r2InsertCollider(rigid_body_handle, &collider);
        // DOCUSAURUS: Position3 stop

        // DOCUSAURUS: Position4 start
        /* Set the collider position wrt. its parent after the collider creation. */
        r2Collider_SetPositionWrtParent(collider_handle, r2TranslationPose(r2Vector(1.0, 2.0)));
        R2Vector translation = r2Collider_PositionWrtParent(collider_handle).translation;
        assert(translation.x == 1.0 && translation.y == 2.0);
        // DOCUSAURUS: Position4 stop
    }

    {
        // DOCUSAURUS: Friction1 start
        /* Set the friction coefficient and friction combine rule
        when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.friction = 0.7;
        collider.frictionCombineRule = R2_COMBINE_MIN;
        // DOCUSAURUS: Friction1 stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: Friction2 start
        /* Set the friction coefficient and friction combine rule
        after the collider creation. */
        r2Collider_SetFriction(collider_handle, 0.7);
        r2Collider_SetFrictionCombineRule(collider_handle, R2_COMBINE_MIN);
        assert(r2Collider_Friction(collider_handle) == (R2Real)0.7);
        assert(r2Collider_FrictionCombineRule(collider_handle) == R2_COMBINE_MIN);
        // DOCUSAURUS: Friction2 stop
    }

    {
        // DOCUSAURUS: Restitution1 start
        /* Set the restitution coefficient and restitution combine rule
        when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.restitution = 0.7;
        collider.restitutionCombineRule = R2_COMBINE_MIN;
        // DOCUSAURUS: Restitution1 stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: Restitution2 start
        /* Set the restitution coefficient and restitution combine rule
        after the collider creation. */
        r2Collider_SetRestitution(collider_handle, 0.7);
        r2Collider_SetRestitutionCombineRule(collider_handle, R2_COMBINE_MIN);
        assert(r2Collider_Restitution(collider_handle) == (R2Real)0.7);
        assert(r2Collider_RestitutionCombineRule(collider_handle) == R2_COMBINE_MIN);
        // DOCUSAURUS: Restitution2 stop
    }

    {
        // DOCUSAURUS: Groups1 start
        /* Set the collision groups and solver groups when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.collisionGroups = (R2InteractionGroups){
            .memberships = (1u << 0) | (1u << 2) | (1u << 3), // Groups 0, 2, and 3.
            .filter = 1u << 2,                                // Group 2.
            .test_mode = R2_GROUPS_AND,
        };
        collider.solverGroups = (R2InteractionGroups){
            .memberships = (1u << 0) | (1u << 1),             // Groups 0 and 1.
            .filter = (1u << 0) | (1u << 1) | (1u << 3),      // Groups 0, 1, and 3.
            .test_mode = R2_GROUPS_AND,
        };
        // DOCUSAURUS: Groups1 stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: Groups2 start
        /* Set the collision groups and solver groups after the collider creation. */
        R2InteractionGroups collision_groups = {
            .memberships = (1u << 0) | (1u << 2) | (1u << 3), // Groups 0, 2, and 3.
            .filter = 1u << 2,                                // Group 2.
            .test_mode = R2_GROUPS_AND,
        };
        R2InteractionGroups solver_groups = {
            .memberships = (1u << 0) | (1u << 1),             // Groups 0 and 1.
            .filter = (1u << 0) | (1u << 1) | (1u << 3),      // Groups 0, 1, and 3.
            .test_mode = R2_GROUPS_AND,
        };
        r2Collider_SetCollisionGroups(collider_handle, collision_groups);
        r2Collider_SetSolverGroups(collider_handle, solver_groups);
        assert(r2Collider_CollisionGroups(collider_handle).memberships == collision_groups.memberships);
        assert(r2Collider_SolverGroups(collider_handle).filter == solver_groups.filter);
        // DOCUSAURUS: Groups2 stop
    }

    {
        // DOCUSAURUS: ActiveCollisionTypes1 start
        /* Set the active collision types when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.activeCollisionTypes = R2_COLLISION_TYPES_DEFAULT | R2_COLLISION_TYPES_KINEMATIC_FIXED;
        // DOCUSAURUS: ActiveCollisionTypes1 stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: ActiveCollisionTypes2 start
        /* Set the active collision types after the collider creation. */
        r2Collider_SetActiveCollisionTypes(collider_handle,
                                           R2_COLLISION_TYPES_DEFAULT | R2_COLLISION_TYPES_KINEMATIC_FIXED);
        assert(r2Collider_ActiveCollisionTypes(collider_handle) & R2_COLLISION_TYPES_DYNAMIC_KINEMATIC);
        assert(r2Collider_ActiveCollisionTypes(collider_handle) & R2_COLLISION_TYPES_KINEMATIC_FIXED);
        // DOCUSAURUS: ActiveCollisionTypes2 stop
    }

    {
        // DOCUSAURUS: ActiveEvents1 start
        /* Set the active events when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.activeEvents = R2_COLLISION_EVENTS;
        // DOCUSAURUS: ActiveEvents1 stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: ActiveEvents2 start
        /* Set the active events after the collider creation. */
        r2Collider_SetActiveEvents(collider_handle, R2_COLLISION_EVENTS);
        assert(r2Collider_ActiveEvents(collider_handle) & R2_COLLISION_EVENTS);
        // DOCUSAURUS: ActiveEvents2 stop
    }

    {
        // DOCUSAURUS: ActiveHooks1 start
        /* Set the active hooks when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.activeHooks = R2_FILTER_CONTACT_PAIRS | R2_MODIFY_SOLVER_CONTACTS;
        // DOCUSAURUS: ActiveHooks1 stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: ActiveHooks2 start
        /* Set the active hooks after the collider creation. */
        r2Collider_SetActiveHooks(collider_handle, R2_FILTER_CONTACT_PAIRS | R2_MODIFY_SOLVER_CONTACTS);
        assert(r2Collider_ActiveHooks(collider_handle) & R2_FILTER_CONTACT_PAIRS);
        assert(r2Collider_ActiveHooks(collider_handle) & R2_MODIFY_SOLVER_CONTACTS);
        // DOCUSAURUS: ActiveHooks2 stop
    }

    {
        // DOCUSAURUS: UserData1 start
        /* Set the user-data when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.userData = (R2UserData){.low = 42, .high = 0};
        // DOCUSAURUS: UserData1 stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: UserData2 start
        /* Set the user-data after the collider creation. */
        r2Collider_SetUserData(collider_handle, (R2UserData){.low = 42, .high = 0});
        assert(r2Collider_UserData(collider_handle).low == 42);
        // DOCUSAURUS: UserData2 stop
    }

    {
        // DOCUSAURUS: ContactSkin start
        /* Set the contact skin when the collider is created. */
        R2ColliderDesc collider = r2BallColliderDesc(0.5);
        collider.contactSkin = 0.01;
        /* Set the contact skin after the collider creation. */
        r2Collider_SetContactSkin(collider_handle, 0.01);
        assert(r2Collider_ContactSkin(collider_handle) == (R2Real)0.01);
        // DOCUSAURUS: ContactSkin stop
        r2InsertColliderWithoutParent(world, &collider);
    }

    r2Step(world, NULL, NULL);
    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
