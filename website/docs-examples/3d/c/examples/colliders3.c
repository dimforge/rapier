#include "snippets.h"

#include <assert.h>

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */

    R3Vector vertices[] = {{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}};
    R3Triangle indices[] = {{0, 2, 1}};
    R3Real heights[] = {0.0, 1.0, 0.5, 0.0};
    R3Vector scale = r3Vector(1.0, 1.0, 1.0);

    // DOCUSAURUS: Creation start
    // The world that will contain our colliders.
    R3World *world = r3NewWorld();

    // Description of a ball-shaped collider.
    R3ColliderDesc ball = r3BallColliderDesc(0.5);
    // Description of a cuboid-shaped collider.
    R3ColliderDesc cuboid = r3CuboidColliderDesc(r3Vector(0.5, 0.2, 0.1));
    // Description of a capsule-shaped collider. The capsule principal axis is the `x` coordinate axis.
    R3ColliderDesc capsule_x = r3CapsuleXColliderDesc(0.5, 0.2);
    // Description of a capsule-shaped collider. The capsule principal axis is the `y` coordinate axis.
    R3ColliderDesc capsule_y = r3CapsuleYColliderDesc(0.5, 0.2);
    // Description of a capsule-shaped collider. The capsule principal axis is the `z` coordinate axis.
    R3ColliderDesc capsule_z = r3CapsuleZColliderDesc(0.5, 0.2);
    // Description of a triangle-mesh-shaped collider.
    R3ColliderDesc trimesh = r3DefaultColliderDesc();
    r3ShapeDesc_SetTrimesh(&trimesh.shape, (R3VectorView){vertices, 3}, (R3TriangleView){indices, 1}, 0);
    // Description of a heightfield-shaped collider (heights in column-major order).
    R3ColliderDesc heightfield = r3DefaultColliderDesc();
    heightfield.shape.kind = R3_SHAPE_DESC_HEIGHTFIELD;
    heightfield.shape.heights = (R3RealView){heights, 4};
    heightfield.shape.rows = 2;
    heightfield.shape.columns = 2;
    heightfield.shape.scale = scale;
    // Description of a collider with the given shared shape.
    R3SharedShape *shape = r3BallSharedShape(0.5);
    R3ColliderDesc collider = r3DefaultColliderDesc();
    collider.shape.kind = R3_SHAPE_DESC_SHARED;
    collider.shape.sharedShape = shape;
    // The collider translation wrt. the body it is attached to.
    // Default: the zero vector.
    collider.position.translation = r3Vector(1.0, 2.0, 3.0);
    // The collider rotation wrt. the body it is attached to.
    // Default: the identity rotation.
    collider.position.rotation = r3RotationFromAxisAngle(r3Vector(0.0, 1.0, 0.0), R3_PI);
    // The collider position wrt. the body it is attached to.
    // Default: the identity pose.
    collider.position = r3Pose(r3Vector(1.0, 2.0, 3.0), r3RotationFromAxisAngle(r3Vector(0.0, 1.0, 0.0), R3_PI));
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
    R3ColliderHandle collider_handle = r3InsertColliderWithoutParent(world, &collider);

    R3RigidBodyDesc rigid_body = r3DynamicRigidBodyDesc();
    R3RigidBodyHandle rigid_body_handle = r3InsertRigidBody(world, &rigid_body);
    // Or insert the collider into the world and attach it to a rigid-body.
    R3ColliderHandle handle = r3InsertCollider(rigid_body_handle, &collider);
    // The descriptions only borrow the shared shape: free it once it is no longer needed.
    r3FreeSharedShape(shape);
    // DOCUSAURUS: Creation stop

    r3InsertColliderWithoutParent(world, &ball);
    r3InsertColliderWithoutParent(world, &cuboid);
    r3InsertColliderWithoutParent(world, &capsule_x);
    r3InsertColliderWithoutParent(world, &capsule_y);
    r3InsertColliderWithoutParent(world, &capsule_z);
    r3InsertColliderWithoutParent(world, &trimesh);
    r3InsertColliderWithoutParent(world, &heightfield);
    (void)handle;

    {
        // DOCUSAURUS: VoxelsPoints start
        // A voxels shape from arbitrary points.
        R3Vector points[] = {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
        R3SharedShape *shape = r3VoxelsSharedShapeFromPoints(r3Vector(1.0, 1.0, 1.0), (R3VectorView){points, 2});
        R3ColliderDesc collider = r3DefaultColliderDesc();
        collider.shape.kind = R3_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        // DOCUSAURUS: VoxelsPoints stop
        r3InsertColliderWithoutParent(world, &collider);
        r3FreeSharedShape(shape);
    }

    {
        // DOCUSAURUS: Mass start
        R3RigidBodyDesc rigid_body = r3DynamicRigidBodyDesc();
        R3RigidBodyHandle rigid_body_handle = r3InsertRigidBody(world, &rigid_body);
        // First option: by setting the density of the collider (or we could just leave
        //               its default value 1.0).
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1.0, 2.0, 3.0));
        collider.density = 2.0;
        // Second option: by setting the mass of the collider.
        collider = r3CuboidColliderDesc(r3Vector(1.0, 2.0, 3.0));
        collider.massMode = R3_MASS_TOTAL;
        collider.mass = 0.8;
        // Third option: by setting the mass-properties explicitly.
        collider = r3CuboidColliderDesc(r3Vector(1.0, 2.0, 3.0));
        collider.massMode = R3_MASS_PROPERTIES;
        collider.massProperties = (R3MassProperties){
            .local_com = r3Vector(0.0, 1.0, 0.0),
            .mass = 0.5,
            .principal_inertia = r3Vector(0.3, 0.2, 0.1),
            // The identity rotation: the principal inertia axes are the local axes.
            .principal_inertia_local_frame = {0.0, 0.0, 0.0, 1.0},
        };
        // When the collider is attached, the rigid-body's mass and angular
        // inertia is automatically updated to take the collider into account.
        R3ColliderHandle collider_handle = r3InsertCollider(rigid_body_handle, &collider);
        // DOCUSAURUS: Mass stop
        assert(r3Collider_Mass(collider_handle) == (R3Real)0.5);
    }

    {
        // DOCUSAURUS: Position1 start
        /* Set the collider position when the collider is created. */
        R3ColliderDesc collider = r3BallColliderDesc(0.5);
        collider.position.translation = r3Vector(1.0, 2.0, 3.0);
        collider.position.rotation = r3RotationFromAxisAngle(r3Vector(0.0, 0.0, 1.0), 0.4);
        // Set both translation and rotation at once.
        collider.position =
            r3Pose(r3Vector(1.0, 2.0, 3.0), r3RotationFromAxisAngle(r3Vector(0.0, 0.0, 1.0), 0.4));
        // DOCUSAURUS: Position1 stop
        r3InsertColliderWithoutParent(world, &collider);
    }

    {
        // DOCUSAURUS: Position2 start
        /* Set the collider position after the collider creation. */
        R3Rotation rotation = r3RotationFromAxisAngle(r3Vector(0.0, 0.0, 1.0), 0.4);
        r3Collider_SetTranslation(collider_handle, r3Vector(1.0, 2.0, 3.0));
        r3Collider_SetRotation(collider_handle, rotation);
        // Set both the translation and rotation at once.
        r3Collider_SetPosition(collider_handle, r3Pose(r3Vector(1.0, 2.0, 3.0), rotation));
        R3Vector translation = r3Collider_Translation(collider_handle);
        assert(translation.x == 1.0 && translation.y == 2.0 && translation.z == 3.0);
        // DOCUSAURUS: Position2 stop
    }

    {
        // DOCUSAURUS: Position3 start
        R3RigidBodyDesc rigid_body = r3DynamicRigidBodyDesc();
        R3RigidBodyHandle rigid_body_handle = r3InsertRigidBody(world, &rigid_body);
        R3ColliderDesc collider = r3BallColliderDesc(0.5);
        collider.position.translation = r3Vector(1.0, 2.0, 3.0);
        // Attach the collider to the rigid-body. The description's position is
        // the collider's position wrt. the rigid-body.
        R3ColliderHandle collider_handle = r3InsertCollider(rigid_body_handle, &collider);
        // DOCUSAURUS: Position3 stop

        // DOCUSAURUS: Position4 start
        /* Set the collider position wrt. its parent after the collider creation. */
        r3Collider_SetPositionWrtParent(collider_handle, r3TranslationPose(r3Vector(1.0, 2.0, 3.0)));
        R3Vector translation = r3Collider_PositionWrtParent(collider_handle).translation;
        assert(translation.x == 1.0 && translation.y == 2.0 && translation.z == 3.0);
        // DOCUSAURUS: Position4 stop
    }

    {
        // DOCUSAURUS: ContactSkin start
        /* Set the contact skin when the collider is created. */
        R3ColliderDesc collider = r3BallColliderDesc(0.5);
        collider.contactSkin = 0.01;
        /* Set the contact skin after the collider creation. */
        r3Collider_SetContactSkin(collider_handle, 0.01);
        assert(r3Collider_ContactSkin(collider_handle) == (R3Real)0.01);
        // DOCUSAURUS: ContactSkin stop
        r3InsertColliderWithoutParent(world, &collider);
    }

    r3Step(world, NULL, NULL);
    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
