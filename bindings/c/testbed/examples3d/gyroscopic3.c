/* Port of examples3d/gyroscopic3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbGyroscopic3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3Pose poses[] = {r3TranslationPose(r3Vector(0, 0, 0)), r3TranslationPose(r3Vector(0, 0.8, 0))};
    R3SharedShape *shapes[2] = {NULL};
    shapes[0] = r3CuboidSharedShape(r3Vector(2.0, 0.2, 0.2));
    shapes[1] = r3CuboidSharedShape(r3Vector(0.2, 0.4, 0.2));
    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 0, 0);
    rigidBody.gravityScale = 0;
    rigidBody.angvel = r3Vector(0, 20, 0.1);
    rigidBody.gyroscopicForcesEnabled = 1;
    R3ColliderDesc collider;
    R3CompoundShapeDesc colliderParts[2];
    for (size_t part = 0; part < 2; ++part) {
        colliderParts[part].pose = poses[part];
        colliderParts[part].shape = r3DefaultShapeDesc();
        colliderParts[part].shape.kind = R3_SHAPE_DESC_SHARED;
        colliderParts[part].shape.sharedShape = ((const R3SharedShape *const *)shapes)[part];
    }
    collider = r3DefaultColliderDesc();
    collider.shape.kind = R3_SHAPE_DESC_COMPOUND;
    collider.shape.children = (R3CompoundShapeView){colliderParts, 2};

    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    for (size_t part = 0; part < 2; part++) {
        r3FreeSharedShape(shapes[part]);
    }
    /* Set up the viewer. */
    tbCamera(testbed, 8, 0, 8, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
