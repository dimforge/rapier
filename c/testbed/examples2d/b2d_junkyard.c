/* Port of examples2d/b2d_junkyard.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB2dJunkyard(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, -10));
    R2RigidBodyHandle ground;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 0);
    rigidBody.canSleep = !testbed->noSleep;
    ground = r2InsertRigidBody(world, &rigidBody);

    for (int i = 0; i <= 160; i++) {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.55, 0.5));
        collider.position.translation = r2Vector(-80 + i, 0);
        r2InsertCollider(ground, &collider);
    }
    for (int side = -1; side <= 1; side += 2) {
        for (int i = 0; i < 50; i++) {
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.55));
            collider.position.translation = r2Vector(side * 80, i + 1);
            r2InsertCollider(ground, &collider);
        }
    }
    R2Vector points[5];
    R2Real phi = R2_PI * ((R2Real)sqrt(5) - 1);
    for (int i = 0; i < 5; i++) {
        points[i] = r2Vector(0.25 * cos(phi * i), 0.25 * sin(phi * i));
    }
    R2Real side = -0.1;
    for (int i = 0; i < 200; i++) {
        for (int j = 0; j < 40; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(1.5 * (2 * i - 200) * 0.25 + side, j + 15);
            R2ColliderDesc collider = r2DefaultColliderDesc();
            r2ShapeDesc_SetConvexHull(&collider.shape, (R2VectorView){points, 5});
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);

            side = -side;
        }
    }
    R2RigidBodyHandle pusher = {0};
    R2RigidBodyDesc platformBody = r2KinematicPositionBasedRigidBodyDesc();
    platformBody.position.translation = r2Vector(0, 0);
    platformBody.canSleep = !testbed->noSleep;
    pusher = r2InsertRigidBody(world, &platformBody);

    R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(2, 4));
    boxCollider.position.translation = r2Vector(0, 4);
    r2InsertCollider(pusher, &boxCollider);
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 20, 4);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2RigidBody_SetNextKinematicTranslation(
                pusher, r2Vector(60 * sin(0.2 * testbed->step / 60.0), 0));

            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
