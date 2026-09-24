/* Port of examples2d/debug_box_ball2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugBoxBall2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc floor = r2FixedRigidBodyDesc();
    floor.position.translation = r2Vector(0, -1);
    floor.position = r2Pose(r2Vector(0, -1), r2Rotation(R2_PI / 4));
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1, 1));
    floor.canSleep = !testbed->noSleep;
    R2RigidBodyHandle floorHandle = r2InsertRigidBody(world, &floor);
    r2InsertCollider(floorHandle, &collider);
    R2RigidBodyDesc ball = r2DynamicRigidBodyDesc();
    ball.position.translation = r2Vector(0, 3);
    ball.canSleep = 0;
    R2ColliderDesc ballCollider = r2BallColliderDesc(1);
    if (testbed->noSleep) {
        ball.canSleep = 0;
        ball.sleeping = 0;
    }
    floorHandle = r2InsertRigidBody(world, &ball);
    r2InsertCollider(floorHandle, &ballCollider);
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 0, 50);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
