/* Port of examples3d/debug_shape_modification3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugShapeModification3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(20, 0.1, 20));
        collider.friction = .15;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    R3RigidBodyHandle ballHandle;
    R3ColliderHandle ballCollHandle;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 0.2, 0);
        rigidBody.linvel = r3Vector(10, 0, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3BallColliderDesc(.1);
        collider.density = 100;
        ballHandle = r3InsertRigidBody(world, &rigidBody);
        ballCollHandle = r3InsertCollider(ballHandle, &collider);
    }
    {
        R3ColliderDesc staticCollider = r3BallColliderDesc(3);
        staticCollider.position.translation = r3Vector(-15, 3, 18);
        r3InsertColliderWithoutParent(world, &staticCollider);
    }
    R3SharedShape *shapes[4] = {0};
    shapes[0] = r3BallSharedShape(3);
    shapes[1] = r3CuboidSharedShape(r3Vector(3, 3, 3));
    shapes[2] = r3ConeSharedShape(3, 3);
    shapes[3] = r3CylinderSharedShape(3, 3);
    R3ColliderDesc shapeshiftingCollider = r3DefaultColliderDesc();
    shapeshiftingCollider.shape.kind = R3_SHAPE_DESC_SHARED;
    shapeshiftingCollider.shape.sharedShape = shapes[0];
    shapeshiftingCollider.position.translation = r3Vector(-15, 3, 9);
    R3ColliderHandle shapeshiftingCollHandle =
        r3InsertColliderWithoutParent(world, &shapeshiftingCollider);

    R3Real offset = -12;
    for (int j = 0; j < 20; ++j) {
        for (int i = 0; i < 8; ++i) {
            for (int k = 0; k < 8; ++k) {
                R3ColliderDesc collider;
                switch (j % 5) {
                case 0:
                    collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
                    break;
                case 1:
                    collider = r3BallColliderDesc(1);
                    break;
                case 2:
                    collider = r3RoundCylinderColliderDesc(1, 1, .1);
                    break;
                case 3:
                    collider = r3ConeColliderDesc(1, 1);
                    break;
                default:
                    collider = r3CapsuleYColliderDesc(1, 1);
                    break;
                }
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(i * 3 - 12 + offset + 5, j * 3 + 4.5, k * 3 - 12 + offset);
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
        offset -= .35;
    }
    tbCamera(testbed, 40, 40, 40, 0, 0, 0);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    size_t shapeIdx = 0, step = 0;
    const size_t snappedFrame = 51;
    R3Vector linvel = {0}, angvel = {0};
    R3Pose pos = r3TranslationPose(r3Vector(0, 0, 0));
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
            ++step;

            if (step == snappedFrame) {
                linvel = r3RigidBody_Linvel(ballHandle);
                angvel = r3RigidBody_Angvel(ballHandle);
                pos = r3RigidBody_Position(ballHandle);
            }

            if (step % 50 == 0) {
                shapeIdx = (shapeIdx + 1) % 4;
                r3Collider_SetShape(shapeshiftingCollHandle, shapes[shapeIdx]);
            }
            if (step == 100) {
                r3RigidBody_SetLinvel(ballHandle, linvel, 1);
                r3RigidBody_SetAngvel(ballHandle, angvel, 1);
                r3RigidBody_SetPosition(ballHandle, pos, 1);
                step = snappedFrame;
            }

            R3SharedShape *shape = r3BallSharedShape(.1 * step * 2);
            r3Collider_SetShape(ballCollHandle, shape);
            r3FreeSharedShape(shape);
        }
    }
    for (size_t i = 0; i < TB_COUNT(shapes); ++i) {
        r3FreeSharedShape(shapes[i]);
    }
    r3FreeWorld(world);
}
