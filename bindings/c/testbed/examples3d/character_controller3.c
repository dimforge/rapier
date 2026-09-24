/* Port of examples3d/character_controller3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include "utils/character.h"

void tbCharacterController3(Testbed *testbed) {
    R3World *world = r3NewWorld();

    const R3Real groundSize = 5, groundHeight = .1;
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -groundHeight, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider =
            r3CuboidColliderDesc(r3Vector(groundSize, groundHeight, groundSize));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -groundHeight, -groundSize);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider =
            r3CuboidColliderDesc(r3Vector(groundSize, groundSize, groundHeight));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Character with stronger gravity and predictive CCD for PID control. */
    R3RigidBodyHandle characterHandle;
    {
        R3RigidBodyDesc rigidBody = r3KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 0.5, 0);
        rigidBody.gravityScale = 10;
        rigidBody.softCcdPrediction = 10;
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CapsuleYColliderDesc(.3, .15);
        characterHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(characterHandle, &collider);
    }
    tbBodyColor(testbed, characterHandle, .8f, .1f, .1f, 1);
    /* Cubes. */
    const int num = 8;
    const R3Real rad = .1, shift = rad * 2, centerx = shift * (num / 2), centery = rad;
    for (int j = 0; j < 4; ++j) {
        for (int k = 0; k < 4; ++k) {
            for (int i = 0; i < num; ++i) {
                const R3Real x = i * shift - centerx, y = j * shift + centery;
                const R3Real z = k * shift + centerx;
                {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation = r3Vector(x, y, z);
                    rigidBody.canSleep = !testbed->noSleep;
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(rad, rad, rad));
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);
                }
            }
        }
    }
    /* Stairs. */
    const R3Real stairWidth = 1, stairHeight = .1;
    for (int i = 0; i < 10; ++i) {
        const R3Real x = i * stairWidth / 2, y = i * stairHeight * 1.5 + 3;
        {
            R3ColliderDesc collider =
                r3CuboidColliderDesc(r3Vector(stairWidth / 2, stairHeight / 2, stairWidth));
            collider.position.translation = r3Vector(x, y, 0);
            r3InsertColliderWithoutParent(world, &collider);
        }
    }
    /* Climbable and unclimbable slopes. */
    const R3Real slopeAngle = .2, slopeSize = 2, impossibleSlopeSize = 2;
    const R3Real impossibleSlopeAngle = .6;
    {
        R3ColliderDesc collider =
            r3CuboidColliderDesc(r3Vector(slopeSize, groundHeight, slopeSize));
        collider.position.translation = r3Vector(.1 + slopeSize, -groundHeight + .4, 0);
        collider.position.rotation = r3RotationFromAxisAngle(r3Vector(0, 0, 1), slopeAngle);
        r3InsertColliderWithoutParent(world, &collider);
    }
    {
        R3ColliderDesc collider =
            r3CuboidColliderDesc(r3Vector(slopeSize, groundHeight, groundSize));
        collider.position.translation =
            r3Vector(.1 + slopeSize * 2 + impossibleSlopeSize - .9, -groundHeight + 1.7, 0);
        collider.position.rotation =
            r3RotationFromAxisAngle(r3Vector(0, 0, 1), impossibleSlopeAngle);
        r3InsertColliderWithoutParent(world, &collider);
    }
    /* Moving platform. */
    R3RigidBodyHandle platformHandle;
    {
        R3RigidBodyDesc rigidBody = r3KinematicVelocityBasedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-8, 0, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(2, groundHeight, 2));
        platformHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(platformHandle, &collider);
    }
    /* Wavy heightfield. */
    const int nsubdivs = 20;
    R3Real heights[21 * 21];
    for (int j = 0; j <= nsubdivs; ++j) {
        for (int i = 0; i <= nsubdivs; ++i) {
            heights[j * 21 + i] = cos(i * 10.0 / nsubdivs / 2) + cos(j * 10.0 / nsubdivs / 2);
        }
    }
    {
        R3ColliderDesc collider = r3DefaultColliderDesc();
        collider.shape.kind = R3_SHAPE_DESC_HEIGHTFIELD;
        collider.shape.heights = (R3RealView){heights, (21) * (21)};
        collider.shape.rows = 21;
        collider.shape.columns = 21;
        collider.shape.scale = r3Vector(10, 1, 10);
        collider.shape.flags = 0;
        collider.position.translation = r3Vector(-8, 5, 0);
        r3InsertColliderWithoutParent(world, &collider);
    }
    /* Tilting dynamic body with a limited joint. */
    R3RigidBodyDesc ground = r3FixedRigidBodyDesc();
    ground.position.translation = r3Vector(0, 5, 0);
    R3RigidBodyHandle groundHandle, handle;
    groundHandle = r3InsertRigidBody(world, &ground);

    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 5, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 0.1, 2));
        handle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handle, &collider);
    }
    {
        R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
        r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_X, -.3, .3);
        r3InsertImpulseJoint(groundHandle, handle, &joint);
    }
    CharacterControlMode controlMode = CHARACTER_KINEMATIC;
    R3KinematicCharacterController *controller = NULL;
    R3PidController *pid = NULL;
    controller = r3NewKinematicCharacterController();
    pid = r3NewPidController();
    r3KinematicCharacterController_SetSlopes(controller, impossibleSlopeAngle - .02,
                                            impossibleSlopeAngle - .02);
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    tbSetWorld(testbed, world);
    size_t stepId = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
            ++stepId;

            R3Real dt = r3TimeStep(world);
            const R3Vector linvel =
                r3Vector(sin(stepId * dt * 2) * 2, sin(stepId * dt * 5) * 1.5, 0);

            r3RigidBody_SetLinvel(platformHandle, linvel, 1);
            updateCharacter(testbed, world, &controlMode, controller, pid, characterHandle);
        }
    }
    r3FreePidController(pid);
    r3FreeKinematicCharacterController(controller);
    r3FreeWorld(world);
}
