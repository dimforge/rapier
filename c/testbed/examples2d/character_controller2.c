/* Port of examples2d/character_controller2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include "utils/character.h"

void tbCharacterController2(Testbed *testbed) {
    R2World *world = r2NewWorld();

    const R2Real groundSize = 5, groundHeight = .1;
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -groundHeight);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(groundSize, groundHeight));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Character with stronger gravity and predictive CCD for PID control. */
    R2RigidBodyHandle characterHandle;
    {
        R2RigidBodyDesc rigidBody = r2KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-3, 5);
        rigidBody.gravityScale = 10;
        rigidBody.softCcdPrediction = 10;
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CapsuleYColliderDesc(.3, .15);
        characterHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(characterHandle, &collider);
    }
    tbBodyColor(testbed, characterHandle, .8f, .1f, .1f, 1);
    /* Cubes. */
    const int num = 8;
    const R2Real rad = .1, shift = rad * 2, centerx = shift * (num / 2), centery = rad;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < num; ++i) {
            const R2Real x = i * shift - centerx, y = j * shift + centery;
            {
                R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                rigidBody.position.translation = r2Vector(x, y);
                rigidBody.canSleep = !testbed->noSleep;
                R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(rad, rad));
                R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
                r2InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    /* Stairs. */
    const R2Real stairWidth = 1, stairHeight = .1;
    for (int i = 0; i < 10; ++i) {
        const R2Real x = i * stairWidth / 2, y = i * stairHeight * 1.5 + 3;
        {
            R2ColliderDesc collider =
                r2CuboidColliderDesc(r2Vector(stairWidth / 2, stairHeight / 2));
            collider.position.translation = r2Vector(x, y);
            r2InsertColliderWithoutParent(world, &collider);
        }
    }
    /* Climbable and unclimbable slopes. */
    const R2Real slopeAngle = .2, slopeSize = 2, impossibleSlopeSize = 2;
    const R2Real impossibleSlopeAngle = .9;
    {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(slopeSize, groundHeight));
        collider.position.translation = r2Vector(groundSize + slopeSize, -groundHeight + .4);
        collider.position.rotation = r2Rotation(slopeAngle);
        r2InsertColliderWithoutParent(world, &collider);
    }
    {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(slopeSize, groundHeight));
        collider.position.translation =
            r2Vector(groundSize + slopeSize * 2 + impossibleSlopeSize - .9, -groundHeight + 2.3);
        collider.position.rotation = r2Rotation(impossibleSlopeAngle);
        r2InsertColliderWithoutParent(world, &collider);
    }
    /* Wall and horizontal ledge. */
    const R2Vector wallPos =
        r2Vector(groundSize + slopeSize * 2 + impossibleSlopeSize + .35, -groundHeight + 2.5 * 2.3);
    {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(2, groundHeight));
        collider.position.translation = wallPos;
        collider.position.rotation = r2Rotation(R2_PI / 2);
        r2InsertColliderWithoutParent(world, &collider);
    }
    {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(2, groundHeight));
        collider.position.translation = wallPos;
        r2InsertColliderWithoutParent(world, &collider);
    }
    /* Moving platform. */
    R2RigidBodyHandle platformHandle;
    {
        R2RigidBodyDesc rigidBody = r2KinematicVelocityBasedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-8, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(2, groundHeight));
        platformHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(platformHandle, &collider);
    }
    /* Wavy heightfield. */
    const int nsubdivs = 20;
    R2Real heights[21];
    for (int i = 0; i <= nsubdivs; ++i) {
        heights[i] = cos(i * 10.0 / nsubdivs / 2) * 1.5;
    }
    {
        R2ColliderDesc collider = r2DefaultColliderDesc();
        collider.shape.kind = R2_SHAPE_DESC_HEIGHTFIELD;
        collider.shape.heights = (R2RealView){heights, (21) * (1)};
        collider.shape.rows = 21;
        collider.shape.columns = 1;
        collider.shape.scale = r2Vector(10, 1);
        collider.shape.flags = 0;
        collider.position.translation = r2Vector(-8, 5);
        r2InsertColliderWithoutParent(world, &collider);
    }
    /* Tilting dynamic body with a limited joint. */
    R2RigidBodyDesc ground = r2FixedRigidBodyDesc();
    ground.position.translation = r2Vector(0, 5);
    R2RigidBodyHandle groundHandle, handle;
    groundHandle = r2InsertRigidBody(world, &ground);

    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, 5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1, 0.1));
        handle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(handle, &collider);
    }
    {
        R2JointDesc joint = r2RevoluteJointDesc();
        r2JointDesc_SetLimits(&joint, R2_AXIS_ANG_X, -.3, .3);
        r2InsertImpulseJoint(groundHandle, handle, &joint);
    }
    CharacterControlMode controlMode = CHARACTER_KINEMATIC;
    R2KinematicCharacterController *controller = NULL;
    R2PidController *pid = NULL;
    controller = r2NewKinematicCharacterController();
    pid = r2NewPidController();
    r2KinematicCharacterController_SetSlopes(controller, impossibleSlopeAngle - .02,
                                            impossibleSlopeAngle - .02);
    tbCamera2(testbed, 0, 1, 100);

    tbSetWorld(testbed, world);
    size_t stepId = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
            ++stepId;

            R2Real dt = r2TimeStep(world);
            const R2Vector linvel = r2Vector(sin(stepId * dt * 2) * 2, sin(stepId * dt * 5) * 1.5);

            r2RigidBody_SetLinvel(platformHandle, linvel, 1);
            updateCharacter(testbed, world, &controlMode, controller, pid, characterHandle);
        }
    }
    r2FreePidController(pid);
    r2FreeKinematicCharacterController(controller);
    r2FreeWorld(world);
}
