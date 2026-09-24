/* Port of examples3d/urdf3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"
#ifdef RAPIER_ROBOTICS
void tbUrdf3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    R3UrdfLoaderOptions options = r3DefaultUrdfLoaderOptions();
    options.createCollidersFromVisualShapes = 1;
    options.createCollidersFromCollisionShapes = 0;
    options.makeRootsFixed = 1;
    /* Z-up to Y-up, matching the Rust example's model convention. */
    options.shift = r3Pose(r3Vector(0, 0, 0), r3RotationFromAxisAngle(r3Vector(1, 0, 0), R3_PI / 2));
    R3RigidBodyDesc blueprint = r3DynamicRigidBodyDesc();
    blueprint.canSleep = !testbed->noSleep;
    options.rigidBodyBlueprint = blueprint;

    char path[4096];
    snprintf(path, sizeof(path), "%s/3d/T12/urdf/T12.URDF", testbed->assetRoot);
    R3UrdfRobot *robot = r3UrdfRobotFromFile(path, &options);
    /* Insert the same robot with each joint representation. */
    R3UrdfRobotHandles *impulse = NULL, *multibody = NULL;
    impulse = r3UrdfRobot_InsertUsingImpulseJoints(world, robot);
    r3UrdfRobot_AppendTransform(robot, r3TranslationPose(r3Vector(10, 0, 0)));
    multibody =
        r3UrdfRobot_InsertUsingMultibodyJoints(world, robot, R3_MULTIBODY_DISABLE_SELF_CONTACTS);
    r3FreeUrdfRobotHandles(impulse);
    r3FreeUrdfRobotHandles(multibody);
    r3FreeUrdfRobot(robot);
    tbSetWorld(testbed, world);
    tbCamera(testbed, 20, 20, 20, 5, 0, 0);
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
#endif
