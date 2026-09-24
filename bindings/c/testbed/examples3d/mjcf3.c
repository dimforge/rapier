/* Port of examples3d/mjcf3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"
#ifdef RAPIER_ROBOTICS
void tbMjcf3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    R3MjcfLoaderOptions options = r3DefaultMjcfLoaderOptions();

    options.makeRootsFixed = 1;
    /* Z-up to Y-up, matching the Rust example's model convention. */
    options.shift = r3Pose(r3Vector(0, 0, 0), r3RotationFromAxisAngle(r3Vector(1, 0, 0), -R3_PI / 2));
    R3RigidBodyDesc blueprint = r3DynamicRigidBodyDesc();
    blueprint.canSleep = !testbed->noSleep;
    options.rigidBodyBlueprint = blueprint;

    char path[4096];
    snprintf(path, sizeof(path), "%s/3d/agility_cassie/scene.xml", testbed->assetRoot);
    R3MjcfRobot *robot = r3MjcfRobotFromFile(path, &options);
    /* Insert the same robot with each joint representation. */
    R3MjcfRobotHandles *impulse = NULL, *multibody = NULL;
    impulse = r3MjcfRobot_InsertUsingImpulseJoints(world, robot);
    r3MjcfRobot_AppendTransform(robot, r3TranslationPose(r3Vector(0, 0, 1)));
    multibody = r3MjcfRobot_InsertUsingMultibodyJoints(
        world, robot, R3_MULTIBODY_SKIP_LOOP_CLOSURES | R3_MULTIBODY_DISABLE_SELF_CONTACTS);
    r3FreeMjcfRobotHandles(impulse);
    r3FreeMjcfRobotHandles(multibody);
    r3FreeMjcfRobot(robot);
    tbSetWorld(testbed, world);
    tbCamera(testbed, 2, 2, 2, 0, 0, 0);
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
#endif
