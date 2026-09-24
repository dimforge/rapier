#include "snippets.h"

/* The snippets run from `website/docs-examples/3d/c`: the assets are those of the Rapier repository. */
#define URDF_PATH "../../../../assets/3d/T12/urdf/T12.URDF"
#define MJCF_PATH "../../../../assets/3d/agility_cassie/scene.xml"
#define MESH_PATH "../../../../assets/3d/chair.obj"

static void urdf(const char *path) {
    // DOCUSAURUS: Urdf start
    R3World *world = r3NewWorld();

    // Read the robot (`path` is the path of the URDF file), then insert its links and joints into the world.
    R3UrdfLoaderOptions options = r3DefaultUrdfLoaderOptions();
    R3UrdfRobot *robot = r3UrdfRobotFromFile(path, &options);
    R3UrdfRobotHandles *handles = r3UrdfRobot_InsertUsingMultibodyJoints(world, robot, 0);
    printf("The robot has %zu links.\n", r3UrdfRobotHandles_Bodies(handles, NULL, 0));

    // The loaded robot and the handles are owned by the application.
    r3FreeUrdfRobotHandles(handles);
    r3FreeUrdfRobot(robot);
    // DOCUSAURUS: Urdf stop
    r3FreeWorld(world);
}

static void urdf_options(const char *path) {
    R3World *world = r3NewWorld();
    // DOCUSAURUS: UrdfOptions start
    R3UrdfLoaderOptions options = r3DefaultUrdfLoaderOptions();
    // Whether colliders are created from the collision shapes of the links.
    // Default: 1
    options.createCollidersFromCollisionShapes = 1;
    // Whether colliders are created from the visual shapes of the links.
    // Default: 0
    options.createCollidersFromVisualShapes = 0;
    // Whether the mass properties declared by the links are applied to their rigid-bodies.
    // Default: 1
    options.applyImportedMassProps = 1;
    // Whether the colliders of two links attached by a joint can collide.
    // Default: 0
    options.enableJointCollisions = 0;
    // Whether the root links are fixed rigid-bodies.
    // Default: 0
    options.makeRootsFixed = 1;
    // The pose applied to the whole robot, e.g., to convert its Z-up convention to Y-up.
    // Default: the identity pose.
    options.shift = r3Pose(r3Vector(0.0, 0.0, 0.0), r3RotationFromAxisAngle(r3Vector(1.0, 0.0, 0.0), R3_PI / 2.0));
    // The description every rigid-body is created from (before the URDF data is applied).
    // Default: a dynamic rigid-body.
    options.rigidBodyBlueprint = r3DynamicRigidBodyDesc();
    R3UrdfRobot *robot = r3UrdfRobotFromFile(path, &options);

    // Insert the same robot twice: once with impulse joints, then 10 units further with multibody joints.
    R3UrdfRobotHandles *impulse_robot = r3UrdfRobot_InsertUsingImpulseJoints(world, robot);
    r3UrdfRobot_AppendTransform(robot, r3TranslationPose(r3Vector(10.0, 0.0, 0.0)));
    R3UrdfRobotHandles *multibody_robot =
        r3UrdfRobot_InsertUsingMultibodyJoints(world, robot, R3_MULTIBODY_DISABLE_SELF_CONTACTS);
    // DOCUSAURUS: UrdfOptions stop

    // Check that the defaults documented above are accurate.
    R3UrdfLoaderOptions defaults = r3DefaultUrdfLoaderOptions();
    if (!defaults.createCollidersFromCollisionShapes || defaults.createCollidersFromVisualShapes ||
        !defaults.applyImportedMassProps || defaults.enableJointCollisions || defaults.makeRootsFixed ||
        defaults.rigidBodyBlueprint.bodyType != R3_DYNAMIC) {
        fprintf(stderr, "Unexpected default URDF loader options.\n");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < 10; i++) {
        r3Step(world, NULL, NULL);
    }
    r3FreeUrdfRobotHandles(impulse_robot);
    r3FreeUrdfRobotHandles(multibody_robot);
    r3FreeUrdfRobot(robot);
    r3FreeWorld(world);
}

static void urdf_string(const char *urdf_xml, const char *mesh_dir) {
    R3World *world = r3NewWorld();
    // DOCUSAURUS: UrdfString start
    // Read the robot from a string containing its URDF description. The relative paths of its meshes are resolved
    // from `mesh_dir` (or from the current directory if it is NULL).
    R3UrdfLoaderOptions options = r3DefaultUrdfLoaderOptions();
    R3UrdfRobot *robot = r3UrdfRobotFromString(urdf_xml, mesh_dir, &options);
    // DOCUSAURUS: UrdfString stop
    r3FreeUrdfRobotHandles(r3UrdfRobot_InsertUsingImpulseJoints(world, robot));
    r3FreeUrdfRobot(robot);
    r3FreeWorld(world);
}

static void mjcf(const char *path) {
    // DOCUSAURUS: Mjcf start
    R3World *world = r3NewWorld();

    // Read the model (`path` is the path of the MJCF file), then insert its bodies and joints into the world.
    R3MjcfLoaderOptions options = r3DefaultMjcfLoaderOptions();
    R3MjcfRobot *robot = r3MjcfRobotFromFile(path, &options);
    R3MjcfRobotHandles *handles = r3MjcfRobot_InsertUsingImpulseJoints(world, robot);

    // The loaded model and the handles are owned by the application.
    r3FreeMjcfRobotHandles(handles);
    r3FreeMjcfRobot(robot);
    // DOCUSAURUS: Mjcf stop
    r3FreeWorld(world);
}

static void mjcf_actuators(const char *path) {
    R3World *world = r3NewWorld();
    R3MjcfLoaderOptions options = r3DefaultMjcfLoaderOptions();
    options.shift = r3Pose(r3Vector(0.0, 0.0, 0.0), r3RotationFromAxisAngle(r3Vector(1.0, 0.0, 0.0), -R3_PI / 2.0));
    R3MjcfRobot *robot = r3MjcfRobotFromFile(path, &options);
    // DOCUSAURUS: MjcfActuators start
    // Unlike the URDF loader, joints are generally inserted as multibody joints (like in MuJoCo).
    R3MjcfRobotHandles *handles = r3MjcfRobot_InsertUsingMultibodyJoints(
        world, robot, R3_MULTIBODY_SKIP_LOOP_CLOSURES | R3_MULTIBODY_DISABLE_SELF_CONTACTS);
    // The gravity declared by the model isn't applied automatically. It is expressed in the frame of the
    // model file, so we rotate it like `options.shift` rotated the model.
    R3Vector gravity = r3MjcfRobot_Gravity(robot);
    r3SetGravity(world, r3RotationTransformVector(options.shift.rotation, gravity));

    // Drive the actuators of the model: one control input per actuator.
    size_t actuator_count = r3MjcfRobotHandles_ActuatorCount(handles);
    R3Real *controls = calloc(actuator_count, sizeof(R3Real));
    controls[0] = 0.5;
    r3MjcfRobotHandles_ApplyControlsScaled(handles, controls, actuator_count, 1.0);
    free(controls);

    // Reset the robot to the first keyframe declared by the model (if any).
    if (r3MjcfRobot_KeyframeCount(robot) > 0) {
        r3MjcfRobotHandles_ApplyKeyframe(handles, robot, 0);
    }
    // DOCUSAURUS: MjcfActuators stop
    for (int i = 0; i < 10; i++) {
        r3Step(world, NULL, NULL);
    }
    r3FreeMjcfRobotHandles(handles);
    r3FreeMjcfRobot(robot);
    r3FreeWorld(world);
}

static void mjcf_contact_hooks(const char *path) {
    R3World *world = r3NewWorld();
    R3MjcfLoaderOptions options = r3DefaultMjcfLoaderOptions();
    R3MjcfRobot *robot = r3MjcfRobotFromFile(path, &options);
    R3MjcfRobotHandles *handles = r3MjcfRobot_InsertUsingMultibodyJoints(world, robot, 0);
    // DOCUSAURUS: MjcfContactHooks start
    // The contact rules of the model, applied by physics hooks given to every step.
    R3MjcfContactHooks *contact_hooks = r3MjcfRobotHandles_ContactHooks(handles, robot);
    R3PhysicsHooks hooks = r3MjcfContactHooks_PhysicsHooks(contact_hooks);
    r3Step(world, &hooks, NULL);
    // The hooks must be freed once they are no longer used by the steps.
    r3FreeMjcfContactHooks(contact_hooks);
    // DOCUSAURUS: MjcfContactHooks stop
    r3FreeMjcfRobotHandles(handles);
    r3FreeMjcfRobot(robot);
    r3FreeWorld(world);
}

static void meshes(const char *path) {
    // DOCUSAURUS: Meshes start
    R3World *world = r3NewWorld();

    // Every mesh of the file (`path`) becomes one shape, converted here into its convex hull.
    R3LoadedMeshes *meshes = r3LoadedMeshesFromFile(path, R3_MESH_CONVERTER_CONVEX_HULL, 0, r3Vector(1.0, 1.0, 1.0));
    for (size_t i = 0; i < r3LoadedMeshes_Count(meshes); i++) {
        R3SharedShape *shape = r3LoadedMeshes_CloneShape(meshes, i);
        R3ColliderDesc collider = r3DefaultColliderDesc();
        collider.shape.kind = R3_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        collider.position = r3LoadedMeshes_Pose(meshes, i);
        r3InsertColliderWithoutParent(world, &collider);
        // The collider keeps its own reference to the shape.
        r3FreeSharedShape(shape);
    }
    r3FreeLoadedMeshes(meshes);
    // DOCUSAURUS: Meshes stop
    r3FreeWorld(world);
}

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    urdf(URDF_PATH);
    urdf_options(URDF_PATH);
    mjcf(MJCF_PATH);
    mjcf_actuators(MJCF_PATH);
    urdf_string("<robot name=\"empty\"><link name=\"base\"/></robot>", NULL);
    mjcf_contact_hooks(MJCF_PATH);
    meshes(MESH_PATH);
    return EXIT_SUCCESS;
}
