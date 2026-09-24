/* Port of examples3d/mujoco_menagerie3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"
#ifdef RAPIER_ROBOTICS
#include "utils/files.h"

static size_t lastFramedScene = SIZE_MAX;

static int comparePaths(const void *a, const void *b) {
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}

/* Like the Rust discovery loop, look one directory below the root for scene*.xml. */
static FileNames discoverScenes(const char *root) {
    FileNames scenes = {0}, robots = listDirectory(root);
    for (size_t i = 0; i < robots.count; ++i) {
        char directory[4096];
        snprintf(directory, sizeof(directory), "%s/%s", root, robots.names[i]);
        FileNames files = listDirectory(directory);
        for (size_t j = 0; j < files.count; ++j) {
            const char *name = files.names[j];
            size_t length = strlen(name);
            if (strncmp(name, "scene", 5) || length < 4 || strcmp(name + length - 4, ".xml")) {
                continue;
            }
            char path[8192];
            snprintf(path, sizeof(path), "%s/%s", directory, name);
            addFilename(&scenes, path);
        }
        freeFilenames(&files);
    }
    freeFilenames(&robots);
    qsort(scenes.names, scenes.count, sizeof(*scenes.names), comparePaths);
    return scenes;
}

static R3MjcfLoaderOptions loaderOptions(Testbed *testbed) {
    R3MjcfLoaderOptions options = r3DefaultMjcfLoaderOptions();
    options.skipPlaneGeoms = 1;
    options.makeRootsFixed = 0;
    options.createCollidersFromVisualShapes = 0;
    R3ColliderDesc collider = r3BallColliderDesc(.5);
    collider.density = 0;
    options.colliderBlueprint = collider;

    R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
    body.canSleep = !testbed->noSleep;
    options.rigidBodyBlueprint = body;

    return options;
}

static char *keyframeName(const R3MjcfRobot *robot, size_t key) {
    size_t count = r3MjcfRobot_KeyframeName(robot, key, NULL, 0);
    char *name = malloc(count);
    if (!name) {
        abort();
    }
    count = r3MjcfRobot_KeyframeName(robot, key, name, count);
    return name;
}

static void mergeSiblingKeyframes(R3MjcfRobot *robot, const char *path,
                                  const R3MjcfLoaderOptions *options) {
    const char *slash = strrchr(path, '/');
    if (!slash) {
        return;
    }
    char sibling[8192];
    snprintf(sibling, sizeof(sibling), "%.*s/keyframes.xml", (int)(slash - path), path);
    FILE *file = fopen(sibling, "rb");
    if (!file) {
        return;
    }
    fclose(file);
    R3MjcfRobot *keys = NULL;
    R3ErrorHandler handler = r3SetErrorHandler((R3ErrorHandler){0});
    keys = r3MjcfRobotFromFile(sibling, options);
    R3Status status = r3LastStatus();
    r3SetErrorHandler(handler);
    if (status != R3_OK) {
        fprintf(stderr, "Failed to load sibling keyframes %s: %s\n", sibling, r3LastError());
        return;
    }
    size_t originalCount = 0, count = 0;
    originalCount = r3MjcfRobot_KeyframeCount(robot);
    count = r3MjcfRobot_KeyframeCount(keys);
    for (size_t i = 0; i < count; ++i) {
        char *name = keyframeName(keys, i);
        int exists = 0;
        for (size_t j = 0; name[0] && j < originalCount; ++j) {
            char *existing = keyframeName(robot, j);
            exists |= !strcmp(existing, name);
            free(existing);
        }
        if (!exists) {
            r3MjcfRobot_AppendKeyframe(robot, keys, i);
        }
        free(name);
    }
    r3FreeMjcfRobot(keys);
}

static void addFloor(R3World *world) {
    size_t count = r3ColliderHandles(world, NULL, 0);
    if (!count) {
        return;
    }
    R3ColliderHandle *handles = malloc(count * sizeof(*handles));
    if (!handles) {
        abort();
    }
    count = r3ColliderHandles(world, handles, count);
    R3Aabb total = {r3Vector(INFINITY, INFINITY, INFINITY),
                    r3Vector(-INFINITY, -INFINITY, -INFINITY)};
    for (size_t i = 0; i < count; ++i) {
        R3Aabb bounds = r3Collider_ComputeAabb(handles[i]);
        total.mins.x = fmin(total.mins.x, bounds.mins.x);
        total.mins.y = fmin(total.mins.y, bounds.mins.y);
        total.mins.z = fmin(total.mins.z, bounds.mins.z);
        total.maxs.x = fmax(total.maxs.x, bounds.maxs.x);
        total.maxs.y = fmax(total.maxs.y, bounds.maxs.y);
        total.maxs.z = fmax(total.maxs.z, bounds.maxs.z);
    }
    free(handles);
    R3Vector half = r3VectorScale(r3VectorSub(total.maxs, total.mins), .5);
    R3Vector center = r3VectorScale(r3VectorAdd(total.maxs, total.mins), .5);
    half.x *= 10;
    half.y *= 10;
    center.z -= half.z;
    half.z = .2;
    center.z -= .2;
    R3ColliderDesc floor = r3CuboidColliderDesc(half);
    floor.position.translation = center;
    r3InsertColliderWithoutParent(world, &floor);
}

static void registerVisualMeshes(Testbed *testbed, const R3MjcfRobot *robot,
                                 const R3MjcfRobotHandles *handles, int includePrimitives) {
    size_t bodyCount = r3MjcfRobotHandles_Bodies(handles, NULL, 0);
    R3RigidBodyHandle *bodies = malloc(bodyCount * sizeof(*bodies));
    if (!bodies && bodyCount) {
        abort();
    }
    bodyCount = r3MjcfRobotHandles_Bodies(handles, bodies, bodyCount);
    for (size_t i = 0; i < bodyCount; ++i) {
        if (bodies[i].index == UINT32_MAX) {
            continue;
        }
        size_t count = r3MjcfRobot_BodyVisualCount(robot, i);
        for (size_t j = 0; j < count; ++j) {
            const R3MjcfVisualMesh *visual = NULL;
            R3MjcfVisualMeshInfo info;
            visual = r3MjcfRobot_BodyVisual(robot, i, j);
            info = r3MjcfVisualMesh_Info(visual);
            if (!info.is_trimesh && !includePrimitives) {
                continue;
            }
            R3SharedShape *shape = NULL;
            R3TriMeshData *geometry = NULL;
            shape = r3MjcfVisualMesh_CloneShape(visual);
            geometry = r3SharedShape_ToTrimesh(shape, 24, 12);
            r3FreeSharedShape(shape);
            TbRenderMesh mesh = {.body = bodies[i],
                                 .localPose = info.local_pose,
                                 .metallic = info.material.metallic,
                                 .roughness = info.material.roughness,
                                 .reflectance = info.material.reflectance};
            memcpy(mesh.rgba, info.rgba, sizeof(mesh.rgba));
            memcpy(mesh.emissive, info.material.emissive, sizeof(mesh.emissive));
            mesh.vertexCount = r3TriMeshData_Vertices(geometry, NULL, 0);
            mesh.indexCount = r3TriMeshData_Indices(geometry, NULL, 0);
            mesh.vertices = malloc(mesh.vertexCount * sizeof(*mesh.vertices));
            mesh.indices = malloc(mesh.indexCount * sizeof(*mesh.indices));
            if (!mesh.vertices || !mesh.indices) {
                abort();
            }
            mesh.vertexCount = r3TriMeshData_Vertices(geometry, mesh.vertices, mesh.vertexCount);
            mesh.indexCount = r3TriMeshData_Indices(geometry, mesh.indices, mesh.indexCount);
            r3FreeTriMeshData(geometry);
            size_t uvCount = 0, normalCount = 0, pathCount = 0;
            uvCount = r3MjcfVisualMesh_Uvs(visual, NULL, 0);
            normalCount = r3MjcfVisualMesh_Normals(visual, NULL, 0);
            pathCount = r3MjcfVisualMesh_Texture(visual, NULL, 0);
            if (uvCount == mesh.vertexCount * 2) {
                mesh.uvs = malloc(uvCount * sizeof(float));
                if (!mesh.uvs) {
                    abort();
                }
                uvCount = r3MjcfVisualMesh_Uvs(visual, mesh.uvs, uvCount);
            }
            if (normalCount == mesh.vertexCount * 3) {
                mesh.normals = malloc(normalCount * sizeof(float));
                if (!mesh.normals) {
                    abort();
                }
                normalCount = r3MjcfVisualMesh_Normals(visual, mesh.normals, normalCount);
            }
            mesh.texture = malloc(pathCount);
            if (!mesh.texture) {
                abort();
            }
            pathCount = r3MjcfVisualMesh_Texture(visual, mesh.texture, pathCount);
            if (mesh.texture[0] && !info.has_color) {
                for (int k = 0; k < 4; ++k) {
                    mesh.rgba[k] = 1;
                }
            }
            tbAddBodyRenderMesh(testbed, &mesh);
            free(mesh.vertices);
            free(mesh.indices);
            free(mesh.uvs);
            free(mesh.normals);
            free(mesh.texture);
        }
    }
    free(bodies);
}

void tbMujocoMenagerie3(Testbed *testbed) {
    const char *root = getenv("RAPIER_MENAGERIE_DIR");
    if (!root) {
        root = "../mujoco_menagerie";
    }
    FileNames scenes = discoverScenes(root), labels = {0}, keyNames = {0};
    size_t defaultScene = 0;
    const char *rootName = strrchr(root, '/');
    rootName = rootName ? rootName + 1 : root;
    for (size_t i = 0; i < scenes.count; ++i) {
        if (strstr(scenes.names[i], "unitree_a1")) {
            defaultScene = i;
        }
        char label[8192];
        snprintf(label, sizeof(label), "%s/%s", rootName, scenes.names[i] + strlen(root) + 1);
        addFilename(&labels, label);
    }
    const int useMultibody = tbSetting(testbed, "Use multibody joints", 1, 0, 1, 1);
    const int renderColliders = tbSetting(testbed, "Render colliders", 0, 0, 1, 1);
    const int renderVisualMeshes = tbSetting(testbed, "Render visual meshes", 1, 0, 1, 1);
    const int renderVisualPrimitives = tbSetting(testbed, "Render visual primitives", 0, 0, 1, 1);
    const int disableCollisions = tbSetting(testbed, "Disable collisions", 1, 0, 1, 1);
    const int enableControls = tbSetting(testbed, "Enable joint controls", 1, 0, 1, 1);
    tbLiveSetting(testbed, "Actuator strength", 1, .02, 2, 0);
    const int enableSprings = tbSetting(testbed, "Enable joint springs", 1, 0, 1, 1);
    /* Reserve the keyframe slot above the scene list. */
    tbSetting(testbed, "Keyframe", 0, 0, 0, 1);
    size_t selected = tbChoice(testbed, "Scene", defaultScene, (const char *const *)labels.names,
                               labels.count, 0, 0);
    int sceneChanged = selected != lastFramedScene;
    lastFramedScene = selected;
    R3World *world = r3NewWorld();
    R3MjcfRobot *robot = NULL;
    R3MjcfRobotHandles *handles = NULL;
    size_t keyCount = 0, actuatorCount = 0;
    R3Real **controls = NULL;
    size_t *controlCounts = NULL;
    if (!scenes.count) {
        tbLabel(testbed, "NO MODEL FOUND",
                "Set RAPIER_MENAGERIE_DIR to your google-deepmind/mujoco_menagerie checkout.");
    } else {
        R3MjcfLoaderOptions options = loaderOptions(testbed);
        robot = r3MjcfRobotFromFile(scenes.names[selected], &options);
        mergeSiblingKeyframes(robot, scenes.names[selected], &options);
        if (disableCollisions) {
            size_t bodyCount = r3MjcfRobot_BodyCount(robot);
            for (size_t i = 0; i < bodyCount; ++i) {
                size_t colliderCount = r3MjcfRobot_BodyColliderCount(robot, i);
                for (size_t j = 0; j < colliderCount; ++j) {
                    r3MjcfRobot_SetBodyColliderCollisionGroups(robot, i, j,
                                                              (R3InteractionGroups){1, 2, 0});
                }
            }
        }
        R3Vector gravity = r3MjcfRobot_Gravity(robot);
        r3SetGravity(world, r3Vector(0, 0, -r3VectorLength(gravity)));
        uint8_t flags = disableCollisions ? R3_MULTIBODY_DISABLE_SELF_CONTACTS : 0;
        if (!enableSprings) {
            flags |= R3_MULTIBODY_SKIP_JOINT_SPRINGS;
        }
        keyCount = r3MjcfRobot_KeyframeCount(robot);
        addFilename(&keyNames, "(none)");
        size_t defaultKey = keyCount ? 1 : 0;
        for (size_t i = 0; i < keyCount; ++i) {
            char *name = keyframeName(robot, i);
            char fallback[64];
            snprintf(fallback, sizeof(fallback), "key %zu", i);
            addFilename(&keyNames, name[0] ? name : fallback);
            if (!strcmp(name, "home")) {
                defaultKey = i + 1;
            }
            free(name);
        }
        size_t key = tbChoice(testbed, "Keyframe", defaultKey, (const char *const *)keyNames.names,
                              keyNames.count, useMultibody && enableControls, sceneChanged);
        if (useMultibody) {
            handles = r3MjcfRobot_InsertUsingMultibodyJoints(world, robot, flags);
        } else {
            handles = r3MjcfRobot_InsertUsingImpulseJoints(world, robot);
        }
        if (key) {
            r3MjcfRobotHandles_ApplyKeyframe(handles, robot, key - 1);
        }
        addFloor(world);
        if (renderVisualMeshes) {
            registerVisualMeshes(testbed, robot, handles, renderVisualPrimitives);
        }
        if (useMultibody && enableControls) {
            actuatorCount = r3MjcfRobotHandles_ActuatorCount(handles);
            controls = calloc(keyCount + 1, sizeof(*controls));
            controlCounts = calloc(keyCount + 1, sizeof(*controlCounts));
            if (!controls || !controlCounts) {
                abort();
            }
            controls[0] = calloc(actuatorCount ? actuatorCount : 1, sizeof(R3Real));
            controlCounts[0] = actuatorCount;
            if (!controls[0]) {
                abort();
            }
            for (size_t i = 0; i < keyCount; ++i) {
                controlCounts[i + 1] = r3MjcfRobot_KeyframeControls(robot, i, NULL, 0);
                size_t count = controlCounts[i + 1];
                controls[i + 1] = calloc(count ? count : 1, sizeof(R3Real));
                if (!controls[i + 1]) {
                    abort();
                }
                controlCounts[i + 1] =
                    r3MjcfRobot_KeyframeControls(robot, i, controls[i + 1], count);
            }
        }
    }
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0; /* Source robot and actuator handles are scene-local. */
    testbed->collidersVisible = renderColliders;
    testbed->up[1] = 0;
    testbed->up[2] = 1;
    testbed->frameAll = sceneChanged;
    testbed->preserveCamera = !sceneChanged;
    tbCamera(testbed, 2, 2, 2, 0, 0, .5);
    if (!useMultibody) {
        r3SetTimeStep(testbed->world, 1.0 / 240.0);
        r3SetNumSolverIterations(testbed->world, 12);
    } else {
        r3SetNumInternalPgsIterations(testbed->world, 4);
    }
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
            if (controls) {
                size_t key = tbChoice(testbed, "Keyframe", 0, (const char *const *)keyNames.names,
                                      keyNames.count, 1, 0);
                R3Real gain = tbLiveSetting(testbed, "Actuator strength", 1, .02, 2, 0);
                r3MjcfRobotHandles_ApplyControlsScaled(handles, controls[key],
                                                      controlCounts[key], gain);
            }
        }
    }
    if (controls) {
        for (size_t i = 0; i <= keyCount; ++i) {
            free(controls[i]);
        }
    }
    free(controls);
    free(controlCounts);
    r3FreeMjcfRobotHandles(handles);
    r3FreeMjcfRobot(robot);
    r3FreeWorld(world);
    freeFilenames(&scenes);
    freeFilenames(&labels);
    freeFilenames(&keyNames);
}
#endif
