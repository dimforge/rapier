/* Port of examples2d/b2d_rain.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

enum {
    ROW_COUNT = 5,
    COLUMN_COUNT = 40,
    GROUP_SIZE = 5,
    GRID_COUNT = 500,
    BONE_COUNT = 11,
    HIP = 0,
    TORSO = 1,
    HEAD = 2,
    UPPER_LEFT_LEG = 3,
    LOWER_LEFT_LEG = 4,
    UPPER_RIGHT_LEG = 5,
    LOWER_RIGHT_LEG = 6,
    UPPER_LEFT_ARM = 7,
    LOWER_LEFT_ARM = 8,
    UPPER_RIGHT_ARM = 9,
    LOWER_RIGHT_ARM = 10
};

const R2Real GRID_SIZE = .5;

typedef struct BoneDef {
    int parent;
    R2Real posY;
    R2Vector capA, capB;
    R2Real capR;
    int hasFoot;
    R2Real pivotY, limits[2], frameAAngle;
} BoneDef;

static const BoneDef boneDefs[BONE_COUNT] = {
    // hip (root, no joint)
    {.parent = -1,
     .posY = 0.95,
     .capA = {0.0, -0.02},
     .capB = {0.0, 0.02},
     .capR = 0.095,
     .hasFoot = 0,
     .pivotY = 0.0,
     .limits = {0.0, 0.0},
     .frameAAngle = 0.0},
    // torso
    {.parent = HIP,
     .posY = 1.2,
     .capA = {0.0, -0.135},
     .capB = {0.0, 0.135},
     .capR = 0.09,
     .hasFoot = 0,
     .pivotY = 1.0,
     .limits = {-0.25 * R2_PI, 0.0},
     .frameAAngle = 0.0},
    // head
    {.parent = TORSO,
     .posY = 1.475,
     .capA = {0.0, -0.038},
     .capB = {0.0, 0.039},
     .capR = 0.075,
     .hasFoot = 0,
     .pivotY = 1.4,
     .limits = {-0.3 * R2_PI, 0.1 * R2_PI},
     .frameAAngle = 0.0},
    // upper left leg
    {.parent = HIP,
     .posY = 0.775,
     .capA = {0.0, -0.125},
     .capB = {0.0, 0.125},
     .capR = 0.06,
     .hasFoot = 0,
     .pivotY = 0.9,
     .limits = {-0.05 * R2_PI, 0.4 * R2_PI},
     .frameAAngle = 0.0},
    // lower left leg
    {.parent = UPPER_LEFT_LEG,
     .posY = 0.475,
     .capA = {0.0, -0.155},
     .capB = {0.0, 0.125},
     .capR = 0.045,
     .hasFoot = 1,
     .pivotY = 0.625,
     .limits = {-0.5 * R2_PI, -0.02 * R2_PI},
     .frameAAngle = 0.0},
    // upper right leg
    {.parent = HIP,
     .posY = 0.775,
     .capA = {0.0, -0.125},
     .capB = {0.0, 0.125},
     .capR = 0.06,
     .hasFoot = 0,
     .pivotY = 0.9,
     .limits = {-0.05 * R2_PI, 0.4 * R2_PI},
     .frameAAngle = 0.0},
    // lower right leg
    {.parent = UPPER_RIGHT_LEG,
     .posY = 0.475,
     .capA = {0.0, -0.155},
     .capB = {0.0, 0.125},
     .capR = 0.045,
     .hasFoot = 1,
     .pivotY = 0.625,
     .limits = {-0.5 * R2_PI, -0.02 * R2_PI},
     .frameAAngle = 0.0},
    // upper left arm
    {.parent = TORSO,
     .posY = 1.225,
     .capA = {0.0, -0.125},
     .capB = {0.0, 0.125},
     .capR = 0.035,
     .hasFoot = 0,
     .pivotY = 1.35,
     .limits = {-0.1 * R2_PI, 0.8 * R2_PI},
     .frameAAngle = 0.0},
    // lower left arm
    {.parent = UPPER_LEFT_ARM,
     .posY = 0.975,
     .capA = {0.0, -0.125},
     .capB = {0.0, 0.125},
     .capR = 0.03,
     .hasFoot = 0,
     .pivotY = 1.1,
     .limits = {-0.2 * R2_PI, 0.3 * R2_PI},
     .frameAAngle = 0.25 * R2_PI},
    // upper right arm
    {.parent = TORSO,
     .posY = 1.225,
     .capA = {0.0, -0.125},
     .capB = {0.0, 0.125},
     .capR = 0.035,
     .hasFoot = 0,
     .pivotY = 1.35,
     .limits = {-0.1 * R2_PI, 0.8 * R2_PI},
     .frameAAngle = 0.0},
    // lower right arm
    {.parent = UPPER_RIGHT_ARM,
     .posY = 0.975,
     .capA = {0.0, -0.125},
     .capB = {0.0, 0.125},
     .capR = 0.03,
     .hasFoot = 0,
     .pivotY = 1.1,
     .limits = {-0.2 * R2_PI, 0.3 * R2_PI},
     .frameAAngle = 0.25 * R2_PI},
};

typedef struct HumanHandles {
    R2RigidBodyHandle bones[BONE_COUNT];
} HumanHandles;

static HumanHandles createHuman(Testbed *testbed, R2World *world, R2Vector position, R2Real scale,
                                R2Real hertz, R2Real damping, uint32_t groupBit) {
    HumanHandles human;
    const uint32_t bit = UINT32_C(1) << (groupBit % 24);
    const R2InteractionGroups groups = {bit, ~bit, 0};
    const R2Vector footPoints[] = {{-.03 * scale, -.185 * scale},
                                   {.11 * scale, -.185 * scale},
                                   {.11 * scale, -.16 * scale},
                                   {-.03 * scale, -.14 * scale}};
    for (size_t i = 0; i < BONE_COUNT; ++i) {
        const BoneDef *def = &boneDefs[i];
        R2RigidBodyDesc body = r2DynamicRigidBodyDesc();
        body.canSleep = !testbed->noSleep;
        body.position.translation = r2VectorAdd(position, r2Vector(0, def->posY * scale));
        human.bones[i] = r2InsertRigidBody(world, &body);

        R2ColliderDesc collider = r2CapsuleColliderDesc(
            r2VectorScale(def->capA, scale), r2VectorScale(def->capB, scale), def->capR * scale);
        collider.friction = .2;
        collider.collisionGroups = groups;
        r2InsertCollider(human.bones[i], &collider);

        if (def->hasFoot) {
            R2SharedShape *shape =
                r2RoundConvexHullSharedShape((R2VectorView){footPoints, 4}, .015 * scale);
            collider = r2DefaultColliderDesc();
            collider.shape.kind = R2_SHAPE_DESC_SHARED;
            collider.shape.sharedShape = shape;

            collider.friction = .05;
            collider.collisionGroups = groups;
            r2InsertCollider(human.bones[i], &collider);
            r2FreeSharedShape(shape);
        }
    }
    const R2Real omega = 2 * R2_PI * hertz, stiffness = omega * omega,
                 motorDamping = 2 * damping * omega;
    for (size_t i = 0; i < BONE_COUNT; ++i) {
        const BoneDef *def = &boneDefs[i];
        if (def->parent < 0) {
            continue;
        }
        R2JointDesc joint = r2DefaultJointDesc();
        joint.lockedAxes = 3;
        const R2Vector anchorA = r2Vector(0, (def->pivotY - boneDefs[def->parent].posY) * scale);
        const R2Vector anchorB = r2Vector(0, (def->pivotY - def->posY) * scale);
        joint.localFrame1 = r2Pose(anchorA, r2Rotation(def->frameAAngle));
        joint.localFrame2 = r2TranslationPose(anchorB);
        r2JointDesc_SetLimits(&joint, R2_AXIS_ANG_X, def->limits[0], def->limits[1]);
        joint.contactsEnabled = 0;
        r2JointDesc_SetMotorModel(&joint, R2_AXIS_ANG_X, 0);
        r2JointDesc_SetMotorPosition(&joint, R2_AXIS_ANG_X, 0, stiffness, motorDamping);
        r2InsertImpulseJoint(human.bones[def->parent], human.bones[i], &joint);
    }
    return human;
}

typedef struct RainState {
    HumanHandles groups[ROW_COUNT * COLUMN_COUNT][GROUP_SIZE];
    size_t columnCount, columnIndex;
} RainState;

static void createGroup(Testbed *testbed, R2World *world, RainState *state, size_t row,
                        size_t col) {
    const size_t groupIndex = row * COLUMN_COUNT + col;
    const R2Real span = GRID_COUNT * GRID_SIZE, groupDistance = span / COLUMN_COUNT;
    R2Real x = -.5 * span + groupDistance * (col + .5);
    const R2Real y = 40 + 45 * row;
    for (size_t i = 0; i < GROUP_SIZE; ++i) {
        state->groups[groupIndex][i] =
            createHuman(testbed, world, r2Vector(x, y), 1, 5, .5, (uint32_t)(i + 1));
        x += .5;
    }
}

static void destroyGroup(R2World *world, RainState *state, size_t row, size_t col) {
    for (size_t i = 0; i < GROUP_SIZE; ++i) {
        for (size_t j = 0; j < BONE_COUNT; ++j) {
            r2RemoveRigidBody(state->groups[row * COLUMN_COUNT + col][i].bones[j], 1);
        }
    }
}

static void stepRain(Testbed *testbed, R2World *world, RainState *state, size_t stepCount) {
    if (stepCount & 0x7) {
        return;
    }
    if (state->columnCount < COLUMN_COUNT) {
        const size_t col = state->columnCount;
        for (size_t row = 0; row < ROW_COUNT; ++row) {
            createGroup(testbed, world, state, row, col);
        }
        ++state->columnCount;
    } else {
        const size_t col = state->columnIndex;
        for (size_t row = 0; row < ROW_COUNT; ++row) {
            destroyGroup(world, state, row, col);
            createGroup(testbed, world, state, row, col);
        }
        state->columnIndex = (state->columnIndex + 1) % COLUMN_COUNT;
    }
}

void tbB2dRain(Testbed *testbed) {
    R2World *world = r2NewWorld();
    r2SetGravity(world, r2Vector(0, -10));
    R2RigidBodyDesc builder = r2FixedRigidBodyDesc();
    R2RigidBodyHandle ground = r2InsertRigidBody(world, &builder);

    R2Real y = 0;
    for (size_t row = 0; row < ROW_COUNT; ++row) {
        R2Real x = -.5 * GRID_COUNT * GRID_SIZE;
        for (size_t i = 0; i <= GRID_COUNT; ++i) {
            R2ColliderDesc collider =
                r2CuboidColliderDesc(r2Vector(.5 * GRID_SIZE, .5 * GRID_SIZE));
            collider.position.translation = r2Vector(x, y);
            r2InsertCollider(ground, &collider);

            x += GRID_SIZE;
        }
        y += 45;
    }
    RainState *state = calloc(1, sizeof(*state));
    if (!state) {
        abort();
    }
    tbCamera2(testbed, 0, 110, 2);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    size_t stepCount = 0;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            stepRain(testbed, world, state, stepCount);
            ++stepCount;
            r2Step(world, NULL, NULL);
        }
    }
    free(state);
    r2FreeWorld(world);
}
