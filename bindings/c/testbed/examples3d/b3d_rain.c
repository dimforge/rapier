/* Port of examples3d/b3d_rain.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

enum {
    GRID_COUNT = 10,
    GROUP_SIZE = 3,
    BONE_COUNT = 14,
    PELVIS = 0,
    SPINE_01 = 1,
    SPINE_02 = 2,
    SPINE_03 = 3,
    NECK = 4,
    HEAD = 5,
    THIGH_L = 6,
    CALF_L = 7,
    THIGH_R = 8,
    CALF_R = 9,
    UPPER_ARM_L = 10,
    LOWER_ARM_L = 11,
    UPPER_ARM_R = 12,
    LOWER_ARM_R = 13
};

const R3Real GRID_SIZE = 15;

enum JointKind { Spherical, Revolute };

typedef struct BoneDef {
    int parent;
    R3Vector refP;
    R3Rotation refQ;
    R3Vector capA, capB;
    R3Real capR;
    enum JointKind kind;
    R3Vector frameAP;
    R3Rotation frameAQ;
    R3Vector frameBP;
    R3Rotation frameBQ;
    R3Real swingDeg, twistDeg[2];
    int filtered;
} BoneDef;

static const BoneDef boneDefs[BONE_COUNT] = {
    // pelvis
    {.parent = -1,
     .refP = {0.0, 0.932087, -0.051708},
     .refQ = {0.739169, 0.0, 0.0, 0.673520},
     .capA = {0.07, 0.0, -0.08},
     .capB = {-0.07, 0.0, -0.08},
     .capR = 0.13,
     .kind = Spherical,
     .frameAP = {0, 0, 0},
     .frameAQ = {0.0, 0.0, 0.0, 1.0},
     .frameBP = {0, 0, 0},
     .frameBQ = {0.0, 0.0, 0.0, 1.0},
     .swingDeg = 0.0,
     .twistDeg = {0.0, 0.0},
     .filtered = 0},
    // spine_01
    {.parent = PELVIS,
     .refP = {0.0, 1.113505, -0.03481},
     .refQ = {0.739973, 0.0, 0.0, 0.672637},
     .capA = {0.06, 0.0, -0.052264},
     .capB = {-0.06, 0.0, -0.052264},
     .capR = 0.12,
     .kind = Spherical,
     .frameAP = {0.0, 0.0, -0.182204},
     .frameAQ = {-0.999999, 0.0, 0.0, 0.001194},
     .frameBP = {0.0, 0.0, -0.007736},
     .frameBQ = {-1.0, 0.0, 0.0, 0.0},
     .swingDeg = 25.0,
     .twistDeg = {-15.0, 15.0},
     .filtered = 1},
    // spine_02
    {.parent = SPINE_01,
     .refP = {0.0, 1.194336, -0.027087},
     .refQ = {0.703611, 0.0, 0.0, 0.710586},
     .capA = {0.08, -0.015133, -0.091801},
     .capB = {-0.08, -0.015133, -0.091801},
     .capR = 0.10,
     .kind = Spherical,
     .frameAP = {0.0, 0.0, -0.088935},
     .frameAQ = {-0.998619, 0.0, 0.0, -0.052540},
     .frameBP = {0.0, 0.0, -0.008199},
     .frameBQ = {-1.0, 0.0, 0.0, 0.0},
     .swingDeg = 25.0,
     .twistDeg = {-15.0, 15.0},
     .filtered = 0},
    // spine_03
    {.parent = SPINE_02,
     .refP = {0.0, 1.31043, -0.028232},
     .refQ = {0.669856, 0.000001, -0.000001, 0.742491},
     .capA = {0.11, -0.039753, -0.13},
     .capB = {-0.11, -0.039753, -0.13},
     .capR = 0.145,
     .kind = Spherical,
     .frameAP = {0.0, 0.0, -0.124298},
     .frameAQ = {-0.998921, 0.000001, -0.000001, -0.046434},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {-1.0, 0.0, -0.000001, 0.0},
     .swingDeg = 15.0,
     .twistDeg = {-10.0, 10.0},
     .filtered = 0},
    // neck
    {.parent = SPINE_03,
     .refP = {0.0, 1.575582, -0.055837},
     .refQ = {0.879922, 0.0, 0.0, 0.475118},
     .capA = {-0.000001, 0.0, -0.02},
     .capB = {0.0, -0.005, -0.08},
     .capR = 0.07,
     .kind = Spherical,
     .frameAP = {0.000001, -0.000259, -0.266585},
     .frameAQ = {-0.942192, -0.000001, 0.0, 0.335074},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {-1.0, 0.0, -0.000001, 0.0},
     .swingDeg = 45.0,
     .twistDeg = {-15.0, 15.0},
     .filtered = 0},
    // head
    {.parent = NECK,
     .refP = {0.0, 1.653348, -0.003241},
     .refQ = {0.750288, 0.0, 0.0, 0.661111},
     .capA = {-0.000001, 0.016892, -0.05869},
     .capB = {0.0, -0.003629, -0.115072},
     .capR = 0.0975,
     .kind = Spherical,
     .frameAP = {0.0, 0.001321, -0.093873},
     .frameAQ = {-0.974301, 0.0, 0.0, -0.225251},
     .frameBP = {0.0, 0.001268, -0.005104},
     .frameBQ = {-1.0, 0.0, 0.0, 0.0},
     .swingDeg = 15.0,
     .twistDeg = {-15.0, 15.0},
     .filtered = 0},
    // thigh_l
    {.parent = PELVIS,
     .refP = {0.090416, 0.986104, -0.035090},
     .refQ = {-0.703287, -0.070715, 0.053866, 0.705327},
     .capA = {0.023719, 0.006008, -0.039068},
     .capB = {-0.064492, -0.004664, -0.424718},
     .capR = 0.09,
     .kind = Spherical,
     .frameAP = {0.05, 0.011537, -0.055325},
     .frameAQ = {-0.714896, -0.022305, -0.698361, -0.026790},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {-0.002064, 0.758987, 0.017046, 0.650880},
     .swingDeg = 10.0,
     .twistDeg = {-60.0, 40.0},
     .filtered = 1},
    // calf_l
    {.parent = THIGH_L,
     .refP = {0.101198, 0.527027, -0.037374},
     .refQ = {-0.653328, -0.066860, 0.058582, 0.751838},
     .capA = {0.001778, 0.0, 0.009841},
     .capB = {-0.078577, 0.014707, -0.41816},
     .capR = 0.075,
     .kind = Revolute,
     .frameAP = {-0.069989, 0.000253, -0.453844},
     .frameAQ = {-0.000677, 0.760087, 0.105674, 0.641171},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {-0.044589, 0.765540, 0.053368, 0.639619},
     .swingDeg = 0.0,
     .twistDeg = {-5.0, 45.0},
     .filtered = 0},
    // thigh_r
    {.parent = PELVIS,
     .refP = {-0.090416, 0.986104, -0.03509},
     .refQ = {-0.703287, 0.070715, -0.053865, 0.705326},
     .capA = {-0.023719, 0.006008, -0.039068},
     .capB = {0.064492, -0.004664, -0.424718},
     .capR = 0.09,
     .kind = Spherical,
     .frameAP = {-0.05, 0.011537, -0.055326},
     .frameAQ = {-0.039089, -0.714094, 0.043177, 0.697623},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {0.758805, -0.019886, -0.651012, -0.001759},
     .swingDeg = 10.0,
     .twistDeg = {-30.0, 60.0},
     .filtered = 1},
    // calf_r
    {.parent = THIGH_R,
     .refP = {-0.101198, 0.527027, -0.037373},
     .refQ = {-0.653327, 0.06686, -0.058582, 0.751839},
     .capA = {-0.001820, 0.0, 0.010071},
     .capB = {0.077883, 0.014825, -0.418047},
     .capR = 0.075,
     .kind = Revolute,
     .frameAP = {0.069988, 0.000253, -0.453844},
     .frameAQ = {0.760086, -0.000675, -0.641171, -0.105676},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {0.765540, -0.044589, -0.639619, -0.053368},
     .swingDeg = 0.0,
     .twistDeg = {-45.0, 5.0},
     .filtered = 0},
    // upper_arm_l
    {.parent = SPINE_03,
     .refP = {0.20378, 1.484275, -0.115897},
     .refQ = {0.143082, 0.695980, -0.690130, 0.13733},
     .capA = {0.0, 0.0, 0.0},
     .capB = {-0.091118, 0.037775, 0.229719},
     .capR = 0.075,
     .kind = Spherical,
     .frameAP = {0.203780, -0.069369, -0.181921},
     .frameAQ = {-0.278486, 0.445600, -0.097014, 0.845266},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {-0.201396, -0.001586, 0.901850, 0.382234},
     .swingDeg = 60.0,
     .twistDeg = {-5.0, 5.0},
     .filtered = 0},
    // lower_arm_l
    {.parent = UPPER_ARM_L,
     .refP = {0.305614, 1.242908, -0.117599},
     .refQ = {0.165048, 0.563437, -0.802002, 0.109959},
     .capA = {0.0, 0.0, 0.0},
     .capB = {-0.142406, 0.039392, 0.261092},
     .capR = 0.05,
     .kind = Revolute,
     .frameAP = {-0.095482, 0.039584, 0.240723},
     .frameAQ = {0.512487, -0.180629, 0.839474, 0.003742},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {0.503803, -0.029831, 0.858168, 0.094017},
     .swingDeg = 0.0,
     .twistDeg = {-5.0, 60.0},
     .filtered = 0},
    // upper_arm_r
    {.parent = SPINE_03,
     .refP = {-0.20378, 1.484276, -0.115899},
     .refQ = {0.143083, -0.695978, 0.690132, 0.137329},
     .capA = {0.0, 0.0, 0.0},
     .capB = {0.091118, 0.037775, 0.229718},
     .capR = 0.075,
     .kind = Spherical,
     .frameAP = {-0.203779, -0.069371, -0.181922},
     .frameAQ = {-0.253621, -0.414842, 0.106962, 0.867261},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {-0.201397, 0.001587, -0.901850, 0.382233},
     .swingDeg = 60.0,
     .twistDeg = {-5.0, 5.0},
     .filtered = 0},
    // lower_arm_r
    {.parent = UPPER_ARM_R,
     .refP = {-0.305614, 1.242907, -0.117599},
     .refQ = {0.165048, -0.563437, 0.802002, 0.109959},
     .capA = {0.0, 0.0, 0.0},
     .capB = {0.142406, 0.039392, 0.261092},
     .capR = 0.05,
     .kind = Revolute,
     .frameAP = {0.095484, 0.039585, 0.240723},
     .frameAQ = {-0.180627, 0.512487, -0.003744, -0.839474},
     .frameBP = {0.0, 0.0, 0.0},
     .frameBQ = {-0.029831, 0.503803, -0.094017, -0.858169},
     .swingDeg = 0.0,
     .twistDeg = {-60.0, 5.0},
     .filtered = 0},
};

typedef struct HumanHandles {
    R3RigidBodyHandle bones[BONE_COUNT];
} HumanHandles;

static R3Rotation quat(R3Rotation q) {
    const R3Real norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return (R3Rotation){q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

static HumanHandles createHuman(Testbed *testbed, R3World *world, R3Vector position,
                                R3Real frictionTorque, R3Real hertz, R3Real damping,
                                uint32_t groupBit) {
    HumanHandles human;
    const uint32_t bit = UINT32_C(1) << (groupBit % 24);
    const R3InteractionGroups groups = {bit, ~bit, 0};
    (void)frictionTorque; /* The Rust port also omits the friction torque clamp. */
    for (size_t i = 0; i < BONE_COUNT; ++i) {
        const BoneDef *def = &boneDefs[i];
        R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
        body.canSleep = !testbed->noSleep;
        body.position = r3Pose(r3VectorAdd(position, def->refP), quat(def->refQ));
        human.bones[i] = r3InsertRigidBody(world, &body);

        R3ColliderDesc collider = r3CapsuleColliderDesc(def->capA, def->capB, def->capR);
        collider.density = 1000;
        if (def->filtered) {
            collider.collisionGroups = groups;
        }
        r3InsertCollider(human.bones[i], &collider);
    }
    const R3Real omega = 2 * R3_PI * hertz, stiffness = omega * omega,
                 motorDamping = 2 * damping * omega;
    for (size_t i = 0; i < BONE_COUNT; ++i) {
        const BoneDef *def = &boneDefs[i];
        if (def->parent < 0) {
            continue;
        }
        R3JointDesc joint = r3DefaultJointDesc();
        joint.lockedAxes = 7;
        joint.localFrame1 = r3Pose(def->frameAP, quat(def->frameAQ));
        joint.localFrame2 = r3Pose(def->frameBP, quat(def->frameBQ));
        r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_X, def->twistDeg[0] * R3_PI / 180,
                             def->twistDeg[1] * R3_PI / 180);
        joint.contactsEnabled = 0;
        r3JointDesc_SetMotorModel(&joint, R3_AXIS_ANG_X, 0);
        r3JointDesc_SetMotorPosition(&joint, R3_AXIS_ANG_X, 0, stiffness, motorDamping);
        if (def->kind == Spherical) {
            const R3Real swing = def->swingDeg * R3_PI / 180;
            r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_Y, -swing, swing);
            r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_Z, -swing, swing);
            r3JointDesc_SetMotorModel(&joint, R3_AXIS_ANG_Y, 0);
            r3JointDesc_SetMotorModel(&joint, R3_AXIS_ANG_Z, 0);
            r3JointDesc_SetMotorPosition(&joint, R3_AXIS_ANG_Y, 0, stiffness, motorDamping);
            r3JointDesc_SetMotorPosition(&joint, R3_AXIS_ANG_Z, 0, stiffness, motorDamping);
        } else {
            joint.lockedAxes = 7 | 16 | 32;
        }
        r3InsertImpulseJoint(human.bones[def->parent], human.bones[i], &joint);
    }
    return human;
}

typedef struct RainState {
    HumanHandles groups[GRID_COUNT * GRID_COUNT][GROUP_SIZE];
    size_t columnCount, columnIndex;
} RainState;

static void createGroup(Testbed *testbed, R3World *world, RainState *state, size_t row,
                        size_t col) {
    const size_t groupIndex = row * GRID_COUNT + col;
    const R3Real span = GRID_COUNT * GRID_SIZE, groupDistance = span / GRID_COUNT;
    R3Real x = -.5 * span + groupDistance * (col + .5);
    const R3Real y = 20;
    const R3Real z = -.5 * span + groupDistance * (row + .5);
    for (size_t i = 0; i < GROUP_SIZE; ++i) {
        state->groups[groupIndex][i] =
            createHuman(testbed, world, r3Vector(x, y, z), 5, 1, .7, (uint32_t)groupIndex);
        x += .75;
    }
}

static void destroyGroup(RainState *state, size_t row, size_t col) {
    for (size_t i = 0; i < GROUP_SIZE; ++i) {
        for (size_t j = 0; j < BONE_COUNT; ++j) {
            r3RemoveRigidBody(state->groups[row * GRID_COUNT + col][i].bones[j], 1);
        }
    }
}

static void stepRain(Testbed *testbed, R3World *world, RainState *state, size_t stepCount) {
    if (stepCount & 0x2f) {
        return;
    }
    if (state->columnCount < GRID_COUNT) {
        const size_t col = state->columnCount;
        for (size_t row = 0; row < GRID_COUNT; ++row) {
            createGroup(testbed, world, state, row, col);
        }
        ++state->columnCount;
    } else {
        const size_t col = state->columnIndex;
        for (size_t row = 0; row < GRID_COUNT; ++row) {
            destroyGroup(state, row, col);
            createGroup(testbed, world, state, row, col);
        }
        state->columnIndex = (state->columnIndex + 1) % GRID_COUNT;
    }
}

void tbB3dRain(Testbed *testbed) {
    R3World *world = r3NewWorld();
    r3SetGravity(world, r3Vector(0, -10, 0));
    /* Flat grid and torus, reused in each static cell. */
    R3Vector gridVerts[81];
    uint32_t gridIndices[8 * 8 * 6];
    size_t next = 0;
    R3Real x = -GRID_SIZE / 2;
    for (size_t ix = 0; ix <= 8; ++ix) {
        R3Real z = -GRID_SIZE / 2;
        for (size_t iz = 0; iz <= 8; ++iz) {
            gridVerts[ix * 9 + iz] = r3Vector(x, 0, z);
            z += GRID_SIZE / 8;
        }
        x += GRID_SIZE / 8;
    }
    for (uint32_t ix = 0; ix < 8; ++ix) {
        for (uint32_t iz = 0; iz < 8; ++iz) {
            uint32_t i1 = iz + 9 * ix, i2 = i1 + 1, i3 = i2 + 9, i4 = i3 - 1;
            uint32_t pair[] = {i1, i2, i3, i3, i4, i1};
            memcpy(&gridIndices[next], pair, sizeof(pair));
            next += 6;
        }
    }
    R3Vector torusVerts[16 * 16];
    uint32_t torusIndices[16 * 16 * 6];
    next = 0;
    for (size_t radial = 0; radial < 16; ++radial) {
        for (size_t tubular = 0; tubular < 16; ++tubular) {
            const R3Real u = tubular / 16.0 * 2 * R3_PI, v = radial / 16.0 * 2 * R3_PI;
            torusVerts[radial * 16 + tubular] = r3Vector(
                (.25 * GRID_SIZE + cos(v)) * cos(u), (.25 * GRID_SIZE + cos(v)) * sin(u), sin(v));
        }
    }
    for (uint32_t radial = 0; radial < 16; ++radial) {
        for (uint32_t tubular = 0; tubular < 16; ++tubular) {
            const uint32_t r2 = (radial + 1) % 16, t2 = (tubular + 1) % 16;
            uint32_t i1 = radial * 16 + tubular, i2 = radial * 16 + t2, i3 = r2 * 16 + t2,
                     i4 = r2 * 16 + tubular;
            uint32_t pair[] = {i1, i2, i3, i3, i4, i1};
            memcpy(&torusIndices[next], pair, sizeof(pair));
            next += 6;
        }
    }
    const R3Real span = GRID_SIZE * GRID_COUNT;
    x = -.5 * span + .5 * GRID_SIZE;
    for (size_t i = 0; i < GRID_COUNT; ++i) {
        R3Real z = -.5 * span + .5 * GRID_SIZE;
        for (size_t j = 0; j < GRID_COUNT; ++j) {
            R3RigidBodyDesc builder = r3FixedRigidBodyDesc();
            builder.position.translation = r3Vector(x, 0, z);
            R3RigidBodyHandle cell = r3InsertRigidBody(world, &builder);

            R3ColliderDesc collider = r3DefaultColliderDesc();
            r3ShapeDesc_SetTrimesh(&collider.shape, (R3VectorView){gridVerts, 81},
                                  (R3TriangleView){(const R3Triangle *)gridIndices, 128}, 0);
            r3InsertCollider(cell, &collider);

            collider = r3DefaultColliderDesc();
            r3ShapeDesc_SetTrimesh(&collider.shape, (R3VectorView){torusVerts, 256},
                                  (R3TriangleView){(const R3Triangle *)torusIndices, 512}, 0);
            r3InsertCollider(cell, &collider);

            z += GRID_SIZE;
        }
        x += GRID_SIZE;
    }
    RainState *state = calloc(1, sizeof(*state));
    if (!state) {
        abort();
    }
    tbCamera(testbed, 70, 30, 70, 0, 5, 0);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    size_t stepCount = 0;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            stepRain(testbed, world, state, stepCount);
            ++stepCount;
            r3Step(world, NULL, NULL);
        }
    }
    free(state);
    r3FreeWorld(world);
}
