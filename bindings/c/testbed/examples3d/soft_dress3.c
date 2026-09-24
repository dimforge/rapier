/* Port of examples3d/soft_dress3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

enum {
    PELVIS,
    TORSO,
    HEAD,
    UPPER_ARM_LEFT,
    UPPER_ARM_RIGHT,
    FOREARM_LEFT,
    FOREARM_RIGHT,
    THIGH_LEFT,
    THIGH_RIGHT,
    SHIN_LEFT,
    SHIN_RIGHT,
    NUM_PARTS
};

static const R3Real UPPER_ARM = .34, FOREARM = .32, THIGH = .5, SHIN = .5, TORSO_RADIUS = .15;

static R3Pose limbPose(R3Vector joint, R3Vector dir, R3Real halfLength) {
    dir = r3VectorNormalize(dir);
    R3Vector axis = r3VectorCross(r3Vector(0, 1, 0), dir);
    R3Rotation rotation;
    if (r3VectorLength(axis) < 1e-7) {
        rotation = r3RotationFromAxisAngle(r3Vector(1, 0, 0), dir.y < 0 ? R3_PI : 0);
    } else {
        rotation = r3RotationFromAxisAngle(axis, acos(fmax(-1, fmin(1, dir.y))));
    }
    return r3Pose(r3VectorAdd(joint, r3VectorScale(dir, halfLength)), rotation);
}

/* Procedural dance, matching frameAt in the Rust example. */
static void frameAt(R3Real t, R3Pose poses[NUM_PARTS]) {
    const R3Real beat = 2 * t, yaw = .7 * sin(.35 * t) + .25 * t;
    const R3Real sway = .25 * sin(beat), bounce = .04 * fabs(sin(2 * beat)), roll = .12 * sin(beat);
    const R3Rotation turn = r3RotationFromAxisAngle(r3Vector(0, 1, 0), yaw);
    const R3Rotation hipsRot =
        r3RotationMul(turn, r3RotationFromAxisAngle(r3Vector(0, 0, 1), roll));
    const R3Vector hips = r3VectorAdd(r3Vector(0, .95 + bounce, 0),
                                      r3RotationTransformVector(turn, r3Vector(sway, 0, 0)));
    const R3Vector up = r3RotationTransformVector(hipsRot, r3Vector(0, 1, 0));
    const R3Vector side = r3RotationTransformVector(hipsRot, r3Vector(1, 0, 0));
    const R3Vector forward = r3RotationTransformVector(hipsRot, r3Vector(0, 0, 1));
    const R3Rotation lean = r3RotationFromAxisAngle(forward, -.5 * roll);
    const R3Rotation torsoRot = r3RotationMul(lean, hipsRot);
    const R3Vector torsoCenter = r3VectorAdd(hips, r3VectorScale(up, .42));
    poses[PELVIS] = r3Pose(hips, hipsRot);
    poses[TORSO] = r3Pose(torsoCenter, torsoRot);
    const R3Vector neck =
        r3VectorAdd(torsoCenter, r3RotationTransformVector(torsoRot, r3Vector(0, .28, 0)));
    poses[HEAD] = r3Pose(r3VectorAdd(neck, r3VectorScale(up, .13)), torsoRot);
    for (size_t i = 0; i < 2; ++i) {
        const R3Real s = i == 0 ? 1 : -1;
        const R3Vector hip =
            r3VectorSub(r3VectorAdd(hips, r3VectorScale(side, .11 * s)), r3VectorScale(up, .05));
        const R3Real phase = i == 0 ? 0 : R3_PI;
        const R3Real swing = .45 * sin(beat + phase);
        const R3Vector thighDir = r3VectorNormalize(r3VectorAdd(
            r3VectorAdd(r3VectorScale(up, -cos(swing)), r3VectorScale(forward, sin(swing))),
            r3VectorScale(side, .05 * s)));
        const R3Vector knee = r3VectorAdd(hip, r3VectorScale(thighDir, THIGH));
        const R3Real bend = .9 * fmax(swing, 0);
        const R3Vector shinDir = r3VectorAdd(r3VectorScale(up, -cos(swing - bend)),
                                             r3VectorScale(forward, sin(swing - bend)));
        poses[THIGH_LEFT + i] = limbPose(hip, thighDir, THIGH * .5);
        poses[SHIN_LEFT + i] = limbPose(knee, shinDir, SHIN * .5);
    }
    for (size_t i = 0; i < 2; ++i) {
        const R3Real s = i == 0 ? 1 : -1;
        const R3Vector shoulder =
            r3VectorAdd(r3VectorSub(neck, r3VectorScale(up, .06)),
                        r3RotationTransformVector(torsoRot, r3Vector(.22 * s, 0, 0)));
        const R3Real raise = .6 + .6 * sin(beat + (i == 0 ? 0 : 1.5));
        const R3Vector armDir = r3VectorNormalize(r3VectorAdd(
            r3VectorSub(r3VectorScale(side, s * cos(raise)), r3VectorScale(up, sin(raise) * .6)),
            r3VectorScale(forward, .2 * sin(.7 * beat))));
        const R3Vector elbow = r3VectorAdd(shoulder, r3VectorScale(armDir, UPPER_ARM));
        const R3Vector foreDir = r3VectorNormalize(
            r3VectorAdd(r3VectorAdd(armDir, r3VectorScale(up, .9)), r3VectorScale(forward, .5)));
        poses[UPPER_ARM_LEFT + i] = limbPose(shoulder, armDir, UPPER_ARM * .5);
        poses[FOREARM_LEFT + i] = limbPose(elbow, foreDir, FOREARM * .5);
    }
}

typedef struct Pin {
    R3SoftBodyHandle handle;
    size_t particle, part;
    R3Vector offset;
} Pin;

static void piece(Testbed *testbed, R3World *world, R3SoftBodyDesc *tube, size_t numAround,
                  size_t part, int selfContacts, const float color[4],
                  const R3Pose frame0[NUM_PARTS], Pin *pins, size_t *pinCount) {
    uint32_t *pinned = malloc(numAround * sizeof(*pinned));
    if (!pinned) {
        abort();
    }
    for (uint32_t i = 0; i < numAround; ++i) {
        pinned[i] = i;
    }
    r3SoftBodyDesc_SetPinnedParticles(tube, (R3IndexView){(const uint32_t *)pinned, numAround});

    R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
    material.edgeSoftness = (R3SpringCoefficients){80, 1.0};
    material.bendSoftness = (R3SpringCoefficients){80, 1.0};
    material.volumeSoftness = (R3SpringCoefficients){80, 1.0};
    material.shapeMatchingSoftness = (R3SpringCoefficients){80, 1.0};
    material.bendSoftness = (R3SpringCoefficients){3, 1};
    tube->material = material;
    tube->particleMass = .01;
    tube->particleRadius = (R3OptionalReal){1, .02};
    tube->selfContacts = selfContacts;
    tube->canSleep = !testbed->noSleep;
    {
        R3ColliderDesc surface = r3BallColliderDesc(.02);
        surface.friction = .5;
        tube->collider = surface;
    }
    R3SoftBodyHandle handle = r3InsertSoftBody(world, tube);
    free(pinned);

    R3RigidBodyHandle root = r3SoftBody_RootBody(handle);
    tbBodyColor(testbed, root, color[0], color[1], color[2], color[3]);
    const R3Pose inverse = r3PoseInverse(frame0[part]);
    for (size_t k = 0; k < numAround; ++k) {
        R3Vector position = r3SoftBody_ParticlePosition(handle, k);
        pins[(*pinCount)++] = (Pin){handle, k, part, r3PoseTransformPoint(inverse, position)};
    }
}

void tbSoftDress3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(10, .1, 10));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    R3Pose frame0[NUM_PARTS];
    frameAt(0, frame0);
    R3ColliderDesc shapes[NUM_PARTS] = {0};
    shapes[PELVIS] = r3CapsuleXColliderDesc(.08, .13);
    shapes[TORSO] = r3CapsuleYColliderDesc(.18, TORSO_RADIUS);
    shapes[HEAD] = r3BallColliderDesc(.12);
    shapes[UPPER_ARM_LEFT] = r3CapsuleYColliderDesc(UPPER_ARM * .5 - .03, .05);
    shapes[UPPER_ARM_RIGHT] = r3CapsuleYColliderDesc(UPPER_ARM * .5 - .03, .05);
    shapes[FOREARM_LEFT] = r3CapsuleYColliderDesc(FOREARM * .5 - .03, .045);
    shapes[FOREARM_RIGHT] = r3CapsuleYColliderDesc(FOREARM * .5 - .03, .045);
    shapes[THIGH_LEFT] = r3CapsuleYColliderDesc(THIGH * .5 - .05, .085);
    shapes[THIGH_RIGHT] = r3CapsuleYColliderDesc(THIGH * .5 - .05, .085);
    shapes[SHIN_LEFT] = r3CapsuleYColliderDesc(SHIN * .5 - .04, .065);
    shapes[SHIN_RIGHT] = r3CapsuleYColliderDesc(SHIN * .5 - .04, .065);
    R3RigidBodyHandle parts[NUM_PARTS];
    for (size_t i = 0; i < NUM_PARTS; ++i) {
        R3RigidBodyDesc body = r3KinematicPositionBasedRigidBodyDesc();
        body.position = frame0[i];
        shapes[i].friction = .4;
        parts[i] = r3InsertRigidBody(world, &body);
        r3InsertCollider(parts[i], &shapes[i]);

        tbBodyColor(testbed, parts[i], .93, .8, .68, 1);
    }
    Pin pins[56 + 48];
    size_t pinCount = 0;
    const R3Vector waist = r3PoseTransformPoint(frame0[PELVIS], r3Vector(0, .1, 0));
    const R3Vector hem = {waist.x, .25, waist.z};
    R3SoftBodyDesc skirt = r3ClothTubeSoftBodyDesc(waist, r3VectorSub(hem, waist), .2, .62, 56, 18);
    const float red[] = {.75, .15, .3, 1};
    piece(testbed, world, &skirt, 56, PELVIS, 1, red, frame0, pins, &pinCount);

    const R3Vector shirtTop = r3PoseTransformPoint(frame0[TORSO], r3Vector(0, .26, 0));
    const R3Vector shirtBottom = r3PoseTransformPoint(frame0[TORSO], r3Vector(0, -.26, 0));
    R3SoftBodyDesc shirt = r3ClothTubeSoftBodyDesc(shirtTop, r3VectorSub(shirtBottom, shirtTop),
                                                   TORSO_RADIUS + .012, TORSO_RADIUS + .09, 48, 16);
    const float white[] = {.95, .95, .9, 1};
    piece(testbed, world, &shirt, 48, TORSO, 0, white, frame0, pins, &pinCount);

    tbCamera(testbed, 3, 2, 4, 0, .9, 0);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    R3Real t = 0;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R3Real dt = r3TimeStep(world);
            t += dt;
            R3Pose frame[NUM_PARTS];
            frameAt(t, frame);

            for (size_t i = 0; i < NUM_PARTS; ++i) {
                r3RigidBody_SetNextKinematicPosition(parts[i], frame[i]);
            }
            for (size_t i = 0; i < pinCount; ++i) {
                r3SoftBody_SetParticleKinematicTarget(
                    pins[i].handle, pins[i].particle,
                    r3PoseTransformPoint(frame[pins[i].part], pins[i].offset));
            }
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
