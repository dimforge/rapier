/* Port of examples2d/debug_self_intersect2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static R2SoftBodyDesc strip(R2Vector center) {
    R2SoftBodyDesc builder = r2GridSoftBodyDesc(center, r2Vector(3, .15), 3, 2);
    static const uint32_t pinned[] = {0, 1, 4, 5};
    r2SoftBodyDesc_SetPinnedParticles(&builder, (R2IndexView){(const uint32_t *)pinned, 4});
    builder.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){3, 1});
    builder.selfContacts = 1;
    return builder;
}

static void reload(R2SoftBodyHandle bullet, R2Vector cannonOrigin) {
    size_t n = r2SoftBody_NumParticles(bullet);
    for (size_t i = 0; i < n; ++i) {
        const R2Real angle = 2 * R2_PI * i / n;
        const R2Vector p =
            r2VectorAdd(cannonOrigin, r2Vector(.3 * cos(angle), 2.5 + .3 * sin(angle)));
        r2SoftBody_SetParticlePosition(bullet, i, p);
        r2SoftBody_SetParticleVelocity(bullet, i, r2Vector(0, -150));
    }
}

static R2SoftBodyHandle eightBlob(Testbed *testbed, R2World *world, R2Vector center, R2Real radius,
                                  size_t n) {
    R2SoftBodyDesc builder = r2DiskSoftBodyDesc(center, radius, n);
    builder.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){4, 1});
    builder.selfContacts = 1;
    builder.particleMass = .05;
    builder.canSleep = !testbed->noSleep;
    R2SoftBodyHandle handle = r2InsertSoftBody(world, &builder);

    return handle;
}

static void geronoEight(R2SoftBodyHandle h, R2Vector center, R2Real r) {
    size_t n = r2SoftBody_NumParticles(h);
    for (size_t i = 0; i < n; ++i) {
        const R2Real t = 2 * R2_PI * i / n;
        r2SoftBody_SetParticlePosition(
            h, i, r2VectorAdd(center, r2Vector(r * cos(t), r * sin(t) * cos(t))));
    }
}

static void asymEight(R2SoftBodyHandle h, R2Vector center, R2Real rb, R2Real rs) {
    size_t n = r2SoftBody_NumParticles(h);
    const size_t nBig = (size_t)(n * rb / (rb + rs));
    for (size_t i = 0; i < n; ++i) {
        R2Vector p;
        if (i < nBig) {
            const R2Real t = 2 * R2_PI * i / nBig;
            p = r2VectorAdd(center, r2Vector(-rb + rb * cos(t), rb * sin(t)));
        } else {
            const R2Real t = 2 * R2_PI * (i - nBig) / (n - nBig);
            p = r2VectorAdd(center, r2Vector(rs - rs * cos(t), -rs * sin(t)));
        }
        r2SoftBody_SetParticlePosition(h, i, p);
    }
}

static void eightFloor(Testbed *testbed, R2World *world, R2Real x) {
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(x, -6.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(3, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
}

void tbDebugSelfIntersect2(Testbed *testbed) {
    R2World *world = r2NewWorld();

    R2SoftBodyHandle captured;
    {
        R2SoftBodyDesc builder = strip(r2Vector(-16, 0));
        builder.canSleep = !testbed->noSleep;
        captured = r2InsertSoftBody(world, &builder);
    }

    r2SoftBody_SetParticlePosition(captured, 3, r2Vector(-15, -.4));
    R2SoftBodyHandle loaded;
    {
        R2SoftBodyDesc builder = strip(r2Vector(-6, 0));
        builder.canSleep = !testbed->noSleep;
        loaded = r2InsertSoftBody(world, &builder);
    }

    r2SoftBody_SetParticlePosition(loaded, 3, r2Vector(-5, -.4));
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-5.7, 0.8);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2BallColliderDesc(.45);
        collider.density = 20;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(2, -1);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(2.5, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    R2SoftBodyHandle blob;
    R2SoftBodyDesc blobBuilder = r2DiskSoftBodyDesc(r2Vector(2, 0), .5, 16);
    blobBuilder.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){4, 1});
    blobBuilder.selfContacts = 1;
    blobBuilder.particleMass = .05;
    blobBuilder.canSleep = !testbed->noSleep;
    blob = r2InsertSoftBody(world, &blobBuilder);

    r2SoftBody_SetParticlePosition(blob, 4, r2Vector(2, -.8));
    const R2Vector grabOrigin = {12, 0};
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(12, -0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(2.5, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    uint32_t bottomRow[7];
    for (uint32_t i = 0; i < 7; ++i) {
        bottomRow[i] = i * 3;
    }
    R2SoftBodyHandle groundStrip;
    R2SoftBodyDesc groundBuilder =
        r2GridSoftBodyDesc(r2Vector(12, 0.15), r2Vector(1.5, 0.15), 7, 3);
    r2SoftBodyDesc_SetPinnedParticles(&groundBuilder, (R2IndexView){(const uint32_t *)bottomRow, 7});
    groundBuilder.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){5, 1});
    groundBuilder.selfContacts = 1;
    groundBuilder.canSleep = !testbed->noSleep;
    groundStrip = r2InsertSoftBody(world, &groundBuilder);

    const uint32_t grabbed = 11;

    R2Vector anchor = r2SoftBody_ParticlePosition(groundStrip, grabbed);
    uint32_t cluster = r2SoftBody_AddCluster(groundStrip, &grabbed, 1);

    R2RigidBodyHandle proxy = r2SoftBody_ClusterProxy(groundStrip, cluster);
    R2RigidBodyDesc mouseBuilder = r2KinematicPositionBasedRigidBodyDesc();
    mouseBuilder.position.translation = anchor;
    R2RigidBodyHandle mouse = r2InsertRigidBody(world, &mouseBuilder);
    R2JointDesc joint = r2DefaultJointDesc();
    joint.lockedAxes = 0;
    r2JointDesc_SetMotorPosition(&joint, R2_AXIS_LIN_X, 0, 1000, 50);
    r2JointDesc_SetMotorPosition(&joint, R2_AXIS_LIN_Y, 0, 1000, 50);
    r2InsertImpulseJoint(mouse, proxy, &joint);

    const R2Vector cannonOrigin = {20.5, 0};
    const uint32_t cannonPins[] = {0, 1, 16, 17};
    R2SoftBodyDesc cannonStrip = r2GridSoftBodyDesc(cannonOrigin, r2Vector(2, 0.1), 9, 2);
    r2SoftBodyDesc_SetPinnedParticles(&cannonStrip, (R2IndexView){(const uint32_t *)cannonPins, 4});
    cannonStrip.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){10, 1});
    cannonStrip.selfContacts = 1;
    cannonStrip.canSleep = !testbed->noSleep;
    r2InsertSoftBody(world, &cannonStrip);

    R2SoftBodyHandle bullet;
    R2SoftBodyDesc bulletBuilder = r2DiskSoftBodyDesc(r2Vector(20.5, 2.5), .3, 12);
    bulletBuilder.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){20, 1});
    bulletBuilder.particleMass = .2;
    bulletBuilder.selfContacts = 1;
    bulletBuilder.canSleep = !testbed->noSleep;
    bullet = r2InsertSoftBody(world, &bulletBuilder);

    reload(bullet, cannonOrigin);
    eightFloor(testbed, world, -14);
    R2SoftBodyHandle asym = eightBlob(testbed, world, r2Vector(-14, -5.35), .6, 24);
    asymEight(asym, r2Vector(-13.9, -5.35), .55, .3);
    eightFloor(testbed, world, -7);
    R2SoftBodyHandle sym = eightBlob(testbed, world, r2Vector(-7, -5.35), .6, 24);
    geronoEight(sym, r2Vector(-7, -5.35), .6);
    eightFloor(testbed, world, 1);
    R2SoftBodyHandle e1 = eightBlob(testbed, world, r2Vector(0.65, -5.35), .6, 24);
    geronoEight(e1, r2Vector(0.65, -5.35), .6);
    R2SoftBodyHandle e2 = eightBlob(testbed, world, r2Vector(1.35, -5.05), .6, 24);
    geronoEight(e2, r2Vector(1.35, -5.05), .6);
    eightFloor(testbed, world, 8);
    R2SoftBodyHandle swallowed = eightBlob(testbed, world, r2Vector(8, -5.35), .6, 24);
    asymEight(swallowed, r2Vector(7.8, -5.35), .55, .3);
    eightBlob(testbed, world, r2Vector(8.65, -5.35), 0.55, 20);
    eightFloor(testbed, world, 15);
    R2SoftBodyHandle overlapped = eightBlob(testbed, world, r2Vector(15, -5.35), .6, 24);
    asymEight(overlapped, r2Vector(15.1, -5.35), .55, .3);
    eightBlob(testbed, world, r2Vector(14.15, -5.35), 0.5, 20);
    eightFloor(testbed, world, 22);
    R2SoftBodyHandle threaded = eightBlob(testbed, world, r2Vector(22, -5.35), .6, 24);
    asymEight(threaded, r2Vector(21.8, -5.35), .55, .3);
    eightBlob(testbed, world, r2Vector(22.1, -5.35), 0.22, 14);
    tbCamera2(testbed, 0, -2.5, 38);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    R2Real t = 0;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            const R2Real cycle = fmod(t, 6);
            R2Vector target;
            if (cycle < 1) {
                target = r2Vector(0, .5 - .48 * cycle);
            } else if (cycle < 4) {
                const R2Real w = 2 * R2_PI * (cycle - 1);
                target = r2Vector(.3 * sin(w), .02 + .03 * (1 - cos(2 * w)));
            } else if (cycle < 5) {
                target = r2Vector(0, .02 + .48 * (cycle - 4));
            } else {
                target = r2Vector(0, .5);
            }

            r2RigidBody_SetNextKinematicTranslation(mouse, r2VectorAdd(grabOrigin, target));
            r2RigidBody_WakeUp(proxy, 1);

            R2Real dt = r2TimeStep(world);
            if (fmod(t, 4) < dt && t > 0) {
                reload(bullet, cannonOrigin);
            }
            r2Step(world, NULL, NULL);
            t += dt;
        }
    }
    r2FreeWorld(world);
}
