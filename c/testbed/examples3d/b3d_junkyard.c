/* Port of examples3d/b3d_junkyard.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static void createCylinder(R3Real height, R3Real radius, R3Real yOffset, size_t sides,
                           R3Vector *points) {
    const R3Real deltaAlpha = 2 * R3_PI / sides;
    R3Real alpha = 0;
    for (size_t i = 0; i < sides; ++i) {
        const R3Real sinA = sin(alpha), cosA = cos(alpha);
        points[2 * i] = r3Vector(radius * cosA, yOffset, radius * sinA);
        points[2 * i + 1] = r3Vector(radius * cosA, yOffset + height, radius * sinA);
        alpha += deltaAlpha;
    }
}

static void createRock(R3Real radius, R3Vector points[10]) {
    const R3Real phi = (1 + sqrt(5)) / 2;
    const R3Real theta = 2 * R3_PI / phi;
    const R3Real deltaSin = sin(theta), deltaCos = cos(theta);
    R3Real c = 1, s = 0;
    for (int i = 0; i < 10; ++i) {
        const R3Real z = 1 - (2.0 * i + 1) / 10;
        const R3Real radiusXy = sqrt(1 - z * z);
        points[i] = r3Vector(radius * radiusXy * c, radius * radiusXy * s, radius * z);
        const R3Real c0 = c, s0 = s;
        c = deltaCos * c0 - deltaSin * s0;
        s = deltaSin * c0 + deltaCos * s0;
    }
}

void tbB3dJunkyard(Testbed *testbed) {
    R3World *world = r3NewWorld();
    r3SetGravity(world, r3Vector(0, -10, 0));
    R3RigidBodyHandle ground;
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(120, 1, 120));
        ground = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(ground, &collider);
    }
    {
        R3ColliderDesc wall = r3CuboidColliderDesc(r3Vector(1, 8, 50));
        wall.position.translation = r3Vector(-50, 8, 0);
        r3InsertCollider(ground, &wall);
    }
    {
        R3ColliderDesc wall = r3CuboidColliderDesc(r3Vector(1, 8, 50));
        wall.position.translation = r3Vector(50, 8, 0);
        r3InsertCollider(ground, &wall);
    }
    {
        R3ColliderDesc wall = r3CuboidColliderDesc(r3Vector(50, 8, 1));
        wall.position.translation = r3Vector(0, 8, -50);
        r3InsertCollider(ground, &wall);
    }
    {
        R3ColliderDesc wall = r3CuboidColliderDesc(r3Vector(50, 8, 1));
        wall.position.translation = r3Vector(0, 8, 50);
        r3InsertCollider(ground, &wall);
    }
    R3Vector rockPoints[10];
    createRock(1.5, rockPoints);
    R3SharedShape *rock = r3ConvexHullSharedShape((R3VectorView){rockPoints, 10});
    for (int y = 0; y < 24; ++y) {
        for (int x = 0; x <= 20; ++x) {
            for (int z = 0; z <= 20; ++z) {
                {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation = r3Vector(-40 + 4 * x, 4 * y + 25, -40 + 4 * z);
                    rigidBody.canSleep = !testbed->noSleep;
                    R3ColliderDesc collider = r3DefaultColliderDesc();
                    collider.shape.kind = R3_SHAPE_DESC_SHARED;
                    collider.shape.sharedShape = rock;
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);
                }
            }
        }
    }
    r3FreeSharedShape(rock);
    const R3Real radius = 35;
    R3Vector pusherHull[32];
    createCylinder(24, 4, 0, 16, pusherHull);
    R3RigidBodyHandle pusher;
    {
        R3RigidBodyDesc rigidBody = r3KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(radius, 0, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3DefaultColliderDesc();
        r3ShapeDesc_SetConvexHull(&collider.shape, (R3VectorView){pusherHull, 32});
        pusher = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(pusher, &collider);
    }
    tbCamera(testbed, 0, 90, 125, 0, 0, 0);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    R3Real degrees = 0;
    const R3Real timeStep = 1.0 / 60.0, omega = -6;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            degrees += omega * timeStep;
            const R3Real rad = degrees * R3_PI / 180;

            r3RigidBody_SetNextKinematicTranslation(
                pusher, r3Vector(radius * cos(rad), 0, radius * sin(rad)));
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
