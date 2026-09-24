/* Port of examples3d/b3d_trees.rs. */
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

static void createWaveMesh(size_t xCount, size_t zCount, R3Real cellWidth, R3Real amplitude,
                           R3Real rowFrequency, R3Real columnFrequency, R3Vector *vertices,
                           uint32_t *indices) {
    const R3Real omegaZ = 2 * R3_PI * rowFrequency * cellWidth;
    const R3Real omegaX = 2 * R3_PI * columnFrequency * cellWidth;
    R3Real x = -.5 * cellWidth * xCount;
    for (size_t ix = 0; ix <= xCount; ++ix) {
        R3Real z = -.5 * cellWidth * zCount;
        for (size_t iz = 0; iz <= zCount; ++iz) {
            vertices[ix * (zCount + 1) + iz] =
                r3Vector(x, amplitude * sin(omegaX * ix) * sin(omegaZ * iz), z);
            z += cellWidth;
        }
        x += cellWidth;
    }
    size_t next = 0;
    for (size_t ix = 0; ix < xCount; ++ix) {
        for (size_t iz = 0; iz < zCount; ++iz) {
            const uint32_t i1 = iz + (zCount + 1) * ix, i2 = i1 + 1;
            const uint32_t i3 = i2 + zCount + 1, i4 = i3 - 1;
            indices[next++] = i1;
            indices[next++] = i2;
            indices[next++] = i3;
            indices[next++] = i3;
            indices[next++] = i4;
            indices[next++] = i1;
        }
    }
}

void b3dTreesRun(Testbed *testbed, size_t scale) {
    R3World *world = r3NewWorld();
    r3SetGravity(world, r3Vector(0, -10, 0));
    const size_t xCount = scale * 150, zCount = scale * 200;
    const size_t vertexCount = (xCount + 1) * (zCount + 1), triangleCount = 2 * xCount * zCount;
    R3Vector *vertices = malloc(vertexCount * sizeof(*vertices));
    uint32_t *indices = malloc(triangleCount * 3 * sizeof(*indices));
    if (!vertices || !indices) {
        abort();
    }
    createWaveMesh(xCount, zCount, 1.0 / scale, .4, .05, .1, vertices, indices);
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 0, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3DefaultColliderDesc();
        r3ShapeDesc_SetTrimesh(&collider.shape, (R3VectorView){vertices, vertexCount},
                              (R3TriangleView){(const R3Triangle *)indices, triangleCount}, 0);
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    free(vertices);
    free(indices);
    R3SharedShape *hulls[22] = {0};
    R3Real y = 1, r = .75;
    const R3Real l = 1.5;
    for (size_t i = 0; i < TB_COUNT(hulls); ++i) {
        R3Vector points[12];
        createCylinder(l + 2 * r, r, y - r, 6, points);
        hulls[i] = r3ConvexHullSharedShape((R3VectorView){points, TB_COUNT(points)});
        y += l + 2 * r;
        r *= .95;
    }

    R3Real angularVelocity = -.5, z = -70;
    const int bodyCount = 50;
    for (int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex) {
        const R3Vector position = r3Vector(0, 1, z);
        R3RigidBodyDesc builder = r3DynamicRigidBodyDesc();
        builder.position.translation = position;
        builder.canSleep = !testbed->noSleep;
        R3RigidBodyHandle handle = r3InsertRigidBody(world, &builder);

        for (size_t i = 0; i < TB_COUNT(hulls); ++i) {
            R3ColliderDesc collider = r3DefaultColliderDesc();
            collider.shape.kind = R3_SHAPE_DESC_SHARED;
            collider.shape.sharedShape = hulls[i];
            collider.density = 1;
            collider.friction = .9;
            r3InsertCollider(handle, &collider);
        }
        const R3Real velocityScale = .5 + .5 * bodyIndex / bodyCount;

        R3Vector center = r3RigidBody_CenterOfMass(handle);
        const R3Vector omega = r3Vector(0, 0, velocityScale * angularVelocity);
        const R3Vector velocity = r3VectorCross(omega, r3VectorSub(center, position));
        r3RigidBody_SetAngvel(handle, omega, 1);
        r3RigidBody_SetLinvel(handle, velocity, 1);
        z += 3;
        angularVelocity = -angularVelocity;
    }
    for (size_t i = 0; i < TB_COUNT(hulls); ++i) {
        r3FreeSharedShape(hulls[i]);
    }
    tbCamera(testbed, 0, 30, 140, 0, 15, 0);
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
