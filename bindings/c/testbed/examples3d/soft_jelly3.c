/* Port of examples3d/soft_jelly3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static R3SoftBodyDesc *jelly(R3SoftBodyDesc *builder, R3Real young) {
    builder->cellModel = R3_SOFT_CELL_COROTATIONAL;
    R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
    material.youngModulus = young;
    material.poissonRatio = .35;
    material.elasticDampingRatio = .5;
    builder->material = material;
    builder->particleMass = .05;
    {
        R3ColliderDesc surface = r3BallColliderDesc(.1);
        surface.friction = .7;
        builder->collider = surface;
    }
    return builder;
}

void tbSoftJelly3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(12, 0.1, 12));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    const R3Real walls[][4] = {
        {-2.5, 0, .1, 2.5}, {2.5, 0, .1, 2.5}, {0, -2.5, 2.5, .1}, {0, 2.5, 2.5, .1}};
    for (size_t i = 0; i < TB_COUNT(walls); ++i) {
        const R3Real dx = walls[i][0], dz = walls[i][1], hx = walls[i][2], hz = walls[i][3];
        {
            R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
            rigidBody.position.translation = r3Vector(dx + 5, 1, dz);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(hx, 1, hz));
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Elastic cubes, softer at the top. */
    const R3Real stiffness[] = {1.0e4, 4.0e3, 1.5e3};
    for (size_t i = 0; i < TB_COUNT(stiffness); ++i) {
        R3SoftBodyDesc cube =
            r3CuboidSoftBodyDesc(r3Vector(-4, .7 + 1.5 * i, 0), r3Vector(0.6, 0.6, 0.6), 5, 5, 5);
        cube.cellModel = R3_SOFT_CELL_COROTATIONAL;
        cube.particleMass = .1;
        {
            R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
            material.youngModulus = stiffness[i];
            material.poissonRatio = .4;
            material.elasticDampingRatio = .5;
            cube.material = material;
        }
        cube.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &cube);
    }
    /* Balloons in the container. */
    for (int i = 0; i < 6; ++i) {
        const R3Real x = 5 + (i % 3 - 1) * 1.2;
        const R3Real z = (i / 3 - .5) * 1.2;
        R3SoftBodyDesc balloon = r3SphereSoftBodyDesc(r3Vector(x, 2 + i * 1.5, z), .6, 2);
        balloon.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){15, 1});
        balloon.volumeFactor = 1.1;
        balloon.particleMass = .03;
        balloon.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &balloon);
    }
    /* Shape-matched cube driven toward an animated pose. */
    R3SoftBodyHandle driven;
    R3SoftBodyDesc cube =
        r3CuboidSoftBodyDesc(r3Vector(0.45, 0.45, 0.45), r3Vector(0.45, 0.45, 0.45), 4, 4, 4);
    cube.shapeMatching = (R3OptionalBool){1, 1};
    cube.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){15, 1});
    cube.particleRadius = (R3OptionalReal){1, .1};
    cube.particleMass = .2;
    cube.gravityScale = 0;
    cube.canSleep = 0;
    driven = r3InsertSoftBody(world, &cube);

    /* Volumetric ball from its boundary mesh. */
    {
        R3SharedShape *shape = r3BallSharedShape(.7);
        R3TriMeshData *mesh = r3SharedShape_ToTrimesh(shape, 16, 16);
        r3FreeSharedShape(shape);
        size_t vertexCount, indexCount;
        vertexCount = r3TriMeshData_Vertices(mesh, NULL, 0);
        indexCount = r3TriMeshData_Indices(mesh, NULL, 0);
        R3Vector *vertices = malloc(vertexCount * sizeof(*vertices));
        uint32_t *indices = malloc(indexCount * sizeof(*indices));
        if (!vertices || !indices) {
            abort();
        }
        vertexCount = r3TriMeshData_Vertices(mesh, vertices, vertexCount);
        indexCount = r3TriMeshData_Indices(mesh, indices, indexCount);
        for (size_t i = 0; i < vertexCount; ++i) {
            vertices[i] = r3VectorAdd(vertices[i], r3Vector(-4, 3, -4));
        }

        R3VolumeMeshParameters ballMeshing = r3NewVolumeMeshParameters(.2);
        R3SoftBodyDesc ball = r3VolumetricSoftBodyDesc(
            (R3VectorView){vertices, vertexCount},
            (R3SurfaceElementView){(const R3Triangle *)indices, indexCount / 3}, ballMeshing);
        {
            jelly(&ball, 1.0e4);
            ball.canSleep = !testbed->noSleep;
            r3InsertSoftBody(world, &ball);
        }
        free(vertices);
        free(indices);
        r3FreeTriMeshData(mesh);
    }
    /* Volumetric capsule from its boundary mesh. */
    {
        R3SharedShape *shape = r3CapsuleSharedShape(r3Vector(0, -.6, 0), r3Vector(0, .6, 0), .45);
        R3TriMeshData *mesh = r3SharedShape_ToTrimesh(shape, 12, 12);
        r3FreeSharedShape(shape);
        size_t vertexCount, indexCount;
        vertexCount = r3TriMeshData_Vertices(mesh, NULL, 0);
        indexCount = r3TriMeshData_Indices(mesh, NULL, 0);
        R3Vector *vertices = malloc(vertexCount * sizeof(*vertices));
        uint32_t *indices = malloc(indexCount * sizeof(*indices));
        if (!vertices || !indices) {
            abort();
        }
        vertexCount = r3TriMeshData_Vertices(mesh, vertices, vertexCount);
        indexCount = r3TriMeshData_Indices(mesh, indices, indexCount);
        for (size_t i = 0; i < vertexCount; ++i) {
            vertices[i] =
                r3VectorAdd(r3RotationTransformVector(
                                r3RotationFromAxisAngle(r3Vector(0, 0, 1), 1.2), vertices[i]),
                            r3Vector(-4, 6, -4));
        }

        R3VolumeMeshParameters capsuleMeshing = r3NewVolumeMeshParameters(.2);
        R3SoftBodyDesc capsule = r3VolumetricSoftBodyDesc(
            (R3VectorView){vertices, vertexCount},
            (R3SurfaceElementView){(const R3Triangle *)indices, indexCount / 3}, capsuleMeshing);
        {
            jelly(&capsule, 3.0e4);
            capsule.canSleep = !testbed->noSleep;
            r3InsertSoftBody(world, &capsule);
        }
        free(vertices);
        free(indices);
        r3FreeTriMeshData(mesh);
    }
    /* Boxes shoved by the driven cube. */
    for (int i = 0; i < 8; ++i) {
        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(-.5 + (i % 4) * .5, 0.25, 4 + (i / 4) * .5);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.2, 0.2, 0.2));
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }

    tbCamera(testbed, 8, 7, 14, 0, 1.5, 1);

    tbSetWorld(testbed, world);
    R3Real t = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R3Real dt = r3TimeStep(world);
            t += dt;
            const R3Vector center = r3Vector(2 * cos(t), .6, 4 + 2 * sin(t));
            const R3Rotation rotation = r3RotationFromAxisAngle(r3Vector(0, 1, 0), 2 * t);
            const R3Pose target = r3Pose(center, rotation);

            r3SoftBody_SetClusterShapeMatchingTarget(driven, 0, &target);
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
