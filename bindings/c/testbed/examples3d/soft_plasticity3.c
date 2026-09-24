/* Port of examples3d/soft_plasticity3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static R3SoftBodyMaterial clay(R3Real young, R3Real plasticYield, R3Real plasticCreep) {
    R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
    material.youngModulus = young;
    material.poissonRatio = .35;
    material.elasticDampingRatio = 1;
    material.plasticYield = plasticYield;
    material.plasticCreep = plasticCreep;
    material.deformationDamping = 4;
    return material;
}

void tbSoftPlasticity3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(14, 0.1, 14));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(9, 2, 5);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.2, 2, 3));
        collider.friction = .8;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Yield ladder: elastic to increasingly plastic, hit by identical balls. */
    const R3Real yields[] = {0, .2, .08, .02};
    for (size_t i = 0; i < TB_COUNT(yields); ++i) {
        const R3Real x = -7 + i * 2.6;
        R3SoftBodyDesc block =
            r3CuboidSoftBodyDesc(r3Vector(x, 0.6, -4), r3Vector(0.6, 0.6, 0.6), 5, 5, 5);
        block.cellModel = R3_SOFT_CELL_COROTATIONAL;
        block.particleMass = .1;
        block.canSleep = !testbed->noSleep;
        {
            R3SoftBodyMaterial material = clay(1.0e4, yields[i], 20);
            block.material = material;
        }
        {
            R3ColliderDesc surface = r3BallColliderDesc(.12);
            surface.friction = .8;
            block.collider = surface;
        }
        r3InsertSoftBody(world, &block);

        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(x, 5, -4);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3BallColliderDesc(.4);
            collider.density = 5;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Clay slab stamped by a kinematic press. */
    R3SoftBodyDesc slab =
        r3CuboidSoftBodyDesc(r3Vector(0, 0.4, 1.5), r3Vector(2.4, 0.4, 1.4), 13, 3, 8);
    slab.cellModel = R3_SOFT_CELL_COROTATIONAL;
    slab.particleMass = .1;
    slab.canSleep = !testbed->noSleep;
    {
        R3SoftBodyMaterial material = clay(3.0e4, .02, 50);
        slab.material = material;
    }
    {
        R3ColliderDesc surface = r3BallColliderDesc(.15);
        surface.friction = .8;
        slab.collider = surface;
    }
    r3InsertSoftBody(world, &slab);

    const R3Vector pressRest = r3Vector(-1.5, 2.2, 1.5);
    R3RigidBodyHandle press;
    {
        R3RigidBodyDesc rigidBody = r3KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = pressRest;
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.35, 0.35, 0.35));
        collider.position.rotation = r3RotationFromAxisAngle(r3Vector(0, 0, 1), R3_PI / 4);
        collider.friction = .5;
        press = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(press, &collider);
    }
    /* Elastic and creeping columns under their own weight. */
    for (int i = 0; i < 2; ++i) {
        const R3Real x = -6 + i * 2;
        R3SoftBodyDesc column =
            r3CuboidSoftBodyDesc(r3Vector(x, 1.2, 5.5), r3Vector(0.3, 1.2, 0.3), 3, 12, 3);
        column.cellModel = R3_SOFT_CELL_COROTATIONAL;
        column.particleMass = .05;
        column.canSleep = !testbed->noSleep;
        {
            R3SoftBodyMaterial material = clay(2.0e3, i == 0 ? 0 : .05, .5);
            column.material = material;
        }
        {
            R3ColliderDesc surface = r3BallColliderDesc(.1);
            surface.friction = 1;
            column.collider = surface;
        }
        r3InsertSoftBody(world, &column);
    }
    /* Volumetric clay balls thrown at the wall. */
    R3SharedShape *shape = r3BallSharedShape(.5);
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
    indexCount = r3TriMeshData_Indices(mesh, indices, indexCount);
    for (int i = 0; i < 3; ++i) {
        const R3Vector center = r3Vector(4 - i * 1.5, 2 + i * .3, 3.5 + i * 1.5);
        vertexCount = r3TriMeshData_Vertices(mesh, vertices, vertexCount);
        for (size_t k = 0; k < vertexCount; ++k) {
            vertices[k] = r3VectorAdd(vertices[k], center);
        }

        R3VolumeMeshParameters ballMeshing = r3NewVolumeMeshParameters(.2);
        R3SoftBodyDesc ball = r3VolumetricSoftBodyDesc(
            (R3VectorView){vertices, vertexCount},
            (R3SurfaceElementView){(const R3Triangle *)indices, indexCount / 3}, ballMeshing);
        {
            ball.cellModel = R3_SOFT_CELL_COROTATIONAL;
            R3SoftBodyMaterial material = clay(1.0e4, .03, 60);
            ball.material = material;
            ball.particleMass = .05;
            ball.canSleep = !testbed->noSleep;
            {
                R3ColliderDesc surface = r3BallColliderDesc(.1);
                surface.friction = .8;
                ball.collider = surface;
            }
            R3SoftBodyHandle handle = r3InsertSoftBody(world, &ball);

            size_t count = r3SoftBody_NumParticles(handle);
            for (size_t k = 0; k < count; ++k) {
                r3SoftBody_SetParticleVelocity(handle, k, r3Vector(10, 1, 0));
            }
        }
    }
    free(vertices);
    free(indices);
    r3FreeTriMeshData(mesh);

    tbCamera(testbed, 4, 9, 16, 0, .5, 0);

    tbSetWorld(testbed, world);
    R3Real t = 0;
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R3Real dt = r3TimeStep(world);
            t += dt;
            /* One second down, one second up, then move to the next stamp. */
            const R3Real period = 3;
            const R3Real cycle = floor(t / period);
            const R3Real phase = t - cycle * period;
            const R3Real x = pressRest.x + fmod(cycle, 4);
            const R3Real depth = 1;
            const R3Real y = pressRest.y - (phase < 1   ? depth * phase
                                            : phase < 2 ? depth * (2 - phase)
                                                        : 0);

            r3RigidBody_SetNextKinematicTranslation(press, r3Vector(x, y, pressRest.z));
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
