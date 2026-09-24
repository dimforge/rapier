/* Port of examples2d/soft_letters2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include "utils/logo_mesh.h"

void tbSoftLetters2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(25, 1.2));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(25, 25);
        rigidBody.position.rotation = r2Rotation(R2_PI / 2);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(25, 1.2));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-25, 25);
        rigidBody.position.rotation = r2Rotation(R2_PI / 2);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(25, 1.2));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    const R2Real cellSize = 2.4;

    struct Letter {
        R2Vector *positions;
        uint32_t *cells;
        size_t particleCount, indexCount;
    } letters[TB_COUNT(logoMeshes)];

    size_t letterCount = 0;
    for (size_t i = 0; i < TB_COUNT(logoMeshes); ++i) {
        const LogoMesh *mesh = &logoMeshes[i];

        R2VolumeMeshParameters letterMeshing = r2NewVolumeMeshParameters(cellSize);
        R2SoftBodyDesc letter = r2VolumetricSoftBodyDesc(
            (R2VectorView){mesh->vertices, mesh->vertexCount},
            (R2SurfaceElementView){(const R2Edge *)mesh->outline, mesh->edgeCount}, letterMeshing);
        if (letter.positions.count == 0) {
            continue;
        }
        struct Letter *data = &letters[letterCount++];
        data->particleCount = r2SoftBodyDesc_ParticlePositions(&letter, NULL, 0);
        data->indexCount = r2SoftBodyDesc_CellIndices(&letter, NULL, 0);
        data->positions = malloc(data->particleCount * sizeof(*data->positions));
        data->cells = malloc(data->indexCount * sizeof(*data->cells));
        if (!data->positions || !data->cells) {
            abort();
        }
        data->particleCount =
            r2SoftBodyDesc_ParticlePositions(&letter, data->positions, data->particleCount);
        data->indexCount = r2SoftBodyDesc_CellIndices(&letter, data->cells, data->indexCount);
    }
    const R2Real stiffnesses[] = {1e3, 5e3, 1e4, 5e4, 1e5, 5e5, 1e6, 5e6};
    for (size_t row = 0; row < TB_COUNT(stiffnesses); ++row) {
        const R2Real young = stiffnesses[TB_COUNT(stiffnesses) - 1 - row];
        for (size_t ith = 0; ith < letterCount; ++ith) {
            const struct Letter *data = &letters[ith];
            const R2Vector offset = r2Vector(ith * 8.0 - 22, 12 + row * 11);
            R2Vector *positions = malloc(data->particleCount * sizeof(*positions));
            if (!positions) {
                abort();
            }
            for (size_t i = 0; i < data->particleCount; ++i) {
                positions[i] = r2VectorAdd(data->positions[i], offset);
            }
            R2SoftBodyDesc letter = r2DefaultSoftBodyDesc();
            r2SoftBodyDesc_SetParticles(&letter, (R2VectorView){positions, data->particleCount});
            r2SoftBodyDesc_SetCells(
                &letter, (R2CellView){(const R2Triangle *)data->cells, data->indexCount / 3});
            letter.cellModel = R2_SOFT_CELL_COROTATIONAL;
            R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
            material.youngModulus = young;
            material.poissonRatio = .4;
            material.elasticDampingRatio = .5;
            material.deformationDamping = fmin(young / 4e4, 50);
            letter.material = material;
            letter.particleMass = .1;
            letter.particleRadius = (R2OptionalReal){1, .15};
            letter.selfContacts = 1;
            letter.canSleep = !testbed->noSleep;
            r2InsertSoftBody(world, &letter);
            free(positions);
        }
    }
    for (size_t i = 0; i < letterCount; ++i) {
        free(letters[i].positions);
        free(letters[i].cells);
    }
    tbCamera2(testbed, 0, 20, 17);
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
