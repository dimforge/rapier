/* Port of examples2d/trimesh2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include "utils/logo_mesh.h"

void tbTrimesh2(Testbed *testbed) {
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
    /* Tessellated by the same SVG utility as the Rust example. */
    for (size_t ith = 0; ith < TB_COUNT(logoMeshes); ++ith) {
        const LogoMesh *mesh = &logoMeshes[ith];
        for (int k = 0; k < 5; ++k) {
            R2ColliderDesc collider = r2DefaultColliderDesc();
            r2ShapeDesc_SetTrimesh(
                &collider.shape, (R2VectorView){mesh->vertices, mesh->vertexCount},
                (R2TriangleView){(const R2Triangle *)mesh->indices, mesh->triangleCount}, 0);
            collider.contactSkin = .2;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(ith * 8.0 - 20, 20 + k * 11);
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
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
