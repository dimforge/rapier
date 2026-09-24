/* Port of examples3d/soft_cloth3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftCloth3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(12, 0.1, 12));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* A sheet dropped over a ball and a box. */
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-1, 1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3BallColliderDesc(1);
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(1.5, 0.6, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.6, 0.6, 0.6));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    const size_t n = 40;
    R3SoftBodyDesc sheet = r3ClothSoftBodyDesc(r3Vector(-3, 3, -2), r3Vector(0.1, 0, 0),
                                               r3Vector(0, 0, 0.1), n + 20, n);
    sheet.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
    sheet.particleMass = .02;
    sheet.canSleep = !testbed->noSleep;
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.edgeSoftness = (R3SpringCoefficients){30, 1.0};
        material.bendSoftness = (R3SpringCoefficients){30, 1.0};
        material.volumeSoftness = (R3SpringCoefficients){30, 1.0};
        material.shapeMatchingSoftness = (R3SpringCoefficients){30, 1.0};
        material.bendSoftness = (R3SpringCoefficients){3, 1};
        sheet.material = material;
    }
    {
        R3ColliderDesc surface = r3BallColliderDesc(.05);
        surface.friction = .8;
        sheet.collider = surface;
    }
    r3InsertSoftBody(world, &sheet);

    /* Curtain pinned along its top edge, with a ball rolling into it. */
    uint32_t pinned[40];
    for (uint32_t i = 0; i < 40; ++i) {
        pinned[i] = i * 30;
    }
    R3SoftBodyDesc curtain =
        r3ClothSoftBodyDesc(r3Vector(-2, 4, 5), r3Vector(0.1, 0, 0), r3Vector(0, -0.1, 0), 40, 30);
    r3SoftBodyDesc_SetPinnedParticles(&curtain, (R3IndexView){(const uint32_t *)pinned, 40});
    curtain.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){30, 1});
    curtain.particleMass = .02;
    curtain.canSleep = !testbed->noSleep;
    r3InsertSoftBody(world, &curtain);

    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 0.5, 9);
        rigidBody.linvel = r3Vector(0, 0, -6);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3BallColliderDesc(.5);
        collider.density = 3;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    tbCamera(testbed, 8, 6, 14, 0, 1.5, 2);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
