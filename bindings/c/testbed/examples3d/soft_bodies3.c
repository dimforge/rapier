/* Port of examples3d/soft_bodies3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftBodies3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    /* Ground. */
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(12, 0.1, 12));

        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }

    /* A cloth pinned by its four corners, with a box dropped on it. */
    const uint32_t n = 24;
    const uint32_t pinnedParticles[] = {0, n - 1, n * (n - 1), n * n - 1};
    R3SoftBodyDesc cloth = r3DefaultSoftBodyDesc();
    cloth.kind = R3_SOFT_DESC_CLOTH;
    cloth.a = r3Vector(-3.5, 2.5, -1.2);
    cloth.du = r3Vector(0.1, 0.0, 0.0);
    cloth.dv = r3Vector(0.0, 0.0, 0.1);
    cloth.nx = n;
    cloth.ny = n;
    cloth.pinned = (R3IndexView){pinnedParticles, TB_COUNT(pinnedParticles)};
    cloth.material.edgeSoftness = cloth.material.bendSoftness = cloth.material.volumeSoftness =
        cloth.material.shapeMatchingSoftness = (R3SpringCoefficients){30.0, 1.0};
    cloth.particleMass = 0.05;
    cloth.canSleep = !testbed->noSleep;
    r3InsertSoftBody(world, &cloth);
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-2.35, 4, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.3, 0.3, 0.3));
        collider.density = 0.5;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }

    /* A balloon: hollow sphere with pressure (global volume preservation). */
    R3SoftBodyDesc balloon = r3SphereSoftBodyDesc(r3Vector(0.5, 3.0, 0.0), 0.8, 2);
    balloon.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){15.0, 1.0});
    balloon.volumeFactor = 1.2;
    balloon.particleMass = 0.05;
    balloon.canSleep = !testbed->noSleep;
    r3InsertSoftBody(world, &balloon);

    /* Jelly bodies: corotational, Neo-Hookean, and per-cell volume constraints. */
    {
        R3SoftBodyDesc jelly =
            r3CuboidSoftBodyDesc(r3Vector(3, 1, 1.5), r3Vector(0.6, 0.6, 0.6), 5, 5, 5);
        jelly.cellModel = R3_SOFT_CELL_COROTATIONAL;
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.youngModulus = 2.0e3;
        material.poissonRatio = 0.35;
        material.elasticDampingRatio = 0.5;
        jelly.material = material;
        jelly.particleMass = 0.2;
        jelly.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &jelly);
    }
    {
        R3SoftBodyDesc jelly =
            r3CuboidSoftBodyDesc(r3Vector(3, 1, 4.5), r3Vector(0.6, 0.6, 0.6), 5, 5, 5);
        jelly.cellModel = R3_SOFT_CELL_NEO_HOOKEAN;
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.youngModulus = 2.0e3;
        material.poissonRatio = 0.35;
        material.elasticDampingRatio = 0.5;
        jelly.material = material;
        jelly.particleMass = 0.2;
        jelly.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &jelly);
    }
    {
        R3SoftBodyDesc jelly =
            r3CuboidSoftBodyDesc(r3Vector(3, 1, -1.5), r3Vector(0.6, 0.6, 0.6), 5, 5, 5);
        jelly.cellModel = R3_SOFT_CELL_VOLUME;
        jelly.material = r3UniformSoftBodyMaterial((R3SpringCoefficients){20.0, 1.0});
        jelly.particleMass = 0.2;
        jelly.canSleep = !testbed->noSleep;
        r3InsertSoftBody(world, &jelly);
    }

    /* A rope hanging from a fixed anchor, holding a rigid weight. */
    const uint32_t pinnedParticle = 0;
    R3SoftBodyDesc rope = r3DefaultSoftBodyDesc();
    rope.kind = R3_SOFT_DESC_ROPE;
    rope.a = r3Vector(-0.5, 5, 3);
    rope.b = r3Vector(2.5, 5, 3);
    rope.nx = 30;
    rope.pinned = (R3IndexView){&pinnedParticle, 1};
    rope.material.edgeSoftness = rope.material.bendSoftness = rope.material.volumeSoftness =
        rope.material.shapeMatchingSoftness = (R3SpringCoefficients){40.0, 1.0};
    rope.particleMass = 0.05;
    rope.canSleep = !testbed->noSleep;
    R3SoftBodyHandle ropeHandle = r3InsertSoftBody(world, &rope);

    R3Vector lastPos = r3SoftBody_ParticlePosition(ropeHandle, 29);
    R3RigidBodyHandle weight;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(lastPos, r3Vector(0, -0.3, 0));
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3BallColliderDesc(0.25);
        collider.density = 2.0;
        weight = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(weight, &collider);
    }

    r3SoftBody_AttachParticle(ropeHandle, 29, weight);

    /* Set up the viewer. */
    tbCamera(testbed, 9.0, 6.0, 12.0, 0.0, 1.5, 0.0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
