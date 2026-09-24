/* Port of examples2d/soft_joints2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static R2SoftBodyDesc jelly(R2Vector center, R2Vector half, R2Real young) {
    R2SoftBodyDesc builder = r2GridSoftBodyDesc(center, half, 5, 5);
    builder.cellModel = R2_SOFT_CELL_COROTATIONAL;
    builder.particleMass = .08;
    builder.particleRadius = (R2OptionalReal){1, .06};
    {
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.youngModulus = young;
        material.poissonRatio = .35;
        material.elasticDampingRatio = .8;
        builder.material = material;
    }
    {
        R2ColliderDesc surface = r2BallColliderDesc(.06);
        surface.friction = .6;
        builder.collider = surface;
    }
    return builder;
}

void tbSoftJoints2(Testbed *testbed) {
    R2World *world = r2NewWorld();

    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(30, 0.5));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Revolute joint and velocity motor. */
    R2SoftBodyHandle spinner;
    {
        R2SoftBodyDesc builder = jelly(r2Vector(-12, 2), r2Vector(0.6, 0.6), 8e3);
        builder.canSleep = !testbed->noSleep;
        spinner = r2InsertSoftBody(world, &builder);
    }
    R2RigidBodyHandle spinnerRoot = r2SoftBody_RootBody(spinner);
    R2Vector com = r2SoftBody_CenterOfMass(spinner);
    R2RigidBodyHandle pivot;
    {
        R2RigidBodyDesc builder = r2FixedRigidBodyDesc();
        builder.position.translation = com;
        pivot = r2InsertRigidBody(world, &builder);
    }
    {
        R2JointDesc joint = r2RevoluteJointDesc();
        r2JointDesc_SetMotorVelocity(&joint, R2_AXIS_ANG_X, 1.5, 60);
        r2InsertImpulseJoint(pivot, spinnerRoot, &joint);
    }
    /* Weld a rigid plate to the jelly's top cluster. */
    R2SoftBodyHandle wobbler;
    {
        R2SoftBodyDesc builder = jelly(r2Vector(-8, 0.61), r2Vector(0.6, 0.6), 2.5e3);
        builder.canSleep = !testbed->noSleep;
        wobbler = r2InsertSoftBody(world, &builder);
    }

    R2Vector positions[64];
    size_t particleCount =
        r2SoftBody_ParticlePositions(wobbler, positions, TB_COUNT(positions));
    R2Real maxY = 0;
    for (size_t i = 0; i < particleCount; ++i) {
        maxY = fmax(maxY, positions[i].y);
    }
    uint32_t top[5];
    size_t topCount = 0;
    for (size_t i = 0; i < particleCount; ++i) {
        if (fabs(positions[i].y - maxY) < 1e-3) {
            top[topCount++] = (uint32_t)i;
        }
    }
    uint32_t topCluster = r2SoftBody_AddCluster(wobbler, top, topCount);
    R2RigidBodyHandle topProxy = r2SoftBody_ClusterProxy(wobbler, topCluster);
    R2Vector topPos;
    { topPos = r2RigidBody_Translation(topProxy); }
    R2RigidBodyHandle plate;
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2VectorAdd(topPos, r2Vector(0, 0.12));
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.7, 0.06));
        collider.density = .4;
        plate = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(plate, &collider);
    }
    {
        R2JointDesc joint = r2FixedJointDesc();
        joint.localFrame1.translation = r2Vector(0, -0.12);
        r2InsertImpulseJoint(plate, topProxy, &joint);
    }
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2VectorAdd(topPos, r2Vector(0.3, 1.4));
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.15, 0.15));
        collider.density = 1.5;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Prismatic motor with two stops. */
    R2SoftBodyHandle shuttle;
    {
        R2SoftBodyDesc builder = jelly(r2Vector(-3, 0.85), r2Vector(0.4, 0.4), 6e3);
        builder.canSleep = !testbed->noSleep;
        shuttle = r2InsertSoftBody(world, &builder);
    }
    R2RigidBodyHandle shuttleRoot = r2SoftBody_RootBody(shuttle);
    R2Vector shuttleCom = r2SoftBody_CenterOfMass(shuttle);
    R2RigidBodyHandle rail;
    {
        R2RigidBodyDesc builder = r2FixedRigidBodyDesc();
        builder.position.translation = shuttleCom;
        rail = r2InsertRigidBody(world, &builder);
    }
    R2ImpulseJointHandle railJoint;
    {
        R2JointDesc joint = r2PrismaticJointDesc(r2Vector(1, 0));
        r2JointDesc_SetLimits(&joint, R2_AXIS_LIN_X, -1.8, 1.8);
        r2JointDesc_SetMotorPosition(&joint, R2_AXIS_LIN_X, 0, 40, 8);
        railJoint = r2InsertImpulseJoint(rail, shuttleRoot, &joint);
    }
    /* Rope over a ledge. */
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(1.5, 1);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1.2, 1));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    R2SoftBodyHandle anchorJelly;
    {
        R2SoftBodyDesc builder = jelly(r2Vector(1.5, 2.6), r2Vector(0.5, 0.5), 1.2e4);
        builder.canSleep = !testbed->noSleep;
        anchorJelly = r2InsertSoftBody(world, &builder);
    }
    R2SoftBodyHandle hangingJelly;
    {
        R2SoftBodyDesc builder = jelly(r2Vector(3.8, 2.6), r2Vector(0.5, 0.5), 1.2e4);
        builder.canSleep = !testbed->noSleep;
        hangingJelly = r2InsertSoftBody(world, &builder);
    }
    R2RigidBodyHandle anchorRoot = r2SoftBody_RootBody(anchorJelly);
    R2RigidBodyHandle hangingRoot = r2SoftBody_RootBody(hangingJelly);
    {
        R2JointDesc joint = r2RopeJointDesc(2.2);
        r2InsertImpulseJoint(anchorRoot, hangingRoot, &joint);
    }
    /* Spring bungee under a gantry. */
    R2SoftBodyHandle bungee;
    {
        R2SoftBodyDesc builder = jelly(r2Vector(6.5, 3.2), r2Vector(0.45, 0.45), 6e3);
        builder.canSleep = !testbed->noSleep;
        bungee = r2InsertSoftBody(world, &builder);
    }
    R2RigidBodyHandle bungeeRoot = r2SoftBody_RootBody(bungee);
    R2RigidBodyHandle gantry;
    {
        R2RigidBodyDesc builder = r2FixedRigidBodyDesc();
        builder.position.translation = r2Vector(6.5, 5.5);
        gantry = r2InsertRigidBody(world, &builder);
    }
    {
        R2JointDesc joint = r2SpringJointDesc(1.2, 25, 1.5);
        r2InsertImpulseJoint(gantry, bungeeRoot, &joint);
    }
    /* Bead on a visual-only pole. */
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(9.5, 2.5);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.05, 2.5));
        collider.collisionGroups = (R2InteractionGroups){0, 0, 0};
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    R2SoftBodyHandle bead;
    {
        R2SoftBodyDesc builder = jelly(r2Vector(9.5, 4.2), r2Vector(0.35, 0.35), 8e3);
        builder.canSleep = !testbed->noSleep;
        bead = r2InsertSoftBody(world, &builder);
    }
    R2RigidBodyHandle beadRoot = r2SoftBody_RootBody(bead);
    R2RigidBodyHandle pole;
    {
        R2RigidBodyDesc builder = r2FixedRigidBodyDesc();
        builder.position.translation = r2Vector(9.5, 2.5);
        pole = r2InsertRigidBody(world, &builder);
    }
    {
        R2JointDesc joint = r2PinSlotJointDesc(r2Vector(0, 1));
        r2JointDesc_SetLimits(&joint, R2_AXIS_LIN_X, -1.8, 1.8);
        r2InsertImpulseJoint(pole, beadRoot, &joint);
    }
    /* Hinge two disjoint clusters of one soft bar. */
    R2SoftBodyHandle bar;
    R2SoftBodyDesc barBuilder = r2GridSoftBodyDesc(r2Vector(13.5, 3), r2Vector(1, 0.22), 9, 3);
    barBuilder.cellModel = R2_SOFT_CELL_COROTATIONAL;
    barBuilder.particleMass = .08;
    barBuilder.particleRadius = (R2OptionalReal){1, .06};
    {
        R2SoftBodyMaterial material = r2DefaultSoftBodyMaterial();
        material.youngModulus = 2e4;
        material.poissonRatio = .35;
        material.elasticDampingRatio = 1;
        barBuilder.material = material;
    }
    {
        R2ColliderDesc surface = r2BallColliderDesc(.06);
        surface.friction = .5;
        barBuilder.collider = surface;
    }
    barBuilder.canSleep = !testbed->noSleep;
    bar = r2InsertSoftBody(world, &barBuilder);

    R2Vector barCom = r2SoftBody_CenterOfMass(bar);
    uint32_t leftHalf[64], rightHalf[64];
    size_t leftCount = 0, rightCount = 0;
    particleCount = r2SoftBody_ParticlePositions(bar, positions, TB_COUNT(positions));
    for (size_t i = 0; i < particleCount; ++i) {
        if (positions[i].x < barCom.x - 1e-3) {
            leftHalf[leftCount++] = i;
        } else if (positions[i].x > barCom.x + 1e-3) {
            rightHalf[rightCount++] = i;
        }
    }
    uint32_t leftCluster = r2SoftBody_AddCluster(bar, leftHalf, leftCount);
    uint32_t rightCluster = r2SoftBody_AddCluster(bar, rightHalf, rightCount);
    R2RigidBodyHandle leftProxy = r2SoftBody_ClusterProxy(bar, leftCluster);
    R2RigidBodyHandle rightProxy = r2SoftBody_ClusterProxy(bar, rightCluster);
    R2Vector leftPos;
    { leftPos = r2RigidBody_Translation(leftProxy); }
    R2Vector rightPos;
    { rightPos = r2RigidBody_Translation(rightProxy); }
    R2RigidBodyHandle barAnchor;
    {
        R2RigidBodyDesc builder = r2FixedRigidBodyDesc();
        builder.position.translation = leftPos;
        barAnchor = r2InsertRigidBody(world, &builder);
    }
    {
        R2JointDesc joint = r2FixedJointDesc();
        r2InsertImpulseJoint(barAnchor, leftProxy, &joint);
    }
    R2ImpulseJointHandle flapJoint;
    {
        R2JointDesc joint = r2RevoluteJointDesc();
        joint.localFrame1.translation = r2VectorSub(barCom, leftPos);
        joint.localFrame2.translation = r2VectorSub(barCom, rightPos);
        r2JointDesc_SetMotorPosition(&joint, R2_AXIS_ANG_X, 0, 80, 10);
        flapJoint = r2InsertImpulseJoint(leftProxy, rightProxy, &joint);
    }
    /* Multibody arm and rope attached to one jelly. */
    R2RigidBodyHandle armRoot;
    {
        R2RigidBodyDesc builder = r2FixedRigidBodyDesc();
        builder.position.translation = r2Vector(17, 6);
        armRoot = r2InsertRigidBody(world, &builder);
    }
    R2RigidBodyHandle link1;
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(18.2, 6);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CapsuleXColliderDesc(.5, .08);
        collider.density = 2;
        link1 = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(link1, &collider);
    }
    {
        R2JointDesc joint = r2RevoluteJointDesc();
        joint.localFrame1.translation = r2Vector(0, 0);
        joint.localFrame2.translation = r2Vector(-1.2, 0);
        r2InsertMultibodyJoint(armRoot, link1, &joint);
    }
    R2RigidBodyHandle link2;
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(19.4, 6);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CapsuleXColliderDesc(.5, .08);
        collider.density = 2;
        link2 = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(link2, &collider);
    }
    {
        R2JointDesc joint = r2RevoluteJointDesc();
        joint.localFrame1.translation = r2Vector(0.6, 0);
        joint.localFrame2.translation = r2Vector(-0.6, 0);
        r2InsertMultibodyJoint(link1, link2, &joint);
    }
    R2SoftBodyHandle pendulum;
    {
        R2SoftBodyDesc builder = jelly(r2Vector(20.2, 5.2), r2Vector(0.5, 0.5), 5e3);
        builder.canSleep = !testbed->noSleep;
        pendulum = r2InsertSoftBody(world, &builder);
    }
    R2RigidBodyHandle pendulumRoot = r2SoftBody_RootBody(pendulum);
    {
        R2JointDesc joint = r2RevoluteJointDesc();
        joint.localFrame1.translation = r2Vector(0.7, 0);
        joint.localFrame2.translation = r2Vector(0, 0.6);
        r2InsertImpulseJoint(link2, pendulumRoot, &joint);
    }
    R2RigidBodyHandle crateBody;
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(20.2, 0.3);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.3, 0.3));
        collider.density = .5;
        crateBody = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(crateBody, &collider);
    }
    {
        R2JointDesc joint = r2RopeJointDesc(4.2);
        r2InsertImpulseJoint(pendulumRoot, crateBody, &joint);
    }
    /* Wave a pinned cluster without a joint. */
    R2SoftBodyHandle strand;
    R2SoftBodyDesc gripBuilder = r2RopeSoftBodyDesc(r2Vector(-16, 5.5), r2Vector(-16, 1.5), 20);
    gripBuilder.particleMass = .05;
    gripBuilder.canSleep = !testbed->noSleep;
    strand = r2InsertSoftBody(world, &gripBuilder);

    uint32_t gripParticles[2];
    for (uint32_t i = 0; i < 2; ++i) {
        gripParticles[i] = i;
    }
    uint32_t grip = r2SoftBody_AddCluster(strand, gripParticles, 2);
    R2RigidBodyHandle gripProxy = r2SoftBody_ClusterProxy(strand, grip);
    r2SoftBody_SetClusterPinned(strand, grip, 1);
    R2Vector gripHome;
    { gripHome = r2RigidBody_Translation(gripProxy); }
    tbCamera2(testbed, 2, 3, 30);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    R2Real t = 0;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R2Real dt = r2TimeStep(world);
            t += dt;

            r2ImpulseJoint_SetMotorPosition(railJoint, R2_AXIS_LIN_X, 1.5 * sin(.6 * t), 40,
                                           8, 1);

            r2ImpulseJoint_SetMotorPosition(flapJoint, R2_AXIS_ANG_X, .8 * sin(1.4 * t), 80,
                                           10, 1);

            r2SoftBody_SetClusterKinematicTarget(
                strand, grip,
                r2Pose(r2VectorAdd(gripHome, r2Vector(1.2 * sin(.7 * t), .15 * sin(1.9 * t))),
                       r2Rotation(.5 * sin(1.1 * t))));
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
