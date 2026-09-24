/* Port of examples3d/soft_joints3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static R3SoftBodyDesc jelly(R3Vector center, R3Vector half, R3Real young) {
    R3SoftBodyDesc builder = r3CuboidSoftBodyDesc(center, half, 4, 4, 4);
    builder.cellModel = R3_SOFT_CELL_COROTATIONAL;
    builder.particleMass = .08;
    builder.particleRadius = (R3OptionalReal){1, .06};
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.youngModulus = young;
        material.poissonRatio = .35;
        material.elasticDampingRatio = .8;
        builder.material = material;
    }
    {
        R3ColliderDesc surface = r3BallColliderDesc(.06);
        surface.friction = .6;
        builder.collider = surface;
    }
    return builder;
}

void tbSoftJoints3(Testbed *testbed) {
    R3World *world = r3NewWorld();

    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(18, 0.1, 18));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Revolute joint and velocity motor. */
    R3SoftBodyHandle spinner;
    {
        R3SoftBodyDesc builder = jelly(r3Vector(-6, 1.6, -4), r3Vector(0.5, 0.5, 0.5), 8e3);
        builder.canSleep = !testbed->noSleep;
        spinner = r3InsertSoftBody(world, &builder);
    }
    R3RigidBodyHandle spinnerRoot = r3SoftBody_RootBody(spinner);
    R3Vector com = r3SoftBody_CenterOfMass(spinner);
    R3RigidBodyHandle pivot;
    {
        R3RigidBodyDesc builder = r3FixedRigidBodyDesc();
        builder.position.translation = com;
        pivot = r3InsertRigidBody(world, &builder);
    }
    {
        R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 1, 0));
        r3JointDesc_SetMotorVelocity(&joint, R3_AXIS_ANG_X, 1.5, 60);
        r3InsertImpulseJoint(pivot, spinnerRoot, &joint);
    }
    /* A cloth corner follows a circling kinematic mover. */
    R3SoftBodyHandle cloth;
    R3SoftBodyDesc clothBuilder = r3ClothSoftBodyDesc(r3Vector(0, 2.6, 2.5), r3Vector(0.16, 0, 0),
                                                      r3Vector(0, 0, 0.16), 12, 12);
    clothBuilder.canSleep = !testbed->noSleep;
    cloth = r3InsertSoftBody(world, &clothBuilder);

    const uint32_t cornerParticles[] = {0, 1, 12};
    uint32_t corner = r3SoftBody_AddCluster(cloth, cornerParticles, 3);
    R3RigidBodyHandle cornerProxy = r3SoftBody_ClusterProxy(cloth, corner);
    R3Vector cornerPos;
    { cornerPos = r3RigidBody_Translation(cornerProxy); }
    R3RigidBodyHandle mover;
    {
        R3RigidBodyDesc builder = r3KinematicPositionBasedRigidBodyDesc();
        builder.position.translation = cornerPos;
        mover = r3InsertRigidBody(world, &builder);
    }
    {
        R3JointDesc joint = r3SphericalJointDesc();
        r3InsertImpulseJoint(mover, cornerProxy, &joint);
    }
    /* Weld a rigid plate to the jelly's top cluster. */
    R3SoftBodyHandle wobbler;
    {
        R3SoftBodyDesc builder = jelly(r3Vector(-2.5, 0.61, -4), r3Vector(0.6, 0.6, 0.6), 2.5e3);
        builder.canSleep = !testbed->noSleep;
        wobbler = r3InsertSoftBody(world, &builder);
    }

    R3Vector positions[64];
    size_t particleCount =
        r3SoftBody_ParticlePositions(wobbler, positions, TB_COUNT(positions));
    R3Real maxY = 0;
    for (size_t i = 0; i < particleCount; ++i) {
        maxY = fmax(maxY, positions[i].y);
    }
    uint32_t top[16];
    size_t topCount = 0;
    for (size_t i = 0; i < particleCount; ++i) {
        if (fabs(positions[i].y - maxY) < 1e-3) {
            top[topCount++] = (uint32_t)i;
        }
    }
    uint32_t topCluster = r3SoftBody_AddCluster(wobbler, top, topCount);
    R3RigidBodyHandle topProxy = r3SoftBody_ClusterProxy(wobbler, topCluster);
    R3Vector topPos;
    { topPos = r3RigidBody_Translation(topProxy); }
    R3RigidBodyHandle plate;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(topPos, r3Vector(0, 0.12, 0));
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.7, 0.06, 0.7));
        collider.density = .4;
        plate = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(plate, &collider);
    }
    {
        R3JointDesc joint = r3FixedJointDesc();
        joint.localFrame1.translation = r3Vector(0, -0.12, 0);
        r3InsertImpulseJoint(plate, topProxy, &joint);
    }
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(topPos, r3Vector(0.3, 1.4, 0.2));
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.15, 0.15, 0.15));
        collider.density = 1.5;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Prismatic motor with two stops. */
    R3SoftBodyHandle shuttle;
    {
        R3SoftBodyDesc builder = jelly(r3Vector(2, 0.85, -4), r3Vector(0.4, 0.4, 0.4), 6e3);
        builder.canSleep = !testbed->noSleep;
        shuttle = r3InsertSoftBody(world, &builder);
    }
    R3RigidBodyHandle shuttleRoot = r3SoftBody_RootBody(shuttle);
    R3Vector shuttleCom = r3SoftBody_CenterOfMass(shuttle);
    R3RigidBodyHandle rail;
    {
        R3RigidBodyDesc builder = r3FixedRigidBodyDesc();
        builder.position.translation = shuttleCom;
        rail = r3InsertRigidBody(world, &builder);
    }
    R3ImpulseJointHandle railJoint;
    {
        R3JointDesc joint = r3PrismaticJointDesc(r3Vector(1, 0, 0));
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_X, -1.8, 1.8);
        r3JointDesc_SetMotorPosition(&joint, R3_AXIS_LIN_X, 0, 40, 8);
        railJoint = r3InsertImpulseJoint(rail, shuttleRoot, &joint);
    }
    /* Rope over a ledge. */
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-1, 1, -9);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1.6, 1, 1.2));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    R3SoftBodyHandle anchorJelly;
    {
        R3SoftBodyDesc builder = jelly(r3Vector(-1, 2.6, -9), r3Vector(0.5, 0.5, 0.5), 1.2e4);
        builder.canSleep = !testbed->noSleep;
        anchorJelly = r3InsertSoftBody(world, &builder);
    }
    R3SoftBodyHandle hangingJelly;
    {
        R3SoftBodyDesc builder = jelly(r3Vector(1.6, 2.6, -9), r3Vector(0.5, 0.5, 0.5), 1.2e4);
        builder.canSleep = !testbed->noSleep;
        hangingJelly = r3InsertSoftBody(world, &builder);
    }
    R3RigidBodyHandle anchorRoot = r3SoftBody_RootBody(anchorJelly);
    R3RigidBodyHandle hangingRoot = r3SoftBody_RootBody(hangingJelly);
    {
        R3JointDesc joint = r3RopeJointDesc(2.2);
        r3InsertImpulseJoint(anchorRoot, hangingRoot, &joint);
    }
    /* Spring bungee under a gantry. */
    R3SoftBodyHandle bungee;
    {
        R3SoftBodyDesc builder = jelly(r3Vector(5.5, 3.2, 2.5), r3Vector(0.45, 0.45, 0.45), 6e3);
        builder.canSleep = !testbed->noSleep;
        bungee = r3InsertSoftBody(world, &builder);
    }
    R3RigidBodyHandle bungeeRoot = r3SoftBody_RootBody(bungee);
    R3RigidBodyHandle gantry;
    {
        R3RigidBodyDesc builder = r3FixedRigidBodyDesc();
        builder.position.translation = r3Vector(5.5, 5.5, 2.5);
        gantry = r3InsertRigidBody(world, &builder);
    }
    {
        R3JointDesc joint = r3SpringJointDesc(1.2, 25, 1.5);
        r3InsertImpulseJoint(gantry, bungeeRoot, &joint);
    }
    /* Bead on a visual-only pole. */
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(8.5, 2.5, -4);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CylinderColliderDesc(2.5, .05);
        collider.collisionGroups = (R3InteractionGroups){0, 0, 0};
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    R3SoftBodyHandle bead;
    {
        R3SoftBodyDesc builder = jelly(r3Vector(8.5, 4.2, -4), r3Vector(0.35, 0.35, 0.35), 8e3);
        builder.canSleep = !testbed->noSleep;
        bead = r3InsertSoftBody(world, &builder);
    }
    R3RigidBodyHandle beadRoot = r3SoftBody_RootBody(bead);
    R3RigidBodyHandle pole;
    {
        R3RigidBodyDesc builder = r3FixedRigidBodyDesc();
        builder.position.translation = r3Vector(8.5, 2.5, -4);
        pole = r3InsertRigidBody(world, &builder);
    }
    {
        R3JointDesc joint = r3DefaultJointDesc();
        joint.lockedAxes = 1 | 4;
        r3JointDesc_SetLocalAxis1(&joint, r3Vector(0, 1, 0));
        r3JointDesc_SetLocalAxis2(&joint, r3Vector(0, 1, 0));
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_Y, -1.8, 1.8);
        r3InsertImpulseJoint(pole, beadRoot, &joint);
    }
    /* Hinge two disjoint clusters of one soft bar. */
    R3SoftBodyHandle bar;
    R3SoftBodyDesc barBuilder =
        r3CuboidSoftBodyDesc(r3Vector(2.5, 2.2, 5.5), r3Vector(1, 0.22, 0.4), 7, 3, 3);
    barBuilder.cellModel = R3_SOFT_CELL_COROTATIONAL;
    barBuilder.particleMass = .08;
    barBuilder.particleRadius = (R3OptionalReal){1, .06};
    {
        R3SoftBodyMaterial material = r3DefaultSoftBodyMaterial();
        material.youngModulus = 2e4;
        material.poissonRatio = .35;
        material.elasticDampingRatio = 1;
        barBuilder.material = material;
    }
    {
        R3ColliderDesc surface = r3BallColliderDesc(.06);
        surface.friction = .5;
        barBuilder.collider = surface;
    }
    barBuilder.canSleep = !testbed->noSleep;
    bar = r3InsertSoftBody(world, &barBuilder);

    R3Vector barCom = r3SoftBody_CenterOfMass(bar);
    uint32_t leftHalf[64], rightHalf[64];
    size_t leftCount = 0, rightCount = 0;
    particleCount = r3SoftBody_ParticlePositions(bar, positions, TB_COUNT(positions));
    for (size_t i = 0; i < particleCount; ++i) {
        if (positions[i].x < barCom.x - 1e-3) {
            leftHalf[leftCount++] = i;
        } else if (positions[i].x > barCom.x + 1e-3) {
            rightHalf[rightCount++] = i;
        }
    }
    uint32_t leftCluster = r3SoftBody_AddCluster(bar, leftHalf, leftCount);
    uint32_t rightCluster = r3SoftBody_AddCluster(bar, rightHalf, rightCount);
    R3RigidBodyHandle leftProxy = r3SoftBody_ClusterProxy(bar, leftCluster);
    R3RigidBodyHandle rightProxy = r3SoftBody_ClusterProxy(bar, rightCluster);
    R3Vector leftPos;
    { leftPos = r3RigidBody_Translation(leftProxy); }
    R3Vector rightPos;
    { rightPos = r3RigidBody_Translation(rightProxy); }
    R3RigidBodyHandle barAnchor;
    {
        R3RigidBodyDesc builder = r3FixedRigidBodyDesc();
        builder.position.translation = leftPos;
        barAnchor = r3InsertRigidBody(world, &builder);
    }
    {
        R3JointDesc joint = r3FixedJointDesc();
        r3InsertImpulseJoint(barAnchor, leftProxy, &joint);
    }
    R3ImpulseJointHandle flapJoint;
    {
        R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
        joint.localFrame1.translation = r3VectorSub(barCom, leftPos);
        joint.localFrame2.translation = r3VectorSub(barCom, rightPos);
        r3JointDesc_SetMotorPosition(&joint, R3_AXIS_ANG_X, 0, 80, 10);
        flapJoint = r3InsertImpulseJoint(leftProxy, rightProxy, &joint);
    }
    /* Multibody arm and rope attached to one jelly. */
    R3RigidBodyHandle armRoot;
    {
        R3RigidBodyDesc builder = r3FixedRigidBodyDesc();
        builder.position.translation = r3Vector(7, 5, 6);
        armRoot = r3InsertRigidBody(world, &builder);
    }
    R3RigidBodyHandle link1;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(8.2, 5, 6);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CapsuleXColliderDesc(.5, .08);
        collider.density = 2;
        link1 = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(link1, &collider);
    }
    {
        R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
        joint.localFrame1.translation = r3Vector(0, 0, 0);
        joint.localFrame2.translation = r3Vector(-1.2, 0, 0);
        r3InsertMultibodyJoint(armRoot, link1, &joint);
    }
    R3RigidBodyHandle link2;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(9.4, 5, 6);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CapsuleXColliderDesc(.5, .08);
        collider.density = 2;
        link2 = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(link2, &collider);
    }
    {
        R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
        joint.localFrame1.translation = r3Vector(0.6, 0, 0);
        joint.localFrame2.translation = r3Vector(-0.6, 0, 0);
        r3InsertMultibodyJoint(link1, link2, &joint);
    }
    R3SoftBodyHandle pendulum;
    {
        R3SoftBodyDesc builder = jelly(r3Vector(10.2, 4.2, 6), r3Vector(0.5, 0.5, 0.5), 5e3);
        builder.canSleep = !testbed->noSleep;
        pendulum = r3InsertSoftBody(world, &builder);
    }
    R3RigidBodyHandle pendulumRoot = r3SoftBody_RootBody(pendulum);
    {
        R3JointDesc joint = r3SphericalJointDesc();
        joint.localFrame1.translation = r3Vector(0.7, 0, 0);
        joint.localFrame2.translation = r3Vector(0, 0.6, 0);
        r3InsertImpulseJoint(link2, pendulumRoot, &joint);
    }
    R3RigidBodyHandle crateBody;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(10.2, 0.3, 6);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.3, 0.3, 0.3));
        collider.density = .5;
        crateBody = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(crateBody, &collider);
    }
    {
        R3JointDesc joint = r3RopeJointDesc(3.2);
        r3InsertImpulseJoint(pendulumRoot, crateBody, &joint);
    }
    /* Wave a pinned cluster without a joint. */
    R3SoftBodyHandle banner;
    R3SoftBodyDesc gripBuilder = r3ClothSoftBodyDesc(r3Vector(-8, 3.2, 4), r3Vector(0.18, 0, 0),
                                                     r3Vector(0, -0.18, 0), 14, 10);
    gripBuilder.canSleep = !testbed->noSleep;
    banner = r3InsertSoftBody(world, &gripBuilder);

    uint32_t gripParticles[14];
    for (uint32_t i = 0; i < 14; ++i) {
        gripParticles[i] = i;
    }
    uint32_t grip = r3SoftBody_AddCluster(banner, gripParticles, 14);
    R3RigidBodyHandle gripProxy = r3SoftBody_ClusterProxy(banner, grip);
    r3SoftBody_SetClusterPinned(banner, grip, 1);
    R3Vector gripHome;
    { gripHome = r3RigidBody_Translation(gripProxy); }
    tbCamera(testbed, 13, 9, 15, 0, 1.5, 0);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);
    R3Real t = 0;
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            R3Real dt = r3TimeStep(world);
            t += dt;

            r3RigidBody_SetNextKinematicTranslation(
                mover,
                r3VectorAdd(cornerPos, r3Vector(1.2 * sin(.8 * t), .4 * sin(1.3 * t),
                                                1.2 * cos(.8 * t) - 1.2)));

            r3ImpulseJoint_SetMotorPosition(railJoint, R3_AXIS_LIN_X, 1.5 * sin(.6 * t), 40,
                                           8, 1);

            r3ImpulseJoint_SetMotorPosition(flapJoint, R3_AXIS_ANG_Z, .8 * sin(1.4 * t), 80,
                                           10, 1);

            r3SoftBody_SetClusterKinematicTarget(
                banner, grip,
                r3Pose(r3VectorAdd(gripHome, r3Vector(1.5 * sin(.7 * t), .2 * sin(1.9 * t), 0)),
                       r3RotationFromAxisAngle(r3Vector(1, 0, 0), .35 * sin(1.1 * t))));
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
