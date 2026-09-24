/* Port of examples2d/b2d_spinner.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB2dSpinner(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, -10));
    R2RigidBodyHandle ground;
    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, 0);
    groundBody.canSleep = !testbed->noSleep;
    ground = r2InsertRigidBody(world, &groundBody);

    R2Vector points[360];
    R2Vector position = r2Vector(40, 0);
    uint32_t indices[720];
    R2Real angle = -2 * R2_PI / 360;
    for (uint32_t i = 0; i < 360; i++) {
        points[i] = r2VectorAdd(position, r2Vector(0, 32));
        position = r2Vector(cos(angle) * position.x - sin(angle) * position.y,
                            sin(angle) * position.x + cos(angle) * position.y);
        indices[2 * i] = i;
        indices[2 * i + 1] = (i + 1) % 360;
    }
    R2SharedShape *shape = r2OrientedPolylineSharedShape(
        (R2VectorView){points, 360}, (R2EdgeView){(const R2Edge *)indices, 360});
    R2ColliderDesc wall = r2DefaultColliderDesc();
    wall.shape.kind = R2_SHAPE_DESC_SHARED;
    wall.shape.sharedShape = shape;
    wall.friction = 0.1;
    r2InsertCollider(ground, &wall);
    R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 12);
    rigidBody.canSleep = 0;
    R2ColliderDesc collider = r2RoundCuboidColliderDesc(r2Vector(0.4, 20), 0.2);
    collider.friction = 0;
    R2RigidBodyHandle spinner;
    if (testbed->noSleep) {
        rigidBody.canSleep = 0;
        rigidBody.sleeping = 0;
    }
    spinner = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(spinner, &collider);
    R2JointDesc joint = r2RevoluteJointDesc();
    joint.localFrame1.translation = r2Vector(0, 12);
    joint.localFrame2.translation = r2Vector(0, 0);
    r2JointDesc_SetMotor(&joint, R2_AXIS_ANG_X, 0, 5, 0, 1.0e5);
    r2JointDesc_SetMotorMaxForce(&joint, R2_AXIS_ANG_X, 1.0e9);
    R2ImpulseJointHandle jointHandle = r2InsertImpulseJoint(ground, spinner, &joint);
    R2Real x = -23;
    R2Real y = 2;
    for (int i = 0; i < 6076; i++) {
        R2ColliderDesc particleCollider;
        if (i % 3 == 0) {
            particleCollider =
                r2CapsuleColliderDesc(r2Vector(-0.25, 0.0), r2Vector(0.25, 0.0), 0.25);
        } else if (i % 3 == 1) {
            particleCollider = r2BallColliderDesc(0.35);
        } else {
            particleCollider = r2CuboidColliderDesc(r2Vector(0.35, 0.35));
        }
        particleCollider.density = 0.25;
        particleCollider.friction = 0.1;
        particleCollider.restitution = 0.1;
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(x, y);
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &particleCollider);

        x += 0.5;
        if (x >= 23) {
            x = -23;
            y += 0.5;
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 32, 6);

    r2FreeSharedShape(shape);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
