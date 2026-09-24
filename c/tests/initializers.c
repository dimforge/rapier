#include "rapier_helpers.h"
#include <assert.h>

int main(void) {
    RAPIER_TYPE(RigidBodyDesc)
    bodies[] = {RAPIER_FN(DynamicRigidBodyDesc)(), RAPIER_FN(FixedRigidBodyDesc)(),
                RAPIER_FN(KinematicPositionBasedRigidBodyDesc)(),
                RAPIER_FN(KinematicVelocityBasedRigidBodyDesc)()};
    RAPIER_TYPE(World) *world = RAPIER_FN(NewWorld)();
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
    RAPIER_TYPE(ColliderDesc) collider = RAPIER_FN(DefaultColliderDesc)();
    RAPIER_TYPE(RigidBodyHandle) handle = RAPIER_CONST(INVALID_RIGID_BODY_HANDLE);
    assert(handle.index == UINT32_MAX && handle.generation == UINT32_MAX);
    for (unsigned i = 0; i < 4; ++i) {
        assert(bodies[i].bodyType == i);
        bodies[i].position.translation.x = 2 * i;
        bodies[i].position.translation.y = 5;
        handle = RAPIER_FN(InsertRigidBody)(world, &bodies[i]);
        RAPIER_FN(InsertCollider)(handle, &collider);
        assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
    }
    RAPIER_TYPE(ShapeDesc) shape = RAPIER_FN(DefaultShapeDesc)();
    assert(shape.kind == RAPIER_CONST(SHAPE_DESC_BALL) && !shape.vertices.data);
    RAPIER_TYPE(SoftBodyDesc) soft = RAPIER_FN(DefaultSoftBodyDesc)();
    RAPIER_TYPE(SoftBodyMaterial) material = RAPIER_FN(DefaultSoftBodyMaterial)();
    soft.material = material;
    soft.kind = RAPIER_CONST(SOFT_DESC_ROPE);
    RAPIER_FN(InsertSoftBody)(world, &soft);
    assert(RAPIER_FN(LastStatus)() == RAPIER_CONST(OK));
    RAPIER_TYPE(JointDesc) joint = RAPIER_FN(DefaultJointDesc)();
    assert(joint.enabled && !joint.lockedAxes);
    RAPIER_TYPE(SoftMeshBindingDesc) binding = RAPIER_FN(DefaultSoftMeshBindingDesc)();
    assert(binding.kind == RAPIER_CONST(SOFT_BINDING_SKINNED) && !binding.particles.data);
    RAPIER_TYPE(IntegrationParameters) settings = RAPIER_FN(DefaultIntegrationParameters)();
    settings.softBodies = RAPIER_FN(DefaultSoftBodiesSettings)();
    settings.softBodies.recovery = RAPIER_FN(DefaultSoftRecoverySettings)();
#ifdef RAPIER_FEM
    settings.softBodies.fem = RAPIER_FN(DefaultSoftFemParameters)();
#endif

    assert(RAPIER_FN(SetIntegrationParameters)(world, &settings) == RAPIER_CONST(OK));
    assert(RAPIER_FN(Step)(world, NULL, NULL) == RAPIER_CONST(OK));
    RAPIER_TYPE(QueryFilter) filter = RAPIER_FN(DefaultQueryFilter)();
    assert(filter.exclude_collider.index == UINT32_MAX &&
           filter.exclude_rigid_body.index == UINT32_MAX);
    RAPIER_TYPE(QueryOptions) query = RAPIER_FN(DefaultQueryOptions)();
    query.filter = *(&filter);
    RAPIER_TYPE(ShapeCastOptions) options = RAPIER_FN(DefaultShapeCastOptions)();
    assert(options.max_time_of_impact > 0);
    assert(RAPIER_CONST(INVALID_COLLIDER_HANDLE).index == UINT32_MAX);
    assert(RAPIER_CONST(INVALID_IMPULSE_JOINT_HANDLE).index == UINT32_MAX);
    assert(RAPIER_CONST(INVALID_MULTIBODY_JOINT_HANDLE).index == UINT32_MAX);
    assert(RAPIER_CONST(INVALID_SOFT_BODY_HANDLE).index == UINT32_MAX);
    assert(RAPIER_FN(FreeWorld)(world) == RAPIER_CONST(OK));
    return 0;
}
