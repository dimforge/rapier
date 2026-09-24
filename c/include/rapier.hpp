#ifndef RAPIER_HPP
#define RAPIER_HPP
#include "rapier_helpers.h"
#include <memory>
#include <stdexcept>
#include <string>

namespace rapier {
inline void check(RAPIER_TYPE(Status) status) {
    if (status != RAPIER_CONST(OK)) {
        throw std::runtime_error(std::string(RAPIER_FN(LastError)()));
    }
}

// Use Owner only for owned pointers, never for a borrowed callback context.
template <class T, RAPIER_TYPE(Status) (*Free)(T *)> struct Deleter {
    void operator()(T *value) const noexcept {
        (void)Free(value);
    }
};
template <class T, RAPIER_TYPE(Status) (*Free)(T *)>
using Owner = std::unique_ptr<T, Deleter<T, Free>>;
using World = Owner<RAPIER_TYPE(World), RAPIER_FN(FreeWorld)>;
using Shape = Owner<RAPIER_TYPE(SharedShape), RAPIER_FN(FreeSharedShape)>;
using EventCollector = Owner<RAPIER_TYPE(EventCollector), RAPIER_FN(FreeEventCollector)>;
using Snapshot = Owner<RAPIER_TYPE(Bytes), RAPIER_FN(FreeBytes)>;

using SoftBodyTearEvent = Owner<RAPIER_TYPE(SoftBodyTearEvent), RAPIER_FN(FreeSoftBodyTearEvent)>;
using ShapeMesh = Owner<RAPIER_TYPE(ShapeMesh), RAPIER_FN(FreeShapeMesh)>;
using KinematicCharacterController =
    Owner<RAPIER_TYPE(KinematicCharacterController), RAPIER_FN(FreeKinematicCharacterController)>;
using PidController = Owner<RAPIER_TYPE(PidController), RAPIER_FN(FreePidController)>;
#if defined(RAPIER_DIM3)
using DynamicRayCastVehicleController =
    Owner<RAPIER_TYPE(DynamicRayCastVehicleController),
          RAPIER_FN(FreeDynamicRayCastVehicleController)>;
using TriMeshData = Owner<RAPIER_TYPE(TriMeshData), RAPIER_FN(FreeTriMeshData)>;
#endif
#if defined(RAPIER_ROBOTICS) && defined(RAPIER_DIM3) && defined(RAPIER_F32)
using UrdfRobot = Owner<RAPIER_TYPE(UrdfRobot), RAPIER_FN(FreeUrdfRobot)>;
using UrdfRobotHandles = Owner<RAPIER_TYPE(UrdfRobotHandles), RAPIER_FN(FreeUrdfRobotHandles)>;
using MjcfRobot = Owner<RAPIER_TYPE(MjcfRobot), RAPIER_FN(FreeMjcfRobot)>;
using MjcfRobotHandles = Owner<RAPIER_TYPE(MjcfRobotHandles), RAPIER_FN(FreeMjcfRobotHandles)>;
#endif

// These factories return ordinary values. No allocation or deleter is needed.
inline RAPIER_TYPE(RigidBodyDesc) rigid_body(uint32_t kind = RAPIER_CONST(DYNAMIC)) {
    RAPIER_TYPE(RigidBodyDesc) value = RAPIER_FN(DynamicRigidBodyDesc)();
    value.bodyType = kind;
    return value;
}

inline RAPIER_TYPE(ColliderDesc) ball(RAPIER_TYPE(Real) radius) {
    return RAPIER_FN(BallColliderDesc)(radius);
}

inline RAPIER_TYPE(ColliderDesc) cuboid(RAPIER_TYPE(Vector) half_extents) {
    return RAPIER_FN(CuboidColliderDesc)(half_extents);
}

inline RAPIER_TYPE(SoftBodyDesc) soft_body() {
    return RAPIER_FN(DefaultSoftBodyDesc)();
}

inline RAPIER_TYPE(JointDesc) joint(uint8_t locked_axes = 0) {
    RAPIER_TYPE(JointDesc) value = RAPIER_FN(DefaultJointDesc)();
    value.lockedAxes = locked_axes;
    return value;
}

inline RAPIER_TYPE(QueryOptions) queryOptions() {
    return RAPIER_FN(DefaultQueryOptions)();
}

inline World make_world() {
    check(RAPIER_FN(CheckAbi)(RAPIER_CONST(ABI_VERSION), RAPIER_CONST(DIMENSION),
                              sizeof(RAPIER_TYPE(Real)), sizeof(RAPIER_TYPE(Vector)),
                              sizeof(RAPIER_TYPE(Pose))));
    RAPIER_TYPE(World) *value = RAPIER_FN(NewWorld)();
    check(RAPIER_FN(LastStatus)());
    return World(value);
}
} // namespace rapier
#endif
