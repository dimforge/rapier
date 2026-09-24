/** @file
 * Optional C++ RAII wrappers.
 * @ingroup cpp
 */
#ifndef RAPIER_HPP
#define RAPIER_HPP
#include "rapier_helpers.h"
#include <memory>
#include <stdexcept>
#include <string>

/** Optional C++ ownership helpers. @ingroup cpp */
namespace rapier {
/** Throw std::runtime_error with LastError when status is not OK. */
inline void check(RAPIER_TYPE(Status) status) {
    if (status != RAPIER_CONST(OK)) {
        throw std::runtime_error(std::string(RAPIER_FN(LastError)()));
    }
}

// Use Owner only for owned pointers, never for a borrowed callback context.
/** Deleter for an owned pointer; Free must succeed at destruction time. */
template <class T, RAPIER_TYPE(Status) (*Free)(T *)> struct Deleter {
    /** Release the pointer through its matching C API function. */
    void operator()(T *value) const noexcept {
        (void)Free(value);
    }
};
/** Unique owner of a native pointer; never wrap a borrowed pointer. */
template <class T, RAPIER_TYPE(Status) (*Free)(T *)>
using Owner = std::unique_ptr<T, Deleter<T, Free>>;
/** Own a native World using its matching Free function. */
using World = Owner<RAPIER_TYPE(World), RAPIER_FN(FreeWorld)>;
/** Own a native Shape using its matching Free function. */
using Shape = Owner<RAPIER_TYPE(SharedShape), RAPIER_FN(FreeSharedShape)>;
/** Own a native EventCollector using its matching Free function. */
using EventCollector = Owner<RAPIER_TYPE(EventCollector), RAPIER_FN(FreeEventCollector)>;
/** Own a native Snapshot using its matching Free function. */
using Snapshot = Owner<RAPIER_TYPE(Bytes), RAPIER_FN(FreeBytes)>;

/** Own a native SoftBodyTearEvent using its matching Free function. */
using SoftBodyTearEvent = Owner<RAPIER_TYPE(SoftBodyTearEvent), RAPIER_FN(FreeSoftBodyTearEvent)>;
/** Own a native ShapeMesh using its matching Free function. */
using ShapeMesh = Owner<RAPIER_TYPE(ShapeMesh), RAPIER_FN(FreeShapeMesh)>;
/** Own a native KinematicCharacterController using its matching Free function. */
using KinematicCharacterController =
    Owner<RAPIER_TYPE(KinematicCharacterController), RAPIER_FN(FreeKinematicCharacterController)>;
/** Own a native PidController using its matching Free function. */
using PidController = Owner<RAPIER_TYPE(PidController), RAPIER_FN(FreePidController)>;
#if defined(RAPIER_DIM3)
/** Own a native DynamicRayCastVehicleController using its matching Free function. */
using DynamicRayCastVehicleController =
    Owner<RAPIER_TYPE(DynamicRayCastVehicleController),
          RAPIER_FN(FreeDynamicRayCastVehicleController)>;
/** Own a native TriMeshData using its matching Free function. */
using TriMeshData = Owner<RAPIER_TYPE(TriMeshData), RAPIER_FN(FreeTriMeshData)>;
#endif
#if defined(RAPIER_ROBOTICS) && defined(RAPIER_DIM3) && defined(RAPIER_F32)
/** Own a native UrdfRobot using its matching Free function. */
using UrdfRobot = Owner<RAPIER_TYPE(UrdfRobot), RAPIER_FN(FreeUrdfRobot)>;
/** Own a native UrdfRobotHandles using its matching Free function. */
using UrdfRobotHandles = Owner<RAPIER_TYPE(UrdfRobotHandles), RAPIER_FN(FreeUrdfRobotHandles)>;
/** Own a native MjcfRobot using its matching Free function. */
using MjcfRobot = Owner<RAPIER_TYPE(MjcfRobot), RAPIER_FN(FreeMjcfRobot)>;
/** Own a native MjcfRobotHandles using its matching Free function. */
using MjcfRobotHandles = Owner<RAPIER_TYPE(MjcfRobotHandles), RAPIER_FN(FreeMjcfRobotHandles)>;
#endif

// These factories return ordinary values. No allocation or deleter is needed.
/** Return a POD rigid-body description with the selected body type. */
inline RAPIER_TYPE(RigidBodyDesc) rigid_body(uint32_t kind = RAPIER_CONST(DYNAMIC)) {
    RAPIER_TYPE(RigidBodyDesc) value = RAPIER_FN(DynamicRigidBodyDesc)();
    value.bodyType = kind;
    return value;
}

/** Return a POD ball collider description. */
inline RAPIER_TYPE(ColliderDesc) ball(RAPIER_TYPE(Real) radius) {
    return RAPIER_FN(BallColliderDesc)(radius);
}

/** Return a POD cuboid collider description. */
inline RAPIER_TYPE(ColliderDesc) cuboid(RAPIER_TYPE(Vector) half_extents) {
    return RAPIER_FN(CuboidColliderDesc)(half_extents);
}

/** Return native soft-body description defaults. */
inline RAPIER_TYPE(SoftBodyDesc) soft_body() {
    return RAPIER_FN(DefaultSoftBodyDesc)();
}

/** Return a joint description with the selected locked-axis mask. */
inline RAPIER_TYPE(JointDesc) joint(uint8_t locked_axes = 0) {
    RAPIER_TYPE(JointDesc) value = RAPIER_FN(DefaultJointDesc)();
    value.lockedAxes = locked_axes;
    return value;
}

/** Return query defaults with no callback or exclusions. */
inline RAPIER_TYPE(QueryOptions) queryOptions() {
    return RAPIER_FN(DefaultQueryOptions)();
}

/** Check the ABI and create an owned world; throw on failure. */
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
