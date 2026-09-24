#ifndef RAPIER_HELPERS_H
#define RAPIER_HELPERS_H
#include "rapier.h"

/* Value initialization functions are exported by rapier.h. */

/* Explicit invalid values; zero-initialized Rapier handles are not invalid.
 * These constants have internal linkage and own no resources. */
static const RAPIER_TYPE(RigidBodyHandle) RAPIER_CONST(INVALID_RIGID_BODY_HANDLE) = {NULL, UINT32_MAX, UINT32_MAX};
static const RAPIER_TYPE(ColliderHandle) RAPIER_CONST(INVALID_COLLIDER_HANDLE) = {NULL, UINT32_MAX, UINT32_MAX};
static const RAPIER_TYPE(ImpulseJointHandle) RAPIER_CONST(INVALID_IMPULSE_JOINT_HANDLE) = {NULL, UINT32_MAX, UINT32_MAX};
static const RAPIER_TYPE(MultibodyJointHandle) RAPIER_CONST(INVALID_MULTIBODY_JOINT_HANDLE) = {NULL, UINT32_MAX, UINT32_MAX};
static const RAPIER_TYPE(SoftBodyHandle) RAPIER_CONST(INVALID_SOFT_BODY_HANDLE) = {NULL, UINT32_MAX, UINT32_MAX};

#endif
