/** @file
 * Inline value constructors and arithmetic; no allocation or error-state changes.
 * @defgroup inline_math Inline math
 * @ingroup math
 * @{
 */
#ifndef RAPIER_MATH_H
#define RAPIER_MATH_H
#include "rapier.h"
#include <math.h>

#if defined(RAPIER_DIM2)
/** Pi in the selected scalar precision. */
#define R2_PI ((R2Real)3.14159265358979323846)
#else
/** Pi in the selected scalar precision. */
#define R3_PI ((R3Real)3.14159265358979323846)
#endif

/* Value constructors and arithmetic for Rapier's public C math types. */
#if defined(RAPIER_DIM2)
/** Construct a vector from its components. */
static inline RAPIER_TYPE(Vector) RAPIER_FN(Vector)(RAPIER_TYPE(Real) x, RAPIER_TYPE(Real) y) {
  RAPIER_TYPE(Vector) result = {x, y};
  return result;
}

/** Construct a 2D rotation from an angle in radians. */
static inline RAPIER_TYPE(Rotation) RAPIER_FN(Rotation)(RAPIER_TYPE(Real) angle) {
  RAPIER_TYPE(Rotation) result = {angle};
  return result;
}
#else
/** Construct a vector from its components. */
static inline RAPIER_TYPE(Vector) RAPIER_FN(Vector)(RAPIER_TYPE(Real) x, RAPIER_TYPE(Real) y, RAPIER_TYPE(Real) z) {
  RAPIER_TYPE(Vector) result = {x, y, z};
  return result;
}

/** Construct a 3D unit quaternion; normalizes axis, and returns identity for a zero axis. Angle is in radians. */
static inline RAPIER_TYPE(Rotation) RAPIER_FN(RotationFromAxisAngle)(RAPIER_TYPE(Vector) axis,
                                                        RAPIER_TYPE(Real) angle) {
  RAPIER_TYPE(Real) length =
      (RAPIER_TYPE(Real))sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  if (length == 0) {
    RAPIER_TYPE(Rotation) result = {0, 0, 0, 1};
    return result;
  }
  RAPIER_TYPE(Real) scale = (RAPIER_TYPE(Real))sin(angle / 2) / length;
  RAPIER_TYPE(Rotation) result = {axis.x * scale, axis.y * scale, axis.z * scale,
                        (RAPIER_TYPE(Real))cos(angle / 2)};
  return result;
}
#endif

/** Return a + b. */
static inline RAPIER_TYPE(Vector) RAPIER_FN(VectorAdd)(RAPIER_TYPE(Vector) a, RAPIER_TYPE(Vector) b) {
#if defined(RAPIER_DIM2)
  return RAPIER_FN(Vector)(a.x + b.x, a.y + b.y);
#else
  return RAPIER_FN(Vector)(a.x + b.x, a.y + b.y, a.z + b.z);
#endif
}

/** Return a - b. */
static inline RAPIER_TYPE(Vector) RAPIER_FN(VectorSub)(RAPIER_TYPE(Vector) a, RAPIER_TYPE(Vector) b) {
#if defined(RAPIER_DIM2)
  return RAPIER_FN(Vector)(a.x - b.x, a.y - b.y);
#else
  return RAPIER_FN(Vector)(a.x - b.x, a.y - b.y, a.z - b.z);
#endif
}

/** Multiply each component by scale. */
static inline RAPIER_TYPE(Vector) RAPIER_FN(VectorScale)(RAPIER_TYPE(Vector) vector, RAPIER_TYPE(Real) scale) {
#if defined(RAPIER_DIM2)
  return RAPIER_FN(Vector)(vector.x * scale, vector.y * scale);
#else
  return RAPIER_FN(Vector)(vector.x * scale, vector.y * scale, vector.z * scale);
#endif
}

/** Return the dot product. */
static inline RAPIER_TYPE(Real) RAPIER_FN(VectorDot)(RAPIER_TYPE(Vector) a, RAPIER_TYPE(Vector) b) {
#if defined(RAPIER_DIM2)
  return a.x * b.x + a.y * b.y;
#else
  return a.x * b.x + a.y * b.y + a.z * b.z;
#endif
}
/** Return the Euclidean length. */
static inline RAPIER_TYPE(Real) RAPIER_FN(VectorLength)(RAPIER_TYPE(Vector) vector) {
  return (RAPIER_TYPE(Real))sqrt(RAPIER_FN(VectorDot)(vector, vector));
}
/** Normalize a nonzero vector; a zero vector is returned unchanged. */
static inline RAPIER_TYPE(Vector) RAPIER_FN(VectorNormalize)(RAPIER_TYPE(Vector) vector) {
  RAPIER_TYPE(Real) length = RAPIER_FN(VectorLength)(vector);
  return length > 0 ? RAPIER_FN(VectorScale)(vector, 1 / length) : vector;
}
#if defined(RAPIER_DIM3)
/** Return the 3D cross product a x b. */
static inline RAPIER_TYPE(Vector) RAPIER_FN(VectorCross)(RAPIER_TYPE(Vector) a, RAPIER_TYPE(Vector) b) {
  return RAPIER_FN(Vector)(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                        a.x * b.y - a.y * b.x);
}
#endif

/** Compose rotations, applying b then a. Inputs must be normalized. */
static inline RAPIER_TYPE(Rotation) RAPIER_FN(RotationMul)(RAPIER_TYPE(Rotation) a, RAPIER_TYPE(Rotation) b) {
#if defined(RAPIER_DIM2)
  RAPIER_TYPE(Rotation) result = {a.angle + b.angle};
  return result;
#else
  RAPIER_TYPE(Rotation) result = {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
  return result;
#endif
}

/* Rotate a vector without changing its length. The rotation must be normalized.
 */
/** Rotate a vector; rotation must be normalized. */
static inline RAPIER_TYPE(Vector) RAPIER_FN(RotationTransformVector)(RAPIER_TYPE(Rotation) rotation,
                                                        RAPIER_TYPE(Vector) vector) {
#if defined(RAPIER_DIM2)
  const RAPIER_TYPE(Real) c = (RAPIER_TYPE(Real))cos(rotation.angle),
                s = (RAPIER_TYPE(Real))sin(rotation.angle);
  return RAPIER_FN(Vector)(c * vector.x - s * vector.y,
                        s * vector.x + c * vector.y);
#else
  const RAPIER_TYPE(Vector) t = {2 * (rotation.y * vector.z - rotation.z * vector.y),
                       2 * (rotation.z * vector.x - rotation.x * vector.z),
                       2 * (rotation.x * vector.y - rotation.y * vector.x)};
  return RAPIER_FN(Vector)(
      vector.x + rotation.w * t.x + rotation.y * t.z - rotation.z * t.y,
      vector.y + rotation.w * t.y + rotation.z * t.x - rotation.x * t.z,
      vector.z + rotation.w * t.z + rotation.x * t.y - rotation.y * t.x);
#endif
}

/** Construct a pose from translation and rotation without validation. */
static inline RAPIER_TYPE(Pose) RAPIER_FN(Pose)(RAPIER_TYPE(Vector) translation,
                                   RAPIER_TYPE(Rotation) rotation) {
  RAPIER_TYPE(Pose) result = {translation, rotation};
  return result;
}

/** Construct a pose with the supplied translation and identity rotation. */
static inline RAPIER_TYPE(Pose) RAPIER_FN(TranslationPose)(RAPIER_TYPE(Vector) translation) {
#if defined(RAPIER_DIM2)
  RAPIER_TYPE(Rotation) rotation = {0};
#else
  RAPIER_TYPE(Rotation) rotation = {0, 0, 0, 1};
#endif
  return RAPIER_FN(Pose)(translation, rotation);
}
/** Return the inverse of a normalized rotation. */
static inline RAPIER_TYPE(Rotation) RAPIER_FN(RotationInverse)(RAPIER_TYPE(Rotation) rotation) {
#if defined(RAPIER_DIM2)
  return RAPIER_FN(Rotation)(-rotation.angle);
#else
  RAPIER_TYPE(Rotation) result = {-rotation.x, -rotation.y, -rotation.z, rotation.w};
  return result;
#endif
}
/** Transform a point by rotation then translation. Rotation must be normalized. */
static inline RAPIER_TYPE(Vector) RAPIER_FN(PoseTransformPoint)(RAPIER_TYPE(Pose) pose,
                                                   RAPIER_TYPE(Vector) point) {
  return RAPIER_FN(VectorAdd)(
      pose.translation, RAPIER_FN(RotationTransformVector)(pose.rotation, point));
}
/** Return the inverse rigid transform. Rotation must be normalized. */
static inline RAPIER_TYPE(Pose) RAPIER_FN(PoseInverse)(RAPIER_TYPE(Pose) pose) {
  RAPIER_TYPE(Rotation) rotation = RAPIER_FN(RotationInverse)(pose.rotation);
  return RAPIER_FN(Pose)(RAPIER_FN(RotationTransformVector)(
                          rotation, RAPIER_FN(VectorScale)(pose.translation, -1)),
                      rotation);
}
#endif

/** @} */
