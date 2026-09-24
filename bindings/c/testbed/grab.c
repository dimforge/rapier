#include "grab.h"
#include "rapier_math.h"
#include "rapier_helpers.h"
#include <float.h>

#define TRY(call)                                                                                  \
    do {                                                                                           \
        status = (call);                                                                           \
        if (status != RAPIER_CONST(OK))                                                            \
            goto done;                                                                             \
    } while (0)

static bool contains(Testbed *t, RAPIER_TYPE(RigidBodyHandle) handle) {
    if (handle.world != t->world) return false;
    RAPIER_TYPE(Bool) found = RAPIER_FN(RigidBody_Contains)(handle);
    return RAPIER_FN(LastStatus)() == RAPIER_CONST(OK) && found;
}

static RAPIER_TYPE(RigidBodyHandle) pulledBody(Testbed *t, const TbGrab *grab) {
    if (grab->joint.world != t->world) return RAPIER_CONST(INVALID_RIGID_BODY_HANDLE);
    RAPIER_TYPE(RigidBodyHandle) second =
        RAPIER_FN(ImpulseJoint_Bodies)(grab->joint).body2;
    if (RAPIER_FN(LastStatus)() == RAPIER_CONST(OK)) {
        return second; /* Tearing may transfer the joint to another proxy. */
    }
    return grab->body;
}

RAPIER_TYPE(Status) tbGrabRelease(Testbed *t, TbGrab *grab) {
    if (grab->active && grab->body.world != t->world) {
        *grab = (TbGrab){0}; /* The old world was replaced, so its handles are unusable. */
    }
    if (!grab->active) {
        return RAPIER_CONST(OK);
    }
    RAPIER_TYPE(Status) status = RAPIER_CONST(OK);
    RAPIER_TYPE(RigidBodyHandle) pulled = pulledBody(t, grab);
    grab->active = false;
    if (contains(t, pulled)) {
        TRY(RAPIER_FN(RigidBody_WakeUp)(pulled, 1));
    }
    if (contains(t, grab->mouseBody)) { /* A scene may have removed it already. */
        TRY(RAPIER_FN(RemoveRigidBody)(grab->mouseBody, 1));
    }
    if (grab->soft) {
        RAPIER_TYPE(RigidBodyHandle) proxies[] = {grab->body, pulled};
        for (size_t i = 0; i < TB_COUNT(proxies); ++i) {
            if (!contains(t, proxies[i])) {
                continue;
            }

            RAPIER_TYPE(Bool) isFrame;
            TRY(RAPIER_FN(RigidBody_ValidateHandle)(proxies[i]));
            isFrame = RAPIER_FN(RigidBody_IsSoftFrame)(proxies[i]);
            TRY(RAPIER_FN(LastStatus)());
            if (isFrame) {
                /* A soft-body root cannot be removed (a tear may have made the grab cluster one). */
                RAPIER_TYPE(SoftBodyHandle) soft = RAPIER_FN(RigidBody_SoftBody)(proxies[i]);
                TRY(RAPIER_FN(LastStatus)());
                RAPIER_TYPE(RigidBodyHandle) root = RAPIER_FN(SoftBody_RootBody)(soft);
                TRY(RAPIER_FN(LastStatus)());
                if (root.index == proxies[i].index && root.generation == proxies[i].generation) {
                    continue;
                }
                TRY(RAPIER_FN(RemoveRigidBody)(proxies[i], 1));
            }
        }
    }
done:
    return status;
}

static RAPIER_TYPE(Status) pick(Testbed *t, RAPIER_TYPE(Real) radius,
                                RAPIER_TYPE(RigidBodyHandle) *body, RAPIER_TYPE(Vector) *point,
                                bool *found) {
    RAPIER_TYPE(Status) status = RAPIER_CONST(OK);
    RAPIER_TYPE(QueryOptions) query;
    RAPIER_TYPE(QueryFilter) filter;
#if defined(RAPIER_DIM2)
    RAPIER_TYPE(ColliderHandle) *hits = NULL;
#endif
    RAPIER_TYPE(ColliderHandle) collider;
    *found = false;
    filter = RAPIER_FN(DefaultQueryFilter)();
    filter.flags = RAPIER_CONST(QUERY_EXCLUDE_FIXED) | RAPIER_CONST(QUERY_EXCLUDE_KINEMATIC) |
                   RAPIER_CONST(QUERY_EXCLUDE_SENSORS);
    query = RAPIER_FN(DefaultQueryOptions)();
    query.filter = filter;
#if defined(RAPIER_DIM2)
    if (!t->cursorValid) {
        goto done;
    }
    /* Prefer a solid interior, but ignore the outward side of an open rope. */
    size_t count = RAPIER_FN(IntersectPoint)(t->world, &query, t->cursor, NULL, 0);
    TRY(RAPIER_FN(LastStatus)());
    if (count) {
        hits = malloc(count * sizeof(*hits));
        if (!hits) {
            abort();
        }
        count = RAPIER_FN(IntersectPoint)(t->world, &query, t->cursor, hits, count);
        TRY(RAPIER_FN(LastStatus)());
        for (size_t i = 0; i < count; ++i) {
            RAPIER_TYPE(SoftBodyHandle) soft;
            TRY(RAPIER_FN(Collider_ValidateHandle)(hits[i]));
            *body = RAPIER_FN(Collider_Parent)(hits[i]);
            TRY(RAPIER_FN(LastStatus)());
            if (body->index == UINT32_MAX) {
                continue;
            }
            TRY(RAPIER_FN(RigidBody_ValidateHandle)(*body));
            soft = RAPIER_FN(RigidBody_SoftBody)(*body);
            TRY(RAPIER_FN(LastStatus)());
            if (soft.index != UINT32_MAX) {
                RAPIER_TYPE(Bool) closed;
                TRY(RAPIER_FN(SoftBody_ValidateHandle)(soft));
                closed = RAPIER_FN(SoftBody_MeshIsClosed)(soft, hits[i]);
                TRY(RAPIER_FN(LastStatus)());
                if (!closed) {
                    continue;
                }
            }
            *point = t->cursor;
            *found = true;
            goto done;
        }
    }
    /* Thin surfaces can also be picked within eight screen pixels. Project onto
     * the boundary so an open polyline's nominal inside half-plane is not a hit. */
    RAPIER_TYPE(PointProjection)
    hit = RAPIER_FN(ProjectPoint)(t->world, &query, t->cursor, radius, 0);
    status = RAPIER_FN(LastStatus)();
    if (status == RAPIER_CONST(NOT_FOUND)) {
        status = RAPIER_CONST(OK);
        goto done;
    }
    if (status != RAPIER_CONST(OK)) {
        goto done;
    }
    collider = hit.collider;
    *point = hit.point;
#else
    (void)radius;
    if (!t->rayValid) {
        goto done;
    }
    RAPIER_TYPE(Real) toi;
    RAPIER_TYPE(Bool) hit;
    RAPIER_TYPE(RayToi)
    castRayToiResult2 =
        RAPIER_FN(CastRayToi)(t->world, &query, t->rayOrigin, t->rayDirection, FLT_MAX, 1);
    collider = castRayToiResult2.collider;
    toi = castRayToiResult2.toi;
    hit = castRayToiResult2.found;
    TRY(RAPIER_FN(LastStatus)());
    if (!hit) {
        goto done;
    }
    *point = RAPIER_FN(VectorAdd)(t->rayOrigin, RAPIER_FN(VectorScale)(t->rayDirection, toi));
#endif

    TRY(RAPIER_FN(Collider_ValidateHandle)(collider));
    *body = RAPIER_FN(Collider_Parent)(collider);
    TRY(RAPIER_FN(LastStatus)());
    *found = body->index != UINT32_MAX;
done:
#if defined(RAPIER_DIM2)
    free(hits);
#endif
    return status;
}

RAPIER_TYPE(Status) tbGrabBegin(Testbed *t, TbGrab *grab, RAPIER_TYPE(Real) pickRadius) {
    if (grab->active) {
        return RAPIER_CONST(OK);
    }
    RAPIER_TYPE(Status) status = RAPIER_CONST(OK);
    RAPIER_TYPE(RigidBodyDesc) builder;
    RAPIER_TYPE(JointDesc) joint;
    RAPIER_TYPE(Vector) point, localAnchor = V(0, 0, 0);
    RAPIER_TYPE(RigidBodyHandle) body;
    bool found;
    TRY(pick(t, pickRadius, &body, &point, &found));
    if (!found) {
        goto done;
    }

    RAPIER_TYPE(Bool) dynamic;
    RAPIER_TYPE(SoftBodyHandle) soft;
    TRY(RAPIER_FN(RigidBody_ValidateHandle)(body));
    dynamic = RAPIER_FN(RigidBody_IsDynamic)(body);
    TRY(RAPIER_FN(LastStatus)());
    if (!dynamic) {
        goto done;
    }
    soft = RAPIER_FN(RigidBody_SoftBody)(body);
    TRY(RAPIER_FN(LastStatus)());
    *grab = (TbGrab){.body = body,
                     .mouseBody = {NULL, UINT32_MAX, UINT32_MAX},
                     .joint = {NULL, UINT32_MAX, UINT32_MAX},
                     .soft = soft.index != UINT32_MAX};
    if (grab->soft) {
        size_t count;
        uint32_t nearest = 0, cluster;
        RAPIER_TYPE(Real) best = FLT_MAX;
        TRY(RAPIER_FN(SoftBody_ValidateHandle)(soft));
        count = RAPIER_FN(SoftBody_NumParticles)(soft);
        TRY(RAPIER_FN(LastStatus)());
        if (!count) {
            goto done;
        }
        for (size_t i = 0; i < count; ++i) {
            RAPIER_TYPE(Vector) position = RAPIER_FN(SoftBody_ParticlePosition)(soft, i);
            TRY(RAPIER_FN(LastStatus)());
            RAPIER_TYPE(Vector) delta = RAPIER_FN(VectorSub)(position, point);
            RAPIER_TYPE(Real) distance = RAPIER_FN(VectorDot)(delta, delta);
            if (distance < best) {
                best = distance;
                nearest = (uint32_t)i;
            }
        }
        point = RAPIER_FN(SoftBody_ParticlePosition)(soft, nearest);
        TRY(RAPIER_FN(LastStatus)());
        cluster = RAPIER_FN(SoftBody_AddCluster)(soft, &nearest, 1);
        TRY(RAPIER_FN(LastStatus)());
        TRY(RAPIER_FN(SoftBody_ValidateHandle)(soft));
        grab->body = RAPIER_FN(SoftBody_ClusterProxy)(soft, cluster);
        TRY(RAPIER_FN(LastStatus)());
    } else {
        RAPIER_TYPE(Pose) pose = RAPIER_FN(RigidBody_Position)(body);
        TRY(RAPIER_FN(LastStatus)());
        localAnchor = RAPIER_FN(PoseTransformPoint)(RAPIER_FN(PoseInverse)(pose), point);
    }
    grab->active = true;
    grab->planePoint = point;
    builder = RAPIER_FN(KinematicPositionBasedRigidBodyDesc)();
    builder.position.translation = point;
    grab->mouseBody = RAPIER_FN(InsertRigidBody)(t->world, &builder);
    TRY(RAPIER_FN(LastStatus)());
    joint = RAPIER_FN(DefaultJointDesc)();
    joint.localFrame2.translation = localAnchor;
    joint.motorAxes = (1u << RAPIER_CONST(DIMENSION)) - 1;
    for (uint32_t axis = 0; axis < RAPIER_CONST(DIMENSION); ++axis) {
        joint.motors[axis].stiffness = 1000;
        joint.motors[axis].damping = 50;
    }
    grab->joint = RAPIER_FN(InsertImpulseJoint)(grab->mouseBody, grab->body, &joint);
    TRY(RAPIER_FN(LastStatus)());
    TRY(RAPIER_FN(RigidBody_WakeUp)(grab->body, 1));
done:
    if (status != RAPIER_CONST(OK)) {
        (void)tbGrabRelease(t, grab);
    }
    return status;
}

RAPIER_TYPE(Status) tbGrabUpdate(Testbed *t, TbGrab *grab, RAPIER_TYPE(Vector) cameraForward) {
    if (!grab->active) {
        return RAPIER_CONST(OK);
    }
    RAPIER_TYPE(RigidBodyHandle) pulled = pulledBody(t, grab);
    if (!contains(t, pulled) || !contains(t, grab->mouseBody)) {
        return tbGrabRelease(t, grab);
    }
    RAPIER_TYPE(Vector) target;
#if defined(RAPIER_DIM2)
    (void)cameraForward;
    if (!t->cursorValid) {
        return RAPIER_CONST(OK);
    }
    target = t->cursor;
#else
    if (!t->rayValid) {
        return RAPIER_CONST(OK);
    }
    RAPIER_TYPE(Real) denominator = RAPIER_FN(VectorDot)(t->rayDirection, cameraForward);
    if (fabs(denominator) < 1.0e-6) {
        return RAPIER_CONST(OK);
    }
    RAPIER_TYPE(Real)
    distance =
        RAPIER_FN(VectorDot)(RAPIER_FN(VectorSub)(grab->planePoint, t->rayOrigin), cameraForward) /
        denominator;
    if (distance <= 0) {
        return RAPIER_CONST(OK);
    }
    target = RAPIER_FN(VectorAdd)(t->rayOrigin, RAPIER_FN(VectorScale)(t->rayDirection, distance));
#endif
    RAPIER_TYPE(Status) status = RAPIER_CONST(OK);

    TRY(RAPIER_FN(RigidBody_ValidateHandle)(grab->mouseBody));
    TRY(RAPIER_FN(RigidBody_SetNextKinematicTranslation)(grab->mouseBody, target));
    TRY(RAPIER_FN(RigidBody_WakeUp)(pulled, 1));
done:
    return status;
}

void tbGrabDrawCue(Testbed *t, const TbGrab *grab) {
    if (!grab->active) {
        return;
    }
    RAPIER_TYPE(RigidBodyHandle) pulled = pulledBody(t, grab);
    if (!contains(t, pulled) || !contains(t, grab->mouseBody)) {
        return;
    }

    RAPIER_TYPE(Vector) a, b;
    a = RAPIER_FN(RigidBody_Translation)(grab->mouseBody);
    b = RAPIER_FN(RigidBody_Translation)(pulled);
    if (RAPIER_FN(RigidBody_ValidateHandle)(grab->mouseBody) != RAPIER_CONST(OK) ||
        RAPIER_FN(LastStatus)() != RAPIER_CONST(OK) ||
        RAPIER_FN(RigidBody_ValidateHandle)(pulled) != RAPIER_CONST(OK) ||
        RAPIER_FN(LastStatus)() != RAPIER_CONST(OK)) {
        return;
    }
    tbLine(t, a, b, .2f, .3f, .4f, 1);
}
