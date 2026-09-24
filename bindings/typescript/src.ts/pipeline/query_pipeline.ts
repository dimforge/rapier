import {RawRayColliderIntersection} from "../raw";
import {
    ColliderHandle,
    ColliderSet,
    InteractionGroups,
    PointColliderProjection,
    Ray,
    RayColliderIntersection,
    RayColliderHit,
    Shape,
    ColliderShapeCastHit,
} from "../geometry";
import {IslandManager, RigidBodyHandle, RigidBodySet} from "../dynamics";
import {Rotation, RotationOps, Vector, VectorOps} from "../math";

// NOTE: must match the bits in the QueryFilterFlags on the Rust side.
/**
 * Flags for excluding whole sets of colliders from a scene query.
 */
export enum QueryFilterFlags {
    /**
     * Exclude from the query any collider attached to a fixed rigid-body and colliders with no rigid-body attached.
     */
    EXCLUDE_FIXED = 0b0000_0001,
    /**
     * Exclude from the query any collider attached to a dynamic rigid-body.
     */
    EXCLUDE_KINEMATIC = 0b0000_0010,
    /**
     * Exclude from the query any collider attached to a kinematic rigid-body.
     */
    EXCLUDE_DYNAMIC = 0b0000_0100,
    /**
     * Exclude from the query any collider that is a sensor.
     */
    EXCLUDE_SENSORS = 0b0000_1000,
    /**
     * Exclude from the query any collider that is not a sensor.
     */
    EXCLUDE_SOLIDS = 0b0001_0000,
    /**
     * Excludes all colliders not attached to a dynamic rigid-body.
     */
    ONLY_DYNAMIC = QueryFilterFlags.EXCLUDE_FIXED |
        QueryFilterFlags.EXCLUDE_KINEMATIC,
    /**
     * Excludes all colliders not attached to a kinematic rigid-body.
     */
    ONLY_KINEMATIC = QueryFilterFlags.EXCLUDE_DYNAMIC |
        QueryFilterFlags.EXCLUDE_FIXED,
    /**
     * Exclude all colliders attached to a non-fixed rigid-body
     * (this will not exclude colliders not attached to any rigid-body).
     */
    ONLY_FIXED = QueryFilterFlags.EXCLUDE_DYNAMIC |
        QueryFilterFlags.EXCLUDE_KINEMATIC,
}
