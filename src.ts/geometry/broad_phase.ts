import {RawBroadPhase, RawRayColliderIntersection} from "../raw";
import {RigidBodyHandle, RigidBodySet} from "../dynamics";
import {ColliderSet} from "./collider_set";
import {Ray, RayColliderHit, RayColliderIntersection} from "./ray";
import {InteractionGroups} from "./interaction_groups";
import {ColliderHandle} from "./collider";
import {Rotation, RotationOps, Vector, VectorOps, scratchBuffer} from "../math";
import {Shape} from "./shape";
import {PointColliderProjection} from "./point";
import {ColliderShapeCastHit} from "./toi";
import {QueryFilterFlags} from "../pipeline";
import {NarrowPhase} from "./narrow_phase";

/**
 * The broad-phase used for coarse collision-detection.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `broadPhase.free()`
 * once you are done using it.
 */
export class BroadPhase {
    raw: RawBroadPhase;

    /**
     * Release the WASM memory occupied by this broad-phase.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }

    constructor(raw?: RawBroadPhase) {
        this.raw = raw || new RawBroadPhase();
    }

    /**
     * Find the closest intersection between a ray and a set of collider.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @param groups - Used to filter the colliders that can or cannot be hit by the ray.
     * @param filter - The callback to filter out which collider will be hit.
     */
    public castRay(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        ray: Ray,
        maxToi: number,
        solid: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
    ): RayColliderHit | null {
        let rawOrig = VectorOps.intoRaw(ray.origin);
        let rawDir = VectorOps.intoRaw(ray.dir);
        let result = RayColliderHit.fromRaw(
            colliders,
            this.raw.castRay(
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                rawOrig,
                rawDir,
                maxToi,
                solid,
                filterFlags,
                filterGroups,
                filterExcludeCollider,
                filterExcludeRigidBody,
                filterPredicate,
            ),
        );

        rawOrig.free();
        rawDir.free();

        return result;
    }

    /**
     * Find the closest intersection between a ray and a set of collider.
     *
     * This also computes the normal at the hit point.
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @param groups - Used to filter the colliders that can or cannot be hit by the ray.
     */
    public castRayAndGetNormal(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        ray: Ray,
        maxToi: number,
        solid: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
        target?: RayColliderIntersection,
    ): RayColliderIntersection | null {
        let rawOrig = VectorOps.intoRaw(ray.origin);
        let rawDir = VectorOps.intoRaw(ray.dir);
        let result = RayColliderIntersection.fromBuffer(
            colliders,
            this.raw.castRayAndGetNormal(
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                rawOrig,
                rawDir,
                maxToi,
                solid,
                filterFlags,
                filterGroups,
                filterExcludeCollider,
                filterExcludeRigidBody,
                filterPredicate,
            ),
            target,
        );

        rawOrig.free();
        rawDir.free();

        return result;
    }

    /**
     * Cast a ray and collects all the intersections between a ray and the scene.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param ray - The ray to cast.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the length of the ray to `ray.dir.norm() * maxToi`.
     * @param solid - If `false` then the ray will attempt to hit the boundary of a shape, even if its
     *   origin already lies inside of a shape. In other terms, `true` implies that all shapes are plain,
     *   whereas `false` implies that all shapes are hollow for this ray-cast.
     * @param groups - Used to filter the colliders that can or cannot be hit by the ray.
     * @param callback - The callback called once per hit (in no particular order) between a ray and a collider.
     *   If this callback returns `false`, then the cast will stop and no further hits will be detected/reported.
     */
    public intersectionsWithRay(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        ray: Ray,
        maxToi: number,
        solid: boolean,
        callback: (intersect: RayColliderIntersection) => boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
    ) {
        let rawOrig = VectorOps.intoRaw(ray.origin);
        let rawDir = VectorOps.intoRaw(ray.dir);
        let rawCallback = (rawInter: RawRayColliderIntersection) => {
            return callback(
                RayColliderIntersection.fromBuffer(colliders, rawInter),
            );
        };

        this.raw.intersectionsWithRay(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            rawOrig,
            rawDir,
            maxToi,
            solid,
            rawCallback,
            filterFlags,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate,
        );

        rawOrig.free();
        rawDir.free();
    }

    /**
     * Gets the handle of up to one collider intersecting the given shape.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param shapePos - The position of the shape used for the intersection test.
     * @param shapeRot - The orientation of the shape used for the intersection test.
     * @param shape - The shape used for the intersection test.
     * @param groups - The bit groups and filter associated to the ray, in order to only
     *   hit the colliders with collision groups compatible with the ray's group.
     */
    public intersectionWithShape(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        shapePos: Vector,
        shapeRot: Rotation,
        shape: Shape,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
    ): ColliderHandle | null {
        let rawPos = VectorOps.intoRaw(shapePos);
        let rawRot = RotationOps.intoRaw(shapeRot);
        let rawShape = shape.intoRaw();
        let result = this.raw.intersectionWithShape(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            rawPos,
            rawRot,
            rawShape,
            filterFlags,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate,
        );

        rawPos.free();
        rawRot.free();
        rawShape.free();

        return result;
    }

    /**
     * Find the projection of a point on the closest collider.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param point - The point to project.
     * @param solid - If this is set to `true` then the collider shapes are considered to
     *   be plain (if the point is located inside of a plain shape, its projection is the point
     *   itself). If it is set to `false` the collider shapes are considered to be hollow
     *   (if the point is located inside of an hollow shape, it is projected on the shape's
     *   boundary).
     * @param groups - The bit groups and filter associated to the point to project, in order to only
     *   project on colliders with collision groups compatible with the ray's group.
     */
    public projectPoint(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        point: Vector,
        solid: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
        target?: PointColliderProjection,
    ): PointColliderProjection | null {
        let rawPoint = VectorOps.intoRaw(point);
        let result = PointColliderProjection.fromBuffer(
            colliders,
            this.raw.projectPoint(
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                rawPoint,
                solid,
                filterFlags,
                filterGroups,
                filterExcludeCollider,
                filterExcludeRigidBody,
                filterPredicate,
            ),
            target,
        );

        rawPoint.free();

        return result;
    }

    /**
     * Find the projection of a point on the closest collider.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param point - The point to project.
     * @param groups - The bit groups and filter associated to the point to project, in order to only
     *   project on colliders with collision groups compatible with the ray's group.
     */
    public projectPointAndGetFeature(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        point: Vector,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
        target?: PointColliderProjection,
    ): PointColliderProjection | null {
        let rawPoint = VectorOps.intoRaw(point);
        let result = PointColliderProjection.fromBuffer(
            colliders,
            this.raw.projectPointAndGetFeature(
                narrowPhase.raw,
                bodies.raw,
                colliders.raw,
                rawPoint,
                filterFlags,
                filterGroups,
                filterExcludeCollider,
                filterExcludeRigidBody,
                filterPredicate,
            ),
            target,
        );

        rawPoint.free();

        return result;
    }

    /**
     * Find all the colliders containing the given point.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param point - The point used for the containment test.
     * @param groups - The bit groups and filter associated to the point to test, in order to only
     *   test on colliders with collision groups compatible with the ray's group.
     * @param callback - A function called with the handles of each collider with a shape
     *   containing the `point`.
     */
    public intersectionsWithPoint(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        point: Vector,
        callback: (handle: ColliderHandle) => boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
    ) {
        let rawPoint = VectorOps.intoRaw(point);

        this.raw.intersectionsWithPoint(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            rawPoint,
            callback,
            filterFlags,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate,
        );

        rawPoint.free();
    }

    /**
     * Casts a shape at a constant linear velocity and retrieve the first collider it hits.
     * This is similar to ray-casting except that we are casting a whole shape instead of
     * just a point (the ray origin).
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param shapePos - The initial position of the shape to cast.
     * @param shapeRot - The initial rotation of the shape to cast.
     * @param shapeVel - The constant velocity of the shape to cast (i.e. the cast direction).
     * @param shape - The shape to cast.
     * @param targetDistance − If the shape moves closer to this distance from a collider, a hit
     *                       will be returned.
     * @param maxToi - The maximum time-of-impact that can be reported by this cast. This effectively
     *   limits the distance traveled by the shape to `shapeVel.norm() * maxToi`.
     * @param stopAtPenetration - If set to `false`, the linear shape-cast won’t immediately stop if
     *   the shape is penetrating another shape at its starting point **and** its trajectory is such
     *   that it’s on a path to exit that penetration state.
     * @param groups - The bit groups and filter associated to the shape to cast, in order to only
     *   test on colliders with collision groups compatible with this group.
     * @param {ColliderShapeCastHit?} target - The object to be populated. If provided,
     * the function returns this object instead of creating a new one.
     */
    public castShape(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        shapePos: Vector,
        shapeRot: Rotation,
        shapeVel: Vector,
        shape: Shape,
        targetDistance: number,
        maxToi: number,
        stopAtPenetration: boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
        target?: ColliderShapeCastHit,
    ): ColliderShapeCastHit | null {
        let rawPos = VectorOps.intoRaw(shapePos);
        let rawRot = RotationOps.intoRaw(shapeRot);
        let rawVel = VectorOps.intoRaw(shapeVel);
        let rawShape = shape.intoRaw();

        const rawColliderShapeCastHit = this.raw.castShape(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            rawPos,
            rawRot,
            rawVel,
            rawShape,
            targetDistance,
            maxToi,
            stopAtPenetration,
            filterFlags,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate,
        );

        let result = null;
        if (rawColliderShapeCastHit) {
            const colliderHandle: number =
                rawColliderShapeCastHit.colliderHandle();
            rawColliderShapeCastHit.getComponents(scratchBuffer);
            result = ColliderShapeCastHit.fromBuffer(
                colliders.get(colliderHandle),
                scratchBuffer,
                target,
            );
            rawColliderShapeCastHit.free();
        }

        rawPos.free();
        rawRot.free();
        rawVel.free();
        rawShape.free();

        return result;
    }

    /**
     * Retrieve all the colliders intersecting the given shape.
     *
     * @param colliders - The set of colliders taking part in this pipeline.
     * @param shapePos - The position of the shape to test.
     * @param shapeRot - The orientation of the shape to test.
     * @param shape - The shape to test.
     * @param groups - The bit groups and filter associated to the shape to test, in order to only
     *   test on colliders with collision groups compatible with this group.
     * @param callback - A function called with the handles of each collider intersecting the `shape`.
     */
    public intersectionsWithShape(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        shapePos: Vector,
        shapeRot: Rotation,
        shape: Shape,
        callback: (handle: ColliderHandle) => boolean,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterExcludeCollider?: ColliderHandle,
        filterExcludeRigidBody?: RigidBodyHandle,
        filterPredicate?: (collider: ColliderHandle) => boolean,
    ) {
        let rawPos = VectorOps.intoRaw(shapePos);
        let rawRot = RotationOps.intoRaw(shapeRot);
        let rawShape = shape.intoRaw();

        this.raw.intersectionsWithShape(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            rawPos,
            rawRot,
            rawShape,
            callback,
            filterFlags,
            filterGroups,
            filterExcludeCollider,
            filterExcludeRigidBody,
            filterPredicate,
        );

        rawPos.free();
        rawRot.free();
        rawShape.free();
    }

    /**
     * Finds the handles of all the colliders with an AABB intersecting the given AABB.
     *
     * @param aabbCenter - The center of the AABB to test.
     * @param aabbHalfExtents - The half-extents of the AABB to test.
     * @param callback - The callback that will be called with the handles of all the colliders
     *                   currently intersecting the given AABB.
     */
    public collidersWithAabbIntersectingAabb(
        narrowPhase: NarrowPhase,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        aabbCenter: Vector,
        aabbHalfExtents: Vector,
        callback: (handle: ColliderHandle) => boolean,
    ) {
        let rawCenter = VectorOps.intoRaw(aabbCenter);
        let rawHalfExtents = VectorOps.intoRaw(aabbHalfExtents);
        this.raw.collidersWithAabbIntersectingAabb(
            narrowPhase.raw,
            bodies.raw,
            colliders.raw,
            rawCenter,
            rawHalfExtents,
            callback,
        );
        rawCenter.free();
        rawHalfExtents.free();
    }
}
