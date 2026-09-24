import {Collider, ColliderHandle} from "./collider";
import {Vector, VectorOps, scratchBuffer} from "../math";
import {
    RawFeatureType,
    RawPointColliderProjection,
    RawPointProjection,
} from "../raw";
import {FeatureType} from "./feature";
import {ColliderSet} from "./collider_set";

/**
 * The projection of a point on a collider.
 */
export class PointProjection {
    /**
     * The projection of the point on the collider.
     */
    point: Vector;
    /**
     * Is the point inside of the collider?
     */
    isInside: boolean;

    constructor(point: Vector, isInside: boolean) {
        this.point = point;
        this.isInside = isInside;
    }

    /**
     * @param raw - The raw projection returned by the WASM query. It is freed by this method.
     * @param target - If provided, this object is populated and returned instead of
     * allocating a new one.
     */
    public static fromBuffer(
        raw: RawPointProjection,
        target?: PointProjection,
    ): PointProjection {
        if (!raw) return null;

        raw.point(scratchBuffer);

        target ??= new PointProjection(VectorOps.zeros(), false);
        target.point = VectorOps.fromBuffer(scratchBuffer, target.point);
        target.isInside = raw.isInside();

        raw.free();
        return target;
    }
}

/**
 * The projection of a point on a collider (includes the collider handle).
 */
export class PointColliderProjection {
    /**
     * The collider hit by the ray.
     */
    collider: Collider;
    /**
     * The projection of the point on the collider.
     */
    point: Vector;
    /**
     * Is the point inside of the collider?
     */
    isInside: boolean;

    /**
     * The type of the geometric feature the point was projected on.
     */
    featureType = FeatureType.Unknown;

    /**
     * The id of the geometric feature the point was projected on.
     */
    featureId: number | undefined = undefined;

    constructor(
        collider: Collider,
        point: Vector,
        isInside: boolean,
        featureType?: FeatureType,
        featureId?: number,
    ) {
        this.collider = collider;
        this.point = point;
        this.isInside = isInside;
        if (featureId !== undefined) this.featureId = featureId;
        if (featureType !== undefined) this.featureType = featureType;
    }

    /**
     * @param colliderSet - The set the projected-on collider belongs to.
     * @param raw - The raw projection returned by the WASM query. It is freed by this method.
     * @param target - If provided, this object is populated and returned instead of
     * allocating a new one.
     */
    public static fromBuffer(
        colliderSet: ColliderSet,
        raw: RawPointColliderProjection,
        target?: PointColliderProjection,
    ): PointColliderProjection {
        if (!raw) return null;

        raw.point(scratchBuffer);

        target ??= new PointColliderProjection(null, VectorOps.zeros(), false);
        target.collider = colliderSet.get(raw.colliderHandle());
        target.point = VectorOps.fromBuffer(scratchBuffer, target.point);
        target.isInside = raw.isInside();
        target.featureType = raw.featureType() as number as FeatureType;
        target.featureId = raw.featureId();

        raw.free();
        return target;
    }
}
