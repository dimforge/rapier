import {Collider} from "./collider";
import {Vector, VectorOps} from "../math";

/**
 * The intersection between a ray and a collider.
 */
export class ShapeCastHit {
    /**
     * The time of impact of the two shapes.
     */
    time_of_impact: number;
    /**
     * The local-space contact point on the first shape, at
     * the time of impact.
     */
    witness1: Vector;
    /**
     * The local-space contact point on the second shape, at
     * the time of impact.
     */
    witness2: Vector;
    /**
     * The local-space normal on the first shape, at
     * the time of impact.
     */
    normal1: Vector;
    /**
     * The local-space normal on the second shape, at
     * the time of impact.
     */
    normal2: Vector;

    constructor(
        time_of_impact: number,
        witness1: Vector,
        witness2: Vector,
        normal1: Vector,
        normal2: Vector,
    ) {
        this.time_of_impact = time_of_impact;
        this.witness1 = witness1;
        this.witness2 = witness2;
        this.normal1 = normal1;
        this.normal2 = normal2;
    }

    public static fromBuffer(
        collider: Collider,
        buffer: Float32Array,
        target?: ShapeCastHit,
    ): ShapeCastHit {
        if (!buffer) return null;

        target ??= new ShapeCastHit(
            0,
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
        );

        target.time_of_impact = buffer[0];

        // #if DIM2
        target.witness1.x = buffer[1];
        target.witness1.y = buffer[2];
        target.witness2.x = buffer[3];
        target.witness2.y = buffer[4];
        target.normal1.x = buffer[5];
        target.normal1.y = buffer[6];
        target.normal2.x = buffer[7];
        target.normal2.y = buffer[8];
        // #endif

        // #if DIM3
        target.witness1.x = buffer[1];
        target.witness1.y = buffer[2];
        target.witness1.z = buffer[3];
        target.witness2.x = buffer[4];
        target.witness2.y = buffer[5];
        target.witness2.z = buffer[6];
        target.normal1.x = buffer[7];
        target.normal1.y = buffer[8];
        target.normal1.z = buffer[9];
        target.normal2.x = buffer[10];
        target.normal2.y = buffer[11];
        target.normal2.z = buffer[12];
        // #endif

        return target;
    }
}

/**
 * The intersection between a ray and a collider.
 */
export class ColliderShapeCastHit extends ShapeCastHit {
    /**
     * The handle of the collider hit by the ray.
     */
    collider: Collider;

    constructor(
        collider: Collider,
        time_of_impact: number,
        witness1: Vector,
        witness2: Vector,
        normal1: Vector,
        normal2: Vector,
    ) {
        super(time_of_impact, witness1, witness2, normal1, normal2);
        this.collider = collider;
    }

    public static fromBuffer(
        collider: Collider,
        buffer: Float32Array,
        target?: ColliderShapeCastHit,
    ): ColliderShapeCastHit {
        if (!buffer) return null;

        target ??= new ColliderShapeCastHit(
            null,
            0,
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
        );

        target.collider = collider;

        target.time_of_impact = buffer[0];

        // #if DIM2
        target.witness1.x = buffer[1];
        target.witness1.y = buffer[2];
        target.witness2.x = buffer[3];
        target.witness2.y = buffer[4];
        target.normal1.x = buffer[5];
        target.normal1.y = buffer[6];
        target.normal2.x = buffer[7];
        target.normal2.y = buffer[8];
        // #endif

        // #if DIM3
        target.witness1.x = buffer[1];
        target.witness1.y = buffer[2];
        target.witness1.z = buffer[3];
        target.witness2.x = buffer[4];
        target.witness2.y = buffer[5];
        target.witness2.z = buffer[6];
        target.normal1.x = buffer[7];
        target.normal1.y = buffer[8];
        target.normal1.z = buffer[9];
        target.normal2.x = buffer[10];
        target.normal2.y = buffer[11];
        target.normal2.z = buffer[12];
        // #endif

        return target;
    }
}
