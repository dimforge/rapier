import {Vector, VectorOps, scratchBuffer} from "../math";
import {RawShapeContact} from "../raw";

/**
 * The contact info between two shapes.
 */
export class ShapeContact {
    /**
     * Distance between the two contact points.
     * If this is negative, this contact represents a penetration.
     */
    distance: number;

    /**
     * Position of the contact on the first shape.
     */
    point1: Vector;

    /**
     * Position of the contact on the second shape.
     */
    point2: Vector;

    /**
     * Contact normal, pointing towards the exterior of the first shape.
     */
    normal1: Vector;

    /**
     * Contact normal, pointing towards the exterior of the second shape.
     * If these contact data are expressed in world-space, this normal is equal to -normal1.
     */
    normal2: Vector;

    constructor(
        dist: number,
        point1: Vector,
        point2: Vector,
        normal1: Vector,
        normal2: Vector,
    ) {
        this.distance = dist;
        this.point1 = point1;
        this.point2 = point2;
        this.normal1 = normal1;
        this.normal2 = normal2;
    }

    /**
     * @param raw - The raw contact returned by the WASM query. It is freed by this method.
     * @param target - If provided, this object is populated and returned instead of
     * allocating a new one.
     */
    public static fromBuffer(
        raw: RawShapeContact,
        target?: ShapeContact,
    ): ShapeContact {
        if (!raw) return null;

        raw.getComponents(scratchBuffer);
        raw.free();

        target ??= new ShapeContact(
            0,
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
            VectorOps.zeros(),
        );

        target.distance = scratchBuffer[0];

        // #if DIM2
        target.point1.x = scratchBuffer[1];
        target.point1.y = scratchBuffer[2];
        target.point2.x = scratchBuffer[3];
        target.point2.y = scratchBuffer[4];
        target.normal1.x = scratchBuffer[5];
        target.normal1.y = scratchBuffer[6];
        target.normal2.x = scratchBuffer[7];
        target.normal2.y = scratchBuffer[8];
        // #endif

        // #if DIM3
        target.point1.x = scratchBuffer[1];
        target.point1.y = scratchBuffer[2];
        target.point1.z = scratchBuffer[3];
        target.point2.x = scratchBuffer[4];
        target.point2.y = scratchBuffer[5];
        target.point2.z = scratchBuffer[6];
        target.normal1.x = scratchBuffer[7];
        target.normal1.y = scratchBuffer[8];
        target.normal1.z = scratchBuffer[9];
        target.normal2.x = scratchBuffer[10];
        target.normal2.y = scratchBuffer[11];
        target.normal2.z = scratchBuffer[12];
        // #endif

        return target;
    }
}
