import {RawDebugRenderPipeline} from "../raw";
import {Vector, VectorOps} from "../math";
import {
    IntegrationParameters,
    IslandManager,
    ImpulseJointSet,
    MultibodyJointSet,
    RigidBodySet,
} from "../dynamics";
import {BroadPhase, Collider, ColliderSet, NarrowPhase} from "../geometry";
import {QueryFilterFlags} from "./query_pipeline";

/**
 * The vertex and color buffers for debug-redering the physics scene.
 */
export class DebugRenderBuffers {
    /**
     * The lines to render. This is a flat array containing all the lines
     * to render. Each line is described as two consecutive point. Each
     * point is described as two (in 2D) or three (in 3D) consecutive
     * floats. For example, in 2D, the array: `[1, 2, 3, 4, 5, 6, 7, 8]`
     * describes the two segments `[[1, 2], [3, 4]]` and `[[5, 6], [7, 8]]`.
     */
    public vertices: Float32Array;
    /**
     * The color buffer. There is one color per vertex, and each color
     * has four consecutive components (in RGBA format).
     */
    public colors: Float32Array;

    constructor(vertices: Float32Array, colors: Float32Array) {
        this.vertices = vertices;
        this.colors = colors;
    }
}

/**
 * A pipeline for rendering the physics scene.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `debugRenderPipeline.free()`
 * once you are done using it (and all the rigid-bodies it created).
 */
export class DebugRenderPipeline {
    raw: RawDebugRenderPipeline;
    public vertices: Float32Array;
    public colors: Float32Array;

    /**
     * Release the WASM memory occupied by this serialization pipeline.
     */
    free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
        this.vertices = undefined;
        this.colors = undefined;
    }

    constructor(raw?: RawDebugRenderPipeline) {
        this.raw = raw || new RawDebugRenderPipeline();
    }

    public render(
        bodies: RigidBodySet,
        colliders: ColliderSet,
        impulse_joints: ImpulseJointSet,
        multibody_joints: MultibodyJointSet,
        narrow_phase: NarrowPhase,
        filterFlags?: QueryFilterFlags,
        filterPredicate?: (collider: Collider) => boolean,
    ) {
        this.raw.render(
            bodies.raw,
            colliders.raw,
            impulse_joints.raw,
            multibody_joints.raw,
            narrow_phase.raw,
            filterFlags,
            colliders.castClosure(filterPredicate),
        );
        this.vertices = this.raw.vertices();
        this.colors = this.raw.colors();
    }
}
