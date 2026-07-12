import {RawIslandManager} from "../raw";
import {RigidBodyHandle} from "./rigid_body";

/**
 * The CCD solver responsible for resolving Continuous Collision Detection.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `ccdSolver.free()`
 * once you are done using it.
 */
export class IslandManager {
    raw: RawIslandManager;

    /**
     * Release the WASM memory occupied by this narrow-phase.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }

    constructor(raw?: RawIslandManager) {
        this.raw = raw || new RawIslandManager();
    }

    /**
     * Applies the given closure to the handle of each active rigid-bodies contained by this set.
     *
     * A rigid-body is active if it is not sleeping, i.e., if it moved recently.
     *
     * @param f - The closure to apply.
     */
    public forEachActiveRigidBodyHandle(f: (handle: RigidBodyHandle) => void) {
        this.raw.forEachActiveRigidBodyHandle(f);
    }
}
