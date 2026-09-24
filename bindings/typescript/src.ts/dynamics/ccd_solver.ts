import {RawCCDSolver} from "../raw";

/**
 * The CCD solver responsible for resolving Continuous Collision Detection.
 *
 * To avoid leaking WASM resources, this MUST be freed manually with `ccdSolver.free()`
 * once you are done using it.
 */
export class CCDSolver {
    raw: RawCCDSolver;

    /**
     * Release the WASM memory occupied by this narrow-phase.
     */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }
        this.raw = undefined;
    }

    constructor(raw?: RawCCDSolver) {
        this.raw = raw || new RawCCDSolver();
    }
}
