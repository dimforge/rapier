// @ts-ignore
import wasmBase64 from "../pkg/dist/rapier_wasm2d_bg.wasm";
import wasmInit from "../pkg/dist/rapier_wasm2d";
import base64 from "base64-js";

/**
 * Initializes RAPIER.
 * Has to be called and awaited before using any library methods.
 */
export async function init() {
    await wasmInit(base64.toByteArray(wasmBase64 as unknown as string).buffer);
}
