import {version as vers, reserve_memory as reserve} from "./raw";

export function version(): string {
    return vers();
}

/// Reserves additional memory in WASM land.
///
/// This will grow the internal WASM memory buffer so that it can fit at least
/// the specified amount of extra bytes. This can help reduce future runtime
/// overhead due to dynamic internal memory growth once the limit of the
/// pre-allocated memory is reached.
///
/// This feature is still experimental. Due to the nature of the internal
/// allocator, there can be situations where the allocator decides to perform
/// additional internal memory growth even though not all `extraBytesCount`
/// are occupied yet.
export function reserveMemory(extraBytesCount: number) {
    reserve(extraBytesCount);
}

export * from "./math";
export * from "./dynamics";
export * from "./geometry";
export * from "./pipeline";
export * from "./init";
export * from "./control";
