interface RestorableWorld {
    free(): void;
}

interface SnapshotWorld extends RestorableWorld {
    takeSnapshot(): Uint8Array;
}

interface RapierApi {
    init(): Promise<void> | void;
}

class NoopFinalizationRegistry {
    constructor(_callback: unknown) {}

    register(_target: object, _heldValue: unknown, _token?: object) {}

    unregister(_token: object) {
        return true;
    }
}

async function initializeWithCapturedMemory<T extends RapierApi>(
    load: () => Promise<T>,
): Promise<{rapier: T; memory: WebAssembly.Memory}> {
    const originalFinalizationRegistry = (globalThis as any)
        .FinalizationRegistry;
    const originalInstantiate = WebAssembly.instantiate;
    let memory: WebAssembly.Memory | undefined;

    (globalThis as any).FinalizationRegistry = NoopFinalizationRegistry;
    (WebAssembly as any).instantiate = async (...args: any[]) => {
        const result = await (originalInstantiate as any)(...args);
        const instance =
            result instanceof WebAssembly.Instance ? result : result.instance;
        if (instance.exports.memory instanceof WebAssembly.Memory) {
            memory = instance.exports.memory;
        }
        return result;
    };

    try {
        const rapier = await load();
        await rapier.init();
        if (!memory) throw new Error("Rapier WASM memory was not captured");
        return {rapier, memory};
    } finally {
        (globalThis as any).FinalizationRegistry = originalFinalizationRegistry;
        WebAssembly.instantiate = originalInstantiate;
    }
}

function expectRestoresToReuseWasmMemory(
    source: SnapshotWorld,
    memory: WebAssembly.Memory,
    restore: (snapshot: Uint8Array) => RestorableWorld,
) {
    const snapshot = source.takeSnapshot();
    source.free();

    for (let i = 0; i < 200; i++) restore(snapshot).free();
    const bytesBefore = memory.buffer.byteLength;

    for (let i = 0; i < 2500; i++) restore(snapshot).free();

    expect(memory.buffer.byteLength).toBe(bytesBefore);
}

test("2d snapshot restores reuse WASM memory after worlds are freed", async () => {
    const {rapier, memory} = await initializeWithCapturedMemory(
        () => import("../builds/2d-deterministic/pkg"),
    );
    const source = new rapier.World(new rapier.Vector2(0, -9.81));

    expectRestoresToReuseWasmMemory(
        source,
        memory,
        rapier.World.restoreSnapshot,
    );
});

test("3d snapshot restores reuse WASM memory after worlds are freed", async () => {
    const {rapier, memory} = await initializeWithCapturedMemory(
        () => import("../builds/3d-deterministic/pkg"),
    );
    const source = new rapier.World(new rapier.Vector3(0, -9.81, 0));

    expectRestoresToReuseWasmMemory(
        source,
        memory,
        rapier.World.restoreSnapshot,
    );
});
