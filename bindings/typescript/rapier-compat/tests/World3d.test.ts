import {
    ColliderDesc,
    init,
    RigidBodyDesc,
    Vector3,
    World,
} from "../builds/3d-deterministic/pkg";

describe("3d/World", () => {
    let world: World;

    beforeAll(init);

    afterAll(async () => {
        await Promise.resolve();
    });

    beforeEach(() => {
        world = new World(new Vector3(0, 9.8, 0));
    });

    afterEach(() => {
        world.free();
    });

    test("constructor", () => {
        expect(world.colliders.len()).toBe(0);
    });

    test("restored parentless colliders have no parent", () => {
        // A body at index 0: a parentless collider must not resolve to it after a restore.
        const body = world.createRigidBody(RigidBodyDesc.dynamic());
        const attached = world.createCollider(ColliderDesc.ball(0.5), body);
        const standalone = world.createCollider(
            ColliderDesc.cuboid(1.0, 1.0, 1.0),
        );

        const restored = World.restoreSnapshot(world.takeSnapshot());
        expect(restored.getCollider(standalone.handle).parent()).toBeNull();
        expect(restored.getCollider(attached.handle).parent().handle).toBe(
            body.handle,
        );
        restored.free();
    });
});
