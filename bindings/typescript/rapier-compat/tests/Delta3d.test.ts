import {
    init,
    ColliderDesc,
    JointData,
    RigidBody,
    RigidBodyDesc,
    Vector3,
    World,
} from "../builds/3d-deterministic/pkg";

function buildWorld(): {world: World; bodies: RigidBody[]} {
    const world = new World(new Vector3(0, -9.81, 0));
    const bodies = [];
    world.createCollider(ColliderDesc.cuboid(20, 0.5, 20));
    for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 3; j++) {
            const body = world.createRigidBody(
                RigidBodyDesc.dynamic().setTranslation(
                    i * 1.1 - 2,
                    1 + j * 1.1,
                    0.1 * j,
                ),
            );
            const desc =
                (i + j) % 2 == 0
                    ? ColliderDesc.cuboid(0.5, 0.5, 0.5)
                    : ColliderDesc.ball(0.5);
            world.createCollider(desc, body);
            bodies.push(body);
        }
    }
    return {world, bodies};
}

function sameBytes(a: Uint8Array, b: Uint8Array): boolean {
    return a.length == b.length && a.every((byte, i) => byte == b[i]);
}

function expectSameState(sender: World, receiver: World) {
    expect(sameBytes(receiver.takeSnapshot(), sender.takeSnapshot())).toBe(
        true,
    );
    expect(receiver.bodies.len()).toBe(sender.bodies.len());
    expect(receiver.colliders.len()).toBe(sender.colliders.len());
    expect(receiver.impulseJoints.len()).toBe(sender.impulseJoints.len());
    // The gravity is stored as f32 by the engine.
    expect(receiver.gravity.x).toBe(Math.fround(sender.gravity.x));
    expect(receiver.gravity.y).toBe(Math.fround(sender.gravity.y));
    expect(receiver.gravity.z).toBe(Math.fround(sender.gravity.z));
    expect(receiver.timestep).toBe(sender.timestep);
}

describe("3d/snapshot deltas", () => {
    beforeAll(init);

    test("replicate a world through deltas", () => {
        const {world: sender, bodies} = buildWorld();
        for (let i = 0; i < 10; i++) sender.step();
        sender.enableChangeTracking();

        // Receiver bootstrapped by restoring a snapshot taken with its baseline tick.
        const snapshot = sender.takeSnapshot();
        let lagAck = sender.takeDelta().tick;
        const lagging = World.restoreSnapshot(snapshot);

        // Receiver bootstrapped by a full delta into an empty world.
        const full = sender.takeDelta();
        const fast = new World(new Vector3(0, 0, 0));
        let fastAck = fast.applyDelta(full.delta);
        expect(sameBytes(fastAck, full.tick)).toBe(true);
        expectSameState(sender, fast);
        expectSameState(sender, lagging);

        let removed: RigidBody = null;
        let spawned: RigidBody = null;
        let a: RigidBody = null;
        let b: RigidBody = null;
        const steps = 40;
        for (let step = 0; step < steps; step++) {
            if (step == 3) {
                a = sender.createRigidBody(
                    RigidBodyDesc.dynamic().setTranslation(-4, 6, 0),
                );
                sender.createCollider(ColliderDesc.ball(0.3), a);
                b = sender.createRigidBody(
                    RigidBodyDesc.dynamic().setTranslation(-4, 7, 0),
                );
                sender.createCollider(ColliderDesc.ball(0.3), b);
                sender.createImpulseJoint(
                    JointData.spherical(
                        new Vector3(0, 0.5, 0),
                        new Vector3(0, -0.5, 0),
                    ),
                    a,
                    b,
                    true,
                );
            }
            if (step == 8) {
                removed = bodies[5];
                sender.removeRigidBody(removed);
            }
            if (step == 12) {
                // Likely reuses the index of the removed body, with a new generation.
                spawned = sender.createRigidBody(
                    RigidBodyDesc.dynamic().setTranslation(3, 5, 0),
                );
                sender.createCollider(
                    ColliderDesc.cuboid(0.2, 0.2, 0.2),
                    spawned,
                );
            }
            if (step == 17) sender.gravity = new Vector3(0, -5, 0.5);
            if (step == 22) sender.timestep = 1 / 120;

            sender.step();

            const delta = sender.takeDelta(fastAck);
            fastAck = fast.applyDelta(delta.delta);
            expect(sameBytes(fastAck, delta.tick)).toBe(true);
            expectSameState(sender, fast);

            // The lagging receiver only syncs every 5 steps: its deltas cover several steps.
            if (step % 5 == 4) {
                const lagDelta = sender.takeDelta(lagAck);
                lagAck = lagging.applyDelta(lagDelta.delta);
                expectSameState(sender, lagging);
            }
        }

        // The wrappers of added and removed objects follow the deltas.
        for (const replica of [fast, lagging]) {
            expect(replica.getRigidBody(spawned.handle).handle).toBe(
                spawned.handle,
            );
            expect(replica.getRigidBody(spawned.handle).isValid()).toBe(true);
            const joint = replica.impulseJoints.getAll()[0];
            expect(joint.body1().handle).toBe(a.handle);
            expect(joint.body2().handle).toBe(b.handle);
            expect(replica.bodies.raw.contains(removed.handle)).toBe(false);
            expect(replica.getRigidBody(removed.handle)?.handle).not.toBe(
                removed.handle,
            );
            const body = replica.getRigidBody(spawned.handle);
            expect(body.numColliders()).toBe(1);
            expect(body.collider(0).parent()).toBe(body);
        }
        expect(fast.gravity).toEqual(new Vector3(0, -5, 0.5));
        expect(fast.timestep).toBeCloseTo(1 / 120);

        // The replicas keep simulating identically on their own.
        for (let step = 0; step < 30; step++) {
            sender.step();
            fast.step();
            lagging.step();
            expectSameState(sender, fast);
            expectSameState(sender, lagging);
        }

        sender.free();
        fast.free();
        lagging.free();
    });

    test("delta errors throw", () => {
        const {world: sender} = buildWorld();
        expect(() => sender.takeDelta()).toThrow(/not enabled/);

        sender.enableChangeTracking();
        const first = sender.takeDelta();
        sender.step();
        const second = sender.takeDelta(first.tick);
        expect(() => sender.takeDelta(new Uint8Array([1, 2]))).toThrow(
            /malformed/,
        );

        // A delta applied to a receiver that does not hold its baseline.
        const receiver = new World(new Vector3(0, 0, 0));
        receiver.applyDelta(first.delta);
        receiver.applyDelta(second.delta);
        expect(() => receiver.applyDelta(second.delta)).toThrow(/baseline/);
        expect(() => receiver.applyDelta(new Uint8Array([1, 2, 3]))).toThrow(
            /malformed/,
        );

        // Restarting the tracking invalidates the ticks issued before.
        sender.disableChangeTracking();
        sender.enableChangeTracking();
        expect(() => sender.takeDelta(second.tick)).toThrow(/baseline/);

        sender.free();
        receiver.free();
    });
});
