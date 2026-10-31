import {
    init,
    Ball,
    ColliderDesc,
    JointData,
    RigidBody,
    RigidBodyDesc,
    SoftBodyDesc,
    Vector2,
    World,
} from "../builds/2d-deterministic/pkg";

function buildWorld(): {world: World; bodies: RigidBody[]} {
    const world = new World(new Vector2(0, -9.81));
    const bodies = [];
    world.createCollider(ColliderDesc.cuboid(20, 0.5));
    for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 3; j++) {
            const body = world.createRigidBody(
                RigidBodyDesc.dynamic().setTranslation(
                    i * 1.1 - 2 + 0.1 * j,
                    1 + j * 1.1,
                ),
            );
            const desc =
                (i + j) % 2 == 0
                    ? ColliderDesc.cuboid(0.5, 0.5)
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
    expect(receiver.multibodyJoints.len()).toBe(sender.multibodyJoints.len());
    expect(receiver.softBodies.len()).toBe(sender.softBodies.len());
    // The gravity is stored as f32 by the engine.
    expect(receiver.gravity.x).toBe(Math.fround(sender.gravity.x));
    expect(receiver.gravity.y).toBe(Math.fround(sender.gravity.y));
    expect(receiver.timestep).toBe(sender.timestep);
}

/** The `kind` of the error `f` throws. */
function errorKind(f: () => unknown): string {
    try {
        f();
    } catch (e) {
        expect(e).toBeInstanceOf(Error);
        return e.kind;
    }
    throw new Error("expected an error");
}

describe("2d/snapshot deltas", () => {
    beforeAll(init);

    test("replicate a world through deltas", () => {
        const {world: sender, bodies} = buildWorld();
        for (let i = 0; i < 10; i++) sender.step();
        sender.enableChangeTracking();

        // Receiver bootstrapped by restoring a snapshot taken with its baseline tick.
        let lagAck = sender.changeTick();
        const snapshot = sender.takeSnapshot();
        const lagging = World.restoreSnapshot(snapshot);
        // A restored world has no baseline until it is declared.
        expect(
            errorKind(() => lagging.applyDelta(sender.takeDelta(lagAck).delta)),
        ).toBe("baseline_mismatch");
        lagging.setDeltaBaseline(lagAck);

        // Receiver bootstrapped by a full delta into an empty world.
        const full = sender.takeDelta();
        const fast = new World(new Vector2(0, 0));
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
                    RigidBodyDesc.dynamic().setTranslation(-4, 6),
                );
                sender.createCollider(ColliderDesc.ball(0.3), a);
                b = sender.createRigidBody(
                    RigidBodyDesc.dynamic().setTranslation(-4, 7),
                );
                sender.createCollider(ColliderDesc.ball(0.3), b);
                sender.createImpulseJoint(
                    JointData.revolute(
                        new Vector2(0, 0.5),
                        new Vector2(0, -0.5),
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
                    RigidBodyDesc.dynamic().setTranslation(3, 5),
                );
                sender.createCollider(ColliderDesc.cuboid(0.2, 0.2), spawned);
            }
            if (step == 17) sender.gravity = new Vector2(0.5, -5);
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
        expect(fast.gravity).toEqual(new Vector2(0.5, -5));
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

    test("wrappers follow multibody joints, soft bodies and shapes", () => {
        const {world: sender, bodies} = buildWorld();
        sender.enableChangeTracking();
        const receiver = new World(new Vector2(0, 0));
        let full = sender.takeDelta();
        let ack = receiver.applyDelta(full.delta);
        const sync = () => {
            const delta = sender.takeDelta(ack);
            ack = receiver.applyDelta(delta.delta);
            expectSameState(sender, receiver);
        };

        // A multibody joint inserted on the sender.
        const m1 = sender.createRigidBody(
            RigidBodyDesc.dynamic().setTranslation(6, 6),
        );
        const m2 = sender.createRigidBody(
            RigidBodyDesc.dynamic().setTranslation(6, 7),
        );
        sender.createCollider(ColliderDesc.ball(0.3), m1);
        sender.createCollider(ColliderDesc.ball(0.3), m2);
        const multibodyJoint = sender.createMultibodyJoint(
            JointData.revolute(new Vector2(0, 0.5), new Vector2(0, -0.5)),
            m1,
            m2,
            true,
        );
        sender.step();
        sync();
        const replicatedJoint = receiver.getMultibodyJoint(
            multibodyJoint.handle,
        );
        expect(replicatedJoint).not.toBeNull();
        expect(replicatedJoint.handle).toBe(multibodyJoint.handle);
        expect(replicatedJoint.isValid()).toBe(true);

        // A soft body added, then removed.
        const softBody = sender.createSoftBody(
            SoftBodyDesc.rope(new Vector2(-6, 4), new Vector2(-3, 4), 8),
        );
        sender.step();
        sync();
        const replicatedSoftBody = receiver.getSoftBody(softBody.handle);
        expect(replicatedSoftBody).not.toBeNull();
        expect(replicatedSoftBody.handle).toBe(softBody.handle);
        sender.removeSoftBody(softBody);
        sender.step();
        sync();
        expect(receiver.getSoftBody(softBody.handle)).toBeNull();
        expect(receiver.softBodies.len()).toBe(0);

        // A shape changed on the sender: the receiver's cached shape is dropped.
        const collider = bodies[1].collider(0);
        const before = receiver.getCollider(collider.handle).shape as Ball;
        expect(before.radius).toBe(0.5);
        collider.setShape(new Ball(0.25));
        sender.step();
        sync();
        const after = receiver.getCollider(collider.handle).shape as Ball;
        expect(after.radius).toBe(0.25);

        sender.free();
        receiver.free();
    });

    test("delta errors throw with a kind", () => {
        const {world: sender} = buildWorld();
        expect(errorKind(() => sender.takeDelta())).toBe("tracking_disabled");
        expect(errorKind(() => sender.changeTick())).toBe("tracking_disabled");

        sender.enableChangeTracking();
        const first = sender.takeDelta();
        sender.step();
        const second = sender.takeDelta(first.tick);
        expect(errorKind(() => sender.takeDelta(new Uint8Array([1, 2])))).toBe(
            "malformed",
        );
        // Trailing bytes after a valid tick are rejected.
        const padded = new Uint8Array(first.tick.length + 1);
        padded.set(first.tick);
        expect(errorKind(() => sender.takeDelta(padded))).toBe("malformed");

        // A delta applied to a receiver that does not hold its baseline.
        const receiver = new World(new Vector2(0, 0));
        receiver.applyDelta(first.delta);
        receiver.applyDelta(second.delta);
        expect(errorKind(() => receiver.applyDelta(second.delta))).toBe(
            "baseline_mismatch",
        );
        expect(
            errorKind(() => receiver.applyDelta(new Uint8Array([1, 2, 3]))),
        ).toBe("malformed");
        expect(
            errorKind(() => receiver.setDeltaBaseline(new Uint8Array([1]))),
        ).toBe("malformed");

        // A rejected delta still updates the gravity and the integration parameters.
        sender.gravity = new Vector2(0.5, -5);
        sender.timestep = 1 / 30;
        const mismatched = sender.takeDelta(first.tick);
        expect(errorKind(() => receiver.applyDelta(mismatched.delta))).toBe(
            "baseline_mismatch",
        );
        expect(receiver.gravity).toEqual(new Vector2(0.5, -5));
        expect(receiver.timestep).toBe(sender.timestep);

        // Restarting the tracking invalidates the ticks issued before.
        sender.disableChangeTracking();
        sender.enableChangeTracking();
        expect(errorKind(() => sender.takeDelta(second.tick))).toBe(
            "unknown_baseline",
        );

        sender.free();
        receiver.free();
    });

    test("a stepped receiver needs a full delta", () => {
        const {world: sender} = buildWorld();
        sender.enableChangeTracking();
        const receiver = new World(new Vector2(0, 0));
        const ack = receiver.applyDelta(sender.takeDelta().delta);
        expectSameState(sender, receiver);

        sender.step();
        receiver.step();
        expect(
            errorKind(() => receiver.applyDelta(sender.takeDelta(ack).delta)),
        ).toBe("baseline_mismatch");

        const full = sender.takeDelta();
        const newAck = receiver.applyDelta(full.delta);
        expect(sameBytes(newAck, full.tick)).toBe(true);
        expectSameState(sender, receiver);

        sender.step();
        const delta = sender.takeDelta(newAck);
        receiver.applyDelta(delta.delta);
        expectSameState(sender, receiver);

        sender.free();
        receiver.free();
    });
});
