import {
    init,
    Ball,
    ColliderDesc,
    JointData,
    PrismaticImpulseJoint,
    PrismaticMultibodyJoint,
    RevoluteImpulseJoint,
    RevoluteMultibodyJoint,
    RigidBody,
    RigidBodyDesc,
    SnapshotDeltaError,
    SnapshotDeltaErrorKind,
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

/** The snapshot-delta error `f` throws. */
function deltaError(f: () => unknown): SnapshotDeltaError {
    try {
        f();
    } catch (e) {
        expect(e).toBeInstanceOf(Error);
        return e as SnapshotDeltaError;
    }
    throw new Error("expected an error");
}

/** The `kind` of the error `f` throws. */
function errorKind(f: () => unknown): SnapshotDeltaErrorKind {
    return deltaError(f).kind;
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

        // A falling convex hull: a shape the engine deduplicates by content.
        const hullBody = sender.createRigidBody(
            RigidBodyDesc.dynamic().setTranslation(0, 8),
        );
        sender.createCollider(
            ColliderDesc.convexHull(
                new Float32Array([-0.4, -0.3, 0.4, -0.3, 0.5, 0.3, -0.3, 0.4]),
            ),
            hullBody,
        );
        sender.step();
        sync();

        // A shape changed on the sender: only that collider's cached shape is dropped.
        const collider = bodies[1].collider(0);
        const cached = new Map(
            receiver.colliders.getAll().map((co) => [co.handle, co.shape]),
        );
        const before = cached.get(collider.handle) as Ball;
        expect(before.radius).toBe(0.5);
        collider.setShape(new Ball(0.25));
        sender.step();
        sync();
        const after = receiver.getCollider(collider.handle).shape as Ball;
        expect(after.radius).toBe(0.25);
        // The other colliders, moving or not, keep their cached shape objects.
        expect(cached.size).toBe(receiver.colliders.len());
        for (const [handle, shape] of cached) {
            if (handle != collider.handle) {
                expect(receiver.getCollider(handle).shape).toBe(shape);
            }
        }
        expect(
            receiver.getCollider(hullBody.collider(0).handle).parent().handle,
        ).toBe(hullBody.handle);

        sender.free();
        receiver.free();
    });

    test("a multibody joint replaced by another type gets a new wrapper", () => {
        const sender = new World(new Vector2(0, -9.81));
        sender.enableChangeTracking();
        const receiver = new World(new Vector2(0, 0));
        let ack = receiver.applyDelta(sender.takeDelta().delta);
        const sync = () => {
            const delta = sender.takeDelta(ack);
            ack = receiver.applyDelta(delta.delta);
            expectSameState(sender, receiver);
        };

        const root = sender.createRigidBody(RigidBodyDesc.fixed());
        const child = sender.createRigidBody(
            RigidBodyDesc.dynamic().setTranslation(0, -1),
        );
        sender.createCollider(ColliderDesc.ball(0.3), child);
        const anchor1 = new Vector2(0, 0);
        const anchor2 = new Vector2(0, 1);
        const revolute = sender.createMultibodyJoint(
            JointData.revolute(anchor1, anchor2),
            root,
            child,
            true,
        );
        sender.step();
        sync();
        expect(receiver.getMultibodyJoint(revolute.handle)).toBeInstanceOf(
            RevoluteMultibodyJoint,
        );

        // Within one ack window: the same child, so the same handle, with another joint type.
        sender.removeMultibodyJoint(revolute, true);
        const prismatic = sender.createMultibodyJoint(
            JointData.prismatic(anchor1, anchor2, new Vector2(1, 0)),
            root,
            child,
            true,
        );
        expect(prismatic.handle).toBe(revolute.handle);
        expect(prismatic).toBeInstanceOf(PrismaticMultibodyJoint);
        sender.step();
        sync();
        const replicated = receiver.getMultibodyJoint(prismatic.handle);
        expect(replicated).toBeInstanceOf(PrismaticMultibodyJoint);
        expect(replicated.isValid()).toBe(true);

        sender.free();
        receiver.free();
    });

    test("a full delta retypes the impulse joint wrappers of an independent world", () => {
        const anchor1 = new Vector2(0, 0.5);
        const anchor2 = new Vector2(0, -0.5);
        const build = (gravity: Vector2, data: JointData) => {
            const world = new World(gravity);
            const a = world.createRigidBody(RigidBodyDesc.dynamic());
            const b = world.createRigidBody(
                RigidBodyDesc.dynamic().setTranslation(0, 1),
            );
            const joint = world.createImpulseJoint(data, a, b, true);
            return {world, joint};
        };
        const sender = build(
            new Vector2(0, -9.81),
            JointData.revolute(anchor1, anchor2),
        );
        const receiver = build(
            new Vector2(0, 0),
            JointData.prismatic(anchor1, anchor2, new Vector2(1, 0)),
        );
        // Same handle, another joint type.
        expect(receiver.joint.handle).toBe(sender.joint.handle);
        expect(receiver.joint).toBeInstanceOf(PrismaticImpulseJoint);

        sender.world.enableChangeTracking();
        receiver.world.applyDelta(sender.world.takeDelta().delta);
        expectSameState(sender.world, receiver.world);
        const replicated = receiver.world.getImpulseJoint(sender.joint.handle);
        expect(replicated).toBeInstanceOf(RevoluteImpulseJoint);
        expect(replicated.isValid()).toBe(true);

        sender.world.free();
        receiver.world.free();
    });

    test("a reparented collider keeps its wrapper", () => {
        // rapier.js cannot reparent a collider: a full delta onto a world built with another
        // parent for the same collider handle does it instead.
        const build = (parentIndex: number) => {
            const world = new World(new Vector2(0, -9.81));
            const parents = [0, 1].map((i) =>
                world.createRigidBody(
                    RigidBodyDesc.dynamic().setTranslation(i * 3, 2),
                ),
            );
            const collider = world.createCollider(
                ColliderDesc.ball(0.5),
                parents[parentIndex],
            );
            return {world, parents, collider};
        };
        const sender = build(1);
        sender.world.enableChangeTracking();
        const receiver = build(0);
        const wrapper = receiver.collider;
        const shape = wrapper.shape;
        expect(wrapper.parent()).toBe(receiver.parents[0]);

        receiver.world.applyDelta(sender.world.takeDelta().delta);
        expectSameState(sender.world, receiver.world);
        const replicated = receiver.world.getCollider(sender.collider.handle);
        expect(replicated).toBe(wrapper);
        expect(replicated.parent()).toBe(receiver.parents[1]);
        expect(replicated.parent().handle).toBe(sender.parents[1].handle);
        // The shape did not change: the cached shape object is kept.
        expect(replicated.shape).toBe(shape);

        sender.world.free();
        receiver.world.free();
    });

    test("a delta missing a shape leaves the receiver untouched", () => {
        const {world: sender} = buildWorld();
        sender.enableChangeTracking();
        const snapshot = sender.takeSnapshot();
        const hullBody = sender.createRigidBody(
            RigidBodyDesc.dynamic().setTranslation(0, 8),
        );
        sender.createCollider(
            ColliderDesc.convexHull(
                new Float32Array([-0.4, -0.3, 0.4, -0.3, 0.5, 0.3, -0.3, 0.4]),
            ),
            hullBody,
        );
        const tick = sender.changeTick();
        sender.step();
        const delta = sender.takeDelta(tick);

        // A receiver wrongly declaring the baseline lacks the hull's shape, which the delta
        // omits: the shapes are checked before anything is patched.
        const receiver = World.restoreSnapshot(snapshot);
        receiver.setDeltaBaseline(tick);
        const before = receiver.takeSnapshot();
        const gravity = receiver.gravity;
        expect(errorKind(() => receiver.applyDelta(delta.delta))).toBe(
            "missing_shape",
        );
        expect(sameBytes(receiver.takeSnapshot(), before)).toBe(true);
        expect(receiver.gravity).toBe(gravity);
        expect(receiver.getRigidBody(hullBody.handle)).toBeNull();

        // A full delta recovers.
        receiver.applyDelta(sender.takeDelta().delta);
        expectSameState(sender, receiver);
        expect(receiver.getRigidBody(hullBody.handle).numColliders()).toBe(1);

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
            "invalid_bytes",
        );
        // Trailing bytes after a valid tick are rejected.
        const padded = new Uint8Array(first.tick.length + 1);
        padded.set(first.tick);
        expect(errorKind(() => sender.takeDelta(padded))).toBe("invalid_bytes");

        // A delta applied to a receiver that does not hold its baseline.
        const receiver = new World(new Vector2(0, 0));
        receiver.applyDelta(first.delta);
        receiver.applyDelta(second.delta);
        expect(errorKind(() => receiver.applyDelta(second.delta))).toBe(
            "baseline_mismatch",
        );
        expect(
            errorKind(() => receiver.applyDelta(new Uint8Array([1, 2, 3]))),
        ).toBe("invalid_bytes");
        expect(
            errorKind(() => receiver.setDeltaBaseline(new Uint8Array([1]))),
        ).toBe("invalid_bytes");

        // A delta against another baseline changes nothing, not even the gravity object.
        const gravity = receiver.gravity;
        const timestep = receiver.timestep;
        sender.gravity = new Vector2(0.5, -5);
        sender.timestep = 1 / 30;
        const mismatched = sender.takeDelta(first.tick);
        expect(errorKind(() => receiver.applyDelta(mismatched.delta))).toBe(
            "baseline_mismatch",
        );
        expect(receiver.gravity).toBe(gravity);
        expect(receiver.timestep).toBe(timestep);
        const truncated = mismatched.delta.slice(
            0,
            mismatched.delta.length - 1,
        );
        expect(errorKind(() => receiver.applyDelta(truncated))).toBe(
            "invalid_bytes",
        );
        expect(receiver.gravity).toBe(gravity);

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

    test("an empty baseline takes a full delta", () => {
        const {world: sender} = buildWorld();
        sender.enableChangeTracking();
        sender.step();
        const full = sender.takeDelta(new Uint8Array(0));
        const receiver = new World(new Vector2(0, 0));
        expect(sameBytes(receiver.applyDelta(full.delta), full.tick)).toBe(
            true,
        );
        expectSameState(sender, receiver);

        sender.free();
        receiver.free();
    });

    test("deltas and ticks carry a header", () => {
        const {world: sender} = buildWorld();
        sender.enableChangeTracking();
        const {delta, tick} = sender.takeDelta();
        const receiver = new World(new Vector2(0, 0));
        const rejected = (bytes: Uint8Array, message: RegExp) => {
            const error = deltaError(() => receiver.applyDelta(bytes));
            expect(error.kind).toBe("invalid_bytes");
            expect(error.message).toMatch(message);
        };
        const tampered = (offset: number, value: number) => {
            const bytes = delta.slice();
            bytes[offset] = value;
            return bytes;
        };

        rejected(tick, /expected a delta, got a tick/);
        expect(
            deltaError(() => receiver.setDeltaBaseline(delta)).message,
        ).toMatch(/expected a tick, got a delta/);
        rejected(tampered(0, 0), /bad magic/);
        rejected(tampered(4, 99), /format version 99/);
        rejected(tampered(6, 3), /3D build, this is a 2D build/);
        rejected(tampered(7, 8), /8-byte scalars/);
        rejected(delta.slice(0, 5), /too short/);

        // A rejected header leaves the receiver untouched.
        expect(receiver.bodies.len()).toBe(0);
        expect(receiver.colliders.len()).toBe(0);
        receiver.applyDelta(delta);
        expectSameState(sender, receiver);

        sender.free();
        receiver.free();
    });
});
