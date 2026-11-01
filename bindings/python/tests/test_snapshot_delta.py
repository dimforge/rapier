"""Snapshot deltas: replicating a stepping world through `delta_since` / `apply_delta`."""

from __future__ import annotations

import pytest

import rapier3d as r


def _build_world():
    """A pile of boxes and balls on a ground, with impulse and multibody joints."""
    w = r.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
    w.add_collider(r.Collider.cuboid(20.0, 0.1, 20.0))
    bodies = []
    for i in range(12):
        x, z = (i % 3) * 1.1 - 1.1, (i // 3 % 2) * 1.1
        shape = r.Collider.ball(0.4) if i % 2 else r.Collider.cuboid(0.4, 0.4, 0.4)
        bodies.append(w.add_body(r.RigidBody.dynamic(translation=(x, 1.0 + i * 0.5, z)), [shape]))
    w.impulse_joints.insert(bodies[0], bodies[1], r.FixedJointBuilder().build())
    w.impulse_joints.insert(bodies[2], bodies[3], r.RevoluteJointBuilder(axis=(0, 0, 1)).build())
    # A small multibody pendulum hanging from a fixed anchor.
    anchor = w.add_body(r.RigidBody.fixed(translation=(5.0, 4.0, 0.0)))
    link = w.add_body(r.RigidBody.dynamic(translation=(6.0, 4.0, 0.0)), [r.Collider.ball(0.2)])
    joint = r.RevoluteJointBuilder(axis=(0, 0, 1)).local_anchor2((-1.0, 0.0, 0.0)).build()
    assert w.multibody_joints.insert(anchor, link, joint) is not None
    return w


def _edit(w, step, spawned):
    """Structural churn and parameter changes, so deltas carry more than poses."""
    if step % 7 == 0:
        spawned.append(
            w.add_body(
                r.RigidBody.dynamic(translation=(0.3 * (step % 5), 6.0, 0.2)),
                [r.Collider.cuboid(0.2, 0.3, 0.2)],
            )
        )
    if step % 11 == 0 and spawned:
        w.remove_body(spawned.pop(0))
    if step == 20:
        w.gravity = (0.5, -9.81, 0.0)


def _sync(sender, receiver, acked):
    delta, tick = sender.delta_since(acked)
    assert isinstance(delta, bytes) and isinstance(tick, bytes)
    assert receiver.apply_delta(delta) == tick
    return tick


def test_delta_replication_matches_sender_bytes():
    world = _build_world()
    world.enable_change_tracking()
    for _ in range(10):
        world.step()

    # (a) restored from a snapshot taken right after its baseline tick.
    restored_acked = world.change_tick()
    restored = r.PhysicsWorld.restore(world.snapshot())
    restored.set_delta_baseline(restored_acked)
    # (b) an empty world bootstrapped by a full delta.
    full, tick = world.delta_since(None)
    empty = r.PhysicsWorld()
    empty_acked = empty.apply_delta(full)
    assert empty_acked == tick
    assert empty.snapshot() == world.snapshot()
    # (c) a lagging receiver, synced every 5 steps against its own acknowledged tick.
    lagging_acked = world.change_tick()
    lagging = r.PhysicsWorld.restore(world.snapshot())
    lagging.set_delta_baseline(lagging_acked)

    spawned = []
    for step in range(60):
        _edit(world, step, spawned)
        world.step()
        restored_acked = _sync(world, restored, restored_acked)
        empty_acked = _sync(world, empty, empty_acked)
        expected = world.snapshot()
        assert restored.snapshot() == expected, f"restored receiver differs at step {step}"
        assert empty.snapshot() == expected, f"bootstrapped receiver differs at step {step}"
        if step % 5 == 4:
            lagging_acked = _sync(world, lagging, lagging_acked)
            assert lagging.snapshot() == expected, f"lagging receiver differs at step {step}"

    assert len(empty.rigid_bodies) == len(world.rigid_bodies)
    assert tuple(empty.gravity) == tuple(world.gravity)

    # The replicas keep simulating identically without further syncs.
    for step in range(60):
        world.step()
        restored.step()
        empty.step()
        expected = world.snapshot()
        assert restored.snapshot() == expected, f"restored replica diverged at step {step}"
        assert empty.snapshot() == expected, f"bootstrapped replica diverged at step {step}"


def test_delta_of_a_mostly_static_world_is_small():
    w = r.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
    for i in range(200):
        w.add_collider(r.Collider.cuboid(0.5, 0.5, 0.5).translation((i * 2.0, 0.0, 0.0)))
    w.add_body(r.RigidBody.dynamic(translation=(0.0, 3.0, 0.0)), [r.Collider.ball(0.5)])
    w.enable_change_tracking()
    # The first step registers every collider in the broad phase: start after it.
    w.step()
    _, acked = w.delta_since(None)
    w.step()
    delta, _ = w.delta_since(acked)
    assert len(delta) * 4 < len(w.snapshot())


def test_delta_since_requires_change_tracking():
    w = _build_world()
    with pytest.raises(r.DeltaError) as e:
        w.delta_since(None)
    assert e.value.kind == "tracking_disabled"

    w.enable_change_tracking()
    w.delta_since(None)
    w.disable_change_tracking()
    with pytest.raises(r.DeltaError) as e:
        w.delta_since(None)
    assert e.value.kind == "tracking_disabled"


def test_delta_since_rejects_foreign_or_stale_ticks():
    a, b = _build_world(), _build_world()
    a.enable_change_tracking()
    b.enable_change_tracking()
    _, tick_b = b.delta_since(None)
    with pytest.raises(r.DeltaError) as e:
        a.delta_since(tick_b)
    assert e.value.kind == "unknown_baseline"

    # Restarting the tracking invalidates every tick issued before.
    _, tick_a = a.delta_since(None)
    a.enable_change_tracking()
    with pytest.raises(r.DeltaError) as e:
        a.delta_since(tick_a)
    assert e.value.kind == "unknown_baseline"


def test_stale_delta_leaves_the_receiver_untouched_then_recovers():
    sender = _build_world()
    sender.enable_change_tracking()
    receiver = r.PhysicsWorld()
    t0 = receiver.apply_delta(sender.delta_since(None)[0])
    sender.step()
    t1 = receiver.apply_delta(sender.delta_since(t0)[0])

    # A delta against t0 no longer matches a receiver at t1, and its gravity change must not land.
    sender.gravity = (1.0, -3.0, 0.5)
    sender.step()
    stale, _ = sender.delta_since(t0)
    gravity, snapshot = tuple(receiver.gravity), receiver.snapshot()
    with pytest.raises(r.DeltaError) as e:
        receiver.apply_delta(stale)
    assert e.value.kind == "baseline_mismatch"
    assert isinstance(e.value, r.SerializationError)
    assert isinstance(e.value, r.RapierError)
    assert tuple(receiver.gravity) == gravity
    assert receiver.snapshot() == snapshot

    # The receiver still holds t1, so the matching delta applies.
    receiver.apply_delta(sender.delta_since(t1)[0])
    assert tuple(receiver.gravity) == tuple(sender.gravity)
    assert receiver.snapshot() == sender.snapshot()

    # A full delta always applies, whatever the receiver's state.
    sender.step()
    receiver.apply_delta(sender.delta_since(None)[0])
    assert receiver.snapshot() == sender.snapshot()


def test_missing_shape_leaves_the_receiver_untouched():
    sender = _build_world()
    sender.enable_change_tracking()
    old_state = sender.snapshot()
    # Hulls are sent by content, so a delta omits a hull its baseline already holds.
    hull = r.SharedShape.convex_hull([(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)])
    sender.add_body(r.RigidBody.dynamic(translation=(0.0, 8.0, 0.0)), [r.Collider.new(hull)])
    tick = sender.change_tick()
    sender.step()
    delta, _ = sender.delta_since(tick)

    # A receiver wrongly declaring that baseline lacks the hull: nothing is patched.
    receiver = r.PhysicsWorld.restore(old_state)
    receiver.set_delta_baseline(tick)
    before = receiver.snapshot()
    with pytest.raises(r.DeltaError) as e:
        receiver.apply_delta(delta)
    assert e.value.kind == "missing_shape"
    assert receiver.snapshot() == before

    receiver.apply_delta(sender.delta_since(None)[0])
    assert receiver.snapshot() == sender.snapshot()


def test_malformed_delta_and_tick_blobs_raise():
    sender = _build_world()
    sender.enable_change_tracking()
    delta, tick = sender.delta_since(None)
    receiver = r.PhysicsWorld()

    for garbage in (b"", b"not a delta", delta[:-7], delta + b"\x00"):
        with pytest.raises(r.SerializationError):
            receiver.apply_delta(garbage)
    # Deltas and ticks of another format version, with the right magic.
    for version in (1, 99):
        header = version.to_bytes(4, "little")
        with pytest.raises(r.SerializationError, match=f"version {version}"):
            receiver.apply_delta(b"RPYD" + header + delta[8:])
        with pytest.raises(r.SerializationError, match=f"version {version}"):
            sender.delta_since(b"RPYT" + header + tick[8:])
    for garbage in (b"garbage", tick[:-1], tick + b"\x00"):
        with pytest.raises(r.SerializationError):
            sender.delta_since(garbage)
        with pytest.raises(r.SerializationError):
            receiver.set_delta_baseline(garbage)
    # An empty tick is no baseline for delta_since, but not a baseline to declare.
    with pytest.raises(r.SerializationError):
        receiver.set_delta_baseline(b"")
    with pytest.raises(TypeError):
        receiver.apply_delta("not bytes")

    # The receiver is still usable.
    receiver.apply_delta(delta)
    assert receiver.snapshot() == sender.snapshot()


def test_empty_baseline_takes_a_full_delta():
    sender = _build_world()
    sender.enable_change_tracking()
    sender.step()
    for empty in (b"", bytearray(), memoryview(b"")):
        delta, tick = sender.delta_since(empty)
        receiver = r.PhysicsWorld()
        assert receiver.apply_delta(delta) == tick
        assert receiver.snapshot() == sender.snapshot()


def test_delta_methods_accept_buffers():
    sender = _build_world()
    sender.enable_change_tracking()
    full, acked = sender.delta_since(None)
    receiver = r.PhysicsWorld()
    assert receiver.apply_delta(bytearray(full)) == acked
    for wrap in (bytearray, memoryview):
        sender.step()
        delta, tick = sender.delta_since(wrap(acked))
        acked = receiver.apply_delta(wrap(delta))
        assert acked == tick
        assert receiver.snapshot() == sender.snapshot()

    # A restored receiver declares its baseline from a buffer too.
    tick = sender.change_tick()
    restored = r.PhysicsWorld.restore(sender.snapshot())
    restored.set_delta_baseline(memoryview(bytearray(tick)))
    sender.step()
    restored.apply_delta(memoryview(sender.delta_since(tick)[0]))
    assert restored.snapshot() == sender.snapshot()
    for not_a_buffer in ("not bytes", 3, None):
        with pytest.raises(TypeError):
            restored.apply_delta(not_a_buffer)


def test_blob_kinds_have_their_own_magic():
    sender = _build_world()
    sender.enable_change_tracking()
    delta, tick = sender.delta_since(None)
    snapshot = sender.snapshot()
    version = (2).to_bytes(4, "little")
    assert delta[:8] == b"RPYD" + version
    assert tick[:8] == b"RPYT" + version
    assert sender.change_tick()[:8] == b"RPYT" + version
    assert snapshot[:8] == b"RPYS" + version

    receiver = r.PhysicsWorld()
    acked = receiver.apply_delta(delta)
    before = receiver.snapshot()
    with pytest.raises(r.SerializationError, match="expected a delta, got a tick"):
        receiver.apply_delta(tick)
    with pytest.raises(r.SerializationError, match="expected a delta, got a snapshot"):
        receiver.apply_delta(snapshot)
    with pytest.raises(r.SerializationError, match="expected a delta, got a snapshot"):
        receiver.apply_delta(r.Vec3(1.0, 2.0, 3.0).to_bytes())
    with pytest.raises(r.SerializationError, match="expected a tick, got a delta"):
        receiver.set_delta_baseline(delta)
    with pytest.raises(r.SerializationError, match="expected a tick, got a delta"):
        sender.delta_since(delta)
    with pytest.raises(r.SerializationError, match="expected a tick, got a snapshot"):
        sender.delta_since(snapshot)
    with pytest.raises(r.SerializationError, match="expected a snapshot, got a delta"):
        r.PhysicsWorld.restore(delta)
    with pytest.raises(r.SerializationError, match="expected a snapshot, got a tick"):
        r.PhysicsWorld.restore(tick)

    # The rejected blobs left the receiver at its baseline.
    assert receiver.snapshot() == before
    sender.step()
    receiver.apply_delta(sender.delta_since(acked)[0])
    assert receiver.snapshot() == sender.snapshot()


def test_fresh_receiver_rejects_an_incremental_delta():
    sender = _build_world()
    sender.enable_change_tracking()
    tick = sender.change_tick()
    sender.step()
    delta, _ = sender.delta_since(tick)

    receiver = r.PhysicsWorld()
    with pytest.raises(r.DeltaError) as e:
        receiver.apply_delta(delta)
    assert e.value.kind == "baseline_mismatch"


def test_stepped_receiver_rejects_the_next_delta_then_recovers_with_a_full_delta():
    sender = _build_world()
    sender.enable_change_tracking()
    receiver = r.PhysicsWorld()
    acked = receiver.apply_delta(sender.delta_since(None)[0])

    sender.step()
    receiver.step()
    delta, _ = sender.delta_since(acked)
    with pytest.raises(r.DeltaError) as e:
        receiver.apply_delta(delta)
    assert e.value.kind == "baseline_mismatch"

    full, tick = sender.delta_since(None)
    acked = receiver.apply_delta(full)
    assert acked == tick
    assert receiver.snapshot() == sender.snapshot()
    sender.step()
    acked = _sync(sender, receiver, acked)
    assert receiver.snapshot() == sender.snapshot()


def test_restored_receiver_needs_a_declared_baseline():
    sender = _build_world()
    with pytest.raises(r.DeltaError) as e:
        sender.change_tick()
    assert e.value.kind == "tracking_disabled"

    sender.enable_change_tracking()
    for _ in range(5):
        sender.step()
    tick = sender.change_tick()
    assert isinstance(tick, bytes)
    snapshot = sender.snapshot()
    sender.step()
    delta, _ = sender.delta_since(tick)

    # The same delta is rejected by a restored receiver until its baseline is declared.
    receiver = r.PhysicsWorld.restore(snapshot)
    with pytest.raises(r.DeltaError) as e:
        receiver.apply_delta(delta)
    assert e.value.kind == "baseline_mismatch"

    receiver = r.PhysicsWorld.restore(snapshot)
    receiver.set_delta_baseline(tick)
    receiver.apply_delta(delta)
    assert receiver.snapshot() == sender.snapshot()


def _contact_summary(w):
    out = []
    for pair in w.narrow_phase.contact_pairs():
        for m in pair.manifolds:
            out.append(
                (
                    (pair.collider1.index, pair.collider1.generation),
                    (pair.collider2.index, pair.collider2.generation),
                    tuple(m.data.normal),
                    len(m.data.solver_contacts),
                )
            )
    return sorted(out)


def test_restored_world_has_contacts_before_stepping():
    world = _build_world()
    for _ in range(30):
        world.step()
    expected = _contact_summary(world)
    assert any(n > 0 for *_, n in expected)

    for restored in (
        r.PhysicsWorld.restore(world.snapshot()),
        r.PhysicsWorld.restore_json(world.snapshot_json()),
    ):
        assert _contact_summary(restored) == expected
