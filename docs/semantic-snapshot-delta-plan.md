# Semantic snapshot deltas plan

Goal: per-tick snapshot deltas whose **encode cost, wire size and apply cost** scale with
what changed since a baseline tick (roughly the awake set plus user edits), not with the
world. Unchanged state is never serialized, never scanned byte by byte, and never
deserialized on the receiver.

This replaces the byte-level `pipeline::snapshot_delta` (phase 9 of
`snapshot-size-plan.md`). That layer always pays a full serialize plus a linear scan, and
its encoder costs ~0.8 s per tick on an awake 28 MB pyramid. It stays only as a
benchmark baseline until phase 6, then is removed from the public API.

Use case: a server or peer that simulates sends state to receivers that may not simulate
(server-authoritative replication, late joiners, relays). Input recording (the
box2d/box3d approach) was considered and set aside: it needs every receiver to run
rapier deterministically with identical hooks.

## Two API levels

Deltas work on each serializable component on its own, not only through `PhysicsWorld`:
`RigidBodySet`, `ColliderSet`, `ImpulseJointSet`, `MultibodyJointSet`, `IslandManager`,
`BroadPhaseBvh`, `NarrowPhase` and `SoftBodySet`. Many users drive `PhysicsPipeline::step`
with the individual sets and never use the wrapper.

`PhysicsWorld`'s delta is a composition of its components' deltas plus its small scalars
(gravity and `IntegrationParameters`, sent whole when they differ). It adds no logic of
its own.

## Contract

The contract holds per component, and therefore for the world. For a source world `S`
and a receiver `R` holding `S`'s state at tick `T0`:

```
R.apply_delta(S.delta_since(T0))  ⇒  full_serialize(R) == full_serialize(S)
```

The comparison is byte-for-byte, at every tick, for any `T0` the source can still
answer (not only the previous tick), and after any user edits or structural changes.
After that, `R` and `S` continue byte-identically, exactly like a full snapshot restore.
The existing `snapshot_roundtrip` machinery (`record` / `assert_same`) is the oracle.

## What the audit found

Three read-only audits of the serialized containers (2026-10-28) drive the design.

1. **Existing change tracking only covers user edits.**
   - `modified_bodies` / `modified_colliders` record user mutations and are consumed
     early in `step`.
   - Nearly all pipeline writes go through untracked `get_mut_internal` /
     `index_mut_internal`.
   - Impulse joints and multibodies have no durable tracking at all.
2. **Most step writes are confined to the awake set**, with one timing trap:
   `update_islands` puts bodies to sleep *before* the solver runs (`solve.rs:310`).
   Bodies that fell asleep this step were still written, so the dirty set must be the
   union of awake sets across the step, CCD passes included.
3. **Sleeping or unrelated bytes still change through index fixups.**
   - Each `swap_remove` rewrites exactly one other entry: the awake-set Vec, persistent
     island `bodies` / `contact_links` / `joint_links`, `contact_link_locs`, solver
     buckets, `force_event_pairs`, and the `Graph` edges and nodes of the contact,
     intersection, joint and multibody-connectivity graphs.
   - `Graph` edge removal also rewrites `next` / `prev` / `node` links in up to ~4
     nodes and ~8 neighbour edges, sleeping ones included.
   - Persistent-island merges rewrite every member of the smaller island. Two
     *sleeping* islands can merge through a joint without waking.
   - Multibody insert/remove renumbers `rb2mb`, link ids and graph ids of possibly
     unrelated multibodies.
4. **Derived fields defeat naive stamping.** A full solver-graph rebuild rewrites
   `graph_pos`, `solver_body_ids` and the manifold `solver_color` on every pair, and
   writes `graph_pos = NONE` on all non-qualified ones. Rebuilds trigger on any island
   epoch bump and on any fully-updated composite pair (trimesh, heightfield, compound),
   which is nearly every step in mesh scenes.
5. **Interned shape ids are positional.** `ColliderSet`'s shape table assigns ids in
   arena-slot order, so changing or removing one shape can renumber the shape id of
   every later collider.
6. **Per-step global churn is small and bounded**: broad-phase scalars, `frame_index`,
   `prev_updated_leaves`, `pending_set_aabb`, `sleep_scan_stamp`, `split_island`, island
   epochs. Sleeping soft bodies are byte-stable after their first sleeping step.
7. **The broad-phase optimizer and refits never change leaf data** of leaves that did not
   move. A leaf that moved at tick `t` shows a change-flag flip at `t+1`, so leaf dirtiness
   is `prev_updated ∪ curr_updated`.

## Design

### Stamps, not dirty lists

- Every serialized slot gets a `u32` "last written at tick" stamp. A delta since `T0`
  sends the slots with `stamp > T0`. This answers deltas against any older acked tick
  (needed for netcode acks), which a reset-on-read dirty list cannot.
- Stamps live in `serde(skip)` side arrays owned by each container (`ChangeStamps`), not
  in the serialized structs. This is what makes per-component deltas possible: no state
  lives in `PhysicsWorld` or `PhysicsPipeline`.
- Tracking is opt-in per world. When off, the arrays are empty and every stamp site is
  one predictable branch.
- Side arrays mirror their container's structural operations:
  - `Arena` / `Coarena` slots: stamp on insert, remove and write.
  - `Graph` edges and nodes: on `swap_remove`, the stamp array is `swap_remove`d too; the
    moved slot and every relinked neighbour are stamped, and the length goes in the header.
  - History-ordered vectors (solver buckets, `force_event_pairs`, the awake-set Vec,
    `contact_link_locs`, island link lists): a `StampedVec` with one stamp per 64-entry
    chunk plus a length.
- Stamping can be **coarse where cheap and correct**. Bodies and colliders are stamped as
  "awake at any point in the step" plus tracked user edits plus explicit index-fixup
  choke points. Hooking every internal write is not required.

### Per-component change clocks

There is no world-level tick, because a component used on its own has no world to ask.

- Each tracked component owns a `u32` change clock, read-advanced like Bevy's change
  ticks. Writes are stamped with the current clock value.
- `delta_since(since)` returns the slots stamped in `(since, clock]`, together with the
  new baseline tick, and advances the clock. Writes made after the call are stamped
  later.
- No pipeline involvement is needed, so the same rules hold whether a component is
  stepped by `PhysicsPipeline`, edited by the user between steps, or both.
- Users keep one acked tick per component. `PhysicsWorld` bundles them in a
  `WorldDeltaTick` (one tick per component), so world users see a single opaque value.
- Wrap-around: at one tick per step and 60 Hz, `u32` lasts about two years. Stamps are
  compared as wrapping ages, and a baseline older than the oldest answerable tick is
  rejected with a "send a full snapshot" error.

### Cross-component references

Components reference each other's handles and indices. For example, `RigidBodyIds`
stores island indices, and the narrow phase's replay reads bodies and colliders.

- A write into another component's data is stamped in the component that owns the bytes.
  For example, the island manager writing `RigidBodyIds` stamps the body slot in
  `RigidBodySet`.
- Post-apply fixups that need other components stay deferred to the next step, exactly as
  after a full deserialization. The compact solver-contact replay, for instance, runs at
  the next narrow-phase update through `pending_replay`.
- So each component's `apply_delta` only needs that component. A receiver replicating
  several components applies all their deltas before stepping, as it would with full
  snapshots of each component.

### Compare before stamping, or stop serializing

Derived fields that full rebuilds rewrite wholesale leave the serialized format and are
rebuilt on restore and on apply: `graph_pos` (the inverse of the buckets), manifold
`solver_color` and `solver_body_ids`, `Edge.prev` (derivable from `next` plus node heads),
and `force_event_pos` (the inverse of `force_event_pairs`). This also shrinks full
snapshots.

Where a rebuild rewrites real state (bucket order after a full rebuild), stamp by chunk
only where the new content differs from the old.

### Stable shape ids

A `ColliderSet` delta refers to shapes by their 128-bit content key, which both sides can
compute. This works whether the receiver was seeded from a full snapshot or from earlier
deltas.

- The sender memoizes keys by `Arc` identity and records, per shape, the tick at which it
  first appeared in the set.
- A delta sends shape bytes only for shapes that first appeared after `since`; instanced
  shapes are deduplicated within the delta, as in full snapshots.
- Every other shape is sent as its key, and the receiver resolves it against shapes it
  already holds. A miss is a desync error.
- When a key is first registered, it is confirmed by byte comparison, as box3d's geometry
  registry does.

Full snapshots keep their own content-interned table, unchanged.

### Delta format

```
header: from_tick, to_tick, format version, target state hash
per container: header fields (lengths, free-list heads, generations, scalars),
               then [(slot index, payload)] for stamped slots
```

- A payload uses the same serde form as that element in a full snapshot (the custom
  `pair_serde`, the interned collider form, and so on), so there is no second encoding to
  keep in sync.
- Removed and moved slots are just slots whose new content is `Free` or the moved record.
- Counts are bounds-checked before any allocation. Box2D's snapshot reader does this;
  the byte delta does not.

### Apply in place

- Each component's `apply_delta(&mut self, &[u8])` patches slots and lengths, then runs
  its own post-deserialization fixups only for what was patched.
- `PhysicsWorld::apply_delta` applies its components' deltas in order.
- The fixups are:
  - compact solver-contact replay through `pending_replay`;
  - multibody derived state for patched multibodies;
  - broad-phase tree inserts, removals and updates for patched leaves;
  - `pair_adjacency` for patched pairs.
- The receiver's world then serializes byte-identically to the source's. The tree topology
  may differ, as with full restores today.
- Applying a delta stamps the slots it patched, so a receiver can relay deltas downstream.
- It rejects a delta whose `from_tick` doesn't match the receiver's tick, and checks the
  target state hash.

### State hash

`PhysicsWorld::state_hash()` in two variants, following box2d:
- cheap: poses and velocities;
- deep: adds contact and joint impulses and internal indices.

It is used for desync detection and to validate deltas.

## Phases

Every phase keeps the full `snapshot_roundtrip` suite green (2D, 3D, `parallel`,
`enhanced-determinism`) and adds the delta oracle below. Size and encode/apply times go
in the ledger.

### Phase 0: pre-existing issues the audit reported (verify on master, fix separately)

Each item gets a failing test first; only real bugs are fixed, as standalone commits that
can go to master.

1. Persistent-island `bootstrapped` is serialized although its doc says the islands are
   not (`persistent.rs:186-189`). Pending `island_events` / `island_chain_events` /
   `modified_joints` are `serde(skip)`, so a snapshot taken between a joint insert/edit
   and the next step may lose island link events. Test: snapshot right after
   `insert_joint`, then compare continuation.
2. `RigidBodySet::iter_mut` / `ColliderSet::iter_mut` clear the modified list eagerly.
   A partially consumed iterator leaves bodies flagged `IN_MODIFIED_SET` but out of the
   list, so their changes are lost permanently.
3. `ColliderSet::set_modified` overwrites colliders pushed mid-step by
   `Quarantine::apply_end_step` (same stuck-flag outcome).
4. `remove_collider` does not release the solver colours of the pairs it removes, so stale
   bits stay in surviving bodies' `body_solver_color_masks`.
5. `CollisionPipeline` never runs solver-graph maintenance, so the serialized
   `force_event_flagged` grows without bound there (and should be `serde(skip)` anyway).
6. Multibody forward kinematics rewrites every multibody each step, sleeping ones
   included. Verify the rewrite is bit-idempotent (the `set_free_pos` →
   `body_to_parent` round-trip could flip `±0.0`). If it isn't, it is already a
   sleeping-state churn bug.

### Phase 1: framework, coarse everywhere

- `ChangeStamps` and `StampedVec` infrastructure, per-component change clocks,
  enable/disable, the delta header, and `delta_since` / `apply_delta` on every component.
- `PhysicsWorld` composition with `WorldDeltaTick`.
- Every container starts in **coarse mode**: one container-level stamp, resent whole when
  bumped. Bumps are conservative: any step bumps every container except gravity and
  integration parameters.
- No size win yet. The point is a correct end-to-end path and the oracle.

The oracle (`check_delta_roundtrip`) runs at both levels: world deltas, and per-component
deltas applied to individually deserialized sets that are then stepped with
`PhysicsPipeline::step` directly. It covers:
- deltas every tick;
- deltas spanning several ticks;
- deltas against an older acked tick;
- structural churn (the `structural_changes` and `shared_shapes` scenes);
- joints, multibodies, CCD, sensors and soft bodies.

The roundtrip-scene builders are reused.

### Phase 2: bodies and colliders per slot

- Bodies are stamped as the union of awake sets during the step, CCD passes included,
  plus user edits (`get_mut` paths) and these index-fixup choke points:
  - `merge_islands`, `remove_body_raw`, `move_component_out`, `split_island_now`;
  - `extract_sleeping_sub_island`, `commit_sleeping_chunks`;
  - `wake_up` / `wake_up_island`, including the disabled-body case;
  - `update_substep_groups` reorders;
  - multibody writebacks.
- Colliders are stamped as colliders of stamped bodies, plus user edits, removals,
  quarantine and soft-body sync.
- Stable shape registry for deltas.
- Ledger check: a sleeping pile should send roughly the awake movers only.

### Phase 3: narrow phase per pair

1. Drop the derived fields from serialization (see the design section) and rebuild them
   on restore and apply. This reduces full snapshot size and must keep the portability
   goldens consistent (re-mint on wasm32-wasip1).
2. Per-edge stamps at the ~10 pair-write choke points from the audit:
   - after the recycle early-return in `process_pair`;
   - transitions and deferred colouring;
   - the recolour loop in `handle_user_changes_on_colliders`;
   - `reconcile_pair` and the parallel scatter (compare before stamping);
   - `remove_and_fixup`'s moved entry;
   - after the solve, every bucket entry (each one is written by the impulse writeback);
   - `contact_pair_mut` for the soft writeback;
   - `emit_contact_force_events`;
   - `add_pair` / `remove_pair`;
   - intersections.
3. Graph link stamps in `add_edge`, `unlink_edge`, `relink_edge`, the edge `swap_remove`
   and the `remove_node_with` walk (an optional hook, since `Graph` is shared with
   joints).
4. `StampedVec` for buckets, hints, `force_event_pairs`, `body_solver_color_masks` and
   `graph_indices`.
5. Ledger check: an awake pile's delta is about the awake pairs; mesh scenes do not resend
   every pair after composite-driven rebuilds.

### Phase 4: islands and broad phase

- **Islands:** per-island stamps for both the active-set chunks and the persistent
  islands, `StampedVec` for `contact_link_locs` and the link lists, and global scalars
  always sent.
- **Broad phase:**
  - per-leaf stamps for `prev_updated ∪ curr_updated`, inserts, removals and
    reinsertions;
  - pair add/remove from the `AddPair` / `DeletePair` batches plus the silent
    removed-collider drops;
  - scalars and the small `prev_updated_leaves` / `pending_set_aabb` lists always sent.
- **Receiver side:** incremental tree updates instead of a rebuild, keeping the pending
  change-flag trick of `bp_serde`.

### Phase 5: joints, multibodies, soft bodies

- Impulse joints are stamped per edge: selected (awake) joints after the solve, plus user
  edits. This needs new durable tracking, because `get_mut` only feeds serde-skipped
  island events today. Graph link stamps come from phase 3.
- Multibodies are stamped per multibody slot: awake roots, user edits through
  `get_mut` / `get_multibody_mut` (currently untracked), and every multibody touched by
  insert/remove renumbering, including `rb2mb` entries.
- Soft bodies are stamped per soft body while awake, plus tears, splits and user edits;
  a sleeping soft body costs nothing.

### Phase 6: API, hardening, bindings, perf gate

- Public API and docs, on every component and on `PhysicsWorld`:
  `enable_change_tracking`, `delta_since(tick)`, `apply_delta`, and `state_hash` (world
  and per component). Error types cover wrong baseline, baseline too old, missing shape,
  malformed input and version mismatch.
- Docs include an example that replicates the individual sets without `PhysicsWorld`.
- Fuzz the apply path: truncation, corrupt counts, wrong baselines.
- Bindings: TS (`serialization_pipeline.rs`), Python, C.
- Perf gate: harness A/B with tracking off must be at parity, and the tracking-on step
  overhead is measured and reported per scene.
- Remove the byte-level `snapshot_delta` from the public API.

## Measurement

- The `snapshot_size` harness gains columns for semantic delta size, encode time and apply
  time per tick, next to full size and the byte-delta baseline.
- Targets:
  - settled scenes: tens of bytes per tick and microseconds to encode;
  - awake scenes: roughly the awake bodies and pairs payload, at a small fraction of a
    full serialize;
  - tracking off: no measurable step cost.
- The ledger also gets a 2D twin of the harness if the 2D numbers diverge.

## Risks

- **Missed stamps desync silently.** Mitigations:
  - the oracle in every phase;
  - an env-gated validator (`RAPIER_DBG_VALIDATE` style) that mirrors deltas in
    ordinary runs;
  - coarse-first rollout, so each container only becomes fine-grained once its oracle is
    green.
- **Hot-path cost of stamping.** Stamps are plain stores into side arrays indexed by data
  already in hand. Parallel stages stamp disjoint slots (per edge, per body) or defer to
  a serial post-pass (the bucket walk after the solve).
- **Receiver fixups must also stay proportional.** Phase 1 allows O(world) fixups on
  apply; later phases make each one incremental.
- **Format churn.** Every serialized-format change re-mints both `snapshot_portability`
  goldens (wasm32-wasip1) and `parallel_path_parity`. Phase 3.1 is one such change.

## Status (2026-10-28): implemented

All phases landed on `snapshot-size`. Deviations from the plan above:

- **Phase 0 outcome.** Five reported issues were real and are fixed, each with a regression
  test: pending island events lost by snapshots, `iter_mut` stuck flags, the quarantine's
  overwritten modified set, collider removal leaking solver colors, and unbounded
  `force_event_flagged` under `CollisionPipeline`. Multibody forward kinematics on sleeping
  multibodies is byte-stable, so not a bug.
- **No coarse phase.** Every component went straight to slot granularity, each verified by
  the oracle as it landed.
- **Stamping strategies.** Three are used, per container:
  - stamps at the write sites, built into `Arena`, `Coarena`, `Graph`, `StampedVec` and
    `StampedVecMap` (every mutable accessor stamps);
  - superset stamping where writes go through raw pointers: the awake set and its
    colliders at the start and end of a step, every narrow-phase update candidate, every
    solver-graph member after the solve;
  - shadow comparison (`ShadowStamps`) for the narrow phase's small per-edge and per-body
    arrays and solver buckets, which are written from many places, some in parallel.
- **Derived fields stay serialized.** Instead of dropping `graph_pos`/`solver_body_ids`/
  manifold colors from the format, the solver-graph maintenance compares old and new values
  inline and stamps only the pairs it changed.
- **Value-neutral passes run unstamped.** The per-step multibody forward kinematics and the
  soft-body sync loops stamp only what they actually change.
- **Shapes.** Collider deltas reference heavy shapes by content key; the sender only includes
  bytes when no collider unchanged since the baseline holds that content, the receiver
  resolves keys from a registry of shapes it holds. `Collider::shape_revision` (not
  serialized) lets the key memo see in-place shape mutations.
- **Ticks.** One read-advanced clock per component (`ChangeTick<C>`, typed by the component,
  with a tracker id); `PhysicsWorld` bundles them in `WorldChangeTick`.

Fixes to the snapshot-size work found by the oracle:

- Parry's bulk BVH build assumes leaf ids 0 and 1 for one or two leaves: the broad phase
  inserts such small trees leaf by leaf on restore.
- Re-saving a restored world before stepping it changed the bytes of pairs awaiting their
  solver-contact replay: their compact encoding is now reproduced from the pending set.
- Solver contacts of multibody links were replayed from collider poses that the per-step
  forward kinematics had already moved away from the body poses: replay now requires the
  body pose to equal the collider pose bit for bit.

### Measured (3D harness, f32, this machine)

| scene | full snapshot | per-tick delta (avg) | encode | apply |
|---|---:|---:|---:|---:|
| instanced_shapes (settled) | 1,481,031 | 1,047 | 0.002 ms | 0.31 ms |
| pile_sleepy (4 permanent movers) | 750,226 | 20,287 | 0.02 ms | 0.05 ms |
| mixed | 426,374 | 19,450 | 0.01 ms | 0.03 ms |
| pile_awake (everything awake) | 746,111 | 672,033 | 0.45 ms | 0.67 ms |
| pyramid30 (everything awake) | 28,185,467 | 25,515,967 | 24.8 ms | 32.0 ms |

Settled scenes cost about 1 KB per tick, mostly fixed per-component headers. A fully awake
scene changes nearly everything every tick, and its delta is about 90% of a full snapshot.
Step cost with tracking disabled: parity with the pre-delta commit (−0.3%, 8 interleaved
pyramid30 rounds). With tracking enabled: about +1%.

### Known limitations

- **Composite contact workspaces in default builds.** Parry serializes them (compound,
  trimesh soft surfaces, voxels) in hash-map order, which depends on the map's growth
  history. A receiver rebuilds the maps of the pairs it patches, so its bytes can differ from
  the sender's while the content is identical. With `enhanced-determinism` (insertion-ordered
  maps) bytes match. The fix belongs in parry: serialize these maps sorted by key.
- **Receiver-side broad-phase cost.** Applying a broad-phase delta rebuilds the receiver's
  tree from its leaves (O(n log n)), like a deserialization. Incremental tree patching would
  make it proportional to the change.
- **Encode cost of awake scenes.** Encoding clones the changed slots, about a full
  serialization when everything is awake.
- **Bindings.** The TypeScript, Python and C bindings don't expose deltas yet.
