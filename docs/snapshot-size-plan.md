# Snapshot size reduction plan

Goal: shrink the serialized state needed for the simulate → serialize → deserialize →
simulate determinism cycle, so large simulations can be snapshotted cheaply (rollback
netcode, save files), without giving up byte-identical continuation and without
regressing per-step performance.

Contract, unchanged throughout: at every step after a restore, re-serializing the world
must yield the same bytes as the uninterrupted run (`snapshot_roundtrip` tests, 2D and
3D). Cross-target byte identity is pinned by `snapshot_portability` and must be
re-minted on `wasm32-wasip1` whenever the serialized format changes.

## Where the bytes go (baseline inventory)

1. Narrow phase: ~800 B per touching 3D pair. Manifolds (~100 B/point: 36 B tracked
   contact + 64 B `ContactData`), a second manifold-shaped copy in `solver_clusters`,
   `solver_contacts` (48 B each, serialized only because sleeping pairs are not
   refreshed; see the comment on `ContactManifoldData::solver_contacts`), 69 B
   `recycle_state`, composite-shape workspaces.
2. Collider shapes: `SharedShape` serializes full geometry per collider; `Arc` sharing
   is not preserved.
3. Broad phase: full BVH (nodes, parents, leaf indices, free lists, optimizer cursor)
   plus the pair map.
4. Multibodies: `O(4·ndofs²)` dense matrices and LU factorizations, jacobians, coriolis
   workspaces, all serialized (existing TODO in `multibody.rs`).
5. Bodies/colliders: ~300 B/body, mostly irreducible user state.

Structural constraint: determinism currently keys on storage layout (contact-graph edge
ids, BVH slot reuse), so layout is semantics and must ship. Tier 2 changes that.

## The key subtlety: replay poses, not current poses

Solver contacts, the manifold `normal`, and the frozen anchors (`solver_dp1/2`,
`warmstart_tangent_world`) are computed at the pair's *last full narrow-phase update*
and kept verbatim while the pair recycles. A restored awake pair may have drifted since
that update, so recomputing this data from *current* poses would not reproduce the
uninterrupted run. The recompute must replay from the stored inputs of the last full
update: manifold local data (unchanged since that update) plus the frozen rotations in
`ContactRecycleState` (`rot1`, `rot2`, `pos12`). Pairs without a `recycle_state`
(recycling disabled, or never fully updated) get a full refresh on the first
post-restore step in both runs, which is already the documented recycling-miss path.

Hooks: pairs with `ActiveHooks::MODIFY_SOLVER_CONTACTS` cannot be replayed without the
user's hook, which need not be pure. Those pairs keep serializing their solver contacts
(an `Option` tag per manifold, bincode-safe).

## Phases

Each phase is independently verified: full `snapshot_roundtrip` suite (3D + 2D) green,
sizes recorded in the ledger below, and a perf spot-check. Perf A/B gates apply to
phases 7 and 8 only; earlier phases must simply not touch the per-step path.

### Phase 0: measurement harness and baselines
A `snapshot_size` example in `crates/rapier3d/examples/` (and a 2D twin if useful):
builds the roundtrip scenes plus a mid-size pyramid, steps them, prints per-part
serialized sizes (the nine `PhysicsWorld` parts) and average ms/step in release mode.
Record the baseline ledger. Also record a baseline `pyramid(30)` step timing series as
the perf reference.

### Phase 1: multibody derived-state skip
`serde(skip)` on `Multibody`'s derived and workspace fields (`body_jacobians`,
`augmented_mass`, `inv_augmented_mass`, `acc_*`, `augmented_mass_indices`, `workspace`,
`coriolis_v`, `coriolis_w`, `i_coriolis_dt`), recomputed on deserialization from
`links` + `velocities` (pure function, no external inputs, so a custom `Deserialize`
finalizer suffices). Resolves the existing TODO. Validates the skip-and-rebuild pattern
on the easiest structure.

### Phase 2: shape interning
Custom serde for `ColliderSet`: serialize a table of distinct shapes (deduped by `Arc`
pointer identity) and per-collider indices into it. Deserialization reconstructs the
sharing (a bonus: today a restore loses it and duplicates memory). `Collider`'s own
derive stays as-is for users serializing colliders individually. Biggest win for
instanced trimesh/convex-decomposition scenes.

### Phase 3: solver-contact recompute
Stop serializing `ContactManifoldData::solver_contacts` and `normal`; rebuild them on
the first post-restore step via the replay described above, factored out of the
existing full-update path. Restored-ness is tracked by a `serde(skip)` flag defaulting
to "needs rebuild" so it never appears in the bytes. Hook-modified pairs keep
serializing theirs.

### Phase 4: solver-cluster compaction
`solver_clusters` geometry is a deterministic reduction of `manifolds`; only the
applied impulses in its `ContactData` are state. Serialize clusters through a compact
`serde(with)` representation (per-point warm-start data plus whatever matching keys the
cluster rebuild needs) and replay the geometry on restore.

### Phase 5: frozen-anchor skip
Drop `solver_dp1`, `solver_dp2`, `warmstart_tangent_world` from the serialized
`ContactData` via a compact representation, re-deriving them in the same replay pass as
phase 3 (they are functions of the manifold local data and the frozen rotations).

### Phase 6: hygiene
Sorted serialization for `ImpulseJointSet::to_wake_up` / `to_join` (and the multibody
twins): the last unsorted hash containers in the snapshot. Normally empty, but two
snapshots of identical state must not differ.

### Phase 7 (tier 2a): canonical rekeying, then stop shipping the contact graph
Rekey every order-sensitive pass (pair transitions, event emission, island
link/unlink, colouring seed order, solver-bucket order) from contact-graph edge ids to
sorted collider-handle pairs. Once layout is semantically inert, serialize the contact
graph as a sorted pair list + payloads and rebuild threading, adjacency,
`graph_indices`, and the solver buckets canonically on restore; the persistent-island
edge-id mirrors rebuild with it. Perf gate: end-to-end A/B on the harness scenes before
and after; any regression on the sorted passes' hot path blocks the phase.

### Phase 8 (tier 2b): broad-phase leaf-state-only serialization
Establish that pair appearance depends only on per-leaf fattened AABBs (not tree
topology or optimizer timing), then serialize just the fattened AABBs, the pair map,
and the few scalar regime fields, rebuilding the tree canonically on restore. Perf
gate: broad-phase timings on the harness scenes plus a large-world spot check.

### Phase 9 (tier 3): delta snapshots
A snapshot layer for per-tick use: one full baseline plus per-tick deltas carrying only
awake-island state and the structural-change journal, applied onto a baseline world.
Sleeping state is frozen by construction, so the recurring cost is proportional to the
awake set. Design details decided when reached; the roundtrip contract extends to
"baseline + deltas replayed = uninterrupted run".

## Ledger

Total snapshot bytes from the `snapshot_size` harness (release, f32, 3D), recorded
after each phase; the per-part breakdowns live in `snapshot-size-ledger-*.txt` next to
this file. Step time is avg ms/step on this machine, single-threaded, and must stay at
parity through phase 6.

| scene | baseline B | ms/step |
|-------|-----------:|--------:|
| pile_awake (193 bodies) | 816,106 | 0.152 |
| pile_sleepy (197) | 820,595 | 0.009 |
| mixed (122, joints+mb+ccd) | 481,390 | 0.102 |
| instanced_shapes (100+trimesh tiles) | 3,301,765 | 0.088 |
| pyramid20 (2,871) | 11,572,748 | 1.796 |
| pyramid30 (9,456) | 39,296,184 | 11.496 |

Baseline observations: narrow phase is 73–77% of the box scenes (pyramid30: 30.2 MB of
39.3 MB); `colliders` is 47% of instanced_shapes (1.55 MB for 17 distinct shapes);
`bodies` runs ~410 B/body and `islands` ~130 B/body on contact-dense scenes.

## Risks / notes

- `snapshot_portability` GOLDEN must be re-minted under `wasm32-wasip1` after every
  format change (wasmtime required; aarch64-native digests diverge from the golden
  domain and must not be used to mint).
- bincode is not self-describing: only unconditional `serde(skip)` or explicit `Option`
  tags are allowed; `skip_serializing_if` is forbidden.
- Phase 3 replay must reproduce clustering decisions bit-exactly; if any cluster
  matching turns out to depend on state not kept, fall back to keeping that key
  serialized and re-measure.
- Phases 7/8 are reversible individually: if the A/B shows a regression, keep the
  serialization win off for that structure and retain the earlier phases.
