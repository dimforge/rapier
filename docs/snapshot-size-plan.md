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
NOT DONE — deliberately deferred. Rekeying the order-sensitive passes from edge ids to
handle pairs is not enough: the solver's constraint iteration order equals the
solver-bucket content order, which is maintained incrementally in event order and is
therefore itself history/layout-dependent. Making it canonical means either keeping
buckets sorted online (per-event insertion cost on the hottest maintenance path) or
changing the solve order semantics — both behavior- and perf-sensitive enough to be
their own project with its own A/B campaign. The estimated payoff is ~10% of the
remaining snapshot (graph threading, adjacency, `graph_indices`, hints, buckets,
island edge-id mirrors ≈ 60–70 B of ~600 B per touching pair), against tier 1's
already-banked 30–55%. Revisit only with a dedicated design pass.

### Phase 8 (tier 2b): broad-phase leaf-state-only serialization
DONE. The `DeletePair` batch was already canonically sorted; sorting the `AddPair`
batch the same way removed the last semantic leak of tree topology (edge-id
assignment order). The serialized broad phase is now the sorted leaves (index,
fattened AABB, resolved change flag), the sorted pair map, and the scalar regime
fields; the BVH is rebuilt canonically (`Bvh::from_iter`) on restore and the pair
adjacency is rebuilt from the pair map. Two pieces of genuine leaf state surfaced
during verification: the resolved change flags (they persist across steps in
full-refit regimes and gate both traversal and stale-pair detection), and the
*pending* change flags left by the pipeline's end-of-step `set_aabb` calls — the
latter are invisible through parry's public API but `pending_set_aabb` records
exactly those leaves, and a fresh post-build `insert` recreates pending state with
verbatim bounds. Spatial-query iteration order after a restore may differ from the
uninterrupted run (different, equally valid tree); it was never part of the
determinism contract.

### Phase 9 (tier 3): delta snapshots
DONE as a byte-level layer: `pipeline::snapshot_delta` / `apply_snapshot_delta` encode
one snapshot against another with rsync-style block matching (locality-first, hash
fallback), so unchanged bytes — sleeping islands, static state, interned shapes — cost
nothing on the wire regardless of serialization format. See the ledger's tick-delta
column. A semantic delta with CPU proportional to the awake set would need the
phase-7 canonical addressing first (slot renumbering leaks into sleeping state), which
is why it stayed byte-level.

## Ledger

Total snapshot bytes from the `snapshot_size` harness (release, f32, 3D), recorded
after each phase; the per-part breakdowns live in `snapshot-size-ledger-*.txt` next to
this file. Step time is avg ms/step on this machine, single-threaded, and must stay at
parity through phase 6.

| scene | baseline B | ph2 | ph5 (tier 1) | ph8 | tick delta (ph9) |
|-------|-----------:|----:|----:|----:|----:|
| pile_awake (193 bodies) | 816,106 | 813,830 | 773,468 | 734,492 | 82,539 |
| pile_sleepy (197) | 820,595 | 818,311 | 777,771 | 738,475 | 4,831 |
| mixed (122, joints+mb+ccd) | 481,390 | 467,006 | 441,435 | 419,623 | 8,090 |
| instanced_shapes (100+trimesh) | 3,301,765 | 1,855,580 | 1,498,316 | 1,473,040 | 16 |
| pyramid20 (2,871) | 11,572,748 | 11,538,336 | 8,472,206 | 8,123,406 | 17 |
| pyramid30 (9,456) | 39,296,184 | 39,182,752 | 28,698,945 | 27,530,649 | 19 |

Net through phase 8: −30% on box piles/pyramids, −55% on the shape-instanced scene
(intermediate per-phase totals live in the `snapshot-size-ledger-*.txt` files). The
wasm32 portability goldens track it (3D 481,608 → 419,695 B; 2D 88,572 → 77,496),
re-minted and verified under wasmtime on 2026-09-02, along with the
`parallel_path_parity` checksum (sequential and parallel builds agree on the new
value; the old golden also passed on this aarch64 machine, so the constant is
target-independent).

The tick-delta column is the phase-9 layer: one step encoded against the previous
snapshot. Settled scenes cost ~17 B/tick; the sleeping pile 0.7% of full; a fully
awake pile 11%. Encoding still pays a full serialize plus a linear diff (34 ms on the
27.5 MB pyramid30), so it is bandwidth-proportional, not CPU-proportional.

Perf: interleaved A/B of the harness across phase binaries on pyramid30 (12 runs)
reads base 10.21 ms, ph3 10.36, ph6/final ~10.4 — a possible ~1–1.5% step cost from
the phase-3 per-full-update bookkeeping (larger recycle state, replay bits, dead-field
canonicalization), at the edge of the ±1.5% run-to-run noise; pyramid20 measures
*faster* than baseline (1.68 vs 1.70 ms) and pile/mixed/instanced are at parity. Worth
a finer pass if it matters. Full 2D+3D suites green, plus the roundtrip suite under
`parallel` and `enhanced-determinism`.

Phase notes:
- ph1 (multibody skip) cut `multibody_joints` 20,780 → 7,720 in `mixed`.
- ph2 (shape interning) cut `colliders` 1,548,693 → 102,508 in `instanced_shapes`;
  box-only scenes save only ~12 B/cuboid as expected. Interning is by serialized
  content (identity-memoized), not `Arc` identity, so grouping stays canonical across
  restores and peers; the `shared_shapes` roundtrip test pins the pathological case.
- ph3/ph4 (solver-contact replay, plain + clusters) store one byte per solver contact.
  The replay poses come from an extended `ContactRecycleState` (both translations +
  skins, +32 B/pair); hook-modified pairs, offset colliders (non-identity
  `pos_wrt_parent`), and recycling-off worlds keep the verbatim encoding
  (`stored_solver_contacts` test).
- ph5 (compact points) drops the frozen lever arms (replayed) and stores
  `warmstart_tangent_world` per point only when it does not bit-match the recompute
  from the manifold normal's tangent basis (virgin never-solved points and stale-basis
  corners). Two live-side canonicalization passes were needed so dead per-point fields
  cannot leak divergent bytes through later verbatim-encoded snapshots.
- ph6 sorted the joint sets' `to_wake_up`/`to_join` serialization.

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
