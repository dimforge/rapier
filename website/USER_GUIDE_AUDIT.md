# User-guide audit

Audit of `docs/user_guides/templates/` against the engine on the `soft-bodies` branch (rapier 0.35.3,
`@dimforge/rapier` 0.17.3 as pinned by `docs-examples`). Every API name and default value quoted by the guide was
checked against `src/` and `typescript/src.ts`.

One commit per item.

## Outdated content

- [x] 1. `docs-examples`: the `[patch.crates-io]` pointing at a sibling parry checkout is stale since the switch to
      parry 0.31, and breaks `generate_user_guides.sh` outside the main checkout (fixed on master already).
- [x] 2. `rigid_body_ccd.mdx`: CCD is no longer disabled by default (fast dynamic bodies sweep fixed colliders and
      soft-body meshes), `ccd_enabled` is the bullet upgrade, the motion-clamping description and the role of
      `max_ccd_substeps` (including `0` as the global switch) are stale.
- [x] 3. `common_mistakes.mdx`: the documented broad-phase panic no longer exists; non-finite state is now contained by
      the quarantine instead.
- [x] 4. `common_mistakes.mdx`: triangle-mesh colliders do get their mass properties computed; only polylines,
      half-spaces, segments and (in 3D) triangles have a zero mass.
- [x] 5. `collider_friction.mdx` + `collider_restitution.mdx`: `ClampedSum` and `GeometricMean` are missing from the
      combine rules, and the rule precedence is incomplete.
- [x] 6. `scene_queries_point_projection.mdx` + `scene_queries_intersection_test.mdx`: `QueryPipeline` methods renamed
      to `intersect_point`, `intersect_shape` and `intersect_aabb_conservative`.
- [x] 7. `determinism.mdx`: `World.createSnapshot` doesn't exist (it is `takeSnapshot`), and the cross-platform
      determinism of the parallel solver isn't mentioned.
- [x] 8. `rigid_body_sleeping.mdx`: `World.removeJoint` is now `removeImpulseJoint`/`removeMultibodyJoint`.
- [x] 9. `getting_started.mdx`: the cargo features list mentions the removed `wasm-bindgen` feature and misses
      `simd8`, `fem`, `block-solver`, `unsync-callbacks`, `debug-render` and `profiler`.
- [x] 10. `advanced_collision_detection.mdx`: `ChannelEventCollector` uses `std::sync::mpsc` channels, not crossbeam,
      and takes a third sender for the soft-body tear events.
- [x] 11. `simulation_structures.mdx`: the `SoftBodySet` is missing from the structures needed by the pipeline, and the
      `PhysicsWorld` isn't mentioned at all.
- [x] 12. `sidebar_docs.js`: the Rust user-guide is labelled 0.32 instead of 0.35.
- [x] 13. `integration_parameters.mdx`: only `dt` and `min_ccd_dt` still exist among the documented parameters. The
      page must be rewritten against the current fields and re-enabled in the sidebar.

## Missing content

- [x] 14. The `PhysicsWorld`, i.e., the structure owning every set of one simulation, and the recommended entry point
      since the getting-started example still wires every structure by hand.
- [x] 15. The vehicle controller (`DynamicRayCastVehicleController`).
- [x] 16. The PID controller, i.e., the building block for velocity-based character controllers.
- [x] 17. The debug-renderer of Rapier itself (the guide only mentions the one of the Bevy plugin).
- [x] 18. The per-rigid-body solver settings: additional solver iterations, additional PGS iterations, soft-CCD
      prediction, fast-rotation and gyroscopic forces.
- [x] 19. The contact skin of the colliders, and the one-sided (oriented) triangle-meshes and polylines.
- [x] 20. The scene loaders: `rapier3d-urdf`, `rapier3d-mjcf` and `rapier3d-meshloader`.
- [x] 21. The Python bindings, which live in `python/` with their own documentation.
- [x] 22. `the_rapier_testbed.mdx` contains nothing but its front matter.
- [x] 23. `common_recipes.mdx` contains nothing but its section titles.

## Examples

- [x] 24. The Rust examples build every structure of the simulation by hand instead of using the `PhysicsWorld`, which
      makes them longer than needed and hides the recommended entry point.

## Deferred

- The Bevy plugin pages (including the `simd-stable` feature of `getting_started_bevy.mdx`) are left untouched until
  `bevy_rapier` is updated.
- `docs-examples` pins `@dimforge/rapier` 0.17.3 while 0.20.0 is released. Bumping it is not a documentation change:
  the JavaScript snippets must be adapted to the breaking changes of the three releases in between.
