# Docs examples

This folder exists to prove examples used in the documentation compile.

By running `./generate_user_guide.sh` at the root path, These examples are copied and injected within a new folder containing the whole documentation pages.

## Rust

The produced binary might or might not produce something useful, it's
intended usage is to run `cargo check --workspace --examples`.

## Javascript

The snippets build against the bindings **of this repository**, not the ones published on
npm, so a snippet may use an API before it is released. `@dimforge/rapier{2,3}d` is a `file:`
dependency on `typescript/builds/rapier{2,3}d/pkg`, which is generated; build it once (per
dimension) before installing:

```sh
cd typescript
npm ci
cargo run -p prepare_builds -- -d dim2 -f non-deterministic
cd builds/rapier2d && npm i && npm run build:wasm && npm run build:ts
```

Then, within the javascript folder, run `npm ci` and `npm run build`.
