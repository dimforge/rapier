<p align="center">
  <img src="https://www.rapier.rs/img/rapier_logo_color_textpath_dark.svg" alt="crates.io">
</p>
<p align="center">
    <a href="https://discord.gg/vt9DJSW">
        <img src="https://img.shields.io/discord/507548572338880513.svg?logo=discord&colorB=7289DA">
    </a>
    <a href="https://github.com/dimforge/rapier.js/actions">
        <img src="https://github.com/dimforge/rapier.js/workflows/main/badge.svg" alt="Build status">
    </a>
    <a href="https://opensource.org/licenses/Apache-2.0">
        <img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg">
    </a>
</p>
<p align = "center">
    <strong>
        <a href="https://rapier.rs">Website</a> | <a href="https://rapier.rs/docs/">Documentation</a> |
        <a href="https://github.com/dimforge/rapier.js/tree/master/testbed2d/src/demos">2D examples (sources)</a> | 
        <a href="https://github.com/dimforge/rapier.js/tree/master/testbed3d/src/demos">3D examples (sources)</a>
    </strong>
</p>

---

<p align = "center">
<b>2D and 3D physics engines</b>
<i>for the JavaScript programming language (official bindings).</i>
</p>

---

## Building packages manually

From the root of the repository, run:

```shell
./builds/prepare_builds/prepare_all_projects.sh
./builds/prepare_builds/build_all_projects.sh
```

Note that `prepare_all_projects.sh` only needs to be run once. It needs to be re-run if any file from the
`builds/prepare_builds` directory (and subdirectories) are modified.

The built packages will be in `builds/rapier2d/pkg`, `builds/rapier3d/pkg`, etc. To build the `-compat` variant of the
packages, run `npm run build` in the `rapier-compat` directory. Note that this will only work if you already ran
`prepare_all_projects.sh`. The compat packages are then generated in, e.g., `rapier-compat/builds/3d/pkg`.

## Feature selection

Multiple NPM packages exist for Rapier, depending on your needs:
- [`@dimforge/rapier2d`](https://www.npmjs.com/package/@dimforge/rapier2d) or
  [`@dimforge/rapier3d`](https://www.npmjs.com/package/@dimforge/rapier3d):
  The main build of the Rapier physics engine for 2D or 3D physics simulation. This should have wide browser
  support while offering great performances. This does **not** guarantee cross-platform determinism of the physics
  simulation (but it is still locally deterministic, on the same machine).
- [`@dimforge/rapier2d-simd`](https://www.npmjs.com/package/@dimforge/rapier2d-simd) or
  [`@dimforge/rapier3d-simd`](https://www.npmjs.com/package/@dimforge/rapier3d-simd):
  A build with internal SIMD optimizations enabled. More limited browser support (requires support for [simd128](https://caniuse.com/?search=simd)).
- [`@dimforge/rapier2d-deterministic`](https://www.npmjs.com/package/@dimforge/rapier2d-deterministic) or
  [`@dimforge/rapier3d-deterministic`](https://www.npmjs.com/package/@dimforge/rapier3d-deterministic):
  A less optimized build but with a guarantee of a cross-platform deterministic execution of the physics simulation.

## Bundler support

Some bundlers will struggle with the `.wasm` file package into the builds above. Alternative `-compat` versions exist
which embed the `.wasm` file into the `.js` sources encoded with base64. This results in a bigger package size, but
much wider bundler support.

Just append `-compat` to the build you are interested in:
[`rapier2d-compat`](https://www.npmjs.com/package/@dimforge/rapier2d-compat),
[`rapier2d-simd-compat`](https://www.npmjs.com/package/@dimforge/rapier2d-simd-compat),
[`rapier2d-deterministic-compat`](https://www.npmjs.com/package/@dimforge/rapier2d-deterministic-compat),
[`rapier3d-compat`](https://www.npmjs.com/package/@dimforge/rapier3d-compat),
[`rapier3d-simd-compat`](https://www.npmjs.com/package/@dimforge/rapier3d-simd-compat),
[`rapier3d-deterministic-compat`](https://www.npmjs.com/package/@dimforge/rapier3d-deterministic-compat).

## Nightly builds

Each time a new Pull Request is merged to the `main` branch of the [`rapier.js` repository](https://github.com/dimforge/rapier.js),
an automatic _canary_ build is triggered. Builds published to npmjs under the _canary_ tag does not come with any
stability guarantee and does not follow semver versioning. But it can be a useful solution to try out the latest
features until a proper release is cut.