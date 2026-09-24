# Website

This website is built using [Docusaurus 3](https://docusaurus.io/), a modern static website generator.

### Installation

```sh
$ yarn
```

### Build

```sh
$ yarn build
```

Above command generates static content into the `build` directory and can be served using any static contents hosting service.

```sh
$ yarn start
```

Above command builds and starts a local development server and open up a browser window. Most changes are reflected live without having to restart the server.

### User guide

The user guide is written once, in `docs/user_guides/templates`, with flavour tags selecting the content of each
variant: `<rapier>` (Rust), `<bevy>`, `<js>`, `<c>`, `<py>`, and the `<notjs>`, `<notc>`, `<notpy>` exclusions. The
code snippets are loaded with `<load path='…' marker='…' />` tags from the compiled examples of `docs-examples`.

The `plugins/user-guide-inject` plugin generates one folder per variant (`docs/user_guides/rust`, `bevy_plugin`,
`javascript`, `c`, `python`, gitignored) when the site starts or builds, and runs the `docs-examples/inject_file` tool on
every page at compile time: editing a template, or any example file it loads, hot-reloads every variant. The templates
themselves are published as the "All" variant, where the content of every flavour is shown with its color code. The
pages of each variant and their sidebar order are listed in `src/userGuideVariants.js`, and a selector on top of each
page of the guide switches between the variants of that page. Adding or removing a template, or changing its front
matter, requires a restart of `yarn start`. Unresolved `<load>` tags are reported as build warnings.

`./generate_user_guides.sh` generates the variants without starting the site, e.g. to inspect them.

### Demos

The `/demos` page embeds the `all_examples2` / `all_examples3` testbeds compiled to
WebAssembly. They are build artifacts: `static/demos` is gitignored, so it must be
generated once before `yarn build` (or `yarn start`) can serve them.

```sh
$ yarn build:demos              # both demos
$ yarn build:demos all_examples2  # just one
$ yarn build:all                # demos, then the website
```

Requires the `wasm32-unknown-unknown` target (`rustup target add wasm32-unknown-unknown`);
a matching `wasm-bindgen-cli` is installed under `target/` automatically if the global one
has the wrong version. `SKIP_WASM_OPT=1` skips the (slow) `wasm-opt` pass for fast
iteration, at the cost of a bigger `.wasm`. `publish.sh` runs the demo build itself.

Examples that read assets from disk (URDF/MJCF robots, `.obj` meshes, scene dumps) are
`#[cfg]`-ed out of wasm builds; see `examples2d/all_examples2.rs` and
`examples3d/all_examples3.rs`.

### API references

The JavaScript, C, and Python API pages embed the references generated from the bindings of this repository
(typedoc for `bindings/typescript/`, Doxygen for `bindings/c/`, Sphinx for `bindings/python/`). They are build artifacts: `static/javascript2d`,
`static/javascript3d`, `static/c`, and `static/python` are gitignored, so they must be generated once before `yarn build`
(or `yarn start`) can serve them.

```sh
$ yarn build:api-docs                    # both references
$ ./scripts/build-api-docs.sh js         # just the JavaScript one
$ ./scripts/build-api-docs.sh c          # just the C one
$ ./scripts/build-api-docs.sh py         # just the Python one
```

The JavaScript reference requires `wasm-pack` and `npm` (it builds the 2D and 3D packages first); the C
reference requires `cmake`, a C compiler, `python3`, and Doxygen 1.9.4 or later; the Python reference builds the
package with `bindings/python/dev.sh docs` (Rust toolchain and `python3`). `publish.sh` and
`yarn build:all` run this build themselves.

### Deployment

```sh
$ GIT_USER=<Your GitHub username> USE_SSH=true yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
