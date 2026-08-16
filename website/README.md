# Website

This website is built using [Docusaurus 3](https://docusaurus.io/), a modern static website generator.

### Installation

```sh
$ yarn
```

### Build

```sh
$ ./generate_user_guides.sh
```

Above command reads the templates directory and extracts specific instructions for bevy / rust / rapier integrations.
It also injects code contained in the example files.

```sh
$ yarn build
```

Above command generates static content into the `build` directory and can be served using any static contents hosting service.

```sh
$ yarn start
```

Above command builds and starts a local development server and open up a browser window. Most changes are reflected live without having to restart the server.

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

### Deployment

```sh
$ GIT_USER=<Your GitHub username> USE_SSH=true yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
