# Contributing to Rapier

Thank you for wanting to contribute! Contribution can take many forms, including:
- Reporting a bug.
- Submitting a fix.
- Fixing typos.
- Improving the docs.
- [Donations on GitHub Sponsors](https://github.com/sponsors/dimforge).

It is strongly recommended to [open an issue](https://github.com/dimforge/rapier/issues) or to discuss
with us [on Discord][discord] before fixing complicated issues, or implementing new
features.


## Contributing to the Rust code
The Rust source code of the Rapier physics engines is available on our `rapier` repository
[on GitHub](https://github.com/dimforge/rapier).

1. Fork our `rapier` repository [on GitHub](https://github.com/dimforge/rapier).
2. Clone the repository and make the necessary changes.
3. In order to debug your changes and check that it works, do the following:
   - Run the tests `cargo test`
   - Run the 2D examples and see if they behave as expected: `cargo run --release --bin all_examples2`
   - Run the 3D examples and see if they behave as expected: `cargo run --release --bin all_examples3`
   - Run the 2D examples with the `parallel` feature enabled: `cargo run --release --bin all_examples2 --features parallel`
   - Run the 3D examples with the `parallel` feature enabled: `cargo run --release --bin all_examples3 --features parallel`
4. Once you are satisfied with your changes, submit them by [opening a Pull Request](https://github.com/dimforge/rapier/pulls) on GitHub.
5. If that Pull Request does something you need urgently, or if you think it has been forgotten, don't hesitate
   to ask **@sebcrozet** directly [on Discord][discord] for a review.
6. Iterate with the reviewer until the PR gets merged.

## Contributing to the user-guide [rapier](https://rapier.rs)
The [official website](https://rapier.rs) for Rapier is built with [Docusaurus 3](https://docusaurus.io/).
Its source code can be found in the [`website/`](website/) directory of the `rapier` repository: the user guide is
written once in `website/docs/user_guides/templates`, and its code snippets are compiled from
`website/docs-examples` (see the [website README](website/README.md)).
If you find a typo or some outdated information on the website, please don't hesitate to reach
out! We can't stress enough how helpful it is for you to report (or fix) typo errors.

If you would like to fix it yourself, here is the procedure:

1. Fork our `rapier` repository [on GitHub](https://github.com/dimforge/rapier).
2. Clone the repository and make the necessary changes.
3. In order to debug your changes and check that it works, do the following:
    - `cd website; yarn install; yarn start;` This will open the website on your browser locally.
      The default local address is http://localhost:3000 .
    - If you changed a code snippet, check that it still compiles, e.g., with `cd website/docs-examples; cargo test`
      for the Rust snippets (see the [docs-examples README](website/docs-examples/README.md) for the other languages).
4. Once you are satisfied with your changes, submit them by [opening a Pull Request](https://github.com/dimforge/rapier/pulls) on GitHub.
5. If that Pull Request does something you need urgently, or if you think it has been forgotten, don't hesitate
   to ask **@sebcrozet** directly [on Discord][discord] for a review.
6. Iterate with the reviewer until the PR gets merged.

## Contributing to the bindings and the Bevy plugin
The official bindings and the Bevy plugin live in the [`bindings/`](bindings/) directory of the `rapier` repository:
- [`bindings/typescript`](bindings/typescript): the JavaScript/TypeScript bindings. You will need
  [wasm-pack](https://github.com/rustwasm/wasm-pack) to build them; see its [README](bindings/typescript/README.md).
- [`bindings/c`](bindings/c): the C and C++ bindings; see its [README](bindings/c/README.md).
- [`bindings/python`](bindings/python): the Python bindings; see its [README](bindings/python/README.md).
- [`bindings/bevy_rapier`](bindings/bevy_rapier): the `bevy_rapier2d` and `bevy_rapier3d` plugins for the
  [Bevy](https://bevyengine.org) game engine. They build against the rapier crates of this repository.

The procedure is the same as for the Rust code:

1. Fork our `rapier` repository [on GitHub](https://github.com/dimforge/rapier).
2. Clone the repository and make the necessary changes.
3. In order to debug your changes and check that it works:
   - JavaScript: from `bindings/typescript`, build the packages (see its README), then run the 3D examples with
     `cd testbed3d; npm install; npm run start` (the procedure for the 2D version is similar).
   - Bevy: from `bindings/bevy_rapier`, run the tests with `cargo test` and the examples with, e.g.,
     `cargo run --release -p bevy_rapier3d --example boxes3`.
4. Once you are satisfied with your changes, submit them by [opening a Pull Request](https://github.com/dimforge/rapier/pulls) on GitHub.
5. If that Pull Request does something you need urgently, or if you think it has been forgotten, don't hesitate
   to ask **@sebcrozet** directly [on Discord][discord] for a review.
6. Iterate with the reviewer until the PR gets merged.

[discord]: https://discord.gg/vt9DJSW
