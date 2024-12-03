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
The Rust source code of the Rapier physics engines is available  on our `rapier` repository
[on GitHub](https://github.com/dimforge/rapier.rs).

1. Fork our `rapier` repository [on GitHub](https://github.com/dimforge/rapier).
2. Clone the repository and make the necessary changes.
3. In order to debug your changes and check that it works, do the following:
   - Run the tests `cargo test`
   - Run the 2D examples and see if they behave as expected: `cargo run --release --bin all_examples2`
   - Run the 3D examples and see if they behave as expected: `cargo run --release --bin all_examples3`
   - Run the 2D examples with the `parallel` and `simd-stable` features enabled: `cargo run --release --bin all_examples2 --features parallel,simd-stable`
   - Run the 3D examples with the `parallel` and `simd-stable` features enabled: `cargo run --release --bin all_examples3 --features parallel,simd-stable`
4. Once you are satisfied with your changes, submit them by [opening a Pull Request](https://github.com/dimforge/rapier/pulls) on GitHub.
5. If that Pull Request does something you need urgently, or if you think it has been forgotten, don't hesitate
   to ask **@sebcrozet** directly [on Discord][discord] for a review.
6. Iterate with the reviewer until the PR gets merged.

## Contributing to the user-guide [rapier](https://rapier.rs)
The [official website](https://rapier.rs) for Rapier is built with [Docusaurus 2](https://docusaurus.io/).
Its source code can be found on our `rapier.rs` repository [on GitHub](https://github.com/dimforge/rapier.rs).
If you  find a typo or some outdated information on the website, please don't hesitate to reach
out! We can't stress enough how helpful it is for you to report (or fix) typo errors.

If you would like to fix it yourself, here is the procedure:

1. Fork our `rapier.rs` repository [on GitHub](https://github.com/dimforge/rapier.rs).
2. Clone the repository and make the necessary changes.
3. In order to debug your changes and check that it works, do the following:
    - `cd website; yarn install; yarn start;` This will open the website on your browser locally.
      The default local address is http://localhost:3000 .
4. Once you are satisfied with your changes, submit them by [opening a Pull Request](https://github.com/dimforge/rapier.rs/pulls) on GitHub.
5. If that Pull Request does something you need urgently, or if you think it has been forgotten, don't hesitate
   to ask **@sebcrozet** directly [on Discord][discord] for a review.
6. Iterate with the reviewer until the PR gets merged.

## Contributing to the JavaScript/TypeScript bindings
The source code of the official JavaScript/TypeScript bindings for Rapier are available
on our `rapier.js` repository [on GitHub](https://github.com/dimforge/rapier.js).

You will have to make sure that you have [wasm-pack](https://github.com/rustwasm/wasm-pack) installed because
it is responsible for generating the low-level bindings. In order to modify the bindings and test your
changes you may:

1. Fork our `rapier.js` repository [on GitHub](https://github.com/dimforge/rapier.js).
2. Clone the repository and make the necessary changes.
3. In order to debug your changes and check that it works, do the following for the 3D version of rapier (the procedure
   for the 2D version is similar):
   - `cd rapier3d; npm install; ./build_all.sh`. This will build the JS bindings, TS type definitions, and generate
      the documentation. This will take several minutes to complete.
   - After making changes to the Rust code on that repository, you need to run `./build_rust.sh`.
   - After making changes to the TypeScript code on that repository, you need to run `./build_typescript.sh`.
   - To test your changes, go back to the repository's root directory, and do: `cd testbed3d; npm install; npm link ../rapier3d/pkg; npm run start`.
     This will open the 3D examples on your browser.
   - Our build system for these JS bindings is still a bit messy right now. If you have any trouble, don't
     hesitate to contact us [on Discord][discord].
4. Once you are satisfied with your changes, submit them by [opening a Pull Request](https://github.com/dimforge/rapier.js/pulls) on GitHub.
5. If that Pull Request does something you need urgently, or if you think it has been forgotten, don't hesitate
   to ask **@sebcrozet** directly [on Discord][discord] for a review.
6. Iterate with the reviewer until the PR gets merged.

[discord]: https://discord.gg/vt9DJSW
