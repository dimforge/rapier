## Repository architecture

The architecture of this repository is a bit unusual because we are using some tricks to have both
the 2D and 3D version of Rapier share the same code-base. Here are the main folders:
- **`build/`**: contains one folder per Rapier crate (for the 2D, 3D, `f32`, and `f64` versions). Each
  crate has its own `Cargo.toml` file that adjusts some cargo features, and reference the `src` folder.
- **`src/`**: contains the actual `.rs` source code of the Rapier physics engine.
- **`src_testbed/`**: contains the `.rs` source code of the Rapier testbed (which our examples are based on).
- **`examples2d/`**: simple 2D scenes showcasing some of Rapier's capabilities.
  Run them with `cargo run --release --bin all_examples2`.
- **`examples3d/`**: simple 3D scenes showcasing some of Rapier's capabilities.
  Run them with `cargo run --release --bin all_examples3`.
- **`benchmarks2d/`**: a set of 2D stress-tests, to see how Rapier performs when it has lots of elements
  to simulate.
- **`benchmarks3d/`**: a set of 3D stress-tests, to see how Rapier performs when it has lots of elements
  to simulate. We use the these benchmarks to track the performances of Rapier after some changes,
  and spot unexpected regressions: https://www.rapier.rs/benchmarks/