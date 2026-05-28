## MJCF parser for Rust

> **Disclaimer.** Most of this crate — source, tests, and documentation — was
> produced by an AI coding assistant working iteratively from MJCF reference
> scenes (primarily the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)),
> under human direction and review.

`mjcf-rs` is a pure-Rust parser for the [MuJoCo XML format (MJCF)](https://mujoco.readthedocs.io/en/stable/XMLreference.html).

It produces a typed AST that mirrors MJCF 1:1 in element names, with all
preprocessing (`<include>` resolution, `<default>` class inheritance, angle
unit normalization) already done — so consumers don't need to revisit the
spec for any of those.

This crate has **no dependency** on `rapier3d` or any physics engine. Pair
it with [`rapier3d-mjcf`](../rapier3d-mjcf) to actually simulate MJCF
models.

### Status

See [`rapier3d-mjcf`'s feature matrix](../rapier3d-mjcf/README.md) for an
up-to-date, phase-by-phase list of what's supported.
