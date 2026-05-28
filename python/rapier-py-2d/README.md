# rapier2d

Python bindings for [Rapier](https://rapier.rs) — a fast, deterministic
physics engine written in Rust. This package provides the **2D, f32**
build.

```bash
pip install rapier2d
```

```python
import rapier2d as rp

bodies = rp.RigidBodySet()
colliders = rp.ColliderSet()
body = bodies.insert(rp.RigidBody.dynamic().translation((0.0, 5.0)))
```

## Related packages

| Package        | Dimension | Scalar |
| -------------- | --------- | ------ |
| `rapier2d`     | 2D        | f32    |
| `rapier3d`     | 3D        | f32    |
| `rapier2d-f64` | 2D        | f64    |
| `rapier3d-f64` | 3D        | f64    |

All four share an identical API differing only in dimension and scalar type.

## License

Apache-2.0
