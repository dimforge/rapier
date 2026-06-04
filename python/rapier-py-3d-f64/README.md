# rapier3d-f64

Python bindings for [Rapier](https://rapier.rs) — a fast, deterministic
physics engine written in Rust. This package provides the **3D, f64**
build.

```bash
pip install rapier3d-f64
```

```python
import rapier3d_f64 as rp

bodies = rp.RigidBodySet()
colliders = rp.ColliderSet()
body = bodies.insert(rp.RigidBody.dynamic().translation((0.0, 5.0, 0.0)))
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
