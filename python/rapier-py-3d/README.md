# rapier3d

Python bindings for [Rapier](https://rapier.rs) — a fast, deterministic
physics engine written in Rust. This package provides the **3D, f32**
build.

```bash
pip install rapier3d
```

```python
import rapier3d as rp

bodies = rp.RigidBodySet()
colliders = rp.ColliderSet()
body = bodies.insert(rp.RigidBody.dynamic().translation((0.0, 5.0, 0.0)))
```

## License

Apache-2.0
