import hashlib
import math

import rapier3d as rp

# DOCUSAURUS: DeterminismWrong start
# WRONG version:
# The following will not work cross-platform-deterministically because the functions of the
# `math` module (and of NumPy) give different results on different platforms.
collider = rp.Collider.ball(0.5).translation((math.exp(1.0), math.sin(2.0), math.cos(3.0))).build()
# DOCUSAURUS: DeterminismWrong stop

# DOCUSAURUS: Determinism start
import rapier3d.math as rpm

# CORRECT version:
# The following will work cross-platform-deterministically because we use the functions
# of Rapier.
collider = rp.Collider.ball(0.5).translation((rpm.exp(1.0), rpm.sin(2.0), rpm.cos(3.0))).build()
# DOCUSAURUS: Determinism stop

# DOCUSAURUS: CheckDeterminism start
# Make sure the loaded bindings are built with the `determinism` feature.
if not rp.build_features().enhanced_determinism:
    print("Warning: the rapier3d bindings aren't cross-platform deterministic.")
# DOCUSAURUS: CheckDeterminism stop


def build_world():
    world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
    world.add_collider(rp.Collider.cuboid(100.0, 0.1, 100.0))
    for i in range(10):
        world.add_body(
            rp.RigidBody.dynamic(translation=(0.1 * i, 1.0 + 1.1 * i, 0.0)),
            colliders=[rp.Collider.cuboid(0.5, 0.5, 0.5)],
        )
    return world


world = build_world()
for _ in range(100):
    world.step()

# DOCUSAURUS: SnapshotHash start
# Two simulations are in the exact same state if their snapshots are identical.
digest = hashlib.sha256(world.snapshot()).hexdigest()
print("State after 100 timesteps:", digest)
# DOCUSAURUS: SnapshotHash stop

# The same simulation run again, with another number of threads, gives the same state.
other = build_world()
other.set_num_threads(1)
for _ in range(100):
    other.step()
assert hashlib.sha256(other.snapshot()).hexdigest() == digest
