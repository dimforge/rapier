"""3D dynamics tests (f32)."""

from __future__ import annotations

import math as pymath

import pytest

import rapier3d as rapier
import rapier3d as dim3


@pytest.fixture(params=[dim3], ids=["f32"])
def ns(request):
    return request.param


# ---- RigidBodyHandle ------------------------------------------------------


def test_handle_hash_and_eq(ns):
    h1 = ns.RigidBodyHandle.from_raw_parts(3, 7)
    h2 = ns.RigidBodyHandle.from_raw_parts(3, 7)
    h3 = ns.RigidBodyHandle.from_raw_parts(3, 8)
    assert h1 == h2
    assert h1 != h3
    assert hash(h1) == hash(h2)
    assert "index=3" in repr(h1)
    assert h1.index == 3 and h1.generation == 7


# ---- RigidBody / RigidBodySet --------------------------------------------


def test_build_insert_lookup(ns):
    s = ns.RigidBodySet()
    b = ns.RigidBody.dynamic(translation=(1.0, 2.0, 3.0)).build()
    h = s.insert(b)
    assert h in s
    body = s[h]
    t = body.translation
    assert abs(t.x - 1.0) < 1e-5
    assert abs(t.y - 2.0) < 1e-5
    assert abs(t.z - 3.0) < 1e-5


def test_set_iter_and_handles(ns):
    s = ns.RigidBodySet()
    h1 = s.insert(ns.RigidBody.dynamic(translation=(1, 0, 0)).build())
    h2 = s.insert(ns.RigidBody.dynamic(translation=(2, 0, 0)).build())
    assert len(s) == 2
    handles = list(s.handles())
    assert h1 in handles and h2 in handles
    pairs = list(s)
    assert len(pairs) == 2
    for handle, body in pairs:
        assert handle in (h1, h2)
        assert isinstance(body.translation, ns.Vec3)


def test_remove_invalidates_handle(ns):
    s = ns.RigidBodySet()
    islands = ns.IslandManager()
    cset = ns.ColliderSet()
    ijs = ns.ImpulseJointSet()
    mjs = ns.MultibodyJointSet()
    h = s.insert(ns.RigidBody.dynamic(translation=(0, 0, 0)).build())
    removed = s.remove(h, islands, cset, ijs, mjs)
    assert removed is not None
    assert h not in s
    with pytest.raises(ns.InvalidHandle):
        _ = s[h]


# ---- Body type ----


def test_body_type_setter(ns):
    s = ns.RigidBodySet()
    h = s.insert(ns.RigidBody.dynamic().build())
    body = s[h]
    assert body.body_type == ns.RigidBodyType.DYNAMIC
    body.body_type = ns.RigidBodyType.FIXED
    assert body.body_type == ns.RigidBodyType.FIXED


# ---- LockedAxes -----------------------------------------------------------


def test_locked_axes_bitflags(ns):
    la = ns.LockedAxes.TRANSLATION_LOCKED_X | ns.LockedAxes.TRANSLATION_LOCKED_Z
    assert ns.LockedAxes.TRANSLATION_LOCKED_X in la
    assert ns.LockedAxes.TRANSLATION_LOCKED_Z in la
    assert ns.LockedAxes.TRANSLATION_LOCKED_Y not in la
    assert (la & ns.LockedAxes.TRANSLATION_LOCKED_X) == ns.LockedAxes.TRANSLATION_LOCKED_X
    # Round-trip through a body
    b = ns.RigidBody.dynamic().build()
    b.locked_axes = la
    got = b.locked_axes
    assert got.bits == la.bits


# ---- Forces / impulses ----------------------------------------------------


def test_apply_impulse_changes_linvel(ns):
    # `set_additional_mass(1.0)` only materializes the additional-mass
    # property; the local mass-properties get composed at the next physics
    # step (or via `recompute_mass_properties_from_colliders`). We
    # materialize manually with an empty collider set.
    b = ns.RigidBody.dynamic().build()
    b.set_additional_mass(1.0)
    cset = ns.ColliderSet()
    b.recompute_mass_properties_from_colliders(cset)
    b.apply_impulse((10.0, 0.0, 0.0))
    # Should now have ~10 m/s linvel along X (mass=1).
    assert abs(b.linvel.x - 10.0) < 1e-4


def test_add_force_accumulates(ns):
    b = ns.RigidBody.dynamic().build()
    b.set_additional_mass(1.0)
    cset = ns.ColliderSet()
    b.recompute_mass_properties_from_colliders(cset)
    b.add_force((3.0, 0.0, 0.0))
    uf = b.user_force
    assert abs(uf.x - 3.0) < 1e-5


# ---- Energy / kinematics --------------------------------------------------


def test_kinetic_energy(ns):
    b = ns.RigidBody.dynamic().build()
    b.set_additional_mass(2.0)
    cset = ns.ColliderSet()
    b.recompute_mass_properties_from_colliders(cset)
    b.linvel = (3.0, 0.0, 0.0)
    ke = b.kinetic_energy()
    # 0.5 * m * v^2 = 0.5 * 2 * 9 = 9
    assert abs(ke - 9.0) < 1e-4


def test_integration_parameters(ns):
    p = ns.IntegrationParameters()
    assert abs(p.dt - 1.0 / 60.0) < 1e-6
    p.dt = 1.0 / 30.0
    assert abs(p.dt - 1.0 / 30.0) < 1e-6


def test_integration_parameters_default_equal(ns):
    a = ns.IntegrationParameters()
    b = ns.IntegrationParameters.default_params()
    assert a == b


# ---- MassProperties -------------------------------------------------------


def test_mass_properties_ball(ns):
    mp = ns.MassProperties.from_ball(1.0, 1.0)
    expected = 4.0 * pymath.pi / 3.0
    assert abs(mp.mass - expected) < 1e-4


def test_mass_properties_cuboid(ns):
    mp = ns.MassProperties.from_cuboid(2.0, (1.0, 0.5, 0.25))
    # Volume = 2 * 1 * 0.5 * 0.25 * 2 = ... actually it's
    # (2*half_x)*(2*half_y)*(2*half_z) * density = 2 * 1 * 0.5 * density = 1.0
    expected = 2.0 * 1.0 * 0.5 * 2.0
    assert abs(mp.mass - expected) < 1e-4


# ---- RigidBodyType enum ----------------------------------------------------


def test_rigid_body_type_enum(ns):
    assert ns.RigidBodyType.DYNAMIC != ns.RigidBodyType.FIXED
    assert ns.RigidBodyType.DYNAMIC == ns.RigidBodyType.DYNAMIC


# ---- Activation -----------------------------------------------------------


def test_activation_struct(ns):
    a = ns.RigidBodyActivation.active()
    assert a.is_active()
    a2 = ns.RigidBodyActivation.inactive()
    assert not a2.is_active()


# ---- SpringCoefficients ---------------------------------------------------


def test_spring_coeffs_defaults(ns):
    sc = ns.SpringCoefficients.contact_defaults()
    assert sc.stiffness > 0.0
    assert sc.damping > 0.0


# ---- CCDSolver ------------------------------------------------------------


def test_ccd_solver_constructor(ns):
    cs = ns.CCDSolver()
    cs.clear()
    with pytest.raises(NotImplementedError):
        cs.solve_ccd()


# ---- IslandManager --------------------------------------------------------


def test_island_manager(ns):
    im = ns.IslandManager()
    assert len(im) == 0
