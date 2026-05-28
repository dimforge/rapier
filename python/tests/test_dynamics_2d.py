"""2D dynamics tests.

Parametrized across f32 and f64 flavors.
"""

from __future__ import annotations

import math as pymath

import pytest

import rapier2d as dim2
import rapier2d_f64 as dim2_f64


@pytest.fixture(params=[dim2, dim2_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def test_handle_hash_and_eq(ns):
    h1 = ns.RigidBodyHandle.from_raw_parts(3, 7)
    h2 = ns.RigidBodyHandle.from_raw_parts(3, 7)
    h3 = ns.RigidBodyHandle.from_raw_parts(3, 8)
    assert h1 == h2
    assert h1 != h3
    assert hash(h1) == hash(h2)


def test_build_insert_lookup(ns):
    s = ns.RigidBodySet()
    b = ns.RigidBody.dynamic(translation=(1.0, 2.0)).build()
    h = s.insert(b)
    assert h in s
    body = s[h]
    t = body.translation
    assert abs(t.x - 1.0) < 1e-5
    assert abs(t.y - 2.0) < 1e-5


def test_remove_invalidates_handle(ns):
    s = ns.RigidBodySet()
    islands = ns.IslandManager()
    cset = ns.ColliderSet()
    ijs = ns.ImpulseJointSet()
    mjs = ns.MultibodyJointSet()
    h = s.insert(ns.RigidBody.dynamic(translation=(0, 0)).build())
    removed = s.remove(h, islands, cset, ijs, mjs)
    assert removed is not None
    assert h not in s
    with pytest.raises(ns.InvalidHandle):
        _ = s[h]


def test_body_type_setter(ns):
    s = ns.RigidBodySet()
    h = s.insert(ns.RigidBody.dynamic().build())
    body = s[h]
    body.body_type = ns.RigidBodyType.FIXED
    assert body.body_type == ns.RigidBodyType.FIXED


def test_locked_axes_2d(ns):
    la = ns.LockedAxes.TRANSLATION_LOCKED_X | ns.LockedAxes.ROTATION_LOCKED
    assert ns.LockedAxes.TRANSLATION_LOCKED_X in la
    assert ns.LockedAxes.ROTATION_LOCKED in la
    # Round-trip
    b = ns.RigidBody.dynamic().build()
    b.locked_axes = la
    assert b.locked_axes.bits == la.bits


def test_apply_impulse_2d(ns):
    b = ns.RigidBody.dynamic().build()
    b.set_additional_mass(1.0)
    cset = ns.ColliderSet()
    b.recompute_mass_properties_from_colliders(cset)
    b.apply_impulse((10.0, 0.0))
    assert abs(b.linvel.x - 10.0) < 1e-4


def test_kinetic_energy_2d(ns):
    b = ns.RigidBody.dynamic().build()
    b.set_additional_mass(2.0)
    cset = ns.ColliderSet()
    b.recompute_mass_properties_from_colliders(cset)
    b.linvel = (3.0, 0.0)
    ke = b.kinetic_energy()
    assert abs(ke - 9.0) < 1e-4


def test_angvel_scalar_2d(ns):
    b = ns.RigidBody.dynamic().build()
    b.angvel = 1.5
    assert abs(b.angvel - 1.5) < 1e-5


def test_integration_parameters_2d(ns):
    p = ns.IntegrationParameters()
    p.dt = 1.0 / 30.0
    assert abs(p.dt - 1.0 / 30.0) < 1e-6


def test_mass_properties_ball_2d(ns):
    mp = ns.MassProperties.from_ball(1.0, 1.0)
    expected = pymath.pi  # 2D ball mass = density * pi * r^2 = 1 * pi * 1 = pi
    assert abs(mp.mass - expected) < 1e-4


def test_activation_struct_2d(ns):
    a = ns.RigidBodyActivation.active()
    assert a.is_active()


def test_island_manager_2d(ns):
    im = ns.IslandManager()
    assert len(im) == 0


def test_spring_coefficients_2d(ns):
    sc = ns.SpringCoefficients.contact_defaults()
    assert sc.stiffness > 0.0


def test_ccd_solver_2d(ns):
    cs = ns.CCDSolver()
    cs.clear()
    with pytest.raises(NotImplementedError):
        cs.solve_ccd()
