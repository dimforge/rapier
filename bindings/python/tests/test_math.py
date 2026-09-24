"""3D math layer tests.

These run against the default `rapier.dim3` (f32) namespace.
"""

from __future__ import annotations

import math as pymath

import pytest

import rapier3d as rapier
import rapier3d as dim3


@pytest.fixture(params=[dim3], ids=["f32"])
def ns(request):
    return request.param


# ---- Vec3 -----------------------------------------------------------------


def test_vec3_repr_and_attrs(ns):
    v = ns.Vec3(1, 2, 3)
    assert v.x == 1.0
    assert v.y == 2.0
    assert v.z == 3.0
    assert repr(v).startswith("Vec3(")


def test_vec3_construction_equivalence(ns):
    np = pytest.importorskip("numpy")
    a = ns.Vec3(1, 2, 3)
    b = ns.Vec3.from_tuple((1.0, 2.0, 3.0))
    c = ns.Vec3.from_tuple([1.0, 2.0, 3.0])
    dtype = np.float32 if ns is dim3 else np.float64
    d = ns.Vec3.from_ndarray(np.array([1.0, 2.0, 3.0], dtype=dtype))
    assert a == b == c == d


def test_vec3_add(ns):
    assert ns.Vec3(1, 2, 3) + ns.Vec3(4, 5, 6) == ns.Vec3(5, 7, 9)
    # implicit conversion: tuple as RHS
    assert ns.Vec3(1, 2, 3) + (4, 5, 6) == ns.Vec3(5, 7, 9)


def test_vec3_sub_neg(ns):
    assert ns.Vec3(5, 7, 9) - ns.Vec3(1, 2, 3) == ns.Vec3(4, 5, 6)
    assert -ns.Vec3(1, 2, 3) == ns.Vec3(-1, -2, -3)


def test_vec3_mul_div(ns):
    assert ns.Vec3(1, 2, 3) * 2 == ns.Vec3(2, 4, 6)
    assert 2 * ns.Vec3(1, 2, 3) == ns.Vec3(2, 4, 6)
    assert ns.Vec3(2, 4, 6) / 2 == ns.Vec3(1, 2, 3)
    with pytest.raises(ValueError):
        _ = ns.Vec3(1, 2, 3) / 0


def test_vec3_dot_cross_norm(ns):
    assert ns.Vec3(1, 2, 3).dot((4, 5, 6)) == pytest.approx(32.0)
    assert ns.Vec3(1, 0, 0).cross((0, 1, 0)) == ns.Vec3(0, 0, 1)
    assert ns.Vec3(3, 4, 0).norm() == pytest.approx(5.0)
    assert ns.Vec3(3, 4, 0).norm_squared() == pytest.approx(25.0)
    assert ns.Vec3(3, 4, 0).normalize() == ns.Vec3(0.6, 0.8, 0.0)


def test_vec3_iter_len(ns):
    v = ns.Vec3(1, 2, 3)
    assert len(v) == 3
    assert list(v) == [1.0, 2.0, 3.0]
    assert v[0] == 1.0
    assert v[-1] == 3.0
    with pytest.raises(IndexError):
        _ = v[3]


def test_vec3_to_ndarray(ns):
    np = pytest.importorskip("numpy")
    v = ns.Vec3(1, 2, 3)
    a = v.to_ndarray()
    expected_dtype = np.float32 if ns is dim3 else np.float64
    assert a.dtype == expected_dtype
    assert list(a) == [1.0, 2.0, 3.0]


def test_vec3_bitwise_equal(ns):
    a = ns.Vec3(1, 2, 3)
    b = ns.Vec3(1, 2, 3)
    assert a.bitwise_equal(b)
    # f32: pick a perturbation that approx_eq (tol 1e-7) tolerates but
    # bitwise check rejects.
    c = ns.Vec3(1.0 + 1e-8, 2, 3)
    # approx __eq__ tolerates the perturbation
    assert a == c
    # For f32, 1+1e-8 rounds to exactly 1.0 (below f32 epsilon ~1.19e-7), so
    # c == a bitwise, hence we don't assert the negative bitwise case here.


def test_vec3_lerp(ns):
    a = ns.Vec3(0, 0, 0)
    b = ns.Vec3(2, 4, 6)
    assert a.lerp(b, 0.5) == ns.Vec3(1, 2, 3)


# ---- Point3 ---------------------------------------------------------------


def test_point3_basic(ns):
    p = ns.Point3(1, 2, 3)
    assert (p.x, p.y, p.z) == (1.0, 2.0, 3.0)
    assert p == ns.Point3.from_tuple((1, 2, 3))


def test_point3_sub_returns_vec(ns):
    p1 = ns.Point3(5, 7, 9)
    p2 = ns.Point3(1, 2, 3)
    d = p1 - p2
    assert isinstance(d, ns.Vec3)
    assert d == ns.Vec3(4, 5, 6)


def test_point3_add_vec_returns_point(ns):
    p = ns.Point3(1, 2, 3)
    r = p + ns.Vec3(1, 1, 1)
    assert isinstance(r, ns.Point3)
    assert r == ns.Point3(2, 3, 4)


def test_point3_coords(ns):
    p = ns.Point3(1, 2, 3)
    assert isinstance(p.coords, ns.Vec3)
    assert p.coords == ns.Vec3(1, 2, 3)


# ---- Rotation3 / Quaternion ----------------------------------------------


def test_quaternion_is_rotation3(ns):
    assert ns.Quaternion is ns.Rotation3


def test_rotation3_identity(ns):
    assert ns.Rotation3.identity().transform_vector((1, 2, 3)) == ns.Vec3(1, 2, 3)


def test_rotation3_axis_angle(ns):
    r = ns.Rotation3.from_axis_angle((0, 0, 1), pymath.pi / 2)
    assert r.transform_vector((1, 0, 0)) == ns.Vec3(0, 1, 0)
    assert r.transform_point((1, 0, 0)) == ns.Point3(0, 1, 0)
    # Inverse cancels.
    assert (r.inverse() * r) == ns.Rotation3.identity()


def test_rotation3_zero_axis_errors(ns):
    with pytest.raises(ValueError):
        _ = ns.Rotation3.from_axis_angle((0, 0, 0), 1.0)


def test_rotation3_compose(ns):
    a = ns.Rotation3.from_axis_angle((0, 0, 1), pymath.pi / 2)
    b = ns.Rotation3.from_axis_angle((0, 0, 1), pymath.pi / 2)
    composed = a * b
    assert composed.transform_vector((1, 0, 0)) == ns.Vec3(-1, 0, 0)


def test_rotation3_slerp(ns):
    # Use pi/2 instead of pi: at exactly 180° the slerp axis is ambiguous
    # (q and -q represent the same rotation), so we'd just be checking the
    # tie-breaker convention. pi/2 produces a deterministic shortest path.
    a = ns.Rotation3.identity()
    b = ns.Rotation3.from_axis_angle((0, 0, 1), pymath.pi / 2)
    mid = a.slerp(b, 0.5)
    expected = ns.Rotation3.from_axis_angle((0, 0, 1), pymath.pi / 4)
    assert mid == expected


def test_rotation3_quaternion_repr_roundtrip(ns):
    r = ns.Rotation3.from_axis_angle((0, 0, 1), pymath.pi / 2)
    x, y, z, w = r.quaternion
    r2 = ns.Rotation3.from_quaternion(w, x, y, z)
    assert r == r2
    # Implicit conversion from a 4-tuple (x,y,z,w) — same convention as PyRotation.
    r3 = ns.Rotation3.from_tuple((x, y, z, w))
    assert r == r3


def test_rotation3_euler(ns):
    r = ns.Rotation3.from_euler_angles(0.1, 0.2, 0.3)
    rr, p, yy = r.euler_angles
    assert rr == pytest.approx(0.1, abs=1e-5)
    assert p == pytest.approx(0.2, abs=1e-5)
    assert yy == pytest.approx(0.3, abs=1e-5)


def test_rotation3_scaled_axis(ns):
    r = ns.Rotation3.from_scaled_axis((0, 0, pymath.pi / 2))
    assert r.transform_vector((1, 0, 0)) == ns.Vec3(0, 1, 0)


def test_rotation3_to_matrix(ns):
    pytest.importorskip("numpy")
    r = ns.Rotation3.identity()
    m = r.to_matrix()
    assert len(m) == 9


# ---- Isometry3 -----------------------------------------------------------


def test_isometry3_translation(ns):
    iso = ns.Isometry3(translation=(1, 2, 3), rotation=ns.Rotation3.identity())
    assert iso.transform_point((0, 0, 0)) == ns.Point3(1, 2, 3)


def test_isometry3_translation_factory(ns):
    iso = ns.Isometry3.from_translation(1, 2, 3)
    assert iso * ns.Point3(0, 0, 0) == ns.Point3(1, 2, 3)


def test_isometry3_compose_inverse(ns):
    a = ns.Isometry3.from_translation(1, 2, 3)
    b = ns.Isometry3(
        translation=(0, 0, 0),
        rotation=ns.Rotation3.from_axis_angle((0, 0, 1), pymath.pi / 2),
    )
    composed = a * b
    p = composed * ns.Point3(1, 0, 0)
    assert p == ns.Point3(1, 3, 3)  # rotate (1,0,0)->(0,1,0), then translate by (1,2,3)
    # Inverse cancels
    inv = composed.inverse()
    assert (inv * composed) == ns.Isometry3.identity()


def test_isometry3_transform_vector_ignores_translation(ns):
    iso = ns.Isometry3.from_translation(1, 2, 3)
    assert iso.transform_vector((4, 5, 6)) == ns.Vec3(4, 5, 6)


def test_isometry3_to_matrix(ns):
    pytest.importorskip("numpy")
    iso = ns.Isometry3.identity()
    m = iso.to_matrix()
    assert len(m) == 16


# ---- Free helpers --------------------------------------------------------


def test_rotation_from_angle_3d(ns):
    r = ns.rotation_from_angle((0, 0, pymath.pi / 2))
    assert r.transform_vector((1, 0, 0)) == ns.Vec3(0, 1, 0)


def test_math_helpers():
    # `rapier.math` is dim3-flavored by default.
    r = rapier.math.rotation_from_angle((0, 0, pymath.pi / 2))
    assert isinstance(r, dim3.Rotation3)
    v = rapier.math.lerp(dim3.Vec3(0, 0, 0), dim3.Vec3(2, 4, 6), 0.5)
    assert v == dim3.Vec3(1, 2, 3)
    assert rapier.math.wrap_to_pi(3 * pymath.pi) == pytest.approx(pymath.pi)
    assert rapier.math.wrap_to_pi(-3 * pymath.pi) == pytest.approx(pymath.pi)


def test_top_level_reexports():
    # Default umbrella re-exports dim3 math types.
    assert rapier.Vec3 is dim3.Vec3
    assert rapier.Quaternion is dim3.Rotation3
    assert rapier.AngVector3 is dim3.Vec3
