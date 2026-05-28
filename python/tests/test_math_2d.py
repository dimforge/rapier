"""2D math layer tests."""

from __future__ import annotations

import math as pymath

import pytest

import rapier2d as dim2
import rapier2d_f64 as dim2_f64


@pytest.fixture(params=[dim2, dim2_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def test_vec2_basic(ns):
    v = ns.Vec2(3, 4)
    assert (v.x, v.y) == (3.0, 4.0)
    assert v.norm() == pytest.approx(5.0)
    assert v.normalize() == ns.Vec2(0.6, 0.8)


def test_vec2_from_tuple(ns):
    np = pytest.importorskip("numpy")
    dtype = np.float32 if ns is dim2 else np.float64
    a = ns.Vec2(1, 2)
    b = ns.Vec2.from_tuple((1.0, 2.0))
    c = ns.Vec2.from_ndarray(np.array([1.0, 2.0], dtype=dtype))
    assert a == b == c


def test_vec2_add_sub(ns):
    assert ns.Vec2(1, 2) + ns.Vec2(3, 4) == ns.Vec2(4, 6)
    assert ns.Vec2(3, 4) - (1, 2) == ns.Vec2(2, 2)
    assert -ns.Vec2(1, 2) == ns.Vec2(-1, -2)


def test_vec2_dot(ns):
    assert ns.Vec2(1, 2).dot((3, 4)) == pytest.approx(11.0)


def test_vec2_iter(ns):
    assert list(ns.Vec2(1, 2)) == [1.0, 2.0]
    assert len(ns.Vec2(1, 2)) == 2


def test_point2_sub_returns_vec(ns):
    d = ns.Point2(5, 7) - ns.Point2(1, 2)
    assert isinstance(d, ns.Vec2)
    assert d == ns.Vec2(4, 5)


def test_rotation2_angle(ns):
    r = ns.Rotation2.from_angle(pymath.pi / 2)
    assert r.angle == pytest.approx(pymath.pi / 2)
    assert r.transform_vector((1, 0)) == ns.Vec2(0, 1)
    assert (r.inverse() * r) == ns.Rotation2.identity()


def test_rotation2_compose(ns):
    a = ns.Rotation2.from_angle(pymath.pi / 4)
    b = ns.Rotation2.from_angle(pymath.pi / 4)
    composed = a * b
    assert composed.transform_vector((1, 0)) == ns.Vec2(0, 1)


def test_rotation2_slerp(ns):
    # Use pi/2 instead of pi: slerp at exactly 180° is ambiguous in direction.
    a = ns.Rotation2.identity()
    b = ns.Rotation2.from_angle(pymath.pi / 2)
    mid = a.slerp(b, 0.5)
    assert mid == ns.Rotation2.from_angle(pymath.pi / 4)


def test_isometry2(ns):
    iso = ns.Isometry2(translation=(1, 2), rotation=ns.Rotation2.identity())
    assert iso * ns.Point2(0, 0) == ns.Point2(1, 2)

    iso2 = ns.Isometry2.from_translation(3, 4)
    assert iso2.transform_point((1, 1)) == ns.Point2(4, 5)


def test_isometry2_compose(ns):
    a = ns.Isometry2.from_translation(1, 0)
    b = ns.Isometry2.from_rotation(pymath.pi / 2)
    composed = a * b
    p = composed.transform_point((1, 0))
    assert p == ns.Point2(1, 1)


def test_rotation_from_angle_2d(ns):
    r = ns.rotation_from_angle(pymath.pi / 2)
    assert r.transform_vector((1, 0)) == ns.Vec2(0, 1)
