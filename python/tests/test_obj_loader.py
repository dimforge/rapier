"""Tests for the testbed's minimal OBJ loader (``rapier_testbed._obj``).

Skipped unless the testbed package is installed (it isn't part of the engine
test flow); install with ``pip install --no-deps -e ./python/rapier-testbed``.
"""
import numpy as np
import pytest

_obj = pytest.importorskip("rapier_testbed._obj")


def _write(tmp_path, text):
    p = tmp_path / "mesh.obj"
    p.write_text(text)
    return p


def test_triangle_quad_and_comments(tmp_path):
    # A quad face must be fan-triangulated into two triangles; comments,
    # blank lines, and texture/normal components are ignored.
    obj = """
# a comment
v 0 0 0
v 1 0 0
v 1 1 0
v 0 1 0
f 1/1/1 2/2/1 3/3/1 4/4/1
"""
    verts, idx = _obj.load_obj(_write(tmp_path, obj))
    assert verts.shape == (4, 3)
    assert idx.shape == (2, 3)
    # Fan triangulation of [0,1,2,3] -> (0,1,2), (0,2,3).
    assert idx.tolist() == [[0, 1, 2], [0, 2, 3]]
    assert idx.dtype == np.uint32
    assert verts.dtype == np.float32


def test_negative_indices(tmp_path):
    # OBJ allows negative (relative-to-end) indices: -1 is the last vertex.
    obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n"
    verts, idx = _obj.load_obj(_write(tmp_path, obj))
    assert idx.tolist() == [[0, 1, 2]]


def test_normalize_to_unit_centers_and_scales():
    verts = np.array(
        [[0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2]], dtype=np.float32
    )
    out = _obj.normalize_to_unit(verts, target_diag=10.0)
    # Centered: the AABB center maps to the origin.
    mn, mx = out.min(axis=0), out.max(axis=0)
    np.testing.assert_allclose((mn + mx) * 0.5, [0, 0, 0], atol=1e-5)
    # Scaled: the AABB diagonal becomes target_diag.
    assert np.linalg.norm(mx - mn) == pytest.approx(10.0, rel=1e-5)
