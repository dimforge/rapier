"""Smoke tests.

These verify that the umbrella `rapier` package is importable and that the
exception tree and dim/scalar namespaces are wired correctly. They do NOT
exercise any physics — that's covered by the other test modules.
"""

from __future__ import annotations

import rapier3d as rapier
from rapier3d import RapierError
import rapier2d as dim2
import rapier3d as dim3
import rapier3d_f64 as dim3_f64
import rapier2d_f64 as dim2_f64


def test_version() -> None:
    assert isinstance(rapier.__version__, str)
    assert rapier.__version__  # non-empty


def test_error_hierarchy() -> None:
    assert issubclass(rapier.InvalidHandle, RapierError)
    assert issubclass(rapier.UrdfError, RapierError)
    assert issubclass(rapier.MjcfError, RapierError)
    assert issubclass(rapier.MeshConversionError, RapierError)
    assert issubclass(rapier.MeshLoaderError, RapierError)
    assert issubclass(rapier.SerializationError, RapierError)
    assert issubclass(rapier.QueryFailure, RapierError)


def test_error_is_exception() -> None:
    assert issubclass(RapierError, Exception)
    with __import__("pytest").raises(RapierError):
        raise RapierError("boom")


def test_dim_namespaces() -> None:
    assert dim2 is not dim3
    assert dim3_f64 is not dim3
    assert dim2_f64 is not dim2
    # The exception type is *defined per-extension*; reachable from each.
    assert hasattr(dim2, "RapierError")
    assert hasattr(dim3, "RapierError")
    assert hasattr(dim3_f64, "RapierError")
    assert hasattr(dim2_f64, "RapierError")


def test_dim_versions() -> None:
    assert isinstance(dim2.__version__, str)
    assert isinstance(dim3.__version__, str)
    assert isinstance(dim3_f64.__version__, str)
    assert isinstance(dim2_f64.__version__, str)
