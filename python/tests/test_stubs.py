"""Consistency between the type stubs (`*.pyi`) and the runtime package."""

from __future__ import annotations

import ast
import pathlib

import pytest

import rapier3d
import rapier3d._rapier3d
import rapier3d.math

PACKAGE = pathlib.Path(rapier3d.__file__).parent


def _parse(name):
    return ast.parse((PACKAGE / name).read_text())


def _stub_classes():
    return {n.name: n for n in _parse("_rapier3d.pyi").body if isinstance(n, ast.ClassDef)}


def _members(node):
    names = set()
    for item in node.body:
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
            names.add(item.name)
        elif isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name):
            names.add(item.target.id)
        elif isinstance(item, ast.Assign):
            names.update(t.id for t in item.targets if isinstance(t, ast.Name))
    return names


def test_init_stub_reexports_the_whole_package():
    tree = _parse("__init__.pyi")
    imported = set()
    declared_all = None
    for node in tree.body:
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                # Explicit re-exports use the `name as name` form.
                assert alias.asname == alias.name, alias.name
                imported.add(alias.asname)
        elif isinstance(node, ast.Assign) and node.targets[0].id == "__all__":
            declared_all = [elt.value for elt in node.value.elts]
        elif isinstance(node, ast.AnnAssign):
            imported.add(node.target.id)
    assert declared_all == rapier3d.__all__
    assert set(rapier3d.__all__) <= imported


def test_math_stub_matches_the_module():
    stubbed = {n.name for n in _parse("math.pyi").body if isinstance(n, ast.FunctionDef)}
    assert set(rapier3d.math.__all__) == stubbed


def _runtime_classes():
    """The public classes of the extension, except the exceptions and the iterators."""
    ext = rapier3d._rapier3d
    return sorted(
        name
        for name in dir(ext)
        if not name.startswith("_")
        and isinstance(getattr(ext, name), type)
        and not issubclass(getattr(ext, name), BaseException)
        and not name.endswith("Iter")
        # Aliases of other classes (`Quaternion = Rotation3`).
        and getattr(ext, name).__name__ == name
    )


@pytest.mark.parametrize("name", _runtime_classes())
def test_class_stub_matches_the_runtime(name):
    stub = _stub_classes().get(name)
    assert stub is not None, f"{name} has no stub"
    runtime = getattr(rapier3d._rapier3d, name)
    runtime_members = {m for m in dir(runtime) if not m.startswith("_")}
    stub_members = {m for m in _members(stub) if not m.startswith("_")}
    assert stub_members == runtime_members


def test_no_duplicated_stub_classes():
    names = [n.name for n in _parse("_rapier3d.pyi").body if isinstance(n, ast.ClassDef)]
    assert len(names) == len(set(names))


def test_build_features_is_stubbed():
    functions = {n.name for n in _parse("_rapier3d.pyi").body if isinstance(n, ast.FunctionDef)}
    assert "build_features" in functions


@pytest.mark.parametrize("name", ["Vec3", "Point3"])
def test_immutable_coordinates_are_read_only_properties(name):
    stub = _stub_classes()[name]
    properties = {
        item.name
        for item in stub.body
        if isinstance(item, ast.FunctionDef)
        and any(isinstance(d, ast.Name) and d.id == "property" for d in item.decorator_list)
    }
    assert {"x", "y", "z"} <= properties
    value = getattr(rapier3d, name)(1.0, 2.0, 3.0)
    with pytest.raises(AttributeError):
        value.x = 0.0


def test_stubs_type_check():
    api = pytest.importorskip("mypy.api")
    stubs = [str(PACKAGE / name) for name in ("_rapier3d.pyi", "__init__.pyi", "math.pyi")]
    stdout, stderr, status = api.run(["--no-incremental", *stubs])
    assert status == 0, stdout + stderr
