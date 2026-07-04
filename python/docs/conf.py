"""Sphinx configuration for the Rapier Python bindings.

These docs cover the ``rapier3d`` engine package (3D / f32).

Build from the ``python/docs/`` directory after the engine package is
installed (``pip install ./python/rapier-py-3d``, or ``maturin develop``
in the crate)::

    sphinx-build -b html . _build/html
"""

from __future__ import annotations

import os
import sys
import warnings

# -- Path bootstrap ---------------------------------------------------------
#
# We need to ``import rapier3d`` so autodoc can read the real pyclass
# docstrings populated by PyO3 (``///`` Rust doc comments → Python
# ``__doc__``). The package is normally installed editable (``maturin
# develop``). As a fallback — to support running ``sphinx-build`` from a
# stand-alone checkout where nothing has been installed yet — we also
# prepend the package's ``python/`` source dir to ``sys.path``.

_HERE = os.path.dirname(os.path.abspath(__file__))
for _crate in ("rapier-py-3d",):
    _src = os.path.abspath(os.path.join(_HERE, "..", _crate, "python"))
    if _src not in sys.path:
        sys.path.insert(0, _src)

# -- Project information ----------------------------------------------------

project = "rapier"
author = "Sébastien Crozet"
copyright = "2024-2026, Dimforge"

try:
    import rapier3d as _rapier  # type: ignore[import-not-found]
    version = _rapier.__version__
except Exception as _import_err:  # pragma: no cover - diagnostic path only
    warnings.warn(
        f"Could not import `rapier3d` to read __version__ ({_import_err!r}); "
        "falling back to '0.32.0'. Did you install the engine packages "
        "(e.g. `maturin develop` in python/rapier-py-3d)?",
        stacklevel=1,
    )
    version = "0.32.0"

release = version

# -- General Sphinx configuration ------------------------------------------

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
    "sphinx.ext.autosummary",
    "sphinx_autodoc_typehints",
]

# We author docstrings in reST directly — disable Napoleon's
# Google/Numpy heuristics so it doesn't try to "rescue" reST text that
# looks Numpy-ish.
napoleon_google_docstring = False
napoleon_numpy_docstring = False

autodoc_default_options = {
    "members": True,
    "undoc-members": False,
    "show-inheritance": True,
    "inherited-members": False,
}
# Render class + __init__ doc together when ``autoclass::`` is used.
autoclass_content = "both"
# Put type hints in the description block (next to :param:) rather than
# leaking them into the rendered signature.
autodoc_typehints = "description"
autodoc_typehints_format = "short"

# Don't qualify `rapier.dim3.RigidBody` etc. with the module prefix in
# headings — the namespace context is set per-page via
# ``.. currentmodule::``.
add_module_names = False

# autosummary stubs are generated on first build for any
# ``.. autosummary:: :toctree:`` directive we add later.
autosummary_generate = True

# Cross-reference targets for the wider Python ecosystem.
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
}

# Templates and excludes.
templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "repackaging"]

# -- HTML output ------------------------------------------------------------

html_theme = "furo"
html_title = f"rapier {release}"
html_static_path: list[str] = []

# Suppress a small number of expected-noisy warnings that come from the
# upstream pyclass surface (e.g. PyO3 emitting an inherited ``__new__``
# without a docstring under our ``undoc-members: False`` setting).
# These don't reflect doc-quality issues.
nitpicky = False
