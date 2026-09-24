#!/usr/bin/env python3
"""Generate dimension-specific C names from the shared Rust binding source."""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from export_names import EXPORT_ATTRIBUTE, c_name

ROOT = Path(__file__).resolve().parents[2]
DESTINATION = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "c/include/rapier.h"


with tempfile.TemporaryDirectory(prefix="rapier-header-") as tmp:
    project = Path(tmp)
    source_dir = project / "src"
    source_dir.mkdir()
    # cbindgen parses source without expanding custom attributes. Give it a
    # temporary declaration view where our export marker is the standard one.
    # Signatures, docs, types, and feature gates all come from the real source.
    sources = sorted((ROOT / "c/src").glob("*.rs"))
    for source in sources:
        text = re.sub(EXPORT_ATTRIBUTE, "#[unsafe(no_mangle)]", source.read_text())
        (source_dir / source.name).write_text(text)
    (project / "Cargo.toml").write_text('''[package]
name = "rapier-c-header"
version = "0.0.0"
edition = "2024"
[workspace]
[features]
default = ["dim3", "f32"]
dim2 = []
dim3 = []
f32 = []
f64 = []
fem = []
parallel = []
robotics = []
''')
    raw = project / "rapier.h"
    subprocess.run([
        "cbindgen", "--config", str(ROOT / "c/cbindgen.toml"),
        "--crate", "rapier-c-header", "--output", str(raw),
    ], cwd=project, check=True)
    text = raw.read_text()
    # MSVC requires __cdecl after the return type, notably for struct returns.
    # cbindgen's function prefix goes before that type, so insert the calling
    # convention at the function declarator instead.
    text = re.sub(r"(RAPIER_API\s+[\w\s*]+?)\b(rpr_[a-z0-9_]+)\(",
                  r"\1RAPIER_CALL \2(", text)
    # Transparent native wrappers remain opaque to C.
    for source in sources:
        for name, native in re.findall(r"pub struct (Rpr\w+)\(pub\(crate\) (\w+)\)", source.read_text()):
            text = text.replace(f"typedef {native} {name};", f"typedef struct {name} {name};")
    for name in ["RprPairFilter", "RprModifyContacts", "RprErrorCallback", "RprModifyContactContext", "RprIkJointCanMove", "RprQueryPredicate", "RprCollisionEventCallback", "RprContactForceEventCallback"]:
        text = text.replace(f"(*{name})", f"(RAPIER_CALL *{name})")
    text = text.replace("\n#endif\n  ;", ";\n#endif")

    # cbindgen emits this mutually recursive pair in pointer-dependency order.
    # C requires the by-value ShapeDesc field to be complete first.
    compound = re.search(r"(?:/\*\*(?:(?!\*/).)*\*/\s*)?typedef struct RprCompoundShapeDesc \{.*?\} RprCompoundShapeDesc;\n", text, re.S)
    if compound:
        declaration = compound[0]
        text = text[:compound.start()] + text[compound.end():]
        shape_end = text.index("} RprShapeDesc;") + len("} RprShapeDesc;")
        text = text[:shape_end] + "\n\n" + declaration + text[shape_end:]

    # Rust stays dimension-neutral. Emit concrete C type, constant, and function
    # names for each selected dimension, without compatibility aliases.
    start = list(re.finditer(r"^#include <[^>]+>\n", text, re.M))[-1].end()
    end = text.rindex("#endif  /* RAPIER_H */")
    declarations = text[start:end]
    variants = []
    for dim in (2, 3):
        variant = re.sub(r"\brpr_[a-z0-9_]+\b", lambda m: c_name(m[0], dim), declarations)
        variant = re.sub(r"\bRpr(\w+)\b", lambda m: f"R{dim}{m[1]}", variant)
        variant = re.sub(r"\bRPR_(\w+)\b", lambda m: f"R{dim}_{m[1]}", variant)
        variants.append(variant)
    text = (text[:start] + "\n#if defined(RAPIER_DIM2)\n" + variants[0]
            + "#else /* RAPIER_DIM3 */\n" + variants[1] + "#endif\n\n" + text[end:])
    DESTINATION.parent.mkdir(parents=True, exist_ok=True)
    DESTINATION.write_text(text)
