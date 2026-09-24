#!/usr/bin/env python3
"""Test all native ABIs, exported names, linkage modes, and dimension coexistence."""
import ctypes
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

from export_names import EXPORTS, c_name

ROOT = Path(__file__).resolve().parents[2]
VERSION = (ROOT / "c/VERSION").read_text().strip()
CC = shlex.split(os.environ.get("CC", "cc"))
CXX = shlex.split(os.environ.get("CXX", "c++"))
VARIANTS = [(d, p) for d in (2, 3) for p in (32, 64)]
PACKAGES = [f"rapier{d}d" + ("-f64" if p == 64 else "") + "-ffi" for d, p in VARIANTS]
subprocess.run(["cargo", "build", *[arg for package in PACKAGES for arg in ("-p", package)]],
               cwd=ROOT, check=True)
metadata = subprocess.check_output(["cargo", "metadata", "--no-deps", "--format-version", "1"],
                                   cwd=ROOT, text=True)
LIBDIR = Path(json.loads(metadata)["target_directory"]) / "debug"
SYSTEM_LIBS = ["-lpthread", "-lm"] + (["-framework", "Security", "-framework", "CoreFoundation"]
                                     if sys.platform == "darwin" else ["-ldl", "-lrt", "-lutil"])


def link_flags(libraries, linkage):
    if linkage == "shared":
        return [f"-L{LIBDIR}", *[f"-l{lib}" for lib in libraries], f"-Wl,-rpath,{LIBDIR}"]
    return [*[str(LIBDIR / f"lib{lib}.a") for lib in libraries], *SYSTEM_LIBS]


rust_names = set(EXPORTS)

with tempfile.TemporaryDirectory(prefix="rapier-c-tests-") as directory:
    temp = Path(directory)
    for (dim, precision), package in zip(VARIANTS, PACKAGES):
        library = package.replace("-", "_")
        flags = ["-Wall", "-Wextra", "-Werror", "-UNDEBUG", "-I", str(ROOT / "c/include"),
                 f"-DRAPIER_DIM{dim}", f"-DRAPIER_F{precision}",
                 f'-DRAPIER_EXPECTED_VERSION="{VERSION}"']
        header = subprocess.check_output([*CC, "-E", "-P", "-x", "c", *flags,
                                          str(ROOT / "c/include/rapier.h")], text=True)
        symbols = set(re.findall(r"\b(r[23][A-Z]\w*)\s*\(", header))
        assert symbols and all(name.startswith(f"r{dim}") for name in symbols)
        assert f"r{dim}InsertRigidBody" in symbols
        # Reject the replaced abbreviated free-function names.
        for old, new in {
            "RemoveBody": "RemoveRigidBody",
            "InsertDeformable": "InsertDeformableCollider",
            "ActiveBodies": "ActiveRigidBodies",
            "Parameters": "IntegrationParameters",
            "SetParameters": "SetIntegrationParameters",
            "Serialize": "SerializeWorld",
            "Deserialize": "DeserializeWorld",
            "RigidBodyLen": "RigidBodyCount",
            "ColliderLen": "ColliderCount",
            "SoftBodyLen": "SoftBodyCount",
            "ReadRigidBodyLen": "ReadRigidBodyCount",
            "ReadColliderLen": "ReadColliderCount",
            "Dt": "TimeStep",
            "SetDt": "SetTimeStep",
        }.items():
            assert f"r{dim}{old}" not in symbols
            assert f"r{dim}{new}" in symbols
        assert f"r{dim}InsertColliderWithoutParent" in symbols
        assert f"r{dim}Insert" not in symbols and f"r{dim}InsertBody" not in symbols
        assert not re.search(rf"\bR{dim}BodyCollider\b", header)
        assert not re.search(r"\brpr_\w+\s*\(", header)
        assert not re.search(r"\b(?:Rpr\w*|RPR_\w+|R" + str(5 - dim) + r"[A-Z_]\w*)\b", header)
        macros = subprocess.check_output([*CC, "-E", "-dM", "-x", "c", *flags,
                                          str(ROOT / "c/include/rapier.h")], text=True)
        assert re.search(rf"^#define R{dim}_OK 0$", macros, re.M)
        assert re.search(rf"^#define R{dim}_ABI_VERSION 1$", macros, re.M)
        assert not re.search(r"^#define (?:RPR_|R" + str(5 - dim) + r"_)", macros, re.M)
        # Produced values must use direct returns, not the former scalar output pointers.
        declarations = re.findall(r"\br[23][A-Z]\w*\([^;]*?\);", header, re.S)
        assert not any(re.search(r"\*\s*(?:out(?:_\w+)?|count|found)\b", d)
                       for d in declarations)
        # ABI 7 has one owner: no legacy component objects, query views, or aliases.
        for legacy_type in ("PhysicsWorld", "PhysicsPipeline", "CollisionPipeline",
                            "RigidBodySet", "ColliderSet", "SoftBodySet",
                            "ImpulseJointSet", "MultibodyJointSet", "QueryView"):
            assert not re.search(rf"\bR{dim}" + legacy_type + r"\b", header), legacy_type
        assert not any(re.match(r"r[23](PhysicsWorld|PhysicsPipeline|CollisionPipeline|QueryView)",
                                symbol) for symbol in symbols)
        # Compare the entire dynamic export set, not just the names used in examples.
        extension = "dylib" if sys.platform == "darwin" else "so"
        binary = LIBDIR / f"lib{library}.{extension}"
        nm_args = ["-gU"] if sys.platform == "darwin" else ["-D", "--defined-only"]
        exports = subprocess.check_output(["nm", *nm_args, str(binary)], text=True)
        actual = set(re.findall(r"\b(r[23][A-Z]\w*)$", exports, re.MULTILINE))
        # macOS nm prints a leading underscore before C symbols.
        if sys.platform == "darwin":
            actual = set(re.findall(r"\b_(r[23][A-Z]\w*)$", exports, re.MULTILINE))
        assert actual == symbols, (package, "missing", symbols - actual, "unexpected", actual - symbols)
        assert not re.search(r"\b_?rpr_\w+$", exports, re.MULTILINE)
        loaded = ctypes.CDLL(str(binary))
        for name in rust_names:
            assert not hasattr(loaded, name), (package, "legacy export", name)
            assert not hasattr(loaded, c_name(name, 5 - dim)), (package, "wrong dimension", name)
            if "_" in EXPORTS[name]:
                old_name = c_name(name, dim).replace("_", "")
                assert not hasattr(loaded, old_name), (package, "obsolete method export", old_name)

        symbol_source = temp / "symbols.c"
        symbol_source.write_text(
            '#include "rapier.h"\nstatic void (*const symbols[])(void) = {\n'
            + "".join(f"    (void (*)(void)){name},\n" for name in sorted(symbols))
            + "};\nint main(void) { for (unsigned i = 0; i < sizeof(symbols)/sizeof(symbols[0]); ++i) "
              "if (!symbols[i]) return 1; return 0; }\n")
        for linkage in ("shared", "static"):
            for source, compiler, standard in [(ROOT / "c/tests/integration.c", CC, "c11"),
                                                (ROOT / "c/tests/cpp.cpp", CXX, "c++17"),
                                                (ROOT / "c/tests/pod.c", CC, "c11"),
                                                (ROOT / "c/tests/handles.c", CC, "c11"),
                                                (ROOT / "c/tests/initializers.c", CC, "c11"),
                                                (ROOT / "c/tests/initializers.cpp", CXX, "c++17"),
                                                (ROOT / "c/tests/array_views.c", CC, "c11"),
                                                (ROOT / "c/tests/array_views.cpp", CXX, "c++17"),
                                                (symbol_source, CC, "c11")]:
                output = temp / f"{source.stem}-{library}-{linkage}"
                subprocess.run([*compiler, f"-std={standard}", *flags, str(source),
                                *link_flags([library], linkage), "-o", str(output)], check=True)
                subprocess.run([str(output)], check=True)
        print(f"{package}: C, C++, and {len(symbols)} exports passed shared + static linking", flush=True)

    # Each dimension keeps its own POD layouts in a separate translation unit.
    # Both libraries must coexist in one executable without symbol collisions.
    main = temp / "both.c"
    main.write_text("int run2(void); int run3(void); int main(void) { return run2() || run3(); }\n")
    for precision in (32, 64):
        objects, libraries = [], []
        for dim in (2, 3):
            source, obj = temp / f"dim{dim}.c", temp / f"dim{dim}.o"
            source.write_text(f'''#include "rapier.h"
int run{dim}(void) {{
    R{dim}World *world = 0;
    if (r{dim}CheckAbi(R{dim}_ABI_VERSION, {dim}, sizeof(R{dim}Real), sizeof(R{dim}Vector), sizeof(R{dim}Pose), R{dim}_ABI_FEATURES)) return 1;
    world = r{dim}NewWorld();
    if (r{dim}LastStatus()) return 2;
    R{dim}Status status = r{dim}Step(world, 0, 0);
    return r{dim}FreeWorld(world) || status;
}}
''')
            subprocess.run([*CC, "-std=c11", "-Wall", "-Wextra", "-Werror",
                            f"-DRAPIER_DIM{dim}", f"-DRAPIER_F{precision}", "-I", str(ROOT / "c/include"),
                            "-c", str(source), "-o", str(obj)], check=True)
            objects.append(str(obj))
            libraries.append(f"rapier{dim}d" + ("_f64" if precision == 64 else "") + "_ffi")
        for linkage in ("shared", "static"):
            output = temp / f"both-f{precision}-{linkage}"
            subprocess.run([*CC, str(main), *objects, *link_flags(libraries, linkage),
                            "-o", str(output)], check=True)
            subprocess.run([str(output)], check=True)
        print(f"2D + 3D / f{precision}: coexistence passed shared + static linking", flush=True)
