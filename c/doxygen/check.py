#!/usr/bin/env python3
"""Check that Doxygen documents every function visible to the C compiler."""
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

xml_dir, dimension, precision, compiler, compiler_id = sys.argv[1:]
xml_dir = Path(xml_dir)
c_dir = Path(__file__).resolve().parents[1]
prefix = f"r{dimension}"
definitions = [f"RAPIER_DIM{dimension}", f"RAPIER_F{precision}", "RAPIER_FEM", "RAPIER_PARALLEL", "RAPIER_STATIC"]
if (dimension, precision) == ("3", "32"):
    definitions.append("RAPIER_ROBOTICS")
with tempfile.TemporaryDirectory(prefix="rapier-doc-check-") as tmp:
    source = Path(tmp) / "api.c"
    source.write_text('#include "rapier_math.h"\n#include "rapier_helpers.h"\n')
    if compiler_id == "MSVC":
        args = [compiler, "/nologo", "/EP", "/TC", f"/I{c_dir / 'include'}", *[f"/D{x}" for x in definitions], str(source)]
    else:
        args = [compiler, "-E", "-P", "-x", "c", "-I", str(c_dir / "include"), *[f"-D{x}" for x in definitions], str(source)]
    if sys.platform == "darwin":
        sdk = subprocess.check_output(["xcrun", "--show-sdk-path"], text=True).strip()
        args[1:1] = ["-isysroot", sdk]
    preprocessed = subprocess.check_output(args, text=True)
expected = set(re.findall(rf"\b({prefix}\w+)\s*\(", preprocessed))
index = ET.parse(xml_dir / "index.xml")
actual = {m.findtext("name") for m in index.findall('.//member[@kind="function"]') if m.findtext("name", "").startswith(prefix)}
if missing := expected - actual:
    raise SystemExit(f"Functions missing from reference: {sorted(missing)}")
if extra := actual - expected:
    raise SystemExit(f"Functions from the wrong build variant: {sorted(extra)}")
# Function comments need a brief, not just an @ingroup/@see tag. Also reject
# accidental internal Rust names leaking through the header generator.
checked = set()
for compound in index.findall("compound"):
    path = xml_dir / f"{compound.attrib['refid']}.xml"
    tree = ET.parse(path)
    for member in tree.findall(".//memberdef"):
        name = member.findtext("name", "")
        if name not in actual or name in checked:
            continue
        checked.add(name)
        brief = "".join(member.find("briefdescription").itertext()).strip()
        detail = "".join(member.find("detaileddescription").itertext()).strip()
        if not brief:
            raise SystemExit(f"Missing brief description: {name}")
        if re.search(r"\brpr_\w+|\bRpr\w+|\bRPR_\w+", brief + detail):
            raise SystemExit(f"Internal Rust spelling in documentation: {name}")
if not expected:
    raise SystemExit("No C API functions found during preprocessing")
print(f"Verified {len(expected)} documented functions for {dimension}D/f{precision}.")
