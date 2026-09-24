"""Read C export names from the same receiver annotations used by rapier_export."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
EXPORT_ATTRIBUTE = r"#\[rapier_export(?:\(([a-z][a-z0-9_]*)\))?\]"
DECLARATION = r'pub (?:unsafe )?extern "C" fn (rpr_[a-z0-9_]+)'


def pascal_case(name):
    if not re.fullmatch(r"[a-z0-9]+(?:_[a-z0-9]+)*", name):
        raise ValueError(f"Invalid snake_case export name: {name}")
    return "".join(word[0].upper() + word[1:] for word in name.split("_"))


def export_suffix(name, receiver=None):
    if not name.startswith("rpr_"):
        raise ValueError(f"Missing rpr_ prefix: {name}")
    suffix = name[4:]
    if receiver is None:
        return pascal_case(suffix)
    if not suffix.startswith(receiver + "_"):
        raise ValueError(f"{name} does not start with receiver {receiver}")
    return pascal_case(receiver) + "_" + pascal_case(suffix[len(receiver) + 1:])


def read_exports():
    exports = {}
    # Attributes may be followed by cfg attributes and documentation comments.
    pattern = EXPORT_ATTRIBUTE + r"(?:(?!#\[rapier_export)[\s\S])*?" + DECLARATION
    for source in sorted((ROOT / "c/src").glob("*.rs")):
        text = source.read_text()
        declarations = set(re.findall(DECLARATION, text))
        annotated = set()
        for receiver, name in re.findall(pattern, text):
            suffix = export_suffix(name, receiver or None)
            if name in exports and exports[name] != suffix:
                raise ValueError(f"Conflicting export annotations: {name}")
            exports[name] = suffix
            annotated.add(name)
        if declarations != annotated:
            raise ValueError(f"Missing export annotation in {source}: {declarations - annotated}")
    if len(set(exports.values())) != len(exports):
        raise ValueError("Duplicate C export names")
    return exports


EXPORTS = read_exports()


def c_name(name, dimension):
    return f"r{dimension}{EXPORTS[name]}"
