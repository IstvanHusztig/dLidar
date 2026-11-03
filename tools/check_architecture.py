#!/usr/bin/env python3
"""Checks the dependency rule of dLidar's layers (docs/architecture.md).

    check_architecture.py [--strict] [SRC_DIR]     (default: src/ next to tools/)
    check_architecture.py --self-test

Layers under SRC_DIR and what their #includes may resolve to:
  domain/        C/C++ standard headers and domain/
  application/   the above plus application/
  adapters/<x>/  anything except app/ and other adapters
  app/           anything
Quoted includes are resolved against the including file's directory, then
SRC_DIR. Angle includes in domain/ and application/ must be standard headers.
--strict also fails on sources outside the layers and on a leftover include/
directory next to SRC_DIR (the pre-refactor layout).
Exit code 0 if the rules hold, 1 with one line per violation otherwise.
"""
import re
import sys
import tempfile
from pathlib import Path

C_HEADERS = {"assert.h", "ctype.h", "errno.h", "float.h", "inttypes.h", "limits.h", "math.h", "stddef.h",
             "stdint.h", "stdio.h", "stdlib.h", "string.h", "time.h"}
INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]', re.M)
CORE = ("domain", "application")
SOURCES = (".h", ".hpp", ".cpp", ".cc")


def layer(rel):
    """('domain',) / ('application',) / ('adapters', name) / ('app',) / None for a path relative to src."""
    parts = rel.parts
    if not parts:
        return None
    if parts[0] in CORE + ("app",):
        return (parts[0],)
    if parts[0] == "adapters" and len(parts) > 2:
        return ("adapters", parts[1])
    return None


def allowed(src_layer, dst_layer):
    if src_layer[0] == "app":
        return True
    if dst_layer is None:
        return src_layer[0] == "adapters"  # e.g. 3rd-party headers found on disk
    if src_layer[0] == "domain":
        return dst_layer[0] == "domain"
    if src_layer[0] == "application":
        return dst_layer[0] in CORE
    # adapters: core and their own adapter only
    return dst_layer[0] in CORE or dst_layer == src_layer


def standard_header(name):
    return name in C_HEADERS or ("/" not in name and "." not in name)


def check(src, strict):
    src = src.resolve()
    errors = []
    for f in sorted(p for p in src.rglob("*") if p.suffix in SOURCES and p.is_file()):
        rel = f.relative_to(src)
        own = layer(rel)
        if own is None:
            if strict:
                errors.append(f"{rel}: not inside a layer directory")
            continue
        for kind, name in INCLUDE.findall(f.read_text(errors="replace")):
            if kind == "<":
                if own[0] in CORE and not standard_header(name):
                    errors.append(f"{rel}: {own[0]} includes non-standard <{name}>")
                continue
            target = next((c.resolve() for c in (f.parent / name, src / name) if c.is_file()), None)
            if target is None:
                if own[0] in CORE:
                    errors.append(f'{rel}: {own[0]} includes "{name}", not found under {src.name}/')
                continue
            try:
                dst = layer(target.relative_to(src))
            except ValueError:
                dst = None  # outside src/ (vendor lib dir, 3rd/)
            if not allowed(own, dst):
                where = "/".join(dst) if dst else "outside the layers"
                errors.append(f'{rel}: {"/".join(own)} must not include "{name}" ({where})')
    if strict and (src.parent / "include").is_dir():
        errors.append(f"{src.parent.name}/include/ still exists (pre-refactor layout)")
    return errors


def self_test():
    with tempfile.TemporaryDirectory() as d:
        src = Path(d) / "src"
        files = {
            "domain/a.h": '#include <vector>\n#include <stdint.h>\n#include "domain/b.h"\n',
            "domain/b.h": "",
            "application/c.h": '#include "domain/a.h"\n#include "application/d.h"\n',
            "application/d.h": "",
            "adapters/x/e.cpp": '#include "application/c.h"\n#include "adapters/x/f.h"\n#include <dll/laszip_api.h>\n',
            "adapters/x/f.h": "",
            "adapters/y/g.h": "",
            "app/main.cpp": '#include "adapters/x/f.h"\n#include "adapters/y/g.h"\n',
        }
        for name, text in files.items():
            (src / name).parent.mkdir(parents=True, exist_ok=True)
            (src / name).write_text(text)
        ok = check(src, True) == []
        bad = {
            "domain/bad1.h": '#include "application/d.h"\n',
            "domain/bad2.h": "#include <unitree_lidar_sdk.h>\n",
            "domain/bad3.h": '#include "unitree_lidar_sdk.h"\n',
            "application/bad4.h": '#include "adapters/x/f.h"\n',
            "adapters/x/bad5.h": '#include "adapters/y/g.h"\n',
            "adapters/y/bad6.h": '#include "app/main.cpp"\n',
            "stray.h": "",
        }
        for name, text in bad.items():
            (src / name).parent.mkdir(parents=True, exist_ok=True)
            (src / name).write_text(text)
        (Path(d) / "include").mkdir()
        errors = check(src, True)
        caught = all(any(e.startswith(n) for e in errors) for n in bad) and any("include/" in e for e in errors)
        print("self-test:", "ok" if ok and caught else "FAILED", *([] if caught else errors), sep="\n  ")
        return ok and caught


def main(argv):
    if "--self-test" in argv:
        return 0 if self_test() else 1
    strict = "--strict" in argv
    rest = [a for a in argv if not a.startswith("--")]
    src = Path(rest[0]) if rest else Path(__file__).resolve().parent.parent / "src"
    errors = check(src, strict)
    for e in errors:
        print(e)
    print(f"architecture check ({'strict' if strict else 'permissive'}): "
          f"{'ok' if not errors else f'{len(errors)} violation(s)'}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
