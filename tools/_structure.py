"""Fail the build on parts that are in the wrong place, duplicated, or alone.

`audit_geometry.py` counts vertices, and counting vertices is the metric that
let all of this through: an air intake floating 44 mm off the fuselage, two
steering wheels in the same cockpit, eight pairs of connecting rods modelled
inside each other, and a part named `cone` that is 0.6 mm long and 340 mm
across. Every one of those cleared a vertex floor, because a wrong shape
finely tessellated has just as many vertices as a right one.

So this checks the four things a vertex count cannot see:

  attached    every part touches something. A part with clear air all round
              it is either in the wrong place or should not exist.
  mirrored    a part whose name ends _l / _fl / _rl must be the mirror of its
              _r / _fr / _rr twin, in x and z exactly and in y by reflection.
  distinct    no two parts occupy the same space. Two objects with the same
              bounding box are one part built twice by two modules that have
              never heard of each other.
  shaped      a part named for a shape has to have it. A cone is not flat, a
              ring is not solid, a tube is longer than it is wide.
  single      there is exactly one of the things there is exactly one of. Two
              modules that have never heard of each other will each build a
              steering wheel, and both will pass every other check.
  clear       named pairs that must not occupy each other. The hybrid battery
              was rammed 46 mm up into the oil pan, with the sump drain plug
              inside it.
  joined      named pairs that must reach each other. "Touches something" is
              not enough: a driveshaft that stops 80 mm short of its hub still
              touches the gearbox at the other end.

Each project supplies its own config: the mirror tolerance, the parts that
are legitimately asymmetric (a deflected control surface, a differential), and
the ones that are legitimately alone. Exemptions are listed by name with a
reason, so adding one shows up in a diff and has to be argued for.

    python3 tools/audit_structure.py
"""

import csv
import math
import os
import sys

AXES = ("x_min_mm", "x_max_mm", "y_min_mm", "y_max_mm", "z_min_mm", "z_max_mm")

# A part whose name contains one of these claims a shape. If the geometry does
# not have it, the name is a lie and the part will read as wrong however many
# vertices it has.
SHAPE_RULES = {
    "cone": ("axial length at least a fifth of the diameter", 0.20),
    "spinner": ("axial length at least a fifth of the diameter", 0.20),
    "dome": ("height at least a tenth of the width", 0.10),
}


def load(path):
    parts = []
    for r in csv.DictReader(open(path)):
        try:
            box = tuple(float(r[k]) for k in AXES)
        except (KeyError, ValueError):
            continue
        parts.append({"name": r["name"], "coll": r.get("collection", ""),
                      "verts": int(r["verts"]), "box": box})
    return parts


def _gap(a, b):
    """Largest separation along any axis. <= 0 means the boxes overlap."""
    return max(max(a[2 * i] - b[2 * i + 1], b[2 * i] - a[2 * i + 1])
               for i in range(3))


def _size(b):
    return (b[1] - b[0], b[3] - b[2], b[5] - b[4])


def _matched(name, exempt):
    return any(name.startswith(k) for k in exempt)


def check_attached(parts, gap_mm, exempt):
    bad = []
    for i, p in enumerate(parts):
        if _matched(p["name"], exempt):
            continue
        best = min((_gap(p["box"], q["box"])
                    for j, q in enumerate(parts) if j != i), default=0.0)
        if best > gap_mm:
            bad.append((p["name"], f"{best:.1f} mm clear of every other part"))
    return bad


MIRROR_SUFFIXES = (("_l", "_r"), ("_fl", "_fr"), ("_rl", "_rr"),
                   ("_port", "_stbd"))


def check_mirrored(parts, tol, exempt):
    by_name = {p["name"]: p for p in parts}
    bad = []
    for p in parts:
        n = p["name"]
        if _matched(n, exempt):
            continue
        for a, b in sorted(MIRROR_SUFFIXES, key=lambda s: -len(s[0])):
            if not n.endswith(a):
                continue
            twin = n[:-len(a)] + b
            if twin not in by_name:
                bad.append((n, f"has no mirror part {twin}"))
                break
            L, R = p["box"], by_name[twin]["box"]
            err = max(abs(L[0] - R[0]), abs(L[1] - R[1]),      # x
                      abs(L[4] - R[4]), abs(L[5] - R[5]),      # z
                      abs(L[2] + R[3]), abs(L[3] + R[2]))      # y, reflected
            if err > tol:
                bad.append((n, f"is {err:.1f} mm from mirroring {twin}"))
            break
    return bad


def check_distinct(parts, tol, exempt):
    bad, seen = [], {}
    for p in parts:
        if _matched(p["name"], exempt):
            continue
        key = tuple(round(v / max(tol, 1e-6)) for v in p["box"])
        if key in seen:
            bad.append((p["name"],
                        f"occupies the same space as {seen[key]}"))
        else:
            seen[key] = p["name"]
    return bad


def check_shaped(parts, exempt):
    bad = []
    for p in parts:
        n = p["name"]
        if _matched(n, exempt):
            continue
        for word, (why, ratio) in SHAPE_RULES.items():
            if word not in n:
                continue
            d = sorted(_size(p["box"]))
            if d[2] < 1e-6:
                continue
            if d[0] / d[2] < ratio:
                bad.append((n, f"is named {word} but is "
                               f"{d[0]:.2f} x {d[2]:.1f} mm -- {why}"))
            break
    return bad


def check_single(parts, singletons):
    """Concepts the machine has exactly one of.

    Bounding boxes cannot catch this: the cockpit had two steering wheels
    30 mm apart, built by two modules, filed under two different collections,
    and every geometric check passed both of them. The only thing that knows
    a car has one steering wheel is a list saying so.
    """
    bad = []
    for concept, (patterns, want) in sorted(singletons.items()):
        found = [p["name"] for p in parts
                 if any(_glob(p["name"], g) for g in patterns)]
        if len(found) != want:
            bad.append((concept,
                        f"expected {want}, found {len(found)}: "
                        + (", ".join(sorted(found)) if found else "none")))
    return bad


def _glob(name, pattern):
    """`pattern` matches the whole name, with * as the only wildcard."""
    import fnmatch
    return fnmatch.fnmatchcase(name, pattern)


def check_clear(parts, pairs):
    """Pairs of parts that must keep a stated distance from each other.

    Most overlaps in an assembly are correct -- a piston is inside its bore --
    so this cannot be a blanket rule. It is a list of the pairs where an
    overlap means something is broken, with the gap each one needs.
    """
    by = {p["name"]: p for p in parts}
    bad = []
    for a, b, want in pairs:
        if a not in by or b not in by:
            bad.append((f"{a} / {b}", "one of the pair does not exist"))
            continue
        g = _gap(by[a]["box"], by[b]["box"])
        if g < want:
            bad.append((f"{a} / {b}",
                        f"gap {g:.1f} mm, needs {want:.1f} mm"))
    return bad


def check_joined(parts, pairs):
    """Pairs of parts that have to reach each other.

    The generic `attached` test only asks whether a part touches anything at
    all, which a driveshaft satisfies by touching the gearbox while stopping
    80 mm short of the wheel it is supposed to drive. This names the joints
    that matter and requires them to close.
    """
    by = {p["name"]: p for p in parts}
    bad = []
    for a, b in pairs:
        if a not in by or b not in by:
            bad.append((f"{a} / {b}", "one of the pair does not exist"))
            continue
        g = _gap(by[a]["box"], by[b]["box"])
        if g > 0.0:
            bad.append((f"{a} / {b}", f"{g:.1f} mm short of meeting"))
    return bad


def audit(path, cfg):
    parts = load(path)
    groups = [
        ("attached", check_attached(parts, cfg.get("gap_mm", 0.5),
                                    cfg.get("exempt_attached", {}))),
        ("mirrored", check_mirrored(parts, cfg.get("mirror_tol_mm", 1.0),
                                    cfg.get("exempt_mirror", {}))),
        ("distinct", check_distinct(parts, cfg.get("distinct_tol_mm", 0.5),
                                    cfg.get("exempt_distinct", {}))),
        ("shaped", check_shaped(parts, cfg.get("exempt_shape", {}))),
        ("single", check_single(parts, cfg.get("singletons", {}))),
        ("clear", check_clear(parts, cfg.get("clearances", ()))),
        ("joined", check_joined(parts, cfg.get("joins", ()))),
    ]
    return parts, groups


def report(path, cfg, title):
    parts, groups = audit(path, cfg)
    print(f"{title}: {len(parts)} parts\n")
    total = 0
    for name, bad in groups:
        total += len(bad)
        mark = "ok  " if not bad else "x   "
        print(f"  {mark}{name:10s} {'clean' if not bad else str(len(bad)) + ' problems'}")
        for n, why in bad[:24]:
            print(f"         {n:30s} {why}")
        if len(bad) > 24:
            print(f"         ... and {len(bad) - 24} more")
    print("\n" + ("PASS  every part is attached, mirrored, distinct, unique "
                  "and the shape it is named"
                  if not total else f"FAIL  {total} structural problems"))
    return total
