"""Generate viewer/parts.json from build/parts.csv and plane/spec.py."""

import csv
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "plane"))
import spec  # noqa: E402

GROUPS = [
    ("01 Fuselage",        "Fuselage",      "#8A9299"),
    ("02 Wing",            "Wing",          "#6E93A8"),
    ("03 Tail",            "Tail",          "#7C86A6"),
    ("04 Intake and Duct", "Intake & duct", "#5E6B74"),
    ("05 Canopy",          "Canopy",        "#79A6B4"),
    ("06 Landing Gear",    "Gear",          "#6F6F72"),
    ("07 Engine",          "Engine",        "#A8763E"),
    ("08 RC Systems",      "RC systems",    "#4F8A5E"),
    ("09 Structure",       "Structure",     "#9A7B4A"),
    ("10 Detail",          "Detail",        "#A1A8AF"),
    ("11 Skin",            "Skin detail",   "#98A0A6"),
]


def main():
    rows = list(csv.DictReader(open(os.path.join(ROOT, "build", "parts.csv"))))
    groups = []
    for key, label, colour in GROUPS:
        mine = [r for r in rows if r["collection"] == key]
        if not mine:
            continue
        groups.append({
            "key": key, "label": label, "color": colour,
            "parts": len(mine),
            "faces": sum(int(r["faces"]) for r in mine),
            "x0": min(float(r["x_min_mm"]) for r in mine),
            "x1": max(float(r["x_max_mm"]) for r in mine),
        })

    parts = {r["name"]: {
        "g": r["collection"], "mat": r["material"],
        "x0": float(r["x_min_mm"]), "x1": float(r["x_max_mm"]),
        "f": int(r["faces"]),
    } for r in rows}

    masses = [{"name": n, "x": x, "m": m} for (n, x, m) in spec.all_masses()]
    masses.sort(key=lambda d: -d["m"])

    out = {
        "name": "RC jet",
        "length": spec.LENGTH, "span": spec.SPAN,
        "envelope": [spec.ENVELOPE_LENGTH, spec.ENVELOPE_SPAN],
        "mass_g": spec.total_mass_g(),
        "wing_loading": spec.wing_loading_g_dm2(),
        "wing_area_cm2": spec.wing_area_mm2() / 100.0,
        "mac": spec.mean_aero_chord(),
        "mac_le": spec.mac_leading_edge_x(),
        "cg_x": spec.cg_x(),
        "cg_frac": spec.cg_frac_mac(),
        "cg_target": spec.TARGET_CG_FRAC,
        "cg_tol": spec.CG_TOLERANCE,
        "engine_scale": spec.ENGINE_SCALE,
        "engine_len": 4630 * spec.ENGINE_SCALE,
        "engine_fan": 1180 * spec.ENGINE_SCALE,
        "gear": {"nose_x": spec.GEAR["nose_x"], "main_x": spec.GEAR["main_x"]},
        "palette": {k: {"rgb": list(v[0]), "metal": v[1], "rough": v[2]}
                    for k, v in spec.PALETTE.items()},
        "groups": groups, "parts": parts, "masses": masses,
    }
    p = os.path.join(ROOT, "viewer", "parts.json")
    json.dump(out, open(p, "w"), indent=1)
    print(f"  -> {p}  ({len(parts)} parts, {len(groups)} groups, "
          f"{len(masses)} masses)")


if __name__ == "__main__":
    main()
