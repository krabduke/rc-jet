"""Illustrative flight-control routing; not a clearance or redundancy validation."""


import math
from collections.abc import Mapping

import mesh
import spec
from parts import intake


def _default_build():
    fcs = spec.FCS
    fcc = fcs["fcc"]
    route_names = ("lower_l", "lower_r", "upper_l", "upper_r")
    if fcc["channels"] != len(route_names) or len(fcs["ports"]) != len(route_names):
        raise ValueError("The FCC requires four channel ports and four routes")
    out = {
        "fcs_fcc_envelope": mesh.box(
            fcc["x"], fcc["y"], fcc["z"],
            fcc["length"], fcc["width"], fcc["height"],
        )
    }
    for name, port in zip(route_names, fcs["ports"]):
        points = [port, *fcs["routes"][name]]
        out[f"fcs_loom_trunk_{name}"] = mesh.pipe(
            points, fcs["loom_r"], 32, subdiv=100
        )
    for name, points in fcs.get("tail_runs", {}).items():
        out[f"fcs_loom_{name}"] = mesh.pipe(
            points, fcs.get("tail_r", fcs["loom_r"]), 20, subdiv=60
        )
    for name, (verts, _) in out.items():
        if any(intake.in_duct(point) for point in verts):
            raise ValueError(f"Specified geometry for {name} enters the intake duct")
    return out


def build(channels=None):
    if channels is None:
        return _default_build()
    if not isinstance(channels, Mapping):
        raise TypeError("channels must map channel names to routes")
    if not channels:
        raise ValueError("At least one flight-control channel route is required")
    out = {}
    for name, route in channels.items():
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Channel names must be nonempty strings")
        points = [tuple(point) for point in route]
        if len(points) < 2 or any(
            len(point) != 3 or not all(math.isfinite(x) for x in point)
            for point in points
        ):
            raise ValueError(f"Channel {name!r} needs at least two finite 3D points")
        if any(math.dist(a, b) <= 0.4 for a, b in zip(points, points[1:])):
            raise ValueError(f"Channel {name!r} segments must exceed 0.4 drawing units")
        out[f"fcs_illustrative_channel_{name}"] = mesh.pipe(
            points, 0.15, 12, subdiv=1
        )
        for end, port, adjacent in (
            ("computer", points[0], points[1]),
            ("actuator", points[-1], points[-2]),
        ):
            distance = math.dist(port, adjacent)
            tip = tuple(a + (b - a) * 0.4 / distance for a, b in zip(port, adjacent))
            out[f"fcs_illustrative_connector_{name}_{end}"] = mesh.pipe(
                [port, tip], 0.3, 12, subdiv=1
            )
    return out
