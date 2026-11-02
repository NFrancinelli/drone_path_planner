#!/usr/bin/env python3
"""Plot the routes flown in a benchmark, seen from above, one panel per leg.

    plot_routes.py benchmark/ --world src/drone_sim/worlds/factory.sdf -o routes.png [--legs 2]

Reads the trails mission_metrics.py --trail wrote, named
trail_cost<unknown_cost>_run<run>_leg<leg>.csv, and draws every run of a cost in that
cost's colour over the world's boxes (boxes lower than the planner's ceiling, which the
drone can fly over, are drawn lighter). Runs on the host: needs only matplotlib.
"""

import argparse
import csv
import glob
import os
import re
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# First three categorical slots of the validated palette (blue, orange, aqua): they stay
# apart for colour-blind readers with all pairs on screen, as routes overlap.
COLORS = ["#2a78d6", "#eb6834", "#1baf7a"]
INK, MUTED, WALL, LOW = "#0b0b0b", "#52514e", "#b9b8b3", "#dcdbd5"
SURFACE = "#fcfcfb"
CEILING = 4.0  # [m] planner's bounds_max z: boxes lower than this can be flown over


def boxes(sdf):
    """(x0, y0, width, depth, top) of every box collision in the world."""
    out = []
    for model in ET.parse(sdf).getroot().iter("model"):
        origin = _pose(model.find("pose"))
        for col in model.iter("collision"):
            size = col.find("geometry/box/size")
            if size is None:
                continue
            c = [o + d for o, d in zip(origin, _pose(col.find("pose")))]
            sx, sy, sz = (float(v) for v in size.text.split())
            out.append((c[0] - sx / 2, c[1] - sy / 2, sx, sy, c[2] + sz / 2))
    return out


def _pose(element):
    return [0.0] * 3 if element is None else [float(v) for v in element.text.split()[:3]]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("dir", help="directory with the trail CSVs")
    parser.add_argument("--world", required=True, help="world SDF file")
    parser.add_argument("-o", "--output", default="routes.png")
    parser.add_argument("--legs", type=int, nargs="+", help="only these legs (default: all)")
    args = parser.parse_args()

    trails = {}  # (leg, cost) -> [[(x, y)], ...]
    for path in sorted(glob.glob(os.path.join(args.dir, "trail_cost*_run*_leg*.csv"))):
        cost, _, leg = re.search(r"cost([\d.]+)_run(\w+)_leg(\d+)", path).groups()
        with open(path) as f:
            pts = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
        trails.setdefault((int(leg), float(cost)), []).append(pts)
    legs = sorted({leg for leg, _ in trails if not args.legs or leg in args.legs})
    costs = sorted({cost for _, cost in trails})
    if len(costs) > len(COLORS):
        raise SystemExit(f"at most {len(COLORS)} costs, got {costs}")

    plt.rcParams.update({"font.size": 10, "text.color": INK, "axes.labelcolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED})
    fig, axes = plt.subplots(len(legs), 1, figsize=(8, 5.4 * len(legs)), squeeze=False)
    world = boxes(args.world)
    for ax, leg in zip(axes[:, 0], legs):
        for x0, y0, w, d, top in world:
            ax.add_patch(Rectangle((x0, y0), w, d, linewidth=0,
                                   facecolor=WALL if top > CEILING else LOW))
        for cost, color in zip(costs, COLORS):
            for i, pts in enumerate(trails.get((leg, cost), [])):
                xs, ys = zip(*pts)
                ax.plot(xs, ys, color=color, linewidth=2, alpha=0.85,
                        label=f"unknown_cost {cost:g}" if i == 0 and leg == legs[0] else None)
        first = next(iter(trails[(leg, costs[0])]))
        ax.plot(*first[0], "o", color=INK, markersize=8)
        ax.plot(*first[-1], "*", color=INK, markersize=14)
        ax.set_title(f"Leg {leg}", loc="left", fontweight="bold")
        ax.set_aspect("equal")
        ax.set_facecolor(SURFACE)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(LOW)
    fig.legend(loc="upper center", ncol=len(costs), frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(args.output, dpi=150, facecolor=SURFACE)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
