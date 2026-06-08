#!/usr/bin/env python3
"""Visualize cross-backend calibration agreement per (sample, secondary) cell.

For each cell: 6 backend estimates plotted around the cross-backend median.
- X axis: x − median_x (m)
- Y axis: y − median_y (m)
- Marker color: backend
- Marker size: |z − median_z| (visualized as second-order disagreement)
- Background ring: 10 cm / 30 cm trustworthiness thresholds (§21.7)

Layout: 7 rows (samples) × 3 cols (secondaries). Output one figure.
"""
from pathlib import Path
import argparse
import yaml
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

SAMPLES = ["ZL11626", "ZL10359", "ZL10966", "ZL10968", "ZL11881", "ZL12332", "ZL12382"]
BACKENDS = [
    ("FAST-LIO", "ghcr_run_v3", ["calib_B2_pl_infow.yaml", "calibrated_extrinsics.yaml"], "tab:blue"),
    ("KISS-ICP", "kiss_icp_run", ["calibrated_extrinsics.yaml"], "tab:green"),
    ("LIO-SAM",  "liosam_run",   ["calibrated_extrinsics.yaml"], "tab:red"),
    ("LIO-SAM*", "liosam_run_hybrid", ["calibrated_extrinsics.yaml"], "tab:purple"),
    ("GenZ-ICP", "genz_icp_run", ["calibrated_extrinsics.yaml"], "tab:orange"),
    ("MAD-ICP",  "mad_icp_run",  ["calibrated_extrinsics.yaml"], "tab:brown"),
    ("LIVO2",    "fastlivo2_run", ["calibrated_extrinsics.yaml"], "tab:olive"),
]
SECONDARIES = [("flash_front", "flash_front_pointcloud"),
               ("flash_rear",  "flash_rear_pointcloud"),
               ("rfr",         "remote_front_right_pointcloud")]
ROOT = Path("output/multi_sample")


def find_calib(sample, backend_dir, fns):
    for fn in fns:
        p = ROOT / sample / backend_dir / fn
        if p.exists():
            return yaml.safe_load(open(p)).get("calibrated_extrinsics", {})
    return None


def trust_color(spread_norm):
    """std_norm < 0.10 m: green, 0.10–0.30: orange, > 0.30: red."""
    if spread_norm < 0.10:
        return "#9ee493"  # green
    if spread_norm < 0.30:
        return "#fec877"  # orange
    return "#f29c9c"  # red


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path,
                    default=Path("output/multi_sample/calib_scatter_grid.png"))
    ap.add_argument("--zoom", action="store_true",
                    help="Auto-zoom each subplot to ±2σ instead of fixed range")
    args = ap.parse_args()

    fig, axes = plt.subplots(len(SAMPLES), len(SECONDARIES),
                              figsize=(15, 26))

    handles_all = {}
    for r, sample in enumerate(SAMPLES):
        if not (ROOT / sample).exists():
            for c in range(len(SECONDARIES)):
                axes[r][c].axis("off")
            continue
        for c, (sec_short, sec_full) in enumerate(SECONDARIES):
            ax = axes[r][c]

            # Gather backend xyzs
            bs, xyzs = [], []
            for bname, bdir, fns, color in BACKENDS:
                calib = find_calib(sample, bdir, fns)
                if not calib or sec_full not in calib:
                    continue
                rec = calib[sec_full]
                xyz = rec.get("translation_xyz_m")
                if xyz and all(x is not None for x in xyz):
                    bs.append((bname, color, np.array(xyz)))
                    xyzs.append(xyz)

            if len(xyzs) < 2:
                ax.set_axis_off()
                continue

            xyzs = np.array(xyzs)
            median = np.median(xyzs, axis=0)
            std = xyzs.std(axis=0)
            std_norm = float(np.linalg.norm(std))

            # Background patch by trust level
            ax.set_facecolor(trust_color(std_norm))

            # Trust threshold rings (10 cm and 30 cm)
            for r_thresh, ls in [(0.10, "--"), (0.30, ":")]:
                ax.add_patch(Circle((0, 0), r_thresh, fill=False,
                                     edgecolor="black", lw=0.7,
                                     linestyle=ls, alpha=0.6))

            # Plot each backend
            for bname, color, xyz in bs:
                dx = xyz[0] - median[0]
                dy = xyz[1] - median[1]
                dz = xyz[2] - median[2]
                size = 80 + 1500 * abs(dz)
                h = ax.scatter(dx, dy, s=size, c=color, alpha=0.75,
                               edgecolors="black", linewidth=0.8, zorder=5)
                handles_all[bname] = (h, color)
                # Label backend at marker
                ax.annotate(bname, (dx, dy), fontsize=6,
                            xytext=(3, 3), textcoords="offset points",
                            zorder=6)

            ax.axhline(0, color="white", lw=0.5, alpha=0.7)
            ax.axvline(0, color="white", lw=0.5, alpha=0.7)
            ax.scatter(0, 0, marker="+", color="black", s=80, zorder=4)

            if args.zoom:
                # Auto-zoom to encompass all backends with margin
                margin = 0.05
                xrng = max(0.12, abs(xyzs[:, 0] - median[0]).max() * 1.4)
                yrng = max(0.12, abs(xyzs[:, 1] - median[1]).max() * 1.4)
                ax.set_xlim(-xrng, xrng)
                ax.set_ylim(-yrng, yrng)
            else:
                lim = max(0.4, std_norm * 2.5)
                ax.set_xlim(-lim, lim)
                ax.set_ylim(-lim, lim)
            ax.set_aspect("equal")
            ax.tick_params(labelsize=7)

            title = f"{sample} / {sec_short}\nstd={std_norm:.3f}m"
            ax.set_title(title, fontsize=8)

            if r == len(SAMPLES) - 1:
                ax.set_xlabel("x − median_x (m)", fontsize=7)
            if c == 0:
                ax.set_ylabel("y − median_y (m)", fontsize=7)

    # Legend (color = backend, size = |Δz|)
    handles, labels = [], []
    for bname, _, _, color in BACKENDS:
        if bname in handles_all:
            from matplotlib.lines import Line2D
            handles.append(Line2D([], [], marker="o", linestyle="",
                                   markerfacecolor=color, markeredgecolor="black",
                                   markersize=8, alpha=0.75))
            labels.append(bname)
    # Trust band legend
    handles.append(plt.Rectangle((0, 0), 1, 1, fc="#9ee493", ec="black"))
    labels.append("std < 10 cm (high trust)")
    handles.append(plt.Rectangle((0, 0), 1, 1, fc="#fec877", ec="black"))
    labels.append("10–30 cm (medium)")
    handles.append(plt.Rectangle((0, 0), 1, 1, fc="#f29c9c", ec="black"))
    labels.append("std > 30 cm (reject)")

    fig.legend(handles, labels, loc="upper center", ncol=4,
               fontsize=9, bbox_to_anchor=(0.5, 0.995))
    fig.suptitle("Cross-backend calibration agreement (xy plane around median; "
                 "marker size ∝ |Δz|; ring = 10 cm / 30 cm trust thresholds)",
                 fontsize=11, y=0.987)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=140)
    plt.close(fig)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
