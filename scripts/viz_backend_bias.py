#!/usr/bin/env python3
"""Per-backend systematic-bias view: each backend's deviation from median
across all 21 cells, broken down by axis.

For each backend, plot 3 box-and-strip subplots (Δx, Δy, Δz vs median)
with one dot per cell. Colors mark which secondary the cell is from.
Reveals systematic biases like FAST-LIO's x-shift on flash_front/rear.
"""
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SAMPLES = ["ZL11626", "ZL10359", "ZL10966", "ZL10968", "ZL11881", "ZL12332", "ZL12382"]
BACKENDS = [
    ("FAST-LIO", "ghcr_run_v3", ["calib_B2_pl_infow.yaml", "calibrated_extrinsics.yaml"]),
    ("KISS-ICP", "kiss_icp_run", ["calibrated_extrinsics.yaml"]),
    ("LIO-SAM",  "liosam_run",   ["calibrated_extrinsics.yaml"]),
    ("LIO-SAM*", "liosam_run_hybrid", ["calibrated_extrinsics.yaml"]),
    ("GenZ-ICP", "genz_icp_run", ["calibrated_extrinsics.yaml"]),
    ("MAD-ICP",  "mad_icp_run",  ["calibrated_extrinsics.yaml"]),
]
SECONDARIES = [("flash_front", "flash_front_pointcloud", "tab:blue"),
               ("flash_rear",  "flash_rear_pointcloud",  "tab:orange"),
               ("rfr",         "remote_front_right_pointcloud", "tab:green")]
ROOT = Path("output/multi_sample")


def find_calib(sample, backend_dir, fns):
    for fn in fns:
        p = ROOT / sample / backend_dir / fn
        if p.exists():
            return yaml.safe_load(open(p)).get("calibrated_extrinsics", {})
    return None


def main():
    deltas = {bname: {"x": [], "y": [], "z": [], "sec": [], "label": []}
              for bname, _, _ in BACKENDS}

    for sample in SAMPLES:
        if not (ROOT / sample).exists():
            continue
        for sec_short, sec_full, _ in SECONDARIES:
            xyzs = {}
            for bname, bdir, fns in BACKENDS:
                calib = find_calib(sample, bdir, fns)
                if calib and sec_full in calib:
                    xyz = calib[sec_full].get("translation_xyz_m")
                    if xyz and all(v is not None for v in xyz):
                        xyzs[bname] = np.array(xyz)
            if len(xyzs) < 2:
                continue
            stack = np.stack(list(xyzs.values()))
            median = np.median(stack, axis=0)
            for bname, xyz in xyzs.items():
                d = xyz - median
                deltas[bname]["x"].append(float(d[0]))
                deltas[bname]["y"].append(float(d[1]))
                deltas[bname]["z"].append(float(d[2]))
                deltas[bname]["sec"].append(sec_short)
                deltas[bname]["label"].append(f"{sample}/{sec_short}")

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    sec_color = {s: c for s, _, c in SECONDARIES}

    for ax_idx, axis in enumerate(["x", "y", "z"]):
        ax = axes[ax_idx]
        for bi, (bname, _, _) in enumerate(BACKENDS):
            d = deltas[bname][axis]
            secs = deltas[bname]["sec"]
            colors = [sec_color[s] for s in secs]
            xs = np.full(len(d), bi) + np.random.uniform(-0.18, 0.18, len(d))
            ax.scatter(xs, d, c=colors, s=45, alpha=0.75,
                        edgecolors="black", linewidth=0.5, zorder=3)
            # Box (median ± IQR)
            if d:
                q25, med, q75 = np.percentile(d, [25, 50, 75])
                ax.plot([bi - 0.3, bi + 0.3], [med, med], "k-", lw=2, zorder=4)
                ax.add_patch(plt.Rectangle((bi - 0.3, q25), 0.6, q75 - q25,
                                            fill=False, edgecolor="black",
                                            linewidth=1.0, zorder=2))
        ax.axhline(0, color="gray", lw=0.8, ls="--", zorder=1)
        ax.set_ylabel(f"Δ{axis} = {axis} − median_{axis}  (m)")
        ax.grid(True, alpha=0.3)

        # Mark heavy outliers (|Δ| > 0.3 m) with the cell label
        for bi, (bname, _, _) in enumerate(BACKENDS):
            d = deltas[bname][axis]
            labs = deltas[bname]["label"]
            for i, val in enumerate(d):
                if abs(val) > 0.3:
                    ax.annotate(labs[i], (bi, val), fontsize=6,
                                xytext=(7, 0), textcoords="offset points",
                                ha="left", va="center")

    axes[-1].set_xticks(range(len(BACKENDS)))
    axes[-1].set_xticklabels([b for b, _, _ in BACKENDS])
    axes[-1].set_xlabel("Backend")

    # Legend
    handles = [plt.Line2D([], [], marker="o", linestyle="",
                            markerfacecolor=c, markeredgecolor="black",
                            markersize=9, label=s)
                for s, _, c in SECONDARIES]
    axes[0].legend(handles=handles, loc="upper right", fontsize=9,
                    title="secondary")

    fig.suptitle("Per-backend axis-wise deviation from cross-backend median, "
                 "across 21 cells", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    out = Path("output/multi_sample/calib_backend_bias.png")
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"Wrote {out}")

    # Also print numeric summary
    print()
    print(f"{'Backend':<10} | {'Δx mean':>9} {'Δx std':>8} | {'Δy mean':>9} {'Δy std':>8} | {'Δz mean':>9} {'Δz std':>8}")
    print("-" * 75)
    for bname, _, _ in BACKENDS:
        d = deltas[bname]
        print(f"{bname:<10} | "
              f"{np.mean(d['x']):>+8.3f}m {np.std(d['x']):>7.3f}m | "
              f"{np.mean(d['y']):>+8.3f}m {np.std(d['y']):>7.3f}m | "
              f"{np.mean(d['z']):>+8.3f}m {np.std(d['z']):>7.3f}m")


if __name__ == "__main__":
    main()
