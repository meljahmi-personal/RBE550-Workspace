#!/usr/bin/env python3
# scripts/plot_from_logs.py
import sys, re, pathlib, statistics as stats
import matplotlib.pyplot as plt
from collections import defaultdict
import csv, pathlib

METRIC_RE = re.compile(
    r"\[rbe550\]\s*success=(?P<success>\w+),\s*path_len=(?P<path_len>\d+),\s*"
    r"runtime_ms=(?P<runtime_ms>[\d\.]+),\s*nodes_expanded=(?P<nodes_expanded>\d+)"
)
RUN_RE = re.compile(
    r"\[rbe550\]\s*Running.*?algo=(?P<algo>\w+).*?(?:weight=(?P<weight>[\d\.]+))?.*?moves=(?P<moves>\d+)",
    re.IGNORECASE,
)
ALT_ALGO_RE = re.compile(r"\[rbe550\].*?algo=(?P<algo>\w+).*?(?:weight=(?P<weight>[\d\.]+))?", re.IGNORECASE)

def parse_log(path: pathlib.Path):
    algo = None
    weight = None
    moves = None
    metrics = None
    for line in path.read_text(errors="ignore").splitlines():
        if algo is None:
            m = RUN_RE.search(line) or ALT_ALGO_RE.search(line)
            if m:
                algo = m.group("algo")
                weight = m.groupdict().get("weight")
                moves = m.groupdict().get("moves")
        if metrics is None:
            m2 = METRIC_RE.search(line)
            if m2:
                d = m2.groupdict()
                metrics = {
                    "success": d["success"] == "True",
                    "path_len": int(d["path_len"]),
                    "runtime_ms": float(d["runtime_ms"]),
                    "nodes_expanded": int(d["nodes_expanded"]),
                }
    if metrics is None:
        raise ValueError("no metrics line")
    if algo is None:
        algo = "unknown"
    if moves is not None:
        try:
            moves = int(moves)
        except:
            moves = None
    return {
        "file": path.name,
        "algo": algo,
        "weight": float(weight) if weight not in (None, "") else None,
        "moves": moves,
        **metrics,
    }



def bar(ax, labels, values, title, ylabel):
    x = list(range(len(labels)))
    ax.bar(x, values)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xticks(x)  # set tick locations first
    ax.set_xticklabels(labels, rotation=15, ha="right")


def main(argv):
    if not argv:
        print("usage: python3 scripts/plot_from_logs.py outputs/*.log")
        sys.exit(2)

    records = []
    skipped = []
    for p in argv:
        path = pathlib.Path(p)
        if not path.exists():
            continue
        try:
            rec = parse_log(path)
            records.append(rec)
        except Exception:
            skipped.append(path.name)

    if not records:
        print("No parsable logs. Skipped:", skipped)
        sys.exit(1)

    # Summary by algorithm (ignore weight except to label W-A*)
    # Label weighted A* as 'wA* (w=...)' so it appears distinctly
    labels = []
    rt = []
    ne = []
    pl = []
    for r in records:
        label = r["algo"]
        if r["algo"] in ("weighted_astar", "wastar") and r["weight"] is not None:
            label = f"wA* (w={r['weight']})"
        labels.append(label)
        rt.append(r["runtime_ms"])
        ne.append(r["nodes_expanded"])
        pl.append(r["path_len"])

    # If there are many records per label, aggregate (median)
    agg = defaultdict(lambda: {"runtime_ms": [], "nodes_expanded": [], "path_len": []})
    for lab, a, b, c in zip(labels, rt, ne, pl):
        agg[lab]["runtime_ms"].append(a)
        agg[lab]["nodes_expanded"].append(b)
        agg[lab]["path_len"].append(c)

    agg_labels = sorted(agg.keys())
    agg_rt = [stats.median(agg[k]["runtime_ms"]) for k in agg_labels]
    agg_ne = [stats.median(agg[k]["nodes_expanded"]) for k in agg_labels]
    agg_pl = [stats.median(agg[k]["path_len"]) for k in agg_labels]

    outdir = pathlib.Path("outputs/plots")
    outdir.mkdir(parents=True, exist_ok=True)

    # Runtime
    fig1, ax1 = plt.subplots(figsize=(8,4))
    bar(ax1, agg_labels, agg_rt, "Runtime (ms)", "ms")
    fig1.tight_layout()
    fig1.savefig(outdir / "runtime_ms.png", dpi=150)

    # Nodes expanded
    fig2, ax2 = plt.subplots(figsize=(8,4))
    bar(ax2, agg_labels, agg_ne, "Nodes Expanded", "count")
    fig2.tight_layout()
    fig2.savefig(outdir / "nodes_expanded.png", dpi=150)

    # Path length
    fig3, ax3 = plt.subplots(figsize=(8,4))
    bar(ax3, agg_labels, agg_pl, "Path Length", "cells")
    fig3.tight_layout()
    fig3.savefig(outdir / "path_len.png", dpi=150)

    # Print a tiny table to stdout too
    print("\nParsed logs:")
    for lab in agg_labels:
        print(f" - {lab:16s}  median(rt_ms)={stats.median(agg[lab]['runtime_ms']):.3f}  "
              f"median(nodes)={stats.median(agg[lab]['nodes_expanded'])}  "
              f"median(path)={stats.median(agg[lab]['path_len'])}")
    if skipped:
        print("\nSkipped files (no metrics found):", ", ".join(skipped))
        

    csv_path = pathlib.Path("outputs/plots/summary.csv")
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["label", "median_runtime_ms", "median_nodes_expanded", "median_path_len"])
        for lab in agg_labels:
            w.writerow([lab,
                        f"{stats.median(agg[lab]['runtime_ms']):.3f}",
                        int(stats.median(agg[lab]['nodes_expanded'])),
                        int(stats.median(agg[lab]['path_len']))])
    print(f"\nWrote {csv_path}")


if __name__ == "__main__":
    main(sys.argv[1:])

