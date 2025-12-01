#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Plot benchmark results from bench_all.csv"
    )
    p.add_argument(
        "--csv",
        type=str,
        default="results/bench_all.csv",
        help="Path to CSV file (default: results/bench_all.csv)",
    )
    p.add_argument(
        "--outdir",
        type=str,
        default="results",
        help="Output directory for PNGs (default: results/)",
    )
    return p


def load_bench(csv_path: Path):
    algos = []
    path_len = []
    runtime_ms = []
    nodes_expanded = []

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            algos.append(row["algo"])
            path_len.append(int(row["path_len"]))
            runtime_ms.append(float(row["runtime_ms"]))
            nodes_expanded.append(int(row["nodes_expanded"]))

    return algos, path_len, runtime_ms, nodes_expanded


def bar_plot(x_labels, values, ylabel, title, out_path: Path):
    # One simple bar chart
    plt.figure()
    x_pos = range(len(x_labels))
    plt.bar(x_pos, values)
    plt.xticks(x_pos, x_labels, rotation=45, ha="right")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path)
    plt.close()
    print(f"[plot_bench_all] Saved {out_path}")


def main() -> None:
    args = build_parser().parse_args()

    csv_path = Path(args.csv)
    outdir = Path(args.outdir)

    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    algos, path_len, runtime_ms, nodes_expanded = load_bench(csv_path)

    # 1) Runtime
    bar_plot(
        algos,
        runtime_ms,
        ylabel="Runtime (ms)",
        title="Runtime by Planner (64×64, fill=0.2, moves=8)",
        out_path=outdir / "bench_runtime_ms.png",
    )

    # 2) Nodes expanded
    bar_plot(
        algos,
        nodes_expanded,
        ylabel="Nodes Expanded",
        title="Nodes Expanded by Planner (64×64, fill=0.2, moves=8)",
        out_path=outdir / "bench_nodes_expanded.png",
    )

    # 3) Path length
    bar_plot(
        algos,
        path_len,
        ylabel="Path Length (cells)",
        title="Path Length by Planner (64×64, fill=0.2, moves=8)",
        out_path=outdir / "bench_path_len.png",
    )


if __name__ == "__main__":
    main()

