#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')  # CRITICAL: Force non-interactive backend
import matplotlib.pyplot as plt
import numpy as np


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
    memory_nodes = [] 
    turns = []

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            algos.append(row["algo"])
            path_len.append(int(row["path_len"]))
            runtime_ms.append(float(row["runtime_ms"]))
            nodes_expanded.append(int(row["nodes_expanded"]))
            
            # Handle turns
            turns_val = row.get("turns", "0")
            turns.append(int(turns_val) if turns_val else 0)

            # Handle memory nodes (peak_open + peak_closed)
            peak_open = int(row.get("peak_open", 0) or 0)
            peak_closed = int(row.get("peak_closed", 0) or 0)
            memory_nodes.append(peak_open + peak_closed)
            
    return algos, path_len, runtime_ms, nodes_expanded, memory_nodes, turns


def bar_plot(x_labels, values, ylabel, title, out_path: Path):
    # Clean algorithm names for display
    display_names = []
    for name in x_labels:
        if name == "weighted_astar":
            display_names.append("wA*")
        elif name == "theta_star":
            display_names.append("Theta*")
        elif name == "astar":
            display_names.append("A*")
        elif name == "dijkstra":
            display_names.append("Dijkstra")
        elif name == "greedy":
            display_names.append("Greedy")
        elif name == "jps":
            display_names.append("JPS")
        elif name == "bfs":
            display_names.append("BFS")
        else:
            display_names.append(name)
    
    # Create figure
    fig, ax = plt.subplots(figsize=(10, 6))
    x_pos = np.arange(len(display_names))
    
    # Create bars with colors
    colors = plt.cm.Set3(np.arange(len(display_names)) / len(display_names))
    bars = ax.bar(x_pos, values, color=colors, edgecolor='black', linewidth=1)
    
    # Add value labels on top of bars
    for bar, val in zip(bars, values):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, height + 0.01*max(values),
                f'{val}', ha='center', va='bottom', fontsize=10)
    
    # Configure plot
    ax.set_xticks(x_pos)
    ax.set_xticklabels(display_names, rotation=45, ha='right', fontsize=11)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_title(title, fontsize=14, pad=20)
    
    # Add grid
    ax.grid(axis='y', alpha=0.3, linestyle='--')
    
    # Save
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"[plot_bench_all] Saved {out_path}")


def plot_efficiency_scatter(algos, path_len, runtime_ms, nodes_expanded, out_path: Path):
    """Create scatter plot showing efficiency trade-off: speed vs quality"""
    # Clean algorithm names
    display_names = []
    for name in algos:
        if name == "weighted_astar":
            display_names.append("wA*")
        elif name == "theta_star":
            display_names.append("Theta*")
        elif name == "astar":
            display_names.append("A*")
        elif name == "dijkstra":
            display_names.append("Dijkstra")
        elif name == "greedy":
            display_names.append("Greedy")
        elif name == "jps":
            display_names.append("JPS")
        elif name == "bfs":
            display_names.append("BFS")
        else:
            display_names.append(name)
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Create scatter plot with bubble sizes based on nodes expanded
    scatter = ax.scatter(runtime_ms, path_len, 
                         s=[n/5 for n in nodes_expanded],  # Scale for visibility
                         c=range(len(display_names)), 
                         cmap='Set3',
                         edgecolors='black',
                         alpha=0.8,
                         linewidth=1)
    
    # Label each point with algorithm name
    for i, name in enumerate(display_names):
        ax.annotate(name, 
                   (runtime_ms[i], path_len[i]),
                   fontsize=11,
                   ha='center',
                   va='bottom' if name != "Theta*" else 'top',
                   fontweight='bold' if name == "Theta*" else 'normal')
    
    # Configure plot
    ax.set_xlabel('Runtime (milliseconds)', fontsize=12)
    ax.set_ylabel('Path Length (cells)', fontsize=12)
    ax.set_title('Algorithm Efficiency: Speed vs Path Quality\n(Bubble size = Nodes Expanded)', 
                 fontsize=14, fontweight='bold', pad=20)
    
    # Add grid
    ax.grid(alpha=0.3, linestyle='--')
    
    # Add quadrant labels
    x_mid = max(runtime_ms) / 2
    y_mid = max(path_len) / 2
    
    ax.text(x_mid * 0.25, y_mid * 0.25, 'Fast & Short\n(Ideal)', 
            ha='center', va='center', fontsize=10, 
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
    ax.text(x_mid * 1.75, y_mid * 0.25, 'Slow & Short', 
            ha='center', va='center', fontsize=10,
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.7))
    ax.text(x_mid * 0.25, y_mid * 1.75, 'Fast & Long', 
            ha='center', va='center', fontsize=10,
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.7))
    ax.text(x_mid * 1.75, y_mid * 1.75, 'Slow & Long', 
            ha='center', va='center', fontsize=10,
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral", alpha=0.7))
    
    # Save
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"[plot_bench_all] Saved efficiency scatter: {out_path}")


def plot_comprehensive_dashboard(algos, path_len, runtime_ms, nodes_expanded, memory_nodes, turns, out_path: Path):
    """Create a comprehensive dashboard with all metrics in one figure"""
    # Clean algorithm names
    display_names = []
    for name in algos:
        if name == "weighted_astar":
            display_names.append("wA*")
        elif name == "theta_star":
            display_names.append("Theta*")
        elif name == "astar":
            display_names.append("A*")
        elif name == "dijkstra":
            display_names.append("Dijkstra")
        elif name == "greedy":
            display_names.append("Greedy")
        elif name == "jps":
            display_names.append("JPS")
        elif name == "bfs":
            display_names.append("BFS")
        else:
            display_names.append(name)
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('Comprehensive Algorithm Performance Dashboard\n(64×64 grid, 20% obstacles, 8-connected)', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    x_pos = np.arange(len(display_names))
    colors = plt.cm.Set3(np.arange(len(display_names)) / len(display_names))
    
    # 1. Runtime
    bars1 = axes[0, 0].bar(x_pos, runtime_ms, color=colors, edgecolor='black')
    axes[0, 0].set_title('A. Runtime (ms)', fontsize=14, fontweight='bold')
    axes[0, 0].set_ylabel('Milliseconds', fontsize=12)
    axes[0, 0].set_xticks(x_pos)
    axes[0, 0].set_xticklabels(display_names, rotation=45, ha='right', fontsize=10)
    axes[0, 0].grid(axis='y', alpha=0.3)
    
    # 2. Path Length
    bars2 = axes[0, 1].bar(x_pos, path_len, color=colors, edgecolor='black')
    axes[0, 1].set_title('B. Path Length', fontsize=14, fontweight='bold')
    axes[0, 1].set_ylabel('Cells', fontsize=12)
    axes[0, 1].set_xticks(x_pos)
    axes[0, 1].set_xticklabels(display_names, rotation=45, ha='right', fontsize=10)
    axes[0, 1].grid(axis='y', alpha=0.3)
    
    # 3. Nodes Expanded
    bars3 = axes[0, 2].bar(x_pos, nodes_expanded, color=colors, edgecolor='black')
    axes[0, 2].set_title('C. Nodes Expanded', fontsize=14, fontweight='bold')
    axes[0, 2].set_ylabel('Count', fontsize=12)
    axes[0, 2].set_xticks(x_pos)
    axes[0, 2].set_xticklabels(display_names, rotation=45, ha='right', fontsize=10)
    axes[0, 2].grid(axis='y', alpha=0.3)
    
    # 4. Memory Footprint
    bars4 = axes[1, 0].bar(x_pos, memory_nodes, color=colors, edgecolor='black')
    axes[1, 0].set_title('D. Memory Footprint', fontsize=14, fontweight='bold')
    axes[1, 0].set_ylabel('Nodes in Memory', fontsize=12)
    axes[1, 0].set_xticks(x_pos)
    axes[1, 0].set_xticklabels(display_names, rotation=45, ha='right', fontsize=10)
    axes[1, 0].grid(axis='y', alpha=0.3)
    
    # 5. Turns
    bars5 = axes[1, 1].bar(x_pos, turns, color=colors, edgecolor='black')
    axes[1, 1].set_title('E. Direction Changes', fontsize=14, fontweight='bold')
    axes[1, 1].set_ylabel('Number of Turns', fontsize=12)
    axes[1, 1].set_xticks(x_pos)
    axes[1, 1].set_xticklabels(display_names, rotation=45, ha='right', fontsize=10)
    axes[1, 1].grid(axis='y', alpha=0.3)
    
    # 6. Efficiency Scatter (simplified for dashboard)
    scatter = axes[1, 2].scatter(runtime_ms, path_len, 
                                  s=50,  # Fixed size for dashboard
                                  c=range(len(display_names)), 
                                  cmap='Set3', 
                                  edgecolors='black',
                                  alpha=0.8)
    axes[1, 2].set_title('F. Efficiency Overview', fontsize=14, fontweight='bold')
    axes[1, 2].set_xlabel('Runtime (ms)', fontsize=12)
    axes[1, 2].set_ylabel('Path Length (cells)', fontsize=12)
    axes[1, 2].grid(alpha=0.3)
    
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"[plot_bench_all] Saved comprehensive dashboard: {out_path}")


def main() -> None:
    args = build_parser().parse_args()

    csv_path = Path(args.csv)
    outdir = Path(args.outdir)

    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    algos, path_len, runtime_ms, nodes_expanded, memory_nodes, turns = load_bench(csv_path)

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

    # 4) Memory footprint (OPEN + CLOSED)
    bar_plot(
        algos,
        memory_nodes,
        ylabel="Nodes in Memory",
        title="Memory Footprint by Planner (64×64, fill=0.2, moves=8)",
        out_path=outdir / "bench_memory_nodes.png",
    )
    
    # 5) Path quality: number of direction changes (turns)
    bar_plot(
        algos,
        turns,
        ylabel="Direction Changes",
        title="Path Direction Changes by Planner (64×64, fill=0.2, moves=8)",
        out_path=outdir / "bench_path_turns.png",
    )
    
    # 6) NEW: Efficiency scatter plot (separate)
    plot_efficiency_scatter(
        algos, path_len, runtime_ms, nodes_expanded,
        out_path=outdir / "bench_efficiency_scatter.png"
    )
    
    # 7) NEW: Comprehensive Dashboard
    plot_comprehensive_dashboard(
        algos, path_len, runtime_ms, nodes_expanded, memory_nodes, turns,
        out_path=outdir / "bench_comprehensive_dashboard.png"
    )


if __name__ == "__main__":
    main()
