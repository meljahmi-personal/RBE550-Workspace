# RBE550 Grid Bench – Search Benchmark & RViz Demo

This repository contains my final RBE‑550 project: a **2‑D gridworld search benchmark**
with RViz visualization and optional Docker support.

Core features

- Multiple planners: **BFS, Dijkstra, Greedy Best‑First, A\*, Weighted A\*, Theta\***, **JPS**
- Random and fixed maps (32×32 / 64×64, maze and random obstacle fields)
- Metrics: runtime, nodes expanded, path length, number of turns
- ROS 2 RViz visualization with animated “robot” marker
- Batch benchmarking on host or inside Docker (same code path)

The instructions below are written for reproducibility from a clean clone on Ubuntu 22.04.

---

## 1. Host setup: clone, dependencies, and build

These steps assume **Ubuntu 22.04** with a graphical desktop.

### 1.1 Clone the repository

From any directory you like:

```bash
git clone https://github.com/meljahmi-personal/RBE550-Workspace.git
cd RBE550-Workspace
```

This is a standard ROS 2 workspace (with `src/`, `scripts/`, `Dockerfile`, etc.).
No Python virtual environment is used or required.

### 1.2 Install system dependencies (ROS 2 + RViz + tools)

Install ROS 2 Humble and basic tools (if not already installed):

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions git build-essential
```

The `ros-humble-desktop` meta‑package includes **RViz**.

### 1.3 Install Python dependencies (for benchmarking and plotting)

The benchmarking and plotting scripts use NumPy, pandas, and Matplotlib.

```bash
sudo apt install python3-pip
pip3 install --user numpy pandas matplotlib
```

Again: **no virtual environment** is created by this project; the system
Python from Ubuntu 22.04 is used.

### 1.4 Build the workspace

From the repository root (`RBE550-Workspace`):

```bash
./scripts/build.sh
```

This script:

- cleans the workspace (via `colcon`),
- sources `/opt/ros/humble/setup.bash`,
- runs `colcon build --symlink-install`.

After a successful build you can either manually source:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

or simply run:

```bash
./scripts/activate.sh
```

which performs both steps and prints the available ROS 2 commands.

---

## 2. Running from the host (RViz demo + benchmarks)

This section assumes you are **not** using Docker and have completed
Section 1 on the host.

### 2.1 RViz demo with animated robot (two‑terminal workflow)

The main visualization workflow uses **two terminals**.

#### Terminal 1 – build, activate, and launch RViz

From `RBE550-Workspace`:

```bash
./scripts/build.sh
./scripts/activate.sh
./scripts/run_rviz.sh
```

- `run_rviz.sh` sources ROS 2 and the workspace and launches
  `bench.launch.py`, which starts:
  - the `planner_node` (implemented in `src/rbe550_grid_bench/rbe550_grid_bench/planner_node.py`)
  - RViz with the preconfigured layout `rbe550.rviz`

In RViz you should see:

- a **64×64 occupancy grid** centered at the origin,
- obstacles as black cells,
- a shortest path drawn as a polyline,
- start (green sphere), goal (red sphere),
- a **large blue “robot” sphere** that animates along the path,
- a text overlay under the grid with the current algorithm, grid size,
  path length, and number of nodes expanded.

Keep this terminal running while you explore RViz.

#### Terminal 2 – feed different algorithms into RViz

Open a second terminal in `RBE550-Workspace` and run:

```bash
./scripts/activate.sh
# (the first time on a new machine you may also run ./scripts/build.sh here)
./scripts/feed_rviz_demo.sh
```

`feed_rviz_demo.sh`:

- waits for the `planner_node` started by `run_rviz.sh`,
- repeatedly calls the `randomize_grid` service,
- cycles through algorithms and settings,
- sleeps between calls so you can see RViz update.

This is the **exact setup** I used to collect the RViz screenshots
and to record a short video / GIF showing multiple planners and
maps over time.

**Important:** `./scripts/feed_rviz_demo.sh` is only meant to be run
**after** `./scripts/run_rviz.sh` is active in Terminal 1.

### 2.2 Alternate RViz launchers for different maps

In addition to `run_rviz.sh` (64×64 random map), there are three
convenience scripts:

```bash
# 64×64 random map (same as run_rviz.sh)
./scripts/run_random64.sh

# 32×32 random obstacle field
./scripts/run_random32.sh

# 32×32 fixed maze (ASCII map)
./scripts/run_maze32.sh
```

All of these launch the same `planner_node` but with different
parameters (grid size, obstacle density, or `map_path`).

### 2.3 Host‑side benchmarks (CSV + plots)

All benchmarking is driven by one helper script that wraps
`rbe550_grid_bench.bench_runner`.

From `RBE550-Workspace`:

```bash
./scripts/activate.sh
./scripts/run_all_benchmarks.sh --seed 42 --plot
```

This command:

- sweeps algorithms (BFS, Dijkstra, Greedy, A\*, Weighted A\*, Theta\*, JPS),
- sweeps grid sizes (e.g., 32×32 and 64×64) and obstacle densities,
- logs metrics (runtime, nodes expanded, path length, number of turns),
- writes a consolidated CSV to:  
  `results/bench/bench_all.csv`
- generates PNG plots under:  
  `results/figs/`

👉 **This exact command was used to generate all benchmark plots
that appear in the project report:**

```bash
./scripts/run_all_benchmarks.sh --seed 42 --plot
```

On a typical machine this completes quickly and produces a set of
figures suitable for direct inclusion in the write‑up.

---


#3. Docker Workflows

Docker support allows running the benchmarks without installing ROS 2 on the host.
The same code is used inside and outside Docker.

The main wrapper is:
```bash
./scripts/run_docker.sh
```

It supports three modes:

build – build the image

bench – run a single benchmark configuration

bench_all – run the full multi-algorithm benchmark + plots

rviz – (optional) try RViz from inside Docker (depends on X11/OpenGL setup)

#3.1 Build the Docker image

From RBE550-Workspace:

```bash
./scripts/run_docker.sh build
```

This:

- uses the provided Dockerfile

- installs ROS 2 Humble, RViz2, colcon

- installs Python dependencies (NumPy, pandas, Matplotlib)

- builds and installs rbe550_grid_bench under /ws in the image

- tags the image as rbe550-bench:latest (via the script)

- You only need to rebuild if you change the source code.


# 3.2 Single benchmark run inside Docker

Example: run A* once on a 64×64 grid with 8-connected moves, no GUI:

```bash
./scripts/run_docker.sh bench --algo astar --grid 64 --moves 8 --steps 1 --no-show
```

This:

- starts a container

- runs the internal CLI (bench) with your arguments

- prints metrics in the host terminal

# 3.3 Reproducing all plots inside Docker

To reproduce the full benchmark CSV and all plots entirely inside Docker,
using the same settings as the host-side run:

```bash
./scripts/run_docker.sh build           
./scripts/run_docker.sh bench_all
```

and writes the same files to results/ on the host:

- results/bench_all.csv

- results/bench_runtime_ms.png

- results/bench_nodes_expanded.png

- results/bench_path_len.png

- results/bench_memory_nodes.png

- results/bench_path_turns.png

- results/bench_efficiency_scatter.png

- results/bench_comprehensive_dashboard.png

# 3.4 Optional: RViz from within Docker

On some systems you can also run RViz inside Docker, for example:

```bash
xhost +local:root   # allow local docker GUI (X11) access
./scripts/run_docker.sh build
./scripts/run_docker.sh rviz algo:=astar grid:=64 moves:=8
```

This will:

run the same planner_node as the host RViz workflow

launch RViz in the container, displaying the grid and path

Important compatibility note:
RViz inside Docker depends on the host’s X11, GPU drivers, and OpenGL
passthrough. On my machine (Ubuntu 22.04 with working X11/OpenGL),
this configuration runs successfully. On other hardware/driver
combinations, RViz may fail to display or may log OpenGL-related warnings.

The host-side RViz workflow in Section 2.2 is the primary and most
reliable visualization path. Docker is primarily intended for running
benchmarks and reproducing the CSV/plots.

---

# 4. Repository Layout (High Level)

RBE550-Workspace/
├── src/
│   └── rbe550_grid_bench/             # ROS 2 package (planning + benchmarking)
│       ├── launch/
│       │   └── bench.launch.py        # RViz launch entrypoint
│       ├── config/
│       │   └── rbe550.rviz            # RViz display configuration
│       ├── rbe550_grid_bench/
│       │   ├── algorithms/            # BFS, Dijkstra, Greedy, A*, Weighted A*,
│       │   │                          # Theta*, JPS implementations
│       │   ├── bench_runner.py        # Central benchmarking dispatcher
│       │   ├── cli.py                 # `ros2 run ... bench` CLI
│       │   ├── grid_utils.py          # Random grids, ASCII maps, sampling
│       │   ├── neighbors.py           # 4-connected and 8-connected neighbor sets
│       │   ├── metrics.py             # Runtime, path length, memory, CSV writer
│       │   ├── planner_node.py        # RViz visualization node
│       │   ├── plot_bench_all.py      # Plotting backend for benchmarks
│       │   ├── rviz_helpers.py        # Helper functions for RViz markers
│       │   └── maps/                  # Optional ASCII test maps (unused in release)
│       ├── package.xml
│       ├── setup.py
│       └── test/                      # linters + style checks
│
├── scripts/                           # Executable workflows
│   ├── build.sh                       # Clean + colcon build
│   ├── activate.sh                    # Source install/setup.bash automatically
│   ├── run_rviz.sh                    # Host-side RViz launch wrapper
│   ├── feed_rviz_demo.sh              # Timed simulation publisher
│   ├── run_random32.sh                # Convenience: 32×32 random grid
│   ├── run_random64.sh                # Convenience: 64×64 random grid
│   ├── run_maze32.sh                  # Run on maze_32 ASCII map
│   ├── run_all_benchmarks.sh          # Full 7-algorithm benchmark suite
│   ├── run_docker.sh                  # Docker build/bench/rviz frontend
│   ├── run_check.sh                   # Quick development smoke test
│   ├── entrypoint.sh                  # Docker container entrypoint
│   ├── start.sh                       # Developer helper
│   └── dev/                           # Internal development/testing helpers
│       ├── test_no_rviz.sh
│       ├── test_with_rviz.sh
│       ├── demo_flicker.sh
│       └── run_demo.sh
│
├── maps/
│   ├── maze_32.txt                    # 32×32 ASCII maze (handcrafted)
│   └── maze_32_32.txt                 # Duplicate of the above; kept for testing
│                                      # (both contain ASCII obstacle maps)
│
├── results/                           # Auto-generated: CSVs + PNG plots
│   ├── bench_all.csv
│   ├── bench_runtime_ms.png
│   ├── bench_nodes_expanded.png
│   ├── bench_path_len.png
│   ├── bench_memory_nodes.png
│   ├── bench_path_turns.png
│   └── bench_comprehensive_dashboard.png
│
├── outputs/                           # Raw text logs from CLI runs
│   └── run_*.log
│
├── architecture_diagrams/             # PlantUML source + exported diagrams
│   ├── architecture.puml
│   ├── class_core.puml
│   ├── class_algorithms.puml
│   ├── sequence_bench_all.puml
│   └── *.svg / *.png
│
├── Dockerfile                         # Self-contained ROS 2 image
├── README.md
└── video/
    └── multiple_algorithms.mkv        # Demonstration recording


---

# 5. Notes on Portability

This project was tested thoroughly on:

Ubuntu 22.04

ROS 2 Humble

Native host execution (CLI + RViz)

Docker execution (benchmarks + plots, and RViz with working X11/OpenGL)

While the core algorithms and scripts are deterministic, actual behavior of:

Docker-side RViz, and

any GPU-accelerated rendering

can vary across distributions, GPU drivers, and Docker/X11/OpenGL setups.
If a particular combination fails, the recommended approach is to:

run all benchmarks using run_all_benchmarks.sh on the host or via bench_all in Docker

use the host-side RViz workflow for visualization.



