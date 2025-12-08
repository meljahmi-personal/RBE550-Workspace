# RBE550 Grid Bench – Search Benchmark & RViz Demo

This repository contains my final RBE‑550 project: a **2‑D gridworld search benchmark**
with RViz visualization and optional Docker support.

Core features

- Multiple planners: **BFS, Dijkstra, Greedy Best‑First, A\*, Weighted A\*, Theta\***, **JPS**
- Random and fixed maps (32×32 / 64×64, maze and random obstacle fields)
- Metrics: runtime, nodes expanded, path length, number of turns
- ROS 2 RViz visualization with animated “robot” marker
- Batch benchmarking on host or inside Docker (same code path)

The instructions below are written so a grader can reproduce everything
from a clean clone on Ubuntu 22.04.

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
sudo apt install         ros-humble-desktop         python3-colcon-common-extensions         git         build-essential
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

## 3. Running with Docker (benchmarks and optional RViz)

Docker support is provided so the project can be evaluated without
installing ROS 2 on the host. Everything (ROS 2 Humble, the package,
and Python dependencies) runs inside the container.

The main entrypoint for Docker is:

```bash
./scripts/run_docker.sh
```

which supports three subcommands:

- `build` – build the image,
- `bench` – run the benchmark CLI inside the container,
- `rviz` – (optional) run the RViz visualization from inside Docker
  on systems with working X11 / OpenGL forwarding.

### 3.1 Build the Docker image

From the repository root (`RBE550-Workspace`):

```bash
./scripts/run_docker.sh build
```

This calls `docker build` using the provided `Dockerfile` and
installs:

- ROS 2 Humble (base image),
- RViz,
- `colcon`,
- Python dependencies (NumPy, pandas, Matplotlib),
- this package under `/ws`.

The resulting image is tagged (inside the script) so that the other
subcommands can reuse it.

### 3.2 Run benchmarks inside Docker

Example: run A\* on a 64×64 grid with 8‑connected moves and 5 runs:

```bash
./scripts/run_docker.sh bench --algo astar --grid 64 --moves 8 --runs 5
```

`run_docker.sh bench`:

- starts a container from the built image,
- mounts the host `results/` and `bench_cfg/` directories into the
  container,
- forwards all additional arguments (`--algo`, `--grid`, etc.) to
  the internal benchmark CLI.

All CSV and PNG outputs from Docker appear under the **same
host paths** as the host‑side benchmarks:

- `results/bench/bench_all.csv`
- `results/figs/*.png`

You can therefore compare host vs Docker runs directly if desired.

### 3.3 Optional: RViz from inside Docker

On systems where X11 / OpenGL forwarding from Docker is configured,
you can also start the full visualization from inside the container:

```bash
./scripts/run_docker.sh rviz
```

or pass planner parameters through to the launch file, e.g.:

```bash
./scripts/run_docker.sh rviz algo:=astar grid:=64 moves:=8
```

This is not required for grading; the **primary RViz workflow is the
host‑side two‑terminal setup** in Section 2.1.

---

## 4. Notes and troubleshooting

- If RViz shows an empty scene at startup, wait a moment: the
  `planner_node` publishes the occupancy grid, path, and markers
  shortly after launch and then periodically.
- Whenever you modify Python code under `src/rbe550_grid_bench/`,
  re‑run:

  ```bash
  ./scripts/build.sh
  ```

  before launching RViz or benchmarks again.

- The Word version of the report (`report/*.docx`) is intentionally
  **ignored by git** to avoid GitHub’s 100 MB file limit. The PDF
  used for grading is tracked normally as
  `report/meljahmi_RBE550_ProjectFinalSubmission.pdf`.

If any of these steps fail on a clean Ubuntu 22.04 + ROS 2 Humble
setup, it indicates a bug or missing dependency in my configuration.
In that case the commit history and scripts are the single source of
truth for how I generated the results in the report.
