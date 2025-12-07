# RBE550 Grid Bench – Final Project Workspace

This ROS 2 workspace builds and runs the **`rbe550_grid_bench`** package.

It contains:

- Classical grid‑based search algorithms: **BFS, Dijkstra, Greedy Best‑First, A\***, **Weighted A\***, **Theta\***, and **Jump Point Search (JPS)**.
- A random 2‑D gridworld generator (32×32 and 64×64) with adjustable obstacle density.
- An ASCII‑map loader for maze‑style environments.
- A ROS 2 node (`planner_node`) that publishes the grid and path to **RViz** for visualization.
- Benchmarking utilities to compare algorithms under identical conditions and generate summary plots.

The goal is to provide a **reproducible, self‑contained benchmark** that the grader can run either **locally on Ubuntu** or **inside Docker** with minimal effort.

---

## 1. Repository Layout

```text
ros2projrbe550_ws/
├── src/
│   └── rbe550_grid_bench/         # Main ROS 2 package
├── maps/                          # ASCII maps (e.g., maze_32.txt)
├── scripts/                       # Helper scripts (build, run, docker, RViz)
│   ├── build.sh                   # Local colcon build helper
│   ├── run.sh                     # Core runner used by all other scripts
│   ├── run_all_benchmarks.sh      # 7‑algorithm benchmark + plots
│   ├── run_random64.sh            # All algorithms on random 64×64 grid
│   ├── run_random32.sh            # All algorithms on random 32×32 grid
│   ├── run_maze32.sh              # All algorithms on ASCII map maze_32.txt
│   ├── run_check.sh               # Quick smoke test
│   ├── run_rviz.sh                # Launch planner_node + RViz
│   ├── feed_rviz_demo.sh          # Periodically republish demo path to RViz
│   ├── run_docker.sh              # Build & run the Docker container
│   ├── entrypoint.sh              # Container entrypoint (sources ROS + workspace)
│   ├── start.sh                   # Personal helper (not needed for grading)
│   └── dev/                       # Extra dev‑only scripts (ignore for grading)
├── Dockerfile
├── .dockerignore
└── README.md                      # This file
```

Only the files above are required to reproduce my results.  
Anything under `scripts/dev/` can be safely ignored for grading.

---

## 2. Prerequisites (Local / Non‑Docker)

**Important:** This project does **not** use a Python virtual environment.  
ROS 2 and colcon are installed system‑wide.

You will need:

- **OS**: Ubuntu 22.04 (or compatible Linux)
- **ROS 2 Humble** installed with desktop packages (includes RViz):  
  <https://docs.ros.org/en/humble/Installation.html>
- **colcon** build tools:
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```
- Basic build tools:
  ```bash
  sudo apt install build-essential python3-pip
  ```

Once ROS 2 Humble is installed, make sure your shell can use it:

```bash
source /opt/ros/humble/setup.bash
```

I assume all of the commands below are executed from the workspace root:

```bash
cd ~/ros2projrbe550_ws
```

---

## 3. Local Build (non‑Docker)

From the workspace root:

```bash
cd ~/ros2projrbe550_ws

# 1) Source ROS 2
source /opt/ros/humble/setup.bash

# 2) Build the rbe550_grid_bench package
./scripts/build.sh

# 3) Source the newly built workspace
source install/setup.bash
```

You should only need to rebuild if you modify code. For re‑runs / benchmarks you just need to re‑source `install/setup.bash` in a new terminal.

---

## 4. Core Runner: scripts/run.sh

All higher‑level scripts ultimately call:

```bash
./scripts/run.sh [options]
```

The most important options are:

- `--grid N` – random N×N grid (e.g., 32 or 64).  
- `--fill P` – obstacle fill fraction (default 0.2).  
- `--algo {bfs,dijkstra,greedy,astar,weighted_astar,theta_star,jps}`  
- `--moves {4,8}` – 4‑connected vs 8‑connected grid.  
- `--steps K` – simulation steps (1 for pure planning benchmark).  
- `--seed S` – random seed for repeatability.  
- `--csv results/bench_all.csv` – append results row to CSV.  
- `--weight W` – heuristic inflation for `weighted_astar`.  
- `--map maps/maze_32.txt` – load ASCII map instead of random grid.  
- `--start-goal "r1,c1:r2,c2"` – override default start/goal on ASCII maps.  

You rarely need to call `run.sh` directly – the helper scripts wrap it with the correct arguments.

---

## 5. Quick Local Runs

Assuming:

```bash
cd ~/ros2projrbe550_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 5.1 Random 64×64 Grid (all algorithms)

```bash
./scripts/run_random64.sh
```

This:

- Generates a 64×64 random grid with ~20% obstacles (`fill=0.2`).
- Uses seed 42 so every algorithm sees **exactly the same world**.
- Runs **7 algorithms** with 8‑connected moves:
  - BFS, Dijkstra, Greedy, A*, Weighted A*, Theta*, JPS
- Prints per‑run stats in the terminal  
  (`success`, `path_len`, `runtime_ms`, `nodes_expanded`, …).

### 5.2 Random 32×32 Grid (all algorithms)

```bash
./scripts/run_random32.sh
```

Same idea as above, just a 32×32 grid. Helpful to see how scaling affects runtime and nodes expanded.

### 5.3 ASCII Maze (maze_32.txt)

```bash
./scripts/run_maze32.sh
```

This:

- Loads `maps/maze_32.txt`.
- Uses the fixed start/goal: `(1,1)` → `(7,30)`.
- Runs all 7 algorithms on the **same maze**.
- Lets you observe which algorithms manage to solve this narrow corridor maze and which fail.

---

## 6. Benchmarking + Plot Generation

To reproduce my benchmark CSV and plots:

```bash
cd ~/ros2projrbe550_ws
source /opt/ros/humble/setup.bash
./scripts/run_all_benchmarks.sh --seed 42 --plot
```

What this script does:

1. **Clean build** of `rbe550_grid_bench`:
   - Deletes `build/`, `install/`, `log/` and rebuilds via `colcon`.
2. Sources `install/setup.bash` internally.
3. Runs **all 7 algorithms** on the same 64×64 random grid (seed 42).
4. Appends results to:

   ```text
   results/bench_all.csv
   ```

5. If `--plot` is given, it calls `ros2 run rbe550_grid_bench plot_bench ...` (with a Python fallback) to generate several PNG plots in `results/`:

   - `bench_runtime_ms.png`
   - `bench_nodes_expanded.png`
   - `bench_path_len.png`
   - `bench_memory_nodes.png`
   - `bench_path_turns.png`
   - `bench_efficiency_scatter.png`
   - `bench_comprehensive_dashboard.png`

These images are what I use in the final report to compare search algorithms.

---

## 7. RViz Visualization Workflow

The visualization pipeline uses a ROS 2 node (`planner_node`) to publish:

- The occupancy grid (`nav_msgs/OccupancyGrid`),
- The start and goal poses,
- The planned path (`nav_msgs/Path`),
- Marker arrays showing explored nodes / frontier.

The expected workflow uses **two terminals**: one for RViz + node, one as an optional “feeder” to periodically republish demo data.

### 7.1 Terminal 1 – Launch RViz and planner_node

```bash
cd ~/ros2projrbe550_ws

# Source ROS 2 and the workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch planner_node + RViz
./scripts/run_rviz.sh
```

This will:

- Start the `planner_node` (which creates a 64×64 random grid with seed 42).
- Run A* with 8‑connected moves.
- Publish the grid and the path.
- Open RViz with the right topics available.

In RViz, you can add:

- **Map** display subscribed to `/grid`.
- **Path** display subscribed to `/path`.
- **MarkerArray** displays for explored nodes/frontier if desired.

You can capture **screenshots** directly from this RViz session for the report (e.g., initial grid, planned path, zoomed‑in corridors, etc.).

### 7.2 Terminal 2 – Optional demo feeder

If you want the node to republish demo paths periodically (for animations or multiple screenshots), you can also run:

```bash
cd ~/ros2projrbe550_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/feed_rviz_demo.sh
```

This script uses the same environment and repeatedly calls into the planner logic to update the published grid and path, which RViz visualizes in real time.

Use **Ctrl‑C** in each terminal to stop.

---

## 8. Docker‑Based Workflow (Recommended for Reproducibility)

If the grader prefers a clean, isolated environment, the entire workspace can be built and run inside Docker.  
Everything is driven from **scripts/run_docker.sh**.

From the workspace root on the host:

```bash
cd ~/ros2projrbe550_ws

# Build the image and start a container
./scripts/run_docker.sh
```

The script will:

1. Build a Docker image using the provided `Dockerfile` (Ubuntu + ROS 2 Humble).
2. Mount the current workspace into the container.
3. Use `scripts/entrypoint.sh` inside the container to:
   - Source `/opt/ros/humble/setup.bash`.
   - Build the workspace with colcon.
   - Source `install/setup.bash`.

Once inside the container (you should see a shell prompt **inside** Docker), you can run the same commands as in local mode, but **without** reinstalling ROS:

```bash
# Inside Docker container:

# Benchmark + plots
./scripts/run_all_benchmarks.sh --seed 42 --plot

# Random grids
./scripts/run_random64.sh
./scripts/run_random32.sh

# Maze benchmark
./scripts/run_maze32.sh
```

> **Note:** GUI applications like RViz typically require additional Docker flags (X11 or Wayland forwarding, NVIDIA drivers, etc.).  
> For the purposes of grading, it is sufficient to run **non‑GUI benchmarks in Docker** and run **RViz locally** on a ROS‑capable machine.

---

## 9. Developer‑Only Scripts

The following scripts are **personal helpers** and are **not required** for grading:

- `scripts/start.sh`
- Anything inside `scripts/dev/`

They exist only to speed up my own development iteration.

---

## 10. Companion Document Repository

The detailed project proposal and the October 6, 2025 status report are stored in a separate repository:

```text
git@github.com:meljahmi-personal/RBE550-project-proposal.git
```

Relevant subfolder:

```text
project_status_October_6/status/
```

That repository contains:

- The initial project proposal,
- The October 6 status report (PDF),
- Supporting diagrams referenced in the final report.

---

If anything in this README is unclear during grading, **the scripts themselves are the source of truth**.  
All of the experiments reported in my write‑up are reproducible via:

- `scripts/run_all_benchmarks.sh`
- `scripts/run_random32.sh`
- `scripts/run_random64.sh`
- `scripts/run_maze32.sh`
- `scripts/run_rviz.sh` + `scripts/feed_rviz_demo.sh`
- `scripts/run_docker.sh`
