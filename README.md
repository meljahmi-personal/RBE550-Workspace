# RBE550 Workspace

This ROS 2 workspace builds and runs the **rbe550_grid_bench** package.  
It contains implementations of classical search algorithms (**BFS**, **Dijkstra**, **Greedy Best-First**, and **A\***), along with benchmarking utilities for comparing them under identical conditions.

---

## Prerequisites (for local runs)

- **Operating System**: Ubuntu 22.04 or later (Linux recommended)  
- **ROS 2 Distribution**: Humble Hawksbill (or later)  
- **Dependencies**:  
  - Python 3 with `pip`  
  - `colcon` build tools  
  - Standard ROS 2 base installation  

Install the basics on Ubuntu:
```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions
```

---

## Local Build and Run (without Docker)

1. Clone the repository and move into the workspace:
   - using **HTTPS**:
     ```bash
     git clone https://github.com/meljahmi-personal/RBE550-Workspace.git
     cd RBE550-Workspace
     ```
   - using **SSH** (preferred if you have GitHub SSH keys set up):
     ```bash
     git clone git@github.com:meljahmi-personal/RBE550-Workspace.git
     cd RBE550-Workspace
     ```

2. Build the workspace:
   ```bash
   ./scripts/build.sh
   ```

3. Run an example benchmark (A* on random grid):
   ```bash
   ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo astar --moves 8 --no-show
   ```

4. (Optional) Save results to the host:
   ```bash   
    mkdir -p outputs
    ./scripts/run.sh --steps 10 --render-every 2 | tee "outputs/run_$(date +%Y%m%d_%H%M%S).log"
   ```

5. For additional examples:
   - `./scripts/run_random64.sh` – BFS and A* on a seeded 64×64 grid  
   - `./scripts/run_maze32.sh` – BFS and A* on an ASCII maze map  
   - `./scripts/run_check.sh` – quick smoke test with a short run  

---

## Docker (Recommended for Reproducibility)

The project includes a Dockerfile and helper script to guarantee consistent results without requiring a local ROS 2 install.

### One-step build and run
From the workspace root:
```bash
./scripts/run_docker.sh --steps 10 --render-every 2

```

This script will:
- Build the Docker image (`rbe550-bench`)  
- Automatically create `./outputs/` on the host if missing  
- Run the benchmark inside the container  
- Mount `./outputs/` to persist logs and results  

### Examples
- **Random 64×64 grid:**
  ```bash
  ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo astar --moves 8

  ```
- **BFS vs A*** on a maze:
  ```bash
  ./scripts/run_docker.sh --map maps/maze_32.txt --algo bfs   --moves 4
  ./scripts/run_docker.sh --map maps/maze_32.txt --algo astar --moves 8
  ```
**Save container output to host:**
  ```bash
  mkdir -p outputs
  ./scripts/run_docker.sh --steps 10 --render-every 2 | tee "outputs/run_$(date +%Y%m%d_%H%M%S).log"
  
  ```
- **Weighted A*:**
  ```bash
  # Standard A* (weight = 1.0)
    ./scripts/run.sh --algo weighted_astar --moves 4 --weight 1.0

    # Faster but less optimal
    ./scripts/run.sh --algo weighted_astar --moves 4 --weight 1.5

    # 8-connected Weighted A* (wA*)
    ./scripts/run.sh --algo wastar --moves 8 --weight 2.0

  ```

Outputs (images, logs, GIFs) will appear in:
```
./outputs/
```

---

## Repository Layout
```
RBE550-Workspace/
├── Dockerfile             # Docker build file
├── .dockerignore          # excludes build artifacts and temp files
├── README.md              # this guide
├── maps/                  # ASCII maps for benchmarks
├── scripts/               # build/run scripts (local + docker)
│   ├── build.sh           # local build with colcon
│   ├── run.sh             # local run
│   ├── run_random64.sh    # BFS/A* on random grid
│   ├── run_maze32.sh      # BFS/A* on maze map
│   ├── run_check.sh       # quick smoke test
│   ├── entrypoint.sh      # auto-sources ROS inside Docker
│   └── run_docker.sh      # one-step build and run in Docker
└── src/rbe550_grid_bench/ # core ROS 2 package
```

---

## Scripts Overview

The repository includes helper scripts to simplify building, running, and testing both locally and inside Docker.

### Root Directory
- **Dockerfile** — Defines the Docker image (ROS 2 Humble + Python + workspace build).  
- **.dockerignore** — Keeps images clean by excluding build artifacts, venvs, and media files.  
- **README.md** — This guide.

### `scripts/` Directory
- **build.sh** — Builds the workspace locally using `colcon`.  
- **run.sh** — Runs the main benchmark node locally (auto-sources ROS 2).  
- **run_check.sh** — Quick 10-step smoke test for basic validation.  
- **run_random64.sh** — BFS/A* on a random seeded 64×64 grid.  
- **run_maze32.sh** — BFS/A* on an ASCII maze.  
- **entrypoint.sh** — Docker entrypoint that sources ROS 2 and runs inside `/ws`.  
- **run_docker.sh** — One-command Docker build and run, with persistent outputs.  

---

## Documentation and Reports

Project documentation, including the proposal and the October 6 status report, is available in the companion repository:

**RBE550 Project Proposal and Reports (SSH):**
```
git@github.com:meljahmi-personal/RBE550-project-proposal.git
```

Specific document folder:
```
project_status_October_6/status/
```

This companion repository contains:
- The initial project proposal  
- The October 6, 2025 status report (PDF)  
- Supporting diagrams and figures referenced in this workspace  

---
