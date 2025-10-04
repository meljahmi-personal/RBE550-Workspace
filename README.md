# RBE550 Workspace

This ROS 2 workspace builds and runs the **rbe550_grid_bench** package.  
It contains implementations of classical search algorithms (BFS, A*, etc.) and benchmarking utilities for comparing them under identical conditions.

---

## Prerequisites (for local runs)

- **Operating System**: Ubuntu 22.04 or later (Linux recommended).  
- **ROS 2 Distribution**: Humble Hawksbill (or later).  
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
- using HTTPS:

   ```bash
   git clone https://github.com/meljahmi-personal/RBE550-Workspace.git
   cd RBE550-Workspace
   ```
- using SSH (preferred if you have GitHub SSH keys set up):

   ```bash
    git clone git@github.com:meljahmi-personal/RBE550-Workspace.git
    cd RBE550-Workspace
   ```


2. Build the workspace:
   ```bash
   ./scripts/build.sh
   ```

3. Run an example benchmark:
   ```bash
   ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo astar --moves 8 --no-show
   ```

4. For additional examples:
   - `./scripts/run_random64.sh` – BFS and A* on a seeded 64×64 grid.  
   - `./scripts/run_maze32.sh` – BFS and A* on an ASCII maze map.  
   - `./scripts/run_check.sh` – quick smoke test with a short run.  

---

## Docker (Recommended for Reproducibility)

The project includes a Dockerfile and helper script to guarantee consistent results without requiring a local ROS 2 install.

### One-step build and run
From the workspace root:
```bash
./scripts/run_docker.sh --steps 10 --render-every 2 --no-show
```

This script will:
- Build the Docker image (`rbe550-bench`)  
- Run the benchmark inside the container  
- Mount `./outputs/` on the host to persist results  

### Examples
- Random 64×64 grid:
  ```bash
  ./scripts/run_docker.sh --grid 64 --fill 0.20 --seed 42 --algo astar --moves 8 --no-show
  ```

- BFS vs A* on a maze:
  ```bash
  ./scripts/run_docker.sh --map maps/maze_32.txt --algo bfs   --moves 4 --no-show
  ./scripts/run_docker.sh --map maps/maze_32.txt --algo astar --moves 8 --no-show
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
│   ├── run_random64.sh    # convenience: BFS/A* on random grid
│   ├── run_maze32.sh      # convenience: BFS/A* on maze map
│   ├── run_check.sh       # quick smoke test
│   ├── entrypoint.sh      # auto-sources ROS inside Docker
│   └── run_docker.sh      # one-step build and run in Docker
└── src/rbe550_grid_bench/ # core ROS 2 package


```

## Scripts Overview

The repository includes helper scripts to simplify building, running, and testing both locally and inside Docker.

### Root Directory
- **Dockerfile**  
  Defines the Docker image for reproducible builds and runs. It installs ROS 2 Humble, Python dependencies, and builds the workspace inside `/ws`.

- **.dockerignore**  
  Excludes unnecessary files (e.g., `build/`, `install/`, `log/`, venvs, images) from being copied into the Docker image. This keeps builds fast and clean.

- **README.md**  
  This guide.

---

### `scripts/` Directory

- **build.sh**  
  Builds the workspace locally with `colcon`.  
  ```bash
  ./scripts/build.sh
  ```
  
- **run.sh** 
  Runs the main benchmark node locally after building.
   ```bash
  ./scripts/run.sh --grid 64 --fill 0.20 --algo astar --moves 8 --no-show
    ```
- **run_check.sh**
Quick smoke test — runs a short benchmark (--steps 10) to confirm the package executes correctly.

- **run_random64.sh**
Convenience wrapper — runs BFS and A* on a random 64×64 grid with a fixed seed, useful for comparing algorithms under the same conditions.

- **run_maze32.sh**
Convenience wrapper — runs BFS and A* on a 32×32 ASCII maze map.

- **entrypoint.sh**
Entry point for the Docker container. Automatically sources ROS 2 and the built workspace so you can run ros2 run ... directly inside the container.

- **run_docker.sh**
One-step helper: builds the Docker image and then runs the benchmark inside the container. Outputs are persisted to ./outputs/.
   ```bash
    ./scripts/run_docker.sh --steps 10 --render-every 2 --no-show
   ```
  


