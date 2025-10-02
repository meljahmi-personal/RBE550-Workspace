# RBE550 Workspace

This ROS 2 workspace builds and runs the **rbe550_grid_bench** package.  
It contains implementations of classical search algorithms (BFS, A*, etc.) and benchmarking utilities for comparing them under identical conditions.

---

## Prerequisites

- **Operating System**: Ubuntu 22.04 or later (Linux recommended).  
- **ROS 2 Distribution**: Humble Hawksbill (or later).  
- **Dependencies**:  
  - Python 3 with `pip`  
  - `colcon` build tools  
  - Standard ROS 2 base installation  

You can install the core tools on Ubuntu with:  
```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions
```

---

## Local Build and Run (without Docker)

1. Clone the repository and move into the workspace:
   ```bash
   git clone https://github.com/meljahmi-personal/RBE550-Project.git
   cd RBE550-Project/ros2projrbe550_ws
   ```

2. Build the workspace:
   ```bash
   ./scripts/build.sh
   ```

3. Run an example benchmark:
   ```bash
   ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo astar --moves 8 --no-show
   ```

4. For additional examples, see:
   - `./scripts/run_random64.sh` – runs BFS and A* on a random seeded grid.  
   - `./scripts/run_maze32.sh` – runs BFS and A* on an ASCII maze map.  

---

## Docker (Reproducible Runs)

A Dockerfile is included to ensure fully reproducible results without requiring a manual ROS 2 installation.

### Build the Docker image
From the workspace root:
```bash
docker build -t rbe550-bench:humble .
```

### Run benchmarks inside the container
Random 64×64 grid with fixed seed:
```bash
docker run --rm -it rbe550-bench:humble ./scripts/run_random64.sh
```

Explicit examples:
```bash
docker run --rm -it rbe550-bench:humble ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo bfs   --moves 4 --no-show
docker run --rm -it rbe550-bench:humble ./scripts/run.sh --grid 64 --fill 0.20 --seed 42 --algo astar --moves 8 --no-show
```

Maze example:
```bash
docker run --rm -it rbe550-bench:humble ./scripts/run_maze32.sh
```

### Persist outputs
To save outputs to the host machine:
```bash
mkdir -p out
docker run --rm -it -v "$PWD/out:/ws/out" rbe550-bench:humble ./scripts/run_random64.sh
```

---

## Repository Layout
```
ros2projrbe550_ws/
├── Dockerfile             # Docker build file
├── .dockerignore          # excludes build artifacts
├── README.md              # this guide
├── maps/                  # ASCII maps for benchmarks
├── scripts/               # build + run scripts
└── src/rbe550_grid_bench/ # core ROS 2 package
```

