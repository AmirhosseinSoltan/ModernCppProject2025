# 3D Probabilistic Occupancy Grid Mapping (Log-Odds Formulation)

## Overview
This implementation maintains a **3D occupancy grid** from LiDAR point clouds using a 3D Bresenham ray casting algorithm in which each voxel stores the belief that it is occupied, based on sensor observations.  
We use the **log-odds** representation for probabilities, which allows incremental Bayesian updates and avoids numerical instability.

Typical applications:
- 3D SLAM
- Environment mapping for robotics
- Obstacle detection for path planning

---


### Features

- Loads a sequence of point clouds (`.ply`) and corresponding 4x4 poses.
- Transforms points to world frame and casts rays per point using a 3D Bresenham algorithm.
- Builds a voxel occupancy map with probabilistic updates (log-odds) and visualizes voxels exceeding a chosen occupancy probability threshold via Open3D.

---
<!-- 
### Requirements

- C++ compiler with ≥ C++17 support 
- CMake ≥ 3.31
- Eigen (headers only)
- Open3D C++ library

The project’s `CMakeLists.txt` expects:

- Eigen headers at `project/dependancies/eigen-master`
- Open3D CMake config at `project/dependancies/open3d-install/lib/cmake/Open3D`

You can override the Open3D path at configure time with `-DOpen3D_DIR=...`.

### Get the dependencies

Place dependencies under `project/dependancies/` (default expected by CMake):

1) Eigen (headers only)

```bash
git clone --depth=1 https://gitlab.com/libeigen/eigen.git <project_root>/dependancies/eigen-master
```

2) Open3D (build and install locally)

```bash
# Choose a working dir outside the repo root
cd <work_dir>
git clone --recursive https://github.com/isl-org/Open3D.git
cmake -S Open3D -B o3d_build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_GUI=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_PYTHON_MODULE=OFF \
  -DCMAKE_INSTALL_PREFIX=<project_root>/dependancies/open3d-install
cmake --build o3d_build -j
cmake --install o3d_build
```

If you use system Eigen instead of `dependancies/eigen-master`, update `project/CMakeLists.txt` to include your system Eigen path or add `-I` flags accordingly. -->

### Dataset layout

Place your dataset under `project/data/`:

- `project/data/PLY/` — a directory containing ordered `.ply` point clouds
- `project/data/gt_poses.txt` — poses, one per scan, each line is a 3x4 row-major matrix (the last row is assumed to be `[0 0 0 1]`)


Update this string to point to your dataset location if your repo path differs.

### Quick Start - Build & Run

```bash
# 1. Clone the repository
git clone https://github.com/AmirhosseinSoltan/ModernCppProject2025.git
cd ModernCppProject2025

# 2. Build the project
mkdir build && cd build
cmake ..
make -j$(nproc)

# 3. Pass the path to your dataset directory via command-line parameters as following:
./build/occupancy_mapping --data-path <path_to_your_dataset> --voxel-size <requiered_voxel_size>

or 

./build/occupancy_mapping --d <path_to_your_dataset> --s <requiered_voxel_size>

# Show all options
./build/occupancy_mapping --help
```
---

### Key Concepts and notes

- **voxel size**: set in `project/src/main.cpp` (`float VoxelSize = 0.3;`). Smaller voxels increase detail and runtime/memory.
- **probabilistic updates (log-odds)**: each traversed (intermediate) voxel receives a "free" update and the terminal voxel receives an "occupied" update. Updates are applied in log-odds and clamped for stability.
- **visualization threshold**: set in `project/src/main.cpp` via `float occupancy_probability = 0.9;`. Only voxels with probability ≥ this threshold are rendered.

### How the algorithm works

- **Data flow**
  - Load point cloud for scan i and its pose `T_w_s ∈ SE(3)` from `gt_poses.txt`.
  - For each point `p_s` in the scan: transform to world `p_w = (T_w_s · [p_s;1]).head<3>()` and cast a ray from the sensor origin `o_w = T_w_s.translation()` to `p_w`.

- **Voxelization**
  - Convert any world point $p = (x, y, z)$ to a discrete voxel index using the voxel size $v$:
  
   $i = \left\lfloor \frac{x}{v} \right\rfloor$, $j = \left\lfloor \frac{y}{v} \right\rfloor$, $k = \left\lfloor \frac{z}{v} \right\rfloor$.
  - The geometric center of a voxel $(i, j, k)$ is $\left( (i+0.5)v,\, (j+0.5)v,\, (k+0.5)v \right)$.

- **3D Bresenham ray stepping** (integer arithmetic, no floating-point drift)
  - Given start voxel $(x_1, y_1, z_1)$ and end voxel $(x_2, y_2, z_2)$, compute deltas $dx,\, dy,\, dz$ and step directions $x_{\text{inc}},\, y_{\text{inc}},\, z_{\text{inc}} \in \{-1, +1\}$.
  - Choose the driving axis as the largest of $dx,\, dy,\, dz$ and maintain two error terms to decide when to step along the other axes.
  - Visit each intermediate voxel along the grid line from start to end and apply a "free" probabilistic update; apply an "occupied" update at the final voxel.

- **Occupancy representation and updates (probabilistic)**
  - The map is an `unordered_map<VoxelKey,double>` storing log-odds values, hashed via large prime multipliers.
  - Define $L(p) = \ln\left(\frac{p}{1-p}\right)$ and probability from log-odds as $p = 1 - \frac{1}{1 + \exp(l)}$.
  <!-- - Per update: $l_{\text{voxel}} \leftarrow l_{\text{voxel}} + \Delta$, where $\Delta_{\text{free}} = \operatorname{logit}(P_{\text{FREE}}) - \operatorname{logit}(0.5)$ for intermediate voxels and $\Delta_{\text{occ}} = \operatorname{logit}(P_{\text{OCCUPIED}}) - \operatorname{logit}(0.5)$ for terminal voxels. 
   -->
  -**Bayesian Update Rule:**
  Given:
  Let:
  - $p(z_t)$: Probability of occupancy from the **sensor model** for the current measurement
  - $p_{\text{prior}}$: Prior probability before any measurements (often $0.5$, representing "unknown")

  The **log-odds update** formula is:
  $$L_t = L_{t-1} + L_{t} - L_{Prior}$$

  Where:
  - $L_t$: Updated log-odds after observation
  - $L_{t-1}$: Previous log-odds

    We **subtract** $L(p_{\text{prior}})$ to ensure the update is **relative to the prior belief**, not an absolute overwrite.
  - After each update, the resulting probability is clamped to $[P_{\text{MIN}},\, P_{\text{MAX}}]$ for numerical stability.
<!-- 
- **Why hashing instead of dense grids?**
  - A sparse hash map avoids allocating a large 3D array when the traversed space is small relative to the bounds.
  - The chosen hash $(x \times 73856093) \oplus (y \times 19349663) \oplus (z \times 83492791)$ distributes indices well for typical voxel coordinates. -->

- **Visualization**
  - Extract centers of voxels whose probability ≥ `occ_threshold` and render with Open3D (`DrawGeometries`). The threshold is set in `project/src/main.cpp`.

---

### Refrences

1. Wurm, Kai & Hornung, A & Bennewitz, Maren & Stachniss, Cyrill & Burgard, Wolfram. (2010). OctoMap: A Probabilistic, Flexible, and Compact 3D Map Representation for Robotic Systems. 2. 
2. Souza, Anderson & Maia, Rosiery & Aroca, Rafael & Gonçalves, Luiz. (2013). Probabilistic robotic grid mapping based on occupancy and elevation information. 2013 16th International Conference on Advanced Robotics, ICAR 2013. 10.1109/ICAR.2013.6766467. 
3. [OctoMap](https://github.com/OctoMap/octomap )—  C++ implementation for 3D log-odds mapping.
4. [Bresenham's Algorithm for 3-D Line Drawing](https://www.geeksforgeeks.org/python/bresenhams-algorithm-for-3-d-line-drawing/)
5. [C++ Bresenham 3d Line Drawing Algorithm](https://gist.github.com/yamamushi/5823518)