# 3D Probabilistic Occupancy Grid Mapping

## Overview
This implementation maintains a **3D occupancy grid** from LiDAR point clouds using a 3D Bresenham ray casting algorithm in which each voxel stores the belief that it is occupied, based on sensor observations.  
We use the **log-odds** representation for probabilities, which allows incremental Bayesian updates and avoids numerical instability.  This project integrates the Open3D library for visualization and cxxopts for command-line options.

Typical applications:
- 3D SLAM
- Environment mapping for robotics
- Obstacle detection for path planning

---

### Project Demo

<p align="center">
  <img src="output.gif" alt="Occupancy Mapper Demo" width="800"/>
</p>

--- 

### Project Structure
```
ModernCppProject2025/
├── CMakeLists.txt
├── src/
│   ├── main.cpp
│   ├── dataloader.cpp
│   ├── bresenham.cpp
│   └── visualizer.cpp
├── include/
│   ├── dataloader.hpp
│   ├── bresenham.hpp
│   ├── visualizer.hpp
│   └── occupancy_grid.hpp
├── data/        
│   ├── PLY/*.ply
│   ├── gt_poses.txt       
└── README.md
```
---

### Features
- 3D occupancy mapping via Bresenham voxel traversal.
- Uses Eigen for matrix transformations and vector operations.
- Supports dataset loading through a modular dataloader.
- Interactive visualization of final occupied voxels using Open3D.
- Configurable via command-line arguments for flexible experimentation.
- Time performance metrics: per-scan and total processing time reporting.

---

## Getting Started
### Prerequisites
Ensure the following dependencies are installed:

1. CMake (minimum version 3.31)

2. C++20-capable compiler (e.g., GCC ≥ 10, Clang ≥ 11)

3. Eigen3, Open3D, and cxxopts (via package manager or built from source)



### Build & Run

The project uses CMake for its build system.

```bash
# 1. Clone the repository
git clone https://github.com/AmirhosseinSoltan/ModernCppProject2025.git
cd ModernCppProject2025

# 2. Build the project
mkdir build && cd build
cmake ..
make -j$(nproc)

# 3. Pass the path to your dataset directory via command-line parameters as following:
./occupancy_mapping --data-path <path_to_your_dataset> --voxel-size <requiered_voxel_size>

or 

./occupancy_mapping --d <path_to_your_dataset> --s <requiered_voxel_size>

# Show all options
./occupancy_mapping --help
```

### Usage example
```
./occupancy_mapper \
  --data-path /path/to/your/dataset/ \
  --voxel-size 0.8
```

### Available options
| Option             | Description                            | Default Value                      |
| ------------------ | -------------------------------------- | ---------------------------------- |
| `-d, --data-path`  | Directory containing point cloud scans | `data/` (relative to project root) |
| `-s, --voxel-size` | Size of each voxel in meters           | `0.3  `                              |
| `-h, --help`       | Display help and usage instructions    | —                                  |

---

### How the algorithm works?

- **Data flow**
  - First we load a sequence of point clouds (`.ply` files) for each scan and corresponding 4x4 poses from `gt_poses.txt` file.
  - For each point in the scan, the point is transformed to world coordinates.

- **Voxelization**
  - Convert any world point $p = (x, y, z)$ to a discrete voxel index using the voxel size $v$:
  
    $i = floor(x / v)$, $j = floor(y / v)$, $k = floor(z / v)$
   
  - The geometric center of a voxel $(i, j, k)$ is $$\left( (i+0.5)v,\, (j+0.5)v,\, (k+0.5)v \right)$$.
  - **NOTE** `Voxel size` can significantly affect the processing time and computational cost. The smaller the voxel size, the larger the number of rays needed to be cast and therefore, the more the computation.

  - We obsereved the following results through our experiments on MacBook pro, Processor: 2.6 GHz 6-Core Intel Core i7, Memory: 16 GB 2400 MHz DDR4

    | Average per-scan process time (Sec) | Total process tim(Min)  |   Voxel_size  |                  
    | ------------------ | -------------------------------------- | ---------------------------------- |
    | 0.600498 s  | 69.0438 |  0.3 | 
    | 0.45834 s  | 56.2518 |  0.5 | 
    | 0.328146 s  | 38.1198 |  0.8 | 
    | 0.24417 s  | 28.7800 |  1.0 | 


- **3D Bresenham ray stepping** 
  - Cast a ray from the sensor origin to the point, `insertRay()` functionality.
  - Given start voxel $(x_1, y_1, z_1)$ and end voxel $(x_2, y_2, z_2)$, compute deltas $dx,\, dy,\, dz$ and step directions $x_{\text{inc}},\, y_{\text{inc}},\, z_{\text{inc}} \in \{-1, +1\}$.
  - Choose the driving axis as the largest of $dx,\, dy,\, dz$ and maintain two error terms to decide when to step along the other axes.
  - Visit each intermediate voxel along the grid line from start to end and apply a "free" probabilistic update; apply an "occupied" update at the final voxel. Updates are applied in log-odds and clamped for stability.

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
  - Extract centers of voxels whose probability ≥ `occ_threshold` and render via Open3D (`DrawGeometries`).

---

### Refrences

1. Wurm, Kai & Hornung, A & Bennewitz, Maren & Stachniss, Cyrill & Burgard, Wolfram. (2010). OctoMap: A Probabilistic, Flexible, and Compact 3D Map Representation for Robotic Systems. 2. 
2. Souza, Anderson & Maia, Rosiery & Aroca, Rafael & Gonçalves, Luiz. (2013). Probabilistic robotic grid mapping based on occupancy and elevation information. 2013 16th International Conference on Advanced Robotics, ICAR 2013. 10.1109/ICAR.2013.6766467. 
3. [OctoMap](https://github.com/OctoMap/octomap )—  C++ implementation for 3D log-odds mapping.
4. [Bresenham's Algorithm for 3-D Line Drawing](https://www.geeksforgeeks.org/python/bresenhams-algorithm-for-3-d-line-drawing/)
5. [C++ Bresenham 3d Line Drawing Algorithm](https://gist.github.com/yamamushi/5823518)