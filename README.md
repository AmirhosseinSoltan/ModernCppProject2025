# ModernCppProject2025

## Modern C++ Project

This repository contains and a modern C++ projec that builds a 3D occupancy grid map from LiDAR point clouds using a 3D Bresenham ray casting algorithm and visualizes results with Open3D.


### Features

- Loads a sequence of point clouds (`.ply`) and corresponding 4x4 poses.
- Transforms points to world frame and casts rays per point using a 3D Bresenham algorithm.
- Builds a voxel occupancy map and visualizes occupied voxels via Open3D.

### Requirements

- C++ compiler with C++20 support (Apple Clang on macOS works)
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

If you use system Eigen instead of `dependancies/eigen-master`, update `project/CMakeLists.txt` to include your system Eigen path or add `-I` flags accordingly.

### Dataset layout

Place your dataset under `project/data/`:

- `project/data/PLY/` — a directory containing ordered `.ply` point clouds
- `project/data/gt_poses.txt` — poses, one per scan, each line is a 3x4 row-major matrix (the last row is assumed to be `[0 0 0 1]`)

By default, the path is hard-coded in `project/src/main.cpp`:

```cpp
const std::string dataset_dir = "<project_root>/data/";
```

Update this string to point to your dataset location if your repo path differs.

### Build

```bash
# Configure (override Open3D_DIR if needed)
cmake -S <project_root> -B <build_dir> \
  -DOpen3D_DIR=<project_root>/dependancies/open3d-install/lib/cmake/Open3D

# Build
cmake --build <build_dir> -j
```

### Run

```bash
<build_dir>/occupancy_mapper
```

The program prints per-scan timings and opens an Open3D visualization window of the occupied voxels at the end.

### Tuning and notes

- Voxel size is set in `project/src/main.cpp` (`float VoxelSize = 0.8;`). Smaller voxels increase detail and runtime/memory.
- The implementation inserts all free voxels along each ray and marks the last voxel as occupied. Free voxels never overwrite previously marked occupied voxels.
- Parallel STL is not enabled on macOS’s libc++; the code currently uses STL algorithm `std::for_each` for portability.

### How the algorithm works

- **Data flow**
  - Load point cloud for scan i and its pose `T_w_s ∈ SE(3)` from `gt_poses.txt`.
  - For each point `p_s` in the scan: transform to world `p_w = (T_w_s · [p_s;1]).head<3>()` and cast a ray from the sensor origin `o_w = T_w_s.translation()` to `p_w`.

- **Voxelization**
  - Convert any world point `p = (x,y,z)` to a discrete voxel index using the voxel size `v`: `i = ⌊x/v⌋`, `j = ⌊y/v⌋`, `k = ⌊z/v⌋`.
  - The geometric center of a voxel `(i,j,k)` is `( (i+0.5)v, (j+0.5)v, (k+0.5)v )`.

- **3D Bresenham ray stepping** (integer arithmetic, no floating-point drift)
  - Given start voxel `(x1,y1,z1)` and end voxel `(x2,y2,z2)`, compute deltas `dx, dy, dz` and step directions `xs, ys, zs ∈ {−1, +1}`.
  - Choose the driving axis as the largest of `dx, dy, dz` and maintain two error terms (`p1`, `p2`) to decide when to step along the other axes.
  - Visit each intermediate voxel along the grid line from start to end; these are considered free space.
  - Mark the final voxel containing the hit point as occupied.

- **Occupancy update policy**
  - The map is an `unordered_map<VoxelKey,bool>` hashed via large prime multipliers.
  - Intermediate (free) voxels are inserted with `emplace(key,false)`, which does not overwrite existing values — so once a voxel is marked occupied, a later free insertion leaves it occupied.
  - The end voxel is set with assignment `map[key_end] = true` to ensure occupancy.

- **Complexity**
  - Per point: `O(L)` where `L ≈ max(dx,dy,dz)` voxels traversed; the algorithm is cache-friendly and branch-light.
  - Memory: proportional to the number of unique voxels visited across all rays.

- **Why hashing instead of dense grids?**
  - A sparse hash map avoids allocating a large 3D array when the traversed space is small relative to the bounds.
  - The chosen hash (`x*73856093 ⊕ y*19349663 ⊕ z*83492791`) distributes indices well for typical voxel coordinates.

- **Visualization**
  - Extract centers of voxels with `true` occupancy and render with Open3D (`DrawGeometries`).

### Troubleshooting

- Open3D not found: pass the correct `-DOpen3D_DIR=...` to CMake or install Open3D into `project/dependancies/open3d-install` as shown above.
- Eigen headers not found: ensure `project/dependancies/eigen-master` exists or adjust include paths in `project/CMakeLists.txt` to your system Eigen.
- No `.ply` files detected: ensure they are inside `project/data/PLY/` and have the `.ply` extension.


