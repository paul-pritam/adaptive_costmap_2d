# Adaptive Costmap 2D (adaptive_costmap_2d)

An open-source, high-performance, block-sparse 2D costmap layer plugin for **ROS 2 Nav2** (tested on ROS 2 Jazzy Jalisco).

[![Build & Test](https://img.shields.io/badge/Build-Passing-brightgreen)]()
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)]()
[![Standard](https://img.shields.io/badge/C%2B%2B-20-orange.svg)]()
[![Nav2](https://img.shields.io/badge/Nav2-Plugin-purple.svg)]()

---

## 1. The Core Problem in Large-Scale Navigation

Standard Nav2 (`nav2_costmap_2d`) maintains costmaps as dense, flat one-dimensional byte buffers (`std::vector<unsigned char>`):

$$\text{Memory} = \text{Width} \times \text{Height} \times 1\text{ byte per layer}$$

In modern AMR (Autonomous Mobile Robot) warehouse facilities:
* A 500 m x 500 m warehouse at 0.05 m resolution requires **100,000,000 cells (100 MB per layer)**.
* With static, obstacle, and inflation layers, a single AMR consumes **300 MB to 500 MB of continuous heap RAM**.
* Over 90% of a warehouse floor is empty open concrete (`FREE_SPACE = 0`). Standard grids allocate millions of identical zero-bytes, wasting memory bandwidth and forcing raycasters and global planners (A*, Hybrid A*) to iterate cell by cell across empty space.

In 3D robotics, this spatial sparsity was solved by **OctoMap** (octrees) and **OpenVDB** (hierarchical B-trees). `adaptive_costmap_2d` brings spatial memory compression to 2D robotics, engineered to avoid CPU pointer-chasing cache penalties.

---

## 2. Architecture: Paged Block-Sparse Grid

Rather than a pointer-heavy node-based quadtree, `adaptive_costmap_2d` utilizes a **Two-Level Paged Block-Sparse Architecture**:

1. **Top Level (Spatial Directory):** A 64-bit spatial hash directory mapping 2D block coordinates $(B_x, B_y)$ to allocated tiles using a high-throughput Murmur3 64-bit finalizer.
2. **Leaf Level (Cache-Aligned Tiles):** Contiguous 16x16 cell tiles (`SparseBlock<16>`), exactly 256 bytes (4 CPU cache lines, 64-byte aligned).
3. **Zero-Byte Sentinel Logic:** Unexplored or empty warehouse regions consume **0 bytes of heap memory**. When reading unallocated coordinates, $O(1)$ amortized lookups return the sentinel cost without allocating blocks.
4. **Hierarchical DDA Raycasting:** Raycasting uses a two-level Digital Differential Analyzer (DDA). When traversing a uniform empty block, the ray jumps directly to the block exit boundary in one calculation, bypassing cell-by-cell stepping across open aisles.

---

## 3. Empirical Benchmarks

Evaluated with C++20 Release builds (`-O3`) across realistic warehouse topologies (aisles, racks, pillars, walls):

| Environment | Size (m) | Grid Dimensions | Total Cells | Dense RAM (MB) | Sparse RAM (MB) | RAM Savings (%) | Insert Throughput | Raycast Latency |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Small Facility | 50m | 1,000 x 1,000 | 1,000,000 | 0.95 MB | 0.32 MB | **66.3%** | 24.37 Mops/s | 0.01 us |
| Medium Warehouse | 200m | 4,000 x 4,000 | 16,000,000 | 15.26 MB | 4.70 MB | **69.2%** | 5.90 Mops/s | 0.01 us |
| Logistics Hub | 500m | 10,000 x 10,000 | 100,000,000 | 95.37 MB | 29.14 MB | **69.4%** | 2.34 Mops/s | 0.01 us |
| Megafacility | 1,000m | 20,000 x 20,000 | 400,000,000 | 381.47 MB | 113.00 MB | **70.4%** | 1.63 Mops/s | 0.01 us |

---

## 4. Nav2 Configuration

Add `adaptive_costmap_2d::AdaptiveCostmapLayer` as a plugin in your Nav2 parameters YAML (e.g. `nav2_params.yaml`):

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "adaptive_obstacle_layer", "inflation_layer"]
      adaptive_obstacle_layer:
        plugin: "adaptive_costmap_2d::AdaptiveCostmapLayer"
        enabled: true
        topic: "/bcr_bot_1/scan"
        track_unknown_space: false
        combination_method: 1
```

---

## 5. Building and Running Tests

```bash
# Build package
colcon build --packages-select adaptive_costmap_2d --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run unit tests
./build/adaptive_costmap_2d/test_sparse_grid
./build/adaptive_costmap_2d/test_plugin_loading

# Run empirical benchmark
./build/adaptive_costmap_2d/benchmark_memory_and_speed
```

---

## License

This project is licensed under the Apache 2.0 License.
