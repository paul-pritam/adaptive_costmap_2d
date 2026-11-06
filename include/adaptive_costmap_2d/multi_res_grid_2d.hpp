// Copyright 2026 Pritam Paul
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ADAPTIVE_COSTMAP_2D__MULTI_RES_GRID_2D_HPP_
#define ADAPTIVE_COSTMAP_2D__MULTI_RES_GRID_2D_HPP_

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <algorithm>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "adaptive_costmap_2d/multi_res_quadtree.hpp"
#include "adaptive_costmap_2d/sparse_grid_2d.hpp" // For BlockCoordHash

namespace adaptive_costmap_2d
{

/**
 * @brief Representation of an individual multi-resolution leaf box for RViz rendering.
 */
struct MultiResBox
{
  double wx_min{0.0};
  double wy_min{0.0};
  double wx_max{0.0};
  double wy_max{0.0};
  double size_m{0.0};
  uint8_t depth{0};
  uint8_t cost{255};
  bool has_lethal{false};
};

/**
 * @brief True Multi-Resolution Spatial Grid (OctoMap/Voxel-style LOD for 2D Mobile Robotics).
 *
 * Implements adaptive spatial subdivision:
 * - Open warehouse floors collapse into coarse macro-tiles (0.8m or 1.6m).
 * - Walls and obstacles trigger hierarchical quadtree subdivision down to 0.05m cells.
 * - Delivers full Costmap2D API parity, dynamic decay, and ROS 2 OccupancyGrid streaming.
 *
 * @tparam MaxDepth Root block depth. 4 = 16x16 cells (0.8m), 5 = 32x32 cells (1.6m).
 */
template <int MaxDepth = 4>
class MultiResGrid2D
{
public:
  static constexpr int ROOT_DEPTH = MaxDepth;
  static constexpr int TILE_SIZE = 1 << ROOT_DEPTH;
  static constexpr int TILE_MASK = TILE_SIZE - 1;

  explicit MultiResGrid2D(
    double resolution = 0.05,
    double origin_x = 0.0,
    double origin_y = 0.0,
    uint8_t default_value = 255)
  : resolution_(resolution),
    origin_x_(origin_x),
    origin_y_(origin_y),
    default_value_(default_value)
  {
  }

  [[nodiscard]] static constexpr uint64_t pack_key(int32_t bx, int32_t by) noexcept
  {
    return (static_cast<uint64_t>(static_cast<uint32_t>(bx)) << 32) |
           static_cast<uint32_t>(by);
  }

  static void unpack_key(uint64_t key, int32_t & bx, int32_t & by) noexcept
  {
    bx = static_cast<int32_t>(static_cast<uint32_t>(key >> 32));
    by = static_cast<int32_t>(static_cast<uint32_t>(key & 0xFFFFFFFFULL));
  }

  inline void world_to_cell(double wx, double wy, int & cx, int & cy) const noexcept
  {
    cx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    cy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  }

  inline void cell_to_world(int cx, int cy, double & wx, double & wy) const noexcept
  {
    wx = origin_x_ + (static_cast<double>(cx) + 0.5) * resolution_;
    wy = origin_y_ + (static_cast<double>(cy) + 0.5) * resolution_;
  }

  static inline void cell_to_block(int cx, int cy, int & bx, int & by, int & lx, int & ly) noexcept
  {
    bx = cx >> ROOT_DEPTH;
    by = cy >> ROOT_DEPTH;
    lx = cx & TILE_MASK;
    ly = cy & TILE_MASK;
  }

  [[nodiscard]] inline uint8_t get_cost(int cx, int cy) const noexcept
  {
    int bx, by, lx, ly;
    cell_to_block(cx, cy, bx, by, lx, ly);

    const auto it = macro_blocks_.find(pack_key(bx, by));
    if (it == macro_blocks_.end()) {
      return default_value_;
    }
    return it->second->get(lx, ly);
  }

  [[nodiscard]] inline uint8_t get_cost_world(double wx, double wy) const noexcept
  {
    int cx, cy;
    world_to_cell(wx, wy, cx, cy);
    return get_cost(cx, cy);
  }

  inline void set_cost(int cx, int cy, uint8_t cost, double timestamp = 0.0)
  {
    int bx, by, lx, ly;
    cell_to_block(cx, cy, bx, by, lx, ly);
    const uint64_t key = pack_key(bx, by);

    auto it = macro_blocks_.find(key);
    if (it == macro_blocks_.end()) {
      if (cost == default_value_) {
        return; // Zero allocation for default sentinels
      }
      auto new_root = std::make_unique<QuadNode>(ROOT_DEPTH, default_value_);
      new_root->set(lx, ly, cost, timestamp);
      macro_blocks_.emplace(key, std::move(new_root));
    } else {
      it->second->set(lx, ly, cost, timestamp);
    }

    update_bounds(cx, cy);
  }

  inline void set_cost_world(double wx, double wy, uint8_t cost, double timestamp = 0.0)
  {
    int cx, cy;
    world_to_cell(wx, wy, cx, cy);
    set_cost(cx, cy, cost, timestamp);
  }

  template <typename RayAction>
  void raytrace(int x0, int y0, int x1, int y1, RayAction && action)
  {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int cx = x0;
    int cy = y0;

    while (true) {
      action(cx, cy);

      if (cx == x1 && cy == y1) {
        break;
      }

      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        cx += sx;
      }
      if (e2 < dx) {
        err += dx;
        cy += sy;
      }
    }
  }

  /**
   * @brief Collects all active multi-resolution leaf boxes for hierarchical RViz visualization.
   */
  void collect_multires_boxes(std::vector<MultiResBox> & out_boxes) const
  {
    out_boxes.clear();

    const double tile_world_size = static_cast<double>(TILE_SIZE) * resolution_;

    for (const auto & [key, root] : macro_blocks_) {
      int32_t bx, by;
      unpack_key(key, bx, by);

      const double block_wx_min = origin_x_ + static_cast<double>(bx * TILE_SIZE) * resolution_;
      const double block_wy_min = origin_y_ + static_cast<double>(by * TILE_SIZE) * resolution_;

      // Recursive lambda to collect all leaves with their respective spatial extents
      std::function<void(const QuadNode &, double, double, double)> traverse;
      traverse = [&](const QuadNode & node, double wx, double wy, double sz) {
        if (node.is_leaf || !node.children) {
          // Only emit non-unknown boxes (free or lethal)
          if (node.cost != default_value_ || node.has_lethal) {
            MultiResBox box;
            box.wx_min = wx;
            box.wy_min = wy;
            box.wx_max = wx + sz;
            box.wy_max = wy + sz;
            box.size_m = sz;
            box.depth = node.depth;
            box.cost = node.cost;
            box.has_lethal = node.has_lethal;
            out_boxes.push_back(box);
          }
          return;
        }

        const double half_sz = sz * 0.5;
        // [0]=NW: (wx, wy + half_sz)
        // [1]=NE: (wx + half_sz, wy + half_sz)
        // [2]=SW: (wx, wy)
        // [3]=SE: (wx + half_sz, wy)
        traverse((*node.children)[0], wx, wy + half_sz, half_sz);
        traverse((*node.children)[1], wx + half_sz, wy + half_sz, half_sz);
        traverse((*node.children)[2], wx, wy, half_sz);
        traverse((*node.children)[3], wx + half_sz, wy, half_sz);
      };

      traverse(*root, block_wx_min, block_wy_min, tile_world_size);
    }
  }

  size_t decay_ephemeral_obstacles(
    double current_time, double expiry_sec = 8.0, uint32_t min_hits = 4)
  {
    size_t decayed = 0;
    for (auto & [key, root] : macro_blocks_) {
      if (root->decay_ephemeral(current_time, expiry_sec, min_hits)) {
        ++decayed;
      }
    }
    return decayed;
  }

  size_t prune_unmodified_blocks()
  {
    size_t pruned = 0;
    for (auto it = macro_blocks_.begin(); it != macro_blocks_.end(); ) {
      it->second->try_prune();
      if (it->second->is_leaf && it->second->cost == default_value_) {
        it = macro_blocks_.erase(it);
        ++pruned;
      } else {
        ++it;
      }
    }
    return pruned;
  }

  void clear() noexcept
  {
    macro_blocks_.clear();
    min_cell_x_ = 1000000000;
    min_cell_y_ = 1000000000;
    max_cell_x_ = -1000000000;
    max_cell_y_ = -1000000000;
  }

  // =========================================================================
  // Costmap2D API Parity & Nav2 / SLAM Toolbox Streaming
  // =========================================================================

  [[nodiscard]] unsigned int getSizeInCellsX() const noexcept
  {
    return (min_cell_x_ <= max_cell_x_) ? static_cast<unsigned int>(max_cell_x_ - min_cell_x_ + 1) : 0U;
  }

  [[nodiscard]] unsigned int getSizeInCellsY() const noexcept
  {
    return (min_cell_y_ <= max_cell_y_) ? static_cast<unsigned int>(max_cell_y_ - min_cell_y_ + 1) : 0U;
  }

  [[nodiscard]] double getSizeInMetersX() const noexcept { return static_cast<double>(getSizeInCellsX()) * resolution_; }
  [[nodiscard]] double getSizeInMetersY() const noexcept { return static_cast<double>(getSizeInCellsY()) * resolution_; }
  [[nodiscard]] double getOriginX() const noexcept { return origin_x_; }
  [[nodiscard]] double getOriginY() const noexcept { return origin_y_; }
  [[nodiscard]] double getResolution() const noexcept { return resolution_; }

  [[nodiscard]] uint8_t getCost(unsigned int mx, unsigned int my) const noexcept
  {
    const int cx = min_cell_x_ + static_cast<int>(mx);
    const int cy = min_cell_y_ + static_cast<int>(my);
    return get_cost(cx, cy);
  }

  void setCost(unsigned int mx, unsigned int my, uint8_t cost, double timestamp = 0.0)
  {
    const int cx = min_cell_x_ + static_cast<int>(mx);
    const int cy = min_cell_y_ + static_cast<int>(my);
    set_cost(cx, cy, cost, timestamp);
  }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const noexcept
  {
    int cx, cy;
    world_to_cell(wx, wy, cx, cy);
    if (cx < min_cell_x_ || cy < min_cell_y_ || cx > max_cell_x_ || cy > max_cell_y_) {
      return false;
    }
    mx = static_cast<unsigned int>(cx - min_cell_x_);
    my = static_cast<unsigned int>(cy - min_cell_y_);
    return true;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const noexcept
  {
    const int cx = min_cell_x_ + static_cast<int>(mx);
    const int cy = min_cell_y_ + static_cast<int>(my);
    cell_to_world(cx, cy, wx, wy);
  }

  void to_occupancy_grid(
    nav_msgs::msg::OccupancyGrid & msg,
    const std::string & frame_id = "map",
    const builtin_interfaces::msg::Time & stamp = builtin_interfaces::msg::Time()) const
  {
    const uint32_t w = getSizeInCellsX();
    const uint32_t h = getSizeInCellsY();
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.info.resolution = static_cast<float>(resolution_);
    msg.info.width = w;
    msg.info.height = h;

    if (w == 0 || h == 0) {
      msg.info.origin.position.x = origin_x_;
      msg.info.origin.position.y = origin_y_;
      msg.info.origin.position.z = 0.0;
      msg.info.origin.orientation.w = 1.0;
      msg.data.clear();
      return;
    }

    double origin_wx, origin_wy;
    cell_to_world(min_cell_x_, min_cell_y_, origin_wx, origin_wy);
    msg.info.origin.position.x = origin_wx - 0.5 * resolution_;
    msg.info.origin.position.y = origin_wy - 0.5 * resolution_;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(w * h, -1);

    for (uint32_t j = 0; j < h; ++j) {
      const int cy = min_cell_y_ + static_cast<int>(j);
      for (uint32_t i = 0; i < w; ++i) {
        const int cx = min_cell_x_ + static_cast<int>(i);
        const uint8_t cost = get_cost(cx, cy);

        int8_t occ_val = -1;
        if (cost == 0) {
          occ_val = 0;
        } else if (cost == 254) {
          occ_val = 100;
        } else if (cost == 255) {
          occ_val = -1;
        } else {
          occ_val = static_cast<int8_t>((static_cast<int>(cost) * 100) / 254);
        }
        msg.data[j * w + i] = occ_val;
      }
    }
  }

  [[nodiscard]] size_t allocated_blocks_count() const noexcept
  {
    return macro_blocks_.size();
  }

  [[nodiscard]] size_t total_quad_nodes_count() const noexcept
  {
    size_t count = 0;
    for (const auto & [key, root] : macro_blocks_) {
      count += root->count_leaves();
    }
    return count;
  }

  [[nodiscard]] size_t memory_bytes_used() const noexcept
  {
    size_t bytes = sizeof(*this);
    for (const auto & [key, root] : macro_blocks_) {
      bytes += sizeof(uint64_t) + root->memory_bytes();
    }
    return bytes;
  }

  [[nodiscard]] double compression_ratio(int min_x, int min_y, int max_x, int max_y) const noexcept
  {
    if (min_x > max_x || min_y > max_y) {
      return 0.0;
    }
    const size_t dense = static_cast<size_t>(max_x - min_x + 1) * static_cast<size_t>(max_y - min_y + 1);
    const size_t sparse = memory_bytes_used();
    if (sparse >= dense || dense == 0) {
      return 0.0;
    }
    return 1.0 - (static_cast<double>(sparse) / static_cast<double>(dense));
  }

  [[nodiscard]] int min_cell_x() const noexcept { return min_cell_x_; }
  [[nodiscard]] int min_cell_y() const noexcept { return min_cell_y_; }
  [[nodiscard]] int max_cell_x() const noexcept { return max_cell_x_; }
  [[nodiscard]] int max_cell_y() const noexcept { return max_cell_y_; }
  [[nodiscard]] double resolution() const noexcept { return resolution_; }
  [[nodiscard]] double origin_x() const noexcept { return origin_x_; }
  [[nodiscard]] double origin_y() const noexcept { return origin_y_; }

private:
  inline void update_bounds(int cx, int cy) noexcept
  {
    min_cell_x_ = std::min(min_cell_x_, cx);
    min_cell_y_ = std::min(min_cell_y_, cy);
    max_cell_x_ = std::max(max_cell_x_, cx);
    max_cell_y_ = std::max(max_cell_y_, cy);
  }

  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  uint8_t default_value_{255};

  int min_cell_x_{1000000000};
  int min_cell_y_{1000000000};
  int max_cell_x_{-1000000000};
  int max_cell_y_{-1000000000};

  std::unordered_map<uint64_t, std::unique_ptr<QuadNode>, BlockCoordHash> macro_blocks_;
};

}  // namespace adaptive_costmap_2d

#endif  // ADAPTIVE_COSTMAP_2D__MULTI_RES_GRID_2D_HPP_
