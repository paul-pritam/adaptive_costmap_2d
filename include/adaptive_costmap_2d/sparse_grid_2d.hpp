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

#ifndef ADAPTIVE_COSTMAP_2D__SPARSE_GRID_2D_HPP_
#define ADAPTIVE_COSTMAP_2D__SPARSE_GRID_2D_HPP_

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "adaptive_costmap_2d/sparse_block.hpp"

namespace adaptive_costmap_2d
{

struct BlockCoordHash
{
  [[nodiscard]] std::size_t operator()(uint64_t k) const noexcept
  {
    // Fast 64-bit MurmurHash3 finalizer
    k ^= k >> 33;
    k *= 0xff51afd7ed558ccdULL;
    k ^= k >> 33;
    k *= 0xc4ceb9fe1a85ec53ULL;
    k ^= k >> 33;
    return static_cast<std::size_t>(k);
  }
};

/**
 * @brief High-performance block-sparse 2D spatial grid.
 *
 * Implements two-level hierarchical memory allocation:
 * - Level 1: Hashed 2D spatial directory of blocks.
 * - Level 2: Flat contiguous SparseBlock tiles (16x16 cells = 256 bytes).
 *
 * Empty warehouse regions and unexplored spaces are represented with 0 bytes of heap memory.
 */
template <int TileSize = 16>
class SparseGrid2D
{
public:
  using BlockType = SparseBlock<TileSize>;
  static constexpr int TILE_SIZE = TileSize;
  static constexpr int TILE_SHIFT = BlockType::SHIFT;
  static constexpr int TILE_MASK = BlockType::MASK;

  explicit SparseGrid2D(
    double resolution = 0.05,
    double origin_x = 0.0,
    double origin_y = 0.0,
    uint8_t default_value = 0)
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
    bx = cx >> TILE_SHIFT;
    by = cy >> TILE_SHIFT;
    lx = cx & TILE_MASK;
    ly = cy & TILE_MASK;
  }

  [[nodiscard]] inline uint8_t get_cost(int cx, int cy) const noexcept
  {
    int bx, by, lx, ly;
    cell_to_block(cx, cy, bx, by, lx, ly);

    const auto it = blocks_.find(pack_key(bx, by));
    if (it == blocks_.end()) {
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

    auto it = blocks_.find(key);
    if (it == blocks_.end()) {
      if (cost == default_value_) {
        return; // Zero allocation for default values
      }
      auto new_block = std::make_unique<BlockType>(default_value_);
      new_block->set(lx, ly, cost);
      new_block->touch(timestamp, cost == 254);
      blocks_.emplace(key, std::move(new_block));
    } else {
      it->second->set(lx, ly, cost);
      it->second->touch(timestamp, cost == 254);
    }

    update_bounds(cx, cy);
  }

  inline void set_cost_world(double wx, double wy, uint8_t cost, double timestamp = 0.0)
  {
    int cx, cy;
    world_to_cell(wx, wy, cx, cy);
    set_cost(cx, cy, cost, timestamp);
  }

  [[nodiscard]] bool is_block_allocated(int bx, int by) const noexcept
  {
    return blocks_.find(pack_key(bx, by)) != blocks_.end();
  }

  [[nodiscard]] bool is_block_uniform(int bx, int by, uint8_t & val) const noexcept
  {
    const auto it = blocks_.find(pack_key(bx, by));
    if (it == blocks_.end()) {
      val = default_value_;
      return true;
    }
    if (it->second->is_uniform()) {
      val = it->second->default_value;
      return true;
    }
    return false;
  }

  [[nodiscard]] bool is_block_free(int bx, int by) const noexcept
  {
    uint8_t val = 255;
    if (is_block_uniform(bx, by, val)) {
      return val == 0;
    }
    return false;
  }

  void clear() noexcept
  {
    blocks_.clear();
    min_cell_x_ = 1000000000;
    min_cell_y_ = 1000000000;
    max_cell_x_ = -1000000000;
    max_cell_y_ = -1000000000;
  }

  void prune()
  {
    for (auto it = blocks_.begin(); it != blocks_.end(); ) {
      if (it->second->is_uniform()) {
        it = blocks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  /**
   * @brief Automatic garbage collection: deallocates blocks that match default_value.
   *
   * Reclaims heap memory back to OS/allocator when areas revert to uniform empty.
   */
  size_t prune_unmodified_blocks()
  {
    size_t pruned = 0;
    for (auto it = blocks_.begin(); it != blocks_.end(); ) {
      if (it->second->can_prune()) {
        it = blocks_.erase(it);
        ++pruned;
      } else {
        ++it;
      }
    }
    return pruned;
  }

  /**
   * @brief Hierarchical 2D DDA raycaster with coarse empty block leaping.
   */
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

      int bx = cx >> TILE_SHIFT;
      int by = cy >> TILE_SHIFT;

      int target_bx = x1 >> TILE_SHIFT;
      int target_by = y1 >> TILE_SHIFT;

      uint8_t uniform_val = 0;
      if ((bx != target_bx || by != target_by) && is_block_uniform(bx, by, uniform_val)) {
        int next_bx_cell = (sx > 0) ? ((bx + 1) << TILE_SHIFT) - 1 : (bx << TILE_SHIFT);
        int next_by_cell = (sy > 0) ? ((by + 1) << TILE_SHIFT) - 1 : (by << TILE_SHIFT);

        while ((cx != next_bx_cell || cy != next_by_cell) && (cx != x1 || cy != y1)) {
          int e2 = 2 * err;
          if (e2 > -dy) {
            err -= dy;
            cx += sx;
          }
          if (e2 < dx) {
            err += dx;
            cy += sy;
          }
          action(cx, cy);
          if ((cx >> TILE_SHIFT) != bx || (cy >> TILE_SHIFT) != by) {
            break;
          }
        }
        continue;
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

  [[nodiscard]] size_t allocated_blocks_count() const noexcept
  {
    return blocks_.size();
  }

  [[nodiscard]] size_t memory_bytes_used() const noexcept
  {
    const size_t block_bytes = blocks_.size() * sizeof(BlockType);
    const size_t hash_overhead = blocks_.bucket_count() * sizeof(void*) +
                                 blocks_.size() * (sizeof(uint64_t) + sizeof(void*));
    return sizeof(*this) + block_bytes + hash_overhead;
  }

  [[nodiscard]] size_t equivalent_dense_bytes(int min_x, int min_y, int max_x, int max_y) const noexcept
  {
    if (min_x > max_x || min_y > max_y) {
      return 0;
    }
    const size_t w = static_cast<size_t>(max_x - min_x + 1);
    const size_t h = static_cast<size_t>(max_y - min_y + 1);
    return w * h;
  }

  [[nodiscard]] double compression_ratio(int min_x, int min_y, int max_x, int max_y) const noexcept
  {
    const size_t dense = equivalent_dense_bytes(min_x, min_y, max_x, max_y);
    if (dense == 0) {
      return 0.0;
    }
    const size_t sparse = memory_bytes_used();
    if (sparse >= dense) {
      return 0.0;
    }
    return 1.0 - (static_cast<double>(sparse) / static_cast<double>(dense));
  }

  [[nodiscard]] double resolution() const noexcept { return resolution_; }
  [[nodiscard]] double origin_x() const noexcept { return origin_x_; }
  [[nodiscard]] double origin_y() const noexcept { return origin_y_; }
  [[nodiscard]] uint8_t default_value() const noexcept { return default_value_; }

  [[nodiscard]] int min_cell_x() const noexcept { return min_cell_x_; }
  [[nodiscard]] int min_cell_y() const noexcept { return min_cell_y_; }
  [[nodiscard]] int max_cell_x() const noexcept { return max_cell_x_; }
  [[nodiscard]] int max_cell_y() const noexcept { return max_cell_y_; }

  void get_world_bounds(double & min_x, double & min_y, double & max_x, double & max_y) const noexcept
  {
    if (blocks_.empty()) {
      min_x = origin_x_;
      min_y = origin_y_;
      max_x = origin_x_;
      max_y = origin_y_;
      return;
    }
    cell_to_world(min_cell_x_, min_cell_y_, min_x, min_y);
    cell_to_world(max_cell_x_, max_cell_y_, max_x, max_y);
  }

  const std::unordered_map<uint64_t, std::unique_ptr<BlockType>, BlockCoordHash> & blocks() const noexcept
  {
    return blocks_;
  }

  // =========================================================================
  // Standard Dense Grid API Parity (Costmap2D & SLAM Toolbox compatibility)
  // =========================================================================

  [[nodiscard]] unsigned int getSizeInCellsX() const noexcept
  {
    return (min_cell_x_ <= max_cell_x_) ? static_cast<unsigned int>(max_cell_x_ - min_cell_x_ + 1) : 0U;
  }

  [[nodiscard]] unsigned int getSizeInCellsY() const noexcept
  {
    return (min_cell_y_ <= max_cell_y_) ? static_cast<unsigned int>(max_cell_y_ - min_cell_y_ + 1) : 0U;
  }

  [[nodiscard]] double getSizeInMetersX() const noexcept
  {
    return static_cast<double>(getSizeInCellsX()) * resolution_;
  }

  [[nodiscard]] double getSizeInMetersY() const noexcept
  {
    return static_cast<double>(getSizeInCellsY()) * resolution_;
  }

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

  /**
   * @brief Extracts a rectangular submap patch for fast scan-matching or local navigation.
   */
  void extract_submap(
    int min_cx, int min_cy, int max_cx, int max_cy,
    std::vector<uint8_t> & out_data, unsigned int & out_w, unsigned int & out_h) const
  {
    if (min_cx > max_cx || min_cy > max_cy) {
      out_w = 0;
      out_h = 0;
      out_data.clear();
      return;
    }
    out_w = static_cast<unsigned int>(max_cx - min_cx + 1);
    out_h = static_cast<unsigned int>(max_cy - min_cy + 1);
    out_data.resize(out_w * out_h);

    for (unsigned int j = 0; j < out_h; ++j) {
      const int cy = min_cy + static_cast<int>(j);
      for (unsigned int i = 0; i < out_w; ++i) {
        const int cx = min_cx + static_cast<int>(i);
        out_data[j * out_w + i] = get_cost(cx, cy);
      }
    }
  }

  size_t decay_ephemeral_obstacles(
    double current_time, double expiry_sec = 8.0, uint32_t min_hits = 4)
  {
    size_t decayed_blocks = 0;
    for (auto & [key, block] : blocks_) {
      if (block->decay_ephemeral(current_time, expiry_sec, min_hits)) {
        ++decayed_blocks;
      }
    }
    return decayed_blocks;
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

  void from_occupancy_grid(const nav_msgs::msg::OccupancyGrid & msg)
  {
    clear();
    resolution_ = static_cast<double>(msg.info.resolution);
    origin_x_ = msg.info.origin.position.x;
    origin_y_ = msg.info.origin.position.y;

    const uint32_t w = msg.info.width;
    const uint32_t h = msg.info.height;

    for (uint32_t j = 0; j < h; ++j) {
      for (uint32_t i = 0; i < w; ++i) {
        const int8_t occ = msg.data[j * w + i];
        if (occ < 0) {
          continue; // Unexplored remains unallocated sentinel
        }
        uint8_t cost = 0;
        if (occ == 0) {
          cost = 0;
        } else if (occ >= 100) {
          cost = 254;
        } else {
          cost = static_cast<uint8_t>((static_cast<int>(occ) * 254) / 100);
        }
        set_cost(static_cast<int>(i), static_cast<int>(j), cost);
      }
    }
  }

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
  uint8_t default_value_{0};

  int min_cell_x_{1000000000};
  int min_cell_y_{1000000000};
  int max_cell_x_{-1000000000};
  int max_cell_y_{-1000000000};

  std::unordered_map<uint64_t, std::unique_ptr<BlockType>, BlockCoordHash> blocks_;
};

}  // namespace adaptive_costmap_2d

#endif  // ADAPTIVE_COSTMAP_2D__SPARSE_GRID_2D_HPP_
