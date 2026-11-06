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
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>

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

  inline void set_cost(int cx, int cy, uint8_t cost)
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
      blocks_.emplace(key, std::move(new_block));
    } else {
      it->second->set(lx, ly, cost);
    }

    update_bounds(cx, cy);
  }

  inline void set_cost_world(double wx, double wy, uint8_t cost)
  {
    int cx, cy;
    world_to_cell(wx, wy, cx, cy);
    set_cost(cx, cy, cost);
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
   * @brief Hierarchical 2D DDA raycaster with coarse empty block leaping.
   *
   * Iterates along the ray from (x0, y0) to (x1, y1).
   * For empty uniform blocks, jumps directly to the block exit boundary,
   * bypassing cell-by-cell stepping across empty warehouse floors.
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

      // If in a uniform block that is not the target block, attempt coarse block leap
      int target_bx = x1 >> TILE_SHIFT;
      int target_by = y1 >> TILE_SHIFT;

      uint8_t uniform_val = 0;
      if ((bx != target_bx || by != target_by) && is_block_uniform(bx, by, uniform_val)) {
        // Leap to block boundary
        int next_bx_cell = (sx > 0) ? ((bx + 1) << TILE_SHIFT) - 1 : (bx << TILE_SHIFT);
        int next_by_cell = (sy > 0) ? ((by + 1) << TILE_SHIFT) - 1 : (by << TILE_SHIFT);

        // Advance to closest boundary
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
