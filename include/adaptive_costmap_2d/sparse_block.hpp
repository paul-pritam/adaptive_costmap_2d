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

#ifndef ADAPTIVE_COSTMAP_2D__SPARSE_BLOCK_HPP_
#define ADAPTIVE_COSTMAP_2D__SPARSE_BLOCK_HPP_

#include <array>
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace adaptive_costmap_2d
{

/**
 * @brief Fixed-size cache-aligned block tile storing a contiguous local 2D cell grid.
 *
 * Designed to fit cleanly across CPU L1 cache lines (64-byte aligned).
 * Default TileSize = 16 corresponds to a 256-byte tile (exactly 4 cache lines).
 */
template <int TileSize = 16>
struct alignas(64) SparseBlock
{
  static constexpr int SIZE = TileSize;
  static constexpr int NUM_CELLS = TileSize * TileSize;
  static constexpr int SHIFT = (TileSize == 16) ? 4 : (TileSize == 32 ? 5 : 4);
  static constexpr int MASK = TileSize - 1;

  std::array<uint8_t, NUM_CELLS> cells;
  uint32_t non_default_count{0};
  uint32_t lethal_count{0};
  uint8_t default_value{0};
  double last_update_time{0.0};
  uint32_t reinforcement_hits{0};

  explicit SparseBlock(uint8_t initial_val = 0)
  : default_value(initial_val)
  {
    cells.fill(initial_val);
  }

  inline void touch(double stamp, bool hit_obstacle = false) noexcept
  {
    last_update_time = stamp;
    if (hit_obstacle) {
      ++reinforcement_hits;
    }
  }

  /**
   * @brief Decays unreinforced ephemeral obstacles after an elapsed timeout.
   *
   * If an obstacle was observed with fewer than min_reinforcement_hits and has not
   * been touched for expiry_sec, it is cleared to free space (cost 0).
   */
  inline bool decay_ephemeral(
    double current_time, double expiry_sec, uint32_t min_reinforcement_hits = 4) noexcept
  {
    if (has_lethal() && reinforcement_hits < min_reinforcement_hits) {
      if ((current_time - last_update_time) > expiry_sec) {
        fill(0);
        reinforcement_hits = 0;
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] inline uint8_t get(int lx, int ly) const noexcept
  {
    return cells[(ly & MASK) * SIZE + (lx & MASK)];
  }

  inline void set(int lx, int ly, uint8_t val) noexcept
  {
    const int idx = (ly & MASK) * SIZE + (lx & MASK);
    const uint8_t old_val = cells[idx];
    if (old_val == val) {
      return;
    }

    if (old_val == default_value && val != default_value) {
      ++non_default_count;
    } else if (old_val != default_value && val == default_value) {
      --non_default_count;
    }

    if (old_val == 254 && val != 254) {
      --lethal_count;
    } else if (old_val != 254 && val == 254) {
      ++lethal_count;
    }

    cells[idx] = val;
  }

  inline void fill(uint8_t val) noexcept
  {
    cells.fill(val);
    non_default_count = (val == default_value) ? 0 : NUM_CELLS;
    lethal_count = (val == 254) ? NUM_CELLS : 0;
  }

  [[nodiscard]] inline bool is_uniform() const noexcept
  {
    return non_default_count == 0;
  }

  [[nodiscard]] inline bool can_prune() const noexcept
  {
    return non_default_count == 0;
  }

  [[nodiscard]] inline bool has_lethal() const noexcept
  {
    return lethal_count > 0;
  }

  [[nodiscard]] static constexpr size_t byte_size() noexcept
  {
    return sizeof(SparseBlock<TileSize>);
  }
};

}  // namespace adaptive_costmap_2d

#endif  // ADAPTIVE_COSTMAP_2D__SPARSE_BLOCK_HPP_
