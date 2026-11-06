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

#ifndef ADAPTIVE_COSTMAP_2D__MULTI_RES_QUADTREE_HPP_
#define ADAPTIVE_COSTMAP_2D__MULTI_RES_QUADTREE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <algorithm>
#include <vector>

namespace adaptive_costmap_2d
{

/**
 * @brief Node in a 2D multi-resolution Quadtree (OctoMap-style LOD).
 *
 * Each node represents a square spatial region:
 * - Depth 0: 1x1 cell (e.g. 0.05m)
 * - Depth 1: 2x2 cells (0.10m)
 * - Depth 2: 4x4 cells (0.20m)
 * - Depth 3: 8x8 cells (0.40m)
 * - Depth 4: 16x16 cells (0.80m)
 * - Depth 5: 32x32 cells (1.60m)
 */
struct alignas(32) QuadNode
{
  uint8_t cost{255};          // 0 = Free, 254 = Lethal, 255 = Unknown
  uint8_t depth{0};          // Distance from leaf (0 = 1x1 cell)
  bool is_leaf{true};
  bool has_lethal{false};
  double last_update_time{0.0};
  uint32_t reinforcement_hits{0};

  // 4 Children: [0]=NW, [1]=NE, [2]=SW, [3]=SE. Allocated only when subdivided.
  std::unique_ptr<std::array<QuadNode, 4>> children{nullptr};

  explicit QuadNode(uint8_t d = 0, uint8_t initial_cost = 255)
  : cost(initial_cost),
    depth(d),
    is_leaf(true),
    has_lethal(initial_cost == 254)
  {
  }

  inline void touch(double stamp, bool hit_obstacle = false) noexcept
  {
    last_update_time = stamp;
    if (hit_obstacle) {
      ++reinforcement_hits;
      has_lethal = true;
    }
  }

  void subdivide()
  {
    if (!is_leaf || depth == 0) {
      return;
    }
    const uint8_t child_depth = depth - 1;
    children = std::make_unique<std::array<QuadNode, 4>>();
    for (int i = 0; i < 4; ++i) {
      (*children)[i] = QuadNode(child_depth, cost);
      (*children)[i].last_update_time = last_update_time;
      (*children)[i].reinforcement_hits = 0; // Fresh child has 0 hits
      (*children)[i].has_lethal = (cost == 254);
    }
    is_leaf = false;
  }

  bool try_prune() noexcept
  {
    if (is_leaf || !children) {
      return false;
    }

    // All 4 children must be leaves
    for (int i = 0; i < 4; ++i) {
      if (!(*children)[i].is_leaf) {
        return false;
      }
    }

    // All 4 children must share the exact same cost
    const uint8_t c0 = (*children)[0].cost;
    for (int i = 1; i < 4; ++i) {
      if ((*children)[i].cost != c0) {
        return false;
      }
    }

    // Collapse children into parent coarse leaf
    cost = c0;
    has_lethal = (c0 == 254);
    is_leaf = true;
    children.reset(); // Free memory of all 4 children!
    return true;
  }

  [[nodiscard]] uint8_t get(int lx, int ly) const noexcept
  {
    if (is_leaf || !children) {
      return cost;
    }
    const int mid = 1 << (depth - 1);
    const int cx = (lx >= mid) ? 1 : 0;
    const int cy = (ly >= mid) ? 1 : 0;
    const int idx = (cy << 1) | cx;
    const int nlx = cx ? (lx - mid) : lx;
    const int nly = cy ? (ly - mid) : ly;
    return (*children)[idx].get(nlx, nly);
  }

  void set(int lx, int ly, uint8_t new_cost, double stamp)
  {
    last_update_time = stamp;
    if (new_cost == 254) {
      has_lethal = true;
    }

    if (depth == 0) {
      cost = new_cost;
      touch(stamp, new_cost == 254);
      return;
    }

    // If leaf already has this exact cost, no subdivision is needed
    if (is_leaf && cost == new_cost) {
      return;
    }

    // Subdivide if writing a non-uniform cost inside a coarse leaf
    if (is_leaf) {
      subdivide();
    }

    const int mid = 1 << (depth - 1);
    const int cx = (lx >= mid) ? 1 : 0;
    const int cy = (ly >= mid) ? 1 : 0;
    const int idx = (cy << 1) | cx;
    const int nlx = cx ? (lx - mid) : lx;
    const int nly = cy ? (ly - mid) : ly;

    (*children)[idx].set(nlx, nly, new_cost, stamp);

    // Recompute lethal status of parent
    has_lethal = false;
    for (int i = 0; i < 4; ++i) {
      if ((*children)[i].has_lethal) {
        has_lethal = true;
        break;
      }
    }

    try_prune();
  }

  bool decay_ephemeral(double current_time, double expiry_sec, uint32_t min_hits = 4) noexcept
  {
    if (!has_lethal) {
      return false;
    }

    bool any_decayed = false;
    if (is_leaf) {
      if (cost == 254 && reinforcement_hits < min_hits) {
        if ((current_time - last_update_time) > expiry_sec) {
          cost = 0; // Revert to free space
          has_lethal = false;
          reinforcement_hits = 0;
          return true;
        }
      }
      return false;
    }

    if (children) {
      for (int i = 0; i < 4; ++i) {
        if ((*children)[i].decay_ephemeral(current_time, expiry_sec, min_hits)) {
          any_decayed = true;
        }
      }
      // Re-evaluate parent has_lethal
      has_lethal = false;
      for (int i = 0; i < 4; ++i) {
        if ((*children)[i].has_lethal) {
          has_lethal = true;
          break;
        }
      }
      // Try collapse if children became uniform free
      try_prune();
    }
    return any_decayed;
  }

  [[nodiscard]] size_t memory_bytes() const noexcept
  {
    size_t bytes = sizeof(QuadNode);
    if (!is_leaf && children) {
      bytes += sizeof(std::array<QuadNode, 4>);
      for (int i = 0; i < 4; ++i) {
        if (!(*children)[i].is_leaf) {
          bytes += (*children)[i].memory_bytes() - sizeof(QuadNode);
        }
      }
    }
    return bytes;
  }

  [[nodiscard]] size_t count_leaves() const noexcept
  {
    if (is_leaf || !children) {
      return 1;
    }
    size_t count = 0;
    for (int i = 0; i < 4; ++i) {
      count += (*children)[i].count_leaves();
    }
    return count;
  }
};

}  // namespace adaptive_costmap_2d

#endif  // ADAPTIVE_COSTMAP_2D__MULTI_RES_QUADTREE_HPP_
