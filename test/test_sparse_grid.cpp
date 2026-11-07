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

#include <cstdint>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "adaptive_costmap_2d/sparse_block.hpp"
#include "adaptive_costmap_2d/sparse_grid_2d.hpp"
#include "adaptive_costmap_2d/block_visualizer.hpp"
#include "adaptive_costmap_2d/karto_sparse_grid.hpp"

using adaptive_costmap_2d::SparseBlock;
using adaptive_costmap_2d::SparseGrid2D;
using adaptive_costmap_2d::BlockVisualizer;
using adaptive_costmap_2d::SparseKartoGrid;

TEST(SparseBlockTest, DefaultStateAndBitmask)
{
  SparseBlock<16> block(0);
  EXPECT_TRUE(block.is_uniform());
  EXPECT_TRUE(block.can_prune());
  EXPECT_FALSE(block.has_lethal());
  EXPECT_EQ(block.get(0, 0), 0);
  EXPECT_EQ(block.get(15, 15), 0);

  block.set(5, 5, 254);
  EXPECT_FALSE(block.is_uniform());
  EXPECT_FALSE(block.can_prune());
  EXPECT_TRUE(block.has_lethal());
  EXPECT_EQ(block.get(5, 5), 254);

  // Reset back to zero
  block.set(5, 5, 0);
  EXPECT_TRUE(block.is_uniform());
  EXPECT_TRUE(block.can_prune());
  EXPECT_FALSE(block.has_lethal());
}

TEST(SparseGridTest, InitialZeroAllocation)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);
  EXPECT_EQ(grid.allocated_blocks_count(), 0UL);
  EXPECT_EQ(grid.get_cost(0, 0), 255);
  EXPECT_EQ(grid.get_cost(1000, 2000), 255);
  EXPECT_EQ(grid.get_cost(-500, -800), 255);
  EXPECT_EQ(grid.allocated_blocks_count(), 0UL);
}

TEST(SparseGridTest, OnDemandBlockAllocation)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 0);
  EXPECT_EQ(grid.allocated_blocks_count(), 0UL);

  // Write inside Block (0, 0)
  grid.set_cost(2, 3, 254);
  EXPECT_EQ(grid.allocated_blocks_count(), 1UL);
  EXPECT_EQ(grid.get_cost(2, 3), 254);
  EXPECT_EQ(grid.get_cost(0, 0), 0);

  // Write another cell inside same block (0, 0)
  grid.set_cost(10, 10, 128);
  EXPECT_EQ(grid.allocated_blocks_count(), 1UL);
  EXPECT_EQ(grid.get_cost(10, 10), 128);

  // Write cell in adjacent block (1, 0) -> cell x = 16
  grid.set_cost(16, 5, 200);
  EXPECT_EQ(grid.allocated_blocks_count(), 2UL);
  EXPECT_EQ(grid.get_cost(16, 5), 200);
}

TEST(SparseGridTest, NegativeCoordinateHandling)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 0);

  grid.set_cost(-1, -1, 150);
  grid.set_cost(-16, -16, 254);
  grid.set_cost(-25, 40, 99);

  EXPECT_EQ(grid.get_cost(-1, -1), 150);
  EXPECT_EQ(grid.get_cost(-16, -16), 254);
  EXPECT_EQ(grid.get_cost(-25, 40), 99);
  EXPECT_EQ(grid.get_cost(-2, -2), 0);
}

TEST(SparseGridTest, WorldCoordinateProjection)
{
  SparseGrid2D<16> grid(0.05, -10.0, -20.0, 0);

  grid.set_cost_world(0.0, 0.0, 254);
  EXPECT_EQ(grid.get_cost_world(0.0, 0.0), 254);
  EXPECT_EQ(grid.get_cost_world(0.5, 0.5), 0);

  double s_min_x, s_min_y, s_max_x, s_max_y;
  grid.get_world_bounds(s_min_x, s_min_y, s_max_x, s_max_y);
  EXPECT_NEAR(s_min_x, 0.0, 0.1);
  EXPECT_NEAR(s_min_y, 0.0, 0.1);
}

TEST(SparseGridTest, HierarchicalRaytraceCoverage)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  int visit_count = 0;
  grid.raytrace(0, 0, 32, 0, [&](int cx, int cy) {
      grid.set_cost(cx, cy, 0);  // clear ray
      ++visit_count;
  });

  EXPECT_EQ(visit_count, 33);  // 0 to 32 inclusive
  for (int x = 0; x <= 32; ++x) {
    EXPECT_EQ(grid.get_cost(x, 0), 0);
  }
}

TEST(SparseGridTest, BoundedMemorySavingsOver80Percent)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 0);

  for (int x = 400; x < 1800; x += 400) {
    for (int y = 400; y < 1600; ++y) {
      grid.set_cost(x, y, 254);
    }
  }

  const size_t dense_bytes = grid.equivalent_dense_bytes(0, 0, 1999, 1999);
  const size_t sparse_bytes = grid.memory_bytes_used();
  const double savings = grid.compression_ratio(0, 0, 1999, 1999);

  EXPECT_EQ(dense_bytes, 4000000UL);
  EXPECT_LT(sparse_bytes, dense_bytes / 4);
  EXPECT_GT(savings, 0.75);
}

TEST(BlockVisualizerTest, CreatesWireframesAndTelemetryHUD)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  grid.set_cost(5, 5, 254);
  grid.set_cost(20, 5, 0);

  auto markers = BlockVisualizer::create_markers(grid, "map", rclcpp::Time(0), 100, 100, 0.012);

  ASSERT_EQ(markers.markers.size(), 3UL);

  const auto & wireframes = markers.markers[0];
  EXPECT_EQ(wireframes.type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(wireframes.points.size(), 16UL);

  const auto & lethal = markers.markers[1];
  EXPECT_EQ(lethal.type, visualization_msgs::msg::Marker::POINTS);
  EXPECT_EQ(lethal.points.size(), 1UL);

  const auto & hud = markers.markers[2];
  EXPECT_EQ(hud.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_FALSE(hud.text.empty());
  EXPECT_NE(hud.text.find("ADAPTIVE BLOCK-SPARSE 2D MAP"), std::string::npos);
}

TEST(SparseGridTest, StandardDenseGridApiParity)
{
  SparseGrid2D<16> grid(0.05, 10.0, 20.0, 255);

  grid.set_cost(0, 0, 0);
  grid.set_cost(50, 50, 254);

  EXPECT_EQ(grid.getSizeInCellsX(), 51U);
  EXPECT_EQ(grid.getSizeInCellsY(), 51U);
  EXPECT_NEAR(grid.getSizeInMetersX(), 51 * 0.05, 1e-4);
  EXPECT_NEAR(grid.getSizeInMetersY(), 51 * 0.05, 1e-4);

  double wx, wy;
  grid.mapToWorld(0, 0, wx, wy);
  EXPECT_NEAR(wx, 10.0 + 0.5 * 0.05, 1e-4);
  EXPECT_NEAR(wy, 20.0 + 0.5 * 0.05, 1e-4);

  unsigned int mx = 0, my = 0;
  EXPECT_TRUE(grid.worldToMap(wx, wy, mx, my));
  EXPECT_EQ(mx, 0U);
  EXPECT_EQ(my, 0U);

  EXPECT_EQ(grid.getCost(0, 0), 0);
  EXPECT_EQ(grid.getCost(50, 50), 254);

  grid.setCost(10, 10, 128);
  EXPECT_EQ(grid.getCost(10, 10), 128);
}

TEST(SparseGridTest, OccupancyGridBidirectionalConversion)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  grid.set_cost(0, 0, 0);      // Free (0)
  grid.set_cost(10, 10, 254);  // Lethal (100 in ROS)

  nav_msgs::msg::OccupancyGrid msg;
  grid.to_occupancy_grid(msg, "map");

  EXPECT_EQ(msg.info.width, 11U);
  EXPECT_EQ(msg.info.height, 11U);
  EXPECT_EQ(msg.data[0], 0);
  EXPECT_EQ(msg.data[10 * 11 + 10], 100);

  SparseGrid2D<16> restored(0.05, 0.0, 0.0, 255);
  restored.from_occupancy_grid(msg);

  EXPECT_EQ(restored.get_cost(0, 0), 0);
  EXPECT_EQ(restored.get_cost(10, 10), 254);
}

TEST(SparseGridTest, TemporalDynamicDecayOfEphemeralObstacles)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  // Transient obstacle with single hit at t=10.0
  grid.set_cost(2, 2, 254, 10.0);
  EXPECT_EQ(grid.get_cost(2, 2), 254);

  // Permanent obstacle reinforced with 5 hits at t=10.0
  for (int h = 0; h < 5; ++h) {
    grid.set_cost(34, 34, 254, 10.0);
  }
  EXPECT_EQ(grid.get_cost(34, 34), 254);

  // t=12.0 (elapsed 2.0s with 5.0s expiry): neither decays
  size_t decayed = grid.decay_ephemeral_obstacles(12.0, 5.0, 4);
  EXPECT_EQ(decayed, 0UL);
  EXPECT_EQ(grid.get_cost(2, 2), 254);
  EXPECT_EQ(grid.get_cost(34, 34), 254);

  // t=20.0 (elapsed 10.0s with 5.0s expiry): transient decays, permanent persists
  decayed = grid.decay_ephemeral_obstacles(20.0, 5.0, 4);
  EXPECT_EQ(decayed, 1UL);
  EXPECT_EQ(grid.get_cost(2, 2), 0);      // Decayed to free
  EXPECT_EQ(grid.get_cost(34, 34), 254);  // Persisted
}

TEST(SparseGridTest, DynamicBlockPruningReclaimsMemory)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  // Allocate block (0, 0)
  grid.set_cost(5, 5, 254);
  EXPECT_EQ(grid.allocated_blocks_count(), 1UL);

  // Reset all cells in block (0, 0) back to default 255
  grid.set_cost(5, 5, 255);
  EXPECT_EQ(grid.allocated_blocks_count(), 1UL);  // Still in hash table until pruned

  // Trigger dynamic pruning
  size_t pruned = grid.prune_unmodified_blocks();
  EXPECT_EQ(pruned, 1UL);
  EXPECT_EQ(grid.allocated_blocks_count(), 0UL);  // Reclaimed!
}

TEST(SparseGridTest, SubmapPatchExtraction)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  grid.set_cost(10, 10, 100);
  grid.set_cost(11, 10, 200);
  grid.set_cost(10, 11, 50);

  std::vector<uint8_t> patch;
  unsigned int pw = 0, ph = 0;
  grid.extract_submap(10, 10, 11, 11, patch, pw, ph);

  EXPECT_EQ(pw, 2U);
  EXPECT_EQ(ph, 2U);
  EXPECT_EQ(patch.size(), 4UL);
  EXPECT_EQ(patch[0], 100);  // (10, 10)
  EXPECT_EQ(patch[1], 200);  // (11, 10)
  EXPECT_EQ(patch[2], 50);   // (10, 11)
  EXPECT_EQ(patch[3], 255);  // (11, 11) default
}

TEST(SparseKartoGridTest, DropInCompatibilityAndMemorySavings)
{
  SparseKartoGrid karto_grid(0.05, -5.0, -5.0);

  // Mark an obstacle and raytrace free space
  karto_grid.set_state(10, 10, SparseKartoGrid::StateOccupied);
  karto_grid.set_state(0, 0, SparseKartoGrid::StateFree);

  EXPECT_EQ(karto_grid.get_state(10, 10), SparseKartoGrid::StateOccupied);
  EXPECT_EQ(karto_grid.get_state(0, 0), SparseKartoGrid::StateFree);
  EXPECT_EQ(karto_grid.get_state(100, 100), SparseKartoGrid::StateUnknown);

  // Check dense equivalent calculation (9 bytes per cell)
  const size_t karto_dense = karto_grid.get_karto_dense_equivalent_bytes();
  const size_t sparse_bytes = karto_grid.get_sparse_memory_bytes();
  EXPECT_GT(karto_dense, 0UL);
  EXPECT_GT(sparse_bytes, 0UL);

  // Check ROS OccupancyGrid export
  nav_msgs::msg::OccupancyGrid ros_map;
  karto_grid.to_ros_occupancy_grid(ros_map, "map");
  EXPECT_EQ(ros_map.info.width, 11U);
  EXPECT_EQ(ros_map.info.height, 11U);
  EXPECT_EQ(ros_map.data[0], 0);                // Free space
  EXPECT_EQ(ros_map.data[10 * 11 + 10], 100);  // Occupied obstacle
}

// ============================================================================
// Milestone 1: Probabilistic Storage & Occupancy Estimation Unit Tests
// ============================================================================

TEST(SparseBlockTest, CacheAlignmentAndSizeConstraint)
{
  // Requirement R2 & PROJECT.md: 64-byte alignment and exact 2,368 bytes (37 cache lines)
  static_assert(
    sizeof(SparseBlock<16>) == 2368,
    "SparseBlock<16> footprint must be exactly 2368 bytes (37 cache lines)");
  static_assert(
    alignof(SparseBlock<16>) == 64,
    "SparseBlock<16> must be 64-byte aligned for AVX-512 and cache efficiency");

  EXPECT_EQ(sizeof(SparseBlock<16>), 2368UL);
  EXPECT_EQ(alignof(SparseBlock<16>), 64UL);

  // Runtime stack alignment validation
  SparseBlock<16> stack_block(255);
  uintptr_t stack_addr = reinterpret_cast<uintptr_t>(&stack_block);
  EXPECT_EQ(stack_addr % 64UL, 0UL) << "Block on stack must be 64-byte aligned";

  // Runtime heap alignment validation
  auto heap_block = std::make_unique<SparseBlock<16>>(255);
  uintptr_t heap_addr = reinterpret_cast<uintptr_t>(heap_block.get());
  EXPECT_EQ(heap_addr % 64UL, 0UL) << "Block on heap must be 64-byte aligned";
}

TEST(SparseBlockTest, ProbabilisticInitialState)
{
  SparseBlock<16> block(255);  // Initialized with NO_INFORMATION (255)

  EXPECT_TRUE(block.is_uniform());
  EXPECT_TRUE(block.can_prune());
  EXPECT_FALSE(block.has_lethal());

  // Verify all 256 cells default to cost 255 with exactly 0 passes and 0 hits
  for (int ly = 0; ly < 16; ++ly) {
    for (int lx = 0; lx < 16; ++lx) {
      EXPECT_EQ(block.get(lx, ly), 255);
      EXPECT_EQ(block.get_passes(lx, ly), 0U);
      EXPECT_EQ(block.get_hits(lx, ly), 0U);
    }
  }
}

TEST(SparseBlockTest, NoiseRejectionBelowMinPassThrough)
{
  SparseBlock<16> block(255);
  const uint32_t min_pass_through = 2;
  const double occupancy_threshold = 0.1;

  // Case 1: Stray single laser hit (e.g. dust particle or reflection): 1 pass, 1 hit
  block.record_hit(4, 4, min_pass_through, occupancy_threshold);
  EXPECT_EQ(block.get_passes(4, 4), 1U);
  EXPECT_EQ(block.get_hits(4, 4), 1U);
  // passes (1) <= min_pass_through (2) -> cell state MUST remain NO_INFORMATION (255)
  EXPECT_EQ(block.get(4, 4), 255);
  EXPECT_FALSE(block.has_lethal());

  // Case 2: Boundary condition: exactly 2 passes, 2 hits (passes == min_pass_through)
  block.record_hit(4, 4, min_pass_through, occupancy_threshold);
  EXPECT_EQ(block.get_passes(4, 4), 2U);
  EXPECT_EQ(block.get_hits(4, 4), 2U);
  // passes (2) <= min_pass_through (2) -> cell state MUST still remain NO_INFORMATION (255)
  EXPECT_EQ(block.get(4, 4), 255);
  EXPECT_FALSE(block.has_lethal());

  // Case 3: Single free-space ray pass: 1 pass, 0 hits
  block.record_pass(8, 8, min_pass_through, occupancy_threshold);
  EXPECT_EQ(block.get_passes(8, 8), 1U);
  EXPECT_EQ(block.get_hits(8, 8), 0U);
  EXPECT_EQ(block.get(8, 8), 255);

  // Case 4: Boundary condition for free ray: 2 passes, 0 hits
  block.record_pass(8, 8, min_pass_through, occupancy_threshold);
  EXPECT_EQ(block.get_passes(8, 8), 2U);
  EXPECT_EQ(block.get_hits(8, 8), 0U);
  EXPECT_EQ(block.get(8, 8), 255);
}

TEST(SparseBlockTest, ProbabilisticHitRatioOccupancy)
{
  SparseBlock<16> block(255);
  const uint32_t min_pass_through = 2;
  const double occupancy_threshold = 0.1;

  // Cell (5, 5): 2 hits and 3 free passes -> total 5 passes, 2 hits
  // hitRatio = 2 / 5 = 0.40 > 0.10 -> must become Occupied (cost 254)
  block.record_hit(5, 5, min_pass_through, occupancy_threshold);
  block.record_hit(5, 5, min_pass_through, occupancy_threshold);
  block.record_pass(5, 5, min_pass_through, occupancy_threshold);
  block.record_pass(5, 5, min_pass_through, occupancy_threshold);
  block.record_pass(5, 5, min_pass_through, occupancy_threshold);

  EXPECT_EQ(block.get_passes(5, 5), 5U);
  EXPECT_EQ(block.get_hits(5, 5), 2U);
  EXPECT_EQ(block.get(5, 5), 254);
  EXPECT_TRUE(block.has_lethal());
  EXPECT_FALSE(block.is_uniform());
  EXPECT_FALSE(block.can_prune());

  // Cell (12, 12): Pure free corridor (5 passes, 0 hits)
  // passes (5) > min_pass_through (2), hitRatio = 0 / 5 = 0.0 <= 0.10 -> must become Free (cost 0)
  for (int i = 0; i < 5; ++i) {
    block.record_pass(12, 12, min_pass_through, occupancy_threshold);
  }
  EXPECT_EQ(block.get_passes(12, 12), 5U);
  EXPECT_EQ(block.get_hits(12, 12), 0U);
  EXPECT_EQ(block.get(12, 12), 0);
}

TEST(SparseBlockTest, ThresholdEqualityEvaluatesToFree)
{
  SparseBlock<16> block(255);
  const uint32_t min_pass_through = 2;
  const double occupancy_threshold = 0.10;

  // Case: 10 passes, 1 hit -> hitRatio = 1.0 / 10.0 = 0.10 exactly
  // In Karto SDK: if (hitRatio > occupancyThreshold) -> strict inequality!
  // Since 0.10 > 0.10 is false, the cell MUST evaluate to Free (0), NOT Occupied (254).
  block.record_hit(3, 7, min_pass_through, occupancy_threshold);
  for (int i = 0; i < 9; ++i) {
    block.record_pass(3, 7, min_pass_through, occupancy_threshold);
  }

  EXPECT_EQ(block.get_passes(3, 7), 10U);
  EXPECT_EQ(block.get_hits(3, 7), 1U);
  EXPECT_EQ(block.get(3, 7), 0)
    << "Exact threshold match (0.10 == 0.10) must evaluate to Free (0) to avoid false obstacles";
  EXPECT_FALSE(block.has_lethal());
}

TEST(SparseBlockTest, DynamicObstacleClearanceDilution)
{
  SparseBlock<16> block(255);
  const uint32_t min_pass_through = 2;
  const double occupancy_threshold = 0.10;

  // Phase 1: Obstacle appears at (5, 5). 2 hits + 3 passes = 5 passes, 2 hits (hitRatio = 0.40)
  block.record_hit(5, 5, min_pass_through, occupancy_threshold);
  block.record_hit(5, 5, min_pass_through, occupancy_threshold);
  block.record_pass(5, 5, min_pass_through, occupancy_threshold);
  block.record_pass(5, 5, min_pass_through, occupancy_threshold);
  block.record_pass(5, 5, min_pass_through, occupancy_threshold);

  ASSERT_EQ(block.get_passes(5, 5), 5U);
  ASSERT_EQ(block.get_hits(5, 5), 2U);
  ASSERT_EQ(block.get(5, 5), 254);
  ASSERT_TRUE(block.has_lethal());

  // Phase 2: Dynamic obstacle moves away.
  // 20 subsequent laser scans penetrate the vacated cell with 0 hits.
  for (int i = 0; i < 20; ++i) {
    block.record_pass(5, 5, min_pass_through, occupancy_threshold);
  }

  // Total passes = 5 + 20 = 25, total hits = 2
  // hitRatio = 2 / 25 = 0.08 <= 0.10 -> flips cell from Occupied (254) to Free (0)
  EXPECT_EQ(block.get_passes(5, 5), 25U);
  EXPECT_EQ(block.get_hits(5, 5), 2U);
  EXPECT_EQ(block.get(5, 5), 0)
    <<
    "Subsequent pass-throughs must dilute hit ratio below threshold and clear obstacle to Free (0)";
  EXPECT_FALSE(block.has_lethal());
}

TEST(SparseBlockTest, BlockFillResetsProbabilisticCounters)
{
  SparseBlock<16> block(255);
  const uint32_t min_pass_through = 2;
  const double occupancy_threshold = 0.10;

  // Populate multiple cells with passes and hits
  block.record_hit(0, 0, min_pass_through, occupancy_threshold);
  block.record_hit(0, 0, min_pass_through, occupancy_threshold);
  block.record_hit(0, 0, min_pass_through, occupancy_threshold);
  block.record_pass(7, 7, min_pass_through, occupancy_threshold);
  block.record_hit(15, 15, min_pass_through, occupancy_threshold);

  EXPECT_GT(block.get_passes(0, 0), 0U);
  EXPECT_GT(block.get_hits(0, 0), 0U);
  EXPECT_GT(block.get_passes(7, 7), 0U);
  EXPECT_GT(block.get_passes(15, 15), 0U);

  // Fill entire block with default value 255
  block.fill(255);

  // All cells must have cost 255, and ALL pass and hit counters must be reset to 0
  for (int ly = 0; ly < 16; ++ly) {
    for (int lx = 0; lx < 16; ++lx) {
      EXPECT_EQ(block.get(lx, ly), 255);
      EXPECT_EQ(block.get_passes(lx, ly), 0U)
        << "fill() must reset passes counter to 0 at (" << lx << ", " << ly << ")";
      EXPECT_EQ(block.get_hits(lx, ly), 0U)
        << "fill() must reset hits counter to 0 at (" << lx << ", " << ly << ")";
    }
  }

  EXPECT_TRUE(block.is_uniform());
  EXPECT_TRUE(block.can_prune());
  EXPECT_FALSE(block.has_lethal());
}

TEST(SparseGridTest, ProbabilisticGridOnDemandAllocation)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);
  EXPECT_EQ(grid.allocated_blocks_count(), 0UL);

  // Querying cost before any record call does NOT allocate blocks (zero allocation)
  EXPECT_EQ(grid.get_cost(100, 100), 255);
  EXPECT_EQ(grid.allocated_blocks_count(), 0UL);

  // Record 1 pass in block (cx=20, cy=30) -> bx=1, by=1
  grid.record_pass(20, 30);
  EXPECT_EQ(grid.allocated_blocks_count(), 1UL);
  // Cost remains 255 due to noise filtering (passes=1 <= min_pass_through=2)
  EXPECT_EQ(grid.get_cost(20, 30), 255);

  // Record 3 hits at same cell -> passes=4, hits=3, ratio=0.75 > 0.10 -> Occupied (254)
  grid.record_hit(20, 30);
  grid.record_hit(20, 30);
  grid.record_hit(20, 30);
  EXPECT_EQ(grid.allocated_blocks_count(), 1UL);
  EXPECT_EQ(grid.get_cost(20, 30), 254);
}

TEST(SparseGridTest, ProbabilisticDynamicObstacleClearanceInGrid)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  const int cx = 50;
  const int cy = 50;

  // Step 1: Establish confirmed obstacle (3 hits: passes=3, hits=3, hitRatio=1.0 > 0.10)
  grid.record_hit(cx, cy);
  grid.record_hit(cx, cy);
  grid.record_hit(cx, cy);
  EXPECT_EQ(grid.get_cost(cx, cy), 254);

  // Step 2: Obstacle moves away. 27 subsequent beam passes traverse without hit.
  // Total passes = 3 + 27 = 30. Total hits = 3.
  // hitRatio = 3 / 30 = 0.10 <= occupancy_threshold (0.10) -> flips cell to Free (0)
  for (int i = 0; i < 27; ++i) {
    grid.record_pass(cx, cy);
  }

  EXPECT_EQ(grid.get_cost(cx, cy), 0)
    << "Grid-level dynamic obstacle clearance must flip cell from 254 to 0 upon hit ratio dilution";
}

TEST(SparseGridTest, ProbabilisticParameterConfiguration)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  // Verify default parameters
  EXPECT_EQ(grid.get_min_pass_through(), 2U);
  EXPECT_NEAR(grid.get_occupancy_threshold(), 0.1, 1e-6);

  // Reconfigure parameters: require 5 passes to filter extreme sensor noise, 30% hit ratio
  grid.set_min_pass_through(5);
  grid.set_occupancy_threshold(0.30);
  EXPECT_EQ(grid.get_min_pass_through(), 5U);
  EXPECT_NEAR(grid.get_occupancy_threshold(), 0.30, 1e-6);

  const int cx = 15;
  const int cy = 15;

  // 5 hits recorded -> passes = 5, hits = 5
  // Under min_pass_through = 5: passes (5) <= min_pass_through (5) -> must remain 255!
  for (int i = 0; i < 5; ++i) {
    grid.record_hit(cx, cy);
  }
  EXPECT_EQ(grid.get_cost(cx, cy), 255);

  // 6th hit recorded -> passes = 6 > 5, hits = 6, hitRatio = 1.0 > 0.30 -> becomes 254!
  grid.record_hit(cx, cy);
  EXPECT_EQ(grid.get_cost(cx, cy), 254);
}

TEST(SparseGridTest, NegativeCoordinateProbabilisticTracking)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  // Negative coordinates: cx = -25, cy = -40
  grid.record_hit(-25, -40);
  grid.record_hit(-25, -40);
  grid.record_hit(-25, -40);

  EXPECT_EQ(grid.get_cost(-25, -40), 254);
  EXPECT_EQ(grid.allocated_blocks_count(), 1UL);

  // Dilute obstacle in negative coordinate space
  for (int i = 0; i < 30; ++i) {
    grid.record_pass(-25, -40);
  }
  EXPECT_EQ(grid.get_cost(-25, -40), 0);
}

TEST(SparseGridTest, UpdateAllOccupancyReevaluation)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  // Cell with 10 passes, 2 hits (hitRatio = 0.20)
  grid.record_hit(10, 10);
  grid.record_hit(10, 10);
  for (int i = 0; i < 8; ++i) {
    grid.record_pass(10, 10);
  }

  // Under default threshold 0.10: 0.20 > 0.10 -> Occupied (254)
  EXPECT_EQ(grid.get_cost(10, 10), 254);

  // Reconfigure threshold to 0.25 (stricter occupancy condition)
  grid.set_occupancy_threshold(0.25);

  // Re-evaluate all cells with updated parameter
  grid.update_all_occupancy();

  // Under threshold 0.25: 0.20 <= 0.25 -> cell flips to Free (0)
  EXPECT_EQ(grid.get_cost(10, 10), 0);
}
