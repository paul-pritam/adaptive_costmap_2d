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

#include <gtest/gtest.h>
#include "adaptive_costmap_2d/sparse_block.hpp"
#include "adaptive_costmap_2d/sparse_grid_2d.hpp"
#include "adaptive_costmap_2d/block_visualizer.hpp"
#include "adaptive_costmap_2d/karto_sparse_grid.hpp"

using namespace adaptive_costmap_2d;

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
    grid.set_cost(cx, cy, 0); // clear ray
    ++visit_count;
  });

  EXPECT_EQ(visit_count, 33); // 0 to 32 inclusive
  for (int x = 0; x <= 32; ++x) {
    EXPECT_EQ(grid.get_cost(x, 0), 0);
  }
}

TEST(SparseGridTest, BoundedMemorySavingsOver80Percent)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 0);

  for (int x = 100; x < 1900; x += 100) {
    for (int y = 200; y < 1800; ++y) {
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

  unsigned int mx, my;
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

  grid.set_cost(0, 0, 0);     // Free (0)
  grid.set_cost(10, 10, 254); // Lethal (100 in ROS)

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
  EXPECT_EQ(grid.get_cost(2, 2), 0); // Decayed to free
  EXPECT_EQ(grid.get_cost(34, 34), 254); // Persisted
}

TEST(SparseGridTest, DynamicBlockPruningReclaimsMemory)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  // Allocate block (0, 0)
  grid.set_cost(5, 5, 254);
  EXPECT_EQ(grid.allocated_blocks_count(), 1UL);

  // Reset all cells in block (0, 0) back to default 255
  grid.set_cost(5, 5, 255);
  EXPECT_EQ(grid.allocated_blocks_count(), 1UL); // Still in hash table until pruned

  // Trigger dynamic pruning
  size_t pruned = grid.prune_unmodified_blocks();
  EXPECT_EQ(pruned, 1UL);
  EXPECT_EQ(grid.allocated_blocks_count(), 0UL); // Reclaimed!
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
  EXPECT_EQ(patch[0], 100); // (10, 10)
  EXPECT_EQ(patch[1], 200); // (11, 10)
  EXPECT_EQ(patch[2], 50);  // (10, 11)
  EXPECT_EQ(patch[3], 255); // (11, 11) default
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
  EXPECT_EQ(ros_map.data[0], 0); // Free space
  EXPECT_EQ(ros_map.data[10 * 11 + 10], 100); // Occupied obstacle
}
