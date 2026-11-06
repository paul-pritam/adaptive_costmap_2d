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

using namespace adaptive_costmap_2d;

TEST(SparseBlockTest, DefaultStateAndBitmask)
{
  SparseBlock<16> block(0);
  EXPECT_TRUE(block.is_uniform());
  EXPECT_FALSE(block.has_lethal());
  EXPECT_EQ(block.get(0, 0), 0);
  EXPECT_EQ(block.get(15, 15), 0);

  block.set(5, 5, 254);
  EXPECT_FALSE(block.is_uniform());
  EXPECT_TRUE(block.has_lethal());
  EXPECT_EQ(block.get(5, 5), 254);

  // Reset back to zero
  block.set(5, 5, 0);
  EXPECT_TRUE(block.is_uniform());
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
  // Simulate a 100m x 100m warehouse grid at 0.05m resolution
  // Total cells = 2,000 x 2,000 = 4,000,000 cells (Dense = 4,000,000 bytes = 3.81 MB)
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 0);

  // Populate 4 warehouse aisles with shelving (typical 5% non-zero obstacle layout)
  for (int x = 100; x < 1900; x += 100) {
    for (int y = 200; y < 1800; ++y) {
      grid.set_cost(x, y, 254);
    }
  }

  const size_t dense_bytes = grid.equivalent_dense_bytes(0, 0, 1999, 1999);
  const size_t sparse_bytes = grid.memory_bytes_used();
  const double savings = grid.compression_ratio(0, 0, 1999, 1999);

  EXPECT_EQ(dense_bytes, 4000000UL);
  EXPECT_LT(sparse_bytes, dense_bytes / 4); // Must be under 25% of dense footprint
  EXPECT_GT(savings, 0.75); // Greater than 75% memory savings
}

TEST(BlockVisualizerTest, CreatesWireframesAndTelemetryHUD)
{
  SparseGrid2D<16> grid(0.05, 0.0, 0.0, 255);

  // Set lethal obstacle in block (0, 0)
  grid.set_cost(5, 5, 254);
  // Set free space in block (1, 0)
  grid.set_cost(20, 5, 0);

  auto markers = BlockVisualizer::create_markers(grid, "map", rclcpp::Time(0), 100, 100);

  // Expect 3 markers: wireframes (LINE_LIST), fills (CUBE_LIST), and hud (TEXT_VIEW_FACING)
  ASSERT_EQ(markers.markers.size(), 3UL);

  const auto & wireframes = markers.markers[0];
  EXPECT_EQ(wireframes.type, visualization_msgs::msg::Marker::LINE_LIST);
  // 2 blocks allocated * 8 points per block wireframe = 16 points
  EXPECT_EQ(wireframes.points.size(), 16UL);

  const auto & lethal = markers.markers[1];
  EXPECT_EQ(lethal.type, visualization_msgs::msg::Marker::POINTS);
  EXPECT_EQ(lethal.points.size(), 1UL); // Exactly 1 lethal point set at (5, 5)

  const auto & hud = markers.markers[2];
  EXPECT_EQ(hud.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_FALSE(hud.text.empty());
  EXPECT_NE(hud.text.find("ADAPTIVE BLOCK-SPARSE 2D MAP"), std::string::npos);
}
