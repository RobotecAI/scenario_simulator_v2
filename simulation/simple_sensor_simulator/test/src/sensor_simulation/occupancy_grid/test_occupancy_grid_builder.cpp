// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid_traversal.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_builder.hpp>

auto makePose(const double x, const double y) -> geometry_msgs::msg::Pose
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(0.0))
    .orientation(
      geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0));
}

auto makeBox(const float size, const double x, const double y)
  -> simple_sensor_simulator::primitives::Box
{
  return simple_sensor_simulator::primitives::Box(size, size, size, makePose(x, y));
}

/**
 * @note Test function behavior when called after the limit has been reached.
 * The number of elements added is equal to int16_t max.
 */
TEST(OccupancyGridBuilder, add_overLimit)
{
  auto builder = simple_sensor_simulator::OccupancyGridBuilder(
    0.1, static_cast<size_t>(20), static_cast<size_t>(40));
  for (int i = 0; i < static_cast<int>(std::numeric_limits<std::int16_t>::max()); ++i) {
    builder.add(makeBox(5.0f, 0.0, 0.0));
  }
  EXPECT_THROW(builder.add(makeBox(5.0f, 0.0, 0.0)), std::runtime_error);
}

/**
 * @note Test basic functionality. Test adding a primitive with a sample primitive (e.g. box)
 * positioned fully inside the grid, but not in the center.
 */
TEST(OccupancyGridBuilder, add_primitiveInside)
{
  const auto result_expected = std::vector<int8_t>{
    // clang-format off
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 2, 2, 2, 2, 2, 2, 1, 1, 1, 
    1, 1, 1, 1, 2, 2, 2, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 1, 1, 1, 
    1, 1, 1, 1, 2, 2, 2, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 1, 1, 1, 
    1, 1, 1, 1, 2, 2, 2, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 1, 1, 1, 
    1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
    // clang-format on
  };

  auto builder = simple_sensor_simulator::OccupancyGridBuilder(
    1.0, static_cast<size_t>(10), static_cast<size_t>(20), static_cast<int8_t>(2),
    static_cast<int8_t>(1));
  builder.add(makeBox(5.0f, 4.0, 0.0));
  builder.add(makeBox(2.0f, -5.0, 0.0));

  builder.build();

  EXPECT_EQ(result_expected, builder.get());
}

/**
 * @note Test basic functionality. Test adding a primitive with a sample primitive (e.g. box)
 * positioned in such a way that it protrudes from the grid area.
 */
TEST(OccupancyGridBuilder, add_primitiveProtruding)
{
  const auto result_expected = std::vector<int8_t>{
    // clang-format off
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // clang-format on
  };

  auto builder = simple_sensor_simulator::OccupancyGridBuilder(
    1.0, static_cast<size_t>(10), static_cast<size_t>(20), static_cast<int8_t>(2),
    static_cast<int8_t>(1));
  builder.add(makeBox(5.0f, 9.0, -4.0));
  builder.add(makeBox(2.0f, -10.0, 3.0));

  builder.build();

  EXPECT_EQ(result_expected, builder.get());
}

/**
 * @note Test basic functionality. Test adding a primitive with a sample primitive (e.g. box)
 * covering the center of the grid - the goal is to get whole grid as invisible.
 */
TEST(OccupancyGridBuilder, add_primitiveInCenter)
{
  const auto result_expected = std::vector<int8_t>{
    // clang-format off
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    // clang-format on
    /* This is how the algorithm's output looks, zeros on the right are unexpected.
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,     
    */
  };

  auto builder = simple_sensor_simulator::OccupancyGridBuilder(
    1.0, static_cast<size_t>(10), static_cast<size_t>(20), static_cast<int8_t>(2),
    static_cast<int8_t>(1));
  builder.add(makeBox(5.0f, 0.0, 0.0));

  builder.build();

  EXPECT_EQ(result_expected, builder.get());
}

/**
 * @note Test basic functionality. Test adding a primitive with a sample primitive (e.g. box)
 * covering the center of the grid - the goal is to get whole grid as invisible.
 */
TEST(OccupancyGridBuilder, reset)
{
  const auto result_expected = std::vector<int8_t>{
    // clang-format off
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // clang-format on
  };

  auto builder = simple_sensor_simulator::OccupancyGridBuilder(
    1.0, static_cast<size_t>(10), static_cast<size_t>(20), static_cast<int8_t>(2),
    static_cast<int8_t>(1));
  builder.add(makeBox(5.0f, 0.0, 0.0));
  builder.reset(makePose(0.0, 0.0));

  builder.build();

  EXPECT_EQ(result_expected, builder.get());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
