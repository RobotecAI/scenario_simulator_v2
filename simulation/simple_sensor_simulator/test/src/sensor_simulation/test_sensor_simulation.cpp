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

#include <simple_sensor_simulator/sensor_simulation/sensor_simulation.hpp>

class SensorSimulationTest : public testing::Test
{
protected:
  SensorSimulationTest() : node(rclcpp::Node{"name", rclcpp::NodeOptions{}}) {}
  rclcpp::Node node;
};

/**
 * @note Test function behavior when called with a configuration architecture type other than
 * "awf/universe" - the goal is to test error throwing.
 */
TEST_F(SensorSimulationTest, attachLidarSensor_wrongArchitecture)
{
  auto configuration = simulation_api_schema::LidarConfiguration{};
  configuration.set_architecture_type("wrong/architecture");

  EXPECT_THROW(
    simple_sensor_simulator::SensorSimulation().attachLidarSensor(0.0, configuration, node),
    std::runtime_error);
}

/**
 * @note Test basic functionality. Test attaching a lidar sensor correctness with a sample lidar configuration.
 */
TEST_F(SensorSimulationTest, attachLidarSensor_correctConfiguration)
{
  auto configuration = simulation_api_schema::LidarConfiguration{};
  configuration.set_architecture_type("awf/universe");

  EXPECT_NO_THROW(
    simple_sensor_simulator::SensorSimulation().attachLidarSensor(0.0, configuration, node));
}

/**
 * @note Test function behavior when called with a configuration architecture type other than
 * "awf/universe" - the goal is to test error throwing.
 */
TEST_F(SensorSimulationTest, attachDetectionSensor_wrongArchitecture)
{
  auto configuration = simulation_api_schema::DetectionSensorConfiguration{};
  configuration.set_architecture_type("wrong/architecture");

  EXPECT_THROW(
    simple_sensor_simulator::SensorSimulation().attachDetectionSensor(0.0, configuration, node),
    std::runtime_error);
}

/**
 * @note Test basic functionality. Test attaching a detection sensor correctness
 * with a sample detection sensor configuration.
 */
TEST_F(SensorSimulationTest, attachDetectionSensor_correctConfiguration)
{
  auto configuration = simulation_api_schema::DetectionSensorConfiguration{};
  configuration.set_architecture_type("awf/universe");

  EXPECT_NO_THROW(
    simple_sensor_simulator::SensorSimulation().attachDetectionSensor(0.0, configuration, node));
}

/**
 * @note Test function behavior when called with a configuration architecture type other than
 * "awf/universe" - the goal is to test error throwing.
 */
TEST_F(SensorSimulationTest, attachOccupancyGridSensor_wrongArchitecture)
{
  auto configuration = simulation_api_schema::OccupancyGridSensorConfiguration{};
  configuration.set_architecture_type("wrong/architecture");

  EXPECT_THROW(
    simple_sensor_simulator::SensorSimulation().attachOccupancyGridSensor(0.0, configuration, node),
    std::runtime_error);
}

/**
 * @note Test basic functionality. Test attaching a occupancy grid sensor correctness
 * with a sample occupancy grid sensor configuration.
 */
TEST_F(SensorSimulationTest, attachOccupancyGridSensor_correctConfiguration)
{
  auto configuration = simulation_api_schema::OccupancyGridSensorConfiguration{};
  configuration.set_architecture_type("awf/universe");

  EXPECT_NO_THROW(simple_sensor_simulator::SensorSimulation().attachOccupancyGridSensor(
    0.0, configuration, node));
}

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
