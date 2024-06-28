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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/simple_sensor_simulator.hpp>
#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_client.hpp>

#include "utils/helper_functions.hpp"

auto makeInitializeRequest() -> simulation_api_schema::InitializeRequest
{
  auto request = simulation_api_schema::InitializeRequest();
  request.set_lanelet2_map_path(
    ament_index_cpp::get_package_share_directory("simple_sensor_simulator") +
    "/map/lanelet2_map.osm");
  return request;
}

auto makeUpdateFrameRequest() -> simulation_api_schema::UpdateFrameRequest
{
  auto request = simulation_api_schema::UpdateFrameRequest();
  request.set_current_scenario_time(1.0);
  request.set_current_simulation_time(1.0);
  return request;
}

auto makeDespawnEntityRequest(const std::string name = "name")
  -> simulation_api_schema::DespawnEntityRequest
{
  auto request = simulation_api_schema::DespawnEntityRequest();
  request.set_name(name);
  return request;
}

auto makeSpawnVehicleEntityRequest(const std::string name = "name", const bool is_ego = false)
  -> simulation_api_schema::SpawnVehicleEntityRequest
{
  auto request = simulation_api_schema::SpawnVehicleEntityRequest();
  const auto params = traffic_simulator_msgs::msg::VehicleParameters{};
  simulation_interface::toProto(params, *request.mutable_parameters());
  request.mutable_parameters()->set_name(name);
  request.set_is_ego(is_ego);
  return request;
}

auto makeSpawnPedestrianEntityRequest(const std::string name = "name")
  -> simulation_api_schema::SpawnPedestrianEntityRequest
{
  auto request = simulation_api_schema::SpawnPedestrianEntityRequest();
  const auto params = traffic_simulator_msgs::msg::PedestrianParameters{};
  simulation_interface::toProto(params, *request.mutable_parameters());
  request.mutable_parameters()->set_name(name);
  return request;
}

auto makeSpawnMiscObjectEntityRequest(const std::string name = "name")
  -> simulation_api_schema::SpawnMiscObjectEntityRequest
{
  auto request = simulation_api_schema::SpawnMiscObjectEntityRequest();
  const auto params = traffic_simulator_msgs::msg::MiscObjectParameters{};
  simulation_interface::toProto(params, *request.mutable_parameters());
  request.mutable_parameters()->set_name(name);
  return request;
}

auto makeAttachDetectionSensorRequest() -> simulation_api_schema::AttachDetectionSensorRequest
{
  auto request = simulation_api_schema::AttachDetectionSensorRequest();
  *request.mutable_configuration() =
    utils::constructDetectionSensorConfiguration("entity_name", "awf/universe", 1.0);
  return request;
}

auto makeAttachLidarSensorRequest() -> simulation_api_schema::AttachLidarSensorRequest
{
  auto request = simulation_api_schema::AttachLidarSensorRequest();
  *request.mutable_configuration() =
    utils::constructLidarConfiguration("entity_name", "awf/universe", 0.0, 1.0 / 180.0 * M_PI);
  return request;
}

auto makeAttachOccupancyGridSensorRequest()
  -> simulation_api_schema::AttachOccupancyGridSensorRequest
{
  auto request = simulation_api_schema::AttachOccupancyGridSensorRequest();
  *request.mutable_configuration() =
    utils::constructOccupancyGridSensorConfiguration("entity_name", "awf/universe", 1.0);
  return request;
}

class ScenarioSimulatorTest : public testing::Test
{
protected:
  ScenarioSimulatorTest()
  : server([] {
      rclcpp::init(0, nullptr);
      rclcpp::NodeOptions options;
      auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
      rclcpp::spin(component);
    }),
    client(simulation_interface::TransportProtocol::TCP, "localhost", 5555U)
  {
  }
  std::thread server;
  zeromq::MultiClient client;
  ~ScenarioSimulatorTest()
  {
    rclcpp::shutdown();
    server.join();
  }
};