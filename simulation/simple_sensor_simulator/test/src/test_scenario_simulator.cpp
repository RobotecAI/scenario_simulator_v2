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
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/simple_sensor_simulator.hpp>
#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_client.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>
#include <zmqpp/zmqpp.hpp>

#include "sensor_simulation/expect_eq_macros.hpp"

auto makeInitializeRequest() -> simulation_api_schema::InitializeRequest
{
  auto request = simulation_api_schema::InitializeRequest();
  request.set_lanelet2_map_path(
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm");
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
  auto configuration = traffic_simulator::helper::constructDetectionSensorConfiguration(
    "entity_name", "awf/universe", 1.0);
  *request.mutable_configuration() = configuration;
  return request;
}

auto makeAttachLidarSensorRequest() -> simulation_api_schema::AttachLidarSensorRequest
{
  auto request = simulation_api_schema::AttachLidarSensorRequest();
  auto configuration = traffic_simulator::helper::constructLidarConfiguration(
    traffic_simulator::helper::LidarType::VLP16, "entity_name", "awf/universe", 1.0);
  *request.mutable_configuration() = configuration;
  return request;
}

auto makeAttachOccupancyGridSensorRequest()
  -> simulation_api_schema::AttachOccupancyGridSensorRequest
{
  auto request = simulation_api_schema::AttachOccupancyGridSensorRequest();
  auto configuration = simulation_api_schema::OccupancyGridSensorConfiguration();

  configuration.set_entity("entity_name");
  configuration.set_architecture_type("awf/universe");
  configuration.set_update_duration(1.0);
  configuration.set_range(300.0);
  configuration.set_resolution(1.0);
  *request.mutable_configuration() = configuration;
  return request;
}

/**
 * @note Test initialization correctness with a sample request with the default port (5555).
 */
TEST(ScenarioSimulator, initialize_defaultPort)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test initialization correctness with a sample request with a custom port.
 */
TEST(ScenarioSimulator, initialize_customPort)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides().push_back(
      rclcpp::Parameter{"port", rclcpp::ParameterValue(1234)});
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 1234U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test updating frame correctness with a sample request.
*/
TEST(ScenarioSimulator, updateFrame_correct)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeUpdateFrameRequest()).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test updating frame correctness with a sample request before requesting initialization.
 */
TEST(ScenarioSimulator, updateFrame_noInitialize)
{
  // undefined behaviour !!! there is missing "initialized_(false)," in the ScenarioSimulator constructor
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_FALSE(multi_client.call(makeUpdateFrameRequest()).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test spawning vehicle entity correctness with a request to spawn
 * Ego vehicle when Ego vehicle is not spawned yet.
 */
TEST(ScenarioSimulator, spawnVehicleEntity_firstEgo)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeSpawnVehicleEntityRequest("ego", true)).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test spawning vehicle entity correctness with a request to spawn a NPC vehicle.
 */
TEST(ScenarioSimulator, spawnVehicleEntity_npc)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeSpawnVehicleEntityRequest("npc", false)).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test spawning pedestrian entity correctness with a request to spawn a NPC pedestrian.
 */
TEST(ScenarioSimulator, spawnPedestrianEntity)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeSpawnPedestrianEntityRequest("bob")).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test spawning misc object entity correctness with a request to spawn a misc object.
 */
TEST(ScenarioSimulator, spawnMiscObjectEntity)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeSpawnMiscObjectEntityRequest("blob")).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test despawning an entity with a request to despawn an existing vehicle.
 */
TEST(ScenarioSimulator, despawnEntity_vehicle)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeSpawnVehicleEntityRequest("npc", false)).result().success());
  EXPECT_TRUE(multi_client.call(makeDespawnEntityRequest("npc")).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test despawning an entity with a request to despawn an existing pedestrian.
 */
TEST(ScenarioSimulator, despawnEntity_pedestrian)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeSpawnPedestrianEntityRequest("bob")).result().success());
  EXPECT_TRUE(multi_client.call(makeDespawnEntityRequest("bob")).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test despawning an entity with a request to despawn an existing misc object.
 */
TEST(ScenarioSimulator, despawnEntity_miscObject)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeSpawnMiscObjectEntityRequest("blob")).result().success());
  EXPECT_TRUE(multi_client.call(makeDespawnEntityRequest("blob")).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test despawning an entity with a request to despawn an entity that does not exist.
 */
TEST(ScenarioSimulator, despawnEntity_invalidName)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_FALSE(multi_client.call(makeDespawnEntityRequest("invalid")).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test attaching detection sensor with a request to attach a sensor with a valid configuration.
 */
TEST(ScenarioSimulator, attachDetectionSensor)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeAttachDetectionSensorRequest()).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test attaching lidar sensor with a request to attach a sensor with a valid configuration.
 */
TEST(ScenarioSimulator, attachLidarSensor)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeAttachLidarSensorRequest()).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test attaching occupancy grid sensor with a request to attach a sensor with a valid configuration.
 */
TEST(ScenarioSimulator, attachOccupancyGridSensor)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(multi_client.call(makeAttachOccupancyGridSensorRequest()).result().success());

  rclcpp::shutdown();
  server.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}