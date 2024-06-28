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

#include "test_scenario_simulator.hpp"

#include "utils/expect_eq_macros.hpp"

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

  auto client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 5555U);

  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());

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

  auto client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 1234U);

  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());

  rclcpp::shutdown();
  server.join();
}

/**
 * @note Test updating frame correctness with a sample request.
*/
TEST_F(ScenarioSimulatorTest, updateFrame_correct)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeUpdateFrameRequest()).result().success());
}

/**
 * @note Test updating frame correctness with a sample request before requesting initialization.
 */
TEST_F(ScenarioSimulatorTest, updateFrame_noInitialize)
{
  // UB is entered during this test. ScenarioSimulator constructor is missing "initialized_(false),"
  EXPECT_FALSE(client.call(makeUpdateFrameRequest()).result().success());
}

/**
 * @note Test spawning vehicle entity correctness with a request to spawn
 * Ego vehicle when Ego vehicle is not spawned yet.
 */
TEST_F(ScenarioSimulatorTest, spawnVehicleEntity_firstEgo)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeSpawnVehicleEntityRequest("ego", true)).result().success());
}

/**
 * @note Test spawning vehicle entity correctness with a request to spawn a NPC vehicle.
 */
TEST_F(ScenarioSimulatorTest, spawnVehicleEntity_npc)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeSpawnVehicleEntityRequest("npc", false)).result().success());
}

/**
 * @note Test spawning pedestrian entity correctness with a request to spawn a NPC pedestrian.
 */
TEST_F(ScenarioSimulatorTest, spawnPedestrianEntity)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeSpawnPedestrianEntityRequest("bob")).result().success());
}

/**
 * @note Test spawning misc object entity correctness with a request to spawn a misc object.
 */
TEST_F(ScenarioSimulatorTest, spawnMiscObjectEntity)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeSpawnMiscObjectEntityRequest("blob")).result().success());
}

/**
 * @note Test despawning an entity with a request to despawn an existing vehicle.
 */
TEST_F(ScenarioSimulatorTest, despawnEntity_vehicle)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeSpawnVehicleEntityRequest("npc", false)).result().success());
  EXPECT_TRUE(client.call(makeDespawnEntityRequest("npc")).result().success());
}

/**
 * @note Test despawning an entity with a request to despawn an existing pedestrian.
 */
TEST_F(ScenarioSimulatorTest, despawnEntity_pedestrian)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeSpawnPedestrianEntityRequest("bob")).result().success());
  EXPECT_TRUE(client.call(makeDespawnEntityRequest("bob")).result().success());
}

/**
 * @note Test despawning an entity with a request to despawn an existing misc object.
 */
TEST_F(ScenarioSimulatorTest, despawnEntity_miscObject)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeSpawnMiscObjectEntityRequest("blob")).result().success());
  EXPECT_TRUE(client.call(makeDespawnEntityRequest("blob")).result().success());
}

/**
 * @note Test despawning an entity with a request to despawn an entity that does not exist.
 */
TEST_F(ScenarioSimulatorTest, despawnEntity_invalidName)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_FALSE(client.call(makeDespawnEntityRequest("invalid")).result().success());
}

/**
 * @note Test attaching detection sensor with a request to attach a sensor with a valid configuration.
 */
TEST_F(ScenarioSimulatorTest, attachDetectionSensor)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeAttachDetectionSensorRequest()).result().success());
}

/**
 * @note Test attaching lidar sensor with a request to attach a sensor with a valid configuration.
 */
TEST_F(ScenarioSimulatorTest, attachLidarSensor)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeAttachLidarSensorRequest()).result().success());
}

/**
 * @note Test attaching occupancy grid sensor with a request to attach a sensor with a valid configuration.
 */
TEST_F(ScenarioSimulatorTest, attachOccupancyGridSensor)
{
  EXPECT_TRUE(client.call(makeInitializeRequest()).result().success());
  EXPECT_TRUE(client.call(makeAttachOccupancyGridSensorRequest()).result().success());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
