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
#include <simple_sensor_simulator/simple_sensor_simulator.hpp>
#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_client.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>
#include <zmqpp/zmqpp.hpp>

#include "../utils/expect_eq_macros.hpp"

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
  return request;
}

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

TEST(ScenarioSimulator, initialize_customPort)
{
  auto server = std::thread([] {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    rclcpp::Parameter port_param("port", rclcpp::ParameterValue(1234));
    options.parameter_overrides().push_back(port_param);
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
  });

  auto multi_client =
    zeromq::MultiClient(simulation_interface::TransportProtocol::TCP, "localhost", 1234U);

  EXPECT_TRUE(multi_client.call(makeInitializeRequest()).result().success());

  rclcpp::shutdown();
  server.join();
}

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

TEST(ScenarioSimulator, updateFrame_noInitialize)
{
  // uhhh theres missing "initialized_(false)," in the constructor
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}