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
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>
#include <zmqpp/zmqpp.hpp>

#include "../utils/expect_eq_macros.hpp"

class Client
{
public:
  Client()
  : context(), socket(this->context, zmqpp::socket_type::request), address{"tcp://localhost:5555"}
  {
    socket.connect(address);
  }
  auto send(zmqpp::message & msg) -> void { socket.send(msg); }
  auto recv(zmqpp::message & msg) -> void { socket.receive(msg); }

private:
  zmqpp::context context;
  zmqpp::socket socket;
  std::string address;
};

int main(int argc, char ** argv)
{
  auto server = std::thread([&argc, &argv] {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<simple_sensor_simulator::ScenarioSimulator>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
  });

  auto client = Client();

  auto clock = traffic_simulator::SimulationClock(true, 1.0, 10.0);
  auto request = simulation_api_schema::InitializeRequest();
  request.set_realtime_factor(1.0);
  request.set_step_time(0.1);
  request.set_initialize_time(0.0);
  simulation_interface::toProto(clock.getCurrentRosTime(), *request.mutable_initialize_ros_time());
  request.set_lanelet2_map_path(
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm");

  zmqpp::message req_message = zeromq::toZMQ(request);
  std::cout << "REQ\n" << request.DebugString() << std::endl;
  client.send(req_message);

  zmqpp::message rep_message;
  client.recv(rep_message);

  auto response = zeromq::toProto<simulation_api_schema::InitializeResponse>(rep_message);
  std::cout << "REP\n" << response.DebugString() << std::endl;
  if (response.result().success()) {
    std::cout << "worked lol\n";
  } else {
    std::cout << ":(\n" << response.result().DebugString() << std::endl;
  }

  {
    zmqpp::message my_final_message;
    my_final_message << "destroy";
    client.send(my_final_message);
  }
  server.join();
  return 0;
}