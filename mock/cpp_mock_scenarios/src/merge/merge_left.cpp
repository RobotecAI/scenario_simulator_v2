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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <vector>

namespace cpp_mock_scenarios
{
class MergeLeftScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit MergeLeftScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "merge_left", ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  bool requested = false;
  void onUpdate() override
  {
    if (api_.getEntity("ego").isInLanelet(34510, 0.1)) {
      stop(cpp_mock_scenarios::Result::SUCCESS);
    }
    // LCOV_EXCL_START
    if (api_.getCurrentTime() >= 10.0) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    if (api_.checkCollision("ego", "npc")) {
      stop(cpp_mock_scenarios::Result::FAILURE);
    }
    // LCOV_EXCL_STOP
  }
  void onInitialize() override
  {
    auto & ego_entity = api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(34462, 15.0, 0.0),
      getVehicleParameters());
    ego_entity.setLinearVelocity(5);
    ego_entity.requestSpeedChange(5, true);
    ego_entity.requestLaneChange(34513);

    auto & npc_entity = api_.spawn(
      "npc", traffic_simulator::helper::constructCanonicalizedLaneletPose(34513, 0.0, 0.0),
      getVehicleParameters());
    npc_entity.setLinearVelocity(10);
  }
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::MergeLeftScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
