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

#ifndef BEHAVIOR_TREE_PLUGIN__VEHICLE__FOLLOW_LANE_SEQUENCE__STOP_AT_STOP_LINE_ACTION_HPP_
#define BEHAVIOR_TREE_PLUGIN__VEHICLE__FOLLOW_LANE_SEQUENCE__STOP_AT_STOP_LINE_ACTION_HPP_

#include <behavior_tree_plugin/vehicle/vehicle_action_node.hpp>
#include <optional>
#include <string>
#include <traffic_simulator/entity/entity_base.hpp>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
class StopAtStopLineAction : public entity_behavior::VehicleActionNode
{
public:
  StopAtStopLineAction(const std::string & name, const BT::NodeConfiguration & config);
  bool checkPreconditions() override;
  BT::NodeStatus doAction() override;
  static BT::PortsList providedPorts()
  {
    return entity_behavior::VehicleActionNode::providedPorts();
  }
  std::optional<double> calculateTargetSpeed(double current_velocity);
  const traffic_simulator_msgs::msg::WaypointsArray calculateWaypoints() override;
  const std::optional<traffic_simulator_msgs::msg::Obstacle> calculateObstacle(
    const traffic_simulator_msgs::msg::WaypointsArray & waypoints) override;

private:
  std::optional<double> distance_to_stopline_;
  static constexpr double waypoint_interval = 1.0;
  static constexpr double bounding_box_half_factor = 0.5;
  static constexpr double stop_margin = 1.0;
  static constexpr double front_stopline_margin = 5.0;
  static constexpr double velocity_epsilon = 0.001;
};
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__VEHICLE__FOLLOW_LANE_SEQUENCE__STOP_AT_STOP_LINE_ACTION_HPP_
