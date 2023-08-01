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

#include <behavior_tree_plugin/vehicle/follow_trajectory_sequence/follow_polyline_trajectory_action.hpp>
#include <type_traits>
#include <typeinfo>
namespace entity_behavior
{
namespace vehicle
{
auto FollowPolylineTrajectoryAction::calculateWaypoints()
  -> const traffic_simulator_msgs::msg::WaypointsArray
{
  auto waypoints = traffic_simulator_msgs::msg::WaypointsArray();
  waypoints.waypoints.push_back(entity_status.pose.position);
  for (const auto & vertex : trajectory_parameter->shape.vertices) {
    waypoints.waypoints.push_back(vertex.position.position);
  }
  return waypoints;
}

auto FollowPolylineTrajectoryAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
  -> const std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  /**
     Obstacle avoidance is not implemented for this action.

     @todo If you implement obstacle avoidance for this action, implement this
     virtual function to return the location of any obstacles blocking the path
     of this action's actor. However, this virtual function is currently used
     only for the visualization of obstacle information, so the obstacle
     avoidance algorithm does not necessarily need to use this virtual
     function.
  */
  return std::nullopt;
}

template <class T>
std::string
type_name()
{
    typedef typename std::remove_reference<T>::type TR;
    std::unique_ptr<char, void(*)(void*)> own
           (
#ifndef _MSC_VER
                abi::__cxa_demangle(typeid(TR).name(), nullptr,
                                           nullptr, nullptr),
#else
                nullptr,
#endif
                std::free
           );
    std::string r = own != nullptr ? own.get() : typeid(TR).name();
    if (std::is_const<TR>::value)
        r += " const";
    if (std::is_volatile<TR>::value)
        r += " volatile";
    if (std::is_lvalue_reference<T>::value)
        r += "&";
    else if (std::is_rvalue_reference<T>::value)
        r += "&&";
    return r;
}

auto FollowPolylineTrajectoryAction::providedPorts() -> BT::PortsList
{
  auto ports = VehicleActionNode::providedPorts();
  ports.emplace(BT::InputPort<decltype(trajectory_parameter)>("polyline_trajectory_parameter"));
  ports.emplace(BT::InputPort<decltype(target_speed)>("target_speed"));
  return ports;
}

auto FollowPolylineTrajectoryAction::tick() -> BT::NodeStatus
{
  if (!trajectory_follower)
  {
    trajectory_follower = std::make_unique<PositionModePolylineTrajectoryFollower>();
  }

  if (getBlackBoardValues();
      request != traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY or
      not getInput<decltype(trajectory_parameter)>("polyline_trajectory_parameter", trajectory_parameter) or
      not getInput<decltype(target_speed)>("target_speed", target_speed) or
      not trajectory_parameter) {
    return BT::NodeStatus::FAILURE;
  } else if (

    const auto updated_status =
      trajectory_follower->followTrajectory(entity_status, trajectory_parameter, behavior_parameter, step_time)) {
      // makeUpdatedStatus(entity_status, trajectory_parameter, behavior_parameter, step_time)) {
    setOutput("updated_status", *updated_status);
    setOutput("waypoints", calculateWaypoints());
    setOutput("obstacle", calculateObstacle(calculateWaypoints()));
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}
}  // namespace vehicle
}  // namespace entity_behavior
