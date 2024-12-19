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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_TRAJECTORY__POLYLINE_TRAJECTORY_POSITIONER_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_TRAJECTORY__POLYLINE_TRAJECTORY_POSITIONER_HPP_

#include <optional>
#include <traffic_simulator/behavior/follow_trajectory/follow_waypoint_controller.hpp>
#include <traffic_simulator/behavior/follow_trajectory/validated_entity_status.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

struct PolylineTrajectoryPositioner
{
public:
  explicit PolylineTrajectoryPositioner(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
    const ValidatedEntityStatus & validated_entity_status,
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const std::optional<double> target_speed, const double matching_distance,
    const double step_time);

  auto makeUpdatedEntityStatus() const -> std::optional<EntityStatus>;

private:
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
  const ValidatedEntityStatus validated_entity_status;
  const traffic_simulator_msgs::msg::PolylineTrajectory polyline_trajectory;
  const double step_time;
  const double matching_distance;

  const std::vector<traffic_simulator_msgs::msg::Vertex>::const_iterator
    nearest_waypoint_with_specified_time_it;
  const geometry_msgs::msg::Point nearest_waypoint_position;
  const double distance_to_nearest_waypoint;
  const double total_remaining_distance;
  const double time_to_nearest_waypoint;
  const double total_remaining_time;

  const FollowWaypointController follow_waypoint_controller;

  auto totalRemainingDistance(
    const double matching_distance,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) const -> double;
  auto totalRemainingTime() const noexcept(false) -> double;
  auto isNearestWaypointWithSpecifiedTimeSameAsLastWaypoint() const -> bool;
  auto nearestWaypointWithSpecifiedTimeIterator() const
    -> std::vector<traffic_simulator_msgs::msg::Vertex>::const_iterator;
  auto validatedEntityTargetPosition() const noexcept(false) -> geometry_msgs::msg::Point;
  auto validatedEntityDesiredAcceleration() const noexcept(false) -> double;
  auto validatedEntityDesiredVelocity(const double desired_speed) const noexcept(false)
    -> geometry_msgs::msg::Vector3;
  auto validatedEntityDesiredSpeed(const double desired_acceleration) const noexcept(false)
    -> double;
  auto validatePredictedState(const double desired_acceleration) const noexcept(false) -> void;
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_TRAJECTORY__POLYLINE_TRAJECTORY_POSITIONER_HPP_
