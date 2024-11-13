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

#include <algorithm>
#include <memory>
#include <string>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
DynamicEntity::DynamicEntity(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  const traffic_simulator_msgs::msg::PedestrianParameters & parameters,
  const std::string & plugin_name)
: EntityBase(name, entity_status, hdmap_utils_ptr),
  plugin_name(plugin_name),
  pedestrian_parameters(parameters),
  loader_(pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase>(
    "traffic_simulator", "entity_behavior::BehaviorPluginBase")),
  behavior_plugin_ptr_(loader_.createSharedInstance(plugin_name)),
  route_planner_(hdmap_utils_ptr_)
{
  behavior_plugin_ptr_->configure(rclcpp::get_logger(name));
  behavior_plugin_ptr_->setPedestrianParameters(parameters);
  behavior_plugin_ptr_->setDebugMarker({});
  behavior_plugin_ptr_->setBehaviorParameter(traffic_simulator_msgs::msg::BehaviorParameter());
  behavior_plugin_ptr_->setHdMapUtils(hdmap_utils_ptr_);
  behavior_plugin_ptr_->setDefaultMatchingDistanceForLaneletPoseCalculation(
    getDefaultMatchingDistanceForLaneletPoseCalculation());
}

void DynamicEntity::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array)
{
  const auto marker = behavior_plugin_ptr_->getDebugMarker();
  std::copy(marker.begin(), marker.end(), std::back_inserter(marker_array.markers));
}

void DynamicEntity::cancelRequest()
{
  behavior_plugin_ptr_->setRequest(behavior::Request::NONE);
  route_planner_.cancelRoute();
}

auto DynamicEntity::getCurrentAction() const -> std::string
{
  return behavior_plugin_ptr_->getCurrentAction();
}

auto DynamicEntity::getDefaultDynamicConstraints() const
  -> const traffic_simulator_msgs::msg::DynamicConstraints &
{
  static auto default_dynamic_constraints = traffic_simulator_msgs::msg::DynamicConstraints();
  default_dynamic_constraints.max_acceleration = 1.0;
  default_dynamic_constraints.max_acceleration_rate = 1.0;
  default_dynamic_constraints.max_deceleration = 1.0;
  default_dynamic_constraints.max_deceleration_rate = 1.0;
  return default_dynamic_constraints;
}

auto DynamicEntity::getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter
{
  return behavior_plugin_ptr_->getBehaviorParameter();
}

auto DynamicEntity::getEntityTypename() const -> const std::string &
{
  static const std::string result = "DynamicEntity";
  return result;
}

auto DynamicEntity::getGoalPoses() -> std::vector<CanonicalizedLaneletPose>
{
  return route_planner_.getGoalPoses();
}

auto DynamicEntity::getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return std::nullopt;
}

auto DynamicEntity::getRouteLanelets(double horizon) -> lanelet::Ids
{
  if (const auto canonicalized_lanelet_pose = status_->getCanonicalizedLaneletPose()) {
    return route_planner_.getRouteLanelets(canonicalized_lanelet_pose.value(), horizon);
  } else {
    return {};
  }
}

auto DynamicEntity::getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray
{
  return traffic_simulator_msgs::msg::WaypointsArray();
}

void DynamicEntity::requestAcquirePosition(const CanonicalizedLaneletPose & lanelet_pose)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  if (status_->laneMatchingSucceed()) {
    route_planner_.setWaypoints({lanelet_pose});
  }
  behavior_plugin_ptr_->setGoalPoses({static_cast<geometry_msgs::msg::Pose>(lanelet_pose)});
}

void DynamicEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  if (
    const auto canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
      map_pose, status_->getBoundingBox(), true,
      getDefaultMatchingDistanceForLaneletPoseCalculation(), hdmap_utils_ptr_)) {
    requestAcquirePosition(canonicalized_lanelet_pose.value());
  } else {
    THROW_SEMANTIC_ERROR("Goal of the entity should be on lane.");
  }
}

void DynamicEntity::requestAssignRoute(const std::vector<CanonicalizedLaneletPose> & waypoints)
{
  if (!laneMatchingSucceed()) {
    return;
  }
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  route_planner_.setWaypoints(waypoints);
  std::vector<geometry_msgs::msg::Pose> goal_poses;
  for (const auto & waypoint : waypoints) {
    goal_poses.emplace_back(static_cast<geometry_msgs::msg::Pose>(waypoint));
  }
  behavior_plugin_ptr_->setGoalPoses(goal_poses);
}

void DynamicEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<CanonicalizedLaneletPose> route;
  for (const auto & waypoint : waypoints) {
    if (
      const auto canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
        waypoint, status_->getBoundingBox(), true,
        getDefaultMatchingDistanceForLaneletPoseCalculation(), hdmap_utils_ptr_)) {
      route.emplace_back(canonicalized_lanelet_pose.value());
    } else {
      THROW_SEMANTIC_ERROR("Waypoint of the entity should be on lane.");
    }
  }
  requestAssignRoute(route);
}

auto DynamicEntity::requestFollowTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & parameter) -> void
{
  behavior_plugin_ptr_->setPolylineTrajectory(parameter);
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_POLYLINE_TRAJECTORY);
}










void DynamicEntity::requestWalkStraight()
{
  behavior_plugin_ptr_->setRequest(behavior::Request::WALK_STRAIGHT);
}

void DynamicEntity::setBehaviorParameter(
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter)
{
  behavior_plugin_ptr_->setBehaviorParameter(behavior_parameter);
}

auto DynamicEntity::getMaxAcceleration() const -> double
{
  return getBehaviorParameter().dynamic_constraints.max_acceleration;
}

auto DynamicEntity::getMaxDeceleration() const -> double
{
  return getBehaviorParameter().dynamic_constraints.max_deceleration;
}

void DynamicEntity::setVelocityLimit(double linear_velocity)
{
  if (linear_velocity < 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_speed = linear_velocity;
  setBehaviorParameter(behavior_parameter);
}

void DynamicEntity::setAccelerationLimit(double acceleration)
{
  if (acceleration < 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration = acceleration;
  setBehaviorParameter(behavior_parameter);
}

void DynamicEntity::setAccelerationRateLimit(double acceleration_rate)
{
  if (acceleration_rate < 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration rate limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration_rate = acceleration_rate;
  setBehaviorParameter(behavior_parameter);
}

void DynamicEntity::setDecelerationLimit(double deceleration)
{
  if (deceleration < 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_deceleration = deceleration;
  setBehaviorParameter(behavior_parameter);
}

void DynamicEntity::setDecelerationRateLimit(double deceleration_rate)
{
  if (deceleration_rate < 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration rate limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_deceleration_rate = deceleration_rate;
  setBehaviorParameter(behavior_parameter);
}

void DynamicEntity::setTrafficLights(
  const std::shared_ptr<traffic_simulator::TrafficLightsBase> & ptr)
{
  EntityBase::setTrafficLights(ptr);
  behavior_plugin_ptr_->setTrafficLights(traffic_lights_);
}

auto DynamicEntity::onUpdate(const double current_time, const double step_time) -> void
{
  EntityBase::onUpdate(current_time, step_time);

  behavior_plugin_ptr_->setOtherEntityStatus(other_status_);
  behavior_plugin_ptr_->setCanonicalizedEntityStatus(status_);
  behavior_plugin_ptr_->setTargetSpeed(target_speed_);
  behavior_plugin_ptr_->setRouteLanelets(getRouteLanelets());
  /// @note CanonicalizedEntityStatus is updated here, it is not skipped even if isAtEndOfLanelets return true
  behavior_plugin_ptr_->update(current_time, step_time);
  if (const auto canonicalized_lanelet_pose = status_->getCanonicalizedLaneletPose()) {
    if (pose::isAtEndOfLanelets(canonicalized_lanelet_pose.value(), hdmap_utils_ptr_)) {
      stopAtCurrentPosition();
      return;
    }
  }
  EntityBase::onPostUpdate(current_time, step_time);
}
}  // namespace entity
}  // namespace traffic_simulator
