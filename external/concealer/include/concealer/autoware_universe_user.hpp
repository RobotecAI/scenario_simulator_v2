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

#ifndef CONCEALER__AUTOWARE_UNIVERSE_USER_HPP_
#define CONCEALER__AUTOWARE_UNIVERSE_USER_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <concealer/autoware_user.hpp>
#include <concealer/cooperator.hpp>
#include <concealer/dirty_hack.hpp>
#include <concealer/task_queue.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
// TODO #include <tier4_external_api_msgs/srv/initialize_pose.hpp>
#include <concealer/utility/service_with_validation.h>

#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>

#include <concealer/utility/subscriber_wrapper.hpp>
#include <concealer/utility/publisher_wrapper.hpp>

namespace concealer
{
class AutowareUniverseUser : public AutowareUser, public TransitionAssertion<AutowareUniverseUser>
{
  friend class TransitionAssertion<AutowareUniverseUser>;

  using Acceleration = geometry_msgs::msg::AccelWithCovarianceStamped;
  using Checkpoint = geometry_msgs::msg::PoseStamped;
  using ControlModeReport = autoware_auto_vehicle_msgs::msg::ControlModeReport;
  using GearReport = autoware_auto_vehicle_msgs::msg::GearReport;
  using GoalPose = geometry_msgs::msg::PoseStamped;
  using InitialPose = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using TurnIndicatorsReport = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using VelocityReport = autoware_auto_vehicle_msgs::msg::VelocityReport;

  PublisherWrapper<Acceleration> setAcceleration;
  PublisherWrapper<Checkpoint> setCheckpoint;
  PublisherWrapper<ControlModeReport> setControlModeReport;
  PublisherWrapper<GearReport> setGearReport;
  PublisherWrapper<GoalPose> setGoalPose;
  PublisherWrapper<InitialPose> setInitialPose;
  PublisherWrapper<Odometry> setOdometry;
  PublisherWrapper<SteeringReport> setSteeringReport;
  PublisherWrapper<TurnIndicatorsReport> setTurnIndicatorsReport;
  PublisherWrapper<VelocityReport> setVelocityReport;

  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
  using CooperateStatusArray = tier4_rtc_msgs::msg::CooperateStatusArray;
  using EmergencyState = autoware_auto_system_msgs::msg::EmergencyState;
  using GearCommand = autoware_auto_vehicle_msgs::msg::GearCommand;
  using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TurnIndicatorsCommand = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

  SubscriberWrapper<AckermannControlCommand> getAckermannControlCommand;
  SubscriberWrapper<AutowareState> getAutowareState;
  SubscriberWrapper<CooperateStatusArray> getCooperateStatusArray;

  CONCEALER_DEFINE_SUBSCRIPTION(EmergencyState, override);
  CONCEALER_DEFINE_SUBSCRIPTION(GearCommand, override);
  SubscriberWrapper<PathWithLaneId> getPathWithLaneId;
  SubscriberWrapper<Trajectory> getTrajectory;
  CONCEALER_DEFINE_SUBSCRIPTION(TurnIndicatorsCommand, override);

  using CooperateCommands = tier4_rtc_msgs::srv::CooperateCommands;
  using Engage = tier4_external_api_msgs::srv::Engage;
  // TODO using InitializePose = tier4_external_api_msgs::srv::InitializePose;
  using SetVelocityLimit = tier4_external_api_msgs::srv::SetVelocityLimit;

  ServiceWithValidation<CooperateCommands> requestCooperateCommands;
  ServiceWithValidation<Engage> requestEngage;
  // TODO ServiceWithValidation<InitializePose> requestInitializePose;
  ServiceWithValidation<SetVelocityLimit> requestSetVelocityLimit;

private:
  Cooperator current_cooperator = Cooperator::simulator;

  TaskQueue cooperation_queue;

  auto approve(const CooperateStatusArray &) -> void;

  auto cooperate(const CooperateStatusArray &) -> void;

public:
#define DEFINE_STATE_PREDICATE(NAME, VALUE)                  \
  auto is##NAME() const noexcept                             \
  {                                                          \
    using autoware_auto_system_msgs::msg::AutowareState;     \
    return getAutowareState().state == AutowareState::VALUE; \
  }                                                          \
  static_assert(true, "")

  DEFINE_STATE_PREDICATE(Initializing, INITIALIZING);            // 1
  DEFINE_STATE_PREDICATE(WaitingForRoute, WAITING_FOR_ROUTE);    // 2
  DEFINE_STATE_PREDICATE(Planning, PLANNING);                    // 3
  DEFINE_STATE_PREDICATE(WaitingForEngage, WAITING_FOR_ENGAGE);  // 4
  DEFINE_STATE_PREDICATE(Driving, DRIVING);                      // 5
  DEFINE_STATE_PREDICATE(ArrivedGoal, ARRIVED_GOAL);             // 6
  DEFINE_STATE_PREDICATE(Finalizing, FINALIZING);                // 7

#undef DEFINE_STATE_PREDICATE

  template <typename... Ts>
  CONCEALER_PUBLIC explicit AutowareUniverseUser(Ts &&... xs)
  : AutowareUser(std::forward<decltype(xs)>(xs)...),
    // clang-format off
    setAcceleration("/localization/acceleration", *this),
    setCheckpoint("/planning/mission_planning/checkpoint", *this),
    setControlModeReport("/vehicle/status/control_mode", *this),
    setGearReport("/vehicle/status/gear_status", *this),
    setGoalPose("/planning/mission_planning/goal", *this),
    setInitialPose("/initialpose", *this),
    setOdometry("/localization/kinematic_state", *this),
    setSteeringReport("/vehicle/status/steering_status", *this),
    setTurnIndicatorsReport("/vehicle/status/turn_indicators_status", *this),
    setVelocityReport("/vehicle/status/velocity_status", *this),
    getAckermannControlCommand("/control/command/control_cmd", *this),
    getAutowareState("/autoware/state", *this),
    getCooperateStatusArray("/api/external/get/rtc_status", *this, [this](const CooperateStatusArray& v) {cooperate(v);}),
    CONCEALER_INIT_SUBSCRIPTION(EmergencyState, "/system/emergency/emergency_state"),
    CONCEALER_INIT_SUBSCRIPTION(GearCommand, "/control/command/gear_cmd"),
    getPathWithLaneId("/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", *this),
    getTrajectory("/planning/scenario_planning/trajectory", *this),
    CONCEALER_INIT_SUBSCRIPTION(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd"),
    requestCooperateCommands("/api/external/set/rtc_commands", *this),
    requestEngage("/api/external/set/engage", *this),
    // TODO requestInitializePose("/api/autoware/set/initialize_pose", *this),
    requestSetVelocityLimit("/api/autoware/set/velocity_limit", *this)
  // clang-format on
  {
    waitpid_options = 0;
  }

  ~AutowareUniverseUser() override;

  auto engage() -> void override;

  auto engageable() const -> bool override;

  auto engaged() const -> bool override;

  auto getAutowareStateName() const -> std::string override;

  auto getGearSign() const -> double override;

  auto getSteeringAngle() const -> double override;

  auto getVehicleCommand() const -> std::tuple<
    autoware_auto_control_msgs::msg::AckermannControlCommand,
    autoware_auto_vehicle_msgs::msg::GearCommand> override;

  auto getVelocity() const -> double override;

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray override;

  auto initialize(const geometry_msgs::msg::Pose &) -> void override;

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void override;

  auto restrictTargetSpeed(double) const -> double override;

  auto sendSIGINT() -> void override;

  auto setCooperator(const std::string & cooperator) -> void override
  {
    current_cooperator = boost::lexical_cast<Cooperator>(cooperator);
  }

  auto setVelocityLimit(double) -> void override;

  auto update() -> void override;
};
}  // namespace concealer

// for boost::lexical_cast
namespace autoware_auto_system_msgs::msg
{
auto operator<<(std::ostream &, const EmergencyState &) -> std::ostream &;

auto operator>>(std::istream &, EmergencyState &) -> std::istream &;
}  // namespace autoware_auto_system_msgs::msg

namespace autoware_auto_vehicle_msgs::msg
{
auto operator<<(std::ostream &, const TurnIndicatorsCommand &) -> std::ostream &;

auto operator>>(std::istream &, TurnIndicatorsCommand &) -> std::istream &;
}  // namespace autoware_auto_vehicle_msgs::msg

#endif  // CONCEALER__AUTOWARE_UNIVERSE_USER_HPP_