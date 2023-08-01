#ifndef BEHAVIOR_TREE_PLUGIN__VEHICLE__TRAJECTORY_FOLLOWER_HPP_
#define BEHAVIOR_TREE_PLUGIN__VEHICLE__TRAJECTORY_FOLLOWER_HPP_

#include <behavior_tree_plugin/vehicle/vehicle_action_node.hpp>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/data_type/follow_trajectory.hpp>

namespace entity_behavior
{
namespace vehicle
{

class PolylineTrajectoryFollower
{
public:
  PolylineTrajectoryFollower() = default;
  ~PolylineTrajectoryFollower() = default;

  using PolylineTrajectoryParameter_ptr =
    std::shared_ptr<traffic_simulator::follow_trajectory::Parameter<
      traffic_simulator::follow_trajectory::Polyline>>;

  PolylineTrajectoryParameter_ptr trajectory_parameter_m;
  traffic_simulator_msgs::msg::EntityStatus entity_status_m;
  traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter_m;
  std::optional<traffic_simulator_msgs::msg::VehicleParameters> vehicle_parameters_m;
  double step_time_m;

  std::optional<traffic_simulator_msgs::msg::EntityStatus> followTrajectory(
    const traffic_simulator_msgs::msg::EntityStatus &, PolylineTrajectoryParameter_ptr &,
    const traffic_simulator_msgs::msg::BehaviorParameter &, double,
    std::optional<traffic_simulator_msgs::msg::VehicleParameters> vehicle_parameters =
      std::nullopt);

  geometry_msgs::msg::Point getCurrentPosition();
  geometry_msgs::msg::Point getTargetPosition();
  double getCurrentAcceleration();
  std::tuple<double, double> getAccelerationLimits(double acceleration);
  double getCurrentSpeed();
  std::optional<double> getTimeRemainingToFrontWaypoint(
    double remaining_time_to_front_waypoint, double distance_to_front_waypoint,
    double desired_speed);
  traffic_simulator_msgs::msg::EntityStatus createUpdatedEntityStatus(
    geometry_msgs::msg::Vector3 velocity);

  geometry_msgs::msg::Vector3 virtual getUpdatedVelocity(
    geometry_msgs::msg::Vector3 desired_velocity, double desired_speed) = 0;
  double virtual getDesiredAcceleration(
    double remaining_time, double acceleration, double distance, double speed) = 0;
  double virtual getDesiredSpeed(
    double desired_acceleration, double min_acceleration, double max_acceleration,
    double speed) = 0;
  geometry_msgs::msg::Vector3 virtual getDesiredVelocity(
    const geometry_msgs::msg::Point target_position, const geometry_msgs::msg::Point position,
    double desired_speed) = 0;
  std::optional<std::tuple<double, double>> virtual getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point target_position, const geometry_msgs::msg::Point position) = 0;
  std::optional<std::tuple<double, double>> virtual getDistanceAndTimeToWaypointWithSpecifiedTime(
    double distance_to_front_waypoint) = 0;

private:
  void discardTheFrontWaypoint();
};

class PositionModePolylineTrajectoryFollower : public PolylineTrajectoryFollower
{
public:
  PositionModePolylineTrajectoryFollower() : PolylineTrajectoryFollower(){};
  geometry_msgs::msg::Vector3 virtual getUpdatedVelocity(
    geometry_msgs::msg::Vector3 desired_velocity, double desired_speed) override;
  double virtual getDesiredAcceleration(
    double remaining_time, double acceleration, double distance, double speed) override;
  double virtual getDesiredSpeed(
    double desired_acceleration, double min_acceleration, double max_acceleration,
    double speed) override;
  geometry_msgs::msg::Vector3 virtual getDesiredVelocity(
    const geometry_msgs::msg::Point target_position, const geometry_msgs::msg::Point position,
    double desired_speed) override;
  std::optional<std::tuple<double, double>> virtual getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point target_position,
    const geometry_msgs::msg::Point position) override;
  std::optional<std::tuple<double, double>> virtual getDistanceAndTimeToWaypointWithSpecifiedTime(
    double distance_to_front_waypoint) override;
};

// class FollowModePolylineTrajectoryFollower : public PolylineTrajectoryFollower {
// public:
//   FollowModePolylineTrajectoryFollower(): PolylineTrajectoryFollower() {};
//   geometry_msgs::msg::Vector3 virtual getUpdatedVelocity(geometry_msgs::msg::Vector3 desired_velocity, double desired_speed) override;
//   double virtual getDesiredAcceleration(double remaining_time, double acceleration, double distance, double speed) override;
//   double virtual getDesiredSpeed(double desired_acceleration, double min_acceleration, double max_acceleration, double speed) override;
//   geometry_msgs::msg::Vector3 virtual getDesiredVelocity(const geometry_msgs::msg::Point target_position, const geometry_msgs::msg::Point position, double desired_speed) override;
//   std::optional<std::tuple<double, double>> virtual getDistanceAndTimeToFrontWaypoint(const geometry_msgs::msg::Point target_position, const geometry_msgs::msg::Point position) override;
//   std::optional<std::tuple<double, double>> virtual getDistanceAndTimeToWaypointWithSpecifiedTime(double distance_to_front_waypoint) override;
// };

}  // namespace vehicle
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__VEHICLE__TRAJECTORY_FOLLOWER_HPP_
