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

  using Polyline_ptr = std::shared_ptr<traffic_simulator::follow_trajectory::Parameter<
    traffic_simulator::follow_trajectory::Polyline>>;

  Polyline_ptr trajectory_parameter_m;
  traffic_simulator_msgs::msg::EntityStatus entity_status_m;
  traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter_m;
  std::optional<traffic_simulator_msgs::msg::VehicleParameters> vehicle_parameters_m;
  double step_time_m;

  std::optional<traffic_simulator_msgs::msg::EntityStatus> followTrajectory(Polyline_ptr &);

  auto getCurrentPosition() -> geometry_msgs::msg::Point;
  auto getTargetPosition() -> geometry_msgs::msg::Point;
  auto getCurrentAcceleration() -> double;
  auto getAccelerationLimits(double acceleration) -> std::tuple<double, double>;
  auto getCurrentSpeed() -> double;
  auto getTimeRemainingToFrontWaypoint(
    double remaining_time_to_front_waypoint, double distance_to_front_waypoint,
    double desired_speed) -> std::optional<double>;
  auto createUpdatedEntityStatus(geometry_msgs::msg::Vector3 velocity)
    -> traffic_simulator_msgs::msg::EntityStatus;
  auto setParameters(
    const traffic_simulator_msgs::msg::EntityStatus &,
    const traffic_simulator_msgs::msg::BehaviorParameter &, double,
    std::optional<traffic_simulator_msgs::msg::VehicleParameters> vehicle_parameters = std::nullopt)
    -> void;

  virtual auto getUpdatedVelocity(
    geometry_msgs::msg::Vector3 desired_velocity, double desired_speed)
    -> geometry_msgs::msg::Vector3 = 0;
  virtual auto getDesiredAcceleration(
    double remaining_time, double acceleration, double distance, double speed) -> double = 0;
  virtual auto getDesiredSpeed(
    double desired_acceleration, double min_acceleration, double max_acceleration, double speed)
    -> double = 0;
  virtual auto getDesiredVelocity(
    const geometry_msgs::msg::Point target_position, const geometry_msgs::msg::Point position,
    double desired_speed) -> geometry_msgs::msg::Vector3 = 0;
  virtual auto getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point target_position, const geometry_msgs::msg::Point position)
    -> std::optional<std::tuple<double, double>> = 0;
  virtual auto getDistanceAndTimeToWaypointWithSpecifiedTime(double distance_to_front_waypoint)
    -> std::optional<std::tuple<double, double>> = 0;

private:
  void discardTheFrontWaypointFromTrajectory();
};

class PositionModePolylineTrajectoryFollower : public PolylineTrajectoryFollower
{
public:
  PositionModePolylineTrajectoryFollower() : PolylineTrajectoryFollower(){};
  virtual auto getUpdatedVelocity(
    geometry_msgs::msg::Vector3 desired_velocity, double desired_speed)
    -> geometry_msgs::msg::Vector3 override;
  virtual auto getDesiredAcceleration(
    double remaining_time, double acceleration, double distance, double speed) -> double override;
  virtual auto getDesiredSpeed(
    double desired_acceleration, double min_acceleration, double max_acceleration, double speed)
    -> double override;
  virtual auto getDesiredVelocity(
    const geometry_msgs::msg::Point target_position, const geometry_msgs::msg::Point position,
    double desired_speed) -> geometry_msgs::msg::Vector3 override;
  virtual auto getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point target_position, const geometry_msgs::msg::Point position)
    -> std::optional<std::tuple<double, double>> override;
  virtual auto getDistanceAndTimeToWaypointWithSpecifiedTime(double distance_to_front_waypoint)
    -> std::optional<std::tuple<double, double>> override;
};

}  // namespace vehicle
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__VEHICLE__TRAJECTORY_FOLLOWER_HPP_
