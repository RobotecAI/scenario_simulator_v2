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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST__UTILS__HELPER_FUNCTIONS_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST__UTILS__HELPER_FUNCTIONS_HPP_

#include <simulation_api_schema.pb.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace utils
{

constexpr auto degToRad(const double deg) -> double { return deg * M_PI / 180.0; }

inline auto makePoint(const double px, const double py, const double pz)
  -> geometry_msgs::msg::Point
{
  return geometry_msgs::build<geometry_msgs::msg::Point>().x(px).y(py).z(pz);
}

inline auto makePose(
  const double px, const double py, const double pz, const double ox, const double oy,
  const double oz, const double ow) -> geometry_msgs::msg::Pose
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(px).y(py).z(pz))
    .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(ox).y(oy).z(oz).w(ow));
}

inline auto makeDimensions(const double x, const double y, const double z)
  -> geometry_msgs::msg::Vector3
{
  return geometry_msgs::build<geometry_msgs::msg::Vector3>().x(x).y(y).z(z);
}

inline auto constructLidarConfiguration(
  const std::string & entity, const std::string & architecture_type,
  const double lidar_sensor_delay, const double horizontal_resolution)
  -> const simulation_api_schema::LidarConfiguration
{
  simulation_api_schema::LidarConfiguration configuration;
  configuration.set_horizontal_resolution(horizontal_resolution);
  configuration.set_architecture_type(architecture_type);
  configuration.set_entity(entity);
  configuration.set_lidar_sensor_delay(lidar_sensor_delay);

  configuration.set_scan_duration(0.1);
  configuration.add_vertical_angles(degToRad(-15.0));
  configuration.add_vertical_angles(degToRad(-13.0));
  configuration.add_vertical_angles(degToRad(-11.0));
  configuration.add_vertical_angles(degToRad(-9.0));
  configuration.add_vertical_angles(degToRad(-7.0));
  configuration.add_vertical_angles(degToRad(-5.0));
  configuration.add_vertical_angles(degToRad(-3.0));
  configuration.add_vertical_angles(degToRad(-1.0));
  configuration.add_vertical_angles(degToRad(1.0));
  configuration.add_vertical_angles(degToRad(3.0));
  configuration.add_vertical_angles(degToRad(5.0));
  configuration.add_vertical_angles(degToRad(7.0));
  configuration.add_vertical_angles(degToRad(9.0));
  configuration.add_vertical_angles(degToRad(11.0));
  configuration.add_vertical_angles(degToRad(13.0));
  configuration.add_vertical_angles(degToRad(15.0));
  return configuration;
}

}  // namespace utils

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__UTILS__HELPER_FUNCTIONS_HPP_
