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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__CONFIG__LIDAR_LIDAR_PARAMETERS_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__CONFIG__LIDAR_LIDAR_PARAMETERS_HPP_

#include <simulation_api_schema.pb.h>

namespace simple_sensor_simulator
{
class LidarParameters
{
public:
  LidarParameters(float h_res, float v_res, float h_fov, float v_fov, float max_r, float min_r)
  : horizontal_resolution_(h_res),
    vertical_resolution_(v_res),
    horizontal_fov_(h_fov),
    vertical_fov_(v_fov),
    max_range_(max_r),
    min_range_(min_r)
  {
  }

  LidarParameters() = default;

  auto updateFromConfig(const simulation_api_schema::LidarConfiguration & configuration)
  {
    if (configuration.horizontal_resolution() > 0) {
      horizontal_resolution_ = configuration.horizontal_resolution(); // 1 [deg]
    }
  }

  auto getHorizontalResolution() const -> float { return horizontal_resolution_; }
  auto getVerticalResolution() const -> float { return vertical_resolution_; }
  auto getHorizontalFOV() const -> float { return horizontal_fov_; }
  auto getVerticalFOV() const -> float { return vertical_fov_; }
  auto getMaxRange() const -> float { return max_range_; }
  auto getMinRange() const -> float { return min_range_; }

private:
  float horizontal_resolution_ = 0.4f;  // [deg]
  float vertical_resolution_ = 2.0f;    // [deg]
  float horizontal_fov_ = 360.0f;       // [deg]
  float vertical_fov_ = 15.0f;          // [deg]
  float max_range_ = 100.0f;            // [m]
  float min_range_ = 0.0f;              // [m]
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__CONFIG__LIDAR_LIDAR_PARAMETERS_HPP_