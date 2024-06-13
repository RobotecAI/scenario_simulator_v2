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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__CONFIG__VOXEL_GRID_OCCLUSION_ESTIMATION_PARAMETERS_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__CONFIG__VOXEL_GRID_OCCLUSION_ESTIMATION_PARAMETERS_HPP_

namespace simple_sensor_simulator
{
struct VoxelGridOcclusionEstimationParameters
{
  static constexpr float leaf_size_{0.08};
  static constexpr int num_clouds_per_target_{10};
  static constexpr float resolution_slope_{0.086f};    // Slope of the linear equation
  static constexpr float resolution_intercept_{0.8f};  // Intercept of the linear equation
  static constexpr int point_cloud_size_threshold_{1000};

  static constexpr auto getLeafSize() -> float { return leaf_size_; }
  static constexpr auto getNumCloudsPerTarget() -> int { return num_clouds_per_target_; }
  static constexpr auto getResolutionSlope() -> float { return resolution_slope_; }
  static constexpr auto getResolutionIntercept() -> float { return resolution_intercept_; }
  static constexpr auto getPointCloudSizeThreshold() -> int { return point_cloud_size_threshold_; }
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__CONFIG__VOXEL_GRID_OCCLUSION_ESTIMATION_PARAMETERS_HPP_