// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "random_test_runner/data_types.hpp"

TrafficLightsGeneratorType trafficLightsGeneratorTypeFromString(
  const std::string & generator_type_str)
{
  if (generator_type_str == "collision_based") {
    return TrafficLightsGeneratorType::COLLISION_BASED;
  } else if (generator_type_str == "direction_based") {
    return TrafficLightsGeneratorType::DIRECTION_BASED;
  }
  throw std::runtime_error(fmt::format(
    "Failed to convert {} to traffic lights phases generator type", generator_type_str));
}

std::string trafficLightsGeneratorTypeToString(TrafficLightsGeneratorType generator_type)
{
  switch (generator_type) {
    case TrafficLightsGeneratorType::COLLISION_BASED:
      return "collision_based";
    case TrafficLightsGeneratorType::DIRECTION_BASED:
      return "direction_based";
  }
  throw std::runtime_error(fmt::format(
    "Failed to convert traffic lights phases generator type {} to string", generator_type));
}
