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

#ifndef RANDOM_TEST_RUNNER__PHASES_GENERATOR_H
#define RANDOM_TEST_RUNNER__PHASES_GENERATOR_H

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>

#include <set>

#include "random_test_runner/data_types.hpp"

class TrafficLightsPhaseGenerator
{
public:
  TrafficLightsPhaseGenerator(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr)
  : lanelet_map_ptr_(std::move(lanelet_map_ptr)),
    vehicle_routing_graph_ptr_(std::move(routing_graph_ptr))
  {
  }
  virtual std::vector<std::set<int64_t>> generate() = 0;

protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
};

using TrafficLightsPhaseGeneratorPtr = std::shared_ptr<TrafficLightsPhaseGenerator>;

TrafficLightsPhaseGeneratorPtr makeTrafficLightsPhaseGenerator(
  TrafficLightsGeneratorType generator_type, lanelet::LaneletMapConstPtr lanelet_map_ptr,
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr);

#endif  //RANDOM_TEST_RUNNER__PHASES_GENERATOR_H
