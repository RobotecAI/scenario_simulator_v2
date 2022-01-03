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

#include "random_test_runner/lanelet_tools/phases_generator.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <cmath>
#include <lanelet2_extension_psim/regulatory_elements/autoware_traffic_light.hpp>
#include <unordered_set>

class CollisionBasedTrafficLightsPhaseGenerator : public TrafficLightsPhaseGenerator
{
private:
  bool isRightTurn(const lanelet::ConstLanelet & lanelet)
  {
    const auto & attributes = lanelet.attributes();
    auto turn_directrion_attr_it = attributes.find("turn_direction");
    if (
      turn_directrion_attr_it != attributes.end() &&
      turn_directrion_attr_it->second.value() == "right") {
      return true;
    }
    return false;
  }

public:
  CollisionBasedTrafficLightsPhaseGenerator(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr)
  : TrafficLightsPhaseGenerator(std::move(lanelet_map_ptr), std::move(routing_graph_ptr))
  {
  }

  std::vector<std::set<int64_t>> generate() override
  {
    std::map<int64_t, std::unordered_set<lanelet::ConstLanelet>> traffic_lights_lanelet_map;
    for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
      auto regulatory_elements =
        lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();
      if (isRightTurn(lanelet)) {
        continue;
      }
      for (auto reg_elem : regulatory_elements) {
        for (auto light : reg_elem->trafficLights()) {
          if (traffic_lights_lanelet_map.find(light.id()) == traffic_lights_lanelet_map.end()) {
            traffic_lights_lanelet_map.emplace(
              light.id(), std::unordered_set<lanelet::ConstLanelet>());
          }
          traffic_lights_lanelet_map.at(light.id()).emplace(lanelet);
        }
      }
    }

    std::map<int64_t, std::unordered_set<lanelet::ConstLanelet>>
      traffic_lights_conflicting_lanelet_map;

    for (const auto & traffic_light_and_lanelets : traffic_lights_lanelet_map) {
      const int64_t traffic_light_id = traffic_light_and_lanelets.first;
      const auto & lanelets = traffic_light_and_lanelets.second;

      for (const auto & lanelet : lanelets) {
        if (
          traffic_lights_conflicting_lanelet_map.find(traffic_light_id) ==
          traffic_lights_conflicting_lanelet_map.end()) {
          traffic_lights_conflicting_lanelet_map.emplace(
            traffic_light_id, std::unordered_set<lanelet::ConstLanelet>());
        }
        for (auto conflicting_lanelet : vehicle_routing_graph_ptr_->conflicting(lanelet)) {
          if (
            traffic_light_and_lanelets.second.find(*conflicting_lanelet.lanelet()) !=
              lanelets.end() ||
            (conflicting_lanelet.lanelet() && isRightTurn(conflicting_lanelet.lanelet().value()))) {
            continue;
          }
          traffic_lights_conflicting_lanelet_map.at(traffic_light_and_lanelets.first)
            .emplace(conflicting_lanelet);
        }
        std::cout << std::endl;
      }
    }

    std::map<int64_t, std::unordered_set<int64_t>> traffic_lights_conflicting_traffic_lights_map;

    for (const auto & traffic_light_and_lanelets : traffic_lights_conflicting_lanelet_map) {
      const int64_t traffic_light_id = traffic_light_and_lanelets.first;
      const auto & lanelets = traffic_light_and_lanelets.second;

      traffic_lights_conflicting_traffic_lights_map.emplace(
        traffic_light_id, std::unordered_set<int64_t>());

      for (const auto & lanelet : lanelets) {
        auto regulatory_elements =
          lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();
        for (const auto & reg_elem : regulatory_elements) {
          for (const auto & traffic_lights : reg_elem->trafficLights()) {
            if (traffic_lights.lineString()) {
              traffic_lights_conflicting_traffic_lights_map.at(traffic_light_id)
                .emplace(traffic_lights.lineString()->id());
            }
          }
        }
      }
    }

    std::vector<std::set<int64_t>> phases_buckets;

    for (const auto & traffic_light_and_conflicting_traffic_lights :
         traffic_lights_conflicting_traffic_lights_map) {
      const int64_t traffic_light_id = traffic_light_and_conflicting_traffic_lights.first;
      const auto & conflicting_traffic_lights = traffic_light_and_conflicting_traffic_lights.second;

      bool not_added_light = true;
      for (auto & phases_bucket : phases_buckets) {
        bool found_conflict = false;
        for (int64_t traffic_light_in_phase : phases_bucket) {
          for (int64_t conflicting_light : conflicting_traffic_lights) {
            if (traffic_light_in_phase == conflicting_light) {
              found_conflict = true;
              break;
            }
          }
          if (found_conflict) {
            break;
          }
        }
        if (found_conflict) {
          continue;
        } else {
          phases_bucket.emplace(traffic_light_id);
          not_added_light = false;
          break;
        }
      }
      if (not_added_light) {
        phases_buckets.emplace_back(std::set<int64_t>{traffic_light_id});
      }
    }

    return phases_buckets;
  }
};

class DirectionBasedTrafficLightsPhaseGenerator : public TrafficLightsPhaseGenerator
{
public:
  DirectionBasedTrafficLightsPhaseGenerator(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr)
  : TrafficLightsPhaseGenerator(std::move(lanelet_map_ptr), std::move(routing_graph_ptr))
  {
  }

  std::vector<std::set<int64_t>> generate() override
  {
    std::vector<std::pair<int64_t, double>> traffic_light_id_to_heading;
    for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
      auto regulatory_elements =
        lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();

      for (const auto & regulatory_element : regulatory_elements) {
        boost::optional<lanelet::ConstLineString3d> curr_stop_line = regulatory_element->stopLine();
        if (!curr_stop_line) {
          continue;
        }
        for (const auto & traffic_light : regulatory_element->trafficLights()) {
          traffic_light_id_to_heading.emplace_back(
            traffic_light.id(), compute2dDirectionOfStopLine(curr_stop_line.value()));
        }
      }
    }

    std::sort(
      traffic_light_id_to_heading.begin(), traffic_light_id_to_heading.end(),
      [](const std::pair<int64_t, double> & lhs, const std::pair<int64_t, double> & rhs) {
        return lhs.second < rhs.second;
      });

    double greatest_distance = 0.0;
    double greatest_distance_direction_average = 0.0;
    for (auto it = traffic_light_id_to_heading.begin(); it != traffic_light_id_to_heading.end();
         it++) {
      bool is_last_element = it + 1 == traffic_light_id_to_heading.end();
      auto next_it = is_last_element ? traffic_light_id_to_heading.begin() : it + 1;

      const double direction = it->second;
      const double next_direction = next_it->second;

      double distance;
      if (is_last_element) {
        distance = M_PI - (direction - next_direction);
      } else {
        distance = next_direction - direction;
      }

      if (distance > greatest_distance) {
        greatest_distance = distance;
        greatest_distance_direction_average = direction + distance / 2.0;
        if (greatest_distance_direction_average > M_PI) {
          greatest_distance_direction_average -= M_PI;
        }
      }
    }

    const double threshold = greatest_distance_direction_average;

    std::vector<std::set<int64_t>> phases(2);
    for (const auto & record : traffic_light_id_to_heading) {
      const double direction = record.second;
      const int64_t traffic_light_id = record.first;
      if (
        isWithinTheRangeRightOpen(direction, threshold - M_PI, threshold - M_PI_2) ||
        isWithinTheRangeRightOpen(direction, threshold, threshold + M_PI_2)) {
        phases[0].emplace(traffic_light_id);
      } else {
        phases[1].emplace(traffic_light_id);
      }
    }
    return phases;
  }

private:
  static double compute2dDirectionOfStopLine(const lanelet::ConstLineString3d & line_string)
  {
    if (line_string.size() != 2) {
      throw std::runtime_error(fmt::format(
        "Only two point line strings supported. Line string {} has {}.", line_string.id(),
        line_string.size()));
    }

    double x_diff = line_string.back().x() - line_string.front().x();
    double y_diff = line_string.back().y() - line_string.front().y();

    double direction = std::atan2(x_diff, y_diff);
    if (direction < 0.0) {
      direction += M_PI;
    }
    return direction;
  }

  static bool isWithinTheRangeRightOpen(double value, double min, double max)
  {
    return value >= min && value < max;
  }
};

TrafficLightsPhaseGeneratorPtr makeTrafficLightsPhaseGenerator(
  TrafficLightsGeneratorType generator_type, lanelet::LaneletMapConstPtr lanelet_map_ptr,
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr)
{
  switch (generator_type) {
    case TrafficLightsGeneratorType::LANELET_COLLISION_BASED:
      return std::make_shared<CollisionBasedTrafficLightsPhaseGenerator>(
        std::move(lanelet_map_ptr), std::move(routing_graph_ptr));
    case TrafficLightsGeneratorType::LANELET_STOPLINE_DIRECTION_BASED:
      return std::make_shared<DirectionBasedTrafficLightsPhaseGenerator>(
        std::move(lanelet_map_ptr), std::move(routing_graph_ptr));
  }
  throw std::runtime_error(fmt::format("Failed to creage generator of {} type", generator_type));
}
