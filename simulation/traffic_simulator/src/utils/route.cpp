// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include <geometry/spline/catmull_rom_spline.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator/utils/route.hpp>

namespace traffic_simulator
{
namespace route
{
auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route_lanelets) -> bool
{
  return std::find_if(route_lanelets.begin(), route_lanelets.end(), [lanelet_id](const auto id) {
           return lanelet_id == id;
         }) != route_lanelets.end();
}

auto toSpline(const lanelet::Ids & route_lanelets) -> Spline
{
  return Spline(lanelet_wrapper::lanelet_map::centerPoints(route_lanelets));
}
}  // namespace route
}  // namespace traffic_simulator
