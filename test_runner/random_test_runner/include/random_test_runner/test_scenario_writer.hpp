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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#ifndef RANDOM_TEST_RUNNER__TEST_SCENARIO_WRITER_HPP
#define RANDOM_TEST_RUNNER__TEST_SCENARIO_WRITER_HPP

#include <memory>
#include <rclcpp/logger.hpp>

#include "random_test_runner/data_types.hpp"
#include "random_test_runner/file_interactions/junit_xml_reporter.hpp"
#include "random_test_runner/metrics/almost_standstill_metric.hpp"
#include "random_test_runner/metrics/ego_collision_metric.hpp"
#include "random_test_runner/metrics/goal_reached_metric.hpp"
#include "traffic_simulator/api/api.hpp"

template <typename traffic_simulator_api_type>
class TestScenarioWriter
{
public:
  TestScenarioWriter(
      std::shared_ptr<traffic_simulator_api_type>, TestDescription description,
      JunitXmlReporterTestCase, double test_timeout,
      ArchitectureType, rclcpp::Logger logger, std::string out_dir, int test_id)
      : test_description_(std::move(description)),
        test_timeout_(test_timeout),
        out_dir_(std::move(out_dir)),
        test_id_(test_id),
        logger_(logger)
  {
  }

  void initialize()
  {
    std::string filename = generateFilename();
    RCLCPP_INFO_STREAM(logger_, fmt::format("Generating scenario: {}", filename));
    std::ofstream ss(filename);
    addHeader(ss);
//    addEgoDef(ss);
    for (const NPCDescription& d : test_description_.npcs_descriptions) {
      addNPCDef(ss, d);
    }
    initializeStoryboard(ss);
//    addEgoInitActions(ss, test_description_.ego_start_position, test_description_.ego_goal_position);
    for (const NPCDescription& d : test_description_.npcs_descriptions) {
      addNPCInitActions(ss, d);
    }
//    addStoryHeaderReachPositionConditionAndTimeoutCondition(ss, test_description_.ego_goal_position);
    addStoryHeaderAndTimeoutCondition(ss);
//    for (const NPCDescription& d : test_description_.npcs_descriptions) {
//      addCollisionCondition(ss, ego_name_, d.name);
//    }

    for (std::size_t i = 0; i + 1 < test_description_.npcs_descriptions.size(); i++) {
      for (std::size_t j = i + 1; j < test_description_.npcs_descriptions.size(); j++) {
        addCollisionCondition(ss, test_description_.npcs_descriptions[i].name, test_description_.npcs_descriptions[j].name);
      }
    }
    closeTheStory(ss);
  }

  auto update() -> void
  {
      scenario_completed_ = true;
  }

  void deinitialize() {}

  bool scenarioCompleted() { return scenario_completed_; }

private:

  std::string generateFilename(){
    std::stringstream ss;
    ss << out_dir_ << "/" << test_id_ << ".yaml";
    return ss.str();
  }

  traffic_simulator_msgs::msg::VehicleParameters getVehicleParameters()
  {
    traffic_simulator_msgs::msg::VehicleParameters parameters;
    parameters.name = "vehicle.volkswagen.t";
    parameters.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
    parameters.performance.max_speed = 69.444;
    parameters.performance.max_acceleration = 200;
    parameters.performance.max_deceleration = 10.0;
    parameters.bounding_box.center.x = 1.5;
    parameters.bounding_box.center.y = 0.0;
    parameters.bounding_box.center.z = 0.9;
    parameters.bounding_box.dimensions.x = 2.1;
    parameters.bounding_box.dimensions.y = 4.5;
    parameters.bounding_box.dimensions.z = 1.8;
    parameters.axles.front_axle.max_steering = 0.5;
    parameters.axles.front_axle.wheel_diameter = 0.6;
    parameters.axles.front_axle.track_width = 1.8;
    parameters.axles.front_axle.position_x = 3.1;
    parameters.axles.front_axle.position_z = 0.3;
    parameters.axles.rear_axle.max_steering = 0.0;
    parameters.axles.rear_axle.wheel_diameter = 0.6;
    parameters.axles.rear_axle.track_width = 1.8;
    parameters.axles.rear_axle.position_x = 0.0;
    parameters.axles.rear_axle.position_z = 0.3;
    return parameters;
  }


  void addHeader(std::ostream & ss) {
    ss << "ScenarioModifiers:\n"
          "  ScenarioModifier: []\n"
          "OpenSCENARIO:\n"
          "  FileHeader:\n"
          "    revMajor: 1\n"
          "    revMinor: 1\n"
          "    date: '2023-11-20T12:12:39.915Z'\n"
          "    description: ''\n"
          "    author: HAL9000\n"
          "  CatalogLocations:\n"
          "    CatalogLocation: []\n"
          "  RoadNetwork:\n"
          "    LogicFile:\n"
          "      filepath: " << map_path << "\n"
          "    TrafficSignals:\n"
          "      TrafficSignalController: []\n"
          "  Entities:\n"
          "    ScenarioObject:\n";
  }

  void addEgoDef(std::ostream & ss) {
    ss << "      - name: " << ego_name_ << "\n";
    addVehicleParameters(ss, getVehicleParameters());
    ss << "        ObjectController:\n"
          "          Controller:\n";
    if (run_autoware_) {
      ss << "            name: 'Autoware'\n"
            "            Properties:\n"
            "              Property: \n"
            "                - name: isEgo\n"
            "                  value: 'true'\n";
    } else {
      ss << "            name: ''\n"
            "            Properties:\n"
            "              Property: []\n";
    }
  }

  void addNPCDef(std::ostream & ss, const NPCDescription& d) {
    ss << "      - name: " << d.name << "\n";
    addVehicleParameters(ss, getVehicleParameters());
  }

  void addVehicleParameters(std::ostream & ss, const traffic_simulator_msgs::msg::VehicleParameters& parameters) {
    ss << "        Vehicle:\n"
          "          - name: Taxi\n"
          "            mass: 2000\n"
          "            vehicleCategory: car\n"
          "            BoundingBox:\n"
          "              Center:\n"
          "                x: " << parameters.bounding_box.center.x << "\n"
          "                y: " << parameters.bounding_box.center.y << "\n"
          "                z: " << parameters.bounding_box.center.z << "\n"
          "              Dimensions:\n"
          "                width: " << parameters.bounding_box.dimensions.x << "\n"
          "                length: " << parameters.bounding_box.dimensions.y << "\n"
          "                height: " << parameters.bounding_box.dimensions.z << "\n"
          "            Performance:\n"
          "              maxSpeed: " << parameters.performance.max_speed << "\n"
          "              maxAcceleration: " << parameters.performance.max_acceleration << "\n"
          "              maxDeceleration: " << parameters.performance.max_deceleration << "\n"
          "            Axles:\n"
          "              FrontAxle:\n"
          "                maxSteering: " << parameters.axles.front_axle.max_steering << "\n"
          "                wheelDiameter: " << parameters.axles.front_axle.wheel_diameter << "\n"
          "                trackWidth: " << parameters.axles.front_axle.track_width << "\n"
          "                positionX: " << parameters.axles.front_axle.position_x << "\n"
          "                positionZ: " << parameters.axles.front_axle.position_z << "\n"
          "              RearAxle:\n"
          "                maxSteering: " << parameters.axles.rear_axle.max_steering << "\n"
          "                wheelDiameter: " << parameters.axles.rear_axle.wheel_diameter << "\n"
          "                trackWidth: " << parameters.axles.rear_axle.track_width << "\n"
          "                positionX: " << parameters.axles.rear_axle.position_x << "\n"
          "                positionZ: " << parameters.axles.rear_axle.position_z << "\n"
          "            Properties:\n"
          "              Property: []\n";
  }

  void initializeStoryboard(std::ostream & ss) {
    ss << "  Storyboard:\n"
          "    Init:\n"
          "      Actions:\n"
          "        Private:\n";
  }

  void addNPCInitActions(std::ostream & ss, const NPCDescription& d) {
    ss << "          - entityRef: " << d.name << "\n"
          "            PrivateAction:\n"
          "              - TeleportAction:\n"
          "                  Position:\n"
          "                    LanePosition:\n"
          "                      roadId: ''\n"
          "                      laneId: '" << d.start_position.lanelet_id << "'\n"
          "                      s: " << d.start_position.s << "\n"
          "                      offset: " << d.start_position.offset << "\n"
          "                      Orientation:\n"
          "                        type: relative\n"
          "                        h: 0\n"
          "                        p: 0\n"
          "                        r: 0\n"
          "              - LongitudinalAction:\n"
          "                  SpeedAction:\n"
          "                    SpeedActionDynamics:\n"
          "                      dynamicsDimension: time\n"
          "                      value: 0\n"
          "                      dynamicsShape: step\n"
          "                    SpeedActionTarget:\n"
          "                      AbsoluteTargetSpeed:\n"
          "                        value: " << d.speed << "\n";
  }

  void addEgoInitActions(std::ostream & ss, const traffic_simulator_msgs::msg::LaneletPose& start_position, const traffic_simulator_msgs::msg::LaneletPose& goal_position) {
    ss << "          - entityRef: " << ego_name_ << "\n"
          "            PrivateAction:\n"
          "              - TeleportAction:\n"
          "                  Position:\n"
          "                    LanePosition:\n"
          "                      roadId: ''\n"
          "                      laneId: '" << start_position.lanelet_id << "'\n"
          "                      s: " << start_position.s << "\n"
          "                      offset: " << start_position.offset << "\n"
          "                      Orientation:\n"
          "                        type: relative\n"
          "                        h: 0\n"
          "                        p: 0\n"
          "              - RoutingAction:\n"
          "                  AcquirePositionAction:\n"
          "                    Position:\n"
          "                      LanePosition:\n"
          "                        roadId: ''\n"
          "                        laneId: '" << goal_position.lanelet_id << "'\n"
          "                        s: " << goal_position.s << "\n"
          "                        offset: " << goal_position.offset << "\n"
          "                        Orientation:\n"
          "                          type: relative\n"
          "                          h: 0\n"
          "                          p: 0\n"
          "                          r: 0\n";
  }
  void addStoryHeaderAndTimeoutCondition(std::ostream& ss) {
    ss << "    Story:\n"
          "      - name: ''\n"
          "        Act:\n"
          "          - name: _EndCondition\n"
          "            ManeuverGroup:\n"
          "              - maximumExecutionCount: 1\n"
          "                name: ''\n"
          "                Actors:\n"
          "                  selectTriggeringEntities: false\n"
          "                  EntityRef: []\n"
          "                Maneuver:\n"
          "                  - name: ''\n"
          "                    Event:\n"
          "                      - name: ''\n"
          "                        priority: parallel\n"
          "                        StartTrigger:\n"
          "                          ConditionGroup:\n"
          "                            - Condition:\n"
          "                                - name: ''\n"
          "                                  delay: 0\n"
          "                                  conditionEdge: none\n"
          "                                  ByValueCondition:\n"
          "                                    SimulationTimeCondition:\n"
          "                                      value: " << test_timeout_ << "\n"
          "                                      rule: greaterThan\n"
          "                        Action:\n"
          "                          - name: ''\n"
          "                            UserDefinedAction:\n"
          "                              CustomCommandAction:\n"
          "                                type: exitSuccess\n"
          "                      - name: ''\n"
          "                        priority: parallel\n"
          "                        StartTrigger:\n"
          "                          ConditionGroup:\n";
  }

  void addStoryHeaderReachPositionConditionAndTimeoutCondition(std::ostream& ss, const traffic_simulator::LaneletPose& pose) {
    ss << "    Story:\n"
          "      - name: ''\n"
          "        Act:\n"
          "          - name: _EndCondition\n"
          "            ManeuverGroup:\n"
          "              - maximumExecutionCount: 1\n"
          "                name: ''\n"
          "                Actors:\n"
          "                  selectTriggeringEntities: false\n"
          "                  EntityRef:\n"
          "                    - entityRef: ego\n"
          "                Maneuver:\n"
          "                  - name: ''\n"
          "                    Event:\n"
          "                      - name: ''\n"
          "                        priority: parallel\n"
          "                        StartTrigger:\n"
          "                          ConditionGroup:\n"
          "                            - Condition:\n"
          "                                - name: ''\n"
          "                                  delay: 0\n"
          "                                  conditionEdge: none\n"
          "                                  ByEntityCondition:\n"
          "                                    TriggeringEntities:\n"
          "                                      triggeringEntitiesRule: any\n"
          "                                      EntityRef:\n"
          "                                        - entityRef: ego\n"
          "                                    EntityCondition:\n"
          "                                      ReachPositionCondition:\n"
          "                                        Position:\n"
          "                                          LanePosition:\n"
          "                                            roadId: ''\n"
          "                                            laneId: '" << pose.lanelet_id << "'\n"
          "                                            s: " << pose.s << "\n"
          "                                            offset: " << pose.offset << "\n"
          "                                            Orientation:\n"
          "                                              type: relative\n"
          "                                              h: 0\n"
          "                                              p: 0\n"
          "                                              r: 0\n"
          "                                        tolerance: 1\n"
          "                        Action:\n"
          "                          - name: ''\n"
          "                            UserDefinedAction:\n"
          "                              CustomCommandAction:\n"
          "                                type: exitSuccess\n"
          "                      - name: ''\n"
          "                        priority: parallel\n"
          "                        StartTrigger:\n"
          "                          ConditionGroup:\n"
          "                            - Condition:\n"
          "                                - name: ''\n"
          "                                  delay: 0\n"
          "                                  conditionEdge: none\n"
          "                                  ByValueCondition:\n"
          "                                    SimulationTimeCondition:\n"
          "                                      value: " << test_timeout_ << "\n"
          "                                      rule: greaterThan\n";
  }

  void addCollisionCondition(std::ostream& ss, const std::string& entity1_name, const std::string& entity2_name) {
    ss << "                            - Condition:\n"
          "                                - name: ''\n"
          "                                  delay: 0\n"
          "                                  conditionEdge: none\n"
          "                                  ByEntityCondition:\n"
          "                                    TriggeringEntities:\n"
          "                                      triggeringEntitiesRule: any\n"
          "                                      EntityRef:\n"
          "                                        - entityRef: " << entity1_name << "\n"
          "                                    EntityCondition:\n"
          "                                      CollisionCondition:\n"
          "                                        EntityRef:\n"
          "                                          entityRef: " << entity2_name << "\n";
  }

  void closeTheStory(std::ostream& ss) {
    ss << "                        Action:\n"
          "                          - name: ''\n"
          "                            UserDefinedAction:\n"
          "                              CustomCommandAction:\n"
          "                                type: exitFailure\n"
          "            StartTrigger:\n"
          "              ConditionGroup:\n"
          "                - Condition:\n"
          "                    - name: ''\n"
          "                      delay: 0\n"
          "                      conditionEdge: none\n"
          "                      ByValueCondition:\n"
          "                        SimulationTimeCondition:\n"
          "                          value: 0\n"
          "                          rule: greaterThan\n"
          "    StopTrigger:\n"
          "      ConditionGroup: []\n";
  }

  void addEmptyStoryAndEndCondition(std::ostream& ss) {
    ss << "    Story:\n"
          "      - name: ''\n"
          "        Act:\n"
          "          - name: _EndCondition\n"
          "            ManeuverGroup:\n"
          "              - maximumExecutionCount: 1\n"
          "                name: ''\n"
          "                Actors:\n"
          "                  selectTriggeringEntities: false\n"
          "                  EntityRef: []\n"
          "                Maneuver:\n"
          "                  - name: ''\n"
          "                    Event:\n"
          "                      - name: ''\n"
          "                        priority: parallel\n"
          "                        StartTrigger:\n"
          "                          ConditionGroup:\n"
          "                            - Condition:\n"
          "                                - name: ''\n"
          "                                  delay: 0\n"
          "                                  conditionEdge: none\n"
          "                                  ByValueCondition:\n"
          "                                    SimulationTimeCondition:\n"
          "                                      value: " << test_timeout_ << "\n"
          "                                      rule: greaterThan\n"
          "                        Action:\n"
          "                          - name: ''\n"
          "                            UserDefinedAction:\n"
          "                              CustomCommandAction:\n"
          "                                type: exitFailure\n"
          "            StartTrigger:\n"
          "              ConditionGroup:\n"
          "                - Condition:\n"
          "                    - name: ''\n"
          "                      delay: 0\n"
          "                      conditionEdge: none\n"
          "                      ByValueCondition:\n"
          "                        SimulationTimeCondition:\n"
          "                          value: 0\n"
          "                          rule: greaterThan\n"
          "    StopTrigger:\n"
          "      ConditionGroup: []\n";
  }

  TestDescription test_description_;
  const std::string ego_name_ = "ego";

  double test_timeout_;
  std::string map_path = "$(ros2 pkg prefix --share shinjuku_map)/map";
  bool run_autoware_ = false;

  bool scenario_completed_ = false;

  std::string out_dir_;
  int test_id_;

  rclcpp::Logger logger_;
};

#endif  // RANDOM_TEST_RUNNER__TEST_SCENARIO_WRITER_HPP
