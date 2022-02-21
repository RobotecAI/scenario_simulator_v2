# Random test runner

Random test runner allows running randomly generated scenarios to test autonomy implementation.

| NOTE: Currently no Autoware is supported. They will be supported in the future. |
|---------------------------------------------------------------------------------|

## How to build

### Prerequisites 

### Autoware.Auto

| NOTE: Autoware.Auto is not currently supported by random testing. |
|-------------------------------------------------------------------|

### AutowareArchitectureProposal

| NOTE: AutowareArchitectureProposal is not yet supported by random testing. |
|----------------------------------------------------------------------------|

### No-autoware mode

Make sure to have installed and sourced ros2 environment. Supported distributions are `foxy` and `galactic`. Also make sure to have `pip` installed.

```shell
sudo apt update
source /opt/ros/foxy/setup.bash
sudo apt install -y python3-pip python3-rosdep2 python3-vcstool python3-colcon-common-extensions
rosdep update
```

Prepare ROS workspace, clone repository on branch `feature/AJD-293-random_traffic_lights`, import and install dependencies

```bash
mkdir -p ~/scenario_simulator_ws/src
cd ~/scenario_simulator_ws/src
git clone https://github.com/tier4/scenario_simulator_v2.git
cd scenario_simulator_v2
git checkout feature/AJD-293-random_traffic_lights
vcs import external < dependency_foxy.repos
cd ~/scenario_simulator_ws
source /opt/ros/foxy/setup.bash
rosdep install -iry --from-paths src/scenario_simulator_v2 --rosdistro foxy
```

Build scenario_simulator

```bash
colcon build --symlink-install
```

### Custom map preparation

Download `kashiwanoha_map_new_traffic_lights` map package from [here](https://drive.google.com/drive/folders/1GNYM-6RA4YskrFUPTqlJEpS1jEvBdCI0) and place it in `<scenario_simulator_root>/map` directory, then call

```bash
colcon build --packages-up-to kashiwanoha_map_new_traffic_lights --symlink-install
```

## How to run

First complete build described in [How to build](#how-to-build) section for autoware architecture you are using.

Then, in the first terminal run: 

```shell
source install/setup.bash
rviz2 -d $(ros2 pkg prefix random_test_runner)/share/random_test_runner/rviz/random_test.rviz
```

In second:

```shell
source install/setup.bash
ros2 launch random_test_runner random_test.launch.py map_name:="kashiwanoha_map_new_traffic_lights"
```

Which will run random tests with default parameters. You should see several npcs spawned in random locations around the ego vehicle, which will move on random path.

After test is completed see `/tmp` directory. Among others, there will be two files:
1. `result.junit.xml` - test result file with information about encountered errors
2. `result.yaml` - yaml file that can be used to replay tests

Exemplary content of those files can be seen below:

`result.junit.xml`:
```xml
<?xml version="1.0"?>
<testsuites failures="0" errors="0" tests="0">
   <testsuite name="random_test" failures="0" errors="0" tests="0">
      <testcase name="2" />
      <testcase name="1" />
      <testcase name="0" />
   </testsuite>
</testsuites>
```

`result.yaml`:
```yaml
random_test:
   test_name: random_test
   map_name: kashiwanoha_map_new_traffic_lights
   ego_goal_s: 0
   ego_goal_lanelet_id: -1
   ego_goal_partial_randomization: false
   ego_goal_partial_randomization_distance: 20
   npc_count: 0
   npc_min_speed: 0.5
   npc_max_speed: 3
   npc_min_spawn_distance_from_ego: 30
   npc_max_spawn_distance_from_ego: 100
   min_green_light_duration: 3
   max_green_light_duration: 10
   test_cases:
      - seed: 4181894907
      - seed: 3229451657
      - seed: 3173578686
```

## How to replay

Prerequisites:
1. Build as instructed in [How to build](#how-to-build)
2. Acquire `result.yaml` file:
   1. Either by running test as stated in [How to run](#how-to-run) part of instruction
   2. Receiving it from someone who already ran it
3. Place `result.yaml` in `<some_directory>` IMPORTANT: Do not change filename
4. Execute:
 
```shell
ros2 launch random_test_runner random_test.launch.py input_dir:=<some_directory>
```

Random test runner will load `result.yaml` file and rerun test.

## Running with unity

Instruction is based on `kashiwanoha_map` Unity project but can be applied to any other projects supporting [`ZeroMQ` interface](https://tier4.github.io/scenario_simulator_v2-docs/design/ZeroMQ/). 

To run `random_test_runner` with Unity Kashiwanoha project: 
1. Build and source latelet2 scene builder using [lanelet2_scene_builder](https://gitlab.com/robotec.ai/tieriv/lanelet2-scene-builder) and [lanelet2_scene_msgs](https://gitlab.com/robotec.ai/tieriv/lanelet2-scene-msgs) instructions
2. Clone [Kashiwanoha project](https://gitlab.com/robotec.ai/tieriv/kashiwanoha).
3. Checkout branch 
```shell
git checkout feature/AJD-293-random_traffic_lights
```
4. Source ros galactic environment
```shell
source /opt/ros/galactic/setup.bash
```
5. Run `kashiwanoha` project 
6. Execute lanelet2 scene builder node. This will spawn traffic lights
```shell
ros2 run lanelet2_scene_builder lanelet2_scene_builder --ros-args -p osm_file:=<AUTOWARE_DIRECTORY>/install/kashiwanoha_map_new_traffic_lights/share/kashiwanoha_map_new_traffic_lights/map/lanelet2_map.osm
```
7. Execute `random_test_runner` launch with `simulator_type` parameter and correct map name:
```shell
ros2 launch random_test_runner random_test.launch.py simulator_type:="unity" map_name:="kashiwanoha_map_new_traffic_lights" test_count:=3
```
|  NOTE: Since currently unity integration does not support ego vehicle, `random_test_runner` does not spawn it. |
|----------------------------------------------------------------------------------------------------------------|

|  NOTE: Kashiwanoha project is only supported on ROS2 `galactic` but simulation interfaces are distribution-independent and random tests can be executed safely on `foxy` |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

## Launch arguments

This section describes arguments accepted by launch file. Please note that those arguments should not be included in parameters file specified in `test_parameters_filename`.

Apart from that, any `random_test_runner` node parameters can be supplied as a launch arguments.
Please refer to [Node parameters](#node-parameters) chapter for more details on parameter source priorities.

### General arguments

| Parameter name               | Default value                 | Description                                                                                                                                                                                                                 |
|------------------------------|-------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `test_parameters_filename`   |  `""`                         | Yaml filename within `random_test_runner/param directory` containing test parameters. With exception from [Launch arguments](#launch-arguments) chapter, parameters specified here will override values passed as arguments |
| `simulator_type`             |  `"simple_sensor_simulator"`  | Backend simulator. Supported values are `unity` and `simple_sensor_simulator`.                                                                                                                                              |

### Autoware related arguments

Launch also accepts autoware parameters that control autoware related behavior. It can set which autoware architecture is in use, which vehicle
and sensor model is used in the simulation

| Parameter name      | Default value                 | Description                                                               |
|---------------------|-------------------------------|---------------------------------------------------------------------------|
| `architecture_type` | `"awf/auto"`                  | Autoware architecture type. Possible values: `awf/auto`, `tier4/proposal` |
| `sensor_model`      | `"aip_xx1"`                   | Ego sensor model                                                          |
| `vehicle_model`     | `"lexus"`                     | Ego vehicle model                                                         |

## Node parameters

Random testing supports several parameters to control test execution. They can be supplied either directly from command 
line or via yaml file inside `<random_test_runner_directory>/param/` named `test_parameters_filename`.

Parameters have the following source precedence priorities:
1. `*.yaml` file
2. Command line
3. Default values

### Parameters reference

Random test runner parameters are split into three categories:

1. [Test control parameters](#test-control-parameters)
2. [Test suite parameters](#test-suite-parameters)
3. [Test case parameters](#test-case-parameters)

#### Test control parameters

High level parameters not directly related to the test itself

| Parameter name    | Default value                 | Description                                                                                                               |
|-------------------|-------------------------------|---------------------------------------------------------------------------------------------------------------------------|
| `input_dir`       |  `""`                         |  Directory containing the result.yaml file to be replayed. If not empty, tests will be replayed from result.yaml          |
| `output_dir`      |  `"/tmp"`                     |  Directory to which result.yaml and result.junit.xml files will be placed                                                 |
| `test_count`      |  `5`                          |  Number of test cases to be performed in the test suite                                                                   |

#### Test suite parameters

Core test parameters. It sets map name, ego goal information and npc spawning parameters.

| Parameter name                            | Default value               | Description                                                                                                                                                                                                                                                                                        |
|-------------------------------------------|-----------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `test_name`                               | `"random_test"`             | Test name. Used for descriptive purposes only                                                                                                                                                                                                                                                      |
| `map_name`                                | `"kashiwanoha_map"`         | Package name containing map information (lanelet, point cloud, etc)                                                                                                                                                                                                                                |
| `traffic_lights_generator_type`           | `"lanelet_collision_based"` | Algorithm which will be used to compute traffic lights phases. Possible values are `lanelet_collision_based` or `lanelet_stopline_direction_based`. Meaning is that phases are generated based on collisions between lanelets, or based on geographical direction of the stop lines, respectively. |
| `spawn_ego`                               | `false`                     | If `true`, ego vehicle will be spawned in start position                                                                                                                                                                                                                                           |
| `ego_goal_lanelet_id`                     | `-1`                        | Ego goal lanelet's id. Must be a valid id for map specified in `map_name` parameter. If `-1`, goal will be chosen randomly                                                                                                                                                                         |
| `ego_goal_s`                              | `0.0`                       | Ego goal lanelet's s (translation along the lanelet in meters). If `ego_goal_lanelet_id` equals `-1`, s will be chosen randomly                                                                                                                                                                    |
| `ego_goal_partial_randomization`          | `false`                     | If `true`, goal will be randomized within distance set in `ego_goal_partial_randomization_distance` value. If `ego_goal_lanelet_id` is set to `-1`, this value is ignored                                                                                                                          |
| `ego_goal_partial_randomization_distance` | `30.0`                      | Distance in meters from goal set by `ego_goal_lanelet_id` and `ego_goal_s`, within which goal pose will be randomized if `ego_goal_partial_randomization` is set to true                                                                                                                           |
| `npc_count`                               | `10`                        | Generated npc count                                                                                                                                                                                                                                                                                |
| `npc_min_speed`                           | `0.5`                       | Minimum speed of generated npcs                                                                                                                                                                                                                                                                    |
| `npc_max_speed`                           | `3.0`                       | Maximum speed of generated npcs                                                                                                                                                                                                                                                                    |
| `npc_min_spawn_distance_from_start`       | `10.0`                      | Minimum distance of generated npcs from ego                                                                                                                                                                                                                                                        |
| `npc_max_spawn_distance_from_start`       | `100.0`                     | Maximum distance of generated npcs from ego                                                                                                                                                                                                                                                        |
| `min_green_light_duration`                | `3.0`                       | Minimum duration of green traffic light                                                                                                                                                                                                                                                            |
| `max_green_light_duration`                | `10.0`                      | Maximum duration of green traffic light                                                                                                                                                                                                                                                            |
| `start_lanelet_id`                        | `-1`                        | Start lanelet id. Must be a valid id for map specified in `map_name` parameter. If applicable, ego will be spawned here and NPCs will be spawned around this place. If -1, start will be chosen randomly                                                                                           |
| `start_s`                                 | `0.0`                       | Start lanelet s (translation along the lanelet in meters). If applicable, ego will be spawned here and NPCs will be spawned around this place. If `start_lanelet_id` equals -1, s will be chosen randomly                                                                                          |

#### Test case parameters

Test case parameters. Currently, only randomization seed.

| Parameter name  | Default value | Description                                                       |
|-----------------|---------------|-------------------------------------------------------------------|
| `seed`          |   `-1`        | Randomization seed. If `-1`, seed will be generated for each test |


