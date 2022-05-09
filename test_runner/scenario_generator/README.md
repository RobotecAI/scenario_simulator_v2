To use the `scenario_generator`:

1. Download `shinjuku_map` ROS2 package from [here](https://drive.google.com/file/d/14rRcfNBrkgrWk8qTdwB7CA2JAowJpEuQ/view?usp=sharing)

2. Extract it to:
   1. `scenario_simulator_v2/map`
   2. `autoware/src/simulator/scenario_simulator/map` (after cloning the autoware repo from point 3. but before building)

3. Follow [these instructions](https://github.com/tier4/scenario_simulator_v2/blob/master/docs/tutorials/RunWithAutowareUniverse.md) to build Autoware and `scenario_simulator_v2`

4. Run `scenario_generator` with:

    ```bash
    ros2 launch scenario_generator scenario_generator.launch.py lanelet_map_path:=map/shinjuku_map/map/lanelet2_map.osm
    ```

5. Run Autoware with:

    ```bash
     ros2 launch random_test_runner random_test.launch.py architecture_type:=awf/universe sensor_model:=sample_sensor_kit vehicle_model:=sample_vehicle map_name:=shinjuku_map
    ```

    and kill it after lanelet map is displayed in rviz

6. Generate python trajectories by init/goal pose in rviz

    > NOTE: You can change trajectory mode by publishing on `/trajectory/option` topic:
    > * 'v' for veicles (lanelet planning)
    > * 'p' for pedestrians (simple collecting of the poses)