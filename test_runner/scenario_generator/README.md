Run rviz with the following command:
```bash
rviz2 -d $(ros2 pkg prefix random_test_runner)/share/random_test_runner/rviz/random_test.rviz --ros-args --remap /move_base_simple/goal:=/trajectory/goal_pose --remap /initialpose:=/trajectory/initial_pose
```