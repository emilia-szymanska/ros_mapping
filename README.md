# SLAM - ros_mapping

## Important information
ROS scripts allowing a user to see the possibilities of rosbag, rviz and scanning the space with the use of a lidar.

### Prerequisites
To start with this project, ROS noetic (with rviz tool) needs to be installed and sourced. Prepare a bag file with scan data from a robot with lidar (an example bag in the repo).

### Running the program
Remember to source the environment in each newly opened terminal and to run roscore in one of them. It is also essential to run `rosparam set use_sim_time true` if you want to run the rosbag the way it is needed here.
After cloning this repository to your computer (and to your workspace's 'src' directory), open 4 terminals and in each run one of the following commands:
```
rviz
```

```
rosbag play example.bag --clock
```

```
./trajectory_visualizer.py _child_frame_id:=pioneer/base_link
```


```
./laserscan_to_points.py _accumulate_points:=True scan:=pioneer/scan
```

In rviz, make sure to add Marker (from topic `/robot_positions` and `/point_positions`) and TF if you want to see the robot's tf. Set `map` as the global frame. The result should be similar to the one presented in `map.png` file.

### Authors
* **Emilia Szymańska**
