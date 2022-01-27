# traversability_projection_ws

### Download the package
```bash
$ git clone --recursive git@github.com:StevenHong/traversability_projection_ws.git
```

The following steps has been taken care of in the git commit.
```bash
$ cd traversability_projection_ws/src/any_node
$ rm -rf any_worker any_node any_node_example
$ cd ../../
```

Note: The CMakeLists.txt will be linked by catkin_make.
```bash
$ ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake CMakeLists.txt
```

### Building with catkin
```bash
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source devel/setup.bash
```

### Running for KITTI dataset
```bash
$ roslaunch elevation_mapping_demos kitti.launch
$ roslaunch traversability_estimation kitti.launch
$ rosrun traversability_projection traversability_projection_node
$ rosrun dataset_ros_player kitti_odom_node
```

### Running for ZED camera
```bash
$ roslaunch elevation_mapping_demos zed.launch
$ roslaunch traversability_estimation zed.launch
$ rosrun traversability_projection traversability_projection_node
$ rosrun dataset_ros_player zed_node
```

### Running for Spot dataset
```bash
$ roslaunch elevation_mapping_demos spot.launch
$ roslaunch traversability_estimation spot.launch
$ rosrun traversability_projection traversability_projection_node
$ rosrun dataset_ros_player spot_node
```

### Running for MiniCheetah
```bash
$ roslaunch dataset_ros_player mini_cheetah.launch
$ roslaunch elevation_mapping_demos mini_cheetah.launch
$ roslaunch traversability_estimation mini_cheetah.launch
$ rosrun traversability_projection traversability_projection_node
```

### Generate data for MiniCheetah IRL project
```bashg
$ roslaunch dataset_ros_player mini_cheetah_irl.launch
$ roslaunch elevation_mapping_demos mini_cheetah_irl.launch
```
