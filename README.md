traversability_projection_ws
========

Introduction
========
This paper presents a novel and flexible multi-task multi-layer Bayesian mapping framework with readily extendable attribute layers. The proposed framework goes beyond modern metric-semantic maps to provide even richer environmental information for robots in a single mapping formalism while exploiting existing inter-layer correlations. It removes the need for a robot to access and process information from many separate maps when performing a complex task and benefits from the correlation between map layers, advancing the way robots interact with their environments. To this end, we design a multi-task deep neural network with attention mechanisms as our front-end to provide multiple observations for multiple map layers simultaneously. Our back-end runs a scalable closed-form Bayesian inference with only logarithmic time complexity. We apply the framework to build a dense robotic map including metric-semantic occupancy and traversability layers. Traversability ground truth labels are automatically generated from exteroceptive sensory data in a self-supervised manner. We present extensive experimental results on publicly available data sets and data collected by a 3D bipedal robot platform on the University of Michigan North Campus and show reliable mapping performance in different environments. Finally, we also discuss how the current framework can be extended to incorporate more information such as friction, signal strength, temperature, and physical quantity concentration using Gaussian map layers. The software for reproducing the presented results or running on customized data is made publicly available.

Note: This repository is not maintained anymore. If you have any question, send an email to either hongsn@umich.edu or ganlu@umich.edu.

Download the package
========
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

Build
========
```bash
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source devel/setup.bash
```

Usage
========
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
$ roslaunch dataset_ros_player spot.launch
$ roslaunch elevation_mapping_demos spot.launch
$ roslaunch traversability_estimation spot.launch
$ rosrun traversability_projection traversability_projection_node
$ rosbag play spot_exp3.bag -r 0.5 --pause --clock -k
```
Note: Need to run dataset_rosplayer package first for sim_time.

### Running for Spot dataset with Spot Model
```bash
$ roslaunch dataset_ros_player spot.launch
$ roslaunch elevation_mapping_demos spot_model.launch
$ roslaunch traversability_estimation spot.launch
$ rosrun traversability_projection traversability_projection_node
$ rosbag play spot_exp3.bag --pause --clock -k
```
Note: Need to run dataset_rosplayer package first for sim_time.

### Running for MiniCheetah
```bash
$ roslaunch dataset_ros_player mini_cheetah.launch
$ roslaunch elevation_mapping_demos mini_cheetah.launch
$ roslaunch traversability_estimation mini_cheetah.launch
$ rosrun traversability_projection traversability_projection_node
```

### Generate data for MiniCheetah IRL project
```bash
$ roslaunch dataset_ros_player mini_cheetah_irl.launch
$ roslaunch elevation_mapping_demos mini_cheetah_irl.launch
```

Modifications for Running the Spot
========

### dataset_ros_player launch file
Need to specify the camera mount angle by changing the static transform publisher parameters in spot.launch. Different poses are labeled in the comment for different rosbag data.

Change the depth camera max cutoff distance in launch file as well for better performance, which is "filter_limit_max".

### elevation_mapping_demos config
In the config/elevation_maps/spot.yaml, change the length_in_x, length_in_y, and resolution accordingly to tune the mesh size. Used 20.0 for the length and 0.05 for the solution in spot_eat_shit demo.

### elevation_mapping_demos RViZ
In the Panels, check the Views tab and choose ThirdPersonFollower as the type. Use body as Target Frame for better visualization.

### roslaunch rosbag file
Use the following code to launch the file for spot_eat_shit, where -s is starting at 150 sec, -u is playing for 68 sec, and -r is running at 0.5 speed.
```bash
rosbag play spot_exp_ice.bag --pause --clock -s 150 -u 68 -r 0.5
```

### rosbag data
The link to the rosbag data is attached below. https://drive.google.com/drive/folders/1fXs9EayUUSFRNWBDa4t0vk-1nNYNUjeZ?usp=sharing

Citation
========
An overview of the theoretical and implementation details has been
published in [https://arxiv.org/abs/2106.14986]. To cite traversability_projection_ws in your academic
research you can use the following BibTeX entry:

      @misc{gan2021multitask,
            title={Multi-Task Learning for Scalable and Dense Multi-Layer Bayesian Map Inference}, 
            author={Lu Gan and Youngji Kim and Jessy W. Grizzle and Jeffrey M. Walls and Ayoung Kim and Ryan M. Eustice and Maani Ghaffari},
            year={2021},
            eprint={2106.14986},
            archivePrefix={arXiv},
            primaryClass={cs.RO}
      }
