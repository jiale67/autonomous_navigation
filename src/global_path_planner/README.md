# Global Planner Library for Autonomous Robots
This package is a library implementing various global waypoint planning algorithms, such as RRT, RRT*, [DEP](https://github.com/Zhefan-Xu/DEP) (our unknown exploration planner), based on the occupancy voxel map and the Octomap for autonomous mobile robots. 

**Authors**: [Zhefan Xu](https://zhefanxu.com/) and Christopher Suzuki, Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).

If you find this work helpful, kindly show your support by giving us a free ⭐️. Your recognition is truly valued.

This repo can be used as a standalone package and also comes as a module of our [autonomy framework](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy).

## I. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on [map_manager](https://github.com/Zhefan-Xu/map_manager) which provides the occupancy voxel map implementation and [octomap](http://wiki.ros.org/octomap) for octree-based map. 

```
# install dependency
sudo apt install ros-[melodic/noetic]-octomap* # octomap

cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/global_planner.git

cd ~/catkin_ws
catkin_make
```
## II. Run Planner DEMO
For simple RRT-based planner demo, please run the following commands:
```
roslaunch global_planner rrtInteractive.launch
```
Use ```2D Nav Goal``` in ```Rviz``` to select start and goal position in the map as shown below:

![Screenshot from 2022-01-22 11-47-43](https://user-images.githubusercontent.com/55560905/150648123-8c1d9102-0b44-4851-82f5-fff0101be0ac.png)

The example of the exploration planner can be shown as below (Please follow this [repo](https://github.com/Zhefan-Xu/autonomous_flight) for instructions):

![290708429-3dfd5516-e50b-4185-b502-1bfad6b36ae3](https://github.com/Zhefan-Xu/global_planner/assets/55560905/597b4e86-04c8-403a-a875-a62f1cd94dfa)

The related paper can be found on:

**Zhefan Xu\*, Christopher Suzuki\*, Xiaoyang Zhan, Kenji Shimada, "Heuristic-based Incremental Probabilistic Roadmap for Efficient UAV Exploration in Dynamic Environments”, IEEE International Conference on Robotics and Automation (ICRA), 2024.** [\[paper\]](https://arxiv.org/pdf/2303.00132.pdf) [\[video\]](https://youtu.be/fjVJCgDemjc?si=9nsWhReMeJH5JC3Q).

**Zhefan Xu, Di Deng, and Kenji Shimada, “Autonomous UAV Exploration of Dynamic Environments via Incremental Sampling and Probabilistic Roadmap”, IEEE Robotics and Automation Letters (RA-L), 2021.** [\[paper\]](https://ieeexplore.ieee.org/document/9362184) [\[video\]](https://youtu.be/ileyP4DRBjU?si=KFJLt-rLCa3tFaRH)

## III. Parameters
- RRT-based planner paramters can be found in ```global_planner/cfg/planner.yaml```. The followings are the default values: 
  - ```collision_box: [0.4, 0.4, 0.4]```
  - ```env_box: [-100, 100, -100, 100, 0, 1.5]```
  - ```timeout: 2.0```
  - ```rrt_incremental_distance: 0.3```
  - ```rrt_connect_goal_ratio: 0.2```
  - ```goal_reach_distance: 0.4```
  - ```map_resolution: 0.2```
  - ```vis_RRT: False``` (Not available for RRT*)
  - ```vis_path: True```
  - ```neighborhood_radius: 1.0``` (RRT* Only)
  - ```max_num_neighbors: 10``` (RRT* Only)
    
- DEP planner parameters (for unknown exploration) can be found in ```global_planner/cfg/dep_params.yaml```.

## IV. Code Exmaple & API
Please see example ```global_planner/src/rrt_interactive_node.cpp``` for RRT-based planner and ```global_planner/src/test_dep_node.cpp``` for unkonwn exploration planner. The example shows how to use the code API for your applications. 

## V. Issues
If you cannot visulize the Octomap with ROS noetic, that is because the [octomap_ros](http://wiki.ros.org/octomap) package has some tf name incompatibility issue. To solve that, please try building the octomap_ros from [source](https://github.com/OctoMap/octomap_mapping) with the following steps:

**Step1: download the octomap mapping source files**

```
cd ~/catkin_ws/src
git clone https://github.com/OctoMap/octomap_mapping.git
```

**Step2: modify source files and catkin make**
Please change all the frame id ```/map``` to ```map``` in ```octomap_mapping/octomap_server/OctomapServer.cpp``` and ```octomap_mapping/octomap_server/octomap_server_static.cpp```. There should be 2 places. 
```
cd ~/catkin_ws
catkin_make
```
## VI. Citation and Reference
If you find this work useful, please cite the paper:
```
@article{xu2023heuristic,
  title={Heuristic-based Incremental Probabilistic Roadmap for Efficient UAV Exploration in Dynamic Environments},
  author={Xu, Zhefan and Suzuki, Christopher and Zhan, Xiaoyang and Shimada, Kenji},
  journal={arXiv preprint arXiv:2309.09121},
  year={2023}
}
```

```
@article{xu2021autonomous,
  title={Autonomous UAV Exploration of Dynamic Environments via Incremental Sampling and Probabilistic Roadmap},
  author={Xu, Zhefan and Deng, Di and Shimada, Kenji},
  journal={IEEE Robotics and Automation Letters},
  year={2021},
  publisher={IEEE}
}
```
