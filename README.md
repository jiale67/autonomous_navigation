# autonomous_navigation
A navigation algorithm based on CMU team's open-source local planner
<img src="img/header.jpg" alt="Header" width="100%"/>

This repository is developed based on the autonomius_deploration_development_denvironment repository, using a constructed octree map and RRT * algorithm to add global guidance to the local path planner for ground autonomous navigation and exploration. Obtained good results in simulated maze and cylindrical obstacle environments.

Video Tutorial:  [BiliBili](https://www.bilibili.com/video/BV1rexCzuECi?t=1.2) (for China mainland)
## Usage

The repository has been tested in Ubuntu 20.04 with ROS Noetic. Follow instructions in [Autonomous Exploration Development Environment](http://cmu-exploration.com) to setup the development environment. 

Install dependencies with command lines below.
```
sudo apt update
sudo apt install libusb-dev
sudo apt install ros-noetic-octomap*
```

Clone the repository.
```
cd your_ws/src
git clone https://github.com/jiale67/autonomous_navigation
catkin_make
```
To run the code, source the ROS workspace, and launch.
```
source ./devel/setup.bash 
roslaunch vehicle_simulator global_navigation.launch 
```
Or only initiate local planning
```
source ./devel/setup.bash 
roslaunch vehicle_simulator local_navigation.launch 
```
Now you can use 2D Nav Goal to send waypoint driven robots in rviz
