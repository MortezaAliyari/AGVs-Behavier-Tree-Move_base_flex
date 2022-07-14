# AGVs-BT-Move_base_flex
In this repo, i will develop and work on Behavier Tree C++, Move_base_flex and solve the aborted situation during navigation base on python and C++

If you like the project give me a star! :star: 

You can see the video: &nbsp;&nbsp;
[![website](./img/youtube-dark.svg)](https://www.youtube.com/channel/UCyRBig4xgAdaRdIz14Xymrg)
&nbsp;&nbsp;
---

### This package runned on below circumstances :
- Ubuntu Focal
- ROS Noetic 
- Gazebo11 & TurtleBot3
- BT C++
- Groot
- Cmake 3.20


## Behavior Tree 
[Behavior Trees](https://github.com/BehaviorTree/BehaviorTree.CPP)
actually began in the videogame industry to define behaviors for non-player characters (NPCs): Both Unreal Engine and Unity (two major forces in this space) have dedicated tools for authoring BTs. This is no surprise; a big advantage of BTs is that they are easy to compose and modify, even at runtime. However, this sacrifices the ease of designing reactive behaviors (for example, mode switches) compared to some of the other abstractions, as you will see later in this repo.
### Install BTC++ on ROS1 or ROS2 as follows:
``` 
sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3 
```
### Install Groot as on Linux follows:
```
   git clone https://github.com/BehaviorTree/Groot.git
   cd Groot
   git submodule update --init --recursive
   cd depend/BehvaiorTree.Cpp
   mkdir build; cd build
   cmake ..
   make
```
You are not allow to built on source so change the path`  cd depend/BehvaiorTree.Cpp`.
### Install Groot as on ROS1 or ROS2 follows:
```
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   git clone https://github.com/BehaviorTree/Groot.git
   cd ..
   ```
   open `CmakeLists.txt` remove the line `find_package(ament_index_cpp REQUIRED)` and continue:
   ```
   rosdep install --from-paths src --ignore-src
   catkin_make  
   ```
   it should work now.
