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
- Move base Flex or MBF
- BT C++
- Groot
- Cmake 3.20
## Move Base Flex

[Move Base Flex](https://uos.github.io/mbf_docs/installation/) install this package to avoid navigation abort or crash during tracking planed path.
```
sudo apt install ros-noetic-move-base-flex
```
## Behavior Tree 
[Behavior Trees](https://github.com/BehaviorTree/BehaviorTree.CPP)
actually began in the videogame industry to define behaviors for non-player characters (NPCs): Both Unreal Engine and Unity (two major forces in this space) have dedicated tools for authoring BTs. This is no surprise; a big advantage of BTs is that they are easy to compose and modify, even at runtime. However, this sacrifices the ease of designing reactive behaviors (for example, mode switches) compared to some of the other abstractions, as you will see later in this repo.
### Install BTC++ on ROS1 as follows:
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
## Run SLAM and Navigation algorithms simultaneously for turtlebot3.
If we be able to run both algorithms to execute independently, we wouldn't need to kill any node during the transition from navigation to creating the map method.
It seems we should download the source file of each algorithm as a package and install them on our catkin workspace.
source code of gmapping algorithm could be downaload from below github accout. last branch of this repo relate to melodic version.
```
https://github.com/ros-perception/slam_gmapping.git
```
then you need to change `slam_gmapping.cpp` node file.
make a new launch file inside the launch folder same as mine which uploaded in this repo.
`slam_gmapping_cleaner.launch`
if you want to cancel the map frame broadcasting you need to change the `  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);
` to zero.
`  private_nh_.param("transform_publish_period", transform_publish_period_, 0.0);
`
now everthing is working just the final result in map doesn't publish any more. so the navigation algorithm can use another map frame which is publishing by amcl node file.
if you want to cancel the map frame broadcasting fron inside the amcl node you need to change `amcl_node,cpp`
```
        if (a12==1){
            this->tfb_->sendTransform(tmp_tf_stamped);
            sent_first_transform_ = true;
        }
        
```
In order to cancel the broadcasting map frame id by gmapping node just replace the `slam_gmapping.cpp` and its header `slam_gmapping.h` in your source file in your slam package.
## Run MBF and BTC++ 
Use [MBF](https://uos.github.io/mbf_docs/tutorials/beginner/basic_navigation/) tutorial. 
rosdep install will fail for eband_local_planner, because the team hasn't released the version for noetic yet. Simply donwload and add eband_local_planner to your workspace in src folder.
add packages and change the folders name to `mbf_advanced` and `mbf_beginner` and `eband_local planner`
## Code Explanation
In this project we consider four different locations from the map. After reaching to each location robot will enable specific function. the last location would be same as first one as home location.
the coordinate of locations will be write in the yamle file and like dictionary in python(key/value).
```  std::string yaml_file;
    ros::param::get("location_file", yaml_file);
    YAML::Node locations = YAML::LoadFile(yaml_file);
    int num_locs = locations.size();
  ```
After adding below headers to your main.cpp file:
```
#include <fstream>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mbf_advanced/mbf_circle_client2.h>
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
```
you need to create two action nodes as below:
```
using State = mbf_advanced::MBFCircleClientState;

class GoToPose : public BT::SyncActionNode
{
public:
    explicit GoToPose(const std::string& name)
      : BT::SyncActionNode(name, {})
      , mbfclient_{}
    { }

    void attachMBFClient(std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient)
    {
        mbfclient_ = mbfclient;
    }

    BT::NodeStatus tick() override
    {
        //ROS_INFO_STREAM("BT: " << this->name());

        if (mbfclient_)
        {

            ROS_INFO_STREAM("BT: " << this->name());
            auto r=mbfclient_->move_to_goal();
           if (r->outcome  == mbf_msgs::MoveBaseResult::SUCCESS) {
               return BT::NodeStatus::SUCCESS;
           }

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_;
};

class LookForObject : public BT::SyncActionNode
{
public:
    explicit LookForObject(const std::string& name)
      : BT::SyncActionNode(name, {})
      , mbfclient_{}
    { }

    void attachMBFClient(std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient)
    {
        mbfclient_ = mbfclient;
    }

    BT::NodeStatus tick() override
    {
        ROS_INFO_STREAM("BT: " << this->name());

        if (mbfclient_)
        {
            ROS_INFO_STREAM("            welldone got to the next position!!        ");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_;
};
```
