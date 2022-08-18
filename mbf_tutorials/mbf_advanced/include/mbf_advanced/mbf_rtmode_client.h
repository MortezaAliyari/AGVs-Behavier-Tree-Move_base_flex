#pragma once

#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseResult.h>
#include </home/morteza/catkin_ws/src/mbf_tutorials/mbf_advanced/include/mbf_advanced/helpers_rtmode.h>
namespace mbf_advanced
{
struct Quaternion
{
    double w, x, y, z;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}
struct MBFCircleClient
{
  explicit MBFCircleClient()
          : acmbf_("move_base_flex/move_base", true)
          , terminate_(false)
  {
      acmbf_.waitForServer();
      ROS_INFO("Connected to MBF action server");
  }

  auto create_goal(float x,float y,float z,float q1,float q2,float q3,float q4){
    mbf_msgs::MoveBaseGoal goal;
    geometry_msgs::Pose pose;
    pose.position.x=x;
    pose.position.y=y;
    pose.position.z=z;
    pose.orientation.x=q1;
    pose.orientation.y=q2;
    pose.orientation.z=q3;
    pose.orientation.w=q4;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position = pose.position;
    goal.target_pose.pose.orientation = pose.orientation;
    return goal;
  }

  auto move_to_goal(mbf_msgs::MoveBaseGoal goal)
  {
    acmbf_.sendGoal(goal);
    acmbf_.waitForResult();
    auto result = acmbf_.getResult();
    std::cout<<"result"<<result->outcome <<std::endl;
    while(result->outcome !=mbf_msgs::MoveBaseResult::SUCCESS);
    if (result->outcome ==mbf_msgs::MoveBaseResult::SUCCESS)
    std::cout<<"Morteza Success"<<std::endl;
    return result;
  }
    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> acmbf_;
    bool terminate_;
    int np=0;
};

} // end
