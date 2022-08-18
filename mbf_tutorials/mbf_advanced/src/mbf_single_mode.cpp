#include <fstream>
#include <ros/ros.h>
#include <mbf_advanced/mbf_rtmode_client.h>
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mbf_single_mode");
  ros::NodeHandle n;
  actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> ac_("move_base_flex/move_base", true);
  ac_.waitForServer();
  ROS_INFO("Connected to MBF action server");
 // std::vector<mbf_msgs::MoveBaseGoal> pose_goals;
//  std::vector<mbf_msgs::MoveBaseGoal> pose_goals_;
  mbf_msgs::MoveBaseGoal goal1;
 // ROS_INFO_STREAM("Attempting to reach " << goal.target_pose.pose.position.x << " / " << goal.target_pose.pose.position.y);
  mbf_advanced::MBFCircleClient mbfclient;
  goal1=mbfclient.create_goal(0.5,-1.0,0,0,0,-0.1,0.99);
  //goal1=mbfclient->create_goal(0.5,-1.0,0,0,0,-0.1,0.99);
  //pose_goals.push_back(goal);
//  ac_.sendGoal(goal1);
//  ac_.waitForResult();

//  auto result = ac_.getResult();
//  cout<<"result"<<result->outcome <<endl;
//  while(result->outcome !=mbf_msgs::MoveBaseResult::SUCCESS){


//};
  mbfclient.move_to_goal(goal1);



  return 0;
}
