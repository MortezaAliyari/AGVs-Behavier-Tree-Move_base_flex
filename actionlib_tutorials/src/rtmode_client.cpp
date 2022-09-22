#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/RobotmodeAction.h>
#include <cmath>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "rtmode_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_tutorials::RobotmodeAction> ac("Robotmode", true);

  ROS_INFO("Waiting for rtmode action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending Robot Mode.");
  // send a goal to the action
  std::vector<float> single{0.5,-1.0, -1};
  actionlib_tutorials::RobotmodeGoal goal;
  goal.order = 15;
  goal.gm=0;
  goal.nav=1;
  goal.nav_single=1;
  goal.nav_file=0;
  goal.singledata={0.5,-1,-1, 1.5,M_PI,M_PI, 0.5,1,-1, 5.5,M_PI,M_PI, 0.5,-1,-1}; //3*5
  goal.singlename="delay";
  goal.filename="B";
  ac.sendGoal(goal);
  ros::Duration(5.0).sleep();
  ac.cancelAllGoals();
  //ac.cancelGoal();
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(200.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Rtmode Action did not finish before the time out.");
  ROS_INFO("Robot should be stopped!");
  //exit
  return 0;
}
