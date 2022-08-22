#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/RobotmodeAction.h>
#include "std_msgs/String.h"
#include </home/morteza/catkin_ws/src/mbf_tutorials/mbf_advanced/include/mbf_advanced/mbf_rtmode_client.h>

using namespace std;
class RobotmodeAction
{
protected:

  ros::NodeHandle nh_;
  std_msgs::String Nav_CM_msg;
  std_msgs::String Navfilename_msg;

  ros::Publisher Nav_CM_pub      = nh_.advertise<std_msgs::String>("Nav_CM", 1);
  ros::Publisher Navfilename_pub = nh_.advertise<std_msgs::String>("Navfilename", 1);

  actionlib::SimpleActionServer<actionlib_tutorials::RobotmodeAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::RobotmodeFeedback feedback_;
  actionlib_tutorials::RobotmodeResult result_;

public:
  mbf_rtmode::MBFCircleClient mbfclient;
  RobotmodeAction(std::string name) :
    as_(nh_, name, boost::bind(&RobotmodeAction::executeCB, this, _1), false),
    action_name_(name)
  {
    Nav_CM_msg.data="DE";
    as_.start();
    ros::Duration(0.01).sleep();//without this delay ros doesn't publish Nav_CM_msg
    Nav_CM_pub.publish(Nav_CM_msg);
    cout<<"////////////////////////////////////////////Change the navigation mode///////////////////////////"<<endl;
  }

  ~RobotmodeAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::RobotmodeGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating robotmode sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
    ROS_INFO("Action messge is as below: ");
    ROS_INFO("Gmapping value is %d",goal->gm);
    ROS_INFO("Navigation value is %d",goal->nav);

    ROS_INFO("Navigation mode is single %d",goal->nav_single);

    ROS_INFO("Navigation mode is file   %d",goal->nav_file);

    ROS_INFO("Single command value is %f",goal->singledata[1]);
    ROS_INFO("Single task name is %s",goal->singlename.c_str());
    ROS_INFO("navigation file name is %s",goal->filename.c_str());
    if (goal->nav==0 && goal->gm==1){
      Nav_CM_msg.data="ED";
      Nav_CM_pub.publish(Nav_CM_msg);
      cout<<"////////////////////////////////////////////Gmapping is Enabled now control the robot! "<<endl;

    }
    else if(goal->nav==1 && goal->gm==0){
      Nav_CM_msg.data="DE";
      Nav_CM_pub.publish(Nav_CM_msg);
      cout<<"////////////////////////////////////////////Navigation is Enabled now select navigation mode! "<<endl;
      if(goal->nav_single==1 && goal->nav_file==0){

        cout<<"////////////////////////////////////////////Single command is selected!"<<endl;
        float x=goal->singledata[0];
        float y=goal->singledata[1];
        mbf_rtmode::Quaternion q;
        q=mbf_rtmode::ToQuaternion(goal->singledata[2],0,0);
        mbfclient.move_to_goal(mbfclient.create_goal(x,y,0,q.x,q.y,q.z,q.w));

      }
      else if (goal->nav_single==0 && goal->nav_file==1) {
        Navfilename_msg.data=goal->singlename;
        Navfilename_pub.publish(Navfilename_msg);
        cout<<"////////////////////////////////////////////["<<goal->singlename.c_str()<<"] File is selected to navigate!"<<endl;
      }
      else {
        cout<<"////////////////////////////////////////////Enable one of the modes: "<<
            "signle  or file Navigation\n"<<
            "////////////////////////////////////////////The defualt is single"<<endl;
      }

    }
    else
      cout<<"////////////////////////////////////////////Enable one of the modes: "<<
          "Gmapping or Navigation\n"<<
          "////////////////////////////////////////////The defualt is Navigation"<<endl;

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtmode_server");

  RobotmodeAction Robotmode("Robotmode");
  ros::spin();

  return 0;
}
