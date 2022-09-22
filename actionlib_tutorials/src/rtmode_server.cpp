#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/RobotmodeAction.h>
#include "std_msgs/String.h"
#include </home/morteza/catkin_ws/src/mbf_tutorials/mbf_advanced/include/mbf_advanced/mbf_rtmode_client.h>
#include </home/morteza/catkin_ws/src/mbf_tutorials/mbf_advanced/include/mbf_advanced/mbf_circle_client2.h>

using namespace std;
class RobotmodeAction
{
protected:

  ros::NodeHandle nh_;
  std_msgs::String Nav_CM_msg;
  std_msgs::String Navfilename_msg;

  ros::Publisher Nav_CM_pub      = nh_.advertise<std_msgs::String>("Nav_CM", 1);
  ros::Publisher Navfilename_pub = nh_.advertise<std_msgs::String>("Navfilename", 1);
  ros::Subscriber NavfileResults_sub=nh_.subscribe("NavfileResults", 10,&RobotmodeAction::NavfileResults_Callback,this); //new format of subscribing topic in constructer function of a class;

  actionlib::SimpleActionServer<actionlib_tutorials::RobotmodeAction> asrtmode_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
//  actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> acmbfnavfile_;

  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::RobotmodeFeedback feedback_;
  actionlib_tutorials::RobotmodeResult result_;

public:
  mbf_rtmode::MBFCircleClient mbfclient_Navsingle; //relate to receiving feedback when running single commands
  //mbf_advanced::MBFCircleClient mbfclient_Navfile; //relate to receiving feedback when running scenario or Navigation file
  bool nfr=false,fbnfr=false,sfbnfr=false,ffbnfr=false;

  string NavfileResults;

  void NavfileResults_Callback(const std_msgs::StringConstPtr& msg)
  {
    size_t found1 = msg->data.find("successfuly reached to target number");
    size_t found2 = msg->data.find("successfuly finished scenario!");
    size_t found3 = msg->data.find("Failed to reach to target number");
    size_t found4 = msg->data.find("scenario Failed!");
    nfr=true;
    if (found1 != string::npos)
    {
      fbnfr=true;//target feedback of navfile result
      NavfileResults=msg->data;
      ROS_INFO("found1");
    }
    else if (found2 != string::npos) {
      sfbnfr=true;//scenario feedback of navfile result
      NavfileResults=msg->data;
      ROS_INFO("found2");
    }
    else if (found3 != string::npos) {
      fbnfr=true;//target feedback of navfile result
      NavfileResults=msg->data;
      ROS_INFO("found3");
    }
    else if (found4 != string::npos) {
      ffbnfr=true;//scenario feedback of navfile result
      NavfileResults=msg->data;
      ROS_INFO("found4");
    }
    else {
      fbnfr =false;//target   feedback of navfile result
      sfbnfr=false;//scenario feedback of navfile result
      ffbnfr=false;
      ROS_INFO("NavfileResults=msg->data nothing matched");
    }
  }

  /////////////////////////
  RobotmodeAction(std::string name) :
    asrtmode_(nh_, name, boost::bind(&RobotmodeAction::executeCB, this, _1), false),
    action_name_(name)
  {
    Nav_CM_msg.data="DE";

    asrtmode_.start();
    ros::Duration(0.1).sleep();//without this delay ros doesn't publish Nav_CM_msg
    for (int i=0;i<5;i++)
      Nav_CM_pub.publish(Nav_CM_msg);
    ROS_INFO("Robot is waiting for Client request!");
  }

  ~RobotmodeAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::RobotmodeGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    feedback_.targetpoint.clear();
    feedback_.scenario.clear();

    int n=0;

    // publish info to the console for the user
    ROS_INFO("Action messge is as below: ");
    ROS_INFO("                          order value is %d",goal->order);
    ROS_INFO("                          Gmapping value is %d",goal->gm);
    ROS_INFO("                          Navigation value is %d",goal->nav);
    ROS_INFO("                          Single Navigation value is %d",goal->nav_single);
    ROS_INFO("                          File   Navigation value is %d",goal->nav_file);
    ROS_INFO("                          Single command value is %f",goal->singledata[1]);
    ROS_INFO("                          Single task name is %s",goal->singlename.c_str());
    ROS_INFO("                          navigation file name is %s",goal->filename.c_str());
    if (goal->nav==0 && goal->gm==1){
      Nav_CM_msg.data="ED";
      Nav_CM_pub.publish(Nav_CM_msg);
      ROS_INFO("Gmapping is Enabled use keyboard to control the robot!");
    }
    else if(goal->nav==1 && goal->gm==0)
    {
      Nav_CM_msg.data="DE";
      Nav_CM_pub.publish(Nav_CM_msg);
      ROS_WARN("Navigation is Enabled!");
      if(goal->nav_single==1 && goal->nav_file==0){
        ROS_INFO("Single command is selected");
        int arrSize=goal->order;
        ROS_INFO("Number of single commands is : %d",arrSize);
        for (int i=0;i<arrSize;i=i+3) {// loop started

          if (goal->singledata[1+i]+goal->singledata[2+i]==M_PI+M_PI){
            ROS_INFO("Robot sleeped for [%f] second",goal->singledata[i]);
            ros::Duration(goal->singledata[i]).sleep();
          }
          else {
            ROS_INFO("Robot Moving to goal point! x= %f , y=%f",goal->singledata[i],goal->singledata[i+1]);
            float x=goal->singledata[i];
            float y=goal->singledata[i+1];
            mbf_rtmode::Quaternion q;
            q=mbf_rtmode::ToQuaternion(goal->singledata[i+2],0,0);
            auto result=mbfclient_Navsingle.move_to_goal(mbfclient_Navsingle.create_goal(x,y,0,q.x,q.y,q.z,q.w));

            if(result->outcome ==mbf_msgs::MoveBaseResult::SUCCESS){
              n++;
              feedback_.targetpoint.append("Success on target number ");
              feedback_.targetpoint.append(to_string(n));
              // publish the feedback
              asrtmode_.publishFeedback(feedback_);
              // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
              r.sleep();
              ROS_INFO("Robot reached to the one of the target points");
            }
            else {
              success = false;
            }
            if (asrtmode_.isPreemptRequested() || !ros::ok())
            {
              ROS_INFO("%s  server: Preempted or cancelled", action_name_.c_str());
              ROS_INFO("MBF server: Preempted or cancelled");
              feedback_.targetpoint=" User Cancelled the NavSingle Mode! ";
              asrtmode_.publishFeedback(feedback_);
              // set the action state to preempted
              asrtmode_.setAborted();
              mbfclient_Navsingle.acmbf_.cancelAllGoals();
              success = false;
              break;
            }

          }
        }
      }
      else if (goal->nav_single==0 && goal->nav_file==1)
      {
        Navfilename_msg.data=goal->filename;
        Navfilename_pub.publish(Navfilename_msg);
        while(1)
        {
          ROS_INFO("waiting for result of File navigation mode!");
          while(!nfr); //stop untile receive result from MBF action
          nfr=false;
          if(sfbnfr)
          {// send successful result to rtmode client

            sfbnfr=false;
            success = true;
            ROS_INFO("sfbnfr");
            break;
          }
          else if (ffbnfr)
          {//send fail result to rtmode client
            ffbnfr=false;
            success = false;
            ROS_INFO("ffbnfr");
            break;
          }
          else if (fbnfr)
          {//send feedback to rtmode client
            ROS_INFO("[%s]",NavfileResults);
            feedback_.scenario=NavfileResults;
            asrtmode_.publishFeedback(feedback_);
            fbnfr=false;
            ROS_INFO("fbnfr");
          }
          else {
            ROS_INFO("No result!");
            sfbnfr=false;
            fbnfr=false;

          }
          if (asrtmode_.isPreemptRequested() || !ros::ok())
          {
            ROS_INFO("%s  server: Preempted or cancelled", action_name_.c_str());
            ROS_INFO("MBF server: Preempted or cancelled");

            // set the action state to preempted
            asrtmode_.setAborted();
            Navfilename_msg.data="cancellallgoals";
            Navfilename_pub.publish(Navfilename_msg);
            success = false;
            break;
          }
        }
      }
      else {
          ROS_INFO("Enable one of the modes:");
          ROS_INFO("                        signle or file Navigation");
          ROS_INFO("                        The defualt is single");
      }
    }
    else{
        ROS_INFO("Enable one of the modes:");
        ROS_INFO("                        Gmapping or Navigation");
        ROS_INFO("                        The defualt is Navigation");
    }
    if(success)
    {
      result_.result = NavfileResults;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      asrtmode_.setSucceeded(result_);
    }
    else {
      result_.result = "Robot has a problem";
      ROS_INFO("%s: Aborted from rtmode server", action_name_.c_str());
      asrtmode_.setAborted(result_);
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
