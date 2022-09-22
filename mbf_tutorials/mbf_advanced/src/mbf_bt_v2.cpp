#include <fstream>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mbf_advanced/mbf_circle_client2.h>
#include "std_msgs/String.h"
#include <algorithm>
#include <vector>
//#include <string>

///
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
///
ros::Subscriber Navfilename_sub;
ros::Publisher velPub;
std::vector<std::string> filenames={"A","B","C"};
std::string filename="nothing";
std::string POSE_PATH;
std::string BT_XML_PATH;
bool cancelflag=false;
void Navfilename_callback(const std_msgs::String::ConstPtr& msg)
{
  filename=msg->data;
  ROS_INFO("Robot received the command from user : [%s]", filename.c_str());
  if(filename=="cancellallgoals"){
    cancelflag=true;
    ROS_INFO("Cancel flag is seen 1 ! ");

  }
}
class GoToPose : public BT::SyncActionNode
{
public:
  std_msgs::String msg;
  geometry_msgs::Twist velmsg;
  explicit GoToPose(const std::string& name)
    : BT::SyncActionNode(name, {})
    , mbfclient_{}
  {
    ros::NodeHandle nh_;
    NavfileResults_pub= nh_.advertise<std_msgs::String>("NavfileResults", 2);


    velPub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

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
        for (int i=0;i<10;i++) {
          ros::spinOnce();
          ros::Duration(0.1).sleep();//without this for loop the user thread and callback will not work.
        }
        if(filename=="cancellallgoals"){
          mbfclient_->ac_.cancelAllGoals();
          mbfclient_->ac_.cancelGoal();
          velmsg.linear.x=0;
          velPub.publish(velmsg);
          ROS_WARN("BT Failed becuase of cancell command from user");
          return BT::NodeStatus::FAILURE;
        }
        else{
          auto r=mbfclient_->move_to_goal();
          std::stringstream ss,ss1;

          if (r->outcome  == mbf_msgs::MoveBaseResult::SUCCESS && cancelflag==false)
          {

            if(mbfclient_->ns>=(mbfclient_->goalsize)){
            // scenario is successful and robot will publish successful scenario as a content of last topic
              ss << "successfuly reached to target number " << mbfclient_->ns;//last target reached
              msg.data = ss.str();
              NavfileResults_pub.publish(msg);

              ss1 << "successfuly finished scenario! with "<<mbfclient_->goalsize<< " targets" ;
              msg.data = ss1.str();
              NavfileResults_pub.publish(msg);

            }
            else {
              // target number [ns] is successfuly reached

              ss << "successfuly reached to target number " << mbfclient_->ns;
              msg.data = ss.str();
              NavfileResults_pub.publish(msg);
              mbfclient_->ac_.cancelAllGoals();
            }
            return BT::NodeStatus::SUCCESS;

        }
        else {
          // The scenario is failed, published failed topic by robot
          ss << "Failed to reach to target number " << (mbfclient_->np);//last target reached
          msg.data = ss.str();
          NavfileResults_pub.publish(msg);

          ss1 << "scenario Failed!";
          msg.data = ss1.str();
          NavfileResults_pub.publish(msg);
         }
        }
      }
      return BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_;
  ros::Publisher  NavfileResults_pub;

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
            ROS_INFO_STREAM("welldone got to the next position!!");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mbf_bt_v2");
  ros::NodeHandle n;

  Navfilename_sub   = n.subscribe("Navfilename", 10, Navfilename_callback);

  ros::Rate r(10);
  //auto mbfclient = std::make_shared<mbf_advanced::MBFCircleClient>(std::move(mbf_advanced::loadPoseGoals(POSE_PATHC)));
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GoToPose>("GoToPose");
  factory.registerNodeType<LookForObject>("LookForObject");
  //auto tree = factory.createTreeFromFile(BT_XML_PATHC);
  ROS_INFO("File Navigation Node is Running");

  while (ros::ok()){


    ROS_INFO("Robot is Waiting to receive the navigation file!");

    while(std::find(filenames.begin(), filenames.end(), filename) == filenames.end())
    {

      ros::spinOnce();
      r.sleep();
      //ROS_INFO("Waiting...");

    };

    ROS_INFO("Robot starting to excute one of scenarios");
    //delete tree
    if (filename=="A"){
      POSE_PATH=POSE_PATHA;
      BT_XML_PATH=BT_XML_PATHA;
      ROS_INFO("The name of scenario is : [%s]", filename.c_str());
    }
    else if(filename=="B"){
      POSE_PATH=POSE_PATHB;
      BT_XML_PATH=BT_XML_PATHB;
      ROS_INFO("The name of scenario is : [%s]", filename.c_str());
          }
    else if(filename=="C"){
      POSE_PATH=POSE_PATHC;
      BT_XML_PATH=BT_XML_PATHC;
      ROS_INFO("The name of scenario is : [%s]", filename.c_str());
          }
    else {
      for (int i=0;i<10;i++)
        ROS_INFO("There is problem! The file name wasn't mached with existing files, so a warning will be played!!");
        POSE_PATH=POSE_PATHWarning;
        BT_XML_PATH=BT_XML_PATHWarning;


    }
    auto mbfclient = std::make_shared<mbf_advanced::MBFCircleClient>(std::move(mbf_advanced::loadPoseGoals(POSE_PATH)));

    auto tree = factory.createTreeFromFile(BT_XML_PATH);

  /*????????????????????????????????????????????????????????????????????????*/
    for( auto& node: tree.nodes)
    {
        if( auto gotopose = dynamic_cast<GoToPose*>( node.get() ))
        {
            gotopose->attachMBFClient(mbfclient);
          //  std::cout<< AW<<"1\n";
          //  std::cout<< dynamic_cast<GoToPose*>( node.get() )<<"1\n";

        }

        if( auto lookforobject = dynamic_cast<LookForObject*>( node.get() ))
        {
            lookforobject->attachMBFClient(mbfclient);
           // std::cout<< AW<<"2\n";
          //  std::cout<< dynamic_cast<LookForObject*>( node.get() )<<"1\n";

        }

    }
  /*???????????????????????????????????????????????????????????????????????*/
    BT::PublisherZMQ publisher_zmq(tree);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    int c=0;
    ROS_INFO("loop %d",c);
    while (ros::ok() && status == BT::NodeStatus::RUNNING && !cancelflag) {
        status = tree.tickRoot();

        c++;
        ROS_INFO("loop %d",c);
        if (cancelflag){
          ROS_INFO("Cancel flag is seen 2! ");

          cancelflag=false;
          break;
        }
        //ros::Duration(5).sleep();
    }

    if(cancelflag)
      ROS_WARN("Robot is Stopped inside the scenario named [%s] ! ", filename.c_str());
    else
      ROS_INFO("Robot finished scenario named [%s] ! ", filename.c_str());

    filename="delete";
  }
  return 0;
}
