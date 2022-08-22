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
std::vector<std::string> filenames={"A","B","C"};
std::string filename="nothing";
std::string POSE_PATH;
std::string BT_XML_PATH;

void Navfilename_callback(const std_msgs::String::ConstPtr& msg)
{
  filename=msg->data;
  ROS_INFO("I heard: [%s]", filename.c_str());
}
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mbf_bt_v1");
  ros::NodeHandle n;
  Navfilename_sub= n.subscribe("Navfilename", 10, Navfilename_callback);
  ros::Rate r(10);
  //auto mbfclient = std::make_shared<mbf_advanced::MBFCircleClient>(std::move(mbf_advanced::loadPoseGoals(POSE_PATHC)));
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<GoToPose>("GoToPose");
  factory.registerNodeType<LookForObject>("LookForObject");
  //auto tree = factory.createTreeFromFile(BT_XML_PATHC);
  while (ros::ok()){



    ROS_INFO("Waiting to recieve the file name!");
    ROS_INFO("Waiting to recieve the file name!");
    ROS_INFO("Waiting to recieve the file name!");
    ROS_INFO("Waiting to recieve the file name!");

    while(std::find(filenames.begin(), filenames.end(), filename) == filenames.end())
    {

      ros::spinOnce();
      r.sleep();
      ROS_INFO("Waiting...");

    };

    ROS_INFO("Pass waiting loop!");
    ROS_INFO("Pass waiting loop!");
    ROS_INFO("Pass waiting loop!");
    ROS_INFO("Pass waiting loop!");
    //delete tree
    if (filename=="A"){
      POSE_PATH=POSE_PATHA;
      BT_XML_PATH=BT_XML_PATHA;
      ROS_INFO("1 The file name was : [%s]", filename.c_str());
    }
    else if(filename=="B"){
      POSE_PATH=POSE_PATHB;
      BT_XML_PATH=BT_XML_PATHB;
      ROS_INFO("2 The file name was : [%s]", filename.c_str());
          }
    else if(filename=="C"){
      POSE_PATH=POSE_PATHC;
      BT_XML_PATH=BT_XML_PATHC;
      ROS_INFO("3 The file name was : [%s]", filename.c_str());
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
    while (status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();


        //ros::Duration(5).sleep();
    }
    filename="delete";
    ROS_INFO("Finished mission file [%s] ! ", filename.c_str());
}


  return 0;
}
