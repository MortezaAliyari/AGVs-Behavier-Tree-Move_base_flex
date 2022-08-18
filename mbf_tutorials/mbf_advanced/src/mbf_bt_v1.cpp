#include <fstream>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mbf_advanced/mbf_circle_client2.h>

///
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
///
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mbf_bt_v1");
    ros::NodeHandle n;

    auto mbfclient = std::make_shared<mbf_advanced::MBFCircleClient>(std::move(mbf_advanced::loadPoseGoals(POSE_PATH2)));

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<GoToPose>("GoToPose");
    factory.registerNodeType<LookForObject>("LookForObject");
    auto tree = factory.createTreeFromFile(BT_XML_PATH2);
//????????????????????????????????????????????????????????????????????????
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
//???????????????????????????????????????????????????????????????????????/
    BT::PublisherZMQ publisher_zmq(tree);
    int c=0;
    std::cout<< AW<<" last line"<<std::endl;
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();
        if (status!=BT::NodeStatus::RUNNING)
            c++;
        std::cout<<"count : "<<c<<", "<<status<<std::endl;
        //ros::Duration(5).sleep();
    }

    return 0;
}
