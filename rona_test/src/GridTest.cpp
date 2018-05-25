
#include "MuronaTarget.h"

MuronaTarget::MuronaTarget()
{
    //rosParam
    ros::NodeHandle privNh("~");
    //std::string string_value;
    //double double_value;
    //int int_val;

   double r0x;
   double r1x;
   double r2x;
   double r3x;
   double r4x;
   double r5x;
   double r0y;
   double r1y;
   double r2y;
   double r3y;
   double r4y;
   double r5y;

    //privNh.param("string_value",string_value,std::string("std_value"));
    //privNh.param<double>("double_value",double_value, 12.34);
    //privNh.param<int>("int_val",int_val, 1234);
   privNh.param<double>("r0x",r0x, 12.34);
   privNh.param<double>("r1x",r1x, 12.34);
   privNh.param<double>("r2x",r2x, 12.34);
   privNh.param<double>("r3x",r3x, 12.34);
   privNh.param<double>("r4x",r4x, 12.34);
   privNh.param<double>("r5x",r5x, 12.34);
   privNh.param<double>("r0y",r0y, 12.34);
   privNh.param<double>("r1y",r1y, 12.34);
   privNh.param<double>("r2y",r2y, 12.34);
   privNh.param<double>("r3y",r3y, 12.34);
   privNh.param<double>("r4y",r4y, 12.34);
   privNh.param<double>("r5y",r5y, 12.34);


    //init publisher
    //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);

   _pubT0 = _nh.advertise<geometry_msgs::PoseStamped>("robot0/target",1);
   _pubT1 = _nh.advertise<geometry_msgs::PoseStamped>("robot1/target",1);
   _pubT2 = _nh.advertise<geometry_msgs::PoseStamped>("robot2/target",1);
   _pubT3 = _nh.advertise<geometry_msgs::PoseStamped>("robot3/target",1);
   _pubT4 = _nh.advertise<geometry_msgs::PoseStamped>("robot4/target",1);
   _pubT5 = _nh.advertise<geometry_msgs::PoseStamped>("robot5/target",1);



   p0 = this->toPS(r0x, r0y);
   p1 = this->toPS(r1x, r1y);
   p2 = this->toPS(r2x, r2y);
   p3 = this->toPS(r3x, r3y);
   p4 = this->toPS(r4x, r4y);
   p5 = this->toPS(r5x, r5y);
}

MuronaTarget::~MuronaTarget()
{
}

void MuronaTarget::start(const double duration)
{
   std::string i;
   std::cin >> i;
   ROS_INFO("Pub Targets");
   _pubT0.publish(p0);
   _pubT1.publish(p1);
   _pubT2.publish(p2);
   _pubT3.publish(p3);
   _pubT4.publish(p4);
   _pubT5.publish(p5);
   ROS_INFO("Pub Targets -> RDY");
   //init timer
   ros::spin();
}

geometry_msgs::PoseStamped MuronaTarget::toPS(double x, double y)
{
   geometry_msgs::PoseStamped pose;

   pose.pose.position.x = x;
   pose.pose.position.y = y;
   pose.pose.position.z = 0;
   pose.header.frame_id = "map";

   return pose;
}

//void Template::subCallback(const ROS_PACK::MESSAGE& msg)
//{
//}



//------------------------------------------------------------------------------
//-- main --
//----------


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rona_test_murona_target_node");
    ros::NodeHandle nh("~");

    MuronaTarget node;
    node.start(0.1);

}


