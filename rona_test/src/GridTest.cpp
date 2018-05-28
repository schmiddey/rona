
#include "GridTest.h"

GridTest::GridTest()
{
  //rosParam
  ros::NodeHandle privNh("~");
  //std::string string_value;
  //double double_value;
  //int int_val;
  std::string map_yaml;
  privNh.param(         "map_yaml" ,    map_yaml,   std::string("map.yaml"));
  std::cout << "yaml: " << map_yaml << std::endl;

  _grid = std::make_shared<rona::map::Grid>(map_yaml);

  ROS_INFO("Test RDY");

}

GridTest::~GridTest()
{
}

void GridTest::start(const double duration)
{
   //init timer
   ros::spin();
}

//void Template::subCallback(const ROS_PACK::MESSAGE& msg)
//{
//}



//------------------------------------------------------------------------------
//-- main --
//----------


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rona_test_grid_node");
    ros::NodeHandle nh("~");

    GridTest node;
    node.start(0.1);

}


