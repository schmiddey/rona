
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

  _sub_map = _nh.subscribe("map", 1, &GridTest::sub_map_callback, this);

  _grid = std::make_shared<rona::map::Grid>(map_yaml);

  cv::Mat grid_cv = _grid->toCvMat();
  cv::imshow("hans2", grid_cv);
  cv::waitKey(100);
  cv::imwrite("/tmp/map.png", grid_cv);
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


