
#include "ObstacleDetector_laser.h"

ObstacleDetector_laser::ObstacleDetector_laser()
{
    //rosParam
    ros::NodeHandle privNh("~");
    //std::string string_value;
    //double double_value;
    //int int_val;
    std::string pub_obstacle_topic     ;
    std::string sub_scan_topic         ;
    //std::string pub_abb_obstacle_topic ;
    //std::string pub_rm_obstacle_topic  ;
    std::string map_frame              ;

    double angle_min              ;
    double angle_max              ;
    double y_min                  ;
    double y_max                  ;
    double save_dist              ;
    double dist_offset            ;
    int    min_laser_points       ;
    double drop_laser_points_fac  ;
    double min_obstacle_size      ;
    int id;

    //privNh.param("string_value",string_value,std::string("std_value"));
    //privNh.param<double>("double_value",double_value, 12.34);
    //privNh.param<int>("int_val",int_val, 1234);
    privNh.param("pub_obstacle_topic"       , pub_obstacle_topic        , std::string("obstacle"     ));
    privNh.param("sub_scan_topic"           , sub_scan_topic            , std::string("scan"         ));
    privNh.param("map_frame"                , map_frame                 , std::string("map"          ));

    privNh.param<double>("angle_min"             ,angle_min               , -1.3 );
    privNh.param<double>("angle_max"             ,angle_max               , 1.3  );
    privNh.param<double>("y_min"                 ,y_min                   , -0.25);
    privNh.param<double>("y_max"                 ,y_max                   , 0.25 );
    privNh.param<double>("save_dist"             ,save_dist               , 0.4  );
    privNh.param<double>("dist_offset"           ,dist_offset             , -0.01);
    privNh.param<int>   ("min_laser_points"      ,min_laser_points        , 15   );
    privNh.param<double>("drop_laser_points_fac" ,drop_laser_points_fac   , 0.35 );
    privNh.param<double>("min_obstacle_size"     ,min_obstacle_size       , 0.3  );
    privNh.param<int>("robot_id"     ,id       , 0  );

    _id = id;

    //init publisher
    //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);
   _pubObstacle = _nh.advertise<geometry_msgs::PolygonStamped>(pub_obstacle_topic,1);

    //inti subscriber
    //_sub = _nh.subscribe("topicName", 1, &Template::subCallback, this);
   _subScan = _nh.subscribe(sub_scan_topic, 1, &ObstacleDetector_laser::subLaserScan_callback, this);
   _subMap  = _nh.subscribe("/map", 1, &ObstacleDetector_laser::subMap_callback, this);

   //hack
//   _subPose0 = _nh.subscribe("/robot0/pose", 1, &ObstacleDetector_laser::subPose0_callback, this);
//   _subPose1 = _nh.subscribe("/robot1/pose", 1, &ObstacleDetector_laser::subPose1_callback, this);
//   _subPose2 = _nh.subscribe("/robot2/pose", 1, &ObstacleDetector_laser::subPose2_callback, this);
//   _subPose3 = _nh.subscribe("/robot3/pose", 1, &ObstacleDetector_laser::subPose3_callback, this);
//   _subPose4 = _nh.subscribe("/robot4/pose", 1, &ObstacleDetector_laser::subPose4_callback, this);
//   _subPose5 = _nh.subscribe("/robot5/pose", 1, &ObstacleDetector_laser::subPose5_callback, this);

   _robotPoses.resize(6);
   _robotPosed_valid.resize(6,false);



   _rad_min = angle_min;
   _rad_max = angle_max;
   _y_min = y_min;
   _y_max = y_max;
   _saveDist = save_dist;
   _dist_offset = dist_offset;
   _min_obstacle_laser_points = min_laser_points; // min 10 - 15
   _drop_min_points_factor = drop_laser_points_fac;
   _min_obstacle_size = min_obstacle_size;

   _map_frame = map_frame;
}

ObstacleDetector_laser::~ObstacleDetector_laser()
{
}

void ObstacleDetector_laser::start(const double duration)
{
   //init timer
   //_loopTimer = _nh.createTimer(ros::Duration(duration), &ObstacleDetector_laser::timer_loopCallback, this);
   ros::spin();
}

void ObstacleDetector_laser::timer_loopCallback(const ros::TimerEvent& e)
{
   if(!ros::ok())
   {
      ROS_INFO("ros::ok() -> false");
      exit(EXIT_SUCCESS);
   }

   //do loop stuff here...
   //ROS_INFO("blabla");
   //pub...
   //_pub.publish(..)



}

void ObstacleDetector_laser::subLaserScan_callback(const sensor_msgs::LaserScan& msg)
{
   //std::cout << "msg.stamp: " << msg.header.stamp << std::endl;
   //std::cout << "now      : " << ros::Time::now() << std::endl;

   std::vector<rona::map::Point2D> points;
   for(unsigned int i=0; i<msg.ranges.size(); ++i)
   {
      //index to rad
      double rad = i * msg.angle_increment + msg.angle_min;
      if(rad < _rad_min || rad > _rad_max)
      {
         continue;
      }
      //in rad range
      rona::map::Point2D p;
      p.x = msg.ranges[i] * ::cos(rad);
      p.y = msg.ranges[i] * ::sin(rad);

      points.push_back(p);
   }
   //this->pub_rect_marker(points, 0.05);

   //detect critical points
   std::vector<rona::map::Point2D> crit_points;

   //std::cout << "--------------------------------------" << std::endl;
   //for(unsigned int i=0; i<points.size(); ++i)
   for(auto e : points)
   {
      if(std::abs(e.x) < 0.01 && std::abs(e.y) < 0.01)
         continue;
      if(e.y >= _y_min && e.y <= _y_max && e.x < _saveDist)
      {
         //std::cout << "x: " << e.x << " ,y: " << e.y << std::endl;
         crit_points.push_back(e);
      }
   }

   if(crit_points.size() < _min_obstacle_laser_points) //noting crit ... -> nothing to do :D
      return;


   //sort minx first. -> lambda cool :D
   std::sort(crit_points.begin(), crit_points.end(), [](const rona::map::Point2D& l, const rona::map::Point2D& r) -> bool{ return (l.x < r.x);});

//   debug
//   std::cout << "after sort num: " << crit_points.size() << std::endl;
//   for(auto e : crit_points)
//   {
//      std::cout << "e: " << e.x << std::endl;
//   }

   //remove num_drop min values from crit_points
   unsigned int num_drop = _min_obstacle_laser_points * _drop_min_points_factor + 0.5;
   crit_points.erase(crit_points.begin(), crit_points.begin() + num_drop);


   //find min x and minmax y;
   double min_x = crit_points[0].x;//9999999;
   double min_y = 9999999;
   double max_y = 0;
   for(auto e : crit_points)
   {
      //if(e.x < min_x)
      //   min_x = e.x;
      if(e.y < min_y)
         min_y = e.y;
      if(e.y > max_y)
         max_y = e.y;
   }

   double scale_raw = std::abs(max_y - min_y);
   double scale = scale_raw < _min_obstacle_size ? _min_obstacle_size : scale_raw;

   double center_y = (min_y + max_y) * 0.5;
   rona::map::Polygon polygon;
   rona::map::Point2D p1;
   rona::map::Point2D p2;
   rona::map::Point2D p3;
   rona::map::Point2D p4;

   p1.x = min_x + _dist_offset;
   p1.y = center_y + (scale * 0.5);
   p2.x = min_x + _dist_offset;
   p2.y = center_y - (scale * 0.5);
   p3.x = min_x + scale + _dist_offset;
   p3.y = center_y - (scale * 0.5);
   p4.x = min_x + scale + _dist_offset;
   p4.y = center_y + (scale * 0.5);

   //important ... coutner clockwise
   polygon.points.push_back(p1);
   polygon.points.push_back(p2);
   polygon.points.push_back(p3);
   polygon.points.push_back(p4);




   //debug:
   //test point in polygon
//   rona::map::Point2D tmp_p;
//   tmp_p.x = 0.3;
//   tmp_p.y = 0.2;
//   bool tmp = rona::map::Operations::pointInPolygon(polygon, tmp_p);
//   std::cout << "p in polygon: " << (tmp ? "true" : "false") << std::endl;

   geometry_msgs::PolygonStamped ros_pol = rona::Utility::toRosPolygon(polygon, msg.header.frame_id);
   ros_pol.header.stamp = msg.header.stamp;// = ros::Time::now();
   //transform to map/world..
   geometry_msgs::PolygonStamped ros_pol_out = rona::Utility::transformPoylgon(_tf_listener, _map_frame, ros_pol);
   //_tf_listener.transformP
   //publish

   if(!_map)
   {
      ROS_WARN("Got no map until now... do nothing");
      return;
   }

   //compute polycenter and prove if is a wall or a robot
   rona::map::Point2D polyc = rona::map::Operations::computeCentroid(rona::Utility::toPolygon(ros_pol_out));


   unsigned int idx = _map->toIdx(polyc);
   if(_map->getData()[idx])
   {//occupied
      ROS_INFO("Current Obstacle is occupied -> must be Wall");
      return;
   }
   //prove if other robot...
   for(unsigned int i=0; i<6; ++i)
   {
      if(rona::map::Operations::computeDistance(_robotPoses[i], polyc) < 0.3)//todo check if id needed
      {
         ROS_INFO("Current Obstacle is in robot -> must be robot");
         return;
      }
   }


   _pubObstacle.publish(ros_pol_out);
}


void ObstacleDetector_laser::subMap_callback(const nav_msgs::OccupancyGrid& msg)
{
   _map = std::shared_ptr<rona::map::Grid>(new rona::map::Grid(msg));

   rona::map::Operations::inflateCirc(_map, 10, 127, 0.3);
   rona::map::Operations::binarize(_map, 0, 1, 0, 255);

}

void ObstacleDetector_laser::subPose0_callback(
      const geometry_msgs::PoseStamped& msg)
{
   _robotPosed_valid[0] = true;
   _robotPoses[0] = rona::Utility::toPoint2D(msg);
}

void ObstacleDetector_laser::subPose1_callback(
      const geometry_msgs::PoseStamped& msg)
{
   _robotPosed_valid[1] = true;
   _robotPoses[1] = rona::Utility::toPoint2D(msg);
}

void ObstacleDetector_laser::subPose2_callback(
      const geometry_msgs::PoseStamped& msg)
{
   _robotPosed_valid[2] = true;
   _robotPoses[2] = rona::Utility::toPoint2D(msg);
}

void ObstacleDetector_laser::subPose3_callback(
      const geometry_msgs::PoseStamped& msg)
{
   _robotPosed_valid[3] = true;
   _robotPoses[3] = rona::Utility::toPoint2D(msg);
}

void ObstacleDetector_laser::subPose4_callback(
      const geometry_msgs::PoseStamped& msg)
{
   _robotPosed_valid[4] = true;
   _robotPoses[4] = rona::Utility::toPoint2D(msg);
}

void ObstacleDetector_laser::subPose5_callback(
      const geometry_msgs::PoseStamped& msg)
{
   _robotPosed_valid[5] = true;
   _robotPoses[5] = rona::Utility::toPoint2D(msg);
}


//void ObstacleDetector_laser::pub_rect_marker(std::vector<rona::map::Point2D> p, double scale)
//{
////   //remove old marker
////   for(unsigned int i=0; i<_oldMarker.markers.size(); ++i)
////   {
////      _oldMarker.markers[i].action = _oldMarker.markers[i].DELETE;
////   }
////   _pubMarker.publish(_oldMarker);
//
//   visualization_msgs::MarkerArray marker;
//
//   geometry_msgs::Vector3 scale_m;
//   scale_m.x = scale;
//   scale_m.y = scale;
//   scale_m.z = scale;
//   std_msgs::ColorRGBA color;
//
//
//   color.r = 1.0;
//   color.g = 0.5;
//   color.b = 0.0;
//   color.a = 0.5;
//
//   for(unsigned int i=0; i<p.size(); ++i)
//   {
//      visualization_msgs::Marker m;
//
//      //to tf
//      geometry_msgs::PoseStamped pose_in = rona::Utility::toRosPoseStamped(p[i],"laser");
//      geometry_msgs::PoseStamped pose_out;
//
//
//      try{
//      _tf_listener.transformPose("map",pose_in, pose_out);
//      } catch (tf::TransformException& e) {
//         ROS_ERROR("exeption at tranformpose: %s", e.what());
//      }
//
//      m.header.frame_id = "map";
//      m.id = i;
//      m.type = m.CUBE;
//      m.action = m.ADD;
//      m.pose = pose_out.pose;  //orientation not used
//      m.scale = scale_m;
//      m.color = color;
//
//      marker.markers.push_back(m);
//   }
//
//   for(unsigned int i=p.size(); i<_oldMarker.markers.size(); ++i)
//   {
//      _oldMarker.markers[i].action = _oldMarker.markers[i].DELETE;
//      marker.markers.push_back(_oldMarker.markers[i]);
//   }
//   _pubMarker.publish(marker);
//   _oldMarker = marker;
//}

//------------------------------------------------------------------------------
//-- main --
//----------


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rona_obstacle_detector_node");
    ros::NodeHandle nh("~");

    ObstacleDetector_laser node;
    node.start(0.1);

}


