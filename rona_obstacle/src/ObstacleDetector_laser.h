
#ifndef OBSTACLEDETECTOR_LASER_H_
#define OBSTACLEDETECTOR_LASER_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>


#include <rona_lib/Map/Grid.h>
#include <rona_lib/Utility.h>
#include <rona_lib/Map/Operations.h>





/**
 * @todo compare obstacle with map... if is wann then drop
 */
class ObstacleDetector_laser
{
private:    //dataelements
    ros::NodeHandle _nh;

    //ros::Publisher _pub;
    //ros::Publisher _pubMarker;
    ros::Publisher _pubObstacle;
    ros::Subscriber _subScan;
    ros::Subscriber _subMap;

    //hack
    ros::Subscriber _subPose0;
    ros::Subscriber _subPose1;
    ros::Subscriber _subPose2;
    ros::Subscriber _subPose3;
    ros::Subscriber _subPose4;
    ros::Subscriber _subPose5;
    std::vector<rona::map::Point2D> _robotPoses;
    std::vector<bool> _robotPosed_valid;
    unsigned int _id;


    ros::Timer _loopTimer;

    tf::TransformListener _tf_listener;

    double _rad_min;
    double _rad_max;

    double _y_min;
    double _y_max;

    double _saveDist;
    double _dist_offset;

    double _min_obstacle_laser_points;
    double _drop_min_points_factor;

    double _min_obstacle_size;

    std::string _map_frame;

    std::shared_ptr<rona::map::Grid> _map;


    //visualization_msgs::MarkerArray _oldMarker;
public:
    ObstacleDetector_laser();
    virtual ~ObstacleDetector_laser();

    /**
     * @fn void start(const unsigned int frames = 10)
     *
     * @brief
     *
     *
     * @param[in] duration  ->  duration of the working loop in [s] -> 1/rate
     *
     *
     * @return  void
     */
    void start(const double duration = 0.1);

private:    //functions

   /**
    * @brief loop callback function
    *
    * @param e
    */
    void timer_loopCallback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
    void subLaserScan_callback(const sensor_msgs::LaserScan& msg);

    void subMap_callback(const nav_msgs::OccupancyGrid& msg);
    //void pub_rect_marker(std::vector<rona::map::Point2D> p, double scale);

    //hack
    void subPose0_callback(const geometry_msgs::PoseStamped& msg);
    void subPose1_callback(const geometry_msgs::PoseStamped& msg);
    void subPose2_callback(const geometry_msgs::PoseStamped& msg);
    void subPose3_callback(const geometry_msgs::PoseStamped& msg);
    void subPose4_callback(const geometry_msgs::PoseStamped& msg);
    void subPose5_callback(const geometry_msgs::PoseStamped& msg);

};

#endif /* OBSTACLEDETECTOR_LASER_H_ */
