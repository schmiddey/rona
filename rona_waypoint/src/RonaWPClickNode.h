
#ifndef RONAWPCLICKNODE_H_
#define RONAWPCLICKNODE_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

#include <rona_msgs/PlanPath.h>

#include "WayPointHandler.h"

#include <rona_lib/marker/MarkerUtility.h>

namespace cfg{

struct WPClick_cfg{
  double step_length = 0.05;
  std::string frame_id = "map";
};

}

/**
 * @todo set ori via setNavGoal...
 */
class RonaWPClickNode
{

public:
  RonaWPClickNode();
    virtual ~RonaWPClickNode();

    /**
     *
     * @brief
     *
     * @return  void
     */
    void start(double duration = 0.01);

private:    //functions

    /**
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void run();


    void publish_waypoints();

    void loop_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
    void sub_clicked_point_callback(const geometry_msgs::PointStamped& p);

    // for removing last wp ... maybe a hack :)
    void sub_estimate_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& pose);

    void sub_map_callback(const nav_msgs::OccupancyGridPtr map);

    std::pair<bool, nav_msgs::Path> compute_direct_path(const geometry_msgs::Point& start, const geometry_msgs::Point& end);

    //compute path via sirona via service....
    nav_msgs::Path compute_path(const geometry_msgs::Point& start, const geometry_msgs::Point& end);

    visualization_msgs::MarkerArray toMarkerArray(const WayPointHandler& wp_handler);

private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pub_wp_path;
    ros::Publisher _pub_marker;
    ros::Subscriber _sub_clicked_point;
    ros::Subscriber _sub_estimate_pose;
    ros::Subscriber _sub_map;

    ros::ServiceClient _srv_plan_path;

    ros::Timer _loopTimer;

    WayPointHandler _wp_handler;

    nav_msgs::OccupancyGridPtr _map;

    geometry_msgs::Quaternion _orientation;

    cfg::WPClick_cfg _cfg;
};

#endif /* RONAWPCLICKNODE_H_ */
