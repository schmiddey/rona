
#ifndef RONAWPCLICKNODE_H_
#define RONAWPCLICKNODE_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

#include <rona_msgs/PlanPath.h>

#include "WayPointHandler.h"

#include <rona_lib/marker/MarkerUtility.h>
#include <rona_lib/Utility.h>

namespace cfg{

struct WPClick_cfg{
  double step_length = 0.05;
  std::string frame_id = "map";
  std::string wp_file_save = "/tmp/waypoints.txt";
  std::string wp_file_load = "";
  bool save_at_exit = true;
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
    
    void add_waypoint(const geometry_msgs::Pose& p);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
    void sub_clicked_point_callback(const geometry_msgs::PointStamped& p);

    // for removing last wp ... maybe a hack :)
    void sub_estimate_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& pose);

    void sub_nav_goal_callback(const geometry_msgs::PoseStamped& pose);

    bool srv_save_wp_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    bool srv_set_curr_tf_pose_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    // std::pair<bool, nav_msgs::Path> compute_direct_path(const geometry_msgs::Point& start, const geometry_msgs::Point& end, const geometry_msgs::Quaternion& ori);

    //compute path via sirona via service....
    nav_msgs::Path compute_path(const geometry_msgs::Point& start, const geometry_msgs::Point& end);

//    visualization_msgs::MarkerArray toMarkerArray(const WayPointHandler& wp_handler);

private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pub_wp_path;
    ros::Publisher _pub_marker;

    ros::Subscriber _sub_clicked_point;
    ros::Subscriber _sub_estimate_pose;
    ros::Subscriber _sub_nav_goal;

    ros::ServiceServer _srv_save;
    ros::ServiceServer _srv_set_curr_tf_pose;

    ros::ServiceClient _srv_plan_path;

    ros::Timer _loopTimer;

    tf::TransformListener _tf_listener;
    std::string _map_frame;
    std::string _robot_frame;

    WayPointHandler _wp_handler;

    geometry_msgs::Quaternion _orientation;

    cfg::WPClick_cfg _cfg;
};

#endif /* RONAWPCLICKNODE_H_ */
