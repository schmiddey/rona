
#ifndef RONAWPCLICKNODE_H_
#define RONAWPCLICKNODE_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "WayPointHandler.h"


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


private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pub_wp_path;
    ros::Publisher _pub_marker;
    ros::Subscriber _sub_clicked_point;
    ros::Subscriber _sub_estimate_pose;

    ros::Timer _loopTimer;

    WayPointHandler _wp_handler;
};

#endif /* RONAWPCLICKNODE_H_ */
