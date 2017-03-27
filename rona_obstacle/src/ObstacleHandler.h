
#ifndef OBSTACLEHANDLER_H_
#define OBSTACLEHANDLER_H_

#include <iostream>
#include <vector>
#include <memory>
#include <limits>

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

#include <rona_lib/Map/Operations.h>
#include <rona_lib/Timer.h>

#include "Obstacle/ObstacleEventHandler.h"
#include "Obstacle/ObstacleHandle.h"

class ObstacleHandler
{
private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pubPolygon;
    ros::Publisher _pubMarker;
    ros::Subscriber _sub;

    ros::Timer _loopTimer;

    std::shared_ptr<rona::ObstacleEventHandler> _eventHandler;

    std::vector<std::unique_ptr<rona::ObstacleHandle>> _obstacleHandles;

    visualization_msgs::MarkerArray _marker;

    double _robot_radius;
    double _duration_valid;
    std::string _map_frame;

public:
    ObstacleHandler();
    virtual ~ObstacleHandler();

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
    void timerLoop_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
    void subCallbackObstacle(const geometry_msgs::PolygonStamped& msg);
};

#endif /* OBSTACLEHANDLER_H_ */
