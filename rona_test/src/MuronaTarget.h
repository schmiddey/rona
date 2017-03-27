
#ifndef MURONATARGET_H_
#define MURONATARGET_H_

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class MuronaTarget
{
private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pubT0;
    ros::Publisher _pubT1;
    ros::Publisher _pubT2;
    ros::Publisher _pubT3;
    ros::Publisher _pubT4;
    ros::Publisher _pubT5;

    geometry_msgs::PoseStamped p0;
    geometry_msgs::PoseStamped p1;
    geometry_msgs::PoseStamped p2;
    geometry_msgs::PoseStamped p3;
    geometry_msgs::PoseStamped p4;
    geometry_msgs::PoseStamped p5;

    ros::Timer _loopTimer;
public:
    MuronaTarget();
    virtual ~MuronaTarget();

    /**
     * @fn void start(const double duration = 0.1)
     *
     * @brief starts ros::spin(), with timercallback-fct as loop
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

    geometry_msgs::PoseStamped toPS(double x, double y);

    //void timerLoop_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
};

#endif /* MURONATARGET_H_ */
