
#ifndef GRIDTEST_H_
#define GRIDTEST_H_




#include <iostream>
#include <string>
#include <rona_lib/Map/Grid.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class GridTest
{
private:    //dataelements
    ros::NodeHandle _nh;
    std::shared_ptr<rona::map::Grid> _grid;
    ros::Timer _loopTimer;
public:
    GridTest();
    virtual ~GridTest();

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

    //void timerLoop_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
};

#endif  //GRIDTEST_H_
