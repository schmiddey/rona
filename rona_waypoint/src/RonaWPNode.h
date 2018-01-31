
#ifndef RONAWPNODE_H_
#define RONAWPNODE_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

class RonaWPNode
{

public:
  RonaWPNode();
    virtual ~RonaWPNode();

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


    void loop_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
    void sub_clicked_point_callback(const geometry_msgs::PointStamped& p);
private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pub;
    ros::Subscriber _sub_clicked_point;

    ros::Timer _loopTimer;
};

#endif /* RONAWPNODE_H_ */
