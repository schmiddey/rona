
#ifndef PUBPOLY_H_
#define PUBPOLY_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
//dyn reconfig
#include <dynamic_reconfigure/server.h>
//#include <template/TemplateConfig.h>

class Template
{

public:
    Template();
    virtual ~Template();

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

    //void dynreconfig_callback(template::TemplateConfig &config, uint32_t level);
private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pub;
    ros::Subscriber _sub;

    dynamic_reconfigure::Server<rona_frontier::ExplorationConfig> _drServer;

    ros::Timer _loopTimer;
};

#endif /* PUBPOLY_H_ */
