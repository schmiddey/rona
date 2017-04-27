
#ifndef EXPLORATIONNODE_H_
#define EXPLORATIONNODE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>

//dyn reconfig
//#include <dynamic_reconfigure/server.h>
//#include <template/TemplateConfig.h>

class ExplorationNode
{

public:
  ExplorationNode();
    virtual ~ExplorationNode();

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

    ros::Publisher _pubTarget;
    ros::Publisher _pubNodeCtrl_frontier;
    ros::Subscriber _subState;
    ros::Subscriber _subFrontiers;
    ros::Subscriber _subNodeCtrl_own; //start //pause ....

    //dynamic_reconfigure::Server<rona_frontier::ExplorationConfig> _drServer;

    ros::Timer _loopTimer;
};

#endif /* EXPLORATIONNODE_H_ */
