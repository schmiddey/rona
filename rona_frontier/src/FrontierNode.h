
#ifndef FRONTIERNODE_H_
#define FRONTIERNODE_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>

#include <rona_lib/exploration/Frontier.h>
#include <rona_lib/exploration/FrontierFinder.h>
#include <rona_lib/exploration/Visualization.h>
#include <rona_lib/exploration/FrontierController.h>

#include <rona_msgs/NodeCtrl.h>

//dyn reconfig
#include <dynamic_reconfigure/server.h>
#include <rona_frontier/FrontierConfig.h>



namespace frontier{
enum enumMode{
   RUN = 0,
   SINGLESHOT,
   STOP
};
}

class FrontierNode
{

public:
  FrontierNode();
    virtual ~FrontierNode();

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

    /**
     * Function to search for frontiers
     */
    void findFrontiers(void);
    /**
     * Function to publish frontiers
     */
    void publishFrontiers(void);

    void loop_callback(const ros::TimerEvent& e);

    void sub_node_ctrl_callback(const rona_msgs::NodeCtrl& msg);
    void sub_map_callback(const nav_msgs::OccupancyGrid& msg);

    void dynreconfig_callback(rona_frontier::FrontierConfig &config, uint32_t level);

private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pubFrontier;
    ros::Publisher _pubFronteirGrid;

    ros::Subscriber _subMap;
    ros::Subscriber _subNodeCtrl;

    dynamic_reconfigure::Server<rona_frontier::FrontierConfig> _drServer;

    ros::Timer _loopTimer;

    rona::frontier::Finder _frontierFinder;
    rona::FrontierController _frontierController;
    std::vector<rona::WeightedFrontier>  _frontiers;

    frontier::enumMode               _mode;
};

#endif /* FRONTIERNODE_H_ */
