
#ifndef RONAWPEXECUTE_H_
#define RONAWPEXECUTE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>

#include <rona_msgs/NodeCtrl.h>

#include <rona_lib/Map/Operations.h>

#include "WayPointHandler.h"

#include <string>

#include <fstream>

namespace cfg {

struct RonaWPExecute_cfg{
  std::string wp_file_load = "";
  double wait_duration_s   = 5.0;
  bool do_loop             = true;
};

}  // namespace cfg

namespace state {

enum enum_state{
  IDLE = 0,
  WAIT,
  MOVE,
  ARRIVED,
  STOP
};

inline const static std::string toString(const enum_state state)
{
  switch (state) {
    case IDLE:
      return "IDLE";
    case WAIT:
      return "WAIT";
    case MOVE:
      return "MOVE";
    case ARRIVED:
      return "ARRIVED";
    case STOP:
      return "STOP";
    default:
      return "INVALID, ERROR!!!!";
  }
}

}  // namespace state

class RonaWPExecute
{

public:
  RonaWPExecute();
    virtual ~RonaWPExecute();

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

    void loop_marker_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);

    void sub_move_state_callback(const std_msgs::Bool& msg);

    void sub_node_ctrl_callback(const rona_msgs::NodeCtrl& msg);

    void sub_load_wp_callback(const std_msgs::String& msg);

    int next_wp()
    {
      if(++_curr_wp_id >= _wp_handler.size())
      {
        _curr_wp_id = 0;
        if(!_cfg.do_loop)
        {
          ROS_INFO("STOP WP execute... Restart with nodectrl start");
          _state = state::STOP;
        }
      }
      return _curr_wp_id;
    }

    void pub_wp_path(const int id)
    {
      _pub_path.publish(_wp_handler.at(id).second);
    }

private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pub_path;
    ros::Publisher _pub_state;
    ros::Publisher _pub_marker;
    ros::Subscriber _sub_move_state;
    ros::Subscriber _sub_node_ctrl;
    ros::Subscriber _sub_load_wp;

    ros::Timer _loopTimer;
    ros::Timer _loopMarkerTimer;
    cfg::RonaWPExecute_cfg _cfg;

    WayPointHandler _wp_handler;

    int _curr_wp_id;

    state::enum_state _state;

    bool _last_move_state;

    bool _stop_at_next_wp;

    double _covered_dist;

    ros::Time _wait_time;
};

#endif /* RONAWPEXECUTE_H_ */
