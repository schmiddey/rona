
#ifndef MURONASIMPLENODE_H_
#define MURONASIMPLENODE_H_

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <stdexcept>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include <visualization_msgs/MarkerArray.h>

#include <rona_msgs/RobotMotion.h>
#include <rona_msgs/State.h>
#include <rona_msgs/NodeCtrl.h>
#include <rona_msgs/Obstacle.h>

#include <rona_lib/Map/GridMap.h>
#include <rona_lib/Map/Operations.h>
#include <rona_lib/Map/Object/MapObjectbase.h>
#include <rona_lib/Map/Object/RobotObject.h>
#include <rona_lib/Map/Object/PathObject.h>
#include <rona_lib/Motion/MotionPredictor.h>
#include <rona_lib/Motion/ConflictAnalyser.h>
#include <rona_lib/Planner/AStar.h>

#include <rona_lib/Utility.h>
#include <rona_lib/Timer.h>


struct MapObject{
   unsigned id;
   rona::map::Object type;
   bool operator<(const MapObject& m) const {
      if(this->id < m.id) return true;
      if(this->id > m.id) return false;
      return this->type < m.type;
   }
   //MapObject(unsigned int id_, rona::map::Object type_) : id(id_), type(type_) { }
};



class MuronaSimpleNode
{
private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pubMotion;         ///< to other robots
    ros::Publisher _pubPathMove;       ///< to path controller
    ros::Publisher _pubState;
    ros::Publisher _pubMoveCtrl;
    ros::Publisher _pubMarker;

    ros::Subscriber _subTarget;
    ros::Subscriber _subMap;
    ros::Subscriber _subMotion;
    ros::Subscriber _subMoveState;
    ros::Subscriber _subMoveProcess;
    ros::Subscriber _subAddOb;
    ros::Subscriber _subRmOb;


    ros::Timer _timerRepubMotion;
    ros::Duration _refreshMotion;

    tf::TransformListener _tf_listnener;

    std::unique_ptr<rona::planner::AStar> _planner;
    std::shared_ptr<rona::map::GridMap> _map;
    std::shared_ptr<rona::map::GridMap> _currMap;

    std::unique_ptr<rona::motion::MotionPredictor> _motionPredictor;
    std::unique_ptr<rona::motion::ConflictAnalyser> _conflictAnalyser;

    std::map<std::string, rona::map::Polygon> _obstacles;

    rona::map::Path _path_raw;
    rona::map::Path _path_curr;

    rona::motion::Motion _motion;
    rona::motion::Motion _motion_abs;
    double _length_path_curr;

    rona::map::Point2D _currentTarget;
    geometry_msgs::Quaternion _currentTarget_orientation;

    std::string _map_frame;
    std::string _robot_frame;

    double _maxPathLength;          ///< max lenth of path what wil be pushlished for other robots
    double _time_margin_collison;
    double _robotRadius;
    double _pathLengthDiffFak;
    unsigned int _currentMovePorcess;  ///< id from current path element... subed from sim_move

    bool _stateMove_old;
    bool _repubMotion;

    double _expectedArival;

    unsigned int _robot_id;
    unsigned int _robot_prio;

    //coll container...
    std::map<unsigned int, unsigned int> _collissions;
    std::map<unsigned int, rona::map::Object> _robot_types;
    std::map<MapObject, std::shared_ptr<rona::map::MapObject_base>>  _map_objects;


    std_msgs::ColorRGBA _color;
public:
    MuronaSimpleNode();
    virtual ~MuronaSimpleNode();

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
    void timerRepubMotion_callback(const ros::TimerEvent& e);

    void subTarget_callback(const geometry_msgs::PoseStamped& msg);
    void subMap_callback(const nav_msgs::OccupancyGrid& msg);
    void subMotion_callback(const rona_msgs::RobotMotion& msg);
    void subStateControl_callback(const std_msgs::Bool& msg);
    void subMoveProcess_callback(const std_msgs::UInt32& msg);

    void subAddOb_callback(const rona_msgs::Obstacle& msg);
    void subRmOb_callback(const std_msgs::String& msg);

    //void subCallback(const ROS_PACK::MESSAGE& msg);
    rona_msgs::RobotMotion computeRobotMotion(rona::map::Path path);

    void startMove();
    void stopMove();
    void pauseMove();
    void pause_auto_unpauseMove();
    void unpauseMove();

    bool isRobotInRange(rona::map::Point2D pos_r);
    bool isRobotObj(unsigned int id);
    bool isCollission(unsigned int id);

    void robotIsStaic_event(const rona_msgs::RobotMotion& msg);
    void robotIsFree_event(const rona_msgs::RobotMotion& msg);
    void wait_event(unsigned int id);
    void noWait_event(unsigned int id);

    void doReplan();

    void pubMarker(std::vector<rona::map::Point2D> p);
};

#endif /* MURONASIMPLENODE_H_ */
