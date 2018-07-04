#ifndef RONA_WAYPOINTHANDLER_H_
#define RONA_WAYPOINTHANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

#include <rona_lib/marker/MarkerUtility.h>

#include <vector>
#include <map>
#include <fstream>
#include <string>
#include <sstream>

class WayPointHelper
{
public:
  static std::pair<bool, nav_msgs::Path> compute_direct_path(const geometry_msgs::Point& start, 
                                                             const geometry_msgs::Point& end, 
                                                             const geometry_msgs::Quaternion& ori,
                                                             const double step_length,
                                                             const std::string frame_id = "map")
  {
    auto ros_time = ros::Time::now();
    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp    = ros_time;
    bool occupied = false;


    tf::Vector3 st(start.x, start.y, 0.0);
    tf::Vector3 en(end.x, end.y, 0.0);

    double dist = st.distance(en);

    tf::Vector3 v_step = ((en - st).normalize()) * step_length;

    unsigned int steps = dist / step_length;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = frame_id;
    p.header.stamp    = ros_time;
    //todo do custom ori may be in line with path ...
    p.pose.orientation = ori;

    //push first wp
    p.pose.position = start;
    path.poses.push_back(p);

    for(unsigned int i=1; i<steps; ++i)
    {
      //todo prove if occupied
      st += v_step;

      p.pose.position.x = st.x();
      p.pose.position.y = st.y();
      p.pose.position.z = 0.0;
  
      path.poses.push_back(p);
    }  

    //push endpoint
    p.pose.position = end;
    path.poses.push_back(p);

    return std::make_pair(occupied, path);
  }
};

using waypoint_t  = std::pair<geometry_msgs::Pose, nav_msgs::Path>;
using waypoints_t = std::vector<waypoint_t>;

class WayPointHandler {

public:

  WayPointHandler(const std::string& file) { this->load(file); }

  //defaults
  WayPointHandler()                         = default;
  WayPointHandler(const WayPointHandler& p) = default;
  WayPointHandler(WayPointHandler&& p)      = default;
  WayPointHandler& operator=(const WayPointHandler& p) = default;
  WayPointHandler& operator=(WayPointHandler&& p)      = default;



  inline void push(const geometry_msgs::Pose& wp, const nav_msgs::Path& path = nav_msgs::Path()) noexcept
  {
    _waypoints.push_back(std::make_pair(wp, path));
  }

  inline void pop_back() noexcept
  {
    _waypoints.pop_back();
  }

  inline void clear()
  {
    _waypoints.clear();
  }

  inline std::size_t size() const noexcept { return _waypoints.size();  }

  inline bool empty() const noexcept { return _waypoints.empty(); }

  inline waypoints_t& getWaypoints() { return _waypoints; }

  const waypoints_t& getWaypoints() const { return _waypoints; }

  inline waypoint_t& at(const unsigned int idx) { return _waypoints.at(idx); }

  inline waypoint_t& operator[](const unsigned int idx) noexcept { return _waypoints[idx]; }

  inline waypoint_t& back() { return _waypoints.back(); }
  inline const waypoint_t& back() const { return _waypoints.back(); }

  inline waypoint_t& front() { return _waypoints.front(); }
  inline const waypoint_t& front() const { return _waypoints.front(); }

  inline nav_msgs::Path getPath(unsigned int from, unsigned int to, const std::string& frame_id = "map", const ros::Time& stamp = ros::Time::now())
  {
    if(to < from || to < 1 || from == to || to >= this->size())
    {
      if(this->size() > 1)
        ROS_WARN("WPHandler: invalid param in get Path");
      return nav_msgs::Path();
    }

    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp    = stamp;


    for(unsigned int i=from + 1; i<=to; ++i)
    {
//      ROS_INFO("Cnt append path: %d", (int)d_path.second.poses.size());
      path.poses.insert(path.poses.end(), this->at(i).second.poses.begin(), this->at(i).second.poses.end());
    }

    return path;
  }

  inline nav_msgs::Path getPathComplete()
  {
    return this->getPath(0,this->size() - 1);
  }

  int getLastWaypointID() const
  {
    return _waypoints.size() -1;
  }

  /**
   * @brief 
   *  @note not working as expected xD -> bug in orientations from path
   */
  inline void flip(const double step_length)
  {
    if(_waypoints.size() < 2)
      return;

    std::string frame_id = _waypoints[0].second.header.frame_id;
    //flip poses except start pose
    std::reverse(_waypoints.begin() + 1, _waypoints.end());

    //recompute direct path... //there is maybe a smarter way :)
    for(unsigned int i = 1; i < _waypoints.size(); i++)
    {
      //maybe bug wrong way but not important in streight line bug not good...
      auto path = WayPointHelper::compute_direct_path(_waypoints[i-1].first.position, _waypoints[i].first.position, _waypoints[i-1].first.orientation, step_length, frame_id);
      _waypoints[i].second = path.second;
    }
    auto path = WayPointHelper::compute_direct_path(_waypoints.back().first.position, _waypoints.front().first.position, _waypoints.back().first.orientation, step_length, frame_id);
    _waypoints.front().second = path.second;
  }

  inline bool serialize(const std::string& file) const
  {
    ROS_INFO_STREAM("Save Waypoints under: " << file);
    std::fstream f(file, std::ios::out);
    if(f.is_open())
    {
      //
      f << "-----------------------------------------------" << std::endl;
      f << "- rona_waypoint - file ------------------------" << std::endl;
      f << "-----------------------------------------------" << std::endl;

      //wp loop
      for(unsigned int i=0; i<_waypoints.size(); ++i)
      {
        f << "wp" << std::endl << i << std::endl << this->pose2string(_waypoints[i].first)  << std::endl;
        //path loop
        const auto& path = _waypoints[i].second;
        f << "path" << std::endl;
        // std::cout << "path: " << std::endl;
        for(unsigned int p=0; p<path.poses.size(); ++p)
        {
          auto tmp_p = this->pose2string(path.poses[p].pose);
          // std::cout << "tmp_p: " << tmp_p << std::endl;
          f << tmp_p << std::endl;
        }
      }

      f.close();
      return true;
    }
    return false;
  }

  inline bool load(const std::string& file)
  {
    ROS_INFO_STREAM("Load Waypoints from: " << file);
    std::fstream f(file, std::ios::in);
    if(f.is_open())
    {
      std::string line;
      do{
        line.clear();
        std::getline(f,line);
        if(line.empty())
        {
//          std::cout << "ret first wp" << std::endl;
          this->clear();
          return false;
        }
      }while(line != "wp");

      //read id
      line.clear();
      std::getline(f,line);
      {
//        std::cout << "read waypoint: " << std::stoi(line) << std::endl;
      }
      while(true)
      {
        //read wp
        line.clear();
        std::getline(f, line);
//        std::cout << "line: " << line << std::endl;
        auto wp = this->string2pose(line);
        //Read path
        line.clear();
        std::getline(f,line);
        //is path?
        if(line != "path")
        {
          std::cout << "ret not path -- " << line  << "line_lenght: " << line.size() << std::endl;
          this->clear();
          return false;
        }
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        while(true)
        {
          line.clear();
          std::getline(f, line);
          if(line == "wp" || line.empty())
          {
            if(line == "wp")
            {
              line.clear();
              std::getline(f,line);
//              std::cout << "read waypoint: " << std::stoi(line) << std::endl;
            }
            break;
          }


          auto pose = this->string2pose(line);
          geometry_msgs::PoseStamped ps;
          ps.pose = pose;
          ps.header.frame_id = "map";
          ps.header.stamp = ros::Time::now();
          path.poses.push_back(ps);

        }
        this->push(wp, path);

        if(line.empty())
        {
          break;
        }
      }

      if(this->size() < 2)
      {
        std::cout << "ret to less wp" << std::endl;

        this->clear();
        return false;
      }

      return true;
    }
    return false;
  }

  inline visualization_msgs::MarkerArray toMarkerArray()
  {
    //black bigger sphere for startpoint, small spherer for interpolated path and LineList for wps
    rona::MarkerArrayHandler m_handler("wp"); //todo adding name to wp handler?!

    //push start
    geometry_msgs::PoseStamped pose_start;
    pose_start.header.frame_id = "map";
    pose_start.header.stamp    = ros::Time::now();
    pose_start.pose.position = this->front().first.position;

    m_handler.push_back(rona::Marker::createSphere(pose_start, 0.1, rona::Color(rona::Color::BLUE) ) );

    std::vector<geometry_msgs::Point> waypoints;

    for(auto& e : this->getWaypoints())
    {
      //add waypoints
      //waypoints.push_back(e.first);
      m_handler.push_back(rona::Marker::createCyliner(e.first.position, 0.2, 0.02, rona::Color(rona::Color::RED) ) );
      m_handler.push_back(rona::Marker::createArrow(e.first, 0.4, 0.02, rona::Color(rona::Color::BLACK) ) );
      //add path
      for(auto& p : e.second.poses)
      {
         m_handler.push_back(rona::Marker::createSphere(p, 0.05, rona::Color(rona::Color::ORANGE) ) );
         m_handler.push_back(rona::Marker::createArrow(p.pose, 0.2, 0.01, rona::Color(rona::Color::BLACK) ) );
      }

    }
    //push waypoints
    //m_handler.push_back(rona::Marker::createLineList(waypoints, 0.2, 0.02, rona::Color(rona::Color::RED) ) );
    return m_handler.get();
  }

  inline std::string pose2string(const geometry_msgs::Pose& p) const
  {
    std::string str;

    str += std::to_string(p.position.x) + " ";
    str += std::to_string(p.position.y) + " ";
    str += std::to_string(p.position.z) + " ";
    str += std::to_string(p.orientation.x) + " ";
    str += std::to_string(p.orientation.y) + " ";
    str += std::to_string(p.orientation.z) + " ";
    str += std::to_string(p.orientation.w);

    return str;
  }

  inline geometry_msgs::Pose string2pose(const std::string& str) const
  {
    geometry_msgs::Pose p;
    std::stringstream sstr(str);
    sstr << std::setprecision(8);

    sstr >> p.position.x;
    sstr >> p.position.y;
    sstr >> p.position.z;
    sstr >> p.orientation.x;
    sstr >> p.orientation.y;
    sstr >> p.orientation.z;
    sstr >> p.orientation.w;

    return p;
  }

private:

  waypoints_t _waypoints;
};


#endif /* RONA_WAYPOINTHANDLER_H_ */
