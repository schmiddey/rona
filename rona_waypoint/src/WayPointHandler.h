#ifndef WAYPOINTHANDLER_H_
#define WAYPOINTHANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <map>

using waypoint_t  = std::pair<geometry_msgs::Point, nav_msgs::Path>;
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



  inline void push(const geometry_msgs::Point& wp, const nav_msgs::Path& path = nav_msgs::Path()) noexcept
  {
    _waypoints.push_back(std::make_pair(wp, path));
  }

  inline void pop_back() noexcept
  {
    _waypoints.pop_back();
  }

  inline std::size_t size() const noexcept { return _waypoints.size();  }

  inline bool empty() const noexcept { return _waypoints.empty(); }

  waypoints_t& getWaypoints() { return _waypoints; }

//  waypoints_t& getWaypoints() const { return _waypoints; }

  waypoint_t& at(const unsigned int idx) { return _waypoints.at(idx); }

  waypoint_t& operator[](const unsigned int idx) noexcept { return _waypoints[idx]; }

  waypoint_t& back() { return _waypoints.back(); }

  waypoint_t& front() { return _waypoints.front(); }

  nav_msgs::Path getPath(unsigned int from, unsigned int to, const std::string& frame_id = "map", const ros::Time& stamp = ros::Time::now())
  {
    if(to < from || to < 1 || from == to || to >= this->size())
    {
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

  nav_msgs::Path getPathComplete()
  {
    return this->getPath(0,this->size() - 1);
  }

  bool serialize(const std::string& file) const
  {
    //todo
    return true;
  }

  bool load(const std::string& file)
  {
    //todo
    return true;
  }

private:
  waypoints_t _waypoints;
};


#endif /* WAYPOINTHANDLER_H_ */
