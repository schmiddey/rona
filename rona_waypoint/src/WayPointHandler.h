#ifndef WAYPOINTHANDLER_H_
#define WAYPOINTHANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

//using waypoints_t = std::vector<geometry_msgs::Point>;

class WayPointHandler {
private:

  unsigned int _test;
  std::vector<geometry_msgs::Point> _waypoints;

public:
  void push(const geometry_msgs::Point& p)
  {

    _waypoints.push_back(p);
  }

  void pop_back()
  {
    _waypoints.pop_back();
  }

  std::size_t size() const { return _waypoints.size();  }

  bool empty() const { return _waypoints.empty(); }

  std::vector<geometry_msgs::Point>& getWaypoints() { return _waypoints; }

  geometry_msgs::Point& at(const unsigned int idx) { return _waypoints.at(idx); }

  geometry_msgs::Point& operator[](const unsigned int idx) { return _waypoints[idx]; }

  bool serialize(const std::string& file) const
  {
    return true;
  }

  bool load(const std::string& file)
  {
    return true;
  }

private:

};


#endif /* WAYPOINTHANDLER_H_ */
