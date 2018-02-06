#ifndef WAYPOINTHANDLER_H_
#define WAYPOINTHANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <map>
#include <fstream>
#include <string>
#include <sstream>

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

  inline bool serialize(const std::string& file) const
  {
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
        std::cout << "path: " << std::endl;
        for(unsigned int p=0; p<path.poses.size(); ++p)
        {
          auto tmp_p = this->pose2string(path.poses[p].pose);
          std::cout << "tmp_p: " << tmp_p << std::endl;
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
    std::fstream f(file, std::ios::in);
    if(f.is_open())
    {
      std::string line;
      do{
        line.clear();
        std::getline(f,line);
        if(line.empty())
        {
          std::cout << "ret first wp" << std::endl;
          this->clear();
          return false;
        }
      }while(line != "wp");

      //read id
      line.clear();
      std::getline(f,line);
      {
        std::cout << "read waypoint: " << std::stoi(line) << std::endl;
      }
      while(true)
      {
        //read wp
        line.clear();
        std::getline(f, line);
        std::cout << "line: " << line << std::endl;
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
              std::cout << "read waypoint: " << std::stoi(line) << std::endl;
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


#endif /* WAYPOINTHANDLER_H_ */
