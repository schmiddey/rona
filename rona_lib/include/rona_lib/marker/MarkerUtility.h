/*
 * MarkerUtility.h
 *
 *  Created on: 04.02.2018
 *      Author: m1ch1
 */

#ifndef MARKERUTILITY_H_
#define MARKERUTILITY_H_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <map>
#include <stdexcept>

namespace rona {

class Color {
//todo move private down (only bug in eclipse force me to put it here :/
private:
  struct RGB{
    float r = 0;
    float g = 0;
    float b = 0;
  };

  RGB _color;
  float _alpha;

  const static std::vector<RGB> _rgb_lut;

public:
  enum enum_color{
    WHITE   = 0,
    BLACK      ,
    BLUE       ,
    GREEN      ,
    RED        ,
    YELLOW     ,
    ORANGE
    //todo do more...
  };

  Color(const enum_color color, float alpha = 0.5f) : _alpha(alpha)
  { _color = _rgb_lut.at(color); }

  Color(const float r, const float g, const float b, const float alpha = 0.5f) : _alpha(alpha)
  { _color = { r, g, b }; }

  std_msgs::ColorRGBA toRosColor() const
  {
    std_msgs::ColorRGBA ros_color;
    ros_color.r = _color.r;
    ros_color.g = _color.g;
    ros_color.b = _color.b;
    ros_color.a = _alpha;
    return ros_color;
  }

};

const std::vector<Color::RGB> Color:: _rgb_lut =
{
  { 1.0f, 1.0f, 1.0f },      //WHITE
  { 0.0f, 0.0f, 0.0f },      //BLACK
  { 0.0f, 0.0f, 1.0f },      //BLUE
  { 0.0f, 1.0f, 0.0f },      //GREEN
  { 1.0f, 0.0f, 0.0f },      //RED
  { 1.0f, 1.0f, 0.0f },      //YELLOW   255,255,0
  { 1.0f, 0.6f, 0.0f }       //ORANGE    255,165,0
};

class Marker {
private:
//  static int _cnt;
public:
  inline static visualization_msgs::Marker createSphere(const geometry_msgs::PoseStamped& pose,
                                                        const double scale,
                                                        const Color& color,
                                                        const std::string& frame_id = "map",
                                                        const int id = 0)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "rona";
    marker.id = id;

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = pose.pose;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color = color.toRosColor();

    marker.lifetime = ros::Duration();
    return marker;
  }

  inline static visualization_msgs::Marker createCube(const geometry_msgs::Pose& pose,
                                                      const double scale,
                                                      const Color& color,
                                                      const std::string& frame_id = "map",
                                                      const int id = 0)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "rona";
    marker.id = id;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = pose;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color = color.toRosColor();

    marker.lifetime = ros::Duration();
    return marker;
  }

  inline static visualization_msgs::Marker createCyliner(const geometry_msgs::Point& point, //todo only point and by default only height
                                                         const double height,
                                                         const double diameter,
                                                         const Color& color,
                                                         const std::string& frame_id = "map",
                                                         const int id = 0)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "rona";
    marker.id = id;

    marker.type = visualization_msgs::Marker::CYLINDER;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.pose.position = point;
    marker.pose.position.z += height * 0.5;

    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = height;

    marker.color = color.toRosColor();

    marker.lifetime = ros::Duration();
    return marker;
  }

  inline static visualization_msgs::Marker createArrow(const geometry_msgs::Pose pose,
                                                       const double length,
                                                       const double diameter,
                                                       const Color& color,
                                                       const std::string& frame_id = "map",
                                                       const int id = 0)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "rona";
    marker.id = id;

    marker.type = visualization_msgs::Marker::ARROW;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = pose;

    marker.scale.x = length;
    marker.scale.y = diameter;
    marker.scale.z = diameter;

    marker.color = color.toRosColor();

    marker.lifetime = ros::Duration();
    return marker;
  }

  /**
   *
   * @param points only the flor point is needed per line...
   * @param scale
   * @param color
   * @param frame_id
   * @return
   */
  inline static visualization_msgs::Marker createLineList(const std::vector<geometry_msgs::Point>& points,
                                                          const double height,
                                                          const double diameter,
                                                          const Color& color,
                                                          const std::string& frame_id = "map",
                                                          const int id = 0)
  {
//    Line lists use the points member of the visualization_msgs/Marker message. It will draw a line between each pair of points, so 0-1, 2-3, 4-5, ...
//
//    Line lists also have some special handling for scale: only scale.x is used and it controls the width of the line segments.
//
//    Note that pose is still used (the points in the line will be transformed by them), and the lines will be correct relative to the frame id specified in the header.
//
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "rona";
    marker.id = id;

    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.points.resize(points.size() * 2);

    for(unsigned int i=0; i<points.size(); ++i)
    {
      geometry_msgs::Point p = points[i];
      marker.points[i*2] = p;
      p.z += height;
      marker.points[i*2 + 1] = p;
    }

    marker.scale.x = diameter;
    marker.scale.y = diameter; // not used
    marker.scale.z = diameter; // not used

    marker.color = color.toRosColor();

    marker.lifetime = ros::Duration();
    return marker;
  }

};

/**
 * @todo may move to private in mahandler
 */
class MarkerArrayStaticModel {
public:
  MarkerArrayStaticModel() {}
//  ~MarkerArrayStaticModel();
  std::size_t getLastSize(const std::string& identifier)
  {
    try {
      return _last_sizes.at(identifier);
    } catch (std::out_of_range& e) {
      ROS_INFO("get -- Create lastSize: %s", identifier.c_str());
      _last_sizes.insert(std::make_pair(identifier, 0));
      return 0;
    }
  }

  void setLastSize(const std::string& identifier, const std::size_t size)
  {
    try {
      _last_sizes.at(identifier) = size;
    } catch (std::out_of_range& e) {
      ROS_INFO("set -- Create lastSize: %s", identifier.c_str());
      _last_sizes.insert(std::make_pair(identifier, size));
    }

  }

  static void reset()
  {
    _last_sizes.clear();
  }

private:
  static std::map<std::string, std::size_t> _last_sizes;
};

std::map<std::string, std::size_t> MarkerArrayStaticModel::_last_sizes;

/**
 * this class is for temporary use only: (reason is staic id handling)
 *
 * // good
 * void myfcn()
 * {
 *   MarkerArrayHandler mah("foo");
 *
 *   //do push stuff
 *   mah.push_back(my_marker);
 *   //...
 *
 *   publish(mah.get());
 * }
 *
 *
 * //bad
 * class Bar{
 * public:
 *  void myfcn()
 *  {
 *    publish(_mah);
 *  }
 *
 *  void addmarker(mymarker);
 *
 * private:
 *  MarkerArrayHandler _mah;
 * }
 *
 *
 *
 */
class MarkerArrayHandler{
private:
  visualization_msgs::MarkerArray _markers;
  std::string _identifier;
  MarkerArrayStaticModel _smodel;
public:

  MarkerArrayHandler() = delete;

  /**
   * @note std constructor must be delegated from all other constructors
   * @param identifier
   */
  MarkerArrayHandler(const std::string& identifier) : _identifier(identifier)
  {
//    _smodel.setLastSize(identifier, 0);
  }

  MarkerArrayHandler(const std::string& identifier, const std::vector<visualization_msgs::Marker>& markers) : MarkerArrayHandler(identifier)
  {
    _markers.markers = markers;
  }

  /**
   * Move constructor
   * @param markers
   */
  MarkerArrayHandler(const std::string& identifier, std::vector<visualization_msgs::Marker>&& markers) : MarkerArrayHandler(identifier)
  {
    _markers.markers = markers;
  }


  void push_back(const visualization_msgs::Marker& marker)
  {
    _markers.markers.push_back(marker);
  }

  void pop_back()
  {
    _markers.markers.pop_back();
  }

  std::size_t size() const
  {
    return _markers.markers.size();
  }

  void clear()
  {
//    _last_marker_cnt = _markers.markers.size();
    _markers.markers.clear();
  }

  /**
   * return copy and clears marker array
   * @return
   */
  visualization_msgs::MarkerArray get()
  {
    auto rm_appendix = this->finalizeMarkerArray();
    //todo create new marker array and may fillup with markers to remove...(if needed)
    visualization_msgs::MarkerArray ret = _markers;
    ret.markers.insert(ret.markers.end(), rm_appendix.markers.begin(), rm_appendix.markers.end());

    _smodel.setLastSize(_identifier, this->size());

    _markers.markers.clear();

    return ret;
  }

//  const visualization_msgs::MarkerArray& get() const
//  {
//    return _markers;
//  }
//
//  visualization_msgs::MarkerArray& get()
//  {
//    return _markers;
//  }


private:

  visualization_msgs::MarkerArray finalizeMarkerArray()
  {
    visualization_msgs::MarkerArray rm_appendix;
    auto last_size = _smodel.getLastSize(_identifier);

//    ROS_INFO_STREAM("last_size: " << last_size);

    int id = 0;

    // fix marker ID;
    for(auto& e : _markers.markers)
    {
       e.id = id++;
    }


    if( last_size <= this->size())
    {
      return rm_appendix;
    }

    //fill up missing marker and define them as remove..
    //hack delete not working... set scale to 0 and set alpha to 0... shitty stuff here ...
    visualization_msgs::Marker rm_marker = Marker::createCube(_markers.markers.front().pose, 0.1, Color(Color::WHITE, 0.0f));
//    rm_marker.header.frame_id = _markers.markers.front().header.frame_id;
//    rm_marker.header.stamp    = ros::Time::now();
//    rm_marker.pose.orientation.w = 1;
//
//    rm_marker.color.a = 0.0f;
//
//    rm_marker.type = visualization_msgs::Marker::CUBE;
//
//    rm_marker.action= visualization_msgs::Marker::ADD;


    int cnt_appendix = last_size - this->size();

    rm_appendix.markers.resize(cnt_appendix);
    for(auto& e : rm_appendix.markers)
    {
       e = rm_marker;
    }

    for(auto& e : rm_appendix.markers)
    {
      e.id = id++;
    }

    return rm_appendix;
  }

};

}  // namespace rona


#endif /* MARKERUTILITY_H_ */
