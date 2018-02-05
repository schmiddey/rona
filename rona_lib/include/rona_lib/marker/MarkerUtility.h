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
  static int _cnt;
public:
  inline static visualization_msgs::Marker createSphere(const geometry_msgs::PoseStamped& pose,
                                                        const double scale,
                                                        const Color& color,
                                                        const std::string& frame_id = "map")
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "rona";
    marker.id = ++_cnt;

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
                                                      const std::string& frame_id = "map")
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "rona";
    marker.id = ++_cnt;

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

  inline static visualization_msgs::Marker createCyliner(const geometry_msgs::Pose& pose, //todo only point and by default only height
                                                         const double height,
                                                         const double diameter,
                                                         const Color& color,
                                                         const std::string& frame_id = "map")
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "rona";
    marker.id = ++_cnt;

    marker.type = visualization_msgs::Marker::CYLINDER;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = pose;

    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = height;

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
                                                          const std::string& frame_id = "map")
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
    marker.id = ++_cnt;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

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

int Marker::_cnt = 0;


class MarkerArrayHandler{
private:
  visualization_msgs::MarkerArray _markers;
public:

  MarkerArrayHandler() = default;

  MarkerArrayHandler(const std::vector<visualization_msgs::Marker>& markers)
  {
    _markers.markers = markers;
  }

  /**
   * Move constructor
   * @param markers
   */
  MarkerArrayHandler(std::vector<visualization_msgs::Marker>&& markers)
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

  //return copy
  visualization_msgs::MarkerArray get() const
  {
    return _markers;
  }

};

}  // namespace rona


#endif /* MARKERUTILITY_H_ */
